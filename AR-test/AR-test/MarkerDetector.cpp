#include <iostream>
#include <sstream>
#include "MarkerDetector.hpp"
#include "Marker.hpp"
#include "TinyLA.hpp"

MarkerDetector::MarkerDetector(CameraCalibration calibration) : m_minContourLengthAllowed(100), markerSize(100,100)
{
    cv::Mat(3,3, CV_32F, const_cast<float*>(&calibration.getIntrinsic().data[0])).copyTo(camMatrix);
    cv::Mat(4,1, CV_32F, const_cast<float*>(&calibration.getDistorsion().data[0])).copyTo(distCoeff);
    bool centerOrigin = true;
    if (centerOrigin)
    {
        m_markerCorners3d.push_back(cv::Point3f(-0.5f,-0.5f,0));
        m_markerCorners3d.push_back(cv::Point3f(+0.5f,-0.5f,0));
        m_markerCorners3d.push_back(cv::Point3f(+0.5f,+0.5f,0));
        m_markerCorners3d.push_back(cv::Point3f(-0.5f,+0.5f,0));
    }
    else
    {
        m_markerCorners3d.push_back(cv::Point3f(0,0,0));
        m_markerCorners3d.push_back(cv::Point3f(1,0,0));
        m_markerCorners3d.push_back(cv::Point3f(1,1,0));
        m_markerCorners3d.push_back(cv::Point3f(0,1,0));    
    }
    m_markerCorners2d.push_back(cv::Point2f(0,0));
    m_markerCorners2d.push_back(cv::Point2f(markerSize.width-1,0));
    m_markerCorners2d.push_back(cv::Point2f(markerSize.width-1,markerSize.height-1));
    m_markerCorners2d.push_back(cv::Point2f(0,markerSize.height-1));
}

void MarkerDetector::processFrame(const BGRAVideoFrame& frame)
{
	std::swap(markers, std::vector<Marker>());
    findMarkers(frame, markers);
    m_transformations.clear();
    for (size_t i=0; i<markers.size(); i++)
    {
        m_transformations.push_back(markers[i].transformation);
    }
}

const std::vector<Transformation>& MarkerDetector::getTransformations() const
{
    return m_transformations;
}


bool MarkerDetector::findMarkers(const BGRAVideoFrame& frame, std::vector<Marker>& detectedMarkers)
{
    cv::Mat bgraMat(frame.height, frame.width, CV_8UC4, frame.data, frame.stride);
    prepareImage(bgraMat, m_grayscaleImage);
    performThreshold(m_grayscaleImage, m_thresholdImg);
    findContours(m_thresholdImg, m_contours, m_grayscaleImage.cols / 5);
    findCandidates(m_contours, detectedMarkers);
    recognizeMarkers(m_grayscaleImage, detectedMarkers);
    estimatePosition(detectedMarkers);
    std::sort(detectedMarkers.begin(), detectedMarkers.end());
    return false;
}

void MarkerDetector::prepareImage(const cv::Mat& bgraMat, cv::Mat& grayscale) const
{
    cv::cvtColor(bgraMat, grayscale, CV_BGRA2GRAY);
}

void MarkerDetector::performThreshold(const cv::Mat& grayscale, cv::Mat& thresholdImg) const
{
	adaptiveThreshold(grayscale,thresholdImg,
		255,
		cv::ADAPTIVE_THRESH_GAUSSIAN_C,
		cv::THRESH_BINARY_INV,
		7,
		7
		);
}
MarkerDetector::ContoursVector allContours;
void MarkerDetector::findContours(cv::Mat& thresholdImg, ContoursVector& contours, int minContourPointsAllowed) const
{
    cv::findContours(thresholdImg, allContours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);
    contours.clear();
    for (size_t i=0; i<allContours.size(); i++)
    {
        int contourSize = allContours[i].size();
        if (contourSize > minContourPointsAllowed)
        {
            contours.push_back(allContours[i]);
        }
    }
	return;
}

void MarkerDetector::findCandidates(const ContoursVector& contours, std::vector<Marker>& detectedMarkers) 
{
    std::vector<cv::Point>  approxCurve;
    std::vector<Marker>     possibleMarkers;


    for (size_t i=0; i<contours.size(); i++)
    {
        double eps = contours[i].size() * 0.05;
        cv::approxPolyDP(contours[i], approxCurve, eps, true);
        if (approxCurve.size() != 4)
            continue;
        if (!cv::isContourConvex(approxCurve))
            continue;
        float minDist = std::numeric_limits<float>::max();

        for (int i = 0; i < 4; i++)
        {
            cv::Point side = approxCurve[i] - approxCurve[(i+1)%4];            
            float squaredSideLength = side.dot(side);
            minDist = std::min(minDist, squaredSideLength);
        }
        if (minDist < m_minContourLengthAllowed)
            continue;
        Marker m;

        for (int i = 0; i<4; i++)
            m.points.push_back( cv::Point2f(approxCurve[i].x,approxCurve[i].y) );

        cv::Point v1 = m.points[1] - m.points[0];
        cv::Point v2 = m.points[2] - m.points[0];

        double o = (v1.x * v2.y) - (v1.y * v2.x);
        if (o < 0.0)
            std::swap(m.points[1], m.points[3]);

        possibleMarkers.push_back(m);
    }
    std::vector< std::pair<int,int> > tooNearCandidates;
    for (size_t i=0;i<possibleMarkers.size();i++)
    { 
        const Marker& m1 = possibleMarkers[i];
        for (size_t j=i+1;j<possibleMarkers.size();j++)
        {
            const Marker& m2 = possibleMarkers[j];
            float distSquared = 0;
            for (int c = 0; c < 4; c++)
            {
                cv::Point v = m1.points[c] - m2.points[c];
                distSquared += v.dot(v);
            }
            distSquared /= 4;
            if (distSquared < 100)
                tooNearCandidates.push_back(std::pair<int,int>(i,j));
        }				
    }

    std::vector<bool> removalMask (possibleMarkers.size(), false);
    for (size_t i=0; i<tooNearCandidates.size(); i++)
    {
        float p1 = perimeter(possibleMarkers[tooNearCandidates[i].first ].points);
        float p2 = perimeter(possibleMarkers[tooNearCandidates[i].second].points);
        size_t removalIndex;
        if (p1 > p2)
            removalIndex = tooNearCandidates[i].second;
        else
            removalIndex = tooNearCandidates[i].first;
        removalMask[removalIndex] = true;
    }

    detectedMarkers.clear();
    for (size_t i=0;i<possibleMarkers.size();i++)
        if (!removalMask[i])
            detectedMarkers.push_back(possibleMarkers[i]);
}

void MarkerDetector::recognizeMarkers(const cv::Mat& grayscale, std::vector<Marker>& detectedMarkers)
{
    std::vector<Marker> goodMarkers;
    for (size_t i=0;i<detectedMarkers.size();i++)
    {
        Marker& marker = detectedMarkers[i];
        cv::Mat markerTransform = cv::getPerspectiveTransform(marker.points, m_markerCorners2d);
        cv::warpPerspective(grayscale, canonicalMarkerImage,  markerTransform, markerSize);
        int nRotations;
        int id = Marker::getMarkerId(canonicalMarkerImage, nRotations);
        if (id !=- 1)
        {
            marker.id = id;
            std::rotate(marker.points.begin(), marker.points.begin() + 4 - nRotations, marker.points.end());
            goodMarkers.push_back(marker);
        }
    }  

    if (goodMarkers.size() > 0)
    {
        std::vector<cv::Point2f> preciseCorners(4 * goodMarkers.size());
        for (size_t i=0; i<goodMarkers.size(); i++)
        {  
            const Marker& marker = goodMarkers[i];      
            for (int c = 0; c <4; c++)
                preciseCorners[i*4 + c] = marker.points[c];
        }
        cv::TermCriteria termCriteria = cv::TermCriteria(cv::TermCriteria::MAX_ITER | cv::TermCriteria::EPS, 30, 0.01);
        cv::cornerSubPix(grayscale, preciseCorners, cvSize(5,5), cvSize(-1,-1), termCriteria);

        for (size_t i=0; i<goodMarkers.size(); i++)
        {
            Marker& marker = goodMarkers[i];      
            for (int c=0;c<4;c++) 
                marker.points[c] = preciseCorners[i*4 + c];
        }
    }
    detectedMarkers = goodMarkers;
}


void MarkerDetector::estimatePosition(std::vector<Marker>& detectedMarkers)
{
    for (size_t i=0; i<detectedMarkers.size(); i++)
    {					
        Marker& m = detectedMarkers[i];
        cv::Mat Rvec;
        cv::Mat_<float> Tvec;
        cv::Mat raux,taux;
        cv::solvePnP(m_markerCorners3d, m.points, camMatrix, distCoeff,raux,taux);
        raux.convertTo(Rvec,CV_32F);
        taux.convertTo(Tvec ,CV_32F);
        cv::Mat_<float> rotMat(3,3); 
        cv::Rodrigues(Rvec, rotMat);
        for (int col=0; col<3; col++)
        {
            for (int row=0; row<3; row++)   
                m.transformation.r().mat[row][col] = rotMat(row,col); 
            m.transformation.t().data[col] = Tvec(col);
        }
        m.transformation = m.transformation.getInverted();
    }
}
