#ifndef _MarkerDetector_hp
#define _MarkerDetector_hpp

#include <vector>
#include <opencv2/opencv.hpp>
#include "BGRAVideoFrame.h"
#include "CameraCalibration.hpp"
#include "Marker.hpp"

class Marker;

class MarkerDetector
{
public:
  typedef std::vector<cv::Point>    PointsVector;
  typedef std::vector<PointsVector> ContoursVector;
  std::vector<Marker> markers;
  MarkerDetector(CameraCalibration calibration);
  void processFrame(const BGRAVideoFrame& frame);
  const std::vector<Transformation>& getTransformations() const;

protected:
  bool findMarkers(const BGRAVideoFrame& frame, std::vector<Marker>& detectedMarkers);
  void prepareImage(const cv::Mat& bgraMat, cv::Mat& grayscale) const;
  void performThreshold(const cv::Mat& grayscale, cv::Mat& thresholdImg) const;
  void findContours(cv::Mat& thresholdImg, ContoursVector& contours, int minContourPointsAllowed) const;
  void findCandidates(const ContoursVector& contours, std::vector<Marker>& detectedMarkers);
  void recognizeMarkers(const cv::Mat& grayscale, std::vector<Marker>& detectedMarkers);
  void estimatePosition(std::vector<Marker>& detectedMarkers);

private:
  float m_minContourLengthAllowed;
  cv::Size markerSize;
  cv::Mat camMatrix;
  cv::Mat distCoeff;
  std::vector<Transformation> m_transformations;
  cv::Mat m_grayscaleImage;
  cv::Mat m_thresholdImg;  
  cv::Mat canonicalMarkerImage;
  ContoursVector           m_contours;
  std::vector<cv::Point3f> m_markerCorners3d;
  std::vector<cv::Point2f> m_markerCorners2d;
};

#endif
