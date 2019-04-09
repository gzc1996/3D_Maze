

#ifndef _Marker_hpp
#define _Marker_hpp
#include <vector>
#include <iostream>
#include <opencv2/opencv.hpp>
#include "GeometryTypes.hpp"

class Marker
{  
public:
  Marker();
  
  friend bool operator<(const Marker &M1,const Marker&M2);
  friend std::ostream & operator<<(std::ostream &str,const Marker &M);

  static cv::Mat rotate(cv::Mat  in);
  static int hammDistMarker(cv::Mat bits);
  static int mat2id(const cv::Mat &bits);
  static int getMarkerId(cv::Mat &in,int &nRotations);
  
public:
  
  int id;
  Transformation transformation;
  std::vector<cv::Point2f> points;
  void drawContour(cv::Mat& image, cv::Scalar color = CV_RGB(0,250,0)) const;
};

#endif
