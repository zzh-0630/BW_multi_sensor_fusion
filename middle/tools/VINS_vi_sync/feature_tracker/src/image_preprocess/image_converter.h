#ifndef IMAGE_CONVERTER_H
#define IMAGE_CONVERTER_H

#include <opencv2/opencv.hpp>
#include <string>
#include "image_format.h"

/**
 * @brief Converts an input image to an 8-bit single-channel grayscale image (CV_8UC1) based on ROS encoding.
 * 
 * This function handles various input image formats specified by ROS encoding strings,
 * converting them into a unified 8-bit grayscale format which is commonly used in 
 * feature tracking algorithms for simplicity and efficiency.
 * 
 * @param input_img The input image in OpenCV Mat format (could be color, different bit depth, etc.)
 * @param encoding ROS image encoding string (e.g., "bgr8", "rgb8", "mono8", "16UC1" etc.)
 *                                        indicating the format of the input image.
 * @return cv::Mat The converted 8-bit single-channel grayscale image (CV_8UC1).
 */
cv::Mat convertToGray(const cv::Mat& input_img, const std::string& encoding);

#endif 