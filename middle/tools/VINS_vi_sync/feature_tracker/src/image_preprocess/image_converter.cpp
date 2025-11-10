#include "image_converter.h"
#include <ros/console.h>

/**
 * Converts an input image of various encodings to an 8-bit single-channel grayscale image.
 * @param input_img Input image (cv::Mat) with different possible encodings (RGB, BGR, Grayscale, etc.)
 * @param encoding String representing the image encoding (e.g., "rgb8", "bgr16", "gray8", "rgba8")
 * @return 8-bit single-channel (CV_8UC1) grayscale image
 */
cv::Mat convertToGray(const cv::Mat& input_img, const std::string& encoding) {
    // Step 1: Handle 16-bit images (scale down to 8-bit)
    cv::Mat img_8bit; // Intermediate 8-bit image
    if (is16Bit(encoding)) {
        // Convert 16-bit to 8-bit: linear scaling (assuming input range 0-65535 → 0-255)
        // Scale factor 1/256 reduces 16-bit values (0-65535) to 8-bit (0-255)
        input_img.convertTo(img_8bit, CV_8UC(input_img.channels()), 1.0 / 256.0);
    } else {
        // For 8-bit images, directly clone the input to avoid modifying the original
        img_8bit = input_img.clone();
    }

    // Step 2: Convert to grayscale based on the encoding
    cv::Mat gray_img;
    if (isRGBFamily(encoding)) {
        // Handle RGB family encodings (including those with alpha channels)
        if (encoding == "rgb8" || encoding == "rgb16") {
            // Convert RGB (8-bit or 16-bit, already scaled to 8-bit) to grayscale
            cv::cvtColor(img_8bit, gray_img, cv::COLOR_RGB2GRAY);
        } else if (encoding == "bgr8" || encoding == "bgr16") {
            // Convert BGR (8-bit or 16-bit, already scaled to 8-bit) to grayscale
            cv::cvtColor(img_8bit, gray_img, cv::COLOR_BGR2GRAY);
        } else if (encoding == "rgba8" || encoding == "rgba16") {
            // Convert RGBA to grayscale (alpha channel is ignored)
            cv::cvtColor(img_8bit, gray_img, cv::COLOR_RGBA2GRAY); 
        } else if (encoding == "bgra8" || encoding == "bgra16") {
            // Convert BGRA to grayscale (alpha channel is ignored)
            cv::cvtColor(img_8bit, gray_img, cv::COLOR_BGRA2GRAY); 
        } else {
            // Unknown RGB variant: fall back to BGR conversion for engineering compatibility
            ROS_WARN("Unknown RGB encoding: %s, using BGR2GRAY as fallback", encoding.c_str());
            cv::cvtColor(img_8bit, gray_img, cv::COLOR_BGR2GRAY);
        }
    } else if (isGrayFamily(encoding)) {
        // Handle grayscale family encodings
        if (img_8bit.channels() == 1) {
            // Valid grayscale image with single channel: use directly
            gray_img = img_8bit;
        } else {
            // Abnormal case: grayscale encoding but multiple channels, force to single channel
            ROS_WARN("Gray encoding %s has %d channels, forcing to 1 channel", 
                     encoding.c_str(), img_8bit.channels());
            cv::cvtColor(img_8bit, gray_img, cv::COLOR_BGR2GRAY); // Universal conversion
        }
    } else {
       // Unsupported/unknown encoding: infer based on channel count
        ROS_WARN("Unsupported encoding: %s, channels=%d, trying to convert", 
                 encoding.c_str(), img_8bit.channels());
        if (img_8bit.channels() == 3) {
            // Assume 3-channel image is BGR (common in OpenCV)
            cv::cvtColor(img_8bit, gray_img, cv::COLOR_BGR2GRAY); 
        } else if (img_8bit.channels() == 4) {
            // Assume 4-channel image is BGRA (common in OpenCV with alpha)
            cv::cvtColor(img_8bit, gray_img, cv::COLOR_BGRA2GRAY); 
        } else {
            // For single channel or other channel counts: take the first channel as grayscale
            std::vector<cv::Mat> channels; // Split image into individual channels
            cv::split(img_8bit, channels);
            gray_img = channels[0]; // Use the first channel
        }
    }

    // Ensure the output is strictly 8-bit single-channel (CV_8UC1)
    if (gray_img.type() != CV_8UC1) {
        gray_img.convertTo(gray_img, CV_8UC1);
    }
    return gray_img;
}