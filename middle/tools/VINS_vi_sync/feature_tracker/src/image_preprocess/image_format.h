#ifndef IMAGE_FORMAT_H
#define IMAGE_FORMAT_H

#include <string>

/**
 * @brief Check if the image encoding belongs to the RGB family (including alpha channel variants).
 * 
 * The RGB family includes common RGB, BGR formats, and their variants with alpha channels,
 * supporting both 8-bit and 16-bit depth.
 * 
 * @param encoding The image encoding string (e.g., "rgb8", "bgra16").
 * @return true If the encoding is in the RGB family.
 * @return false Otherwise.
 */
inline bool isRGBFamily(const std::string& encoding) {
    return encoding == "rgb8"   || encoding == "rgb16"  ||
           encoding == "bgr8"   || encoding == "bgr16"  ||
           encoding == "rgba8"  || encoding == "rgba16" ||
           encoding == "bgra8"  || encoding == "bgra16";
}

/**
 * @brief Check if the image encoding belongs to the grayscale (monochrome) family.
 * 
 * The grayscale family includes 8-bit and 16-bit monochrome formats.
 * 
 * @param encoding The image encoding string (e.g., "mono8", "mono16").
 * @return true If the encoding is a grayscale format.
 * @return false Otherwise.
 */
inline bool isGrayFamily(const std::string& encoding) {
    return encoding == "mono8"  || encoding == "mono16";
}

/**
 * @brief Check if the image encoding corresponds to a 16-bit depth format.
 * 
 * Determined by checking if the encoding string contains the substring "16" (follows common naming conventions
 * where 16-bit formats include "16" in their identifier).
 * 
 * @param encoding The image encoding string (e.g., "rgb16", "mono16").
 * @return true If the encoding is 16-bit depth.
 * @return false Otherwise (e.g., 8-bit depth).
 */
inline bool is16Bit(const std::string& encoding) {
    return encoding.find("16") != std::string::npos;
}

#endif