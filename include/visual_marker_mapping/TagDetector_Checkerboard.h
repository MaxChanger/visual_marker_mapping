#pragma once

#include "DetectionResults.h"
#include <Eigen/StdVector>
#include <string>
#include <vector>

namespace visual_marker_mapping
{
namespace checkerboard
{
    DetectionResult detectTags(
        const std::string& folder, const std::string& tagType = "checkerboard_corner");
    DetectionResult detectTags(const std::vector<std::string>& filePaths,
        const std::string& tagType = "checkerboard_corner");
} // namespace checkerboard
} // namespace visual_marker_mapping
