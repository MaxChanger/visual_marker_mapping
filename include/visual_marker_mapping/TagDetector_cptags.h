#pragma once

#include "DetectionResults.h"
#include <Eigen/StdVector>
#include <string>
#include <vector>

namespace visual_marker_mapping
{
namespace cptags
{
    DetectionResult detectTags(
        const std::string& folder, const std::string& tagType = "cptags_14h2mc3");
    DetectionResult detectTags(
        const std::vector<std::string>& filePaths, const std::string& tagType = "cptags_14h2mc3");
} // namespace cptags
} // namespace visual_marker_mapping
