#include "visual_marker_mapping/TagDetector_cptags.h"
#include "visual_marker_mapping/DetectionResults.h"
#include "visual_marker_mapping/FileUtilities.h"
#include "visual_marker_mapping/TagDetector.h"
#include <boost/filesystem.hpp>
#include <cptags/TagSystem.h>
#include <cptags/detector.h>
#include <memory>
#include <opencv2/opencv.hpp>
#include <regex>
#include <set>

namespace visual_marker_mapping
{
namespace cptags
{
    //-------------------------------------------------------------------------------------------------
    DetectionResult detectTags(
        const std::vector<std::string>& filePaths, const std::string& tagType)
    {
        if (filePaths.empty())
            throw std::runtime_error("Filepaths for tag detections are empty.");

        cv::Mat img = cv::imread(filePaths[0], cv::IMREAD_GRAYSCALE);
        const int imgWidth = img.cols;
        const int imgHeight = img.rows;

        ;
        std::unique_ptr<::cptags::TagSystem> tag_system;
        if (tagType == "cptags_14h2mc3")
            tag_system.reset(new ::cptags::TagSystem(14, 2, 3));
        else
            throw std::runtime_error("Unsupported marker type " + tagType + "!");

        DetectionResult result;

        int imageId = 0;
        int filteredImageId = 0;
        std::set<int> tagIds;
        for (const auto& filePath : filePaths)
        {
            boost::filesystem::path p(filePath);

            if (!boost::filesystem::exists(p))
            {
                std::cerr << "File: " << filePath << " does not exist" << std::endl;
                continue;
            }

            std::cout << "Processing file " << (imageId + 1) << "/" << filePaths.size() << " "
                      << filePath << std::endl;

            const cv::Mat img = cv::imread(filePath, cv::IMREAD_GRAYSCALE);

            if (img.cols != imgWidth || img.rows != imgHeight)
            {
                std::cerr << "Image " << p.filename() << " has not the correct size of "
                          << imgHeight << " x " << imgWidth << std::endl;
                continue;
            }

            cv::Mat visualization = img.clone();
            cv::cvtColor(visualization, visualization, cv::COLOR_GRAY2BGR);

            auto detectedTags = ::cptags::detect_tags(img, *tag_system);
            std::cout << detectedTags.size() << std::endl;

            std::set<int> goodObservations;

            for (const auto& detectedTag : detectedTags)
            {
                goodObservations.insert(detectedTag.id);

                TagObservation tagObs;
                tagObs.imageId = filteredImageId;
                tagObs.tagId = detectedTag.id;
                tagObs.corners.resize(1);
                //for (size_t i = 0; i < 4; ++i)
                    tagObs.corners[0] << detectedTag.x, detectedTag.y;

                result.tagObservations.push_back(tagObs);

                tagIds.insert(detectedTag.id);
            }

            if (!goodObservations.empty())
            {
                std::cout << "   Detected " << goodObservations.size() << " tags: ";
                for (int i : goodObservations)
                    std::cout << i << ", ";
                std::cout << std::endl;

                TagImg img;
                img.filename = boost::filesystem::path(filePath).string();
                img.imageId = filteredImageId;
                result.images.push_back(img);
                filteredImageId++;
            }
            else
                std::cout << "   No tags found!";

            imageId++;
        }

        for (int i : tagIds)
        {
            Tag tag;
            tag.tagId = i;
            tag.tagType = tagType;
            tag.width = 0;
            tag.height = 0;
            result.tags.push_back(tag);
        }

        return result;
    }
    //-------------------------------------------------------------------------------------------------
    DetectionResult detectTags(const std::string& folder, const std::string& tagType)
    {
        std::regex reg(
            "(.*)\\.((png)|(jpg))", std::regex_constants::ECMAScript | std::regex_constants::icase);
        const std::vector<std::string> filePaths = readFilesFromDir(folder, reg);
        return detectTags(filePaths, tagType);
    }
    //-------------------------------------------------------------------------------------------------
} // namespace cptags
} // namespace visual_marker_mapping
