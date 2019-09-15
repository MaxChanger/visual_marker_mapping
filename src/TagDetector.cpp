#include "visual_marker_mapping/TagDetector.h"
#include "visual_marker_mapping/FileUtilities.h"
#include <boost/filesystem.hpp>
#include <memory>
#include <opencv2/opencv.hpp>
#include <regex>
#include <set>

namespace visual_marker_mapping
{
//-------------------------------------------------------------------------------------------------
void visualizeDetectionResults(
    const DetectionResult& detectionResult, const std::string& exportFolder)
{
    if (!boost::filesystem::exists(exportFolder))
    {
        boost::filesystem::create_directories(exportFolder);
    }

    const cv::Scalar colorMap[4] = { cv::Scalar(255, 0, 0), cv::Scalar(0, 255, 0),
        cv::Scalar(0, 0, 255), cv::Scalar(255, 255, 0) };
    for (const auto& image : detectionResult.images)
    {
        cv::Mat cvImg = cv::imread(image.filename);
        for (const auto& tagDetection : detectionResult.tagObservations)
        {
            if (tagDetection.imageId != image.imageId)
                continue;

            Eigen::Vector2d center = Eigen::Vector2d::Zero();
            for (const auto& c : tagDetection.corners)
                center += c;
            center /= tagDetection.corners.size();
            const int px = static_cast<int>(center.x());
            const int py = static_cast<int>(center.y());
            const cv::Point pos(px, py);

            cv::putText(cvImg, std::to_string(tagDetection.tagId), pos, cv::FONT_HERSHEY_SIMPLEX,
                3.5, cv::Scalar(255, 100, 0), 4);

            for (size_t i = 0; i < tagDetection.corners.size(); ++i)
            {
                const cv::Scalar color = colorMap[i];
                cv::circle(cvImg,
                    cv::Point2f(tagDetection.corners[i].x(), tagDetection.corners[i].y()), 3, color,
                    3);
            }
        }

        const boost::filesystem::path p(image.filename);
        const std::string exportFilePath
            = (boost::filesystem::path(exportFolder) / p.filename()).string();
        cv::imwrite(exportFilePath, cvImg);
    }
}
//-------------------------------------------------------------------------------------------------
void refineTagObservation(const cv::Mat& img, TagObservation& tagObs)
{
    std::vector<cv::Point2f> corners;
    for (size_t i = 0; i < tagObs.corners.size(); i++)
        corners.emplace_back(tagObs.corners[i].x(), tagObs.corners[i].y());

    cv::Size winSize = cv::Size(10, 10);
    cv::Size zeroZone = cv::Size(-1, -1);
    cv::TermCriteria criteria
        = cv::TermCriteria(cv::TermCriteria::MAX_ITER + cv::TermCriteria::EPS, 100, 0.001);

    /// Calculate the refined corner locations
    cv::cornerSubPix(img, corners, winSize, zeroZone, criteria);

    for (size_t i = 0; i < tagObs.corners.size(); i++)
    {
        // std::cout << "BEFOR: " << tagObs.corners[i].transpose() <<
        // std::endl;
        tagObs.corners[i] << corners[i].x, corners[i].y;
        // std::cout << "AFTER: " << tagObs.corners[i].transpose() <<
        // std::endl;
    }
}
//-------------------------------------------------------------------------------------------------
} // namespace visual_marker_mapping
