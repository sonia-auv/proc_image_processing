#include <opencv2/opencv.hpp>
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>
#include <vector>

using namespace std;
namespace fs = boost::filesystem;

#ifndef PROC_IMAGE_PROCESSING_TOOLS_H
#define PROC_IMAGE_PROCESSING_TOOLS_H
namespace tools {
    void addTrailingSlash(fs::path &path);
    vector<pair<cv::String, cv::Mat>> loadImages(const string &directory);
    void writeImages(const string &outputDirectory, vector<pair<cv::String, cv::Mat>> &images);
}
#endif //PROC_IMAGE_PROCESSING_TOOLS_H
