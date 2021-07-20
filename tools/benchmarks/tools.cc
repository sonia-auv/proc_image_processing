#include "tools.h"
#include <dirent.h>

namespace tools {
    void addTrailingSlash(fs::path &path) {
        path.remove_trailing_separator();
        path.append("/");
    }

    vector<pair<cv::String, cv::Mat>> loadImages(const string &directory) {
        DIR *dir;
        vector<pair<cv::String, cv::Mat>> images;
        struct dirent *file;
        if ((dir = opendir(directory.c_str())) != nullptr) {
            while ((file = readdir(dir)) != nullptr) {
                cv::Mat m = imread(directory + "/" + file->d_name, cv::IMREAD_GRAYSCALE);
                cv::String path = file->d_name;
                if (file->d_type != DT_DIR && m.data != nullptr) {
                    images.emplace_back(make_pair(path, m));
                }
            }
            closedir(dir);
        }
        return images;
    }

    void writeImages(const string &outputDirectory, vector<pair<cv::String, cv::Mat>> &images) {
        for (const auto &out : images) {
            imwrite(outputDirectory + out.first, out.second);
        }
    }
}
