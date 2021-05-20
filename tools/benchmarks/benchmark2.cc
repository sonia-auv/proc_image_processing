#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>
#include <iostream>
#include <dirent.h>
#include <vector>
#include <thread>
#include <chrono>
#include <opencv2/imgproc.hpp>
#include <string>
#include <opencv2/cudaimgproc.hpp>

/**
 * Benchmark without GPU streams (1:1 comparison)
 */

using namespace std;

vector<cv::String> listFiles(const string &directory) {
    DIR *dir;
    vector<cv::String> medias;
    struct dirent *file;
    if ((dir = opendir(directory.c_str())) != nullptr) {
        while ((file = readdir(dir)) != nullptr) {
            cv::String path = directory + "/" + file->d_name;
            if (file->d_type != DT_DIR) {
                medias.emplace_back(path);
            }
        }
        closedir(dir);
    }
    return medias;
}

void run(const vector<cv::String> &images) {
    // Load all in memory to simulate receiving messages from the ROS provider
    vector<cv::Mat> loadedImages;
    for (const auto &image : images) {
        // cout << image << endl;
        cv::Mat src = imread(image,  cv::IMREAD_GRAYSCALE);
        if (src.data != nullptr) {
            loadedImages.emplace_back(src);
        }
    }
    // Benchmark
    chrono::steady_clock::time_point now = chrono::steady_clock::now();
    for (const auto &loadedImage : loadedImages) {
        cv::Mat resized, blurred, edges, dest;

        chrono::steady_clock::time_point resizeNow = chrono::steady_clock::now();
        resize(loadedImage, resized, cv::Size(1280, 720), 0, 0, cv::INTER_CUBIC);
        chrono::steady_clock::time_point resizeEnd = chrono::steady_clock::now();

        chrono::steady_clock::time_point blurNow = chrono::steady_clock::now();
        blur(resized, blurred, cv::Size(3, 3));
        chrono::steady_clock::time_point blurEnd = chrono::steady_clock::now();

        chrono::steady_clock::time_point cannyNow = chrono::steady_clock::now();
        Canny(blurred, edges, 100, 100 * 3);
        chrono::steady_clock::time_point cannyEnd = chrono::steady_clock::now();

        dest = cv::Scalar::all(0);
        resized.copyTo(dest, edges);

        cout << "OpenCV runtime (CPU) - Resize: " << chrono::duration_cast<chrono::nanoseconds>(resizeEnd - resizeNow).count() << " nanoseconds" << endl;
        cout << "OpenCV runtime (CPU) - Blur: " << chrono::duration_cast<chrono::nanoseconds>(blurEnd - blurNow).count() << " nanoseconds" << endl;
        cout << "OpenCV runtime (CPU) - Canny: " << chrono::duration_cast<chrono::nanoseconds>(cannyEnd - cannyNow).count() << " nanoseconds" << endl;
    }
    chrono::steady_clock::time_point end = chrono::steady_clock::now();
    cout << "OpenCV runtime (CPU): " << chrono::duration_cast<chrono::milliseconds>(end - now).count() << " milliseconds" << endl;
    loadedImages.clear();
}

void runCUDA(const vector<cv::String> &images) {
    // Load all in memory to simulate receiving messages from the ROS provider
    vector<cv::Mat> loadedImages;
    for (const auto &image : images) {
        // cout << image << endl;
        cv::Mat src = imread(image, cv::IMREAD_GRAYSCALE);
        if (src.data != nullptr) {
            loadedImages.emplace_back(src);
        }
    }
    // Benchmark
    chrono::steady_clock::time_point now = chrono::steady_clock::now();
    cv::Ptr<cv::cuda::Filter> gaussian = cv::cuda::createBoxFilter(0, 0,cv::Size(3,3));
    cv::Ptr<cv::cuda::CannyEdgeDetector> canny = cv::cuda::createCannyEdgeDetector(100.0, 300.0);
    for (const auto &loadedImage : loadedImages) {
        cv::cuda::GpuMat src, resized, blurred, edges;
        chrono::steady_clock::time_point uploadNow = chrono::steady_clock::now();
        src.upload(loadedImage);
        chrono::steady_clock::time_point uploadEnd = chrono::steady_clock::now();

        chrono::steady_clock::time_point resizeNow = chrono::steady_clock::now();
        cv::cuda::resize(src, resized, cv::Size(1280, 720),0,0,cv::INTER_CUBIC);
        chrono::steady_clock::time_point resizeEnd = chrono::steady_clock::now();

        chrono::steady_clock::time_point blurNow = chrono::steady_clock::now();
        gaussian->apply(resized, blurred);
        chrono::steady_clock::time_point blurEnd = chrono::steady_clock::now();

        chrono::steady_clock::time_point cannyNow = chrono::steady_clock::now();
        canny->detect(blurred, edges);
        chrono::steady_clock::time_point cannyEnd = chrono::steady_clock::now();

        cv::Mat dest;

        chrono::steady_clock::time_point downloadNow = chrono::steady_clock::now();
        edges.download(dest);
        chrono::steady_clock::time_point downloadEnd = chrono::steady_clock::now();

        cout << "OpenCV runtime (GPU) - Upload: " << chrono::duration_cast<chrono::nanoseconds>(uploadEnd - uploadNow).count() << " nanoseconds" << endl;
        cout << "OpenCV runtime (GPU) - Resize: " << chrono::duration_cast<chrono::nanoseconds>(resizeEnd - resizeNow).count() << " nanoseconds" << endl;
        cout << "OpenCV runtime (GPU) - Blur: " << chrono::duration_cast<chrono::nanoseconds>(blurEnd - blurNow).count() << " nanoseconds" << endl;
        cout << "OpenCV runtime (GPU) - Canny: " << chrono::duration_cast<chrono::nanoseconds>(cannyEnd - cannyNow).count() << " nanoseconds" << endl;
        cout << "OpenCV runtime (GPU) - Download: " << chrono::duration_cast<chrono::nanoseconds>(downloadEnd - downloadNow).count() << " nanoseconds" << endl;
    }
    chrono::steady_clock::time_point end = chrono::steady_clock::now();
    cout << "OpenCV runtime (GPU): " << chrono::duration_cast<chrono::milliseconds>(end - now).count() << " milliseconds" << endl;
    loadedImages.clear();
}

int main(int argc, char *argv[]) {
    cv::cuda::printCudaDeviceInfo(cv::cuda::getDevice());
    run(images);
    runCUDA(images);
    return 0;
}