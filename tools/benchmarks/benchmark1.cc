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
#include <map>

using namespace std;


map<cv::String, cv::Mat> loadImages(const string &directory) {
    DIR *dir;
    map<cv::String, cv::Mat> medias;
    struct dirent *file;
    if ((dir = opendir(directory.c_str())) != nullptr) {
        while ((file = readdir(dir)) != nullptr) {
            cv::Mat m = imread(directory + "/" + file->d_name, cv::IMREAD_GRAYSCALE);
            cv::String path = file->d_name;
            if (file->d_type != DT_DIR && m.data != nullptr){
                medias.insert(make_pair(path, m));
            }
        }
        closedir(dir);
    }
    return medias;
}

void run(const string &directory) {
    // Load all in memory to simulate receiving messages from the ROS provider
    map<cv::String, cv::Mat> loadedImages = loadImages(directory);
    // Benchmark
    chrono::steady_clock::time_point now = chrono::steady_clock::now();
    for (auto &loadedImage : loadedImages) {
        cv::Mat resized, blurred, edges, dest;
        resize(loadedImage.second, resized, cv::Size(1280, 720), 0, 0, cv::INTER_CUBIC);
        blur(resized, blurred, cv::Size(3, 3));
        Canny(blurred, edges, 100, 100 * 3);
        dest = cv::Scalar::all(0);
        resized.copyTo(dest, edges);
        loadedImage.second = dest;
    }
    chrono::steady_clock::time_point end = chrono::steady_clock::now();
    cout << "OpenCV runtime (CPU): " << chrono::duration_cast<chrono::milliseconds>(end - now).count() << " milliseconds" << endl;
    cout << "OpenCV runtime (CPU): " << chrono::duration_cast<chrono::nanoseconds>(end - now).count() << " nanoseconds" << endl;
    for (const auto &out : loadedImages){
        imwrite("/tmp/cpu/" + out.first, out.second);
    }
    loadedImages.clear();
}

void runCUDA(const string &directory) {
    shared_ptr<vector<cv::cuda::Stream>> cudaStreams = make_shared<vector<cv::cuda::Stream>>();
    shared_ptr<vector<cv::cuda::HostMem >> srcMemArray = make_shared<vector<cv::cuda::HostMem >>();
    shared_ptr<vector<cv::cuda::HostMem >> dstMemArray = make_shared<vector<cv::cuda::HostMem >>();
    shared_ptr<vector< cv::cuda::GpuMat >> gpuSrcArray = make_shared<vector<cv::cuda::GpuMat>>();
    shared_ptr<vector< cv::cuda::GpuMat >> gpuDstArray = make_shared<vector<cv::cuda::GpuMat>>();
    shared_ptr<vector< cv::Mat >> outArray = make_shared<vector<cv::Mat>>();
    // Load all in memory to simulate receiving messages from the ROS provider
    map<cv::String, cv::Mat> loadedImages = loadImages(directory);
    for (const auto &image : loadedImages) {
        // cout << image << endl;
        cv::Mat img = imread(directory + "/" + image.first, cv::IMREAD_GRAYSCALE);
        if (img.data != nullptr) {
            cv::cuda::Stream s;
            cudaStreams->emplace_back(s);
            cv::cuda::GpuMat srcMat;
            cv::cuda::GpuMat dstMat;
            cv::Mat outMat;
            cv::cuda::HostMem srcHostMem = cv::cuda::HostMem(img, cv::cuda::HostMem::PAGE_LOCKED);
            cv::cuda::HostMem srcDstMem = cv::cuda::HostMem(outMat, cv::cuda::HostMem::PAGE_LOCKED);
            srcMemArray->emplace_back(srcHostMem);
            dstMemArray->emplace_back(srcDstMem);
            gpuSrcArray->emplace_back(srcMat);
            gpuDstArray->emplace_back(dstMat);
            outArray->emplace_back(outMat);
        }
    }
    // Benchmark
    chrono::steady_clock::time_point now = chrono::steady_clock::now();
    cv::Ptr<cv::cuda::Filter> gaussian = cv::cuda::createBoxFilter(0, 0,cv::Size(3,3));
    cv::Ptr<cv::cuda::CannyEdgeDetector> canny = cv::cuda::createCannyEdgeDetector(100.0, 300.0);
    for(int i =0; i < loadedImages.size(); i++){
        (*gpuSrcArray)[i].upload((*srcMemArray)[i], (*cudaStreams)[i]);
        cv::cuda::GpuMat resized, blurred, edges;
        cv::cuda::resize((*gpuSrcArray)[i], resized, cv::Size(1280, 720),0,0,cv::INTER_CUBIC);

        gaussian->apply(resized, blurred);

        canny->detect(blurred, edges);
        (*gpuDstArray)[i].download((*dstMemArray)[i], (*cudaStreams)[i]);
        (*outArray)[i]= (*dstMemArray)[i].createMatHeader();
    }
    for (int i = 0; i < cudaStreams->size(); i++){
        (*cudaStreams)[i]..waitForCompletion();
    }
    chrono::steady_clock::time_point end = chrono::steady_clock::now();
    cout << "OpenCV runtime (GPU): " << chrono::duration_cast<chrono::milliseconds>(end - now).count()
         << " milliseconds" << endl;
    cout << "OpenCV runtime (GPU): " << chrono::duration_cast<chrono::nanoseconds>(end - now).count() << " nanoseconds"
         << endl;
    for (int i =0; i < outArray->size();i++){
        imwrite("/tmp/cuda/" + to_string(i) + ".png",(*outArray)[i]);
    }
    loadedImages.clear();
}

int start(int argc, char *argv[]) {
    run("/home/sonia/ros_sonia_ws/src/proc_image_processing/imgs");
    runCUDA("/home/sonia/ros_sonia_ws/src/proc_image_processing/imgs");
    return 0;
}