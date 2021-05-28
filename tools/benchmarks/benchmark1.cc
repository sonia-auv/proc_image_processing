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
#include <boost/program_options.hpp>
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>

using namespace std;
namespace gpu = cv::cuda;
namespace po = boost::program_options;
namespace fs = boost::filesystem;

/**
 * Benchmark using GPU streams (Async calls with callbacks)
 */

void addTrailingSlash(fs::path &path){
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

void run(const string &inputDirectory, const string &cpuOutputDirectory) {
    // Load all in memory to simulate receiving messages from the ROS provider
    vector<pair<cv::String, cv::Mat>> loadedImages = loadImages(inputDirectory);
    // Benchmark
    chrono::steady_clock::time_point now = chrono::steady_clock::now();
    for (auto &loadedImage : loadedImages) {
        cv::Mat resized, blurred, edges, dest;
        resize(loadedImage.second, resized, cv::Size(1280, 720), 0, 0, cv::INTER_CUBIC);
        blur(resized, blurred, cv::Size(3, 3));
        Canny(blurred, edges, 100.0, 300.0);
        dest = cv::Scalar::all(0);
        resized.copyTo(dest, edges);
        loadedImage.second = dest;
    }
    chrono::steady_clock::time_point end = chrono::steady_clock::now();
    cout << "OpenCV runtime (CPU): " << chrono::duration_cast<chrono::milliseconds>(end - now).count()
         << " milliseconds" << endl;
    cout << "OpenCV runtime (CPU): " << chrono::duration_cast<chrono::nanoseconds>(end - now).count() << " nanoseconds"
         << endl;
    writeImages(cpuOutputDirectory, loadedImages);
    loadedImages.clear();
}

void runCUDA(const string &directory, const string &gpuOutputDirectory) {
    // See https://developer.ridgerun.com/wiki/index.php?title=How_to_use_OpenCV_CUDA_Streams
    shared_ptr<vector<gpu::Stream>> cudaStreams = make_shared<vector<gpu::Stream>>();
    shared_ptr<vector<gpu::HostMem>> srcMemArray = make_shared<vector<gpu::HostMem>>();
    shared_ptr<vector<gpu::HostMem>> dstMemArray = make_shared<vector<gpu::HostMem>>();
    shared_ptr<vector<gpu::GpuMat>> gpuSrcArray = make_shared<vector<gpu::GpuMat>>();
    shared_ptr<vector<gpu::GpuMat>> gpuDstArray = make_shared<vector<gpu::GpuMat>>();
    shared_ptr<vector<pair<cv::String, cv::Mat>>> out = make_shared<vector<pair<cv::String, cv::Mat>>>();
    // Load all in memory to simulate receiving messages from the ROS provider
    vector<pair<cv::String, cv::Mat>> loadedImages = loadImages(directory);
    for (auto &image : loadedImages) {
        gpu::Stream s;
        cudaStreams->emplace_back(s);

        cv::Mat outMat;
        gpu::HostMem srcHostMem = gpu::HostMem(image.second, gpu::HostMem::PAGE_LOCKED);
        gpu::HostMem srcDstMem = gpu::HostMem(outMat, gpu::HostMem::PAGE_LOCKED);
        srcMemArray->emplace_back(srcHostMem);
        dstMemArray->emplace_back(srcDstMem);
        out->emplace_back(make_pair(image.first, outMat));

        gpu::GpuMat srcMat;
        gpu::GpuMat dstMat;
        gpuSrcArray->emplace_back(srcMat);
        gpuDstArray->emplace_back(dstMat);
    }
    // Benchmark
    chrono::steady_clock::time_point now = chrono::steady_clock::now();
    cv::Ptr<gpu::Filter> gaussian = gpu::createBoxFilter(0, 0, cv::Size(3, 3));
    cv::Ptr<gpu::CannyEdgeDetector> canny = gpu::createCannyEdgeDetector(100.0, 300.0);
    for (int i = 0; i < loadedImages.size(); i++) {
        (*gpuSrcArray)[i].upload((*srcMemArray)[i], (*cudaStreams)[i]);
        gpu::GpuMat resized, blurred, edges;

        gpu::resize((*gpuSrcArray)[i], resized, cv::Size(1280, 720), 0, 0, cv::INTER_CUBIC, (*cudaStreams)[i]);

        gaussian->apply(resized, blurred, (*cudaStreams)[i]);

        canny->detect(blurred, (*gpuDstArray)[i]);

        gpu::GpuMat dest(resized.size(), resized.type(), 0xFF);
        resized.copyTo(dest, (*gpuDstArray)[i]);

        dest.download((*dstMemArray)[i], (*cudaStreams)[i]);
        (*out)[i].second = (*dstMemArray)[i].createMatHeader();
    }
    for (int i = 0; i < cudaStreams->size(); i++) {
        (*cudaStreams)[i].waitForCompletion();
    }
    chrono::steady_clock::time_point end = chrono::steady_clock::now();
    cout << "OpenCV runtime (GPU): " << chrono::duration_cast<chrono::milliseconds>(end - now).count()
         << " milliseconds" << endl;
    cout << "OpenCV runtime (GPU): " << chrono::duration_cast<chrono::nanoseconds>(end - now).count() << " nanoseconds"
         << endl;
    writeImages(gpuOutputDirectory, (*out));
    loadedImages.clear();
}

int main(int argc, char *argv[]) {
    try {
        string baseDefaultValue = "/home/sonia/ros_sonia_ws/src/proc_image_processing/imgs/";
        po::options_description description("Valid parameters");
        description.add_options()
                ("help", "Output valid options")
                ("in", po::value<string>()->default_value(baseDefaultValue), "Directory to read images from")
                ("cpu-out", po::value<string>()->default_value(baseDefaultValue + "cpu/"),
                 "Directory to flush CPU images")
                ("gpu-out", po::value<string>()->default_value(baseDefaultValue + "gpu/"),
                 "Directory to flush GPU images");
        po::variables_map variables;
        po::store(po::parse_command_line(argc, argv, description), variables);
        po::notify(variables);

        if (variables.count("help")) {
            cout << description << endl;
            return 0;
        }

        if (variables.count("in") && variables.count("cpu-out") && variables.count("gpu-out")) {
            bool quit = false;
            fs::path inputDirectory(variables["in"].as<string>());
            addTrailingSlash(inputDirectory);
            if (!fs::is_directory(inputDirectory) || !fs::exists(inputDirectory)) {
                cout << "Input directory does not exist" << endl;
                quit = true;
            }

            fs::path cpuOutputDirectory(variables["cpu-out"].as<string>());
            addTrailingSlash(cpuOutputDirectory);
            if (!fs::is_directory(cpuOutputDirectory) || !fs::exists(cpuOutputDirectory)) {
                cout << "CPU output directory does not exist" << endl;
                quit = true;
            }

            fs::path gpuOutputDirectory(variables["gpu-out"].as<string>());
            addTrailingSlash(gpuOutputDirectory);
            if (!fs::is_directory(gpuOutputDirectory) || !fs::exists(gpuOutputDirectory)) {
                cout << "GPU output directory does not exist" << endl;
                quit = true;
            }
            if (quit) {
                return 1;
            }
            gpu::printCudaDeviceInfo(gpu::getDevice());
            run(inputDirectory.generic_string(), cpuOutputDirectory.generic_string());
            runCUDA(inputDirectory.generic_string(), gpuOutputDirectory.generic_string());
        } else {
            cout << "Invalid parameters." << endl;
            cout << description << endl;
            return 1;
        }
    } catch (exception &e) {
        cerr << "error: " << e.what() << endl;
        return 1;
    } catch (...) {
        cerr << "Unknown error!" << endl;
        return 1;
    }

    return 0;
}