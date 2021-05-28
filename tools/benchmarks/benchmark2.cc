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
 * Benchmark without GPU streams (1:1 comparison)
 */

using namespace std;

void addTrailingSlash(fs::path &path);

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
    int totalTimeResize = 0;
    int totalTimeBlur = 0;
    int totalTimeCanny = 0;
    int totalTimeCopyTo = 0;
    for (const auto &loadedImage : loadedImages) {
        cv::Mat resized, blurred, edges, dest;

        chrono::steady_clock::time_point resizeNow = chrono::steady_clock::now();
        resize(loadedImage.second, resized, cv::Size(1280, 720), 0, 0, cv::INTER_CUBIC);
        chrono::steady_clock::time_point resizeEnd = chrono::steady_clock::now();
        totalTimeResize += chrono::duration_cast<chrono::nanoseconds>(resizeEnd - resizeNow).count();

        chrono::steady_clock::time_point blurNow = chrono::steady_clock::now();
        blur(resized, blurred, cv::Size(3, 3));
        chrono::steady_clock::time_point blurEnd = chrono::steady_clock::now();
        totalTimeBlur += chrono::duration_cast<chrono::nanoseconds>(blurEnd - blurNow).count();


        chrono::steady_clock::time_point cannyNow = chrono::steady_clock::now();
        Canny(blurred, edges, 100, 100 * 3);
        chrono::steady_clock::time_point cannyEnd = chrono::steady_clock::now();
        totalTimeCanny += chrono::duration_cast<chrono::nanoseconds>(cannyEnd - cannyNow).count();

        chrono::steady_clock::time_point copyToNow = chrono::steady_clock::now();
        dest = cv::Scalar::all(0);
        resized.copyTo(dest, edges);
        chrono::steady_clock::time_point copyToEnd = chrono::steady_clock::now();
        totalTimeCopyTo += chrono::duration_cast<chrono::nanoseconds>(copyToEnd - copyToNow).count();
    }
    chrono::steady_clock::time_point end = chrono::steady_clock::now();
    cout << "OpenCV runtime (CPU): " << chrono::duration_cast<chrono::milliseconds>(end - now).count()
         << " milliseconds" << endl;
    cout << "OpenCV runtime (CPU) - Resize Average: " << totalTimeResize / loadedImages.size() << " nanoseconds"
         << endl;
    cout << "OpenCV runtime (CPU) - Blur Average: " << totalTimeBlur / loadedImages.size() << " nanoseconds" << endl;
    cout << "OpenCV runtime (CPU) - Canny Average: " << totalTimeCanny / loadedImages.size() << " nanoseconds" << endl;
    cout << "OpenCV runtime (CPU) - Copy To Average: " << totalTimeCopyTo / loadedImages.size() << " nanoseconds" << endl;
    writeImages(cpuOutputDirectory, loadedImages);
    loadedImages.clear();
}

void runCUDA(const string &directory, const string &gpuOutputDirectory) {
    // Load all in memory to simulate receiving messages from the ROS provider
    vector<pair<cv::String, cv::Mat>> loadedImages = loadImages(directory);

    // Benchmark
    chrono::steady_clock::time_point now = chrono::steady_clock::now();
    cv::Ptr<gpu::Filter> gaussian = gpu::createBoxFilter(0, 0, cv::Size(3, 3));
    cv::Ptr<gpu::CannyEdgeDetector> canny = gpu::createCannyEdgeDetector(100.0, 300.0);
    int totalTimeResize = 0;
    int totalTimeBlur = 0;
    int totalTimeCanny = 0;
    int totalTimeCopyTo = 0;
    for (auto &loadedImage : loadedImages) {
        cv::Mat out;
        gpu::GpuMat src, resized, blurred, edges;
        chrono::steady_clock::time_point uploadNow = chrono::steady_clock::now();
        src.upload(loadedImage.second);
        chrono::steady_clock::time_point uploadEnd = chrono::steady_clock::now();

        chrono::steady_clock::time_point resizeNow = chrono::steady_clock::now();
        gpu::resize(src, resized, cv::Size(1280, 720), 0, 0, cv::INTER_CUBIC);
        chrono::steady_clock::time_point resizeEnd = chrono::steady_clock::now();
        totalTimeResize += chrono::duration_cast<chrono::nanoseconds>(resizeEnd - resizeNow).count();

        chrono::steady_clock::time_point blurNow = chrono::steady_clock::now();
        gaussian->apply(resized, blurred);
        chrono::steady_clock::time_point blurEnd = chrono::steady_clock::now();
        totalTimeBlur += chrono::duration_cast<chrono::nanoseconds>(blurEnd - blurNow).count();

        chrono::steady_clock::time_point cannyNow = chrono::steady_clock::now();
        canny->detect(blurred, edges);
        chrono::steady_clock::time_point cannyEnd = chrono::steady_clock::now();
        totalTimeCanny += chrono::duration_cast<chrono::nanoseconds>(cannyEnd - cannyNow).count();

        chrono::steady_clock::time_point copyToNow = chrono::steady_clock::now();
        gpu::GpuMat dest(resized.size(), resized.type(), 0xFF);
        resized.copyTo(dest, edges);
        chrono::steady_clock::time_point copyToEnd = chrono::steady_clock::now();
        totalTimeCopyTo += chrono::duration_cast<chrono::nanoseconds>(copyToEnd - copyToNow).count();

        chrono::steady_clock::time_point downloadNow = chrono::steady_clock::now();
        dest.download(out);
        chrono::steady_clock::time_point downloadEnd = chrono::steady_clock::now();

        loadedImage.second = out;
    }
    chrono::steady_clock::time_point end = chrono::steady_clock::now();
    cout << "OpenCV runtime (GPU): " << chrono::duration_cast<chrono::milliseconds>(end - now).count()
         << " milliseconds" << endl;
    cout << "OpenCV runtime (GPU) - Resize Average: " << totalTimeResize / loadedImages.size() << " nanoseconds"
         << endl;
    cout << "OpenCV runtime (GPU) - Blur Average: " << totalTimeBlur / loadedImages.size() << " nanoseconds" << endl;
    cout << "OpenCV runtime (GPU) - Canny Average: " << totalTimeCanny / loadedImages.size() << " nanoseconds" << endl;
    cout << "OpenCV runtime (GPU) - Copy To Average: " << totalTimeCopyTo / loadedImages.size() << " nanoseconds" << endl;
    writeImages(gpuOutputDirectory, loadedImages);
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