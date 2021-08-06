#include "tools.h"
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/cudaimgproc.hpp>
#include <boost/program_options.hpp>

namespace gpu = cv::cuda;
namespace po = boost::program_options;


/**
 * Benchmark without GPU streams (1:1 comparison)
 */

void benchmarkCPU(const string &inputDirectory, const string &cpuOutputDirectory) {
    // Load all in memory to simulate receiving messages from the ROS provider
    vector<pair<cv::String, cv::Mat>> loadedImages = tools::loadImages(inputDirectory);

    // Benchmark
    long totalTimeResize = 0;
    long totalTimeBlur = 0;
    long totalTimeCanny = 0;
    long totalTimeCopyTo = 0;
    chrono::steady_clock::time_point now = chrono::steady_clock::now();
    for (const auto &loadedImage : loadedImages) {
        cv::Mat resized, blurred, edges, dest;

        chrono::steady_clock::time_point resizeStart = chrono::steady_clock::now();
        cv::resize(loadedImage.second, resized, cv::Size(1280, 720), 0, 0, cv::INTER_CUBIC);
        chrono::steady_clock::time_point resizeEnd = chrono::steady_clock::now();
        totalTimeResize += chrono::duration_cast<chrono::nanoseconds>(resizeEnd - resizeStart).count();

        chrono::steady_clock::time_point blurStart = chrono::steady_clock::now();
        cv::blur(resized, blurred, cv::Size(3, 3));
        chrono::steady_clock::time_point blurEnd = chrono::steady_clock::now();
        totalTimeBlur += chrono::duration_cast<chrono::nanoseconds>(blurEnd - blurStart).count();


        chrono::steady_clock::time_point cannyStart = chrono::steady_clock::now();
        cv::Canny(blurred, edges, 100.0, 300.0);
        chrono::steady_clock::time_point cannyEnd = chrono::steady_clock::now();
        totalTimeCanny += chrono::duration_cast<chrono::nanoseconds>(cannyEnd - cannyStart).count();

        chrono::steady_clock::time_point copyToStart = chrono::steady_clock::now();
        dest = cv::Scalar::all(0);
        resized.copyTo(dest, edges);
        chrono::steady_clock::time_point copyToEnd = chrono::steady_clock::now();
        totalTimeCopyTo += chrono::duration_cast<chrono::nanoseconds>(copyToEnd - copyToStart).count();
    }
    chrono::steady_clock::time_point end = chrono::steady_clock::now();
    cout << "OpenCV runtime (CPU): " << chrono::duration_cast<chrono::milliseconds>(end - now).count()
         << " milliseconds" << endl;
    cout << "OpenCV runtime (CPU) - Resize Average: " << totalTimeResize / loadedImages.size() << " nanoseconds"
         << endl;
    cout << "OpenCV runtime (CPU) - Blur Average: " << totalTimeBlur / loadedImages.size() << " nanoseconds" << endl;
    cout << "OpenCV runtime (CPU) - Canny Average: " << totalTimeCanny / loadedImages.size() << " nanoseconds" << endl;
    cout << "OpenCV runtime (CPU) - Copy To Average: " << totalTimeCopyTo / loadedImages.size() << " nanoseconds" << endl;
    tools::writeImages(cpuOutputDirectory, loadedImages);
    loadedImages.clear();
}

void benchmarkGPU(const string &directory, const string &gpuOutputDirectory) {
    // Load all in memory to simulate receiving messages from the ROS provider
    vector<pair<cv::String, cv::Mat>> loadedImages = tools::loadImages(directory);

    // Benchmark
    long totalTimeResize = 0;
    long totalTimeBlur = 0;
    long totalTimeCanny = 0;
    long totalTimeCopyTo = 0;
    cv::Ptr<gpu::Filter> gaussian = gpu::createBoxFilter(0, 0, cv::Size(3, 3));
    cv::Ptr<gpu::CannyEdgeDetector> canny = gpu::createCannyEdgeDetector(100.0, 300.0);
    chrono::steady_clock::time_point now = chrono::steady_clock::now();
    for (auto &loadedImage : loadedImages) {
        cv::Mat out;
        gpu::GpuMat src, resized, blurred, edges;
        chrono::steady_clock::time_point uploadStart = chrono::steady_clock::now();
        src.upload(loadedImage.second);
        chrono::steady_clock::time_point uploadEnd = chrono::steady_clock::now();

        chrono::steady_clock::time_point resizeStart = chrono::steady_clock::now();
        gpu::resize(src, resized, cv::Size(1280, 720), 0, 0, cv::INTER_CUBIC);
        chrono::steady_clock::time_point resizeEnd = chrono::steady_clock::now();
        totalTimeResize += chrono::duration_cast<chrono::nanoseconds>(resizeEnd - resizeStart).count();

        chrono::steady_clock::time_point blurStart = chrono::steady_clock::now();
        gaussian->execute(resized, blurred);
        chrono::steady_clock::time_point blurEnd = chrono::steady_clock::now();
        totalTimeBlur += chrono::duration_cast<chrono::nanoseconds>(blurEnd - blurStart).count();

        chrono::steady_clock::time_point cannyStart = chrono::steady_clock::now();
        canny->detect(blurred, edges);
        chrono::steady_clock::time_point cannyEnd = chrono::steady_clock::now();
        totalTimeCanny += chrono::duration_cast<chrono::nanoseconds>(cannyEnd - cannyStart).count();

        chrono::steady_clock::time_point copyToStart = chrono::steady_clock::now();
        gpu::GpuMat dest(resized.size(), resized.type(), 0xFF);
        resized.copyTo(dest, edges);
        chrono::steady_clock::time_point copyToEnd = chrono::steady_clock::now();
        totalTimeCopyTo += chrono::duration_cast<chrono::nanoseconds>(copyToEnd - copyToStart).count();

        chrono::steady_clock::time_point downloadStart = chrono::steady_clock::now();
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
    tools::writeImages(gpuOutputDirectory, loadedImages);
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
            tools::addTrailingSlash(inputDirectory);
            if (!fs::is_directory(inputDirectory) || !fs::exists(inputDirectory)) {
                cout << "Input directory does not exist" << endl;
                quit = true;
            }

            fs::path cpuOutputDirectory(variables["cpu-out"].as<string>());
            tools::addTrailingSlash(cpuOutputDirectory);
            if (!fs::is_directory(cpuOutputDirectory) || !fs::exists(cpuOutputDirectory)) {
                cout << "CPU output directory does not exist" << endl;
                quit = true;
            }

            fs::path gpuOutputDirectory(variables["gpu-out"].as<string>());
            tools::addTrailingSlash(gpuOutputDirectory);
            if (!fs::is_directory(gpuOutputDirectory) || !fs::exists(gpuOutputDirectory)) {
                cout << "GPU output directory does not exist" << endl;
                quit = true;
            }
            if (quit) {
                return 1;
            }
            gpu::printCudaDeviceInfo(gpu::getDevice());
            benchmarkCPU(inputDirectory.generic_string(), cpuOutputDirectory.generic_string());
            benchmarkGPU(inputDirectory.generic_string(), gpuOutputDirectory.generic_string());
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