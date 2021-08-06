#include "tools.h"
#include <opencv2/imgcodecs.hpp>
#include <thread>
#include <opencv2/imgproc.hpp>
#include <opencv2/cudaimgproc.hpp>
#include <boost/program_options.hpp>

namespace gpu = cv::cuda;
namespace po = boost::program_options;


/**
 * Benchmark to measure efficiency of filters
 */

long benchmarkCPU(const string &inputDirectory, const string &cpuOutputDirectory) {
    // Load all in memory to simulate receiving messages from the ROS provider
    vector<pair<cv::String, cv::Mat>> loadedImages = tools::loadImages(inputDirectory);

    // Benchmark
    long totalBilateral = 0;
    long totalThreshold = 0;
    long totalHistogram = 0;
    long totalBoxFilter = 0;
    vector<pair<cv::String, cv::Mat>> outputImages;
    for (const auto &loadedImage : loadedImages) {
        cv::Mat resized, bilateralMat, thresholdMat, histogramMat, boxFilterMat;

        cv::resize(loadedImage.second, resized, cv::Size(1280, 720), 0, 0, cv::INTER_CUBIC);

        chrono::steady_clock::time_point bilateralStart = chrono::steady_clock::now();
        cv::bilateralFilter(resized, bilateralMat, 3, 0, 0);
        chrono::steady_clock::time_point bilateralEnd = chrono::steady_clock::now();
        totalBilateral += chrono::duration_cast<chrono::nanoseconds>(bilateralEnd - bilateralStart).count();
        outputImages.emplace_back(make_pair("bilateral-" + loadedImage.first, bilateralMat));

        chrono::steady_clock::time_point thresholdStart = chrono::steady_clock::now();
        cv::threshold(resized, thresholdMat, 128.0, 255.0, CV_THRESH_BINARY);
        chrono::steady_clock::time_point thresholdEnd = chrono::steady_clock::now();
        totalThreshold += chrono::duration_cast<chrono::nanoseconds>(thresholdEnd - thresholdStart).count();
        outputImages.emplace_back(make_pair("threshold-" + loadedImage.first, thresholdMat));

        chrono::steady_clock::time_point histogramStart = chrono::steady_clock::now();
        cv::equalizeHist(resized, histogramMat);
        chrono::steady_clock::time_point histogramEnd = chrono::steady_clock::now();
        totalHistogram += chrono::duration_cast<chrono::nanoseconds>(histogramEnd - histogramStart).count();
        outputImages.emplace_back(make_pair("histogram-" + loadedImage.first, histogramMat));


        chrono::steady_clock::time_point boxFilterStart = chrono::steady_clock::now();
        cv::boxFilter(resized, boxFilterMat, resized.depth(), cv::Size(3, 3));
        chrono::steady_clock::time_point boxFilterEnd = chrono::steady_clock::now();
        totalBoxFilter += chrono::duration_cast<chrono::nanoseconds>(boxFilterEnd - boxFilterStart).count();
        outputImages.emplace_back(make_pair("boxfilter-" + loadedImage.first, boxFilterMat));
    }

    long totalAverageTime = (totalBilateral + totalThreshold + totalHistogram + totalBoxFilter) / loadedImages.size();
    cout << "OpenCV runtime (CPU) - Total Average Time Per Image: " << totalAverageTime << " nanoseconds" << endl;
    cout << "OpenCV runtime (CPU) - Bilateral Average: " << totalBilateral / loadedImages.size() << " nanoseconds"
         << endl;
    cout << "OpenCV runtime (CPU) - Threshold Average: " << totalThreshold / loadedImages.size() << " nanoseconds"
         << endl;
    cout << "OpenCV runtime (CPU) - Histogram Average: " << totalHistogram / loadedImages.size() << " nanoseconds"
         << endl;
    cout << "OpenCV runtime (CPU) - Box Filter Average: " << totalBoxFilter / loadedImages.size() << " nanoseconds"
         << endl;
    tools::writeImages(cpuOutputDirectory, outputImages);
    loadedImages.clear();
    return totalAverageTime;
}

long benchmarkGPU(const string &directory, const string &gpuOutputDirectory) {
    // Load all in memory to simulate receiving messages from the ROS provider
    vector<pair<cv::String, cv::Mat>> loadedImages = tools::loadImages(directory);

    // Benchmark
    long totalBilateral = 0;
    long totalThreshold = 0;
    long totalHistogram = 0;
    long totalBoxFilter = 0;
    vector<pair<cv::String, cv::Mat>> outputImages;
    cv::Ptr<gpu::Filter> boxFilter = gpu::createBoxFilter(0, 0, cv::Size(3, 3));
    for (const auto &loadedImage : loadedImages) {
        gpu::GpuMat src, resized, bilateralGpuMat, thresholdGpuMat, histogramGpuMat, boxFilterGpuMat;
        cv::Mat bilateralMat, thresholdMat, histogramMat, boxFilterMat;

        src.upload(loadedImage.second);
        gpu::resize(src, resized, cv::Size(1280, 720), 0, 0, cv::INTER_CUBIC);

        chrono::steady_clock::time_point bilateralStart = chrono::steady_clock::now();
        gpu::bilateralFilter(resized, bilateralGpuMat, 3, 0, 0);
        chrono::steady_clock::time_point bilateralEnd = chrono::steady_clock::now();
        totalBilateral += chrono::duration_cast<chrono::nanoseconds>(bilateralEnd - bilateralStart).count();
        bilateralGpuMat.download(bilateralMat);
        outputImages.emplace_back(make_pair("bilateral-" + loadedImage.first, bilateralMat));

        chrono::steady_clock::time_point thresholdStart = chrono::steady_clock::now();
        gpu::threshold(resized, thresholdGpuMat, 128.0, 255.0, CV_THRESH_BINARY);
        chrono::steady_clock::time_point thresholdEnd = chrono::steady_clock::now();
        totalThreshold += chrono::duration_cast<chrono::nanoseconds>(thresholdEnd - thresholdStart).count();
        thresholdGpuMat.download(thresholdMat);
        outputImages.emplace_back(make_pair("threshold-" + loadedImage.first, thresholdMat));

        chrono::steady_clock::time_point histogramStart = chrono::steady_clock::now();
        gpu::equalizeHist(resized, histogramGpuMat);
        chrono::steady_clock::time_point histogramEnd = chrono::steady_clock::now();
        totalHistogram += chrono::duration_cast<chrono::nanoseconds>(histogramEnd - histogramStart).count();
        histogramGpuMat.download(histogramMat);
        outputImages.emplace_back(make_pair("histogram-" + loadedImage.first, histogramMat));


        chrono::steady_clock::time_point boxFilterStart = chrono::steady_clock::now();
        boxFilter->execute(resized, boxFilterGpuMat);
        chrono::steady_clock::time_point boxFilterEnd = chrono::steady_clock::now();
        totalBoxFilter += chrono::duration_cast<chrono::nanoseconds>(boxFilterEnd - boxFilterStart).count();
        boxFilterGpuMat.download(boxFilterMat);
        outputImages.emplace_back(make_pair("boxfilter-" + loadedImage.first, boxFilterMat));
    }
    long totalAverageTime = (totalBilateral + totalThreshold + totalHistogram + totalBoxFilter) / loadedImages.size();
    cout << "OpenCV runtime (GPU) - Total Average Time Per Image: " << totalAverageTime << " nanoseconds" << endl;
    cout << "OpenCV runtime (GPU) - Bilateral Average: " << totalBilateral / loadedImages.size() << " nanoseconds"
         << endl;
    cout << "OpenCV runtime (GPU) - Threshold Average: " << totalThreshold / loadedImages.size() << " nanoseconds"
         << endl;
    cout << "OpenCV runtime (GPU) - Histogram Average: " << totalHistogram / loadedImages.size() << " nanoseconds"
         << endl;
    cout << "OpenCV runtime (GPU) - Box Filter Average: " << totalBoxFilter / loadedImages.size() << " nanoseconds"
         << endl;
    tools::writeImages(gpuOutputDirectory, outputImages);
    loadedImages.clear();
    return totalAverageTime;
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
            long cpuTime = benchmarkCPU(inputDirectory.generic_string(), cpuOutputDirectory.generic_string());
            long gpuTime = benchmarkGPU(inputDirectory.generic_string(), gpuOutputDirectory.generic_string());
            double gain = cpuTime / gpuTime;
            cout << "GPU Speed Factor Gain Over CPU: " << gain << endl;
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