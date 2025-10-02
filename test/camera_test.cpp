#include <fmt/core.h>

#include <chrono>
#include <fstream>
// #include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>

#include "io/camera.hpp"
#include "tools/img_tools.hpp"
#include "tools/exiter.hpp"
#include "tools/logger.hpp"

const std::string keys =
    "{help h usage ? |      | 输出命令行参数说明}"
    "{config-path c  | configs/test.yaml | yaml配置文件的路径}";

int main(int argc, char *argv[])
{
    cv::CommandLineParser parser(argc, argv, keys);
    auto config_path = parser.get<std::string>("config-path");

    tools::Exiter exiter;

    cv::Mat img;

    io::Camera camera(config_path);

    std::chrono::steady_clock::time_point timestamp;

    while (!exiter.exit())
    {
        camera.read(img, timestamp);
        // camera_l.read(img_l, timestamp_l);
        if (img.empty())
        {
            break;
        }
        cv::resize(img, img, {}, 0.5, 0.5);

        cv::imshow("camera", img);

        if (cv::waitKey(1) == 'q')
            break;
    }
}