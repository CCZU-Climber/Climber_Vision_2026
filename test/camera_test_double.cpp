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
    "{r | configs/test.yaml | 右相机配置文件路径 (短名-r) }"  // 移除 @
    "{l | configs/test.yaml | 左相机配置文件路径 (短名-l) }"; // 移除 @

int main(int argc, char * argv[])
{
    cv::CommandLineParser parser(argc, argv, keys);
    auto config_path_r = parser.get<std::string>("r");
    auto config_path_l = parser.get<std::string>("l");

    tools::logger()->info("Reading config R from: {}", config_path_r);
    tools::logger()->info("Reading config L from: {}", config_path_l);

    tools::Exiter exiter;

    cv::Mat img_r,img_l;

    io::Camera camera_r(config_path_r);
    io::Camera camera_l(config_path_l);

    std::chrono::steady_clock::time_point timestamp_r;
    std::chrono::steady_clock::time_point timestamp_l;

    while(!exiter.exit()){
        camera_r.read(img_r, timestamp_r);
        camera_l.read(img_l, timestamp_l);
        if(img_r.empty()){
            break;
        }
        cv::resize(img_r, img_r, {}, 0.5, 0.5);
        cv::resize(img_l, img_l, {}, 0.5, 0.5);

        cv::imshow("camera_l", img_l);
        cv::imshow("camera_r", img_r);

        if(cv::waitKey(1)=='q') break;
    }
}