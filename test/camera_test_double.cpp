#include <fmt/core.h>

#include <chrono>
#include <fstream>
// #include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>

#include "io/camera.hpp"
#include "tools/img_tools.hpp"
#include "tools/exiter.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tasks/auto_aim/aimer.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tasks/auto_aim/detector.hpp"

const std::string keys =
    "{help h usage ? |      | 输出命令行参数说明}"
    "{r | configs/camera_r.yaml | 右相机配置文件路径 (短名-r) }"  // 移除 @
    "{l | configs/camera_l.yaml | 左相机配置文件路径 (短名-l) }"; // 移除 @

int main(int argc, char * argv[])
{
    cv::CommandLineParser parser(argc, argv, keys);
    auto config_path_r = parser.get<std::string>("r");
    auto config_path_l = parser.get<std::string>("l");

    tools::logger()->info("Reading config R from: {}", config_path_r);
    tools::logger()->info("Reading config L from: {}", config_path_l);

    tools::Exiter exiter;

    cv::Mat img_r,img_l,draw_img_r,draw_img_l;

    auto_aim::YOLO yolo_r(config_path_r,false);
    auto_aim::Detector detector_r(config_path_r,false);
    auto_aim::Solver solver_r(config_path_r);
    auto_aim::Tracker tracker_r(config_path_r,solver_r);

    auto_aim::YOLO yolo_l(config_path_l,false);
    auto_aim::Detector detector_l(config_path_l,false);
    auto_aim::Solver solver_l(config_path_l);
    auto_aim::Tracker tracker_l(config_path_l,solver_r);

    io::Camera camera_r(config_path_r);
    io::Camera camera_l(config_path_l);

    std::list<auto_aim::Armor> armors_r;
    std::list<auto_aim::Target> targets_r;

    std::list<auto_aim::Armor> armors_l;
    std::list<auto_aim::Target> targets_l;

    std::chrono::steady_clock::time_point timestamp_r;
    std::chrono::steady_clock::time_point timestamp_l;


    while(!exiter.exit()){
        camera_r.read(img_r, timestamp_r);
        camera_l.read(img_l, timestamp_l);
        if(img_r.empty()||img_l.empty()){
            break;
        }
        auto last = std::chrono::steady_clock::now();

        // armors_r = detector_r.detect(img_r);
        armors_r = yolo_r.detect(img_r);
        draw_img_r = img_r.clone();
        for(const auto & armor : armors_r){
            std::vector<cv::Point> armor_point(4);
            for(int i=0;i<4;i++){
                armor_point[i] = cv::Point(static_cast<int>(armor.points[i].x), static_cast<int>(armor.points[i].y));
            }
            cv::polylines(draw_img_r, std::vector<std::vector<cv::Point>>{armor_point}, true, cv::Scalar(0, 255, 0), 2);

            tools::draw_text(draw_img_r, fmt::format("ID:{} conf{:.2f}",armor.name,armor.confidence), armor.center);
        }

        // armors_l = detector_l.detect(img_l);
        armors_l = yolo_l.detect(img_l);
        draw_img_l = img_l.clone();
        for(const auto & armor : armors_l){
            std::vector<cv::Point> armor_point(4);
            for(int i=0;i<4;i++){
                armor_point[i] = cv::Point(static_cast<int>(armor.points[i].x), static_cast<int>(armor.points[i].y));
            }
            cv::polylines(draw_img_l, std::vector<std::vector<cv::Point>>{armor_point}, true, cv::Scalar(0, 255, 0), 2);

            tools::draw_text(draw_img_l, fmt::format("ID:{} conf{:.2f}",armor.name,armor.confidence), armor.center);
        }


        targets_r = tracker_r.track(armors_r, timestamp_r);
        targets_l = tracker_l.track(armors_l, timestamp_l);

        if (!targets_r.empty())
        {
            auto target = targets_r.front();
            tools::draw_text(draw_img_r, fmt::format("[{}]", tracker_r.state()), {10, 30}, {255, 255, 255});

            // 当前帧target更新后
            std::vector<Eigen::Vector4d> armor_xyza_list = target.armor_xyza_list();
            for (const Eigen::Vector4d &xyza : armor_xyza_list)
            {
                auto image_points =
                    solver_r.reproject_armor(xyza.head(3), xyza[3], target.armor_type, target.name);
                tools::draw_points(draw_img_r, image_points, {0, 255, 255});
            }
        }
        if (!targets_l.empty())
        {
            auto target = targets_l.front();
            tools::draw_text(draw_img_l, fmt::format("[{}]", tracker_l.state()), {10, 30}, {255, 255, 255});

            // 当前帧target更新后
            std::vector<Eigen::Vector4d> armor_xyza_list = target.armor_xyza_list();
            for (const Eigen::Vector4d &xyza : armor_xyza_list)
            {
                auto image_points =
                    solver_r.reproject_armor(xyza.head(3), xyza[3], target.armor_type, target.name);
                tools::draw_points(draw_img_r, image_points, {0, 255, 255});
            }
        }
        auto now = std::chrono::steady_clock::now();
        auto dt = tools::delta_time(now, last);
        tools::logger()->info("{:.2f} fps", 1 / dt);

        cv::resize(draw_img_r, draw_img_r, {}, 0.5, 0.5); // 显示时缩小图片尺寸
        cv::resize(draw_img_l, draw_img_l, {}, 0.5, 0.5); // 显示时缩小图片尺寸
        // cv::putText(draw_img_r, fmt::format("R"), {10, 30}, cv::FONT_HERSHEY_SIMPLEX, 1, {0, 255, 0}, 2);
        cv::imshow("R", draw_img_r);
        cv::imshow("L", draw_img_l);


        if(cv::waitKey(1)=='q') break;
    }
    return 0;
}