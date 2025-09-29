#include <fmt/core.h>

#include <chrono>
#include <fstream>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>

#include "tasks/auto_aim/aimer.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tasks/auto_aim/detector.hpp"
#include "tasks/auto_aim/shooter.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"
#include "io/camera.hpp"

const std::string keys =
    "{help h usage ? |      | 输出命令行参数说明 }"
    "{config-path c  | configs/test.yaml | yaml配置文件的路径}"
    "{tradition t    | false | 是否使用传统方法识别}";

int main(int argc, char *argv[])
{
    // 读取命令行参数
    cv::CommandLineParser cli(argc, argv, keys);
    if (cli.has("help")) {
        cli.printMessage();
        return 0;
    }
    
    auto config_path = cli.get<std::string>("config-path");
    auto use_tradition = cli.get<bool>("tradition");
    
    if (!cli.check()) {
        cli.printErrors();
        return -1;
    }

    // tools::Plotter plotter;
    tools::Exiter exiter;

    auto_aim::YOLO yolo(config_path, false);  // 第二个参数是debug标志，不是文件名
    auto_aim::Detector detector(config_path, false);
    auto_aim::Solver solver(config_path);
    auto_aim::Tracker tracker(config_path, solver);
    auto_aim::Aimer aimer(config_path);
    auto_aim::Shooter shooter(config_path);

    cv::Mat img, drawing;
    std::list<auto_aim::Armor> armors;
    std::list<auto_aim::Target> targets;
    std::chrono::steady_clock::time_point t;

    // 添加错误处理，避免串口初始化失败导致段错误
    std::unique_ptr<io::CBoard> cboard_ptr = nullptr;
    try {
        cboard_ptr = std::make_unique<io::CBoard>(config_path);
    } catch (const std::exception& e) {
        tools::logger()->warn("无法初始化控制板: {}", e.what());
        tools::logger()->warn("将继续运行，但无法发送控制指令");
    }
    
    io::Camera camera(config_path);
    double last_t = -1;

    while (!exiter.exit())
    {
        camera.read(img, t);
        if (img.empty())
            break;

        auto last = std::chrono::steady_clock::now();

        // armors = detector.detect(img);
        armors = yolo.detect(img);
        cv::Mat draw_img = img.clone();
        for (const auto &armor : armors)
        {
            std::vector<cv::Point> armor_point(4);
            for (int i = 0; i < 4; i++)
            {
                armor_point[i] = cv::Point(static_cast<int>(armor.points[i].x), static_cast<int>(armor.points[i].y));
            }
            cv::polylines(draw_img, std::vector<std::vector<cv::Point>>{armor_point}, true, cv::Scalar(0, 255, 0), 2);

            tools::draw_text(draw_img, fmt::format("ID:{} conf{:.2f}", armor.name, armor.confidence), armor.center);
        }

        targets = tracker.track(armors, t);

        if (!targets.empty())
        {
            auto target = targets.front();
            tools::draw_text(draw_img, fmt::format("[{}]", tracker.state()), {10, 30}, {255, 255, 255});

            // 当前帧target更新后
            std::vector<Eigen::Vector4d> armor_xyza_list = target.armor_xyza_list();
            for (const Eigen::Vector4d &xyza : armor_xyza_list)
            {
                auto image_points =
                    solver.reproject_armor(xyza.head(3), xyza[3], target.armor_type, target.name);
                tools::draw_points(draw_img, image_points, {0, 255, 255});
            }

            auto now = std::chrono::steady_clock::now();

            auto dt = tools::delta_time(now, last);
            tools::logger()->info("{:.2f} fps", 1 / dt);
        }

        cv::resize(draw_img, draw_img, {}, 0.5, 0.5); // 显示时缩小图片尺寸
        cv::putText(draw_img, fmt::format("{}", use_tradition ? "CV" : "YOLO"), {10, 30}, cv::FONT_HERSHEY_SIMPLEX, 1, {0, 255, 0}, 2);
        cv::imshow("reprojection", draw_img);

        auto key = cv::waitKey(30);
        if (key == 'q')
            break;

        // 只有在控制板成功初始化的情况下才发送指令
        if (cboard_ptr) {
            auto command = aimer.aim(targets, t, cboard_ptr->bullet_speed);
            cboard_ptr->send(command);
        }
    }

    return 0;
}