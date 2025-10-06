#include <fmt/core.h>
#include <chrono>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic> 

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
    "{r | configs/camera_r.yaml | 右相机配置文件路径 (短名-r) }"
    "{l | configs/camera_l.yaml | 左相机配置文件路径 (短名-l) }";

// 帧缓存结构
struct FrameData {
    cv::Mat img;
    cv::Mat draw_img;
    std::chrono::steady_clock::time_point timestamp;
    std::list<auto_aim::Armor> armors;
    std::list<auto_aim::Target> targets;
    bool processed = false;
};

struct CameraProcessor {
    std::array<FrameData, 2> frame_buffers;  // 双缓冲
    size_t write_index = 0;                  // 写入缓冲区索引
    std::mutex mutex;                        // 保护缓冲区访问
    std::condition_variable frame_ready;      // 通知新帧可用
    std::condition_variable frame_processed;  // 通知帧处理完成（允许采集线程写入）
    bool has_new_frame = false;              // 新帧标志
    auto_aim::YOLO yolo;
    auto_aim::Detector detector;
    auto_aim::Solver solver;
    auto_aim::Tracker tracker;
    io::Camera camera;
    
    std::atomic<size_t> display_read_index = 1;
    
    std::chrono::steady_clock::time_point last_time;
    size_t frame_count = 0;
    float current_fps = 0.0f;
    // -------------------------

    CameraProcessor(const std::string& config_path, auto_aim::Solver& solver_ref)
        : yolo(config_path, false),
          detector(config_path, false),
          solver(config_path),
          tracker(config_path, solver_ref),
          camera(config_path),
          last_time(std::chrono::steady_clock::now()) {} // 初始化时间
};


// 图像采集线程
void capture_thread(CameraProcessor& processor, const std::string& name, bool& running) {
    while(running) {
        cv::Mat img;
        std::chrono::steady_clock::time_point timestamp;
        
        processor.camera.read(img, timestamp);
        if(img.empty()) {
            continue;
        }

        {
            std::unique_lock<std::mutex> lock(processor.mutex);
            processor.frame_processed.wait(lock, [&]() {
                return !processor.has_new_frame;
            });

            auto& frame = processor.frame_buffers[processor.write_index];
            if (frame.img.empty() || frame.img.size() != img.size()) {
                 frame.img = img.clone(); 
            } else {
                img.copyTo(frame.img); 
            }

            frame.timestamp = timestamp;
            frame.processed = false;
            processor.has_new_frame = true;

            processor.write_index = (processor.write_index + 1) % 2;
        }
        
        processor.frame_ready.notify_one();
    }
}

// 相机处理线程
void process_camera(CameraProcessor& processor, const std::string& name, bool& running) {
    while(running) {
        size_t current_index;
        {
            std::unique_lock<std::mutex> lock(processor.mutex);
            processor.frame_ready.wait(lock, [&]() {
                return processor.has_new_frame;
            });
            
            current_index = (processor.write_index + 1) % 2;
        } 

        auto& frame = processor.frame_buffers[current_index];
        
        frame.armors = processor.yolo.detect(frame.img);
        frame.draw_img = frame.img.clone();
            

        for(const auto& armor : frame.armors) {
            std::vector<cv::Point> armor_point(4);
            for(int i = 0; i < 4; i++) {
                armor_point[i] = cv::Point(static_cast<int>(armor.points[i].x), 
                                         static_cast<int>(armor.points[i].y));
            }
            cv::polylines(frame.draw_img, 
                        std::vector<std::vector<cv::Point>>{armor_point}, 
                        true, cv::Scalar(0, 255, 0), 2);
            tools::draw_text(frame.draw_img, 
                           fmt::format("ID:{} conf{:.2f}", armor.name, armor.confidence), 
                           armor.center);
        }
        
        frame.targets = processor.tracker.track(frame.armors, frame.timestamp);
        
        if(!frame.targets.empty()) {
            auto target = frame.targets.front();
            tools::draw_text(frame.draw_img, 
                           fmt::format("[{}]", processor.tracker.state()), 
                           {10, 30}, {255, 255, 255});
            
            std::vector<Eigen::Vector4d> armor_xyza_list = target.armor_xyza_list();
            for(const Eigen::Vector4d& xyza : armor_xyza_list) {
                auto image_points = processor.solver.reproject_armor(
                    xyza.head(3), xyza[3], target.armor_type, target.name);
                tools::draw_points(frame.draw_img, image_points, {0, 255, 255});
            }
        }

        // --- 新增：帧率计算和绘制 ---
        processor.frame_count++;
        auto now = std::chrono::steady_clock::now();
        
        // 每 30 帧或 0.5 秒更新一次 FPS，避免频繁计算和绘制波动
        if (processor.frame_count >= 30 || 
            std::chrono::duration_cast<std::chrono::milliseconds>(now - processor.last_time).count() > 500) {
            
            auto elapsed_time = std::chrono::duration_cast<std::chrono::microseconds>(now - processor.last_time).count();
            if (elapsed_time > 0) {
                processor.current_fps = (float)processor.frame_count * 1000000.0f / elapsed_time;
            } else {
                processor.current_fps = 0.0f;
            }
            processor.last_time = now;
            processor.frame_count = 0;
        }

        // 绘制 FPS 到图像左上角 (20, 60)
        tools::draw_text(frame.draw_img, 
                         fmt::format("FPS: {:.1f}", processor.current_fps), 
                         {20, 60}, {0, 255, 0}, 2.0); // 绿色字体
        // ---------------------------------
        
        cv::resize(frame.draw_img, frame.draw_img, {}, 0.5, 0.5);
        
        { 
            std::lock_guard<std::mutex> lock(processor.mutex);
            processor.has_new_frame = false;
            frame.processed = true;
            processor.display_read_index.store(current_index, std::memory_order_release);
        }
        processor.frame_processed.notify_one();
    }
}


int main(int argc, char * argv[])
{
    cv::CommandLineParser parser(argc, argv, keys);
    auto config_path_r = parser.get<std::string>("r");
    auto config_path_l = parser.get<std::string>("l");

    tools::logger()->info("Reading config R from: {}", config_path_r);
    tools::logger()->info("Reading config L from: {}", config_path_l);

    tools::Exiter exiter;
    bool running = true;
    
    auto_aim::Solver solver_ref(config_path_r); 
    CameraProcessor right_cam(config_path_r, solver_ref);
    CameraProcessor left_cam(config_path_l, solver_ref);
    
    std::thread right_capture(capture_thread, std::ref(right_cam), "right", std::ref(running));
    std::thread left_capture(capture_thread, std::ref(left_cam), "left", std::ref(running));
    std::thread right_process(process_camera, std::ref(right_cam), "right", std::ref(running));
    std::thread left_process(process_camera, std::ref(left_cam), "left", std::ref(running));
    
    while(!exiter.exit()) {
        cv::Mat right_img, left_img;
        
        size_t read_index_r = right_cam.display_read_index.load(std::memory_order_acquire);
        size_t read_index_l = left_cam.display_read_index.load(std::memory_order_acquire);
        
        {
            std::lock_guard<std::mutex> lock_r(right_cam.mutex);
            if(right_cam.frame_buffers[read_index_r].processed) {
                right_img = right_cam.frame_buffers[read_index_r].draw_img.clone();
            }
        }
        {
            std::lock_guard<std::mutex> lock_l(left_cam.mutex);
            if(left_cam.frame_buffers[read_index_l].processed) {
                left_img = left_cam.frame_buffers[read_index_l].draw_img.clone();
            }
        }
        
        if(!right_img.empty()) cv::imshow("R", right_img);
        if(!left_img.empty()) cv::imshow("L", left_img);
        
        if(cv::waitKey(1) == 'q') break;
    }
    
    running = false;
    
    right_cam.frame_ready.notify_all();
    right_cam.frame_processed.notify_all();
    left_cam.frame_ready.notify_all();
    left_cam.frame_processed.notify_all();
    
    right_capture.join();
    left_capture.join();
    right_process.join();
    left_process.join();
    
    return 0;
}