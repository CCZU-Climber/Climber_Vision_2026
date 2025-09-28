#include "camera.hpp"

#include <stdexcept>

#include "hikrobot/hikrobot.hpp"
// #include "mindvision/mindvision.hpp"
#include "tools/yaml.hpp"

namespace io
{
Camera::Camera(const std::string & config_path)
{
  auto yaml = tools::load(config_path);
  auto camera_name = tools::read<std::string>(yaml, "camera_name");
  auto exposure_ms = tools::read<double>(yaml, "exposure_ms");

  if (camera_name == "hikrobot")
  {
    auto gamma = tools::read<double>(yaml, "gamma");
    auto user_id = tools::read<std::string>(yaml, "user_id");

    if (user_id.empty())
    { // 增加一个更明确的错误，指出配置缺失
      throw std::runtime_error("Config error: 'user_id' for hikrobot camera is empty!");
    }

    camera_ = std::make_unique<HikRobot>(exposure_ms, gamma, user_id);
  }

  // else if (camera_name == "hikrobot") {
  //   auto gain = tools::read<double>(yaml, "gain");
  //   auto vid_pid = tools::read<std::string>(yaml, "vid_pid");
  //   camera_ = std::make_unique<HikRobot>(exposure_ms, gain, vid_pid);
  // }

  else {
    throw std::runtime_error("Unknow camera_name: " + camera_name + "!");
  }
}

void Camera::read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp)
{
  camera_->read(img, timestamp);
}

}  // namespace io