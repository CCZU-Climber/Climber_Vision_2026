#ifndef IO__CBOARD_HPP
#define IO__CBOARD_HPP

#include <Eigen/Geometry>
#include <chrono>
#include <cmath>
#include <functional>
#include <string>
#include <vector>
#include <thread>
#include <atomic>
#include <iostream>

#include "io/command.hpp"
#include "serial/serial.h"
#include "tools/logger.hpp"
#include "tools/thread_safe_queue.hpp"

namespace io
{
enum Mode
{
  idle,
  auto_aim,
  small_buff,
  big_buff,
  outpost
};
const std::vector<std::string> MODES = {"idle", "auto_aim", "small_buff", "big_buff", "outpost"};

// // 哨兵专有
// enum ShootMode
// {
//   left_shoot,
//   right_shoot,
//   both_shoot
// };
// const std::vector<std::string> SHOOT_MODES = {"left_shoot", "right_shoot", "both_shoot"};

// --- New Serial Packet Definitions ---

// 接收：下位机 -> Vision (IMU, Speed, Mode)
struct __attribute__((packed)) BoardToVision
{
  uint8_t head[2] = {'B', 'V'}; // 帧头
  uint8_t mode;                 // 对应原 frame.data[2]
  // uint8_t shoot_mode;           // 对应原 frame.data[3]
  float bullet_speed;           // 对应原 frame.data[0] | frame.data[1] / 1e2
  // float ft_angle;               // 对应原 frame.data[4] | frame.data[5] / 1e4
  float q[4];                   // wxyz顺序，对应原四元数数据
  uint16_t crc16;
};
static_assert(sizeof(BoardToVision) <= 64);

// 发送：Vision -> 下位机 (Command)
// 包含原CAN通信中 send_canid_ 的内容
struct __attribute__((packed)) VisionToBoard
{
  uint8_t head[2] = {'V', 'B'}; // 帧头
  uint8_t control_and_shoot;    // Bit 0: control, Bit 1: shoot
  float yaw;          // 单位弧度
  float pitch;
  float horizon_distance;
  uint16_t crc16;
};
static_assert(sizeof(VisionToBoard) <= 64);


class CBoard
{
public:
  double bullet_speed;
  Mode mode;
  // ShootMode shoot_mode;
  // double ft_angle;  //无人机专有

  CBoard(const std::string & config_path);
  ~CBoard();

  Eigen::Quaterniond imu_at(std::chrono::steady_clock::time_point timestamp);

  void send(Command command);


private:
  struct IMUData
  {
    Eigen::Quaterniond q;
    std::chrono::steady_clock::time_point timestamp;
  };

  tools::ThreadSafeQueue<IMUData> queue_{5000}; 
  serial::Serial serial_;

  std::thread thread_;
  std::atomic<bool> quit_=false;

  IMUData data_ahead_;
  IMUData data_behind_;

  BoardToVision rx_data_;
  VisionToBoard tx_data_;

  void read_thread();
  bool read(uint8_t *buffer, size_t size);
  void reconnect();
};

}  // namespace io

#endif  // IO__CBOARD_HPP