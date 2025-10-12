# Climber_2026 自瞄工作空间

## 前言
分为四层：src、tasks、io、tools   
参考学习同济25开源   
主要作为上场保底代码和26赛季的学习平台  

## 1. 安装依赖
[OPENVIO]<https://docs.openvino.ai/2025/get-started/install-openvino/install-openvino-archive-linux.html>   \
[Cere]<http://ceres-solver.org/installation.html>   \
**24.04中cere要把第三方库也clone全**
```bash
git clone --recurse-submodules https://ceres-solver.googlesource.com/ceres-solver
```

其他：

```shell
sudo apt install -y \
    git \
    g++ \
    cmake \
    can-utils \
    libopencv-dev \
    libfmt-dev \
    libeigen3-dev \
    libspdlog-dev \
    libyaml-cpp-dev \
    libusb-1.0-0-dev \
    nlohmann-json3-dev \
    openssh-server \
    screen
```


## 2. 编译
```bash
cmake -B build
make -C build/ -j`nproc`
```
## 3. 运行识别测试程序
```bash
./build/auto_aim_test configs/test.yaml
```

## 4. 相机调用
修改了相机调用逻辑，通过相机名访问设备   
需在yaml文件里改变  
```yaml
user_id: your_camera_name
```
## 参考
深度参考同济25开源，基于**工具层:tools**，修改**硬件层：io、功能层：tasks**,重写**应用层：src**   
链接如下：
<https://github.com/TongjiSuperPower/sp_vision_25.git>

