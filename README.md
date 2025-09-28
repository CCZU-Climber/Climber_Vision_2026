# Climber_2026 自瞄工作空间

## 前言
深度参考同济25开源，基于***工具层:tools***，修改***硬件层：io、功能层：tasks***,重写***应用层***
链接如下：
```bash
https://github.com/TongjiSuperPower/sp_vision_25.git
```

## 1. 编译
```bash
cmake -B build
make -C build/ -j`nproc`
```
## 2. 运行识别测试程序
```bash
./build/auto_aim_test configs/test.yaml
```

## 3. 相机调用
修改了相机调用逻辑，通过相机名访问设备
需在yaml文件里改变
```yaml
user_id: your_camera_name
```