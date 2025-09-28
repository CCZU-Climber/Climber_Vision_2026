# Climber_2026 自瞄工作空间
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
user_id: R
```