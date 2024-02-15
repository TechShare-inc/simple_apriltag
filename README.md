# 概要
シンプルなapriltag検出。
Go2における検出サンプル付き

# build apriltag

```
git clone https://github.com/AprilRobotics/apriltag.git
cd apriltag
mkdir build
cd build
cmake ..
make
sudo make install
```

# usgae
Docking StationのGUI上で下記を行う：
```
mkdir build
cd build
cmake ..
make
./example_detect_gst
```