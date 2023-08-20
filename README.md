# geometric_match_ros

- ROS node for edge based template matching.

<img src="https://user-images.githubusercontent.com/40942409/155449910-6d3edcda-0c2a-4235-8ec6-b612bd1ed6de.png" width="300">

## Subscribe Topics

- /usb_cam/image_raw(sensor_msgs/Image)

## Published Topics

- /result(geometry_msgs/Pose2D) ... result pixel (x,y) and angle(degree)

## Build

```
sudo apt install rapidjson-dev
mkdir build
cd build
cmake ..
make
```

## Binary(※現在エラー出て使えない)
  
https://github.com/hoshianaaa/geometric_match_ros/releases
  

## Environment
- Ubuntu 20.04 x86_64

## Demo Video

https://youtu.be/o8ekNBfvUyo

## Reference

https://www.codeproject.com/Articles/99457/Edge-Based-Template-Matching

## 対外発表

- ROS Japan UG #45 ROS10周年 LT大会：「形状マッチソフトの作成」 https://www.youtube.com/watch?v=dMTucT7MBys

