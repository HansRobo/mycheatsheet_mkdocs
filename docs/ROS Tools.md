---
title: ROS Tools
category: ROS
layout: 2017/sheet
tags: [ROS]
updated: 2022-08-03
weight: -10
intro: ROS Tools
---

<style type="text/css">
video {
    width: 100%;
}
</style>

### PlotJuggler

強力なプロットツール

```bash
sudo apt install ros-$ROS_DISTRO-plotjuggler
```

[facontidavide/PlotJuggler](https://github.com/facontidavide/PlotJuggler)

![PlotJuggler](https://github.com/facontidavide/PlotJuggler/raw/main/docs/plotjuggler3.gif)

### multi_data_monitor

Rviz2の上の部分にデータを表示するRviz2プラグイン

[tier4/multi_data_monitor](https://github.com/tier4/multi_data_monitor)

### rqt_embed_window

任意のウインドウをrqtで表示するプラグイン

[awesomebytes/rqt_embed_window](https://github.com/awesomebytes/rqt_embed_window)

![Screenshot of SimpleScreenRecorder, Rviz, Plotjuggler, rosbag_editor and a normal rqt_console](https://raw.githubusercontent.com/awesomebytes/rqt_embed_window/main/screenshot1.png)

### dear_ros_node_viewer

DearImGuiライブラリを使ったrqt_graphの軽量版  
[takeshi-iwanari/dear_ros_node_viewer](https://github.com/takeshi-iwanari/dear_ros_node_viewer)

<video controls autoplay muted>
  <source src="https://private-user-images.githubusercontent.com/105265012/177068238-eaf4fed9-12c0-4c5b-ac7f-9597483c4c3c.mp4?jwt=eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJpc3MiOiJnaXRodWIuY29tIiwiYXVkIjoicmF3LmdpdGh1YnVzZXJjb250ZW50LmNvbSIsImtleSI6ImtleTUiLCJleHAiOjE3MDg1ODE0MzUsIm5iZiI6MTcwODU4MTEzNSwicGF0aCI6Ii8xMDUyNjUwMTIvMTc3MDY4MjM4LWVhZjRmZWQ5LTEyYzAtNGM1Yi1hYzdmLTk1OTc0ODNjNGMzYy5tcDQ_WC1BbXotQWxnb3JpdGhtPUFXUzQtSE1BQy1TSEEyNTYmWC1BbXotQ3JlZGVudGlhbD1BS0lBVkNPRFlMU0E1M1BRSzRaQSUyRjIwMjQwMjIyJTJGdXMtZWFzdC0xJTJGczMlMkZhd3M0X3JlcXVlc3QmWC1BbXotRGF0ZT0yMDI0MDIyMlQwNTUyMTVaJlgtQW16LUV4cGlyZXM9MzAwJlgtQW16LVNpZ25hdHVyZT1lNGYwNmJmZDdkZjlhNGViNWNiZDFiOGEwNzJkZWNjYjcxNTFlNTk0NmU0OTE4N2MyYzNiOTA0MWVlZDA1Mzc5JlgtQW16LVNpZ25lZEhlYWRlcnM9aG9zdCZhY3Rvcl9pZD0wJmtleV9pZD0wJnJlcG9faWQ9MCJ9.Y3qupChLzMGF3eHuQMIwsujKHHJZaKKaX5EKLzNNI4w" type="video/mp4">
</video>

![](https://user-images.githubusercontent.com/105265012/177068238-eaf4fed9-12c0-4c5b-ac7f-9597483c4c3c.mp4)

### ament_cmake_auto

ROS2のCMakeを楽に書くためのプラグイン  
デフォルトでインストールされている  
詳細は[ブログ](https://hans-robo.hatenablog.com/entry/2020/12/15/153503)を参照

[amnet_cmake_auto](https://github.com/ament/ament_cmake/tree/rolling/ament_cmake_auto)

```CMake
cmake_minimum_required(VERSION 3.5)
project(example)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED
 src/example.cpp include/example.hpp)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
```

### ros_system_fingerprint

ROSのためのシステム状態を一括で出力するコマンド

```bash
ros2 run system_fingerprint imprint
```
<https://github.com/suurjaak/grepros>
[MetroRobots/ros_system_fingerprint](https://github.com/MetroRobots/ros_system_fingerprint/tree/ros2)

以下のものが出力に含まれる

- システム情報（カーネルバージョンなど）
- 環境変数
- 起動しているノード情報
- パラメータ情報
- サービス・アクション・トピック情報
- ROSワークスペース情報

出力例は[こちら](https://github.com/MetroRobots/ros_system_fingerprint/blob/ros2/example_fingerprint.yaml)

### grepros

ROS1/2のbagファイルやtopicをgrepするためのツール

```bash
sudo apt install ros-$ROS_DISTRO-grepros
```
<https://github.com/suurjaak/grepros>

メッセージの型やトピック名，frame_idでフィルタリングしたり…

```bash
grepros frame_id=map --type geometry_msgs/* --live
grepros --topic *lidar* --max-per-topic 1 --live
```

出力結果を新しいbagファイルに書き込んだり…

```bash
grepros --max-per-topic 1 --lines-per-message 30 --live --no-console-output --write my.bag
```

現在のフォルダ以下にあるbagファイルの中で文字列検索が出来たり…

```bash
grepros -r "keyword"
```

topic内容の高度な選択と出力ができる！

```bash
# "name", "message"フィールドのどちらかに"navigation"の文字列を含むtopic frameのheader.stampのみを出力する
grepros --type diagnostic_msgs/* --select-field name message \
        --print-field header.stamp -- navigation
```

その他の使用例は[こちら](https://suurjaak.github.io/grepros/usage.html)
