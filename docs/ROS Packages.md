---
title: ROS便利パッケージ集
category: ROS
layout: 2017/sheet
tags: [ROS]
updated: 2022-08-03
weight: -10
intro: ROS便利パッケージ集
---



### ros2_logging_fmt

[facontidavide/ros2_logging_fmt](https://github.com/facontidavide/ros2_logging_fmt)

fmtlibを使ってログを出力する軽量かつシンプルなロギングツール

```c++
ros2_logging_fmt::Logger logger(node.get_logger());

logger.info("Hello {} number {}", world, 42);
logger.error("We have {} errors", 99);
logger.warn("Warning: {} > {}", 30.1, 30.0);
logger.debug("DEBUG MESSAGE");
```

### grid_map

グリッドマップを扱う強力なライブラリ  
[ANYbotics/grid_map](https://github.com/ANYbotics/grid_map)

ROS2対応はゆっくりなので、TIER IVフォークの方が良い  
[tier4/grid_map/tree/prepare/humble](https://github.com/tier4/grid_map/tree/prepare/humble)

### generate_parameter_library

ROS2のパラメータをライブラリに固めて配布したりできるライブラリ

[PickNikRobotics/generate_parameter_library](https://github.com/PickNikRobotics/generate_parameter_library)

### rosros

ROS1/2の統一的Pythonインタフェースを提供するパッケージ
<https://github.com/suurjaak/rosros>

環境変数からROSバージョンを自動切り替えできる！
以下のようにpub/subはもちろんのこと、serviceやparameterなどにも対応する

<!-- cspell:ignore rosros -->

```python
import rosros

def on_trigger(req):
    pub.publish(True)
    return {"success": True, "message": "Triggered!"}

rosros.init_node("my_node")
params = rosros.init_params(service="/trigger", topic="/triggering")
srv = rosros.create_service(params["service"], "std_srvs/Trigger", on_trigger)
pub = rosros.create_publisher(params["topic"], "std_msgs/Bool", latch=True, queue_size=2)
rosros.spin()
```

### realtime_tools

[GitHub - ros-controls/realtime\_tools](https://github.com/ros-controls/realtime_tools)

色々ツールがあるが、実装を見て何がリアルタイムになって何がリアルタイムにならないか把握し上で使ったほうが良さそう。
たとえば、RealTimePublisherはpublish関数にブロックされる時間が短くなるだけで通信がリアルタイムになったりする効果はない
