---
created: 2023-01-20 10:14:27
title: ROS News
category: ROS
layout: 2017/sheet
tags: [ROS]
updated: 2023-01-20
weight: -10
intro: ROS News
---

## `qos_event`から`event_handler`への以降

### `rclcpp::spin_until_timeout`関数の追加

テストを書く時に便利に使えそう  

```c++
void spin_until_timeout(rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_ptr,
                        std::chrono::duration< TimeRepT, TimeT > timeout)
```

- [提案Issue](https://github.com/ros2/rclcpp/issues/1821)
- [PullRequest #1874](https://github.com/ros2/rclcpp/pull/1874)
  - 2022/01/24に`rolling`にマージ
- [デグレ発覚によりリバート #1956](https://github.com/ros2/rclcpp/pull/1956)
  - 2022/01/25
- [現在再マージに向けて調整中 #1957](https://github.com/ros2/rclcpp/pull/1957)

### `ros2 interface package` にフィルターのオプション追加

[Add interface type filters to ros2 interface package by DLu · Pull Request #765 · ros2/ros2cli · GitHub](https://github.com/ros2/ros2cli/pull/765)

以下のようにズラッとでてくるのをフィルターするため以下のオプションが追加される

- `--only-msgs`
- `--only-srvs`
- `--only-actions`
-

```shell
hans@hans$ ros2 interface package std_msgs
std_msgs/msg/Char
std_msgs/msg/Float32MultiArray
std_msgs/msg/Int64MultiArray
...
```

### ROS2でのNodeHandle

プルリク：[Pull Request #2041 · ros2/rclcpp · GitHub](https://github.com/ros2/rclcpp/pull/2041)  
議論：[Issue #831 · ros2/rclcpp · GitHub](https://github.com/ros2/rclcpp/issues/831#issue-486100658)

ROS1時代に逆行しようと言うのではなく，使いにくかったインタフェースを直すという意味

Nodeクラスのインタフェースは適切に分離されて提供されている
しかし，複数の機能を必要とする関数などにはやや冗長な以下のような書き方を強要していた

```c++
create_service(
  std::shared_ptr<node_interfaces::NodeBaseInterface> node_base,
  std::shared_ptr<node_interfaces::NodeServicesInterface> node_services,

auto service = create_service(
  my_node_class->get_node_base_interface(),
  my_node_class->get_node_services_interface(),
  // ...
```

そこで，コンセプトの考え方を取り入れて適切に権限管理されたNodeHandleを使ってこれを以下のように簡素化しようというのが今回のプルリク

```c++
create_service(
     rclcpp::NodeHandle<Base, Services> node_handle,
     // ...

// User calls with
auto service = create_service(
  my_node_class,
  // ...
```

### `ros2 topic echo`での `--timeout <sec>`オプション

[Adds a timeout feature to rostopic echo by arjo129 · Pull Request #792 · ros2/ros2cli · GitHub](https://github.com/ros2/ros2cli/pull/792)
