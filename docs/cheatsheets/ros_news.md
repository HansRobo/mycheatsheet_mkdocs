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

### ROS2でのNodeHandle

プルリク：[Implement Unified Node Interface (NodeInterfaces class) by methylDragon · Pull Request #2041 · ros2/rclcpp · GitHub](https://github.com/ros2/rclcpp/pull/2041)
議論：[Proposal for NodeHandle concept that simplifies calling APIs with node-like objects · Issue #831 · ros2/rclcpp · GitHub](https://github.com/ros2/rclcpp/issues/831#issue-486100658)

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

