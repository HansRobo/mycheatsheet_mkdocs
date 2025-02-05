---
created: 2024-02-23 10:24:46
---
## Default Context

それぞれのプロセスでは、シングルトンパターンによってデフォルトのContextが生成される。

rclcpp::initではこのデフォルトコンテキストを初期化している。

## オリジナル Contextを使ったROS 2プログラミング

rclcpp::initはデフォルトContextをinitしているに過ぎないので、
オリジナルContextしか使わない場合はContext::initさえしておけばrclcpp::initする必要はない

## CallbackGroup

## NodeOptions

ノードに対応する設定
-

## InitOptions

contextに対応する設定

- domain IDがセットできる

## ExecutorOptions

rclcpp::memory_strategy::MemoryStrategy::SharedPtr memory_strategy;
  rclcpp::Context::SharedPtr context;
  size_t max_conditions;
=>デフォルトコンテキストが使われる
