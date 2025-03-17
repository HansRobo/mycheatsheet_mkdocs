---
created: 2025-03-16 04:33:20
---

## SubscriptionOptionsのビルダ関数を追加する

### before
```c++
auto options = rclcpp::SubscriptionOptions();  
options.topic_stats_options.state = rclcpp::TopicStatisticsState::Enable;  
options.topic_stats_options.publish_topic = "topic_statistics";  
options.topic_stats_options.publish_period = std::chrono::seconds(1);
auto subscription = create_subscription<std_msgs::msg::String>(  
  "/test", 1, [](const std_msgs::msg::String msg) { std::cout << msg.data << std::endl; },  
  options);
```

### After

```c++
auto options = rclcpp::SubscriptionOptions();  
options.topic_stats_options.state = rclcpp::TopicStatisticsState::Enable;  
options.topic_stats_options.publish_topic = "topic_statistics";  
options.topic_stats_options.publish_period = std::chrono::seconds(1);
auto subscription = create_subscription<std_msgs::msg::String>(  
  "/test", 1, [](const std_msgs::msg::String msg) { std::cout << msg.data << std::endl; },  
  options);
```