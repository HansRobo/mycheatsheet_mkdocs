---
title: ROS Tips
category: ROS
layout: 2017/sheet
tags: [ROS]
updated: 2023-01-28
weight: -10
intro: ROS Tips
---



### `ros2 run`と一緒にパラメータを指定する

```bash
ros2 run ros_packages executable --ros-args -p <parameter_name>:=<parameter_value>
```

参考：https://docs.ros.org/en/galactic/How-To-Guides/Node-arguments.html#setting-parameters-directly-from-the-command-line




## 存在するかわからないパッケージを扱いながらament_cmake_autoを使う

```CMake
find_package(ament_cmake_auto REQUIRED)  
  
ament_auto_find_build_dependencies()  
  
find_package(<pkg_name>)  
if( <pkg_name>_FOUND )  
  add_compile_options(-D USE_<pkg_name>)  
  list(APPEND ${PROJECT_NAME}_BUILD_DEPENDS <pkg_name>)  
  list(APPEND ${PROJECT_NAME}_BUILD_EXPORT_DEPENDS <pkg_name>)  
  list(APPEND ${PROJECT_NAME}_EXEC_DEPENDS <pkg_name>)  
  list(APPEND ${PROJECT_NAME}_BUILD_DEPENDS <pkg_name>)  
  list(APPEND ${PROJECT_NAME}_FOUND_BUILD_DEPENDS <pkg_name>)  
  list(APPEND ${PROJECT_NAME}_FOUND_DEFINITIONS ${<pkg_name>_DEFINITIONS})  
  list(APPEND ${PROJECT_NAME}_FOUND_INCLUDE_DIRS ${<pkg_name>_INCLUDE_DIRS})  
  list(APPEND ${PROJECT_NAME}_FOUND_LIBRARIES ${<pkg_name>_LIBRARIES})  
endif()
```