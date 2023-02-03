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

### package.xmlの<depend>に書けるconditionで使える変数

[REP 149 -- Package Manifest Format Three Specification (ROS.org)](https://www.ros.org/reps/rep-0149.html#build-depend-multiple:~:text=condition%3D%22CONDITION_EXPRESSION%22,1%22%3Eroscpp%3C/depend%3E)

基本，環境変数だけ

[rosdep/rospkg_loader.py L146](https://github.com/ros-infrastructure/rosdep/blob/master/src/rosdep2/rospkg_loader.py#L146)
contextとして環境変数をぶち込んでいる．逆にこれ以外のコンテキストは存在しない

```python
pkg.evaluate_conditions(os.environ)
```
  
[catkin\_pkg/condition.py L47](https://github.com/ros-infrastructure/catkin_pkg/blob/master/src/catkin_pkg/condition.py#L47)
`$`付き文字が抽出されて...
  
```python
identifier = pp.Word('$', pp.alphanums + '_', min=2).setName('identifier')
```
  
[catkin\_pkg/condition.py L102-103](https://github.com/ros-infrastructure/catkin_pkg/blob/master/src/catkin_pkg/condition.py#L102-L103)
コンテキストで解決される

```python
def __call__(self, context):
	return str(context.get(self.value[1:], ''))
```