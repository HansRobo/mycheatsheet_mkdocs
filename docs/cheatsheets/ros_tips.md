---
title: ROS Packages
category: ROS
layout: 2017/sheet
tags: [Featured]
updated: 2022-08-03
weight: -10
intro: ros_packages
---



### `ros2 run`と一緒にパラメータを指定する

```bash
ros2 run ros_packages executable --ros-args -p <parameter_name>:=<parameter_value>
```

参考：https://docs.ros.org/en/galactic/How-To-Guides/Node-arguments.html#setting-parameters-directly-from-the-command-line




### cmakeの関数の引数
### 基礎
- 名前をつけた引数は`${引数名}`の形で関数内で参照可能
- ↑からはみ出た引数は`${ARGN}`にセミコロン区切りで詰め込まれる
```cmake
function(関数名 引数名1 引数名2)
	message("ARG1 = ${引数名1}")
	message("ARG2 = ${引数名2}")
	message("ARGN = ${ARGN}") 
endfunction()
```
#### パース

```cmake　　　　　　　　　　
cmake_parse_arguments(
	MYFUNC 　　　       # 変数のプレフィックス
	"BOOL_VAR"         # 真偽値をとる変数の名前
	"VAR1;VAR2"        # 値を1つだけとる変数の名前
	"MULTIPLE_OUTPUT"  # 値を複数とる変数の名前
	${ARGN}　　　　　　　# パース元
)
```
- 変数はセミコロン区切りで複数指定可能
- CMakeには名前空間がないのでプレフィックスをつけることで名前の衝突を回避
	- 今回は`MYFUNC_VAR1`とかになる
- パースされなかったものは`PREFIX_UNPARSED_ARGUMENTS`に詰められる

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