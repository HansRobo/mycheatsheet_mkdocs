---
title: ament_cmake_auto
category: Tool, ROS
layout: 2017/sheet
tags: [ROS, Tools]
updated: 2023-01-28
created: 2023-01-28 04:04:29
weight: -10
intro: ament_cmake_auto
---

## ament_auto_add_library

- 引数  
	- 【bool】NO_TARGET_LINK_LIBRARIES  
	- 【bool】EXCLUDE_FROM_ALL  
	- 【bool】MODULE  
	- 【bool】SHARED  
	- 【bool】STATIC  
	- DIRECTORY  
		- ここにあるソースファイルはすべてビルド対象に  
	- target
- やること
	- add_library
		- DIRECTORYで指定したフォルダのソースファイル全部
	- target_include_directories
		- includeフォルダ
	- target_link_libraries
		- プロジェクトで作ったライブラリ全部
	- ament_target_dependencies
		- プロジェクトの依存関係にあるやつ全部


### ament_auto_package
- 引数
	- 【bool】INSTALL_TO_PATH
	- 【複数指定可】INSTALL_TO_SHARE
		- launchフォルダとかを指定すると追加でインストールできる
- やること
	- ament_export_dependencies
	- ament_export_include_directories
	- ament_export_libraries
		- ament_auto_add_libraryを使っていないライブラリはexportされないので注意
	- install
		- include/*
			- include
		- ライブラリ
			- lib / bin
		- 実行ファイル
			- lib
		- その他
			- `INSTALL_TO_SHARE`で指定したもの
				- share
	- ament_package
		- 使わなかった引数はこちらに渡される

## ament_auto_find_build_dependencies

```CMakeLists.txt
ament_auto_find_build_dependencies(
	REQUIRED geometry_msgs
)
```

ament_package_xml()で `build/<pkg>/ament_cmake_core/package.cmake`にざっと情報が抜き出される
[ament_cmake/ament_package_xml.cmake at humble · ament/ament_cmake · GitHub](https://github.com/ament/ament_cmake/blob/humble/ament_cmake_core/cmake/core/ament_package_xml.cmake)
```package.cmake
set(_AMENT_PACKAGE_NAME "traffic_simulator")
set(traffic_simulator_VERSION "0.6.7")
set(traffic_simulator_MAINTAINER "masaya kataoka <masaya.kataoka@tier4.jp>")
set(traffic_simulator_BUILD_DEPENDS "ament_index_cpp" "concealer")
set(traffic_simulator_BUILDTOOL_DEPENDS "ament_cmake" "ament_cmake_auto")
set(traffic_simulator_BUILD_EXPORT_DEPENDS "ament_index_cpp" "concealer")
set(traffic_simulator_BUILDTOOL_EXPORT_DEPENDS )
set(traffic_simulator_EXEC_DEPENDS "ament_index_cpp" "concealer")
set(traffic_simulator_TEST_DEPENDS "ament_cmake_gtest" "ament_lint_auto" "ament_cmake_clang_format" "ament_cmake_copyright" "ament_cmake_lint_cmake" "ament_cmake_pep257" "ament_cmake_xmllint")
set(traffic_simulator_GROUP_DEPENDS )
set(traffic_simulator_MEMBER_OF_GROUPS )
set(traffic_simulator_DEPRECATED "")
set(traffic_simulator_EXPORT_TAGS)
list(APPEND traffic_simulator_EXPORT_TAGS "<build_type>ament_cmake</build_type>")
```

ここで処理したくない依存を合流させるには以下のような記述が必要
```CMakeLists.txt
find_package(autoware_adapi_v1_msgs)  
if( autoware_adapi_v1_msgs_FOUND )  
  add_compile_options(-D USE_ADAPI_V1_MSGS)  
  list(APPEND ${PROJECT_NAME}_BUILD_DEPENDS autoware_adapi_v1_msgs)  
  list(APPEND ${PROJECT_NAME}_BUILD_EXPORT_DEPENDS autoware_adapi_v1_msgs)  
  list(APPEND ${PROJECT_NAME}_EXEC_DEPENDS autoware_adapi_v1_msgs)  
  list(APPEND ${PROJECT_NAME}_BUILD_DEPENDS autoware_adapi_v1_msgs)  
  list(APPEND ${PROJECT_NAME}_FOUND_BUILD_DEPENDS autoware_adapi_v1_msgs)  
  list(APPEND ${PROJECT_NAME}_FOUND_DEFINITIONS ${autoware_adapi_v1_msgs_DEFINITIONS})  
  list(APPEND ${PROJECT_NAME}_FOUND_INCLUDE_DIRS ${autoware_adapi_v1_msgs_INCLUDE_DIRS})  
  list(APPEND ${PROJECT_NAME}_FOUND_LIBRARIES ${autoware_adapi_v1_msgs_LIBRARIES})  
endif()
```