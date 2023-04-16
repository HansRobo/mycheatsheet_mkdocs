---
title: ROS Tips
category: ROS
layout: 2017/sheet
tags: [ROS]
updated: 2023-01-28
weight: -10
intro: ROS Tips
---

## vcstool

OSRFでROS 2プロジェクトを引っ張ってきたDirk Thomas氏によって作られたメタリポジトリなどでのvcs操作をかんたんにするツール．
ただし，氏がNVIDIAに移籍してからは氏の対応が極端に遅くなり，IssueやPullRequestでは氏へ数多の”Friendly Ping”が送られているがそのほとんどは返信がない．  

ROSコミュニティの中核の一つをなすこのツールがフリーズする状況は芳しくないため，  
Steven! Ragnarök氏のフォークやvcstools2を作るものも現れている

参考：
[Status of vcstool · Issue #242 · dirk-thomas/vcstool · GitHub](https://github.com/dirk-thomas/vcstool/issues/242)  
[GitHub - MaxandreOgeret/vcstool2: Vcstool2 is an attempt at continuing development of vcstool, a command line tool designed to make working with multiple repositories easier.](https://github.com/MaxandreOgeret/vcstool2/)  

## ROS パッケージを debianパッケージ化したい

GitHub Actionが提供されている
[jspricke/ros-deb-builder-action](https://github.com/jspricke/ros-deb-builder-action)

これだけでできるらしい

```yaml
uses: jspricke/ros-deb-builder-action@main
with:
  ROS_DISTRO: rolling
  DEB_DISTRO: jammy
  GITHUB_TOKEN: ${{ secrets.GITHUB_TOKEN }}
```

MoveIt2でも使われている他，Autowareでも試用されている
[Network Dependents · jspricke/ros-deb-builder-action · GitHub](https://github.com/jspricke/ros-deb-builder-action/network/dependents)

### 公式ツール・パッケージの置き場所

複数オーガナイゼーションに別れて配置されているので忘れやすいのでメモ

- [colcon · GitHub](https://github.com/colcon)
	- colcon本体，プラグイン
- [ROS core stacks · GitHub](https://github.com/ros)
	- ROS1のものも多いが，rosdistroなどROS2でも共通で使えるものはここにあることが多い
	- [ros/rosdistro](https://github.com/ros/rosdistro)
		- rosdepの依存解決・aptへの反映のときに参照されるリポジトリ
		- 見どころ
			- 基本：各distroのフォルダの `distribution.yaml`
			- 一般的なライブラリ：`rosdep/base.yaml`
			- pipライブラリ：`rosdep/python.yaml`
- [ROS 2 · GitHub](https://github.com/ros2)
	- ROS2関連はまずここを見る
	- [ros2/ros2cli](https://github.com/ros2/ros2cli)
		- `ros2 <verb>`コマンド
	- [ros2/rclcpp](https://github.com/ros2/rclcpp)
	- サンプル系（demosのほうがサンプルが豊富）
		- [ros2/examples](https://github.com/ros2/examples)
		- [ros2/demos](https://github.com/ros2/demos)
- [ament · GitHub](https://github.com/ament)
	- [ament/ament\_cmake](https://github.com/ament/ament_cmake)
		- ament_cmake_auto
		- ament_cmake_core
	- [ament/ament\_lint](https://github.com/ament/ament_lint)
- [ros-infrastructure · GitHub](https://github.com/ros-infrastructure)
	- [ros-infrastructure/rep](https://github.com/ros-infrastructure/rep)
	- [ros-infrastructure/rosdep](https://github.com/ros-infrastructure/rosdep)
	- [ros-infrastructure/ros\_buildfarm](https://github.com/ros-infrastructure/ros_buildfarm)
- [Open Robotics · GitHub](https://github.com/osrf)
	- [osrf/docker\_images](https://github.com/osrf/docker_images)
	- [osrf/rocker](https://github.com/osrf/rocker)
- その他
	- [ROS 2 Tooling Working Group · GitHub](https://github.com/ros-tooling)
		- CIなど
		- [ros-tooling/topic\_tools](https://github.com/ros-tooling/topic_tools)
	- [ROS Planning · GitHub](https://github.com/ros-planning)
		- Navigation, MoveItなど
		- [ros-planning/navigation2](https://github.com/ros-planning/navigation2)
		- [ros-planning/moveit2](https://github.com/ros-planning/moveit2)
	- [ROS 2 release repositories · GitHub](https://github.com/ros2-gbp)
		- リリースリポジトリ置き場
	- [ros-visualization · GitHub](https://github.com/ros-visualization)
		- Rviz, Rqt関連

### `ros2 run`と一緒にパラメータを指定する

```bash
ros2 run ros_packages executable --ros-args -p <parameter_name>:=<parameter_value>
```

参考：[URL](https://docs.ros.org/en/galactic/How-To-Guides/Node-arguments.html#setting-parameters-directly-from-the-command-line)

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