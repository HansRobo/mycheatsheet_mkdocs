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

### 概要
OSRFでROS 2プロジェクトを引っ張ってきたDirk Thomas氏によって作られたメタリポジトリなどでのvcs操作をかんたんにするツール。

### 現在の状況
- 開発者のNVIDIA移籍後、対応が極端に遅くなっている
- IssueやPullRequestで数多の"Friendly Ping"が送られているが返信なし
- ROSコミュニティの中核ツールがフリーズしている状況

### 代替手段
- Steven! Ragnarök氏のフォーク
- [vcstool2](https://github.com/MaxandreOgeret/vcstool2/) - 新しい実装

### 参考リンク
- [Status of vcstool · Issue #242](https://github.com/dirk-thomas/vcstool/issues/242)
- [GitHub - MaxandreOgeret/vcstool2](https://github.com/MaxandreOgeret/vcstool2/)

## ROSパッケージのDebianパッケージ化

### GitHub Actionを使用した自動化
[jspricke/ros-deb-builder-action](https://github.com/jspricke/ros-deb-builder-action) が提供されている。

### 基本的な使用法

```yaml
uses: jspricke/ros-deb-builder-action@main
with:
  ROS_DISTRO: rolling
  DEB_DISTRO: jammy
  GITHUB_TOKEN: ${{ secrets.GITHUB_TOKEN }}
```

### 採用事例
- **MoveIt2**: 本格採用
- **Autoware**: 試用中
- その他の依存プロジェクト: [Network Dependents](https://github.com/jspricke/ros-deb-builder-action/network/dependents)

## 公式ツール・パッケージの置き場所

複数オーガナイゼーションに別れて配置されているため忘れやすいのでメモ

### 主要オーガナイゼーション

#### [colcon · GitHub](https://github.com/colcon)
- colcon本体・プラグイン

#### [ROS core stacks · GitHub](https://github.com/ros)
- ROS1のものも多いが、rosdistroなどROS2でも共通で使えるものはここにあることが多い
- **重要リポジトリ**:
  - [ros/rosdistro](https://github.com/ros/rosdistro) - rosdepの依存解決・aptへの反映時に参照
    - 各distroのフォルダの `distribution.yaml`
    - 一般的なライブラリ：`rosdep/base.yaml`
    - pipライブラリ：`rosdep/python.yaml`

#### [ROS 2 · GitHub](https://github.com/ros2)
- ROS2関連はまずここを見る
- **主要リポジトリ**:
  - [ros2/ros2cli](https://github.com/ros2/ros2cli) - `ros2 <verb>`コマンド
  - [ros2/rclcpp](https://github.com/ros2/rclcpp)
  - [ros2/examples](https://github.com/ros2/examples) - サンプル
  - [ros2/demos](https://github.com/ros2/demos) - サンプル豊富

#### [ament · GitHub](https://github.com/ament)
- [ament/ament_cmake](https://github.com/ament/ament_cmake)
  - ament_cmake_auto
  - ament_cmake_core
- [ament/ament_lint](https://github.com/ament/ament_lint)

#### [ros-infrastructure · GitHub](https://github.com/ros-infrastructure)
- [ros-infrastructure/rep](https://github.com/ros-infrastructure/rep)
- [ros-infrastructure/rosdep](https://github.com/ros-infrastructure/rosdep)
- [ros-infrastructure/ros_buildfarm](https://github.com/ros-infrastructure/ros_buildfarm)

#### [Open Robotics · GitHub](https://github.com/osrf)
- [osrf/docker_images](https://github.com/osrf/docker_images)
- [osrf/rocker](https://github.com/osrf/rocker)

### 特殊用途オーガナイゼーション

#### [ROS 2 Tooling Working Group · GitHub](https://github.com/ros-tooling)
- CIなど
- [ros-tooling/topic_tools](https://github.com/ros-tooling/topic_tools)

#### [ROS Planning · GitHub](https://github.com/ros-planning)
- Navigation・MoveItなど
- [ros-planning/navigation2](https://github.com/ros-planning/navigation2)
- [ros-planning/moveit2](https://github.com/ros-planning/moveit2)

#### [ROS 2 release repositories · GitHub](https://github.com/ros2-gbp)
- リリースリポジトリ置き場

#### [ros-visualization · GitHub](https://github.com/ros-visualization)
- Rviz・Rqt関連

## コマンドライン操作

### `ros2 run`と一緒にパラメータを指定する

```bash
ros2 run ros_packages executable --ros-args -p <parameter_name>:=<parameter_value>
```

**参考**: [Node arguments documentation](https://docs.ros.org/en/galactic/How-To-Guides/Node-arguments.html#setting-parameters-directly-from-the-command-line)

## 開発テクニック

### 存在するかわからないパッケージを扱いながらament_cmake_autoを使う

```cmake
find_package(ament_cmake_auto REQUIRED)

ament_auto_find_build_dependencies()

find_package(<pkg_name>)
if(<pkg_name>_FOUND)
  add_compile_options(-D USE_<pkg_name>)
  list(APPEND ${PROJECT_NAME}_BUILD_DEPENDS <pkg_name>)
  list(APPEND ${PROJECT_NAME}_BUILD_EXPORT_DEPENDS <pkg_name>)
  list(APPEND ${PROJECT_NAME}_EXEC_DEPENDS <pkg_name>)
  list(APPEND ${PROJECT_NAME}_FOUND_BUILD_DEPENDS <pkg_name>)
  list(APPEND ${PROJECT_NAME}_FOUND_DEFINITIONS ${<pkg_name>_DEFINITIONS})
  list(APPEND ${PROJECT_NAME}_FOUND_INCLUDE_DIRS ${<pkg_name>_INCLUDE_DIRS})
  list(APPEND ${PROJECT_NAME}_FOUND_LIBRARIES ${<pkg_name>_LIBRARIES})
endif()
```

### package.xmlの`<depend/>`に書けるconditionで使える変数

[REP 149 - Package Manifest Format Three Specification](https://www.ros.org/reps/rep-0149.html#build-depend-multiple:~:text=condition%3D%22CONDITION_EXPRESSION%22,1%22%3Eroscpp%3C/depend%3E)

**基本的に環境変数のみ使用可能**

#### 実装詳細
[rosdep/rospkg_loader.py L146](https://github.com/ros-infrastructure/rosdep/blob/master/src/rosdep2/rospkg_loader.py#L146) - contextとして環境変数をぶち込んでいる。これ以外のコンテキストは存在しない。

```python
pkg.evaluate_conditions(os.environ)
```

[catkin_pkg/condition.py L47](https://github.com/ros-infrastructure/catkin_pkg/blob/master/src/catkin_pkg/condition.py#L47) - `$`付き文字が抽出される

```python
identifier = pp.Word('$', pp.alphanums + '_', min=2).setName('identifier')
```

[catkin_pkg/condition.py L102-103](https://github.com/ros-infrastructure/catkin_pkg/blob/master/src/catkin_pkg/condition.py#L102-L103) - コンテキストで解決される

```python
def __call__(self, context):
    return str(context.get(self.value[1:], ''))
```

## MPPIアルゴリズム詳解

### Critic一覧

#### Constraint Critic
- **最大速度制約**: 最大速度を超えている部分をペナルティに追加
- **最低速度制約**: 最低速度を下回っている部分をペナルティに追加

#### Cost Critic
- **衝突検出**: 衝突計算をして衝突したら大きなペナルティを追加
- **経路コスト**: waypointごとのコストの平均

#### Goal Angle Critic
- **角度制約**: ゴールの近くに来たとき角度が合っていないと低評価
- **評価方法**: ゴール近くで、ゴール角度と各Waypointの角度の差の絶対値平均を取って、重み係数をかけてべき乗係数分べき乗

#### Goal Critic
- **距離評価**: ゴール付近に来たとき、各Waypointとゴール座標の差分の平均をコストとする

#### Obstacle Critic
- **障害物回避**: waypointごとに障害物やインフレーション層からコストを計算
- **正規化**: コストを全部足し合わせて長さで割って正規化

#### Path Align Critic
- **適用条件**: ゴール付近では使わない（代わりにGoal Criticが使われる）
- **動的障害物対応**: 動的障害物がパスとかなり重なっているときはコストを足さない
- **評価方法**: それぞれのwaypointについての参照パスの最近傍点との距離の平均をコストとする

#### Path Angle Critic
（詳細記載なし）

## Topic統計機能
（詳細記載なし）