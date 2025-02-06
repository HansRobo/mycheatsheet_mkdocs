---
title: colcon
category: ROS
layout: 2017/sheet
tags: [ROS]
updated: 2022-06-16
weight: -10
intro: colcon
---



### colconのよく使うサブコマンド

| サブコマンド        | 説明                |
|---------------|-------------------|
| `build`       | ビルドする             |
| `test`        | テストを回す            |
| `test-result` | テスト結果を表示          |
| `graph`       | パッケージ間の依存関係グラフを表示 |

### colcon buildのよく使うオプション

| オプション                         | 説明                           |
|-------------------------------|------------------------------|
| `--symlink-install`           | ファイルのインストールをコピーではなくシンボリックで行う |
| `--packages-select <package>` | 指定したパッケージのみビルド               |
| `--packages-up-to <package>`  | 指定したパッケージとその依存パッケージをビルド      |
| `--cmake-args <CMakeに渡す引数>`   | CMakeに渡す引数を指定する              |
| `--parallel-workers <number>` | 最大の並列数を指定する                  |
| `--cmake-clean-first`         | クリーンビルドする                    |

### colcon-ros-buildfarm

```bash
mkdir -p colcon_buildfarm_ws/src
cd colcon_buildfarm_ws
wget https://raw.githubusercontent.com/HansRobo/colcon-ros-buildfarm/refs/heads/devel/crb.repos
source /opt/ros/humble/setup.bash
vcs import src < crb.repos
colcon build
source install/setup.bash
ros_buildfarm release
```

#### 注意

- symlink-installするとビルドが通らない？
- 現状、ros_buildfarm releaseしてもうまく動かないっぽい？
