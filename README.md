# ros2_sample_pkg

ROS 2 のサンプルパッケージ。

## 概要

- talker / listener ノードを含む
- `launch/` に起動用 launch ファイルを含む

## 必要要件

- ROS 2 Humble
- colcon

## ビルド

```bash
cd /home/dev/colcon_ws
colcon build --packages-select ros2_sample_pkg
```

## 実行

### talker

```bash
source /home/dev/colcon_ws/install/setup.bash
ros2 run ros2_sample_pkg talker
```

### listener

```bash
source /home/dev/colcon_ws/install/setup.bash
ros2 run ros2_sample_pkg listener
```

### launch

```bash
source /home/dev/colcon_ws/install/setup.bash
ros2 launch ros2_sample_pkg talker_listener.launch.py
```

## ガイドライン

- 共通ガイド: `../agent_guidelines/AGENTS.md`
- リポジトリ固有の指針: `AGENTS.md`
