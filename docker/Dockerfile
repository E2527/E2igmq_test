# ROS 2 Jazzy 開発・実行環境
# ros:jazzy は amd64 / arm64 のマルチアーキテクチャイメージなので、
# WSL(x86_64)では amd64、Raspberry Pi(arm64)では arm64 が自動で選ばれる。
#
#   base … ビルドに必要なツールだけ(GUI なし)
#   dev  … base + rviz2 などの GUI ツール + STM32 用のコンパイラ。PC(WSL)での開発用
#
# Raspberry Pi に送る実行用イメージは docker/Dockerfile.pi で作る。
ARG ROS_DISTRO=jazzy
ARG BASE_IMAGE=ros:${ROS_DISTRO}

# ---- base: 土台(GUI なし) ----
FROM ${BASE_IMAGE} AS base
ARG ROS_DISTRO
ENV ROS_DISTRO=${ROS_DISTRO} \
    DEBIAN_FRONTEND=noninteractive \
    WS=/root/ros2_ws

RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-dev-tools \
        build-essential \
        git \
        python3-pip \
        bash-completion \
    && rm -rf /var/lib/apt/lists/*

WORKDIR ${WS}

COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh \
    && echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc \
    && echo '[ -f ${WS}/install/setup.bash ] && source ${WS}/install/setup.bash' >> /root/.bashrc

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]

# ---- dev: 開発用(ソースはボリュームでマウントする) ----
FROM base AS dev
# GUI・デモ用と、STM32 のビルド用(firmware/motor_node_f303k8)。
# ソースより先に入れて、src を変えても再インストールされないようにする
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-${ROS_DISTRO}-rviz2 \
        ros-${ROS_DISTRO}-demo-nodes-cpp \
        ros-${ROS_DISTRO}-demo-nodes-py \
        gcc-arm-none-eabi \
        libnewlib-arm-none-eabi \
    && rm -rf /var/lib/apt/lists/*

# MoveIt 2(アームの動作計画)。大きい(1 GB 以上)ので別の層にして、上の層を変えても入れ直さないようにする
#   moveit-resources-panda-moveit-config … 動作確認用のデモ(ros2 launch moveit_resources_panda_moveit_config demo.launch.py)
#   ros2-controllers                       … デモや自作アームで、関節を動かすコントローラー
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-${ROS_DISTRO}-moveit \
        ros-${ROS_DISTRO}-moveit-resources-panda-moveit-config \
        ros-${ROS_DISTRO}-ros2-controllers \
    && rm -rf /var/lib/apt/lists/*

# Nav2(自律移動)と SLAM(地図づくり)。MoveIt と同じく大きいので別の層にする
#   nav2-bringup        … 起動ファイルと、Gazebo の TurtleBot3 で試すシミュレーション
#                         (ros2 launch nav2_bringup tb3_simulation_launch.py)
#   slam-toolbox        … LiDAR で走りながら地図を作る
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-${ROS_DISTRO}-navigation2 \
        ros-${ROS_DISTRO}-nav2-bringup \
        ros-${ROS_DISTRO}-nav2-minimal-tb3-sim \
        ros-${ROS_DISTRO}-slam-toolbox \
    && rm -rf /var/lib/apt/lists/*

# rosdep の依存解決を高速にするため、パッケージ定義だけ先にコピーして依存を入れる
COPY ros2_ws/src ${WS}/src
RUN apt-get update \
    && (rosdep update --rosdistro ${ROS_DISTRO} || true) \
    && rosdep install --from-paths src --ignore-src -y --rosdistro ${ROS_DISTRO} \
    && rm -rf /var/lib/apt/lists/*
