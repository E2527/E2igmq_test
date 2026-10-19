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

# rosdep の依存解決を高速にするため、パッケージ定義だけ先にコピーして依存を入れる
COPY ros2_ws/src ${WS}/src
RUN apt-get update \
    && (rosdep update --rosdistro ${ROS_DISTRO} || true) \
    && rosdep install --from-paths src --ignore-src -y --rosdistro ${ROS_DISTRO} \
    && rm -rf /var/lib/apt/lists/*
