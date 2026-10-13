# ROS 2 Jazzy 開発・実行環境
# ros:jazzy は amd64 / arm64 のマルチアーキテクチャイメージなので、
# WSL(x86_64)では amd64、Raspberry Pi(arm64)では arm64 が自動で選ばれる。
ARG ROS_DISTRO=jazzy
ARG BASE_IMAGE=ros:${ROS_DISTRO}

# ---- dev: 開発用(ソースはボリュームでマウントする) ----
FROM ${BASE_IMAGE} AS dev
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

# rosdep の依存解決を高速にするため、パッケージ定義だけ先にコピーして依存を入れる
WORKDIR ${WS}
COPY ros2_ws/src ${WS}/src
RUN apt-get update \
    && (rosdep update --rosdistro ${ROS_DISTRO} || true) \
    && rosdep install --from-paths src --ignore-src -y --rosdistro ${ROS_DISTRO} \
    && rm -rf /var/lib/apt/lists/*

COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh \
    && echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc \
    && echo '[ -f ${WS}/install/setup.bash ] && source ${WS}/install/setup.bash' >> /root/.bashrc

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]

# ---- runtime: ROS パッケージをビルド済みで焼き込んだイメージ(Pi へ配布する用) ----
FROM dev AS runtime
RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
    && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
