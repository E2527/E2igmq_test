# マルチアーキテクチャビルド定義(docker buildx bake)
# 例: docker buildx bake runtime-arm64   … WSL 上で Pi 用(arm64)イメージを作る
#     docker buildx bake runtime-pi5     … Pi 5 向けに最適化した版
variable "TAG" { default = "jazzy" }

group "default" { targets = ["dev"] }

target "_common" {
  context    = "."
  dockerfile = "docker/Dockerfile"
}

# 開発用(amd64 + arm64)
target "dev" {
  inherits  = ["_common"]
  target    = "dev"
  tags      = ["robocon/ros2:${TAG}-dev"]
  platforms = ["linux/amd64", "linux/arm64"]
}

# Pi へ配布する arm64 のビルド済みイメージ(tar で出力)。Pi 5 / Pi 400 のどちらでも動く
target "runtime-arm64" {
  inherits   = ["_common"]
  dockerfile = "docker/Dockerfile.pi"
  target     = "runtime"
  tags       = ["robocon/ros2:${TAG}"]
  platforms  = ["linux/arm64"]
  output     = ["type=docker,dest=robocon-ros2-arm64.tar"]
}

# CPU 向けに C++ を最適化した版。Pi 5 版は Pi 400 では動かないので注意
target "runtime-pi5" {
  inherits = ["runtime-arm64"]
  args     = { TARGET_CPU = "cortex-a76" }
  tags     = ["robocon/ros2:${TAG}-pi5"]
  output   = ["type=docker,dest=robocon-ros2-pi5.tar"]
}

target "runtime-pi400" {
  inherits = ["runtime-arm64"]
  args     = { TARGET_CPU = "cortex-a72" }
  tags     = ["robocon/ros2:${TAG}-pi400"]
  output   = ["type=docker,dest=robocon-ros2-pi400.tar"]
}
