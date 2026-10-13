# マルチアーキテクチャビルド定義(docker buildx bake)
# 例: docker buildx bake runtime-arm64   … WSL 上で Pi 用(arm64)イメージを作る
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

# Pi へ配布する arm64 のビルド済みイメージ(tar で出力)
target "runtime-arm64" {
  inherits  = ["_common"]
  target    = "runtime"
  tags      = ["robocon/ros2:${TAG}"]
  platforms = ["linux/arm64"]
  output    = ["type=docker,dest=robocon-ros2-arm64.tar"]
}
