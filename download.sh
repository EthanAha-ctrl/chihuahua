#!/usr/bin/env zsh
set -euo pipefail

# Sources:
# https://github.com/sebastianstarke/AI4Animation/tree/master/AI4Animation/SIGGRAPH_2018
# https://github.com/sebastianstarke/AI4Animation/blob/master/AI4Animation/SIGGRAPH_2018/TensorFlow/data/Link.txt

fail() {
  print -u2 -- "FATAL: $*"
  exit 1
}

include_training=0
for argument in "$@"; do
  case "$argument" in
    --with-mann-training) include_training=1 ;;
    -h|--help)
      print -- "用法: ./download.sh [--with-mann-training]"
      print -- "先应用 AI4AnimationPy 的 NumPy 2 兼容补丁；已应用时跳过，冲突时停止。"
      print -- "默认: 下载狗动捕 MotionCapture.zip（33.4 MB）到仓库 artifacts/。"
      print -- "--with-mann-training: 同时下载 MANN Input.txt 和 Output.txt（额外约 1.76 GB）。"
      print -- "已有文件通过大小和 ZIP 完整性检查后跳过；下载或检查失败立即停止。"
      exit 0
      ;;
    *) fail "未知参数: $argument；使用 --help 查看用法。" ;;
  esac
done

for required_command in git curl unzip wc mktemp mv rm; do
  command -v "$required_command" >/dev/null || fail "缺少命令: $required_command"
done

script_dir="${0:A:h}"
destination_dir="${script_dir}/artifacts"
[[ -d "$destination_dir" ]] || fail "下载目录不存在: $destination_dir"
base_url="https://starke-consult.de/AI4Animation/SIGGRAPH_2018"
partial_file=""

cleanup() {
  if [[ -n "$partial_file" && -f "$partial_file" ]]; then
    rm -f -- "$partial_file"
  fi
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
trap 'print -u2 -- "FATAL: download.sh 执行失败。"' ZERR

apply_ai4animationpy_patch() {
  local source_dir="${script_dir}/reference/ai4animationpy"
  local patch_file="${script_dir}/src/components/d_motion_annotation_and_ik/d1_prepare_dataset/ai4animationpy_numpy2.patch"
  local forward_error reverse_error

  [[ -s "$patch_file" ]] || fail "补丁不存在或为空: $patch_file"
  if [[ ! -e "${source_dir}/.git" ]]; then
    print -- "初始化 AI4AnimationPy submodule。"
    git -C "$script_dir" submodule update --init -- reference/ai4animationpy \
      || fail "AI4AnimationPy submodule 初始化失败。"
  fi
  [[ -f "${source_dir}/setup.py" ]] || fail "AI4AnimationPy setup.py 不存在: $source_dir"

  if forward_error=$(git -C "$source_dir" apply --check "$patch_file" 2>&1); then
    git -C "$source_dir" apply "$patch_file" || fail "AI4AnimationPy 补丁应用失败。"
    print -- "已应用 AI4AnimationPy 补丁: $patch_file"
  elif reverse_error=$(git -C "$source_dir" apply --reverse --check "$patch_file" 2>&1); then
    print -- "AI4AnimationPy 补丁已应用，跳过: $patch_file"
  else
    print -u2 -- "$forward_error"
    print -u2 -- "$reverse_error"
    fail "AI4AnimationPy 补丁与当前内容冲突；未修改源码，请检查 $source_dir。"
  fi
}

validate_file() {
  local candidate_file="$1"
  local expected_bytes="$2"
  local remote_name="$3"
  local actual_bytes
  actual_bytes=$(wc -c < "$candidate_file")
  (( actual_bytes == expected_bytes )) || fail "文件大小不符: $candidate_file（期望 $expected_bytes 字节，实际 $actual_bytes 字节）"
  if [[ "$remote_name" == *.zip ]]; then
    unzip -tq "$candidate_file" || fail "ZIP 完整性检查失败: $candidate_file"
  fi
}

download_file() {
  local remote_name="$1"
  local expected_bytes="$2"
  local target_file="${destination_dir}/MANN-${remote_name}"
  local source_url="${base_url}/${remote_name}"

  if [[ -e "$target_file" ]]; then
    [[ -f "$target_file" && ! -L "$target_file" ]] || fail "目标不是普通文件: $target_file"
    validate_file "$target_file" "$expected_bytes" "$remote_name"
    print -- "已存在且检查通过，跳过: $target_file"
    return
  fi

  partial_file=$(mktemp "${target_file}.part.XXXXXX")
  print -- "下载: $source_url"
  if ! curl --fail --location --show-error --proto '=https' \
      --connect-timeout 20 --output "$partial_file" "$source_url"; then
    fail "下载失败: $source_url"
  fi
  validate_file "$partial_file" "$expected_bytes" "$remote_name"
  mv -- "$partial_file" "$target_file"
  partial_file=""
  print -- "已保存: $target_file"
}

# Author-server Content-Length values verified on 2026-11-09.
apply_ai4animationpy_patch
download_file MotionCapture.zip 33422409
if (( include_training )); then
  download_file Input.txt 999715040
  download_file Output.txt 762272294
fi
