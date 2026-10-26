#!/bin/bash
# 删除 colcon 构建产物（build/ install/ log/），删除前会确认。
set -e
WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TARGETS=(build install log)

echo "工作空间：$WS_DIR"
echo "将删除："
for t in "${TARGETS[@]}"; do
  if [ -e "$WS_DIR/$t" ]; then
    printf '  %-8s %s\n' "$t" "$(du -sh "$WS_DIR/$t" 2>/dev/null | cut -f1)"
  else
    printf '  %-8s (不存在)\n' "$t"
  fi
done

printf '确认删除？[y/N] '
read -r answer
case "$answer" in
  y|Y|yes|YES) ;;
  *) echo "已取消，什么都没删"; exit 0 ;;
esac

for t in "${TARGETS[@]}"; do
  if [ -e "$WS_DIR/$t" ]; then
    rm -rf "${WS_DIR:?}/${t:?}"
    echo "  已删除 $t"
  fi
done
echo "完成。重新构建：./build.sh"
