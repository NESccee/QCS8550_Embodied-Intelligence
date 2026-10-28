#!/usr/bin/env bash
# ==============================================================================
# 高通具身智能大赛 QCS8550 关键程序 GitHub 上传辅助脚本
# 目标仓库: https://github.com/NESccee/QCS8550_Embodied-Intelligence (main 分支)
# ==============================================================================

set -e
cd "$(dirname "$0")"

REMOTE_REPO="NESccee/QCS8550_Embodied-Intelligence"
BRANCH="main"

echo "======================================================================"
echo "  高通具身智能大赛 (QCS8550) 核心程序代码同步脚本"
echo "  目标仓库: https://github.com/${REMOTE_REPO}"
echo "  目标分支: ${BRANCH}"
echo "======================================================================"

# 1. 优先尝试 SSH 认证推送
echo -n ">>> 正在检测 SSH 认证状态 (ssh.github.com:443)... "
if ssh -o BatchMode=yes -o ConnectTimeout=5 -T git@github.com 2>&1 | grep -q -i "successfully authenticated"; then
    echo "[OK] SSH 认证成功！"
    echo ">>> 正在执行 git push (通过 SSH)..."
    git push -u git@github.com:${REMOTE_REPO}.git ${BRANCH}
    echo ">>> [SUCCESS] 关键程序已成功上传至 GitHub ${REMOTE_REPO}:${BRANCH}！"
    exit 0
else
    echo "[未配置 SSH 密钥或权限不足]"
fi

# 2. 如果传入了 Personal Access Token (PAT) 参数
TOKEN="$1"
if [ -n "$TOKEN" ]; then
    echo ">>> 检测到提供的 GitHub Token，正在执行 HTTPS 推送..."
    git push -u "https://${TOKEN}@github.com/${REMOTE_REPO}.git" ${BRANCH}
    echo ">>> [SUCCESS] 关键程序已成功上传至 GitHub ${REMOTE_REPO}:${BRANCH}！"
    exit 0
fi

# 3. 如果未配置 SSH 也未提供参数，提示用户选择方案
echo ""
echo "----------------------------------------------------------------------"
echo "【提示】GitHub 要求推送权限认证。请选择以下任意一种方式完成上传："
echo ""
echo "方案 A（推荐，一键完成）："
echo "  在下方命令中带上您的 GitHub Personal Access Token (PAT) 直接运行："
echo "  bash ./upload_to_github.sh <YOUR_GITHUB_TOKEN>"
echo ""
echo "方案 B（使用已为您生成的 SSH 密钥）："
echo "  1. 打开浏览器登录 GitHub: https://github.com/settings/ssh/new"
echo "  2. Title 填写: QCS8550-Aidlux"
echo "  3. Key 栏粘贴以下公钥:"
echo "     $(cat ~/.ssh/id_ed25519.pub)"
echo "  4. 点击 [Add SSH key] 保存后，再次运行:"
echo "     bash ./upload_to_github.sh"
echo "----------------------------------------------------------------------"
