#!/bin/bash

# ================= 强制传参检查 =================
# 如果传入的参数个数不等于 1，则报错并退出
if [ "$#" -ne 1 ]; then
    echo "❌ 错误: 必须指定一个配置文件路径！"
    echo "✅ 用法: sudo ./start_dhcp.sh <配置文件路径>"
    echo "   示例: sudo ./start_dhcp.sh /etc/udhcpd.conf"
    echo "   示例: sudo ./start_dhcp.sh ./my_custom_dhcp.conf"
    exit 1
fi

# 将传入的第一个参数赋值给 CONFIG_FILE 变量
CONFIG_FILE="$1"

# 检查传入的文件是否真实存在
if [ ! -f "$CONFIG_FILE" ]; then
    echo "❌ 错误: 配置文件 '$CONFIG_FILE' 不存在，请检查路径！"
    exit 1
fi

# ================= 环境自动初始化 =================
LEASE_DIR="/var/lib/misc"
LEASE_FILE="$LEASE_DIR/udhcpd.leases"

echo ">>> [1/3] 正在准备 udhcpd 运行环境..."
if [ ! -d "$LEASE_DIR" ]; then
    sudo mkdir -p "$LEASE_DIR"
    echo "   ✔ 已创建目录: $LEASE_DIR"
fi

if [ ! -f "$LEASE_FILE" ]; then
    sudo touch "$LEASE_FILE"
    echo "   ✔ 已创建租约文件: $LEASE_FILE"
fi

# ================= 启动服务 =================
echo ">>> [2/3] 正在加载配置文件: $CONFIG_FILE"
echo ">>> [3/3] 启动 udhcpd 服务..."

# 使用 -f 前台运行，这样您可以实时看到日志输出
# 如果您希望在后台运行，可以去掉 -f 并在末尾加 &
sudo udhcpd -f "$CONFIG_FILE"