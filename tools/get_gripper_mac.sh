#!/bin/bash
# =============================================================
# 获取 Xense 夹爪 MAC 地址
# 已知 IP：
#   左臂夹爪  192.168.1.101
#   右臂夹爪  192.168.1.102
# 输出格式与 XenseGripper.create(mac_addr=...) 一致（无冒号小写）
# =============================================================

LEFT_IP="192.168.1.101"
RIGHT_IP="192.168.1.102"

get_mac() {
    local ip="$1"
    local label="$2"

    printf "%-6s (%s)  → " "$label" "$ip"

    # 先 ping 1 次，确保 ARP 缓存有记录（超时 1s）
    ping -c 1 -W 1 "$ip" > /dev/null 2>&1
    if [ $? -ne 0 ]; then
        echo "❌ 不可达，请检查网络连接"
        return 1
    fi

    # 从 ARP 缓存读取 MAC（支持 arp -n 和 /proc/net/arp 两种方式）
    local raw_mac
    raw_mac=$(arp -n "$ip" 2>/dev/null | awk '/'"$ip"'/{print $3}' | head -1)

    # 某些系统 arp 命令结果为 "<incomplete>"，改读 /proc/net/arp
    if [[ -z "$raw_mac" || "$raw_mac" == "<incomplete>" ]]; then
        raw_mac=$(awk -v ip="$ip" '$1==ip{print $4}' /proc/net/arp | head -1)
    fi

    if [[ -z "$raw_mac" || "$raw_mac" == "00:00:00:00:00:00" ]]; then
        echo "❌ ARP 未解析到 MAC，请确认设备已上电并在同一子网"
        return 1
    fi

    # 去掉冒号，转小写 → XenseGripper 所需格式
    local mac_clean
    mac_clean=$(echo "$raw_mac" | tr -d ':' | tr '[:upper:]' '[:lower:]')

    echo "✅  MAC = $mac_clean   (原始: $raw_mac)"
    return 0
}

echo "================================================"
echo " Xense 夹爪 MAC 地址查询"
echo "================================================"
get_mac "$LEFT_IP"  "左臂"
get_mac "$RIGHT_IP" "右臂"
echo "================================================"
