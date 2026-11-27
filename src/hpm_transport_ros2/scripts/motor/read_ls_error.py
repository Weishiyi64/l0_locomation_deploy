#!/usr/bin/env python3

import argparse
import sys

SERVICE = "/actuator/motor/read_ls_error"
ERROR_NAMES = ("无错误", "电机或 MOS 过热", "过流", "欠压", "编码器错误",
               "UVW 缺相或电流检测偏置过大", "过压", "其它错误")
ALARM_NAMES = {
    0x020: "参数校验和异常",
    0x021: "参数格式异常",
    0x022: "系统参数校验和异常",
    0x030: "主回路检测部故障",
    0x040: "参数设定超出范围",
    0x042: "参数组合异常",
    0x050: "伺服单元与电机组合错误",
    0x0B0: "伺服 ON 指令无效",
    0x100: "过电流检出",
    0x320: "再生过载",
    0x400: "主回路过压",
    0x410: "主回路欠压",
    0x510: "电机超速",
    0x520: "振动报警",
    0x521: "自动调整报警（调整时检出振动）",
    0x550: "最高速度设定异常",
    0x710: "瞬时过载",
    0x720: "连续过载",
    0x730: "动态制动（DB）过载",
    0x731: "动态制动（DB）过载",
    0x7A0: "驱动器内部 MOS 温度异常",
    0x810: "编码器备份报警",
    0x820: "编码器校验和报警",
    0x830: "编码器电池报警",
    0x840: "编码器数据报警",
    0x861: "电机过热",
    0x8A2: "编码器错误（手册未细分）",
    0x900: "警告：位置偏差过大",
    0x901: "警告：伺服 ON 时位置偏差过大",
    0x910: "警告：过载",
    0x911: "警告：振动",
    0x920: "警告：再生过载",
    0x921: "警告：动态制动（DB）过载",
    0x930: "警告：绝对值编码器电池异常",
    0x941: "警告：参数变更后需要重新上电",
    0x950: "警告：运动控制指令设定异常",
    0xB31: "电流检测故障 1",
    0xB32: "电流检测故障 2",
    0xC10: "伺服失控检出",
    0xC50: "磁极检测失败",
    0xC51: "磁极检测时检出超程",
    0xC80: "编码器清除异常",
    0xC90: "编码器通信故障",
    0xC91: "编码器通信位置数据加速度异常",
    0xC92: "编码器通信定时器异常",
    0xCA0: "编码器参数异常",
    0xCB0: "编码器回送校验异常",
    0xCC0: "编码器旋转圈数上限值不一致",
    0xCF1: "编码器通信故障",
    0xD00: "位置偏差过大",
    0xD01: "伺服 ON 时位置偏差过大",
    0xD02: "伺服 ON 时因速度限制引起的位置偏差过大",
    0xD10: "电机与负载位置间偏差过大",
    0xD30: "位置数据过大",
    0xE60: "CAN 总线连接异常",
}


def describe_alarm(error_code, alarm_code):
    if alarm_code == 0:
        detail = "无报警" if error_code == 0 else "未返回具体报警码"
    elif alarm_code in ALARM_NAMES:
        detail = f"(A.{alarm_code:03X})：{ALARM_NAMES[alarm_code]}"
    else:
        detail = "未知报警码"
    return f"errorcode={error_code}（{ERROR_NAMES[error_code]}） | 报警码=0x{alarm_code:04X} {detail}"


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("motor_index", type=int, help="业务节点配置的电机索引（0..255）")
    index = parser.parse_args().motor_index
    if not 0 <= index <= 255:
        parser.error("motor_index 必须在 0..255 范围内")

    try:
        import rclpy
        from luvbot_hardware_interfaces.srv import MotorReadLsError
    except ImportError as exc:
        print(f"缺少 ROS 2 查询接口，请先构建工作区并 source install/setup.bash：{exc}", file=sys.stderr)
        return 1

    rclpy.init(args=[])
    node = rclpy.create_node("read_ls_error")
    try:
        client = node.create_client(MotorReadLsError, SERVICE)
        if not client.wait_for_service(timeout_sec=3):
            print(f"未找到 {SERVICE}，未发送。请确认 ROS_DOMAIN_ID 相同，"
                  "且 hpm_system 已加载新版电机模块。", file=sys.stderr)
            return 1
        future = client.call_async(MotorReadLsError.Request(motor_index=index))
        rclpy.spin_until_future_complete(node, future, timeout_sec=3)
        if not future.done():
            print(f"motor_index={index}：等待 ROS 服务响应超时，结果未知", file=sys.stderr)
            return 1
        response = future.result()
        if not response.success:
            print(f"motor_index={index}：{response.message}", file=sys.stderr)
            return 1
        print(f"motor_index={index} {response.motor_name} MCAN{response.bus_index} "
              f"CAN ID=0x{response.motor_id:03X} | {describe_alarm(response.error_code, response.alarm_code)}")
        return 0
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    sys.exit(main())
