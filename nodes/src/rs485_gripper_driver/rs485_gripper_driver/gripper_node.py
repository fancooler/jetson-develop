#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
import serial
import struct
import threading
import time
from std_msgs.msg import Float32MultiArray

class GripperRS485Driver(Node):
    def __init__(self):
        super().__init__('gripper_rs485_node')

        # 1. 参数声明与加载
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud', 921600)
        self.declare_parameter('device_id', 1)
        self.declare_parameter('query_rate', 50.0)

        self.port = self.get_parameter('port').get_parameter_value().string_value
        self.baudrate = self.get_parameter('baud').get_parameter_value().integer_value
        self.device_id = self.get_parameter('device_id').get_parameter_value().integer_value
        self.loop_rate_val = self.get_parameter('query_rate').get_parameter_value().double_value

        # 2. 状态存储与锁
        self.pending_cmd = None
        self.cmd_lock = threading.Lock()
        self.latest_status = [0.0, 0.0, 0.0]
        self.serial_buffer = bytearray()

        # 3. 串口初始化
        try:
            self.ser = serial.Serial(
                self.port, self.baudrate, timeout=0.01, write_timeout=0.1
            )
            self.get_logger().info(f"Connected to {self.port} at {self.baudrate}")
        except Exception as e:
            self.get_logger().error(f"Serial Open Failed: {e}")
            return

        # 4. ROS 2 接口
        # 使用 qos_profile_sensor_data 或指定深度
        self.sub = self.create_subscription(
            Float32MultiArray, 
            'gripper_cmd', 
            self.cmd_callback, 
            10
        )
        self.pub = self.create_publisher(Float32MultiArray, 'gripper_status', 10)

        # 5. 启动通讯线程 (在 ROS 2 中，依然建议将阻塞式 IO 放在独立线程)
        self.comm_thread = threading.Thread(target=self.main_comm_loop)
        self.comm_thread.daemon = True
        self.comm_thread.start()

    def cmd_callback(self, msg):
        """更新待发送指令"""
        with self.cmd_lock:
            self.pending_cmd = msg.data

    def calculate_checksum(self, data):
        return sum(data) & 0xFF

    def main_comm_loop(self):
        """主通讯循环"""
        # ROS 2 的 Rate 需要关联到节点
        #rate = self.create_rate(self.loop_rate_val)
        # **
        dt = 1.0 / self.loop_rate_val
        
        while rclpy.ok():
            #**
            t_start = time.time()
            # A. 检查并发送控制指令
            current_cmd = None
            with self.cmd_lock:
                if self.pending_cmd is not None:
                    current_cmd = self.pending_cmd
                    self.pending_cmd = None

            if current_cmd:
                print(f"cmd rcved: {current_cmd}")
                self._send_control(current_cmd)
                time.sleep(0.002) # RS485 物理层切换延迟

            # B. 发送查询指令
            self._send_query()

            # C. 等待并处理反馈
            time.sleep(0.005) 
            self._read_and_process()

            # D. 发布反馈
            status_msg = Float32MultiArray()
            status_msg.data = self.latest_status
            self.pub.publish(status_msg)

            #rate.sleep()
            #**
            elapsed = time.time() - t_start
            remaining = dt - elapsed
            if remaining > 0:
                time.sleep(remaining)

    def _send_control(self, data):
        try:
            pos, vel, tor = data[0], data[1], data[2]
            payload = list(struct.pack('<fff', pos, vel, tor))
            prefix = [0x01, 0x0D, self.device_id]
            chk = self.calculate_checksum(prefix + payload)
            frame = b'\xAA\x55' + bytearray(prefix + payload) + struct.pack('B', chk)
            self.ser.write(frame)
            self.ser.flush()
        except Exception as e:
            print("send NOK")
            self.get_logger().error(f"Write CMD Error: {e}")

    def _send_query(self):
        try:
            prefix = [0x02, 0x01, self.device_id]
            chk = self.calculate_checksum(prefix)
            frame = b'\xAA\x55' + bytearray(prefix) + struct.pack('B', chk)
            self.ser.write(frame)
            self.ser.flush()
        except Exception as e:
            self.get_logger().error(f"Write Query Error: {e}")

    def _read_and_process(self):
        try:
            waiting = self.ser.in_waiting
            if waiting > 0:
                self.serial_buffer.extend(self.ser.read(waiting))
            
            while len(self.serial_buffer) >= 18:
                # 寻找帧头 0x55 0xAA (注意：此处应与硬件反馈协议匹配)
                idx = self.serial_buffer.find(b'\x55\xAA')
                if idx == -1:
                    # 缓冲区里没有帧头了，但可能还有残余数据，若长度过大则清理
                    if len(self.serial_buffer) > 50:
                        self.serial_buffer.clear()
                    break
                
                if idx > 0:
                    del self.serial_buffer[:idx]
                    continue
                
                frame = self.serial_buffer[:18]
                if self.calculate_checksum(frame[2:-1]) == frame[-1]:
                    res = struct.unpack('<fff', frame[5:17])
                    self.latest_status = list(res)
                    del self.serial_buffer[:18] # 处理完一帧，删除它
                else:
                    # 校验失败，弹出当前伪帧头继续找下一个
                    del self.serial_buffer[0:1]
                    
        except Exception as e:
            self.get_logger().error(f"Read Error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = GripperRS485Driver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
