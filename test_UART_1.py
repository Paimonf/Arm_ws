#!/usr/bin/env python3
import serial
import time

# 串口配置
PORT = '/dev/ttyUSB0'
BAUDRATE = 115200  # 根据你的STM32设置调整

def simple_serial_test():
    try:
        # 打开串口
        with serial.Serial(PORT, BAUDRATE, timeout=1) as ser:
            print(f"已连接 {PORT}，波特率 {BAUDRATE}")
            print("按 Ctrl+C 停止测试\n")
            
            # 发送测试消息
            test_msg = "@LED_OFF##"
            ser.write(test_msg.encode())
            print(f"[发送] {test_msg.strip()}")
            
            # 接收回显
            time.sleep(0.1)  # 等待响应
            while ser.in_waiting > 0:
                response = ser.readline().decode().strip()
                print(f"[接收] {response}")
            
            # 简单交互循环
            while True:
                user_input = input("输入要发送的消息: ")
                print(f"[发送] {user_input.strip()}")
                ser.write(user_input.encode())
                
                # 读取响应
                time.sleep(0.1)
                while ser.in_waiting > 0:
                    print(f"[响应] {ser.readline().decode().strip()}")
                    
    except serial.SerialException as e:
        print(f"串口错误: {e}")
    except KeyboardInterrupt:
        print("\n测试结束")

if __name__ == "__main__":
    simple_serial_test()