import sys
import rclpy
from PySide6.QtWidgets import QApplication
from main_window import MainWindow
from ros2_worker import ROS2Worker

def main():
    # 创建Qt应用
    app = QApplication(sys.argv)
    
    # 创建ROS2工作线程
    ros_worker = ROS2Worker()
    
    # 启动ROS2
    if not ros_worker.start_ros():
        print("无法启动ROS2，退出应用")
        return 1
    
    # 创建主窗口
    window = MainWindow(ros_worker)
    window.show()
    
    # 运行应用
    result = app.exec()
    
    # 关闭ROS2
    ros_worker.shutdown_ros()
    
    return result

if __name__ == "__main__":
    sys.exit(main())