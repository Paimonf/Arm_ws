#!/usr/bin/env python3
import subprocess
import sys
import os
os.environ['QT_QPA_PLATFORM'] = 'xcb'

from main_window import MainWindow
from PyQt5.QtWidgets import QApplication

def main():
    # 启动ROS2节点
    launch_process = subprocess.Popen([
        'ros2', 'launch', 'test_pkg', 'control.launch.py'
    ])
    
    # 启动Qt界面
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    
    # 设置退出时清理
    def cleanup():
        launch_process.terminate()
        launch_process.wait()
    
    app.aboutToQuit.connect(cleanup)
    
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()