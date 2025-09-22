from PySide6.QtWidgets import (QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, 
                               QPushButton, QLabel, QTextEdit, QStatusBar, QFrame)
from PySide6.QtCore import Qt, Slot
from PySide6.QtGui import QImage, QPixmap
import cv2

class MainWindow(QMainWindow):
    def __init__(self, ros_worker):
        super().__init__()
        self.ros_worker = ros_worker
        self.init_ui()
        self.connect_signals()
        
    def init_ui(self):
        """初始化界面"""
        self.setWindowTitle("蓝莓采摘控制系统")
        self.setGeometry(100, 100, 1000, 700)
        
        # 中央部件
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        # 主布局
        main_layout = QVBoxLayout()
        central_widget.setLayout(main_layout)
        
        # 控制按钮区域
        control_frame = QFrame()
        control_frame.setFrameStyle(QFrame.StyledPanel | QFrame.Raised)
        control_layout = QHBoxLayout()
        control_frame.setLayout(control_layout)
        
        self.start_btn = QPushButton("启动检测")
        self.start_btn.setMinimumHeight(40)
        self.stop_btn = QPushButton("停止检测")
        self.stop_btn.setMinimumHeight(40)
        self.stop_btn.setEnabled(False)
        
        control_layout.addWidget(self.start_btn)
        control_layout.addWidget(self.stop_btn)
        control_layout.addStretch()
        
        # 图像显示区域
        image_frame = QFrame()
        image_frame.setFrameStyle(QFrame.StyledPanel | QFrame.Sunken)
        image_layout = QVBoxLayout()
        image_frame.setLayout(image_layout)
        
        self.image_label = QLabel()
        self.image_label.setAlignment(Qt.AlignCenter)
        self.image_label.setMinimumSize(640, 480)
        self.image_label.setText("等待图像...")
        self.image_label.setStyleSheet("border: 1px solid gray;")
        
        image_layout.addWidget(QLabel("检测图像:"))
        image_layout.addWidget(self.image_label)
        
        # 状态信息区域
        status_frame = QFrame()
        status_frame.setFrameStyle(QFrame.StyledPanel | QFrame.Sunken)
        status_layout = QVBoxLayout()
        status_frame.setLayout(status_layout)
        
        status_layout.addWidget(QLabel("状态信息:"))
        self.status_text = QTextEdit()
        self.status_text.setMaximumHeight(100)
        self.status_text.setReadOnly(True)
        status_layout.addWidget(self.status_text)
        
        # 添加到主布局
        main_layout.addWidget(control_frame)
        main_layout.addWidget(image_frame)
        main_layout.addWidget(status_frame)
        
        # 状态栏
        self.statusBar().showMessage("就绪")
        
        # 连接按钮信号
        self.start_btn.clicked.connect(self.on_start_clicked)
        self.stop_btn.clicked.connect(self.on_stop_clicked)
    
    def connect_signals(self):
        """连接ROS工作线程的信号"""
        self.ros_worker.image_received.connect(self.update_image)
        self.ros_worker.status_update.connect(self.update_status)
        self.ros_worker.error_occurred.connect(self.show_error)
    
    @Slot(object)
    def update_image(self, cv_image):
        """更新显示的图像"""
        try:
            # 调整图像大小以适应标签
            height, width, channel = cv_image.shape
            bytes_per_line = 3 * width
            qt_image = QImage(cv_image.data, width, height, bytes_per_line, QImage.Format_RGB888)
            qt_image = qt_image.rgbSwapped()  # BGR -> RGB
            
            # 缩放图像以适应标签
            pixmap = QPixmap.fromImage(qt_image)
            scaled_pixmap = pixmap.scaled(
                self.image_label.width(), 
                self.image_label.height(),
                Qt.KeepAspectRatio,
                Qt.SmoothTransformation
            )
            
            self.image_label.setPixmap(scaled_pixmap)
        except Exception as e:
            self.status_text.append(f"图像显示错误: {str(e)}")
    
    @Slot(str)
    def update_status(self, message):
        """更新状态信息"""
        self.status_text.append(message)
        self.statusBar().showMessage(message)
    
    @Slot(str)
    def show_error(self, error_message):
        """显示错误信息"""
        self.status_text.append(f"错误: {error_message}")
        self.statusBar().showMessage(f"错误: {error_message}")
    
    @Slot()
    def on_start_clicked(self):
        """启动按钮点击事件"""
        self.start_btn.setEnabled(False)
        self.stop_btn.setEnabled(True)
        self.ros_worker.start_detection()
    
    @Slot()
    def on_stop_clicked(self):
        """停止按钮点击事件"""
        self.start_btn.setEnabled(True)
        self.stop_btn.setEnabled(False)
        self.ros_worker.stop_detection()
    
    def closeEvent(self, event):
        """窗口关闭事件"""
        self.ros_worker.shutdown_ros()
        event.accept()