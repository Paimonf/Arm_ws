# -*- coding: utf-8 -*-
# main.py
import time
import sys
import os
from PIL import ImageFont
from std_srvs.srv import Empty
sys.path.append('UI')
from UI.UiMain import Ui_MainWindow
import sys
import re
from PyQt5.QtCore import QTimer, Qt, QThread, pyqtSignal, QCoreApplication, QEvent
from PyQt5.QtGui import QPixmap, QPalette, QBrush, QFont, QTransform, QIcon
from PyQt5.QtWidgets import QMainWindow, QFileDialog, \
    QMessageBox, QWidget, QHeaderView, QTableWidgetItem, QAbstractItemView, \
    QVBoxLayout, QLineEdit, QPushButton, QApplication, QLabel, QDialog, \
    QHBoxLayout, QSizePolicy
from utils import utils as tools
import cv2
import glob
from utils import config
from UI.QssLoader import QSSLoader
from UI.precess_bar import ProgressBar
import numpy as np
import torch
from utils.database import SqliteDBOperator
import warnings
warnings.filterwarnings("ignore")

# -------------------- ROS2 相关导入 --------------------
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from blueberry_interfaces.msg import DetectedBerries, Berry
from blueberry_interfaces.srv import PathPlan, SetParams
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
# -------------------------------------------------------

class LoginWidget(QWidget):
    login_success_signal = pyqtSignal()

    def __init__(self):
        super().__init__()
        self.setObjectName("LoginWidget")
        self.resize(1000, 600)
        self.init_ui()
        self.setWindowTitle(f"{config.SYSTEM_NAME}-登录")
        style_file = 'UI/style.css'
        qssStyleSheet = QSSLoader.read_qss_file(style_file)
        self.setStyleSheet(qssStyleSheet)

    def init_ui(self):
        system_name_font = QFont("华文楷体")
        system_name_font.setPointSize(24)
        system_name_font.setBold(True)

        custom_font = QFont("华文楷体")
        custom_font.setPointSize(12)

        main_layout = QVBoxLayout(self)
        main_layout.setContentsMargins(50, 50, 50, 50)

        # 标题
        system_name_label = QLabel(config.SYSTEM_NAME)
        system_name_label.setObjectName("Title")
        system_name_label.setFont(system_name_font)
        system_name_label.setStyleSheet("""
            color: #9A3412; 
            background-color: rgba(255, 255, 255, 0.85); 
            border: 1px solid #FED7AA;
            border-radius: 12px; 
            padding: 15px 30px;
        """)
        system_name_label.setAlignment(Qt.AlignCenter)
        main_layout.addWidget(system_name_label, 0, Qt.AlignTop | Qt.AlignHCenter)
        main_layout.addStretch(1)

        # 登录框
        central_widget = QWidget()
        central_widget.setStyleSheet("""
            .QWidget { 
                background-color: rgba(255, 255, 255, 0.95); 
                border-radius: 16px; 
                border: 1px solid #FFEDD5;
            }
        """)
        central_widget.setFixedWidth(400)
        central_layout = QVBoxLayout(central_widget)
        central_layout.setContentsMargins(40, 40, 40, 40)
        central_layout.setSpacing(24)

        # 用户名
        user_layout = QVBoxLayout()
        user_layout.setSpacing(8)
        self.username_label = QLabel('用户名')
        self.username_label.setFont(custom_font)
        self.username_edit = QLineEdit()
        self.username_edit.setPlaceholderText("请输入用户名")
        self.username_edit.setMinimumHeight(45)
        self.username_edit.setStyleSheet("""
            QLineEdit {
                border: 1px solid #E7E5E4;
                border-radius: 8px;
                padding: 8px 12px;
                selection-background-color: #F97316;
            }
            QLineEdit:focus {
                border: 1px solid #F97316;
                background-color: #FFFAF0;
            }
        """)
        user_layout.addWidget(self.username_label)
        user_layout.addWidget(self.username_edit)
        central_layout.addLayout(user_layout)

        # 密码
        pwd_layout = QVBoxLayout()
        pwd_layout.setSpacing(8)
        self.password_label = QLabel('密码')
        self.password_label.setFont(custom_font)
        self.password_edit = QLineEdit()
        self.password_edit.setPlaceholderText("请输入密码")
        self.password_edit.setEchoMode(QLineEdit.Password)
        self.password_edit.setMinimumHeight(45)
        self.password_edit.setStyleSheet("""
            QLineEdit {
                border: 1px solid #E7E5E4;
                border-radius: 8px;
                padding: 8px 12px;
                selection-background-color: #F97316;
            }
            QLineEdit:focus {
                border: 1px solid #F97316;
                background-color: #FFFAF0;
            }
        """)
        pwd_layout.addWidget(self.password_label)
        pwd_layout.addWidget(self.password_edit)
        central_layout.addLayout(pwd_layout)

        central_layout.addSpacing(10)

        # 按钮
        button_layout = QVBoxLayout()
        button_layout.setSpacing(15)
        self.login_button = QPushButton('登 录')
        self.login_button.setMinimumHeight(50)
        self.login_button.setCursor(Qt.PointingHandCursor)
        self.login_button.clicked.connect(self.check_login)
        self.login_button.setStyleSheet("""
            QPushButton {
                background-color: #F97316;
                color: white;
                border: none;
                border-radius: 10px;
                font-weight: 700;
                font-size: 16px;
                letter-spacing: 1px;
            }
            QPushButton:hover {
                background-color: #EA580C;
            }
            QPushButton:pressed {
                background-color: #C2410C;
            }
        """)

        self.register_button = QPushButton('注册账号')
        self.register_button.setMinimumHeight(45)
        self.register_button.setCursor(Qt.PointingHandCursor)
        self.register_button.setStyleSheet("""
            QPushButton {
                background-color: transparent; 
                color: #C2410C; 
                border: 1px solid #FED7AA;
                font-weight: 600;
            }
            QPushButton:hover {
                background-color: #FFF7ED;
                color: #9A3412;
                border-color: #FDBA74;
            }
        """)
        self.register_button.clicked.connect(self.show_register_dialog)

        button_layout.addWidget(self.login_button)
        button_layout.addWidget(self.register_button)
        central_layout.addLayout(button_layout)

        main_layout.addWidget(central_widget, 0, Qt.AlignCenter)
        main_layout.addStretch(2)

        self.username_label.setFont(custom_font)
        self.password_label.setFont(custom_font)
        self.login_button.setFont(custom_font)
        self.register_button.setFont(custom_font)

        self.setStyleSheet("#LoginWidget { background-color: #FFF7ED; }")

    def check_login(self):
        username = self.username_edit.text()
        password = self.password_edit.text()
        db_operator = SqliteDBOperator()
        if db_operator.check_user_login(username, password):
            self.login_success_signal.emit()
            self.close()
            db_operator.close_connection()
        else:
            QMessageBox.warning(self, "登录失败", "账号或密码错误，请重新输入", QMessageBox.Ok)
            db_operator.close_connection()

    def show_register_dialog(self):
        register_dialog = QDialog(self)
        register_dialog.setWindowTitle("用户注册")
        register_dialog.setMinimumWidth(400)
        register_layout = QVBoxLayout()
        register_layout.setContentsMargins(30,30,30,30)
        register_layout.setSpacing(15)

        style_font = QFont("华文楷体", 12)

        # 通用样式
        label_style = "color: #334155; font-weight: bold;"
        edit_style = """
            QLineEdit {
                border: 1px solid #E7E5E4;
                border-radius: 5px;
                padding: 8px;
                selection-background-color: #F97316;
            }
            QLineEdit:focus {
                border: 1px solid #F97316;
                background-color: #FFFAF0;
            }
        """

        # 用户名
        h1 = QHBoxLayout()
        register_username_label = QLabel('用户名:')
        register_username_label.setFont(style_font)
        register_username_label.setStyleSheet(label_style)
        register_username_edit = QLineEdit()
        register_username_edit.setStyleSheet(edit_style)
        h1.addWidget(register_username_label)
        h1.addWidget(register_username_edit)
        register_layout.addLayout(h1)

        # 密码
        h2 = QHBoxLayout()
        register_password_label = QLabel('密  码:')
        register_password_label.setFont(style_font)
        register_password_label.setStyleSheet(label_style)
        register_password_edit = QLineEdit()
        register_password_edit.setEchoMode(QLineEdit.Password)
        register_password_edit.setStyleSheet(edit_style)
        h2.addWidget(register_password_label)
        h2.addWidget(register_password_edit)
        register_layout.addLayout(h2)
        
        # 确认密码
        h3 = QHBoxLayout()
        register_confirm_password_label = QLabel('确认密码:')
        register_confirm_password_label.setFont(style_font)
        register_confirm_password_label.setStyleSheet(label_style)
        register_confirm_password_edit = QLineEdit()
        register_confirm_password_edit.setEchoMode(QLineEdit.Password)
        register_confirm_password_edit.setStyleSheet(edit_style)
        h3.addWidget(register_confirm_password_label)
        h3.addWidget(register_confirm_password_edit)
        register_layout.addLayout(h3)

        register_button = QPushButton('注 册')
        register_button.setFont(style_font)
        register_button.setMinimumHeight(40)
        register_button.setCursor(Qt.PointingHandCursor)
        register_button.setStyleSheet("""
            QPushButton {
                background-color: #F97316;
                color: white;
                border: none;
                border-radius: 8px;
                font-weight: 700;
            }
            QPushButton:hover {
                background-color: #EA580C;
            }
        """)
        register_button.clicked.connect(lambda: self.check_register_info(register_username_edit.text(),
                                                                         register_password_edit.text(),
                                                                         register_confirm_password_edit.text(),
                                                                         register_dialog))
        
        register_layout.addStretch(1)
        register_layout.addWidget(register_button)

        register_dialog.setLayout(register_layout)
        register_dialog.setStyleSheet("QDialog { background-color: #FFF7ED; }")
        register_dialog.exec_()

    def check_register_info(self, username, password, confirm_password, register_dialog):
        # 检查用户名是否为空
        if username == "":
            QMessageBox.warning(self, "注册失败", "用户名为空", QMessageBox.Ok)
            return
        # 检查用户名长度是否符合要求
        elif len(username) < 4:
            QMessageBox.warning(self, "注册失败", "用户名长度不符合要求，至少4位", QMessageBox.Ok)
            return
        elif len(username) > 12:
            QMessageBox.warning(self, "注册失败", "用户名长度不符合要求，至多12位", QMessageBox.Ok)
            return
        # 新增：检查用户名格式是否符合要求
        elif not re.match(r'^[a-zA-Z][a-zA-Z0-9]*$', username):
            QMessageBox.warning(self, "注册失败", "用户名格式不符合要求，只能包含大小写字母和数字，且不能以数字开头", QMessageBox.Ok)
            return
        # 检查两次密码输入是否一致
        elif password != confirm_password:
            QMessageBox.warning(self, "注册失败", "两次密码输入不一致", QMessageBox.Ok)
            return
        # 检查密码长度（至少6位，至多20位）
        elif len(password) < 6:
            QMessageBox.warning(self, "注册失败", "密码长度不符合要求，至少6位", QMessageBox.Ok)
            return
        elif len(password) > 20:
            QMessageBox.warning(self, "注册失败", "密码长度不符合要求，至多20位", QMessageBox.Ok)
            return
        # 检查密码是否至少包含数字、字母、特殊符号中的两种
        categories = [r'\d', r'[a-zA-Z]', r'[^\w]']
        count = 0
        for category in categories:
            if re.search(category, password):
                count += 1
        if count < 2:
            QMessageBox.warning(self, "注册失败", "密码至少需包含数字、字母、特殊符号中的两种", QMessageBox.Ok)
            return
        else:
            db_operator = SqliteDBOperator()
            # 检查用户名是否已存在
            if db_operator.check_user_exists(username):
                QMessageBox.warning(self, "注册失败", "该用户已存在，请直接登录", QMessageBox.Ok)
                return
            db_operator.insert_user_data(username, password)
            msg_box = QMessageBox.information(self, "注册成功", "注册成功", QMessageBox.Ok)
            # 注册成功后，关闭注册对话框，回到登录界面
            if msg_box == QMessageBox.Ok:
                register_dialog.close()

# -------------------- ROS2 线程类 --------------------
class ROS2Thread(QThread):
    image_received = pyqtSignal(object)      # 图像（numpy数组）
    berries_received = pyqtSignal(list)      # 蓝莓列表
    param_update_finished = pyqtSignal(bool, str)

    def __init__(self):
        super().__init__()
        self.node = None
        self.bridge = CvBridge()
        self.latest_berries = []  # 缓存最新检测结果
        self.path_plan_client = None
        self.emergency_client = None
        self.params_client = None

    def run(self):
        rclpy.init(args=[])
        self.node = Node('qt_interface_node')

        # 订阅检测图像
        self.image_sub = self.node.create_subscription(
            Image, 'debug_image', self.image_callback, 10)

        # 订阅检测结果
        self.berries_sub = self.node.create_subscription(
            DetectedBerries, '/detected_berries', self.berries_callback, 10)

        # 创建服务客户端
        self.start_detection_client = self.node.create_client(Empty, 'start_detection')
        self.stop_detection_client = self.node.create_client(Empty, 'stop_detection')
        self.params_client = self.node.create_client(SetParams, 'set_detection_params')

        # 等待服务可用
        self.node.get_logger().info('Waiting for ROS services...')
        # while not self.start_detection_client.wait_for_service(timeout_sec=1.0) :
        #     self.node.get_logger().info('start_detection服务不可用，等待中...')
        # while not self.stop_detection_client.wait_for_service(timeout_sec=1.0) :
        #     self.node.get_logger().info('stop_detection服务不可用，等待中...')
        # while not self.params_client.wait_for_service(timeout_sec=1.0) :
        #     self.node.get_logger().info('setparams服务不可用，等待中...')
        self.node.get_logger().info('All services available.')

        # 开始 spin
        rclpy.spin(self.node)
        self.node.destroy_node()
        rclpy.shutdown()

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            self.image_received.emit(cv_image)
        except Exception as e:
            print(f"Image conversion error: {e}")

    def berries_callback(self, msg):
        #self.node.get_logger().info(f"接收蓝莓信息:{msg}")
        berry_list = []
        for berry in msg.berries:
            berry_list.append({
                'id': berry.id,
                'x': berry.position.x,
                'y': berry.position.y,
                'z': berry.position.z,
                'confidence': berry.confidence,
                'is_ripe': berry.is_ripe,
                'size': berry.size
            })
        #self.node.get_logger().info(f"接收蓝莓信息:{berry_list}")
        self.latest_berries = berry_list
        self.berries_received.emit(berry_list)

    def call_path_plan(self, berries_msg):
        self.call_start_service()

    def path_plan_done(self, future):
        try:
            response = future.result()
            if response.success:
                print("Path plan success")
            else:
                print(f"Path plan failed: {response.message}")
        except Exception as e:
            print(f"Service call failed: {e}")

    def emergency_done(self, future):
        try:
            response = future.result()
            if response.success:
                print("Emergency stop executed")
            else:
                print(f"Emergency stop failed: {response.message}")
        except Exception as e:
            print(f"Emergency stop call failed: {e}")

    def call_start_service(self):
        """调用启动检测服务"""
        req = Empty.Request()
        future = self.start_detection_client.call_async(req)
        future.add_done_callback(self.start_service_callback)
    
    def call_stop_service(self):
        """调用停止检测服务"""
        req = Empty.Request()
        future = self.stop_detection_client.call_async(req)
        future.add_done_callback(self.stop_service_callback)

    def start_service_callback(self, future):
        """启动服务回调"""
        try:
            response = future.result()
            self.node.get_logger().info('检测已启动')
            #self.worker.status_update.emit('检测已启动')
        except Exception as e:
            self.node.get_logger().error(f'启动服务调用失败: {str(e)}')
            #self.worker.error_occurred.emit(f'启动服务调用失败: {str(e)}')
    
    def stop_service_callback(self, future):
        """停止服务回调"""
        try:
            response = future.result()
            self.node.get_logger().info('检测已停止')
            #self.worker.status_update.emit('检测已停止')
        except Exception as e:
            self.node.get_logger().error(f'停止服务调用失败: {str(e)}')
            #self.worker.error_occurred.emit(f'停止服务调用失败: {str(e)}')

    def set_detection_params(self, confidence, iou):
        req = SetParams.Request()
        req.confidence = confidence
        req.iou = iou
        future = self.params_client.call_async(req)
        future.add_done_callback(self.param_done)

    def param_done(self, future):
        try:
            response = future.result()
            self.param_update_finished.emit(response.success, response.message)
        except Exception as e:
            self.param_update_finished.emit(False, str(e))

    def get_latest_berries(self):
        return self.latest_berries

# -------------------- 主窗口类 --------------------
class MainWindow(QMainWindow, Ui_MainWindow):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setupUi(self)
        self.setMinimumSize(800, 500)
        self.resize(1024, 768)
        self.initMain()
        self.signalconnect()

        self.update_model_display()
        # 加载css
        style_file = 'UI/style.css'
        qssStyleSheet = QSSLoader.read_qss_file(style_file)
        self.setStyleSheet(qssStyleSheet)

        # 设置SpinBox
        self.doubleSpinBox.setRange(0.0, 1.0)
        self.doubleSpinBox.setSingleStep(0.05)
        self.doubleSpinBox_2.setRange(0.0, 1.0)
        self.doubleSpinBox_2.setSingleStep(0.05)
        self.doubleSpinBox.setValue(0.45)
        self.doubleSpinBox_2.setValue(0.8)
        self.checkBox.setChecked(True)

        # 连接参数修改信号
        self.doubleSpinBox.valueChanged.connect(self.on_conf_thres_changed)
        self.doubleSpinBox_2.valueChanged.connect(self.on_iou_thres_changed)
        self.checkBox.stateChanged.connect(self.update_show_labels)

        # 启动ROS2线程
        self.ros_thread = ROS2Thread()
        self.ros_thread.image_received.connect(self.update_image_from_ros)
        self.ros_thread.berries_received.connect(self.update_berries_table)
        self.ros_thread.param_update_finished.connect(self.on_param_update_finished)
        self.ros_thread.start()

        # 添加控制按钮（若UI中没有，手动添加）
        # 假设已有按钮对象：self.start_harvest_btn, self.stop_harvest_btn
        # 如果UI中没有，可以在initMain中添加：
         # 根据实际布局调整
        self.start_harvest_btn.clicked.connect(self.on_start_harvest)
        self.stop_harvest_btn.clicked.connect(self.on_stop_harvest)

        # 禁用原有的YOLO模型相关功能
        self.model = None  # 移除YOLO加载

    def update_model_display(self):
        """更新模型名称显示"""
        model_path = config.model_path
        if os.path.exists(model_path):
            self.lineEdit_model.setText(os.path.basename(model_path))
            self.lineEdit_model.setToolTip(os.path.abspath(model_path))
        else:
            self.lineEdit_model.setText("未找到模型")
            self.lineEdit_model.setToolTip(f"路径不存在: {model_path}")

    def signalconnect(self):
        # 原有按钮连接保持不变，但内部实现需修改（如打开图片改为通过ROS？此处我们保留但禁用或重定义）
        self.PicBtn.clicked.connect(self.open_img)  # 可保留，但改为提示功能不可用
        self.comboBox.activated.connect(self.combox_change)
        self.btn_select_model.clicked.connect(self.select_model)
        self.VideoBtn.clicked.connect(self.vedio_show)
        self.CapBtn.clicked.connect(self.camera_show)
        self.SaveBtn.clicked.connect(self.save_detect_video)
        self.ExitBtn.clicked.connect(QCoreApplication.quit)
        self.FilesBtn.clicked.connect(self.detact_batch_imgs)



    def initMain(self):
        # 启用拖拽和点击
        self.setAcceptDrops(True)
        self.label_show.installEventFilter(self)
        self.label_show.setCursor(Qt.PointingHandCursor)

        self.org_path = None
        self.is_camera_open = False
        self.cap = None
        self.draw_img = None

        self.fontC = ImageFont.truetype("Font/platech.ttf", 25, 0)
        self.colors = tools.Colors()
        self.timer_camera = QTimer()
        self.timer_save_video = QTimer()

        # 表格设置
        self.tableWidget.verticalHeader().setSectionResizeMode(QHeaderView.Fixed)
        self.tableWidget.verticalHeader().setDefaultSectionSize(40)
        self.tableWidget.setColumnWidth(0, 80)
        self.tableWidget.setColumnWidth(1, 200)
        self.tableWidget.setColumnWidth(2, 150)
        self.tableWidget.setColumnWidth(3, 90)
        self.tableWidget.setColumnWidth(4, 230)
        self.tableWidget.setSelectionBehavior(QAbstractItemView.SelectRows)
        self.tableWidget.verticalHeader().setVisible(False)
        self.tableWidget.setAlternatingRowColors(True)

    def closeEvent(self, event):
        # 关闭时退出ROS2
        if self.ros_thread.isRunning():
            rclpy.shutdown()
            self.ros_thread.quit()
            self.ros_thread.wait()
        event.accept()

    # -------------------- 槽函数 --------------------
    def update_image_from_ros(self, cv_img):
        pix_img = tools.cvimg_to_qpiximg(cv_img)
        # 缩放以适应label
        label_size = self.label_show.size()
        scaled_pix = pix_img.scaled(label_size, Qt.KeepAspectRatio, Qt.SmoothTransformation)
        self.label_show.setPixmap(scaled_pix)
        self.label_show.setAlignment(Qt.AlignCenter)

    def update_berries_table(self, berry_list):
        # 清空表格
        self.tableWidget.setRowCount(0)
        self.tableWidget.clearContents()

        # 更新目标数目
        self.label_nums.setText(str(len(berry_list)))

        # 更新统计信息
        ripe_count = sum(1 for b in berry_list if b['is_ripe'])
        unripe_count = len(berry_list) - ripe_count
        summary = f"检测完成，共发现 {len(berry_list)} 个目标：\n\n• 成熟蓝莓: {ripe_count} 个\n• 未成熟蓝莓: {unripe_count} 个"
        if berry_list:
            max_conf_berry = max(berry_list, key=lambda b: b['confidence'])
            summary += f"\n\n最高置信度: {max_conf_berry['confidence']*100:.2f}%"
        self.textEdit_analysis.setText(summary)

        # 填充表格
        for i, berry in enumerate(berry_list):
            row_count = self.tableWidget.rowCount()
            self.tableWidget.insertRow(row_count)
            self.tableWidget.setItem(row_count, 0, QTableWidgetItem(str(i+1)))
            self.tableWidget.setItem(row_count, 1, QTableWidgetItem("实时检测"))
            self.tableWidget.setItem(row_count, 2, QTableWidgetItem("成熟" if berry['is_ripe'] else "未成熟"))
            self.tableWidget.setItem(row_count, 3, QTableWidgetItem(f"{berry['confidence']*100:.2f}%"))
            coord = f"({berry['x']:.3f}, {berry['y']:.3f}, {berry['z']:.3f})"
            self.tableWidget.setItem(row_count, 4, QTableWidgetItem(coord))

        # 更新下拉框
        choose_list = ['全部']
        choose_list += [f"蓝莓_{i}" for i in range(len(berry_list))]
        self.comboBox.clear()
        self.comboBox.addItems(choose_list)

    def on_conf_thres_changed(self, value):
        iou = self.doubleSpinBox_2.value()
        self.ros_thread.set_detection_params(value, iou)

    def on_iou_thres_changed(self, value):
        conf = self.doubleSpinBox.value()
        self.ros_thread.set_detection_params(conf, value)

    def on_param_update_finished(self, success, message):
        if not success:
            QMessageBox.warning(self, "参数设置失败", message)

    def on_start_harvest(self):
        berries = self.ros_thread.get_latest_berries()
        print(f"接收蓝莓信息:{berries}")
        if not berries:
            QMessageBox.information(self, "提示", "当前没有检测到蓝莓")
            return
        # 构造DetectedBerries消息
        from geometry_msgs.msg import Point
        msg = DetectedBerries()
        msg.header.stamp = self.ros_thread.node.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        for b in berries:
            berry = Berry()
            berry.id = b['id']
            berry.position = Point(x=b['x'], y=b['y'], z=b['z'])
            berry.confidence = b['confidence']
            berry.is_ripe = b['is_ripe']
            berry.size = b['size']
            msg.berries.append(berry)
        msg.batch_size = len(msg.berries)
        self.ros_thread.call_path_plan(msg)
        QMessageBox.information(self, "提示", "已发送采摘指令")

    def on_stop_harvest(self):
        self.ros_thread.call_stop_service()
        QMessageBox.information(self, "提示", "已发送紧急停止指令")

    # -------------------- 以下为原有功能的重定义（禁用或改为提示）--------------------
    def open_img(self):
        QMessageBox.information(self, "提示", "图像检测由ROS节点实时处理，无需手动打开图片。")

    def select_model(self):
        QMessageBox.information(self, "提示", "模型切换由ROS节点控制，此处不可用。")

    def vedio_show(self):
        QMessageBox.information(self, "提示", "视频检测由ROS节点实时处理。")

    def camera_show(self):
        QMessageBox.information(self, "提示", "摄像头已由ROS节点自动开启。")

    def save_detect_video(self):
        QMessageBox.information(self, "提示", "视频保存功能未集成，请使用ROS节点功能。")

    def detact_batch_imgs(self):
        QMessageBox.information(self, "提示", "批量检测由ROS节点实时处理。")

    def combox_change(self):
        # 可根据需要实现，目前仅提示
        pass

    def update_show_labels(self, state):
        # 标签显示功能在ROS节点中控制，此处可忽略或发送参数
        pass

    def eventFilter(self, obj, event):
        if obj == self.label_show and event.type() == QEvent.MouseButtonPress:
            if event.button() == Qt.LeftButton:
                QMessageBox.information(self, "提示", "图像由ROS节点实时显示")
                return True
        return super().eventFilter(obj, event)

    def dragEnterEvent(self, event):
        event.ignore()  # 禁用拖拽

    def dropEvent(self, event):
        pass

    def resizeEvent(self, event):
        # 当窗口大小改变时，重新缩放当前图像
        if hasattr(self, 'ros_thread') and self.ros_thread.latest_berries:  # 任意条件触发重绘
            if hasattr(self, 'draw_img') and self.draw_img is not None:
                self.update_image_from_ros(self.draw_img)
        super().resizeEvent(event)

# -------------------- 主程序入口 --------------------
if __name__ == "__main__":
    app = QApplication(sys.argv)
    app.setWindowIcon(QIcon('UI/ui_imgs/icons/目标检测.png'))
    #login_widget = LoginWidget()
    main_window = MainWindow()
    #login_widget.login_success_signal.connect(main_window.show)
    #login_widget.show()
    main_window.show()
    sys.exit(app.exec_())