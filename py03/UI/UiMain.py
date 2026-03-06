# -*- coding: utf-8 -*-

from PyQt5 import QtCore, QtGui, QtWidgets

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(1280, 850)
        
        # --- Icons ---
        icon = QtGui.QIcon()
        icon.addPixmap(QtGui.QPixmap(":/icons/ui_imgs/icons/目标检测.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        MainWindow.setWindowIcon(icon)
        
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        
        # --- Main Layout (HBox: Sidebar | Content) ---
        self.horizontalLayoutMain = QtWidgets.QHBoxLayout(self.centralwidget)
        self.horizontalLayoutMain.setContentsMargins(0, 0, 0, 0)
        self.horizontalLayoutMain.setSpacing(0)
        self.horizontalLayoutMain.setObjectName("horizontalLayoutMain")

        # ================== 1. Left Sidebar (Navigation) ==================
        self.sidebarFrame = QtWidgets.QFrame(self.centralwidget)
        self.sidebarFrame.setMinimumWidth(90)
        self.sidebarFrame.setMaximumWidth(90)
        self.sidebarFrame.setObjectName("sidebarFrame")
        self.verticalLayoutSidebar = QtWidgets.QVBoxLayout(self.sidebarFrame)
        self.verticalLayoutSidebar.setContentsMargins(10, 20, 10, 20)
        self.verticalLayoutSidebar.setSpacing(20)
        self.verticalLayoutSidebar.setObjectName("verticalLayoutSidebar")

        # Sidebar Buttons (Icon + Text vertical)
        self.PicBtn = self.create_sidebar_button("PicBtn", ":/icons/ui_imgs/icons/img.png", "🍎 图片检测")
        self.verticalLayoutSidebar.addWidget(self.PicBtn)

        self.FilesBtn = self.create_sidebar_button("FilesBtn", ":/icons/ui_imgs/icons/folder.png", "🍌 批量检测")
        self.verticalLayoutSidebar.addWidget(self.FilesBtn)

        self.VideoBtn = self.create_sidebar_button("VideoBtn", ":/icons/ui_imgs/icons/video.png", "🍇 视频检测")
        self.verticalLayoutSidebar.addWidget(self.VideoBtn)

        self.CapBtn = self.create_sidebar_button("CapBtn", ":/icons/ui_imgs/icons/camera.png", "🎥 实时检测")
        self.verticalLayoutSidebar.addWidget(self.CapBtn)

        # Spacer to push bottom buttons down
        spacerItem = QtWidgets.QSpacerItem(20, 40, QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Expanding)
        self.verticalLayoutSidebar.addItem(spacerItem)

        self.SaveBtn = self.create_sidebar_button("SaveBtn", ":/icons/ui_imgs/icons/保存.png", "💾 保存结果")
        self.verticalLayoutSidebar.addWidget(self.SaveBtn)

        self.ExitBtn = self.create_sidebar_button("ExitBtn", ":/icons/ui_imgs/icons/退出.png", "🚪 退出系统")
        self.verticalLayoutSidebar.addWidget(self.ExitBtn)

        self.horizontalLayoutMain.addWidget(self.sidebarFrame)

        # ================== 2. Main Content Splitter ==================
        self.mainSplitter = QtWidgets.QSplitter(QtCore.Qt.Horizontal)
        self.mainSplitter.setObjectName("mainSplitter")

        # ------ 2.1 Center Area (Image + Table) ------
        self.centerFrame = QtWidgets.QFrame(self.mainSplitter)
        self.centerFrame.setFrameShape(QtWidgets.QFrame.NoFrame)
        self.centerFrame.setObjectName("centerFrame")
        self.verticalLayoutCenter = QtWidgets.QVBoxLayout(self.centerFrame)
        self.verticalLayoutCenter.setContentsMargins(20, 20, 20, 20)
        self.verticalLayoutCenter.setSpacing(15)

        # Title / Header inside content
        self.headerLayout = QtWidgets.QHBoxLayout()
        self.headerLayout.addStretch(1) # Left spacer
        
        self.label_3 = QtWidgets.QLabel(self.centerFrame)
        font = QtGui.QFont()
        font.setFamily("Microsoft YaHei")
        font.setPointSize(24) # Increased font size
        font.setBold(True)
        self.label_3.setFont(font)
        self.label_3.setAlignment(QtCore.Qt.AlignCenter)
        self.label_3.setObjectName("label_3")
        self.headerLayout.addWidget(self.label_3)
        
        self.headerLayout.addStretch(1) # Right spacer
        
        # Developer Info (Tiny) - kept here but maybe not perfectly aligned right if stretched
        self.label_2 = QtWidgets.QLabel(self.centerFrame)
        self.label_2.setObjectName("label_2")
        self.headerLayout.addWidget(self.label_2, 0, QtCore.Qt.AlignBottom)
        
        self.verticalLayoutCenter.addLayout(self.headerLayout)

        # Image Display Area
        self.frame_2 = QtWidgets.QFrame(self.centerFrame)
        self.frame_2.setFrameShape(QtWidgets.QFrame.NoFrame)
        self.frame_2.setObjectName("frame_2")
        self.verticalLayoutImage = QtWidgets.QVBoxLayout(self.frame_2)
        self.verticalLayoutImage.setContentsMargins(0, 0, 0, 0)
        
        self.label_show = QtWidgets.QLabel(self.frame_2)
        self.label_show.setStyleSheet("background-color: #F1F5F9; border-radius: 12px;")
        self.label_show.setAlignment(QtCore.Qt.AlignCenter)
        self.label_show.setText("拖拽文件到此处或点击左侧按钮打开")
        self.label_show.setObjectName("label_show")
        self.label_show.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        self.verticalLayoutImage.addWidget(self.label_show)
        
        # Hidden LineEdits compatibility
        self.PiclineEdit = QtWidgets.QLineEdit(self.frame_2); self.PiclineEdit.setVisible(False)
        self.VideolineEdit = QtWidgets.QLineEdit(self.frame_2); self.VideolineEdit.setVisible(False)
        self.CaplineEdit = QtWidgets.QLineEdit(self.frame_2); self.CaplineEdit.setVisible(False)
        self.verticalLayoutImage.addWidget(self.PiclineEdit)
        self.verticalLayoutImage.addWidget(self.VideolineEdit)
        self.verticalLayoutImage.addWidget(self.CaplineEdit)

        self.verticalLayoutCenter.addWidget(self.frame_2, stretch=4) # Increased stretch factor

        # Table Area (Collapsible/Resizing)
        self.frame_3 = QtWidgets.QFrame(self.centerFrame)
        self.frame_3.setObjectName("frame_3")
        self.frame_3.setMinimumHeight(200) # Reduced min height for table
        self.verticalLayoutTable = QtWidgets.QVBoxLayout(self.frame_3)
        self.verticalLayoutTable.setContentsMargins(0, 0, 0, 0)
        
        self.groupBox_3 = QtWidgets.QGroupBox(self.frame_3)
        self.groupBox_3.setObjectName("groupBox_3")
        self.verticalLayoutGroup3 = QtWidgets.QVBoxLayout(self.groupBox_3)
        
        self.tableWidget = QtWidgets.QTableWidget(self.groupBox_3)
        self.tableWidget.setObjectName("tableWidget")
        self.tableWidget.setColumnCount(5)
        for i in range(5):
            self.tableWidget.setHorizontalHeaderItem(i, QtWidgets.QTableWidgetItem())
            
        self.verticalLayoutGroup3.addWidget(self.tableWidget)
        self.verticalLayoutTable.addWidget(self.groupBox_3)
        
        self.verticalLayoutCenter.addWidget(self.frame_3, stretch=1) # Reduced stretch factor

        # ------ 2.2 Right Panel (Settings & Stats) ------
        self.rightFrame = QtWidgets.QFrame(self.mainSplitter)
        self.rightFrame.setMinimumWidth(300)
        self.rightFrame.setMaximumWidth(400)
        self.rightFrame.setObjectName("rightFrame")
        self.verticalLayoutRight = QtWidgets.QVBoxLayout(self.rightFrame)
        self.verticalLayoutRight.setContentsMargins(10, 20, 20, 20)
        self.verticalLayoutRight.setSpacing(20)

        # Settings Group (Swapped Position)
        self.groupBox = QtWidgets.QGroupBox(self.rightFrame)
        self.groupBox.setObjectName("groupBox")
        self.verticalLayoutSettings = QtWidgets.QVBoxLayout(self.groupBox)
        self.verticalLayoutSettings.setSpacing(15)

        # Model Selection Row
        h_model = QtWidgets.QHBoxLayout()
        label_model = QtWidgets.QLabel("模型选择:")
        label_model.setObjectName("label_model")
        self.lineEdit_model = QtWidgets.QLineEdit()
        self.lineEdit_model.setReadOnly(True)
        self.lineEdit_model.setPlaceholderText("请选择模型文件...")
        self.lineEdit_model.setObjectName("lineEdit_model")
        
        self.btn_select_model = QtWidgets.QPushButton()
        self.btn_select_model.setIcon(QtGui.QIcon(":/icons/ui_imgs/icons/folder.png"))
        self.btn_select_model.setToolTip("选择模型文件")
        self.btn_select_model.setFixedSize(30, 30)
        self.btn_select_model.setObjectName("btn_select_model")
        
        h_model.addWidget(label_model)
        h_model.addWidget(self.lineEdit_model)
        h_model.addWidget(self.btn_select_model)
        self.verticalLayoutSettings.addLayout(h_model)

        self.add_setting_row(self.verticalLayoutSettings, "label_14", "置信度阈值:", "doubleSpinBox")
        self.add_setting_row(self.verticalLayoutSettings, "label_15", "交并比阈值:", "doubleSpinBox_2")
        
        self.checkBox = QtWidgets.QCheckBox(self.groupBox)
        self.checkBox.setObjectName("checkBox")
        self.verticalLayoutSettings.addWidget(self.checkBox)

        self.verticalLayoutRight.addWidget(self.groupBox)

        # Stats Group (Swapped Position)
        self.groupBox_2 = QtWidgets.QGroupBox(self.rightFrame)
        self.groupBox_2.setObjectName("groupBox_2")
        self.gridLayoutResults = QtWidgets.QGridLayout(self.groupBox_2)
        self.gridLayoutResults.setVerticalSpacing(15)
        
        # Stats Items
        self.add_stat_row(self.gridLayoutResults, 0, "label_10", "用时:", "time_lb")
        self.add_stat_row(self.gridLayoutResults, 1, "label_5", "目标选择:", "comboBox", is_widget=True)
        self.add_stat_row(self.gridLayoutResults, 2, "label", "目标数目:", "label_nums")
        self.add_stat_row(self.gridLayoutResults, 3, "label_13", "类型:", "type_lb")
        self.add_stat_row(self.gridLayoutResults, 4, "label_11", "置信度:", "label_conf")

        # Coords Frame
        self.frame_6 = QtWidgets.QFrame(self.groupBox_2)
        self.frame_6.setObjectName("frame_6")
        self.gridLayoutCoords = QtWidgets.QGridLayout(self.frame_6)
        self.add_coord_item(self.gridLayoutCoords, 0, 0, "label_6", "xmin:", "label_xmin")
        self.add_coord_item(self.gridLayoutCoords, 0, 1, "label_8", "ymin:", "label_ymin")
        self.add_coord_item(self.gridLayoutCoords, 1, 0, "label_7", "xmax:", "label_xmax")
        self.add_coord_item(self.gridLayoutCoords, 1, 1, "label_9", "ymax:", "label_ymax")
        self.gridLayoutResults.addWidget(self.frame_6, 5, 0, 1, 2)

        self.verticalLayoutRight.addWidget(self.groupBox_2)
        
        # New Analysis Group (Bottom Right)
        self.groupBox_5 = QtWidgets.QGroupBox(self.rightFrame)
        self.groupBox_5.setTitle("检测分析") # Set default title
        self.groupBox_5.setObjectName("groupBox_5")
        self.verticalLayoutAnalysis = QtWidgets.QVBoxLayout(self.groupBox_5)
        self.textEdit_analysis = QtWidgets.QTextEdit(self.groupBox_5)
        self.textEdit_analysis.setObjectName("textEdit_analysis")
        self.textEdit_analysis.setReadOnly(True)
        self.textEdit_analysis.setPlaceholderText("检测结果分析将显示在这里...")
        self.textEdit_analysis.setMinimumHeight(100) # Ensure visibility
        self.verticalLayoutAnalysis.addWidget(self.textEdit_analysis)
        self.verticalLayoutRight.addWidget(self.groupBox_5, 1) # Add stretch factor 1 to take remaining space

        # Removed label_12 (Org Name) as requested
        
        self.mainSplitter.addWidget(self.centerFrame)
        self.mainSplitter.addWidget(self.rightFrame)
        self.mainSplitter.setStretchFactor(0, 1) # Center expands
        self.mainSplitter.setStretchFactor(1, 0) # Right fixed
        
        self.horizontalLayoutMain.addWidget(self.mainSplitter)

        MainWindow.setCentralWidget(self.centralwidget)
        self.statusbar = QtWidgets.QStatusBar(MainWindow)
        self.statusbar.setObjectName("statusbar")
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def create_sidebar_button(self, name, icon_path, text):
        btn = QtWidgets.QPushButton()
        btn.setObjectName(name)
        btn.setMinimumSize(60, 60)
        btn.setMaximumSize(60, 60)
        # Custom styling for sidebar buttons will be in CSS
        # But we set icon here
        icon = QtGui.QIcon()
        icon.addPixmap(QtGui.QPixmap(icon_path), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        btn.setIcon(icon)
        btn.setIconSize(QtCore.QSize(32, 32))
        btn.setToolTip(text)
        return btn

    def add_stat_row(self, layout, row, label_name, label_text, widget_name, is_widget=False):
        lbl = QtWidgets.QLabel()
        lbl.setObjectName(label_name)
        lbl.setText(label_text)
        layout.addWidget(lbl, row, 0)
        
        if is_widget:
            widget = QtWidgets.QComboBox()
            widget.setObjectName(widget_name)
            layout.addWidget(widget, row, 1)
            setattr(self, widget_name, widget)
        else:
            val_lbl = QtWidgets.QLabel()
            val_lbl.setObjectName(widget_name)
            layout.addWidget(val_lbl, row, 1)
            setattr(self, widget_name, val_lbl)

    def add_coord_item(self, layout, row, col, label_name, label_text, val_name):
        container = QtWidgets.QHBoxLayout()
        lbl = QtWidgets.QLabel(label_text)
        lbl.setObjectName(label_name)
        val = QtWidgets.QLabel()
        val.setObjectName(val_name)
        container.addWidget(lbl)
        container.addWidget(val)
        layout.addLayout(container, row, col)
        setattr(self, val_name, val)
        setattr(self, label_name, lbl)

    def add_setting_row(self, layout, label_name, label_text, widget_name):
        h = QtWidgets.QHBoxLayout()
        lbl = QtWidgets.QLabel(label_text)
        lbl.setObjectName(label_name)
        widget = QtWidgets.QDoubleSpinBox()
        widget.setObjectName(widget_name)
        widget.setDecimals(2)
        h.addWidget(lbl)
        h.addWidget(widget)
        layout.addLayout(h)
        setattr(self, widget_name, widget)
        setattr(self, label_name, lbl)

    def retranslateUi(self, MainWindow):
        from utils import config
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", config.SYSTEM_NAME))
        self.label_3.setText(_translate("MainWindow", config.SYSTEM_NAME))
        self.label_2.setText(_translate("MainWindow", config.DEVELOPER_NAME))
        
        self.groupBox_3.setTitle(_translate("MainWindow", "📊 详细数据"))
        headers = ["序号", "文件路径", "类别", "置信度", "坐标位置"]
        for i, h in enumerate(headers):
            self.tableWidget.horizontalHeaderItem(i).setText(_translate("MainWindow", h))
            
        self.groupBox_2.setTitle(_translate("MainWindow", "🍎 识别结果"))
        self.groupBox.setTitle(_translate("MainWindow", "⚙️ 参数设置"))
        self.groupBox_5.setTitle(_translate("MainWindow", "📈 结果分析")) # Translate new group box
        self.checkBox.setText(_translate("MainWindow", "🏷️ 显示标签"))
        
        # Stat Labels are set in add_stat_row text
        # But we need to ensure object access if needed.
        
import UI.ui_sources_rc
