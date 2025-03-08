# -*- coding: utf-8 -*-

################################################################################
## Form generated from reading UI file 'form.ui'
##
## Created by: Qt User Interface Compiler version 6.8.2
##
## WARNING! All changes made in this file will be lost when recompiling UI file!
################################################################################

from PySide6.QtCore import (QCoreApplication, QDate, QDateTime, QLocale,
    QMetaObject, QObject, QPoint, QRect,
    QSize, QTime, QUrl, Qt)
from PySide6.QtGui import (QAction, QBrush, QColor, QConicalGradient,
    QCursor, QFont, QFontDatabase, QGradient,
    QIcon, QImage, QKeySequence, QLinearGradient,
    QPainter, QPalette, QPixmap, QRadialGradient,
    QTransform)
from PySide6.QtWidgets import (QApplication, QFrame, QHBoxLayout, QLCDNumber,
    QLabel, QLayout, QMainWindow, QMenu,
    QMenuBar, QPushButton, QSizePolicy, QStatusBar,
    QVBoxLayout, QWidget)

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        if not MainWindow.objectName():
            MainWindow.setObjectName(u"MainWindow")
        MainWindow.resize(1058, 547)
        self.actionv1_0 = QAction(MainWindow)
        self.actionv1_0.setObjectName(u"actionv1_0")
        self.actionhttps_github_com_Tang_JingWei = QAction(MainWindow)
        self.actionhttps_github_com_Tang_JingWei.setObjectName(u"actionhttps_github_com_Tang_JingWei")
        self.centralwidget = QWidget(MainWindow)
        self.centralwidget.setObjectName(u"centralwidget")
        self.trackButton = QPushButton(self.centralwidget)
        self.trackButton.setObjectName(u"trackButton")
        self.trackButton.setGeometry(QRect(870, 320, 161, 71))
        font = QFont()
        font.setPointSize(20)
        font.setBold(True)
        self.trackButton.setFont(font)
        self.imageLabel = QLabel(self.centralwidget)
        self.imageLabel.setObjectName(u"imageLabel")
        self.imageLabel.setGeometry(QRect(220, 110, 640, 360))
        font1 = QFont()
        font1.setPointSize(24)
        self.imageLabel.setFont(font1)
        self.imageLabel.setFrameShape(QFrame.WinPanel)
        self.imageLabel.setFrameShadow(QFrame.Sunken)
        self.imageLabel.setTextFormat(Qt.RichText)
        self.imageLabel.setAlignment(Qt.AlignCenter)
        self.clickButton = QPushButton(self.centralwidget)
        self.clickButton.setObjectName(u"clickButton")
        self.clickButton.setGeometry(QRect(870, 400, 161, 71))
        self.clickButton.setFont(font)
        self.infosLabel = QLabel(self.centralwidget)
        self.infosLabel.setObjectName(u"infosLabel")
        self.infosLabel.setGeometry(QRect(20, 100, 191, 41))
        font2 = QFont()
        font2.setPointSize(15)
        font2.setBold(True)
        self.infosLabel.setFont(font2)
        self.infosLabel.setAlignment(Qt.AlignCenter)
        self.verticalLayoutWidget = QWidget(self.centralwidget)
        self.verticalLayoutWidget.setObjectName(u"verticalLayoutWidget")
        self.verticalLayoutWidget.setGeometry(QRect(20, 140, 190, 319))
        self.flyinfos = QVBoxLayout(self.verticalLayoutWidget)
        self.flyinfos.setObjectName(u"flyinfos")
        self.flyinfos.setSizeConstraint(QLayout.SetDefaultConstraint)
        self.flyinfos.setContentsMargins(0, 0, 0, 0)
        self.line_3 = QFrame(self.verticalLayoutWidget)
        self.line_3.setObjectName(u"line_3")
        self.line_3.setFrameShape(QFrame.Shape.HLine)
        self.line_3.setFrameShadow(QFrame.Shadow.Sunken)

        self.flyinfos.addWidget(self.line_3)

        self.info_vel = QLabel(self.verticalLayoutWidget)
        self.info_vel.setObjectName(u"info_vel")
        self.info_vel.setEnabled(True)
        font3 = QFont()
        font3.setPointSize(13)
        self.info_vel.setFont(font3)
        self.info_vel.setLayoutDirection(Qt.LeftToRight)
        self.info_vel.setFrameShadow(QFrame.Plain)
        self.info_vel.setAlignment(Qt.AlignCenter)
        self.info_vel.setMargin(0)

        self.flyinfos.addWidget(self.info_vel)

        self.vel_info = QHBoxLayout()
        self.vel_info.setObjectName(u"vel_info")
        self.vel_x = QLabel(self.verticalLayoutWidget)
        self.vel_x.setObjectName(u"vel_x")
        font4 = QFont()
        font4.setPointSize(12)
        self.vel_x.setFont(font4)
        self.vel_x.setAlignment(Qt.AlignCenter)

        self.vel_info.addWidget(self.vel_x)

        self.vel_y = QLabel(self.verticalLayoutWidget)
        self.vel_y.setObjectName(u"vel_y")
        self.vel_y.setFont(font4)
        self.vel_y.setAlignment(Qt.AlignCenter)

        self.vel_info.addWidget(self.vel_y)

        self.vel_z = QLabel(self.verticalLayoutWidget)
        self.vel_z.setObjectName(u"vel_z")
        self.vel_z.setFont(font4)
        self.vel_z.setAlignment(Qt.AlignCenter)

        self.vel_info.addWidget(self.vel_z)


        self.flyinfos.addLayout(self.vel_info)

        self.line = QFrame(self.verticalLayoutWidget)
        self.line.setObjectName(u"line")
        self.line.setFrameShape(QFrame.Shape.HLine)
        self.line.setFrameShadow(QFrame.Shadow.Sunken)

        self.flyinfos.addWidget(self.line)

        self.info_att = QLabel(self.verticalLayoutWidget)
        self.info_att.setObjectName(u"info_att")
        self.info_att.setFont(font3)
        self.info_att.setLayoutDirection(Qt.LeftToRight)
        self.info_att.setFrameShadow(QFrame.Plain)
        self.info_att.setAlignment(Qt.AlignCenter)
        self.info_att.setMargin(0)

        self.flyinfos.addWidget(self.info_att)

        self.att_info = QHBoxLayout()
        self.att_info.setObjectName(u"att_info")
        self.att_r = QLabel(self.verticalLayoutWidget)
        self.att_r.setObjectName(u"att_r")
        self.att_r.setFont(font4)
        self.att_r.setAlignment(Qt.AlignCenter)

        self.att_info.addWidget(self.att_r)

        self.att_p = QLabel(self.verticalLayoutWidget)
        self.att_p.setObjectName(u"att_p")
        self.att_p.setFont(font4)
        self.att_p.setAlignment(Qt.AlignCenter)

        self.att_info.addWidget(self.att_p)

        self.att_y = QLabel(self.verticalLayoutWidget)
        self.att_y.setObjectName(u"att_y")
        self.att_y.setFont(font4)
        self.att_y.setAlignment(Qt.AlignCenter)

        self.att_info.addWidget(self.att_y)


        self.flyinfos.addLayout(self.att_info)

        self.line_2 = QFrame(self.verticalLayoutWidget)
        self.line_2.setObjectName(u"line_2")
        self.line_2.setFrameShape(QFrame.Shape.HLine)
        self.line_2.setFrameShadow(QFrame.Shadow.Sunken)

        self.flyinfos.addWidget(self.line_2)

        self.info_alt = QLabel(self.verticalLayoutWidget)
        self.info_alt.setObjectName(u"info_alt")
        self.info_alt.setFont(font3)
        self.info_alt.setLayoutDirection(Qt.LeftToRight)
        self.info_alt.setFrameShadow(QFrame.Plain)
        self.info_alt.setAlignment(Qt.AlignCenter)
        self.info_alt.setMargin(0)

        self.flyinfos.addWidget(self.info_alt)

        self.alt_info = QHBoxLayout()
        self.alt_info.setObjectName(u"alt_info")
        self.alt = QLabel(self.verticalLayoutWidget)
        self.alt.setObjectName(u"alt")
        self.alt.setFont(font4)
        self.alt.setAlignment(Qt.AlignCenter)

        self.alt_info.addWidget(self.alt)


        self.flyinfos.addLayout(self.alt_info)

        self.line_4 = QFrame(self.verticalLayoutWidget)
        self.line_4.setObjectName(u"line_4")
        self.line_4.setFrameShape(QFrame.Shape.HLine)
        self.line_4.setFrameShadow(QFrame.Shadow.Sunken)

        self.flyinfos.addWidget(self.line_4)

        self.logoLabel = QLabel(self.centralwidget)
        self.logoLabel.setObjectName(u"logoLabel")
        self.logoLabel.setGeometry(QRect(390, 10, 331, 91))
        font5 = QFont()
        font5.setFamilies([u"dsrom10"])
        font5.setPointSize(51)
        font5.setItalic(False)
        self.logoLabel.setFont(font5)
        self.logoLabel.setAlignment(Qt.AlignCenter)
        self.fpsLCD = QLCDNumber(self.centralwidget)
        self.fpsLCD.setObjectName(u"fpsLCD")
        self.fpsLCD.setGeometry(QRect(960, 110, 51, 51))
        font6 = QFont()
        font6.setPointSize(10)
        font6.setBold(True)
        self.fpsLCD.setFont(font6)
        self.fpsLCD.setLayoutDirection(Qt.LeftToRight)
        self.fpsLCD.setDigitCount(2)
        self.fpslabel = QLabel(self.centralwidget)
        self.fpslabel.setObjectName(u"fpslabel")
        self.fpslabel.setGeometry(QRect(880, 110, 81, 51))
        font7 = QFont()
        font7.setPointSize(24)
        font7.setBold(True)
        self.fpslabel.setFont(font7)
        self.fpslabel.setAlignment(Qt.AlignCenter)
        self.verticalLayoutWidget_2 = QWidget(self.centralwidget)
        self.verticalLayoutWidget_2.setObjectName(u"verticalLayoutWidget_2")
        self.verticalLayoutWidget_2.setGeometry(QRect(870, 170, 161, 141))
        self.targetinfo = QVBoxLayout(self.verticalLayoutWidget_2)
        self.targetinfo.setObjectName(u"targetinfo")
        self.targetinfo.setContentsMargins(0, 0, 0, 0)
        self.detect_flag = QHBoxLayout()
        self.detect_flag.setObjectName(u"detect_flag")
        self.detectLabel = QLabel(self.verticalLayoutWidget_2)
        self.detectLabel.setObjectName(u"detectLabel")
        font8 = QFont()
        font8.setPointSize(11)
        self.detectLabel.setFont(font8)
        self.detectLabel.setAlignment(Qt.AlignCenter)

        self.detect_flag.addWidget(self.detectLabel)

        self.detect_val = QLabel(self.verticalLayoutWidget_2)
        self.detect_val.setObjectName(u"detect_val")
        self.detect_val.setFont(font4)
        self.detect_val.setAlignment(Qt.AlignCenter)

        self.detect_flag.addWidget(self.detect_val)


        self.targetinfo.addLayout(self.detect_flag)

        self.cx = QHBoxLayout()
        self.cx.setObjectName(u"cx")
        self.cxLabel = QLabel(self.verticalLayoutWidget_2)
        self.cxLabel.setObjectName(u"cxLabel")
        self.cxLabel.setFont(font4)
        self.cxLabel.setAlignment(Qt.AlignCenter)

        self.cx.addWidget(self.cxLabel)

        self.cx_val = QLabel(self.verticalLayoutWidget_2)
        self.cx_val.setObjectName(u"cx_val")
        self.cx_val.setFont(font4)
        self.cx_val.setAlignment(Qt.AlignCenter)

        self.cx.addWidget(self.cx_val)


        self.targetinfo.addLayout(self.cx)

        self.cy = QHBoxLayout()
        self.cy.setObjectName(u"cy")
        self.cyLabel = QLabel(self.verticalLayoutWidget_2)
        self.cyLabel.setObjectName(u"cyLabel")
        self.cyLabel.setFont(font4)
        self.cyLabel.setAlignment(Qt.AlignCenter)

        self.cy.addWidget(self.cyLabel)

        self.cy_val = QLabel(self.verticalLayoutWidget_2)
        self.cy_val.setObjectName(u"cy_val")
        self.cy_val.setFont(font4)
        self.cy_val.setAlignment(Qt.AlignCenter)

        self.cy.addWidget(self.cy_val)


        self.targetinfo.addLayout(self.cy)

        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QMenuBar(MainWindow)
        self.menubar.setObjectName(u"menubar")
        self.menubar.setGeometry(QRect(0, 0, 1058, 24))
        self.menu = QMenu(self.menubar)
        self.menu.setObjectName(u"menu")
        self.menu_2 = QMenu(self.menu)
        self.menu_2.setObjectName(u"menu_2")
        self.menu_3 = QMenu(self.menu)
        self.menu_3.setObjectName(u"menu_3")
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QStatusBar(MainWindow)
        self.statusbar.setObjectName(u"statusbar")
        MainWindow.setStatusBar(self.statusbar)

        self.menubar.addAction(self.menu.menuAction())
        self.menu.addAction(self.menu_3.menuAction())
        self.menu.addAction(self.menu_2.menuAction())
        self.menu_2.addAction(self.actionv1_0)
        self.menu_3.addAction(self.actionhttps_github_com_Tang_JingWei)

        self.retranslateUi(MainWindow)

        QMetaObject.connectSlotsByName(MainWindow)
    # setupUi

    def retranslateUi(self, MainWindow):
        MainWindow.setWindowTitle(QCoreApplication.translate("MainWindow", u"MainWindow", None))
        self.actionv1_0.setText(QCoreApplication.translate("MainWindow", u"v1.0", None))
        self.actionhttps_github_com_Tang_JingWei.setText(QCoreApplication.translate("MainWindow", u"https://github.com/Tang-JingWei", None))
        self.trackButton.setText(QCoreApplication.translate("MainWindow", u"\u9501\u5b9a", None))
        self.imageLabel.setText(QCoreApplication.translate("MainWindow", u"\u5b9e\u65f6\u753b\u9762", None))
        self.clickButton.setText(QCoreApplication.translate("MainWindow", u"\u6f2b\u6e38", None))
        self.infosLabel.setText(QCoreApplication.translate("MainWindow", u"\u98de\u884c\u4fe1\u606f\u9762\u677f", None))
        self.info_vel.setText(QCoreApplication.translate("MainWindow", u"\u901f\u5ea6 [x y z]  [m/s]", None))
        self.vel_x.setText(QCoreApplication.translate("MainWindow", u"0.0", None))
        self.vel_y.setText(QCoreApplication.translate("MainWindow", u"0.0", None))
        self.vel_z.setText(QCoreApplication.translate("MainWindow", u"0.0", None))
        self.info_att.setText(QCoreApplication.translate("MainWindow", u"\u59ff\u6001\u901f\u7387 [r p y]  [deg/s]", None))
        self.att_r.setText(QCoreApplication.translate("MainWindow", u"0.0", None))
        self.att_p.setText(QCoreApplication.translate("MainWindow", u"0.0", None))
        self.att_y.setText(QCoreApplication.translate("MainWindow", u"0.0", None))
        self.info_alt.setText(QCoreApplication.translate("MainWindow", u"\u79bb\u5730\u9ad8\u5ea6 [h]  [m]", None))
        self.alt.setText(QCoreApplication.translate("MainWindow", u"0.0", None))
        self.logoLabel.setText(QCoreApplication.translate("MainWindow", u"AirTrack", None))
        self.fpslabel.setText(QCoreApplication.translate("MainWindow", u"FPS", None))
        self.detectLabel.setText(QCoreApplication.translate("MainWindow", u"\u9501\u5b9a\u72b6\u6001 :", None))
        self.detect_val.setText(QCoreApplication.translate("MainWindow", u"0", None))
        self.cxLabel.setText(QCoreApplication.translate("MainWindow", u"\u5750\u6807 X :", None))
        self.cx_val.setText(QCoreApplication.translate("MainWindow", u"0 %", None))
        self.cyLabel.setText(QCoreApplication.translate("MainWindow", u"\u5750\u6807 Y :", None))
        self.cy_val.setText(QCoreApplication.translate("MainWindow", u"0 %", None))
        self.menu.setTitle(QCoreApplication.translate("MainWindow", u"\u5173\u4e8e", None))
        self.menu_2.setTitle(QCoreApplication.translate("MainWindow", u"\u7248\u672c", None))
        self.menu_3.setTitle(QCoreApplication.translate("MainWindow", u"\u4f5c\u8005", None))
    # retranslateUi

