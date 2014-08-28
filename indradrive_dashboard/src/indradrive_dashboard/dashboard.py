from functools import partial

import rospy
from diagnostic_msgs.msg import DiagnosticArray
from std_msgs.msg import Empty, Int32MultiArray

from rqt_robot_dashboard.dashboard import Dashboard
from rqt_robot_dashboard.widgets import MonitorDashWidget, ConsoleDashWidget
from rqt_robot_dashboard.widgets import MenuDashWidget, IconToolButton
from QtGui import QMessageBox, QAction, QTextEdit, QSizePolicy, QLabel
from python_qt_binding.QtCore import QSize, Signal
from QtCore import pyqtSlot, pyqtSignal

from indradrive_hw_iface.drive_status_ctrl import DriveStatusControl

class IndradriveDashboard(Dashboard):

    def setup(self, context):
        self._drv_ctrl = DriveStatusControl(topic_prefix='/excel_ctrl_man_xeno/', timeout=0.)

        self._enable_widget = EnableWidget(self._drv_ctrl)
        self._secu_stop_widget = SecuStopWidget(self._drv_ctrl)
        self._mode_text = StatusTextWidget()
        self._update_timer = rospy.Timer(rospy.Duration(0.1), self._update_state_cb)

    def get_widgets(self):
        dash_name_label = QLabel()
        dash_name_label.setObjectName('dash_name_label')
        dash_name_label.setText('Indradrive Dash')
        fixed_policy = QSizePolicy(QSizePolicy.Fixed)
        dash_name_label.setSizePolicy(fixed_policy)
        return [[dash_name_label],
                [MonitorDashWidget(self.context), 
                 ConsoleDashWidget(self.context)], 
                [self._enable_widget, self._secu_stop_widget], 
                [self._mode_text]]

    def _update_state_cb(self, te):
        if self._drv_ctrl.is_connected():
            if self._drv_ctrl.is_drive_enabled():
                self._enable_widget.update_state(0)
                self._secu_stop_widget.update_state(0)
            elif self._drv_ctrl.is_drive_ready():
                self._enable_widget.update_state(1)
                self._secu_stop_widget.update_state(1)
            elif self._drv_ctrl.is_drive_halted():
                self._enable_widget.update_state(0)
                self._secu_stop_widget.update_state(1)
            else:
                self._enable_widget.update_state(3)
                self._secu_stop_widget.update_state(3)
        else:
            self._enable_widget.update_state(2)
            self._secu_stop_widget.update_state(2)

    def shutdown_dashboard(self):
        self._drv_ctrl.shutdown()

class EnableWidget(IconToolButton):
    def __init__(self, _drv_ctrl):
        self._drv_ctrl = _drv_ctrl

        self._off_icon = ['bg-red.svg', 'ic-motors.svg']
        self._on_icon = ['bg-green.svg', 'ic-motors.svg']
        self._stale_icon = ['bg-grey.svg', 'ic-motors.svg', 'ol-stale-badge.svg']
        self._error_icon = ['bg-red.svg', 'ic-motors.svg', 'ol-err-badge.svg']

        icons = [self._on_icon, self._off_icon, self._stale_icon, self._error_icon]
        super(EnableWidget, self).__init__("enable_button", icons=icons)
        self.setFixedSize(QSize(40,40))

        super(EnableWidget, self).update_state(2)
        self.setToolTip("Drive: Stale")

        self.clicked.connect(self.toggle)


    def update_state(self, state):
        if state is not super(EnableWidget, self).state:
            super(EnableWidget, self).update_state(state)
            if state is 0:
                self.setToolTip("Drive: Enabled (AF)")
            elif state is 1:
                self.setToolTip("Drive: Disabled (Ab)")
            elif state is 2:
                self.setToolTip("Drive: Stale")
            else:
                self.setToolTip("Drive: UNKNOWN/ERROR")

    def toggle(self):
        if super(EnableWidget, self).state is 1:
            self._drv_ctrl.enable_drive(timeout=0.)
        else:
            self._drv_ctrl.disable_drive(timeout=0.)

    def close(self):
        pass

class SecuStopWidget(IconToolButton):
    def __init__(self, _drv_ctrl):
        self._drv_ctrl = _drv_ctrl

        self._off_icon = ['bg-red.svg', 'ic-runstop-on.svg']
        self._on_icon = ['bg-green.svg', 'ic-runstop-off.svg']
        self._stale_icon = ['bg-grey.svg', 'ic-runstop-on.svg', 'ol-stale-badge.svg']
        self._error_icon = ['bg-red.svg', 'ic-runstop.svg', 'ol-err-badge.svg']

        icons = [self._on_icon, self._off_icon, self._stale_icon, self._error_icon]
        super(SecuStopWidget, self).__init__("secu_stop_button", icons=icons)
        self.setFixedSize(QSize(40,40))

        super(SecuStopWidget, self).update_state(2)
        self.setToolTip("Drive: Stale")

        self.clicked.connect(self.toggle)


    def update_state(self, state):
        if state is not super(SecuStopWidget, self).state:
            super(SecuStopWidget, self).update_state(state)
            if state is 0:
                self.setToolTip("SStop: Enabled (AF)")
            elif state is 1:
                self.setToolTip("SStop: Halted (AH)")
            elif state is 2:
                self.setToolTip("SStop: Stale")
            else:
                self.setToolTip("SStop: UNKNOWN/ERROR")

    def toggle(self):
        if super(SecuStopWidget, self).state is 1:
            self._drv_ctrl.enable_drive(timeout=0.)
        else:
            self._drv_ctrl.halt_drive(timeout=0.)

    def close(self):
        pass

class StatusTextWidget(QTextEdit):
    text_changed = Signal()
    def __init__(self):
        super(QTextEdit, self).__init__()
        self.setObjectName("status_text")
        self.setReadOnly(True)
        self.setText('STALE')
        self.setFixedSize(QSize(180,27))
        self.text_changed.connect(self._update_state)
        self.__state = 'STALE'

    def update_state(self, state):
        if type(state) is str:
            self.__state = state
            self.text_changed.emit()
        else:
            raise TypeError('state must be a string')

    def _update_state(self):
        self.setText(self.__state)

    def state(self):
        return self.__state
