import os

from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget, QMainWindow
from python_qt_binding.QtWidgets import QToolButton, QMenu, QAction
from python_qt_binding.QtWidgets import QSlider, QLabel
from python_qt_binding.QtCore import Qt, QTimer

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from canopen_msgs.msg import MotorState
from std_msgs.msg import Float64

PI = 3.141592

class CANopenWidget(QMainWindow):
    QOS_REKL5V = QoSProfile(
        reliability=QoSReliabilityPolicy.RELIABLE,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=5,
        durability=QoSDurabilityPolicy.VOLATILE
    )

    def __init__(self, node):
        super().__init__()
        ui_file = os.path.join(
            get_package_share_directory('canopen_rqt_plugin_pkg'),
            'resource',
            'canopen_rqt.ui'
        )
        try:
            loadUi(ui_file, self)
        except Exception as e:
            rclpy.logging.get_logger('CANopenWidget').error(
                f"{e}"
            )
            raise

        self.motor_slider : QSlider = self.findChild(QSlider, 'HorizontalSlider_tab_1_1_2')
        if self.motor_slider:
            self.motor_slider.valueChanged.connect(
                self.on_motor_slider_changed
            )

        self.min_deg_label : QLabel = self.findChild(QLabel, 'Label_tab_1_1_1')
        self.max_deg_label : QLabel = self.findChild(QLabel, 'Label_tab_1_1_3')
        self.current_deg_label : QLabel = self.findChild(QLabel, 'Label_tab_1_1_5')

        self.update_timer = QTimer(self)
        self.update_timer.timeout.connect(self.update_indicators)
        self.update_timer.start(10)

        self.node = node
        self.motor_state_subscriber = self.node.create_subscription(
            MotorState, "motor_states",
            self.motor_state_callback, self.QOS_REKL5V
        )
        self.motor_command_publishers = {}

        self.motor_state = MotorState()
        self.name_to_index = {}
        self.selected_motor_name = None
        self.FIRST = True

    def add_tool_button_component(self, name_list):
        tool_button: QToolButton = self.findChild(QToolButton, 'ToolButton_tab_1_1_4')
        menu = QMenu(tool_button)

        for name in name_list:
            action = QAction(name, tool_button)
            action.triggered.connect(lambda _, n=name: self.on_motor_selected(n))
            menu.addAction(action)

        tool_button.setMenu(menu)

    def add_motor_command_publisher(self, name_list):
        for idx, motor_name in enumerate(name_list):
            publisher = self.node.create_publisher(
                Float64, f'{motor_name}/command', self.QOS_REKL5V
            )
            self.motor_command_publishers[motor_name] = publisher
            self.name_to_index[motor_name] = idx

    def on_motor_selected(self, motor_name):
        tool_button: QToolButton = self.findChild(QToolButton, 'ToolButton_tab_1_1_4')
        if tool_button is None:
            return
        self.selected_motor_name = motor_name
        tool_button.setText(motor_name)

        self.motor_slider.setRange(-40, 40)

        current_degree = int(self.motor_state.position[self.name_to_index[motor_name]] / PI * 180.0)
        self.motor_slider.setValue(current_degree)

        self.min_deg_label.setText("-40")
        self.max_deg_label.setText("40")
        self.current_deg_label.setText(f"{current_degree}")

    def on_motor_slider_changed(self, value:int):
        if self.selected_motor_name is None:
            return
        self.current_deg_label.setText(f"{value}")
        
        msg = Float64()
        value = value / 180.0 * PI
        msg.data = value
        self.motor_command_publishers[self.selected_motor_name].publish(msg)

    def motor_state_callback(self, msg):
        self.motor_state = msg

    def update_indicators(self):
        if self.FIRST:
            self.add_tool_button_component(self.motor_state.name)
            self.add_motor_command_publisher(self.motor_state.name)
            self.FIRST = False

    def shutdown_widget(self):
        self.update_timer.stop()
        self.node.destroy_node()