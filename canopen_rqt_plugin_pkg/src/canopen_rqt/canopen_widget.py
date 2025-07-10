import os
import pyqtgraph as pg
from itertools import cycle

from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QMainWindow, QWidget, QTabWidget, QGroupBox
from python_qt_binding.QtWidgets import QVBoxLayout, QHBoxLayout
from python_qt_binding.QtWidgets import QLabel, QMenu, QAction, QStyle
from python_qt_binding.QtWidgets import QSlider, QToolButton, QPushButton, QPlainTextEdit
from python_qt_binding.QtCore import Qt, QTimer

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from canopen_msgs.msg import MotorState
from std_msgs.msg import Float64
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

PI = 3.141592653589793

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

        self.node = node

        self.update_timer = QTimer(self)
        self.update_timer.timeout.connect(self.update_indicators)
        self.update_timer.start(10)

        self.motor_state_subscriber = self.node.create_subscription(
            MotorState, 'motor_states',
            self.motor_state_callback, self.QOS_REKL5V
        )
        self.joint_trajectory_publisher = self.node.create_publisher(
            JointTrajectory, 'joint_trajectory', self.QOS_REKL5V
        )
        self.motor_command_publishers = {}
        self.name_to_index = {}

        self.motor_state = None
        self.positions = []
        self.torques = []
        self.times = []
        self.selected_motor_name = None
        self.relative_value = 0

        self.initialize_ui()

    def initialize_ui(self):
        self.q_tab_widget : QTabWidget = self.findChild(QTabWidget, 'TabWidget')
        if self.q_tab_widget is None:
            self.node.get_logger().error("No such a name 'TabWidget'.")
            return
        
        style = self.style()
        refresh_icon = style.standardIcon(QStyle.SP_BrowserReload)

        first_tab = QWidget()
        first_tab_layout = QVBoxLayout(first_tab)

        self.reset_button = QPushButton()
        self.reset_button.setFixedWidth(40)        
        self.reset_button.setIcon(refresh_icon)
        self.reset_button.clicked.connect(
            self.on_reset_button_clicked
        )

        state_monitor = QGroupBox("State Monitor", first_tab)
        state_monitor_layout = QVBoxLayout(state_monitor)

        self.pos_plot_widget = pg.PlotWidget(title="Position")
        self.pos_plot_widget.setBackground('w')
        self.tau_plot_widget = pg.PlotWidget(title="Torque")
        self.tau_plot_widget.setBackground('w')
        
        state_monitor_layout.addWidget(self.pos_plot_widget)
        state_monitor_layout.addWidget(self.tau_plot_widget)
  
        motor_controller = QGroupBox("Motor Controller", first_tab)
        motor_controller_layout = QVBoxLayout(motor_controller)

        abs_controller = QGroupBox("Absolute", motor_controller)
        abs_controller_layout = QHBoxLayout(abs_controller)

        self.cur_pos_label = QLabel("Current Position:    ")
        self.cur_pos_label.setFixedWidth(330)
        self.pos_slider = QSlider(Qt.Horizontal)
        self.pos_slider.valueChanged.connect(
            self.on_pos_slider_changed
        )
        self.max_pos_label = QLabel()
        self.max_pos_label.setFixedWidth(80)
        self.tool_button = QToolButton()
        self.tool_button.setFixedWidth(300)
        self.tool_button.setText("Select a Motor...")
        self.tool_button.setPopupMode(QToolButton.MenuButtonPopup)
        if self.motor_state:
            self.add_tool_button_component(self.tool_button, self.on_control_motor_selected)
            self.add_motor_command_publisher()

        abs_controller_layout.addWidget(self.cur_pos_label)
        abs_controller_layout.addWidget(self.pos_slider)
        abs_controller_layout.addWidget(self.max_pos_label)
        abs_controller_layout.addWidget(self.tool_button)

        rel_controller = QGroupBox("Relative", motor_controller)
        rel_controller_layout = QHBoxLayout(rel_controller)

        self.forward_button = QPushButton("Forward")
        self.forward_button.clicked.connect(
            self.on_forward_button_clicked
        )
        self.value_text = QPlainTextEdit()
        self.value_text.setPlaceholderText("Degree x 100...")
        self.value_text.setMaximumSize(300, 50)
        self.backward_button = QPushButton("Backward")
        self.backward_button.clicked.connect(
            self.on_backward_button_clicked
        )

        rel_controller_layout.addWidget(self.forward_button)
        rel_controller_layout.addWidget(self.value_text)
        rel_controller_layout.addWidget(self.backward_button)

        motor_controller_layout.addWidget(abs_controller)
        motor_controller_layout.addWidget(rel_controller)

        first_tab_layout.addWidget(self.reset_button, 0, Qt.AlignRight)
        first_tab_layout.addWidget(state_monitor)
        first_tab_layout.addWidget(motor_controller)
        self.q_tab_widget.addTab(first_tab, "CANopen Motor Controller")

    def on_reset_button_clicked(self):
        if self.motor_state:
            self.add_tool_button_component(self.tool_button, self.on_control_motor_selected)
            self.motor_command_publishers = {}
            self.name_to_index = {}
            self.add_motor_command_publisher()

    def on_control_motor_selected(self, name):
        if self.motor_state:
            self.selected_motor_name = name
            self.tool_button.setText(name)

            min_limit = int(self.motor_state.min_position_limit[self.name_to_index[name]] * 100)
            max_limit = int(self.motor_state.max_position_limit[self.name_to_index[name]] * 100)
            self.pos_slider.setRange(min_limit, max_limit)

            current_degree = int(self.motor_state.position[self.name_to_index[name]] * 100)
            self.pos_slider.setValue(current_degree)
            
            if current_degree < 0:
                current_degree = -current_degree
                self.cur_pos_label.setText(f"Current Position:   -{int(current_degree//100)}.{int(current_degree%100)}")
            else:
                self.cur_pos_label.setText(f"Current Position:    {int(current_degree//100)}.{int(current_degree%100)}")
            self.max_pos_label.setText(f"{int(max_limit//100)}.{int(max_limit%100)}")

    def on_pos_slider_changed(self, value):
        if self.name_to_index:
            msg = Float64()
            msg.data = value / 180.0 * PI / 100
            self.motor_command_publishers[self.selected_motor_name].publish(msg)

            if value < 0:
                value = -value
                self.cur_pos_label.setText(f"Current Position:   -{int(value//100)}.{int(value%100)}")
            else:
                self.cur_pos_label.setText(f"Current Position:    {int(value//100)}.{int(value%100)}")

    def on_forward_button_clicked(self):
        if self.relative_value != "":
            current_degree = self.motor_state.position[self.name_to_index[self.selected_motor_name]]
            self.node.get_logger().info(f"{current_degree}, {self.relative_value}")
            msg = Float64()
            msg.data = (self.relative_value + current_degree) / 180.0 * PI
            self.motor_command_publishers[self.selected_motor_name].publish(msg)
            
    def on_backward_button_clicked(self):
        if self.relative_value != "":
            current_degree = self.motor_state.position[self.name_to_index[self.selected_motor_name]]
            self.node.get_logger().info(f"{current_degree}")
            msg = Float64()
            msg.data = (-self.relative_value + current_degree) / 180.0 * PI
            self.motor_command_publishers[self.selected_motor_name].publish(msg)

    def update_indicators(self):
        self.plot_graph()
        self.relative_value = self.value_text.toPlainText()
        if self.relative_value != "":
            self.relative_value = float(self.relative_value) / 100

    def add_tool_button_component(self, tool_button, on_motor_selected):
        menu = QMenu(tool_button)
        for name in self.motor_state.name:
            action = QAction(name, tool_button)
            action.triggered.connect(lambda _, n=name: on_motor_selected(n))
            menu.addAction(action)
        tool_button.setMenu(menu)

    def add_motor_command_publisher(self):
        for idx, motor_name in enumerate(self.motor_state.name):
            publisher = self.node.create_publisher(
                Float64, f'{motor_name}/command', self.QOS_REKL5V
            )
            self.motor_command_publishers[motor_name] = publisher
            self.name_to_index[motor_name] = idx

    def plot_graph(self):
        if len(self.positions) == 50:
            self.pos_plot_widget.clear()
            self.tau_plot_widget.clear()

            self.pos_plot_widget.addLegend(offset=(10, 10))
            self.tau_plot_widget.addLegend(offset=(10, 10))

            self.pos_plot_widget.setLabel("left", "Degree")
            self.tau_plot_widget.setLabel("left", "Nm")

            colors = cycle(['r', 'g', 'b', 'm', 'c', 'k'])
            for idx, name in enumerate(self.motor_state.name):
                time = [i*0.01 for i in range(50)]
                pos = [raw[idx] for raw in self.positions]
                tau = [raw[idx] for raw in self.torques]

                color = next(colors)
                pen   = pg.mkPen(color, width=3)

                self.pos_plot_widget.plot(time, pos, pen=pen, name=f"{name} position")
                self.tau_plot_widget.plot(time, tau, pen=pen, name=f"{name} torque")

    def motor_state_callback(self, msg):
        self.motor_state = msg
        self.positions.append([pos for pos in msg.position])
        self.torques.append([tau for tau in msg.torque])
        if len(self.positions) > 50:
            self.positions.pop(0)
            self.torques.pop(0)
        
    def shutdown_widget(self):
        self.update_timer.stop()
        self.node.destroy_node()