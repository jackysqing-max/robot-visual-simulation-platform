"""English desktop workspace for the ROS 2 / PyBullet robot pipeline.

The GUI only publishes ROS commands. The simulator and ros2_control controller
run as separate processes, so other ROS 2 applications can use the same API.
"""
from __future__ import annotations

import argparse
import copy
import json
import math
import os
from pathlib import Path
import signal
import sys
import tempfile
import time
from typing import Optional

from PyQt5.QtCore import QProcess, QProcessEnvironment, QSize, Qt, QTimer, pyqtSignal
from PyQt5.QtGui import QColor, QFont, QImage, QPixmap
from PyQt5.QtWidgets import (
    QAction, QApplication, QComboBox, QDockWidget, QDoubleSpinBox, QFileDialog,
    QFormLayout, QFrame, QGridLayout, QHBoxLayout, QHeaderView, QLabel, QLineEdit,
    QMainWindow, QMessageBox, QPushButton, QScrollArea, QSizePolicy, QSplitter,
    QTabWidget, QTableWidget, QTableWidgetItem, QTextEdit, QToolButton,
    QVBoxLayout, QWidget,
)

from .project_config import ProjectConfig, build_model, inspect_robot, load_project, save_project
from .robot_monitor_widget import RobotMonitorWidget


STYLE = """
QMainWindow { background: #eef2f7; color: #1e2d44; }
QWidget { font-family: 'DejaVu Sans'; font-size: 11px; color: #22334d; }
QWidget#sidebar { background: #15253f; }
QLabel#brand { background: #2e4969; color: white; border-radius: 11px; font-size: 15px; font-weight: bold; }
QToolButton#navButton { background: transparent; color: #a8bbd5; border: none; border-radius: 8px; padding: 12px 3px; font-size: 10px; }
QToolButton#navButton:hover, QToolButton#navButton:checked { background: #294366; color: white; }
QWidget#topbar { background: white; border-bottom: 1px solid #dce4ee; }
QLabel#appTitle { font-size: 19px; font-weight: bold; color: #15253f; }
QLabel#subtitle, QLabel#muted { color: #7a889c; font-size: 10px; }
QLabel#sectionTitle { font-size: 12px; font-weight: bold; color: #223652; }
QLabel#eyebrow { font-size: 9px; font-weight: bold; color: #75859b; letter-spacing: 1px; }
QLabel#chip { background: #edf3fb; color: #365d8c; border: 1px solid #d5e1ef; border-radius: 10px; padding: 4px 9px; }
QLabel#connectionChip { background: #f1f4f8; color: #7a889c; border-radius: 10px; padding: 5px 10px; }
QLabel#pending { background: #fff7e8; color: #936412; border: 1px solid #f3dfb6; border-radius: 5px; padding: 7px; }
QFrame#card, QWidget#card { background: white; border: 1px solid #dce4ed; border-radius: 8px; }
QFrame#viewportFrame { background: #edf2f6; border: 1px solid #d7e0eb; border-radius: 8px; }
QLabel#viewport { background: #e8edf3; color: #76879d; border: none; }
QLabel#viewportPlaceholder { font-size: 17px; color: #49617d; }
QPushButton, QToolButton { background: white; border: 1px solid #d6dfeb; border-radius: 5px; padding: 7px 11px; color: #334b69; }
QPushButton:hover, QToolButton:hover { background: #edf4ff; border-color: #9ab6da; }
QPushButton:disabled { color: #a3aebd; background: #f1f4f8; border-color: #e2e7ee; }
QPushButton#primary { background: #246ad4; color: white; border: 1px solid #246ad4; font-weight: bold; }
QPushButton#primary:hover { background: #1c58b5; }
QPushButton#primary:disabled { background: #a4b9d6; border-color: #a4b9d6; color: #eff3f9; }
QPushButton#danger { color: #b34949; border-color: #e4caca; }
QPushButton#danger:hover { background: #fff1f1; }
QLineEdit, QComboBox, QDoubleSpinBox { background: #fbfcfe; border: 1px solid #d8e2ee; border-radius: 4px; padding: 5px; min-height: 18px; }
QLineEdit:focus, QComboBox:focus, QDoubleSpinBox:focus { border-color: #3880e7; }
QComboBox { padding-right: 20px; }
QComboBox::drop-down { border: 0; width: 19px; }
QTabWidget::pane { background: white; border: 1px solid #dce4ed; border-radius: 6px; }
QTabBar::tab { padding: 9px 17px; background: #edf2f8; border: none; color: #70829a; }
QTabBar::tab:selected { color: #2164c3; background: white; border-bottom: 2px solid #2e73d6; }
QTableWidget { border: none; background: white; gridline-color: #edf1f6; selection-background-color: #e9f1fd; }
QHeaderView::section { background: #f4f7fb; color: #73849a; padding: 5px; border: none; border-bottom: 1px solid #e4ebf3; }
QDockWidget { font-weight: bold; }
QDockWidget::title { text-align: left; background: white; padding: 8px 12px; border-top: 1px solid #dce4ed; border-bottom: 1px solid #e4ebf3; }
QScrollArea { border: none; background: transparent; }
QScrollArea > QWidget > QWidget { background: transparent; }
QScrollBar:vertical { background: transparent; width: 7px; margin: 0; }
QScrollBar::handle:vertical { background: #c7d2e1; border-radius: 3px; min-height: 25px; }
QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical { height: 0px; }
QTextEdit { border: none; background: #f9fbfe; color: #4e647e; font-family: 'DejaVu Sans Mono'; font-size: 10px; }
QStatusBar { background: white; color: #70829a; border-top: 1px solid #dde5ef; }
QSplitter::handle { background: #e9eef5; }
QMenuBar { background: white; }
QToolTip { color: #203650; background: #fff; border: 1px solid #cad6e5; }
"""


def _label(text, name=None, wrap=False):
    label = QLabel(text)
    if name:
        label.setObjectName(name)
    label.setWordWrap(wrap)
    return label


def _button(text, slot=None, primary=False, name=None):
    button = QPushButton(text)
    if primary:
        button.setObjectName('primary')
    if name:
        button.setProperty('testName', name)
    if slot:
        button.clicked.connect(slot)
    return button


def _rpy_quaternion(rpy):
    roll, pitch, yaw = (float(v) / 2 for v in rpy)
    cr, sr, cp, sp, cy, sy = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch), math.cos(yaw), math.sin(yaw)
    return [sr*cp*cy-cr*sp*sy, cr*sp*cy+sr*cp*sy, cr*cp*sy-sr*sp*cy, cr*cp*cy+sr*sp*sy]


def _quaternion_rpy(q):
    x, y, z, w = [float(v) for v in q]
    return [math.atan2(2*(w*x+y*z), 1-2*(x*x+y*y)),
            math.asin(max(-1., min(1., 2*(w*y-z*x)))),
            math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))]


class SimulationViewport(QLabel):
    """ROS camera image with orbit interaction; never creates a second simulator."""
    camera_changed = pyqtSignal(float, float, float)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setObjectName('viewport')
        self.setAlignment(Qt.AlignCenter)
        self.setMinimumSize(420, 280)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self._frame = None
        self._drag = None
        self.yaw, self.pitch, self.distance = 45., -30., 1.7
        self.setText('Start Simulation\n\nA live PyBullet camera view appears here.\nDrag to orbit · Scroll to zoom')
        self.setCursor(Qt.OpenHandCursor)

    def set_frame(self, frame):
        if isinstance(frame, QImage) and not frame.isNull():
            self._frame = QPixmap.fromImage(frame)
            self._rescale()

    def _rescale(self):
        if self._frame is not None:
            self.setPixmap(self._frame.scaled(self.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation))

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self._rescale()

    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton:
            self._drag = event.pos()
            self.setCursor(Qt.ClosedHandCursor)

    def mouseMoveEvent(self, event):
        if self._drag is not None:
            delta = event.pos() - self._drag
            self._drag = event.pos()
            self.yaw += delta.x() * .4
            self.pitch = max(-89., min(-2., self.pitch + delta.y() * .3))
            self.camera_changed.emit(self.yaw, self.pitch, self.distance)

    def mouseReleaseEvent(self, event):
        self._drag = None
        self.setCursor(Qt.OpenHandCursor)

    def wheelEvent(self, event):
        self.distance = max(.35, min(5., self.distance * (.9 if event.angleDelta().y() > 0 else 1.1)))
        self.camera_changed.emit(self.yaw, self.pitch, self.distance)


class WorkbenchWindow(QMainWindow):
    """Project editor, owned backend lifecycle, ROS controls and monitor docks."""
    def __init__(self, bridge=None, project_path=None, parent=None):
        super().__init__(parent)
        self.setObjectName('robotWorkbench')
        self.setWindowTitle('Robot Workbench | LBR × PyBullet')
        self.resize(1560, 1040)
        self.setMinimumSize(1140, 780)
        self.setStyleSheet(STYLE)
        self._runtime = tempfile.TemporaryDirectory(prefix='lbr_workbench_')
        self.project_path = str(project_path) if project_path else None
        self.project = load_project(project_path) if project_path else ProjectConfig()
        self._metadata = inspect_robot(self.project)
        self._staged = False
        self._loading = False
        self._connected = False
        self._snapshot = {}
        self._restart_requested = False
        self._reloading = False
        self._backend_pgid = None
        self._last_owned_exit_at = float('-inf')
        self._last_image = 0.
        self._last_monitor_raw_sequence = None
        self._last_status = 0.
        self._closing = False
        self._camera_last_sent = 0.
        self.backend = QProcess(self)
        self.backend.setProcessChannelMode(QProcess.MergedChannels)
        self.backend.readyReadStandardOutput.connect(self._backend_output)
        self.backend.started.connect(self._backend_started)
        self.backend.finished.connect(self._backend_finished)
        self.backend.errorOccurred.connect(self._backend_error)
        self.bridge = bridge
        self._build_ui()
        self._build_menus()
        self._load_fields()
        if bridge is None:
            from .workbench_bridge import WorkbenchBridge
            self.bridge = WorkbenchBridge()
        self.bridge.image.connect(self._image_received)
        self.bridge.snapshot.connect(self._snapshot_received)
        self.bridge.event.connect(self._event_received)
        self.bridge.connection.connect(self._connection_changed)
        self.spin_timer = QTimer(self)
        self.spin_timer.timeout.connect(self._spin)
        self.spin_timer.start(20)
        self.health_timer = QTimer(self)
        self.health_timer.timeout.connect(self._refresh_health)
        self.health_timer.start(500)
        self._refresh_buttons()
        QTimer.singleShot(150, self._compact_monitor_dock)
        self.log('Ready. Import a robot, configure its instrument, then start the simulation.')

    def _build_ui(self):
        root = QWidget()
        body = QVBoxLayout(root)
        body.setContentsMargins(0, 0, 0, 0)
        body.setSpacing(0)
        top = QWidget()
        top.setObjectName('topbar')
        top_layout = QHBoxLayout(top)
        top_layout.setContentsMargins(20, 13, 20, 13)
        title = QVBoxLayout()
        title.setSpacing(3)
        title.addWidget(_label('Robot Workbench', 'appTitle'))
        title.addWidget(_label('MODEL CONFIGURATION  /  SIMULATION  /  MOTION CONTROL', 'subtitle'))
        top_layout.addLayout(title)
        top_layout.addStretch()
        self.connection_chip = _label('●  ROS 2 waiting', 'connectionChip')
        top_layout.addWidget(self.connection_chip)
        self.open_button = _button('Open Project', self.open_project)
        self.open_button.setObjectName('openProjectButton')
        self.save_button = _button('Save', self.save_project)
        self.save_button.setObjectName('saveProjectButton')
        self.apply_button = _button('Apply && Reload', self.apply_project)
        self.apply_button.setObjectName('applyProjectButton')
        self.start_button = _button('▶  Start Simulation', self.start_backend, primary=True)
        self.start_button.setProperty('action', 'startSimulation')
        self.stop_button = _button('■  Stop', self.stop_backend)
        self.stop_button.setObjectName('danger')
        self.stop_button.setProperty('action', 'stopSimulation')
        for button in (self.open_button, self.save_button, self.apply_button, self.start_button, self.stop_button):
            top_layout.addWidget(button)
        body.addWidget(top)
        workspace = QHBoxLayout()
        workspace.setContentsMargins(0, 0, 0, 0)
        workspace.setSpacing(0)
        sidebar = QWidget()
        sidebar.setObjectName('sidebar')
        sidebar.setFixedWidth(76)
        side = QVBoxLayout(sidebar)
        side.setContentsMargins(7, 17, 7, 14)
        brand = _label('RW', 'brand')
        brand.setFixedSize(43, 43)
        brand.setAlignment(Qt.AlignCenter)
        side.addWidget(brand, 0, Qt.AlignHCenter)
        side.addSpacing(19)
        for caption, callback in [('◈\nWorkspace', self._show_workspace), ('▦\nRobot', self._focus_robot),
                                  ('⚒\nInstrument', self._focus_tool), ('▥\nMonitor', self._show_monitor),
                                  ('≡\nEvents', self._show_log)]:
            nav = QToolButton()
            nav.setObjectName('navButton')
            nav.setText(caption)
            nav.setToolButtonStyle(Qt.ToolButtonTextOnly)
            nav.setFixedWidth(62)
            nav.clicked.connect(callback)
            side.addWidget(nav)
        side.addStretch()
        ros = _label('ROS 2\nHUMBLE')
        ros.setAlignment(Qt.AlignCenter)
        ros.setStyleSheet('color: #718aaa; font-size: 9px;')
        side.addWidget(ros)
        workspace.addWidget(sidebar)
        self.workspace_split = QSplitter(Qt.Horizontal)
        self.workspace_split.setHandleWidth(8)
        self.config_scroll = QScrollArea()
        self.config_scroll.setWidgetResizable(True)
        self.config_scroll.setMinimumWidth(295)
        self.config_scroll.setMaximumWidth(400)
        config = QWidget()
        config_layout = QVBoxLayout(config)
        config_layout.setContentsMargins(13, 14, 5, 14)
        config_layout.setSpacing(12)
        heading = QHBoxLayout()
        heading.addWidget(_label('PROJECT SETUP', 'eyebrow'))
        heading.addStretch()
        self.project_label = _label('Unsaved project', 'muted')
        heading.addWidget(self.project_label)
        config_layout.addLayout(heading)
        self.pending_label = _label('Changes are staged. Apply & Reload to use them.', 'pending', True)
        self.pending_label.hide()
        config_layout.addWidget(self.pending_label)
        self.robot_card, robot = self._card('01', 'Robot model', 'Import a URDF and choose the tool flange.')
        self.robot_path = QLineEdit()
        self.robot_path.setObjectName('robotUrdfPath')
        self.robot_path.setPlaceholderText('Built-in KUKA LBR iiwa 7')
        self.robot_path.editingFinished.connect(self._robot_path_edited)
        robot.addLayout(self._file_row(self.robot_path, self.import_robot))
        reset_robot = _button('Use built-in LBR iiwa 7', self.use_default_robot)
        robot.addWidget(reset_robot)
        form = QFormLayout()
        form.setSpacing(8)
        self.flange_combo = QComboBox()
        self.flange_combo.setObjectName('flangeLinkCombo')
        self.flange_combo.currentTextChanged.connect(self._mark_staged)
        form.addRow('Tool flange', self.flange_combo)
        robot.addLayout(form)
        self.robot_info = _label('', 'muted', True)
        self.compatibility_label = _label('', 'chip', True)
        robot.addWidget(self.robot_info)
        robot.addWidget(self.compatibility_label)
        config_layout.addWidget(self.robot_card)
        self.tool_card, tool = self._card('02', 'End effector', 'Exchange instruments without editing robot files.')
        self.tool_preset = QComboBox()
        self.tool_preset.setObjectName('toolPresetCombo')
        for label, value in [('Bare flange', 'none'), ('Surgical probe', 'probe'),
                             ('Fixed open gripper', 'gripper'), ('Custom URDF / mesh', 'custom')]:
            self.tool_preset.addItem(label, value)
        self.tool_preset.currentIndexChanged.connect(self._tool_preset_changed)
        tool.addWidget(self.tool_preset)
        self.tool_path = QLineEdit()
        self.tool_path.setObjectName('toolAssetPath')
        self.tool_path.setPlaceholderText('Instrument .urdf, .stl or .obj')
        self.tool_path.textEdited.connect(self._mark_staged)
        tool.addLayout(self._file_row(self.tool_path, self.import_tool))
        tool.addWidget(_label('Mesh scale (X / Y / Z)', 'muted'))
        self.scale_fields = self._vector_fields(tool, 'scale', [1, 1, 1], .0001, 100, 4, .001)
        tool.addWidget(_label('MOUNT TRANSFORM  ·  Flange → instrument', 'eyebrow'))
        tool.addWidget(_label('Translation X / Y / Z (m)', 'muted'))
        self.mount_xyz_fields = self._vector_fields(tool, 'mountXYZ', [0, 0, 0], -5, 5, 4, .001)
        tool.addWidget(_label('Rotation R / P / Y (deg)', 'muted'))
        self.mount_rpy_fields = self._vector_fields(tool, 'mountRPY', [0, 0, 0], -360, 360, 1, 1)
        tool.addWidget(_label('TCP OFFSET  ·  Instrument → tool center point', 'eyebrow'))
        tool.addWidget(_label('Translation X / Y / Z (m)', 'muted'))
        self.tcp_xyz_fields = self._vector_fields(tool, 'tcpXYZ', [0, 0, 0], -5, 5, 4, .001)
        tool.addWidget(_label('Rotation R / P / Y (deg)', 'muted'))
        self.tcp_rpy_fields = self._vector_fields(tool, 'tcpRPY', [0, 0, 0], -360, 360, 1, 1)
        self.tool_help = _label('TCP targets and tracking use this tool center point.\nRigid instruments are fixed attachments; finger actuation is separate.', 'muted', True)
        tool.addWidget(self.tool_help)
        config_layout.addWidget(self.tool_card)
        notes, notes_layout = self._card('03', 'ROS 2 interface', 'Connect perception, planning and task modules.')
        notes_layout.addWidget(_label('/lbr_tutorial/joint_goal\n/lbr_tutorial/pose_goal\n/lbr_tutorial/cartesian_path', 'muted'))
        config_layout.addWidget(notes)
        config_layout.addStretch()
        self.config_scroll.setWidget(config)
        self.workspace_split.addWidget(self.config_scroll)
        center = QWidget()
        center_layout = QVBoxLayout(center)
        center_layout.setContentsMargins(5, 14, 14, 12)
        center_layout.setSpacing(11)
        view_header = QHBoxLayout()
        view_title = QVBoxLayout()
        view_title.setSpacing(3)
        view_title.addWidget(_label('Simulation workspace', 'sectionTitle'))
        self.scene_label = _label('KUKA LBR iiwa 7  /  Bare flange', 'muted')
        view_title.addWidget(self.scene_label)
        view_header.addLayout(view_title)
        view_header.addStretch()
        self.mode_chip = _label('LBR CONTROLLER', 'chip')
        view_header.addWidget(self.mode_chip)
        self.camera_combo = QComboBox()
        self.camera_combo.addItems(['Perspective', 'Front', 'Side', 'Top'])
        self.camera_combo.currentTextChanged.connect(self._camera_preset)
        view_header.addWidget(self.camera_combo)
        self.reset_camera_button = _button('Reset View', lambda: self._camera_preset('Perspective'))
        view_header.addWidget(self.reset_camera_button)
        center_layout.addLayout(view_header)
        viewport_frame = QFrame()
        viewport_frame.setObjectName('viewportFrame')
        view_layout = QVBoxLayout(viewport_frame)
        view_layout.setContentsMargins(1, 1, 1, 1)
        view_layout.setSpacing(0)
        self.viewport = SimulationViewport()
        self.viewport.camera_changed.connect(self._set_camera)
        view_layout.addWidget(self.viewport, 1)
        view_footer = QWidget()
        footer = QHBoxLayout(view_footer)
        footer.setContentsMargins(12, 8, 12, 8)
        self.render_status = _label('○  Camera offline', 'muted')
        self.tcp_readout = _label('TCP  X —   Y —   Z —', 'muted')
        footer.addWidget(self.render_status)
        footer.addStretch()
        footer.addWidget(self.tcp_readout)
        view_layout.addWidget(view_footer)
        center_layout.addWidget(viewport_frame, 1)
        self.control_tabs = QTabWidget()
        self.control_tabs.setObjectName('motionControlTabs')
        self.control_tabs.setMaximumHeight(245)
        self.control_tabs.addTab(self._build_joint_control(), 'Joint control')
        self.control_tabs.addTab(self._build_cartesian_control(), 'Cartesian TCP')
        center_layout.addWidget(self.control_tabs)
        self.workspace_split.addWidget(center)
        self.workspace_split.setStretchFactor(0, 0)
        self.workspace_split.setStretchFactor(1, 1)
        self.workspace_split.setSizes([345, 1135])
        workspace.addWidget(self.workspace_split, 1)
        body.addLayout(workspace, 1)
        self.setCentralWidget(root)
        self.monitor = RobotMonitorWidget()
        self.monitor_dock = QDockWidget('Robot Monitor  ·  Live joint feedback & tracking', self)
        self.monitor_dock.setObjectName('robotMonitorDock')
        self.monitor_dock.setAllowedAreas(Qt.BottomDockWidgetArea | Qt.RightDockWidgetArea)
        self.monitor_dock.setWidget(self.monitor)
        self.addDockWidget(Qt.BottomDockWidgetArea, self.monitor_dock)
        self.monitor_dock.setMinimumHeight(230)
        self.resizeDocks([self.monitor_dock], [265], Qt.Vertical)
        self.event_log = QTextEdit()
        self.event_log.setObjectName('eventLog')
        self.event_log.setReadOnly(True)
        self.event_log.document().setMaximumBlockCount(1500)
        self.log_dock = QDockWidget('Event Log', self)
        self.log_dock.setObjectName('eventLogDock')
        self.log_dock.setWidget(self.event_log)
        self.addDockWidget(Qt.BottomDockWidgetArea, self.log_dock)
        self.tabifyDockWidget(self.monitor_dock, self.log_dock)
        self.monitor_dock.raise_()
        self.status_label = _label('Ready  ·  All units in meters, radians and seconds')
        self.statusBar().addWidget(self.status_label, 1)
        self.statusBar().addPermanentWidget(_label('LBR STACK  →  ROS 2 CONTROL  →  PYBULLET', 'muted'))

    def _compact_monitor_dock(self):
        # The responsive plot grid changes after its first full-width resize.
        # Clear the dock's cached initial two-row minimum before sizing it.
        self.monitor_dock.setMinimumSize(0, 0)
        if self.monitor_dock.layout():
            self.monitor_dock.layout().invalidate()
        self.resizeDocks([self.monitor_dock], [310], Qt.Vertical)

    def _build_menus(self):
        menu = self.menuBar().addMenu('File')
        for text, shortcut, method in [('Open Project…', 'Ctrl+O', self.open_project),
                                       ('Save Project', 'Ctrl+S', self.save_project),
                                       ('Save Project As…', 'Ctrl+Shift+S', self.save_project_as)]:
            action = QAction(text, self)
            action.setShortcut(shortcut)
            action.triggered.connect(method)
            menu.addAction(action)
        menu.addSeparator()
        action = menu.addAction('Exit')
        action.triggered.connect(self.close)
        view = self.menuBar().addMenu('View')
        view.addAction(self.monitor_dock.toggleViewAction())
        view.addAction(self.log_dock.toggleViewAction())
        self.legacy_dock = QDockWidget('Legacy Robot Monitor', self)
        self.legacy_dock.setObjectName('legacyRobotMonitorDock')
        legacy_container = QWidget()
        legacy_layout = QVBoxLayout(legacy_container)
        row = QHBoxLayout()
        self.legacy_executable = QLineEdit()
        self.legacy_executable.setPlaceholderText('Choose the existing Robot Monitor executable')
        row.addWidget(self.legacy_executable, 1)
        row.addWidget(_button('Choose…', self._choose_legacy_executable))
        self.legacy_workspace = QLineEdit()
        self.legacy_workspace.setPlaceholderText('Original monitor ROS workspace')
        row.addWidget(self.legacy_workspace)
        row.addWidget(_button('Workspace…', self._choose_legacy_workspace))
        row.addWidget(_button('Open Monitor', self._open_legacy_monitor))
        legacy_layout.addLayout(row)
        self.legacy_monitor = None
        try:
            from .legacy_monitor_widget import LegacyMonitorWidget
            self.legacy_monitor = LegacyMonitorWidget()
            self.legacy_monitor.status_changed.connect(lambda msg: self.log(msg, 'MONITOR'))
            legacy_layout.addWidget(self.legacy_monitor, 1)
        except ImportError:
            legacy_layout.addWidget(_label('The native Robot Monitor is available from View → Robot Monitor.', 'muted'))
        self.legacy_dock.setWidget(legacy_container)
        self.addDockWidget(Qt.RightDockWidgetArea, self.legacy_dock)
        self.legacy_dock.hide()
        view.addAction(self.legacy_dock.toggleViewAction())
        help_menu = self.menuBar().addMenu('Help')
        help_menu.addAction('About Robot Workbench').triggered.connect(self._about)

    def _card(self, number, title, subtitle):
        card = QFrame()
        card.setObjectName('card')
        layout = QVBoxLayout(card)
        layout.setContentsMargins(13, 12, 13, 13)
        layout.setSpacing(9)
        title_row = QHBoxLayout()
        num = _label(number)
        num.setStyleSheet('color: #3474cb; font-weight: bold;')
        title_row.addWidget(num)
        title_row.addWidget(_label(title, 'sectionTitle'))
        title_row.addStretch()
        layout.addLayout(title_row)
        layout.addWidget(_label(subtitle, 'muted', True))
        return card, layout

    def _file_row(self, line_edit, callback):
        layout = QHBoxLayout()
        layout.setSpacing(5)
        layout.addWidget(line_edit, 1)
        choose = _button('…', callback)
        choose.setFixedWidth(31)
        choose.setToolTip('Choose a file')
        layout.addWidget(choose)
        return layout

    def _vector_fields(self, parent_layout, name, values, minimum, maximum, decimals, step):
        row = QHBoxLayout()
        row.setSpacing(5)
        fields = []
        for axis, value in zip('XYZ', values):
            spin = QDoubleSpinBox()
            spin.setObjectName(name + axis)
            spin.setRange(minimum, maximum)
            spin.setDecimals(decimals)
            spin.setSingleStep(step)
            spin.setValue(value)
            spin.setKeyboardTracking(False)
            spin.setMinimumWidth(55)
            spin.valueChanged.connect(self._mark_staged)
            row.addWidget(spin)
            fields.append(spin)
        parent_layout.addLayout(row)
        return fields

    def _build_joint_control(self):
        widget = QWidget()
        layout = QVBoxLayout(widget)
        layout.setContentsMargins(12, 10, 12, 10)
        layout.setSpacing(7)
        header = QHBoxLayout()
        header.addWidget(_label('Joint targets (rad)  ·  Interpolated through the LBR trajectory controller', 'muted'))
        header.addStretch()
        self.capture_joints_button = _button('Use Current', self.capture_joints)
        header.addWidget(self.capture_joints_button)
        layout.addLayout(header)
        self.joint_table = QTableWidget(1, 7)
        self.joint_table.setObjectName('jointTargetTable')
        self.joint_table.setVerticalHeaderLabels(['Target'])
        self.joint_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.joint_table.verticalHeader().hide()
        self.joint_table.setMinimumHeight(62)
        self.joint_table.setMaximumHeight(78)
        layout.addWidget(self.joint_table)
        actions = QHBoxLayout()
        self.command_notice = _label('Start Simulation to enable motion commands.', 'muted', True)
        actions.addWidget(self.command_notice, 1)
        self.cancel_button = _button('Cancel Motion', self.cancel_motion)
        self.cancel_button.setObjectName('cancelMotionButton')
        actions.addWidget(self.cancel_button)
        self.move_joints_button = _button('Move Joints', self.move_joints, primary=True)
        self.move_joints_button.setProperty('action', 'moveJoints')
        actions.addWidget(self.move_joints_button)
        layout.addLayout(actions)
        return widget

    def _build_cartesian_control(self):
        widget = QWidget()
        layout = QVBoxLayout(widget)
        layout.setContentsMargins(12, 10, 12, 10)
        layout.setSpacing(7)
        caption = QHBoxLayout()
        caption.addWidget(_label('TCP target in the robot base frame', 'muted'))
        caption.addStretch()
        self.capture_pose_button = _button('Use Current TCP', self.capture_pose)
        caption.addWidget(self.capture_pose_button)
        layout.addLayout(caption)
        row = QHBoxLayout()
        self.pose_fields = []
        for index, title in enumerate(['X (m)', 'Y (m)', 'Z (m)', 'Roll (deg)', 'Pitch (deg)', 'Yaw (deg)']):
            column = QVBoxLayout()
            column.setSpacing(3)
            column.addWidget(_label(title, 'muted'))
            spin = QDoubleSpinBox()
            spin.setObjectName('targetPose' + str(index))
            spin.setRange(-3 if index < 3 else -360, 3 if index < 3 else 360)
            spin.setDecimals(4 if index < 3 else 1)
            spin.setSingleStep(.005 if index < 3 else 1)
            spin.setValue([.5, 0, .5, 0, 90, 0][index])
            column.addWidget(spin)
            self.pose_fields.append(spin)
            row.addLayout(column)
        layout.addLayout(row)
        actions = QHBoxLayout()
        actions.addWidget(_label('IK → joint interpolation → measured feedback', 'muted'), 1)
        self.square_button = _button('Trace 4 cm Square', self.trace_square)
        self.square_button.setObjectName('traceSquareButton')
        actions.addWidget(self.square_button)
        self.move_pose_button = _button('Move TCP', self.move_pose, primary=True)
        self.move_pose_button.setProperty('action', 'moveTCP')
        actions.addWidget(self.move_pose_button)
        layout.addLayout(actions)
        return widget

    def _load_fields(self):
        self._loading = True
        self.robot_path.setText(self.project.robot_urdf)
        self.flange_combo.clear()
        self.flange_combo.addItems(list(self._metadata.link_names))
        selected = self.project.flange_link or self._metadata.flange_link
        self.flange_combo.setCurrentText(selected)
        self.tool_path.setText(getattr(self.project, 'tool_asset', ''))
        tool = self.project.tool
        tool_index = self.tool_preset.findData(tool)
        self.tool_preset.setCurrentIndex(tool_index if tool_index >= 0 else self.tool_preset.findData('custom'))
        if tool_index < 0 and tool not in ('none', 'probe', 'gripper'):
            self.tool_path.setText(str(tool))
        for fields, values, degrees in [(self.scale_fields, self.project.mesh_scale, False),
                                       (self.mount_xyz_fields, self.project.mount_xyz, False),
                                       (self.mount_rpy_fields, self.project.mount_rpy, True),
                                       (self.tcp_xyz_fields, self.project.tcp_xyz, False),
                                       (self.tcp_rpy_fields, self.project.tcp_rpy, True)]:
            for field, value in zip(fields, values):
                field.setValue(math.degrees(value) if degrees else value)
        self._update_metadata()
        self._loading = False
        self._staged = False
        self.pending_label.hide()
        self._update_project_label()
        self._update_tool_controls()

    def _update_metadata(self):
        metadata = self._metadata
        names = list(metadata.joint_names)
        self.robot_info.setText(f'{len(names)} movable joints  ·  {len(metadata.link_names)} links\nBase: {metadata.root_link}')
        compatible = bool(metadata.control_compatible)
        self.compatibility_label.setText('✓  7-axis LBR trajectory control' if compatible else 'Preview only · ' + str(metadata.control_reason))
        self.mode_chip.setText('LBR CONTROLLER' if compatible else 'PREVIEW ONLY')
        self.joint_table.setColumnCount(len(names))
        self.joint_table.setHorizontalHeaderLabels([f'J{i+1}' for i in range(len(names))])
        for index, name in enumerate(names):
            self.joint_table.horizontalHeaderItem(index).setToolTip(name)
        q = self.project.initial_positions or metadata.default_q
        for index, name in enumerate(names):
            field = QDoubleSpinBox()
            field.setObjectName('jointTarget' + str(index))
            low, high = float(metadata.lower_limits[index]), float(metadata.upper_limits[index])
            if not math.isfinite(low) or not math.isfinite(high) or low >= high:
                low, high = -2*math.pi, 2*math.pi
            field.setRange(low, high)
            field.setDecimals(4)
            field.setSingleStep(.02)
            field.setValue(float(q[index]))
            self.joint_table.setCellWidget(0, index, field)
        robot_name = Path(self.project.robot_urdf).stem if self.project.robot_urdf else 'KUKA LBR iiwa 7'
        self.scene_label.setText(robot_name + '  /  ' + self.tool_preset.currentText())

    def _update_tool_controls(self):
        custom = self.tool_preset.currentData() == 'custom'
        self.tool_path.setEnabled(custom)
        for field in self.scale_fields:
            field.setEnabled(custom)

    def _collect_project(self):
        project = copy.deepcopy(self.project)
        project.robot_urdf = self.robot_path.text().strip()
        project.flange_link = self.flange_combo.currentText()
        preset = self.tool_preset.currentData()
        project.tool = self.tool_path.text().strip() if preset == 'custom' else preset
        if preset == 'custom' and not project.tool:
            raise ValueError('Choose an instrument URDF, STL or OBJ file.')
        project.mesh_scale = [field.value() for field in self.scale_fields]
        project.mount_xyz = [field.value() for field in self.mount_xyz_fields]
        project.mount_rpy = [math.radians(field.value()) for field in self.mount_rpy_fields]
        project.tcp_xyz = [field.value() for field in self.tcp_xyz_fields]
        project.tcp_rpy = [math.radians(field.value()) for field in self.tcp_rpy_fields]
        return project

    def _mark_staged(self, *_):
        if self._loading:
            return
        self._staged = True
        self.pending_label.show()
        self._refresh_buttons()

    def _tool_preset_changed(self, *_):
        self._update_tool_controls()
        if not self._loading:
            preset = self.tool_preset.currentData()
            # A useful TCP default at the tip of each bundled visual instrument.
            tcp_z = {'none': 0., 'probe': .179, 'gripper': .12}.get(preset)
            if tcp_z is not None:
                self._loading = True
                for field in self.tcp_xyz_fields + self.tcp_rpy_fields:
                    field.setValue(0)
                self.tcp_xyz_fields[2].setValue(tcp_z)
                self._loading = False
            self._mark_staged()

    def _robot_path_edited(self):
        if self._loading or self.robot_path.text().strip() == self.project.robot_urdf:
            return
        self._preview_robot(self.robot_path.text().strip())

    def _preview_robot(self, path):
        candidate = copy.deepcopy(self.project)
        candidate.robot_urdf = path
        candidate.flange_link = ''
        candidate.initial_positions = None
        try:
            metadata = inspect_robot(candidate)
        except Exception as error:
            self._error('Robot import failed', str(error))
            return
        self.project.robot_urdf = path
        self.project.flange_link = ''
        self.project.initial_positions = None
        self._metadata = metadata
        self._loading = True
        self.robot_path.setText(path)
        self.flange_combo.clear()
        self.flange_combo.addItems(metadata.link_names)
        self.flange_combo.setCurrentText(metadata.flange_link)
        self._update_metadata()
        self._loading = False
        self._mark_staged()
        self.log(f'Imported {Path(path).name if path else "built-in LBR iiwa 7"}: {len(metadata.joint_names)} movable joints. Apply & Reload to activate.', 'MODEL')

    def import_robot(self):
        path, _ = QFileDialog.getOpenFileName(self, 'Import Robot Model', str(Path.home()),
                                            'Robot description (*.urdf);;All files (*)', options=QFileDialog.DontUseNativeDialog)
        if path:
            self._preview_robot(path)

    def use_default_robot(self):
        self._preview_robot('')

    def import_tool(self):
        path, _ = QFileDialog.getOpenFileName(self, 'Import End Effector', str(Path.home()),
                                            'Instrument assets (*.urdf *.stl *.obj);;All files (*)', options=QFileDialog.DontUseNativeDialog)
        if path:
            self.tool_preset.setCurrentIndex(self.tool_preset.findData('custom'))
            self.tool_path.setText(path)
            self._mark_staged()
            self.log(f'Instrument asset selected: {Path(path).name}', 'TOOL')

    def apply_project(self):
        try:
            candidate = self._collect_project()
            metadata = inspect_robot(candidate)
            build_model(candidate, Path(self._runtime.name) / 'validation')
            # Validate all robot and tool assets before stopping a running process.
            runtime_path = Path(self._runtime.name) / 'project.json'
            save_project(candidate, runtime_path)
        except Exception as error:
            self._error('Project validation failed', str(error))
            return False
        running = self.backend.state() != QProcess.NotRunning
        self._reloading = running
        self.project, self._metadata = candidate, metadata
        self._load_fields()
        self._refresh_buttons()
        self.log('Project applied. ' + ('Restarting the owned simulation.' if running else 'Ready to start simulation.'), 'PROJECT')
        if running:
            self._restart_requested = True
            self.stop_backend(restarting=True)
        return True

    def open_project(self):
        path, _ = QFileDialog.getOpenFileName(self, 'Open Robot Project', str(Path.home()),
                                            'Robot workbench project (*.json);;All files (*)', options=QFileDialog.DontUseNativeDialog)
        if not path:
            return
        try:
            project = load_project(path)
            metadata = inspect_robot(project)
        except Exception as error:
            self._error('Cannot open project', str(error))
            return
        self.project, self._metadata, self.project_path = project, metadata, path
        self._load_fields()
        self._mark_staged()
        self.log(f'Opened project: {path}. Apply & Reload to activate.', 'PROJECT')

    def save_project(self):
        if not self.project_path:
            self.save_project_as()
            return
        self._save_to(self.project_path)

    def save_project_as(self):
        path, _ = QFileDialog.getSaveFileName(self, 'Save Robot Project', self.project_path or str(Path.home() / 'robot_project.json'),
                                            'Robot workbench project (*.json)', options=QFileDialog.DontUseNativeDialog)
        if path:
            if not path.lower().endswith('.json'):
                path += '.json'
            self._save_to(path)

    def _save_to(self, path):
        try:
            candidate = self._collect_project()
            inspect_robot(candidate)
            build_model(candidate, Path(self._runtime.name) / 'validation')
            save_project(candidate, path)
        except Exception as error:
            self._error('Cannot save project', str(error))
            return
        self.project_path = path
        self._update_project_label()
        self.log(f'Project saved: {path}', 'PROJECT')

    def _update_project_label(self):
        self.project_label.setText(Path(self.project_path).stem if self.project_path else 'Unsaved project')
        self.project_label.setToolTip(self.project_path or 'Use Save to preserve this configuration.')

    def start_backend(self):
        if self.backend.state() != QProcess.NotRunning:
            return
        raw_age = self._snapshot.get('raw_age', float('inf'))
        if float(raw_age) < .75 and time.monotonic() - self._last_owned_exit_at > 1.:
            self._error('Simulator already active', 'A simulator is already active in this ROS domain. Stop it or choose another ROS_DOMAIN_ID.')
            return
        if self._staged and not self.apply_project():
            return
        try:
            runtime_path = Path(self._runtime.name) / 'project.json'
            build_model(self.project, Path(self._runtime.name) / 'validation')
            save_project(self.project, runtime_path)
        except Exception as error:
            self._error('Cannot start simulation', str(error))
            return
        environment = QProcessEnvironment.systemEnvironment()
        environment.insert('LBR_PYBULLET_PROJECT', str(runtime_path))
        environment.insert('PYTHONUNBUFFERED', '1')
        self.backend.setProcessEnvironment(environment)
        self._reloading = True
        self._snapshot = {}
        self._connected = False
        launch = 'tutorial.launch.py' if self._metadata.control_compatible else 'sim.launch.py'
        self.backend.setProgram('/usr/bin/setsid')
        self.backend.setArguments(['ros2', 'launch', 'lbr_pybullet_tutorial', launch,
                                   'gui:=false', 'render_camera:=true'])
        self._restart_requested = False
        self._backend_pgid = None
        self.log(f'Starting {launch} with the applied project.', 'SIM')
        self.status_label.setText('Starting simulation and ROS 2 nodes…')
        self.backend.start()
        self._refresh_buttons()

    def _backend_started(self):
        self._backend_pgid = int(self.backend.processId())
        self.log(f'Owned simulation process started (PID {self._backend_pgid}).', 'SIM')
        self._refresh_buttons()

    def stop_backend(self, restarting=False):
        if not restarting:
            self._restart_requested = False
        if self.backend.state() == QProcess.NotRunning:
            return
        self._reloading = True
        self.log('Stopping the simulation process owned by this window.', 'SIM')
        self.status_label.setText('Stopping simulation…')
        pgid = self._backend_pgid or int(self.backend.processId())
        self._signal_owned_process(pgid, signal.SIGINT)
        QTimer.singleShot(4500, lambda: self._escalate_stop(pgid, signal.SIGTERM))
        QTimer.singleShot(7000, lambda: self._escalate_stop(pgid, signal.SIGKILL))
        self._refresh_buttons()

    def _signal_owned_process(self, pgid, sig):
        if pgid and pgid > 1:
            try:
                os.killpg(pgid, sig)
            except ProcessLookupError:
                pass
            except PermissionError as error:
                self.log(str(error), 'ERROR')

    def _escalate_stop(self, pgid, sig):
        if self.backend.state() != QProcess.NotRunning and self._backend_pgid == pgid:
            self._signal_owned_process(pgid, sig)

    def _backend_finished(self, exit_code, exit_status):
        self._backend_output()
        self._backend_pgid = None
        self._last_owned_exit_at = time.monotonic()
        self._connected = False
        self._snapshot = {}
        self.log(f'Owned simulation stopped (exit code {exit_code}).', 'SIM')
        self.status_label.setText('Simulation stopped')
        if not self._restart_requested:
            self._reloading = False
        self._refresh_buttons()
        if self._restart_requested and not self._closing:
            self._restart_requested = False
            QTimer.singleShot(500, self.start_backend)

    def _backend_error(self, error):
        self.log(self.backend.errorString(), 'ERROR')
        self.status_label.setText('Simulation process error · See Event Log')
        self._refresh_buttons()

    def _backend_output(self):
        output = bytes(self.backend.readAllStandardOutput()).decode('utf-8', errors='replace')
        for line in output.splitlines():
            if line.strip():
                self.log(line.strip(), 'BACKEND')

    def _spin(self):
        try:
            self.bridge.spin()
        except Exception as error:
            if time.monotonic() - self._last_status > 5:
                self.log(str(error), 'ROS')
                self._last_status = time.monotonic()

    def _connection_changed(self, connected):
        self._connected = bool(connected)
        self._refresh_buttons()

    def _refresh_health(self):
        camera_live = time.monotonic() - self._last_image < 2.5
        self.render_status.setText('●  Live PyBullet camera' if camera_live else '○  Camera offline')
        self.render_status.setStyleSheet('color: #218965;' if camera_live else 'color: #7a889c;')
        raw_age = self._snapshot.get('raw_age', float('inf'))
        ctrl_age = self._snapshot.get('controller_age', float('inf'))
        try:
            raw_live = float(raw_age) < 1.
            controller_live = float(ctrl_age) < 1.
        except (TypeError, ValueError):
            raw_live, controller_live = self._connected, self._connected
        if self._connected and raw_live:
            text = '●  ROS 2 connected' if controller_live else '●  Simulation feedback'
            self.connection_chip.setText(text)
            self.connection_chip.setStyleSheet('background: #e9f7ef; color: #25845d; border-radius: 10px; padding: 5px 10px;')
        else:
            self.connection_chip.setText('○  ROS 2 waiting')
            self.connection_chip.setStyleSheet('background: #f1f4f8; color: #7a889c; border-radius: 10px; padding: 5px 10px;')
        self._refresh_buttons()

    def _refresh_buttons(self):
        running = self.backend.state() != QProcess.NotRunning
        owns_running_backend = self.backend.state() == QProcess.Running
        ready = owns_running_backend and bool(self._metadata.control_compatible) and self._connected and not self._staged and not self._reloading
        controller_age = self._snapshot.get('controller_age')
        if controller_age is not None:
            ready = ready and float(controller_age) < 1.
        raw_age = self._snapshot.get('raw_age')
        if raw_age is not None:
            ready = ready and float(raw_age) < .75
        self.start_button.setEnabled(not running)
        self.stop_button.setEnabled(running)
        self.apply_button.setEnabled(self._staged)
        for button in (self.move_joints_button, self.move_pose_button, self.square_button):
            button.setEnabled(ready)
        self.cancel_button.setEnabled(owns_running_backend and self._connected and (controller_age is None or float(controller_age) < 1.))
        self.capture_joints_button.setEnabled(self._connected and not self._staged)
        self.capture_pose_button.setEnabled(self._connected and not self._staged)
        if self._staged:
            notice = 'Apply & Reload to activate the edited configuration.'
        elif not self._metadata.control_compatible:
            notice = 'Preview only: motion requires a compatible 7-axis revolute robot.'
        elif not owns_running_backend:
            notice = 'Start Simulation to enable motion commands for this project.'
        elif not ready:
            notice = 'Waiting for fresh simulator feedback and the LBR controller.'
        else:
            notice = 'Controller ready · Joint limits are checked before execution.'
        self.command_notice.setText(notice)

    def _image_received(self, image):
        self.viewport.set_frame(image)
        self._last_image = time.monotonic()

    def _snapshot_received(self, snapshot):
        self._snapshot = snapshot
        if self._reloading and self.backend.state() == QProcess.Running:
            try:
                raw_fresh = float(snapshot.get('raw_age', float('inf'))) < 1.
                control_fresh = float(snapshot.get('controller_age', float('inf'))) < 1.
                if raw_fresh and (control_fresh or not self._metadata.control_compatible):
                    self._reloading = False
            except (TypeError, ValueError):
                pass
        names = list(snapshot.get('names', []))
        positions = list(snapshot.get('positions', []))
        raw_sequence = snapshot.get('raw_sequence')
        try:
            raw_fresh = float(snapshot.get('raw_age', float('inf'))) < .75
        except (TypeError, ValueError):
            raw_fresh = False
        new_raw_sample = raw_sequence is None or raw_sequence != self._last_monitor_raw_sequence
        if raw_fresh and new_raw_sample and names and len(names) == len(positions):
            self.monitor.set_joint_state(names, positions, snapshot.get('velocities', []), snapshot.get('efforts', []))
            self._last_monitor_raw_sequence = raw_sequence
        desired = snapshot.get('desired', [])
        if isinstance(desired, dict):
            self.monitor.set_desired(desired.get('names', names), desired.get('positions', []))
        elif desired:
            self.monitor.set_desired(names, desired)
        xyz = snapshot.get('tcp_xyz') or snapshot.get('ee_xyz')
        if xyz and len(xyz) == 3:
            self.tcp_readout.setText('TCP  X {:+.3f}   Y {:+.3f}   Z {:+.3f} m'.format(*xyz))
        raw_age, ctrl_age = snapshot.get('raw_age'), snapshot.get('controller_age')
        if raw_age is not None:
            status = f'Feedback age {float(raw_age):.2f} s'
            if ctrl_age is not None:
                status += f'  ·  Controller age {float(ctrl_age):.2f} s'
            self.monitor.set_status(status)

    def _event_received(self, event):
        if isinstance(event, dict):
            message = str(event.get('message') or event.get('reason') or event.get('status') or json.dumps(event, ensure_ascii=True))
            category = str(event.get('level') or event.get('event') or 'MOTION').upper()
        else:
            message, category = str(event), 'MOTION'
        self.log(message, category)
        self.status_label.setText(message[:180])

    def capture_joints(self):
        positions = self._snapshot.get('positions', [])
        names = self._snapshot.get('names', [])
        values = dict(zip(names, positions))
        for index, name in enumerate(self._metadata.joint_names):
            if name in values:
                self.joint_table.cellWidget(0, index).setValue(values[name])
            elif len(positions) == len(self._metadata.joint_names):
                # The ROS controller publishes canonical names; URDF labels retain their own names.
                self.joint_table.cellWidget(0, index).setValue(positions[index])
        self.log('Current measured joints copied to targets.', 'COMMAND')

    def capture_pose(self):
        xyz = self._snapshot.get('tcp_xyz') or self._snapshot.get('ee_xyz')
        xyzw = self._snapshot.get('tcp_xyzw') or self._snapshot.get('ee_xyzw')
        if not xyz or not xyzw or len(xyz) != 3 or len(xyzw) != 4:
            return
        values = list(xyz) + [math.degrees(value) for value in _quaternion_rpy(xyzw)]
        for field, value in zip(self.pose_fields, values):
            field.setValue(value)
        self.log('Current measured TCP copied to the Cartesian target.', 'COMMAND')

    def move_joints(self):
        if not self.move_joints_button.isEnabled():
            return
        target = [self.joint_table.cellWidget(0, index).value() for index in range(self.joint_table.columnCount())]
        try:
            self.bridge.send_joint(target)
            self.log('Joint target published to ROS 2.', 'COMMAND')
        except Exception as error:
            self._error('Joint command failed', str(error))

    def move_pose(self):
        if not self.move_pose_button.isEnabled():
            return
        values = [field.value() for field in self.pose_fields]
        quat = _rpy_quaternion([math.radians(value) for value in values[3:]])
        try:
            self.bridge.send_pose(values[:3], quat)
            self.log('TCP target published to ROS 2 for IK and trajectory tracking.', 'COMMAND')
        except Exception as error:
            self._error('Cartesian command failed', str(error))

    def trace_square(self):
        if not self.square_button.isEnabled():
            return
        xyz = self._snapshot.get('tcp_xyz') or self._snapshot.get('ee_xyz')
        xyzw = self._snapshot.get('tcp_xyzw') or self._snapshot.get('ee_xyzw')
        if not xyz or not xyzw or len(xyz) != 3 or len(xyzw) != 4:
            self.log('No measured TCP is available yet.', 'COMMAND')
            return
        poses = []
        for dx, dy in [(0, 0), (.04, 0), (.04, .04), (0, .04), (0, 0)]:
            poses.append(([xyz[0] + dx, xyz[1] + dy, xyz[2]], list(xyzw)))
        try:
            self.bridge.send_path(poses)
            self.log('A 4 cm Cartesian TCP square was published to ROS 2.', 'COMMAND')
        except Exception as error:
            self._error('Path command failed', str(error))

    def cancel_motion(self):
        try:
            self.bridge.cancel()
            self.log('Motion cancellation requested.', 'COMMAND')
        except Exception as error:
            self._error('Cancellation failed', str(error))

    def _set_camera(self, yaw, pitch, distance):
        self.viewport.yaw, self.viewport.pitch, self.viewport.distance = yaw, pitch, distance
        # Limit ROS camera traffic while dragging the viewport.
        now = time.monotonic()
        if now - self._camera_last_sent > .035:
            self._camera_last_sent = now
            try:
                self.bridge.set_camera(yaw, pitch, distance)
            except Exception as error:
                self.log(str(error), 'CAMERA')

    def _camera_preset(self, name):
        yaw, pitch = {'Perspective': (45., -30.), 'Front': (90., -12.),
                      'Side': (0., -12.), 'Top': (0., -89.)}.get(name, (45., -30.))
        self._set_camera(yaw, pitch, 1.7)

    def _show_workspace(self):
        self.config_scroll.show()
        self.monitor_dock.show()
        self.monitor_dock.raise_()

    def _focus_robot(self):
        self.config_scroll.ensureWidgetVisible(self.robot_card)
        self.robot_path.setFocus()

    def _focus_tool(self):
        self.config_scroll.ensureWidgetVisible(self.tool_card)
        self.tool_preset.setFocus()

    def _show_monitor(self):
        self.monitor_dock.show()
        self.monitor_dock.raise_()

    def _show_log(self):
        self.log_dock.show()
        self.log_dock.raise_()

    def _choose_legacy_executable(self):
        path, _ = QFileDialog.getOpenFileName(self, 'Select Robot Monitor Executable', str(Path.home()),
                                            'Executable or Python script (*)', options=QFileDialog.DontUseNativeDialog)
        if path:
            self.legacy_executable.setText(path)

    def _choose_legacy_workspace(self):
        path = QFileDialog.getExistingDirectory(self, 'Select ROS Workspace', str(Path.home()), options=QFileDialog.DontUseNativeDialog)
        if path:
            self.legacy_workspace.setText(path)

    def _open_legacy_monitor(self):
        if not self.legacy_monitor:
            return
        path = self.legacy_executable.text().strip()
        if not path:
            self._error('Choose a Robot Monitor', 'Select the existing Robot Monitor executable first.')
            return
        try:
            self.legacy_monitor.start(path, self.legacy_workspace.text().strip())
        except Exception as error:
            self._error('Cannot open Robot Monitor', str(error))

    def log(self, message, category='INFO'):
        stamp = time.strftime('%H:%M:%S')
        # QTextEdit.append handles plain strings containing markup via HTML; escape explicitly.
        from html import escape
        self.event_log.append(f'<span style="color:#94a1b3">{stamp}</span>  <span style="color:#4877b2">[{escape(category)}]</span>  {escape(str(message))}')

    def _error(self, title, message):
        self.log(f'{title}: {message}', 'ERROR')
        QMessageBox.warning(self, title, message)

    def _about(self):
        QMessageBox.information(self, 'About Robot Workbench',
                                'Robot Workbench\n\nAn English desktop interface for the standalone LBR / PyBullet repository.\n\n'
                                'Robot imports, interchangeable end effectors, a live PyBullet camera, ROS 2 motion commands and a dockable Robot Monitor.\n\n'
                                'Motion pipeline: ROS 2 target → inverse kinematics / interpolation → LBR trajectory controller → PyBullet feedback.')

    def closeEvent(self, event):
        self._closing = True
        self._restart_requested = False
        self.spin_timer.stop()
        self.health_timer.stop()
        if self.legacy_monitor:
            self.legacy_monitor.stop()
        if self.backend.state() != QProcess.NotRunning:
            pgid = self._backend_pgid or int(self.backend.processId())
            self._signal_owned_process(pgid, signal.SIGINT)
            if not self.backend.waitForFinished(2500):
                self._signal_owned_process(pgid, signal.SIGTERM)
                if not self.backend.waitForFinished(1500):
                    self._signal_owned_process(pgid, signal.SIGKILL)
                    self.backend.waitForFinished(1000)
        self.bridge.close()
        self._runtime.cleanup()
        event.accept()


def main(args=None):
    parser = argparse.ArgumentParser(description='Robot Workbench desktop UI')
    parser.add_argument('--project', help='Robot workbench project JSON file')
    parser.add_argument('--start', action='store_true', help='Start the configured simulation on opening')
    options, qt_args = parser.parse_known_args(args)
    app = QApplication([sys.argv[0]] + qt_args)
    app.setApplicationName('Robot Workbench')
    app.setOrganizationName('LBR PyBullet ROS')
    app.setFont(QFont('DejaVu Sans', 9))
    try:
        window = WorkbenchWindow(project_path=options.project)
    except Exception as error:
        QMessageBox.critical(None, 'Robot Workbench', str(error))
        return 1
    window.show()
    if options.start:
        QTimer.singleShot(400, window.start_backend)
    return app.exec_()


if __name__ == '__main__':
    sys.exit(main())
