"""Embeddable, read-only Robot Monitor using standard ROS joint-state values.

The widget deliberately has no ROS executor or robot command publisher. Its owner
delivers snapshots on the Qt GUI thread, which also makes it useful in standalone
model viewers and in tests without a running ROS graph.
"""

import csv
import math
import time
from collections import deque
from dataclasses import dataclass

from PyQt5 import QtCore, QtGui, QtWidgets


COLORS = ('#57b9ff', '#56d4ac', '#f1c56c', '#ec90c7', '#a799ff', '#f28e76',
          '#83d9e6', '#bec77c', '#d3a1ed', '#a4b4c9')


def _finite(values, count):
    """Return finite values where supplied, NaN for unavailable measurements."""
    result = [float('nan')] * count
    for index, value in enumerate(list(values)[:count]):
        try:
            number = float(value)
            if math.isfinite(number):
                result[index] = number
        except (ValueError, TypeError, OverflowError):
            pass
    return tuple(result)


@dataclass(frozen=True)
class JointSample:
    elapsed: float
    position: tuple
    velocity: tuple
    effort: tuple
    desired: tuple
    error: tuple


class JointPlot(QtWidgets.QWidget):
    """Small bounded-history plot without a Qt Charts or plotting dependency."""

    def __init__(self, title, field, unit, monitor):
        super().__init__(monitor)
        self.title, self.field, self.unit, self.monitor = title, field, unit, monitor
        self.setMinimumSize(200, 100)
        self.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)

    def paintEvent(self, event):
        painter = QtGui.QPainter(self)
        painter.setRenderHint(QtGui.QPainter.Antialiasing)
        painter.fillRect(self.rect(), QtGui.QColor('#101b2c'))
        painter.setPen(QtGui.QColor('#d6e1ef'))
        font = painter.font()
        font.setPointSize(9)
        font.setBold(True)
        painter.setFont(font)
        painter.drawText(12, 20, self.title)
        font.setPointSize(8)
        font.setBold(False)
        painter.setFont(font)
        painter.setPen(QtGui.QColor('#8394aa'))
        painter.drawText(self.rect().adjusted(0, 5, -12, 0), QtCore.Qt.AlignTop | QtCore.Qt.AlignRight,
                         self.unit)
        bounds = QtCore.QRectF(55, 32, max(30, self.width() - 67), max(35, self.height() - 58))
        samples = self.monitor.visible_samples()
        latest = samples[-1].elapsed if samples else 0.0
        x_end = max(self.monitor.time_window, latest)
        x_start = x_end - self.monitor.time_window
        values = [value for sample in samples for value in getattr(sample, self.field)
                  if math.isfinite(value)]
        low, high = (min(values), max(values)) if values else (-1.0, 1.0)
        margin = max((high - low) * 0.12, 0.005 if self.field != 'effort' else 0.05)
        low, high = low - margin, high + margin
        painter.setPen(QtGui.QPen(QtGui.QColor('#253449'), 1))
        for step in range(4):
            fraction = step / 3.0
            y = bounds.top() + bounds.height() * fraction
            painter.drawLine(QtCore.QPointF(bounds.left(), y), QtCore.QPointF(bounds.right(), y))
            painter.setPen(QtGui.QColor('#8394aa'))
            painter.drawText(QtCore.QRectF(0, y - 8, 48, 16), QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter,
                             f'{high - fraction * (high - low):.3g}')
            painter.setPen(QtGui.QPen(QtGui.QColor('#253449'), 1))
        painter.setPen(QtGui.QColor('#8394aa'))
        painter.drawText(QtCore.QRectF(bounds.left(), bounds.bottom() + 3, 60, 16),
                         QtCore.Qt.AlignLeft, f'{x_start:.1f} s')
        painter.drawText(QtCore.QRectF(bounds.right() - 60, bounds.bottom() + 3, 60, 16),
                         QtCore.Qt.AlignRight, f'{x_end:.1f} s')
        painter.save()
        painter.setClipRect(bounds)
        for joint in range(len(self.monitor.joint_names)):
            path = QtGui.QPainterPath()
            active = False
            for sample in samples:
                row = getattr(sample, self.field)
                value = row[joint] if joint < len(row) else float('nan')
                if not math.isfinite(value):
                    active = False
                    continue
                point = QtCore.QPointF(
                    bounds.left() + (sample.elapsed - x_start) / self.monitor.time_window * bounds.width(),
                    bounds.bottom() - (value - low) / (high - low) * bounds.height())
                if active:
                    path.lineTo(point)
                else:
                    path.moveTo(point)
                    active = True
            painter.setPen(QtGui.QPen(QtGui.QColor(COLORS[joint % len(COLORS)]), 1.5))
            painter.drawPath(path)
        painter.restore()
        if not values:
            painter.setPen(QtGui.QColor('#8394aa'))
            painter.drawText(bounds, QtCore.Qt.AlignCenter, 'Waiting for measurements')


class RobotMonitorWidget(QtWidgets.QWidget):
    """Dock-ready joint feedback table and position/velocity/effort/error plots.

    Call ``set_joint_state`` and ``set_desired`` from the GUI thread. Names may
    arrive in any order and can contain any number of joints. Missing velocity,
    effort and desired values remain unavailable, rather than being shown as 0.
    All plotted times use local monotonic time, independent of ROS clock resets.
    """

    def __init__(self, parent=None, history_size=1800):
        super().__init__(parent)
        self.joint_names = ()
        self.samples = deque(maxlen=max(2, int(history_size)))
        self.desired_by_name = {}
        self.time_window = 15.0
        self.started_at = time.monotonic()
        self.received_at = None
        self.sample_count = 0
        self.latest = None
        self._dirty = True
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(10, 8, 10, 8)
        layout.setSpacing(8)
        toolbar = QtWidgets.QHBoxLayout()
        self.connection = QtWidgets.QLabel('Waiting for joint state')
        self.connection.setObjectName('monitorConnection')
        toolbar.addWidget(self.connection, 1)
        toolbar.addWidget(QtWidgets.QLabel('History'))
        window = QtWidgets.QComboBox()
        window.addItems(['5 s', '15 s', '30 s', '60 s'])
        window.setCurrentText('15 s')
        window.currentTextChanged.connect(self._set_window)
        toolbar.addWidget(window)
        self.pause = QtWidgets.QCheckBox('Pause plots')
        toolbar.addWidget(self.pause)
        clear = QtWidgets.QPushButton('Clear')
        clear.clicked.connect(self.clear_history)
        toolbar.addWidget(clear)
        export = QtWidgets.QPushButton('Export CSV')
        export.clicked.connect(self._choose_export)
        toolbar.addWidget(export)
        layout.addLayout(toolbar)
        self.table = QtWidgets.QTableWidget(0, 7)
        self.table.setObjectName('robotMonitorTable')
        self.table.setHorizontalHeaderLabels(['Joint', 'Position', 'Desired', 'Error', 'Velocity',
                                             'Effort', 'Units'])
        self.table.verticalHeader().hide()
        self.table.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.Stretch)
        self.table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.table.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows)
        self.table.setAlternatingRowColors(True)
        self.table.setMinimumHeight(70)
        self.table.setMaximumHeight(120)
        layout.addWidget(self.table)
        legend_row = QtWidgets.QHBoxLayout()
        self.legend = QtWidgets.QLabel('Joint colors appear when feedback arrives')
        self.legend.setWordWrap(True)
        self.legend.setTextFormat(QtCore.Qt.RichText)
        legend_row.addWidget(self.legend, 1)
        self.error_label = QtWidgets.QLabel('Max |error|: unavailable')
        legend_row.addWidget(self.error_label)
        layout.addLayout(legend_row)
        self.plot_grid = QtWidgets.QGridLayout()
        self.plot_grid.setSpacing(8)
        self.plots = [JointPlot('Joint Position', 'position', 'rad / m', self),
                      JointPlot('Joint Velocity', 'velocity', 'rad/s / m/s', self),
                      JointPlot('Joint Effort', 'effort', 'Nm / N', self),
                      JointPlot('Tracking Error (actual - desired)', 'error', 'rad / m', self)]
        for index, plot in enumerate(self.plots):
            self.plot_grid.addWidget(plot, index // 2, index % 2)
        self._plot_columns = 2
        layout.addLayout(self.plot_grid, 1)
        self.status = QtWidgets.QLabel('Feedback only | No motion commands are sent by Robot Monitor')
        self.status.setWordWrap(True)
        self.status.setObjectName('monitorStatus')
        layout.addWidget(self.status)
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.refresh)
        self.timer.start(100)

    def set_joint_state(self, names, positions, velocities=(), efforts=(), stamp=None):
        """Ingest one JointState snapshot; ``stamp`` is accepted for API compatibility."""
        del stamp
        names = tuple(str(name) for name in names)
        positions = tuple(positions)
        if not names or len(set(names)) != len(names) or len(positions) != len(names):
            return False
        values = _finite(positions, len(names))
        if not all(math.isfinite(value) for value in values):
            return False
        if names != self.joint_names:
            if set(names) == set(self.joint_names):
                indices = [names.index(name) for name in self.joint_names]
                velocity = _finite(velocities, len(names))
                effort = _finite(efforts, len(names))
                values = tuple(values[index] for index in indices)
                velocities = tuple(velocity[index] for index in indices)
                efforts = tuple(effort[index] for index in indices)
                names = self.joint_names
            else:
                self.joint_names = names
                self.clear_history()
                self.table.setRowCount(len(names))
                self.table.setMaximumHeight(max(70, min(120, 35 + len(names) * 27)))
                from html import escape
                self.legend.setText(' &nbsp; '.join(
                    f'<span style="color:{COLORS[i % len(COLORS)]}">&#9679; {escape(name)}</span>'
                    for i, name in enumerate(names)))
        now = time.monotonic()
        self.received_at = now
        self.sample_count += 1
        desired = tuple(self.desired_by_name.get(name, float('nan')) for name in names)
        sample = JointSample(now - self.started_at, values, _finite(velocities, len(names)),
                             _finite(efforts, len(names)), desired,
                             tuple(actual - target for actual, target in zip(values, desired)))
        self.latest = sample
        if not self.pause.isChecked():
            self.samples.append(sample)
        self._dirty = True
        return True

    update_state = set_joint_state

    def set_desired(self, names, positions):
        names, positions = tuple(names), tuple(positions)
        if len(names) != len(positions) or len(set(names)) != len(names):
            return False
        values = _finite(positions, len(names))
        if not all(math.isfinite(value) for value in values):
            return False
        self.desired_by_name = dict(zip(names, values))
        return True

    def set_status(self, text):
        self.status.setText(str(text))

    def _set_window(self, text):
        self.time_window = float(text.split()[0])
        self._dirty = True

    def resizeEvent(self, event):
        super().resizeEvent(event)
        columns = 4 if self.width() >= 1000 else 2
        if columns != self._plot_columns:
            self._plot_columns = columns
            for index, plot in enumerate(self.plots):
                self.plot_grid.removeWidget(plot)
                self.plot_grid.addWidget(plot, index // columns, index % columns)

    def visible_samples(self):
        if not self.samples:
            return []
        threshold = self.samples[-1].elapsed - self.time_window
        return [sample for sample in self.samples if sample.elapsed >= threshold]

    def clear_history(self):
        self.samples.clear()
        self.started_at = time.monotonic()
        self._dirty = True

    def refresh(self):
        if self.received_at is not None:
            age = max(0.0, time.monotonic() - self.received_at)
            self.connection.setText(('Live' if age < 1.0 else 'Stale feedback') +
                                    f' | {len(self.joint_names)} joints | State age: {age:.1f} s')
            self.connection.setStyleSheet('color: #56d4ac;' if age < 1.0 else 'color: #f1c56c;')
        if not self._dirty:
            return
        self._dirty = False
        if self.latest is not None:
            for row, name in enumerate(self.joint_names):
                sample = self.latest
                values = (name, sample.position[row], sample.desired[row], sample.error[row],
                          sample.velocity[row], sample.effort[row], 'rad / m; Nm / N')
                for column, value in enumerate(values):
                    text = (f'{value:.4f}' if math.isfinite(value) else '\u2014') if isinstance(value, float) else str(value)
                    item = self.table.item(row, column)
                    if item is None:
                        item = QtWidgets.QTableWidgetItem()
                        self.table.setItem(row, column, item)
                    item.setText(text)
                    if column == 0:
                        item.setForeground(QtGui.QColor(COLORS[row % len(COLORS)]))
            errors = [abs(value) for value in self.latest.error if math.isfinite(value)]
            self.error_label.setText(f'Max |error|: {max(errors):.5f}' if errors else 'Max |error|: unavailable')
        for plot in self.plots:
            plot.update()

    def export_csv(self, path):
        """Export captured plot samples; unavailable fields are empty CSV cells."""
        with open(path, 'w', newline='', encoding='utf-8') as stream:
            writer = csv.writer(stream)
            writer.writerow(['elapsed_s', 'joint', 'position', 'desired', 'error', 'velocity', 'effort'])
            for sample in self.samples:
                for index, name in enumerate(self.joint_names):
                    values = [getattr(sample, field)[index]
                              for field in ('position', 'desired', 'error', 'velocity', 'effort')]
                    writer.writerow([f'{sample.elapsed:.6f}', name] +
                                    [value if math.isfinite(value) else '' for value in values])

    def _choose_export(self):
        path, _ = QtWidgets.QFileDialog.getSaveFileName(self, 'Export Robot Monitor Data',
                                                       'robot_monitor.csv', 'CSV files (*.csv)')
        if not path:
            return
        try:
            self.export_csv(path)
        except OSError as exc:
            QtWidgets.QMessageBox.warning(self, 'Export Failed', str(exc))

