"""Optional X11 container for an owned original C++ Robot Monitor subprocess."""

import ctypes
import ctypes.util
import os
import re
import shlex
import shutil
import signal
import subprocess
import tempfile
import time
from pathlib import Path

from PyQt5 import QtCore, QtGui, QtWidgets


class LegacyMonitorWidget(QtWidgets.QWidget):
    """Embed only the Robot Monitor process started by this widget.

    The native monitor remains usable without X11, the old workspace, Qt 6 or
    robot_control_msgs. ``start`` reports missing optional dependencies in English
    through ``status_changed`` and returns False without affecting the parent UI.
    """

    status_changed = QtCore.pyqtSignal(str)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.layout = QtWidgets.QVBoxLayout(self)
        self.layout.setContentsMargins(0, 0, 0, 0)
        self.message = QtWidgets.QLabel(
            'Original Robot Monitor is optional. Select its executable and ROS workspace to embed it.\n'
            'Requires an X11 desktop and the original seven-joint LBR monitor package.')
        self.message.setWordWrap(True)
        self.message.setMargin(12)
        self.layout.addWidget(self.message)
        self.container = None
        self.foreign_window = None
        self.process = None
        self.bridge = None
        self._temporary = None
        self._log = None
        self._deadline = None
        self._embedded = False
        self._pending_window = None
        self.timer = QtCore.QTimer(self)
        self.timer.setInterval(250)
        self.timer.timeout.connect(self._poll)
        app = QtWidgets.QApplication.instance()
        if app is not None:
            app.aboutToQuit.connect(self.stop)

    def _status(self, message):
        self.message.setText(message)
        self.status_changed.emit(message)

    def start(self, executable, workspace):
        self.stop()
        if not os.environ.get('DISPLAY') or QtGui.QGuiApplication.platformName() != 'xcb':
            self._status('Embedding the original Robot Monitor requires an X11 (xcb) desktop. '
                         'Use the built-in Robot Monitor on this platform.')
            return False
        if not shutil.which('xwininfo') or not shutil.which('xprop'):
            self._status('Window embedding requires xwininfo and xprop (package: x11-utils).')
            return False
        executable = Path(executable).expanduser().resolve()
        workspace = Path(workspace).expanduser().resolve()
        setup = workspace / 'install' / 'local_setup.bash'
        if not executable.is_file() or not os.access(executable, os.X_OK):
            self._status(f'Robot Monitor executable is unavailable: {executable}')
            return False
        if not setup.is_file():
            self._status(f'Original ROS workspace setup is unavailable: {setup}')
            return False
        bridge = Path(__file__).with_name('legacy_monitor_bridge.py')
        if not bridge.is_file():
            self._status(f'Legacy monitor adapter is unavailable: {bridge}')
            return False
        try:
            self._temporary = tempfile.TemporaryDirectory(prefix='lbr-robot-monitor-')
            self._log = open(Path(self._temporary.name) / 'monitor.log', 'ab', buffering=0)
            environment = os.environ.copy()
            # OpenCV may inject its incompatible Qt 5 plugin directory into the
            # parent environment; the original monitor is a separate Qt 6 binary.
            environment.pop('QT_QPA_PLATFORM_PLUGIN_PATH', None)
            environment.pop('QT_QPA_FONTDIR', None)
            environment['QT_QPA_PLATFORM'] = 'xcb'
            prefix = f'source /opt/ros/humble/setup.bash && source {shlex.quote(str(setup))} && exec '
            options = dict(cwd=self._temporary.name, env=environment, stdin=subprocess.DEVNULL,
                           stdout=self._log, stderr=subprocess.STDOUT, start_new_session=True)
            self.process = subprocess.Popen(
                ['bash', '--noprofile', '--norc', '-c', prefix + shlex.quote(str(executable)) +
                 ' --ros-args -r robot_states:=/workbench/robot_states'], **options)
            self.bridge = subprocess.Popen(
                ['bash', '--noprofile', '--norc', '-c', prefix + '/usr/bin/python3 ' +
                 shlex.quote(str(bridge))], **options)
        except (OSError, ValueError) as exc:
            self.stop()
            self._status(f'Could not start the original Robot Monitor: {exc}')
            return False
        self._deadline = time.monotonic() + 12.0
        self._status('Starting original Robot Monitor and read-only ROS adapter...')
        self.timer.start()
        return True

    def _find_owned_window(self):
        """Select a titled X11 window only when its PID matches our own child."""
        try:
            result = subprocess.run(['xwininfo', '-root', '-tree'], capture_output=True,
                                    text=True, timeout=0.5, check=False)
            for line in result.stdout.splitlines():
                if '"Robot Monitor"' not in line:
                    continue
                match = re.match(r'\s*(0x[0-9a-fA-F]+)\s', line)
                if not match:
                    continue
                window_id = match.group(1)
                prop = subprocess.run(['xprop', '-id', window_id, '_NET_WM_PID'],
                                      capture_output=True, text=True, timeout=0.5, check=False)
                pid = re.search(r'=\s*(\d+)\s*$', prop.stdout)
                if pid is not None and int(pid.group(1)) == self.process.pid:
                    return int(window_id, 16)
        except (OSError, subprocess.TimeoutExpired):
            pass
        return None

    @staticmethod
    def _x11_window_operation(window_id, parent_id=None):
        """Withdraw a managed top-level before making it a native child.

        Mutter may otherwise immediately put the original Qt 6 window back into
        a desktop frame after Qt 5 creates its foreign-window container.
        """
        library = ctypes.util.find_library('X11')
        if not library:
            raise RuntimeError('The X11 client library is unavailable')
        xlib = ctypes.CDLL(library)
        display_pointer = ctypes.c_void_p
        window_type = ctypes.c_ulong
        xlib.XOpenDisplay.argtypes = [ctypes.c_char_p]
        xlib.XOpenDisplay.restype = display_pointer
        xlib.XCloseDisplay.argtypes = [display_pointer]
        xlib.XDefaultScreen.argtypes = [display_pointer]
        xlib.XDefaultScreen.restype = ctypes.c_int
        xlib.XWithdrawWindow.argtypes = [display_pointer, window_type, ctypes.c_int]
        xlib.XReparentWindow.argtypes = [display_pointer, window_type, window_type,
                                       ctypes.c_int, ctypes.c_int]
        xlib.XMapWindow.argtypes = [display_pointer, window_type]
        xlib.XSync.argtypes = [display_pointer, ctypes.c_int]
        xlib.XSetErrorHandler.argtypes = [ctypes.c_void_p]
        xlib.XSetErrorHandler.restype = ctypes.c_void_p
        display = xlib.XOpenDisplay(None)
        if not display:
            raise RuntimeError('Could not connect to the X11 display')
        errors = []
        handler_type = ctypes.CFUNCTYPE(ctypes.c_int, display_pointer, ctypes.c_void_p)

        @handler_type
        def on_error(display_arg, event):
            errors.append(True)
            return 0

        previous = xlib.XSetErrorHandler(ctypes.cast(on_error, ctypes.c_void_p))
        try:
            if parent_id is None:
                xlib.XWithdrawWindow(display, window_id, xlib.XDefaultScreen(display))
            else:
                xlib.XReparentWindow(display, window_id, parent_id, 0, 0)
                xlib.XMapWindow(display, window_id)
            xlib.XSync(display, 0)
        finally:
            xlib.XSetErrorHandler(previous)
            xlib.XCloseDisplay(display)
        if errors:
            raise RuntimeError('The X11 window operation was rejected')

    def _log_tail(self):
        if self._temporary is None:
            return ''
        try:
            with open(Path(self._temporary.name) / 'monitor.log', 'rb') as stream:
                stream.seek(max(0, stream.seek(0, 2) - 2000))
                return stream.read().decode('utf-8', errors='replace').strip()
        except OSError:
            return ''

    def _poll(self):
        for label, process in (('Robot Monitor', self.process), ('ROS adapter', self.bridge)):
            if process is None or process.poll() is not None:
                tail = self._log_tail()
                self.stop()
                self._status(f'{label} exited. Check the original workspace dependencies.\n{tail}')
                return
        if self._embedded:
            return
        window_id = self._pending_window
        if window_id is None:
            discovered = self._find_owned_window()
            if discovered is not None:
                try:
                    self._x11_window_operation(discovered)
                except (OSError, RuntimeError) as exc:
                    self.stop()
                    self._status(f'Could not prepare the original Robot Monitor window: {exc}')
                    return
                self._pending_window = discovered
                # Give the desktop window manager one event-loop cycle to
                # release its frame before Qt takes ownership of embedding.
                return
        if window_id is not None:
            self.foreign_window = QtGui.QWindow.fromWinId(window_id)
            if self.foreign_window is None:
                self.stop()
                self._status('Qt could not attach the original Robot Monitor window.')
                return
            self.container = QtWidgets.QWidget.createWindowContainer(self.foreign_window, self)
            self.container.setAttribute(QtCore.Qt.WA_NativeWindow)
            self.container.setMinimumSize(300, 250)
            self.container.setFocusPolicy(QtCore.Qt.StrongFocus)
            self.layout.addWidget(self.container, 1)
            self.container.show()
            try:
                self._x11_window_operation(window_id, int(self.container.winId()))
            except (OSError, RuntimeError) as exc:
                self.stop()
                self._status(f'Could not embed the original Robot Monitor window: {exc}')
                return
            self._embedded = True
            self._status('Original Robot Monitor embedded | Seven-joint LBR feedback | '
                         'Waiting for /iiwa7/joint_states and controller state')
            return
        if time.monotonic() > self._deadline:
            tail = self._log_tail()
            self.stop()
            self._status('Timed out waiting for the original Robot Monitor X11 window.\n' + tail)

    def stop(self):
        self.timer.stop()
        # Release the foreign window wrapper while the owned X11 window still
        # exists. Deleting it after the child exits makes Qt try to reparent a
        # dead window and produces avoidable BadWindow errors on shutdown.
        if self.container is not None:
            self.container.hide()
            if self.foreign_window is not None:
                self.foreign_window.setParent(None)
            self.layout.removeWidget(self.container)
            self.container.deleteLater()
            QtCore.QCoreApplication.sendPostedEvents(self.container, QtCore.QEvent.DeferredDelete)
            self.container = None
        self.foreign_window = None
        # Each process is the leader of a session created by this widget. No
        # process-name matching or global ROS shutdown is used here.
        processes = [process for process in (self.bridge, self.process) if process is not None]
        for process in processes:
            if process.poll() is None:
                try:
                    os.killpg(process.pid, signal.SIGTERM)
                except ProcessLookupError:
                    pass
        for process in processes:
            try:
                process.wait(timeout=0.5)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(process.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                process.wait(timeout=0.5)
        self.process = self.bridge = None
        self._embedded = False
        self._pending_window = None
        if self._log is not None:
            self._log.close()
            self._log = None
        if self._temporary is not None:
            self._temporary.cleanup()
            self._temporary = None

    def closeEvent(self, event):
        self.stop()
        super().closeEvent(event)
