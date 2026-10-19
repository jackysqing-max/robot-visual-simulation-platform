#!/usr/bin/env python3
"""Verify the optional original monitor inside the real workbench on X11."""
import argparse
import json
import math
import os
from pathlib import Path
import re
import subprocess
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--executable', required=True)
    parser.add_argument('--workspace', required=True)
    parser.add_argument('--domain', type=int, default=98)
    parser.add_argument('--output', type=Path, default=Path('artifacts/workbench_legacy_x11'))
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    os.environ['ROS_DOMAIN_ID'] = str(args.domain)
    from PyQt5.QtWidgets import QApplication
    from PyQt5.QtTest import QTest
    from lbr_pybullet_tutorial.workbench import WorkbenchWindow
    from gui_smoke_test import xwd_to_png
    root = Path(__file__).resolve().parents[1]
    app = QApplication([])
    window = WorkbenchWindow(project_path=root/'examples/scalpel_project.json')
    window.show()
    def wait(predicate):
        deadline = time.monotonic()+25
        while time.monotonic() < deadline:
            app.processEvents()
            QTest.qWait(30)
            if predicate():
                return
        raise RuntimeError('Monitor integration timeout: '+window.event_log.toPlainText()[-2000:])
    report = {'passed': False}
    try:
        window.start_backend()
        wait(lambda: window.move_pose_button.isEnabled() and window.viewport._frame is not None)
        offset = math.dist(window._snapshot['ee_xyz'], window._snapshot['tcp_xyz'])
        assert abs(offset-0.12) < 0.001
        report['custom_obj_tool_tcp_offset_m'] = offset
        window.legacy_executable.setText(args.executable)
        window.legacy_workspace.setText(args.workspace)
        window.legacy_dock.show()
        window._open_legacy_monitor()
        wait(lambda: window.legacy_monitor._embedded)
        window.capture_joints()
        window.joint_table.cellWidget(0, 0).setValue(0.10)
        window.move_joints()
        until = time.monotonic()+3.0
        while time.monotonic() < until:
            app.processEvents()
            QTest.qWait(30)
        assert window.legacy_monitor.process.poll() is None
        assert window.legacy_monitor.bridge.poll() is None
        info = subprocess.check_output(['xwininfo', '-id', hex(int(window.legacy_monitor.foreign_window.winId())), '-tree'], text=True)
        parent = re.search(r'Parent window id:\s+(0x[0-9a-f]+)', info)
        assert parent and int(parent.group(1), 16) == int(window.legacy_monitor.container.winId())
        report['legacy_native_parent_is_ui_container'] = True
        # QWidget.grab excludes foreign native surfaces. Capture only our owned
        # top-level X11 window to include the actual embedded Qt6 chart window.
        raw = args.output/'legacy_monitor.xwd'
        subprocess.run(['xwd', '-id', str(int(window.winId())), '-silent', '-out', str(raw)], check=True)
        xwd_to_png(raw, args.output/'legacy_monitor.png')
        report['passed'] = True
    finally:
        (args.output/'window.log').write_text(window.event_log.toPlainText())
        window.close()
        app.processEvents()
        report['owned_backend_stopped'] = window.backend.state() == 0
        report['legacy_children_stopped'] = window.legacy_monitor.process is None and window.legacy_monitor.bridge is None
        (args.output/'summary.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
