#!/usr/bin/env python3
"""Exercise the actual desktop UI, owned ROS processes and Bullet camera.

Run after sourcing ROS and this repository's install/local_setup.bash.  Evidence
is captured from the window itself; no desktop-wide screenshot is taken.  Each
run requires a fresh output directory so failed evidence cannot be overwritten.
"""

import argparse
import json
import math
import os
from pathlib import Path
import sys
import time
import traceback
import xml.etree.ElementTree as ET


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", default="artifacts/workbench")
    parser.add_argument("--domain", type=int, default=95)
    parser.add_argument("--timeout", type=float, default=90.0)
    parser.add_argument("--skip-preview", action="store_true")
    parser.add_argument("--offscreen", action="store_true")
    options = parser.parse_args()
    output = Path(options.output).expanduser().resolve()
    if output.is_dir() and any(output.iterdir()):
        parser.error(f"Evidence directory is not empty: {output}; archive it or choose another --output")
    output.mkdir(parents=True, exist_ok=True)
    os.environ["ROS_DOMAIN_ID"] = str(options.domain)
    os.environ["ROS_LOCALHOST_ONLY"] = "1"
    os.environ.pop("LBR_PYBULLET_PROJECT", None)
    if options.offscreen or not os.environ.get("DISPLAY"):
        os.environ["QT_QPA_PLATFORM"] = "offscreen"

    # Prefer the source under review; owned ROS nodes still start through the
    # normal installed launch files and entry points.
    repository = Path(__file__).resolve().parents[1]
    sys.path.insert(0, str(repository / "src" / "lbr_pybullet_tutorial"))
    from PyQt5.QtCore import QProcess, Qt
    from PyQt5.QtTest import QTest
    from PyQt5.QtWidgets import QApplication, QFileDialog
    import pybullet_data
    from lbr_pybullet_tutorial.project_config import load_project, inspect_robot
    from lbr_pybullet_tutorial.workbench import WorkbenchWindow

    app = QApplication.instance() or QApplication(["workbench_smoke_test"])
    app.setQuitOnLastWindowClosed(False)
    window = WorkbenchWindow()
    window.show()
    events, errors = [], []
    image_count = [0]
    summary = dict(passed=False, ros_domain=options.domain,
                   qt_platform=os.environ.get("QT_QPA_PLATFORM", "desktop"), checks={})
    start = time.monotonic()

    def error_received(title, message):
        errors.append(dict(title=title, message=message))
        window.log(f"{title}: {message}", "ERROR")

    window._error = error_received  # Capture failures without a modal test hang.
    window.bridge.event.connect(lambda event: events.append(dict(event, observed_at=time.monotonic() - start)))
    window.bridge.image.connect(lambda frame: image_count.__setitem__(0, image_count[0] + 1))

    def progress(message):
        print(message, flush=True)

    def wait(label, predicate, timeout=None, event_start=None):
        deadline = time.monotonic() + (options.timeout if timeout is None else timeout)
        while time.monotonic() < deadline:
            app.processEvents()
            if errors:
                raise AssertionError(f"{label}: UI error: {errors[-1]}")
            if event_start is not None:
                failures = [event for event in events[event_start:]
                            if event.get("event") in ("rejected", "failed")]
                if failures:
                    raise AssertionError(f"{label}: motion failed: {failures[-1]}")
            if predicate():
                progress(f"PASS: {label}")
                return
            QTest.qWait(40)
        raise AssertionError(f"Timed out waiting for {label}; snapshot={window._snapshot}; events={events[-5:]}")

    def fresh(control=True):
        snapshot = window._snapshot
        return (window.backend.state() == QProcess.Running
                and float(snapshot.get("raw_age", math.inf)) < 0.6
                and (not control or float(snapshot.get("controller_age", math.inf)) < 0.6)
                and time.monotonic() - window._last_image < 1.0
                and image_count[0] > 0
                and not window._reloading)

    def event_seen(name, from_index=0):
        return any(event.get("event") == name for event in events[from_index:])

    def command_completed(from_index):
        accepted = [event.get("request_id") for event in events[from_index:]
                    if event.get("event") == "accepted"]
        return any(event.get("event") == "succeeded" and event.get("request_id") in accepted
                   for event in events[from_index:])

    def screenshot(name):
        window.monitor_dock.show()
        window.monitor_dock.raise_()
        app.processEvents()
        QTest.qWait(200)
        image = window.grab()
        assert not image.isNull() and image.width() >= 1100
        target = output / name
        assert image.save(str(target)), f"Could not save {target}"
        return str(target)

    def snapshot_record():
        snapshot = window._snapshot
        return {key: snapshot.get(key) for key in (
            "names", "positions", "ee_xyz", "ee_xyzw", "tcp_xyz", "tcp_xyzw", "raw_age", "controller_age")}

    try:
        progress("Starting the actual Robot Workbench backend on isolated ROS domain " + str(options.domain))
        window.start_button.click()
        wait("default live camera, raw feedback, LBR controller and ready event",
             lambda: fresh() and window.move_joints_button.isEnabled() and event_seen("ready"))
        assert window.viewport._frame is not None
        assert window.monitor_dock.isVisible()
        summary["checks"]["default_ready"] = snapshot_record()

        window.capture_joints_button.click()
        target = window.joint_table.cellWidget(0, 0)
        q_start = float(window._snapshot["positions"][0])
        target.setValue(target.value() + 0.06)
        q_goal = target.value()
        from_index = len(events)
        window.move_joints_button.click()
        wait("joint command accepted and succeeded through the UI",
             lambda: command_completed(from_index), event_start=from_index)
        wait("measured joint follows the UI target",
             lambda: abs(window._snapshot["positions"][0] - q_goal) < 0.015, timeout=10)
        assert abs(window._snapshot["positions"][0] - q_start) > 0.03
        summary["checks"]["joint_motion"] = dict(target=q_goal, measured=window._snapshot["positions"][0])
        summary["checks"]["default_screenshot"] = screenshot("default_joint_control.png")

        previous_pid = window._backend_pgid
        previous_images = image_count[0]
        event_mark = len(events)
        window.tool_preset.setCurrentIndex(window.tool_preset.findData("probe"))
        assert window._staged and not window.move_joints_button.isEnabled()
        assert window.tcp_xyz_fields[2].value() == 0.179
        window.apply_button.click()
        wait("probe reload starts a different owned process and receives fresh feedback",
             lambda: window._backend_pgid not in (None, previous_pid)
             and image_count[0] >= previous_images + 2 and fresh()
             and event_seen("ready", event_mark) and window.move_pose_button.isEnabled())
        snapshot = window._snapshot
        offset = math.dist(snapshot["ee_xyz"], snapshot["tcp_xyz"])
        assert abs(offset - 0.179) < 0.001, f"Wrong actual probe TCP offset: {offset}"
        summary["checks"]["probe_reload"] = dict(previous_pid=previous_pid,
                                                    current_pid=window._backend_pgid,
                                                    measured_tcp_offset_m=offset)
        window.control_tabs.setCurrentIndex(1)
        window.capture_pose_button.click()
        start_tcp = list(window._snapshot["tcp_xyz"])
        window.pose_fields[0].setValue(window.pose_fields[0].value() + 0.01)
        pose_goal = [field.value() for field in window.pose_fields[:3]]
        from_index = len(events)
        window.move_pose_button.click()
        wait("probe Cartesian TCP move accepted and succeeded through the UI",
             lambda: command_completed(from_index), event_start=from_index)
        wait("measured probe TCP reaches the 1 cm target",
             lambda: math.dist(window._snapshot["tcp_xyz"], pose_goal) < 0.004, timeout=10)
        assert math.dist(start_tcp, window._snapshot["tcp_xyz"]) > 0.005
        summary["checks"]["probe_tcp_motion"] = dict(target=pose_goal, measured=window._snapshot["tcp_xyz"])
        window._focus_tool()
        summary["checks"]["probe_screenshot"] = screenshot("probe_cartesian_control.png")

        profile = output / "probe_project.json"
        window.project_path = str(profile)
        window.save_button.click()
        saved = load_project(profile)
        assert saved.tool == "probe" and saved.tcp_xyz == [0.0, 0.0, 0.179]
        assert inspect_robot(saved).control_compatible
        original_dialog = QFileDialog.getOpenFileName
        QFileDialog.getOpenFileName = lambda *args, **kwargs: (str(profile), "")
        try:
            window.open_button.click()
        finally:
            QFileDialog.getOpenFileName = original_dialog
        assert window.project.tool == "probe" and window._staged
        assert not window.move_pose_button.isEnabled()
        summary["checks"]["saved_profile_reopened"] = str(profile)

        if not options.skip_preview:
            # Import an actual six-axis URDF to verify the unsupported-robot
            # preview path, including its real simulation camera.
            source = Path(pybullet_data.getDataPath()) / "kuka_iiwa" / "model.urdf"
            xml = ET.parse(source).getroot()
            xml.remove(xml.find("joint[@name='lbr_iiwa_joint_7']"))
            xml.remove(xml.find("link[@name='lbr_iiwa_link_7']"))
            for mesh in xml.findall(".//mesh"):
                mesh.set("filename", str(source.parent / mesh.get("filename")))
            imported = output / "six_axis_preview.urdf"
            imported.write_text(ET.tostring(xml, encoding="unicode"))
            window.robot_path.setText(str(imported))
            window._robot_path_edited()
            assert not window._metadata.control_compatible
            assert len(window._metadata.joint_names) == 6
            previous_pid, previous_images = window._backend_pgid, image_count[0]
            window.apply_button.click()
            wait("imported six-axis preview has a new process and a live camera",
                 lambda: window._backend_pgid not in (None, previous_pid)
                 and image_count[0] >= previous_images + 2 and fresh(control=False)
                 and len(window._snapshot.get("positions", [])) == 6)
            assert not window.move_joints_button.isEnabled()
            assert not window.move_pose_button.isEnabled()
            assert not window.square_button.isEnabled()
            summary["checks"]["six_axis_preview"] = snapshot_record()
            window._focus_robot()
            summary["checks"]["preview_screenshot"] = screenshot("imported_six_axis_preview.png")

        summary["passed"] = True
        progress("All Robot Workbench smoke checks passed.")
    except Exception as error:
        summary["error"] = str(error)
        summary["traceback"] = traceback.format_exc()
        progress("FAIL: " + str(error))
        traceback.print_exc()
        try:
            summary["failure_screenshot"] = screenshot("failure.png")
        except Exception:
            pass
    finally:
        summary["images_received"] = image_count[0]
        summary["elapsed_seconds"] = time.monotonic() - start
        summary["ui_errors"] = errors
        (output / "window.log").write_text(window.event_log.toPlainText(), encoding="utf-8")
        (output / "events.json").write_text(json.dumps(events, indent=2), encoding="utf-8")
        window.close()
        app.processEvents()
        summary["owned_backend_stopped"] = window.backend.state() == QProcess.NotRunning
        (output / "summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    return 0 if summary["passed"] and summary["owned_backend_stopped"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
