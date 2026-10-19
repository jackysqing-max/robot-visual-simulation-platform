#!/usr/bin/env python3
"""Exercise both ROS motion demos with a real PyBullet X11 GUI.

Run after sourcing /opt/ros/humble/setup.bash and install/local_setup.bash.
Captures only the newly created PyBullet window, never the full desktop.
Requires X11 access, xwininfo, xprop, xwd, and Pillow (for PNG conversion).
"""

import argparse
import ctypes
import ctypes.util
import json
import os
from pathlib import Path
import re
import signal
import struct
import subprocess
import time

import rclpy

from lbr_pybullet_tutorial.demo import Demo


def bullet_windows():
    """Return only Bullet window IDs without recording unrelated window titles."""
    tree = subprocess.check_output(['xwininfo', '-root', '-tree'], text=True)
    return {
        match.group(1): match.group(2)
        for match in re.finditer(r'^\s*(0x[0-9a-fA-F]+) "([^"]*)"', tree, re.M)
        if 'bullet' in match.group(2).lower()
    }


def own_window(previous, launch_pid, timeout=20):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        candidates = {key: value for key, value in bullet_windows().items()
                      if key not in previous}
        for window, title in candidates.items():
            prop = subprocess.run(['xprop', '-id', window, '_NET_WM_PID'],
                                  capture_output=True, text=True, check=False).stdout
            pid_match = re.search(r'=\s*(\d+)', prop)
            pid = int(pid_match.group(1)) if pid_match else None
            if pid is not None:
                try:
                    # launch starts a dedicated process group inherited by its nodes.
                    if os.getpgid(pid) != launch_pid:
                        continue
                except ProcessLookupError:
                    continue
            return {'id': window, 'title': title, 'pid': pid,
                    'identified_by': 'new Bullet window and process group' if pid else
                                     'new Bullet window after isolated launch'}
        time.sleep(0.1)
    raise RuntimeError('No newly created PyBullet GUI window appeared')


def xwd_to_png(source, destination):
    """Decode the TrueColor XWD layout emitted by common X11 desktops."""
    from PIL import Image

    payload = source.read_bytes()
    header = struct.unpack('>25I', payload[:100])
    (header_size, version, pixmap_format, depth, width, height, xoffset,
     byte_order, bitmap_unit, bitmap_order, bitmap_pad, bits_per_pixel,
     bytes_per_line, visual_class, red_mask, green_mask, blue_mask,
     bits_per_rgb, colormap_entries, ncolors, window_width, window_height,
     window_x, window_y, border_width) = header
    if version != 7 or pixmap_format != 2 or bits_per_pixel not in (24, 32):
        raise RuntimeError(f'Unsupported XWD image format: version={version}, '
                           f'format={pixmap_format}, bits_per_pixel={bits_per_pixel}')
    if (red_mask, green_mask, blue_mask) != (0xff0000, 0xff00, 0xff):
        raise RuntimeError('Unsupported XWD TrueColor masks; raw XWD remains available')
    pixels = payload[header_size + ncolors * 12:]
    if bits_per_pixel == 32:
        rawmode = 'BGRX' if byte_order == 0 else 'XRGB'
    else:
        rawmode = 'BGR' if byte_order == 0 else 'RGB'
    frame = Image.frombytes('RGB', (width, height), pixels, 'raw', rawmode,
                            bytes_per_line, 1)
    frame.save(destination)
    return {'width': width, 'height': height, 'depth': depth}


def capture(window, output, name):
    # Raise only our own simulation window so the X11 readback is unobscured.
    x11 = ctypes.cdll.LoadLibrary(ctypes.util.find_library('X11'))
    x11.XOpenDisplay.restype = ctypes.c_void_p
    x11.XOpenDisplay.argtypes = [ctypes.c_char_p]
    x11.XRaiseWindow.argtypes = [ctypes.c_void_p, ctypes.c_ulong]
    x11.XSync.argtypes = [ctypes.c_void_p, ctypes.c_int]
    x11.XCloseDisplay.argtypes = [ctypes.c_void_p]
    display = x11.XOpenDisplay(None)
    if not display:
        raise RuntimeError('XOpenDisplay failed')
    try:
        x11.XRaiseWindow(display, int(window['id'], 16))
        x11.XSync(display, 0)
    finally:
        x11.XCloseDisplay(display)
    time.sleep(0.2)
    raw_path = output / f'{name}.xwd'
    subprocess.run(['xwd', '-id', window['id'], '-silent', '-out', str(raw_path)],
                   check=True, timeout=10)
    dimensions = xwd_to_png(raw_path, output / f'{name}.png')
    return {**dimensions, 'png': f'{name}.png', 'xwd': f'{name}.xwd'}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--domain', type=int, default=93)
    parser.add_argument('--output', type=Path, default=Path('artifacts/gui'))
    parser.add_argument('--package', default='lbr_pybullet_tutorial')
    parser.add_argument('--launch', default='tutorial.launch.py')
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    if not os.environ.get('DISPLAY'):
        parser.error('DISPLAY is required for the PyBullet GUI test')
    os.environ['ROS_DOMAIN_ID'] = str(args.domain)
    previous = bullet_windows()
    node = None
    report = {'domain': args.domain, 'display': os.environ['DISPLAY'],
              'gui': True, 'checks': {}, 'screenshots': {}}
    with (args.output / 'launch.log').open('w') as log:
        process = subprocess.Popen(
            ['ros2', 'launch', args.package, args.launch, 'gui:=true'],
            stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            window = own_window(previous, process.pid)
            report['window'] = window
            rclpy.init()
            node = Demo()
            node.wait(lambda: node.q is not None and node.ready_seen, 30)
            report['screenshots']['initial'] = capture(window, args.output, 'initial')
            for mode in ('joint', 'cartesian'):
                measurement = node.run(mode, 30)
                (args.output / f'{mode}.json').write_text(
                    json.dumps(measurement, indent=2) + '\n')
                report['checks'][mode] = {key: value for key, value in measurement.items()
                                          if key not in ('samples', 'events')}
                if not measurement['passed']:
                    raise AssertionError({key: measurement.get(key) for key in (
                        'mode', 'result', 'sample_count', 'final_joint_error_rad',
                        'final_position_error_m', 'final_orientation_error_rad',
                        'max_position_tracking_error_m', 'max_actual_path_deviation_m')})
                report['screenshots'][mode] = capture(window, args.output, mode)
            report['passed'] = True
        except BaseException as error:
            report['passed'] = False
            report['error'] = repr(error)
            raise
        finally:
            (args.output / 'summary.json').write_text(json.dumps(report, indent=2) + '\n')
            if node is not None:
                node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
                try:
                    process.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    # This dedicated group contains only this test's ROS launch.
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait()
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
