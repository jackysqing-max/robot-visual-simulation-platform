#!/usr/bin/env python3
"""Export recorded controller/physics measurements as a standalone figure."""
import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('report', type=Path)
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    report = json.loads(args.report.read_text())
    samples = report['samples']
    elapsed = [s['time']-samples[0]['time'] for s in samples]
    fig, axes = plt.subplots(1, 3, figsize=(14, 4), constrained_layout=True)
    for i in range(7):
        line, = axes[0].plot(elapsed, [s['actual_joints'][i] for s in samples],
                             label=f'A{i+1}', linewidth=1.4)
        axes[0].plot(elapsed, [s['desired_joints'][i] for s in samples],
                     '--', color=line.get_color(), linewidth=0.7)
    axes[0].set(xlabel='Time (s)', ylabel='Joint position (rad)',
                title='Measured (solid) / desired (dashed)')
    axes[0].legend(ncol=4, fontsize=8)
    axes[1].plot(elapsed, [s['position_error_m']*1000 for s in samples])
    axes[1].set(xlabel='Time (s)', ylabel='Position tracking error (mm)',
                title='Measured FK relative to controller desired FK')
    vertices = report.get('path_vertices')
    if vertices:
        axes[2].plot([p[0]*1000 for p in vertices], [p[1]*1000 for p in vertices],
                     'k--', label='Requested path', linewidth=1)
    axes[2].plot([s['desired_xyz'][0]*1000 for s in samples],
                 [s['desired_xyz'][1]*1000 for s in samples], label='Desired FK')
    axes[2].plot([s['actual_xyz'][0]*1000 for s in samples],
                 [s['actual_xyz'][1]*1000 for s in samples], label='Measured FK', alpha=0.8)
    axes[2].set(xlabel='Base X (mm)', ylabel='Base Y (mm)', title='End effector path')
    axes[2].set_aspect('equal', adjustable='datalim')
    axes[2].legend(fontsize=8)
    for axis in axes:
        axis.grid(alpha=0.2)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.output, dpi=160)
    plt.close(fig)


if __name__ == '__main__':
    main()
