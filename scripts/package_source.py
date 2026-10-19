#!/usr/bin/env python3
"""Export relocatable source, documentation and evidence, excluding local builds."""
import argparse
import hashlib
from pathlib import Path
import zipfile


def main():
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=root.parent/(root.name+'.zip'))
    args = parser.parse_args()
    output = args.output.resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    excluded = {'.git', 'build', 'install', 'log', 'artifacts', '__pycache__', '.pytest_cache', '.venv'}
    count = 0
    with zipfile.ZipFile(output, 'w', zipfile.ZIP_DEFLATED) as archive:
        for path in sorted(root.rglob('*')):
            relative = path.relative_to(root)
            if (not path.is_file() or path.resolve() == output or path.suffix in ('.xwd', '.pyc')
                    or any(p in excluded or p.endswith('.egg-info') for p in relative.parts)):
                continue
            archive.write(path, Path(root.name)/relative)
            count += 1
    with zipfile.ZipFile(output) as archive:
        if archive.testzip() is not None:
            raise RuntimeError('archive integrity check failed')
    checksum = hashlib.sha256(output.read_bytes()).hexdigest()
    output.with_suffix(output.suffix+'.sha256').write_text(f'{checksum}  {output.name}\n')
    print(f'{output}: {count} files, {output.stat().st_size} bytes\nSHA256 {checksum}')


if __name__ == '__main__':
    main()
