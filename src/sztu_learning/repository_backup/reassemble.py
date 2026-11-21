#!/usr/bin/env python3
"""Reassemble and verify the complete upstream archive without network access."""
import argparse
import hashlib
import json
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    root = Path(__file__).resolve().parent
    manifest = json.loads((root/'archive_parts.json').read_text())
    output = args.output or root/'restored'/manifest['archive_name']
    output.parent.mkdir(parents=True, exist_ok=True)
    overall = hashlib.sha256()
    size = 0
    try:
        with output.open('xb') as destination:
            for part in manifest['parts']:
                digest = hashlib.sha256()
                with (root/part['name']).open('rb') as source:
                    for block in iter(lambda: source.read(4*1024*1024), b''):
                        destination.write(block)
                        digest.update(block)
                        overall.update(block)
                        size += len(block)
                if digest.hexdigest() != part['sha256']:
                    raise ValueError('Part checksum mismatch: '+part['name'])
            if size != manifest['size_bytes'] or overall.hexdigest() != manifest['sha256']:
                raise ValueError('Archive checksum mismatch')
    except FileExistsError:
        raise SystemExit('Output already exists; choose a new --output path.')
    except Exception:
        output.unlink(missing_ok=True)
        raise
    print('Verified:', output)
    print('SHA-256:', overall.hexdigest())


if __name__ == '__main__':
    main()
