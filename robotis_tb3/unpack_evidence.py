"""Expand published gzip JSON evidence; leave existing expanded files unchanged."""
import gzip
from pathlib import Path


def unpack(root=None):
    root=Path(root) if root is not None else Path(__file__).parent/'evidence'
    count=0
    for source in root.rglob('*.json.gz'):
        destination=source.with_suffix('')
        if not destination.exists():
            destination.write_bytes(gzip.decompress(source.read_bytes()))
            count+=1
    return count


if __name__=='__main__':
    print(f'Expanded {unpack()} JSON evidence files')
