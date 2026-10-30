"""Fetch only the recursively referenced benchmark assets, checking Git LFS hashes."""
from concurrent.futures import ThreadPoolExecutor, as_completed
from pathlib import Path
from urllib.request import build_opener, ProxyHandler
urlopen = build_opener(ProxyHandler({})).open
import hashlib
import json
import re
import time

from pxr import UsdUtils

from policies.roboapi.config import REPO, WORK
REV = 'b6cfcf6ccd2113c4cd1216d35b33024a7d5691da'
BASE = f'https://media.githubusercontent.com/media/NVLabs/RoboLab/{REV}/'
CATALOG = json.loads((REPO/'robolab/tasks/_metadata/task_metadata.json').read_text())
assert len(CATALOG) == 120
ROOTS = ['assets/scenes/'+name for name in sorted({item['scene'] for item in CATALOG})] + ['assets/backgrounds/default/home_office.exr']


def process(path):
    path = path.resolve()
    if not path.is_relative_to(REPO):
        raise ValueError(f'Reference leaves repository: {path}')
    if not path.is_file():
        return [], {'path': str(path), 'status': 'missing'}
    with path.open('rb') as f:
        header = f.read(256)
    pointer = header.startswith(b'version https://git-lfs.github.com/spec/v1')
    if pointer:
        oid = re.search(rb'oid sha256:([0-9a-f]+)', header).group(1).decode()
        size = int(re.search(rb'size (\d+)', header).group(1))
        cached = REPO / '.git/lfs/objects' / oid[:2] / oid[2:4] / oid
        if cached.is_file() and cached.stat().st_size == size:
            data = cached.read_bytes()
        else:
            url = BASE + path.relative_to(REPO).as_posix()
            for attempt in range(4):
                try:
                    with urlopen(url, timeout=120) as response:
                        data = response.read()
                    break
                except Exception:
                    if attempt == 3:
                        raise
                    time.sleep(2 ** attempt)
        if len(data) != size or hashlib.sha256(data).hexdigest() != oid:
            raise ValueError(f'Git LFS hash/size mismatch: {path}')
        temporary = path.with_name(path.name + '.asset-download-partial')
        temporary.write_bytes(data)
        temporary.replace(path)
        if not cached.exists():
            cached.parent.mkdir(parents=True, exist_ok=True)
            temporary_cache = cached.with_suffix('.manual-download-partial')
            temporary_cache.write_bytes(data)
            temporary_cache.replace(cached)
        print(f'Fetched {path.relative_to(REPO)} {size} bytes SHA256 verified', flush=True)
    refs = []
    if path.suffix.lower() in {'.usd', '.usda', '.usdc'}:
        refs = [r for group in UsdUtils.ExtractExternalReferences(str(path)) for r in group]
    elif path.suffix.lower() == '.mdl':
        refs = re.findall(r'texture_\w+\("([^"]+)"', path.read_text())
    resolved, remote_refs = [], []
    for ref in refs:
        if ref.endswith('.mdl') and '/' not in ref:
            continue  # Built-in modules resolved by Isaac Sim's MDL search path.
        if '://' in ref:
            remote_refs.append(ref)
            if '/Materials/' in ref:
                resolved.append(REPO / 'assets/materials' / ref.split('/Materials/', 1)[1])
            continue
        resolved.append((path.parent / ref).resolve())
    return resolved, {'path': str(path), 'status': 'ok', 'size': path.stat().st_size, 'sha256': hashlib.sha256(path.read_bytes()).hexdigest(), 'remote_refs': remote_refs}


pending = {REPO / p for p in ROOTS}
seen, records = set(), []
while pending:
    batch = pending - seen
    if not batch:
        break
    seen.update(batch)
    pending = set()
    with ThreadPoolExecutor(max_workers=4) as pool:
        futures = {pool.submit(process, p): p for p in batch}
        for future in as_completed(futures):
            refs, record = future.result()
            records.append(record)
            pending.update(refs)

# Repair one missing upstream texture reference from an identical official file.
supplement_src = REPO / 'assets/fixtures/textures/T_Plastic_Normal_1k.png'
supplement_dst = REPO / 'assets/objects/vomp/container_e05/textures/T_Plastic_Normal_1k.png'
process(supplement_src)
expected_texture = '017086a4fb5d697900eaa4f52a4918ad7d559405dc733e8af3b090efc5875300'
assert hashlib.sha256(supplement_src.read_bytes()).hexdigest() == expected_texture
supplement_dst.parent.mkdir(parents=True, exist_ok=True)
supplement_dst.write_bytes(supplement_src.read_bytes())
WORK.mkdir(parents=True, exist_ok=True)
(WORK/'full120_asset_supplements.json').write_text(json.dumps({
    'missing_referenced_file': str(supplement_dst),
    'restored_from_official_repository_file': str(supplement_src),
    'sha256': expected_texture,
    'evidence': 'Copied the identical official texture; authored USD remains unchanged.',
}, indent=2)+'\n')

manifest = {'revision': REV, 'roots': ROOTS, 'files': sorted(records, key=lambda r: r['path']),
            'missing': [r['path'] for r in records if r['status'] != 'ok']}
(WORK/'roboapi_full120_asset_manifest.json').write_text(json.dumps(manifest, indent=2))
print(json.dumps({'file_count': len(records), 'missing': manifest['missing']}, indent=2), flush=True)
