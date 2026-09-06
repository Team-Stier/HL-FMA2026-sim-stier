"""Validate actual reviewer coverage, preserve evidence, and create a local image index."""
import argparse
from collections import Counter
import hashlib
from html import escape
from html.parser import HTMLParser
import json
from pathlib import Path
import shutil

p = argparse.ArgumentParser(description=__doc__)
p.add_argument('--source', type=Path, required=True)
p.add_argument('--output', type=Path, required=True)
a = p.parse_args()
def read(name):
    return json.loads((a.source / name).read_text())
def sha(path):
    with path.open('rb') as f:
        return hashlib.file_digest(f, 'sha256').hexdigest()
manifest = read('atlas/manifest.json')
supplement = read('supplemental/manifest.json')
inventory = read('coverage-source.json')
no_error_decisions = {'context_only', 'aligned_visible', 'preserve_by_xodr_policy', 'source_boundary',
    'no_visible_conversion_error', 'clear_after_zoom', 'clear_visual', 'retain_xodr_unresolved_object',
    'clear_context_only', 'clear_source_consistent', 'source_semantics_unresolved_preserved',
    'source_unresolved_preserved', 'source_signal_placement_preserved', 'source_non_driving_stop_preserved'}
reviews = []
expected_page = {t['id']: t['page'] for t in manifest['tiles']}
for reviewer in ('root', 'geometry', 'signal', 'visualizer'):
    record = read(f'review-{reviewer}.json')
    viewed = set()
    for filename in record['files_actually_viewed']:
        path = Path(filename)
        if not path.is_absolute():
            path = a.source / path
        assert path.is_file(), str(path)
        viewed.add(path.resolve())
        if filename in record.get('files_sha256', {}):
            assert sha(path) == record['files_sha256'][filename]
    for row in record['tiles']:
        row = dict(row, reviewer=reviewer)
        if isinstance(row['page'], int):
            row['page'] = f"p{row['page']:03d}.jpg"
        if isinstance(row['observations'], str):
            row['observations'] = [row['observations']]
        assert row['observations'] and row['decision']
        assert row['page'] == expected_page[row['tile_id']]
        assert row['decision'] in no_error_decisions, 'Unresolved/new error decision: ' + row['decision']
        assert (a.source / 'atlas/pages' / row['page']).resolve() in viewed
        reviews.append(row)
counts = Counter(r['tile_id'] for r in reviews)
assert set(counts) == {t['id'] for t in manifest['tiles']}
assert all(n == 1 for n in counts.values()), 'Duplicate primary reviews'
assert {r['page'] for r in reviews} == {r['file'] for r in manifest['pages']}
by_id = {r['tile_id']: r for r in reviews}
road_results = [dict(id=r['id'], length_m=r['length_m'], tile_ids=r['tile_ids'],
                     all_tiles_reviewed=all(t in by_id for t in r['tile_ids'])) for r in inventory['roads']]
assert len(road_results) == 651 and all(r['all_tiles_reviewed'] for r in road_results)
extra = read('review-supplemental.json')
assert {r['tile_id'] for r in extra['tiles']} == {t['id'] for t in supplement['tiles']}
assert all(Path(f).is_file() for f in extra['files_actually_viewed'])
assert all(str((a.source/'supplemental/pages'/r['page']).resolve()) in extra['files_actually_viewed'] for r in extra['tiles'])
assert all(r['observations'] and r['decision'] == 'preserve_by_xodr_policy' for r in extra['tiles'])
assert len({i for t in manifest['tiles'] for i in t['native_lanelets']}) == 2845
assert len({i for t in manifest['tiles'] for i in t['cell_ids']}) == 94154
assert len({s['id'] for t in manifest['tiles'] + supplement['tiles'] for s in t['stoplines']}) == 751
for name in ('atlas', 'supplemental'):
    for part in ('tiles', 'pages'):
        shutil.copytree(a.source/name/part, a.output/name/part, dirs_exist_ok=True)
    shutil.copy2(a.source/name/'inputs.json', a.output/name/'inputs.json')
for path in a.source.iterdir():
    if path.suffix in ('.json', '.py'):
        shutil.copy2(path, a.output/path.name)
for data, name, rows in ((manifest, 'atlas', by_id), (supplement, 'supplemental', {r['tile_id']: r for r in extra['tiles']})):
    data['review_status'] = 'visually_reviewed_with_xodr_policy'
    for t in data['tiles']:
        t['review'] = rows[t['id']]
        t['review_status'] = 'reviewed'
    (a.output/name/'manifest.json').write_text(json.dumps(data, ensure_ascii=False, indent=2)+'\n')
summary = dict(scope='All XODR roads; source existence resolves ambiguity, not physical legality or dynamic phase.',
    roads=651, road_reference_length_m=sum(r['length_m'] for r in road_results),
    road_tiles=len(reviews), primary_pages=len(manifest['pages']), supplemental_tiles=len(extra['tiles']),
    native_coverage=read('native-coverage-check.json'), decisions=dict(Counter(r['decision'] for r in reviews)),
    inputs=manifest['inputs'], all_roads_reviewed=True, new_confirmed_conversion_errors=0,
    prior_dynamic_release_blockers_unchanged=True, roads_reviewed=road_results, reviews=reviews)
(a.output/'review-summary.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2)+'\n')
parts = ['<!doctype html><html lang="ko"><meta charset="utf-8"><meta name="viewport" content="width=device-width">',
'<title>LivingLab 전 도로 이미지 감사</title><style>body{font:16px/1.5 system-ui;margin:24px;max-width:1600px}input{font:inherit;padding:8px}img{width:49%;height:auto}section{border-top:1px solid #bbb;padding:16px 0}summary{cursor:pointer}a{color:#0657b8}pre{white-space:pre-wrap;overflow-wrap:anywhere}</style>',
'<h1>LivingLab 전 도로 이미지 감사</h1><p>651개 도로 · 33.245km(도로 기준선 합계) · 306개 도로 구역 + 도로 밖 객체 1개 구역. 원본과 중첩 영상을 모두 검토했습니다. 애매한 객체는 XODR 존재·차로 유형을 따릅니다.</p>',
'<p>왼쪽: 정밀 OSGB 원본 · 오른쪽: 청록 lanelet, 주황 cell, 자홍 정지선, 빨강 미할당 정지선, 초록 물리 등화, 보라 신호–정지선 관계. +X 오른쪽 / +Y 위쪽. 보라선은 차량 경로가 아닙니다.</p>',
'<p>각 그림을 누르면 1280px 원본을 엽니다. 주행·신호 위상 검증과 원본 내부 모순은 별도 기록을 따릅니다. <a href="../../../HDMapVisionAudit.md">보고서</a> · <a href="review-summary.json">전수 판정 JSON</a></p>',
'<label>도로 번호 또는 구역 번호 <input id="search" type="search" placeholder="146 또는 t13_14"></label> <span id="count" aria-live="polite"></span>']
for data, folder in ((manifest, 'atlas'), (supplement, 'supplemental')):
    for t in data['tiles']:
        r = t['review']; tid = t['id']; roads = ', '.join(map(str, t['road_ids']))
        parts.append(f'<section id="{tid}" data-roads="{escape(roads)}"><h2>{tid} · 도로 {roads}</h2><p>{escape(r["decision"])}</p>')
        parts.append('<p>' + '<br>'.join(escape(s) for s in r['observations']) + '</p>')
        for mode, label in (('source','OSGB 원본'),('overlay','지도 중첩')):
            path = f'{folder}/tiles/{tid}_{mode}.jpg'
            parts.append(f'<a href="{path}"><img loading="lazy" src="{path}" alt="{tid} {label}"></a>')
        parts.append(f'<p>lanelet {len(t["native_lanelets"])} · cell {t["cell_count"]} · 정지선 {len(t["stoplines"])}</p><details><summary>XODR 및 개별 판정 근거</summary><pre>{escape(json.dumps(r,ensure_ascii=False,indent=2))}</pre></details></section>')
parts.append('''<script>var q=document.getElementById('search'),sections=[...document.querySelectorAll('section')];q.oninput=()=>{var term=q.value.trim();sections.forEach(s=>s.hidden=!!term&&!(s.id.includes(term)||s.dataset.roads.split(', ').includes(term)));document.getElementById('count').textContent=sections.filter(s=>!s.hidden).length+'개 구역';};q.oninput();</script></html>''')
html = '\n'.join(parts)
class Links(HTMLParser):
    def handle_starttag(self, tag, attrs):
        for key, value in attrs:
            if key in ('href','src'):
                assert (a.output/value).is_file(), value
Links().feed(html)
(a.output/'index.html').write_text(html)
# Every saved image has a checksum; reviewer records refer to the original paths.
images = {str(f.relative_to(a.output)): sha(f) for folder in ('atlas','supplemental') for f in sorted((a.output/folder).rglob('*.jpg'))}
(a.output/'image-checksums.json').write_text(json.dumps(images, indent=2)+'\n')
assert len(images) == 2 * 307 + 154  # 614 tile images + 154 two-column pages
print(json.dumps({k:summary[k] for k in ('roads','road_tiles','primary_pages','supplemental_tiles','decisions')},ensure_ascii=False))
