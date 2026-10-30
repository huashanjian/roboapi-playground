import concurrent.futures, hashlib, json
from pathlib import Path
import httpx
from policies.roboapi.config import REPO, WORK
d=json.loads((WORK/'roboapi_full120_asset_manifest.json').read_text())
aliases={}
for record in d['files']:
 for url in record.get('remote_refs',[]):
  if '/Materials/' in url:
   alias=str(REPO/'assets/materials'/url.split('/Materials/',1)[1])
   aliases.setdefault(alias,set()).add(url)
resolved=[]
remote=[]
for name in d['missing']:
 p=Path(name)
 if p.exists():
  resolved.append({'reported_missing_path':name,'resolution':'supplemented_official_shared_texture','actual_path':str(p),'sha256':hashlib.sha256(p.read_bytes()).hexdigest()})
 elif name.endswith('/gltf/pbr.mdl'):
  import sysconfig
  p=Path(sysconfig.get_paths()['purelib'])/'isaacsim/kit/mdl/core/mdl/gltf/pbr.mdl'
  assert p.is_file()
  resolved.append({'reported_missing_path':name,'resolution':'bundled_MDL_search_path_module','actual_path':str(p),'sha256':hashlib.sha256(p.read_bytes()).hexdigest()})
 elif name in aliases:
  remote.append((name,sorted(aliases[name])))
 else:raise RuntimeError('Unresolved asset: '+name)
def check(pair):
 name,urls=pair
 records=[]
 with httpx.Client(trust_env=False,timeout=20,follow_redirects=True) as client:
  for url in urls:
   response=client.head(url)
   assert response.status_code==200,(url,response.status_code)
   records.append({'url':url,'http_status':response.status_code,'content_length':response.headers.get('content-length'),'etag':response.headers.get('etag')})
 return {'reported_missing_path':name,'resolution':'original_remote_URI_available; initial scanner incorrectly flattened Assets/DigitalTwin prefix','remote':records}
with concurrent.futures.ThreadPoolExecutor(max_workers=4) as pool:
 for result in pool.map(check,remote):resolved.append(result);print(result['reported_missing_path'],flush=True)
report={'passed':True,'local_scene_roots':d['roots'],'initial_scan_files':len(d['files']),'resolved_initial_missing':resolved,'remaining_unresolved':[], 'authored_usd_files_changed':False,'remote_asset_limitation':'Original authored remote URIs remain in use; HEAD availability/ETag verified, not immutable local overrides.','texture_supplement':json.loads((WORK/'full120_asset_supplements.json').read_text())}
(WORK/'full120_asset_validation.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps({'passed':True,'resolved':len(resolved),'remaining_unresolved':[]}))
