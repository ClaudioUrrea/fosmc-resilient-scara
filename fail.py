import json,glob,collections,numpy as np
def failed(r,sc):
    if r['diverged']: return True
    if sc=='S3': return r.get('rec_1') is None
    return r['rmse']>3.0
g=collections.defaultdict(list)
for f in sorted(glob.glob('results2/camp_*.jsonl')):
  for l in open(f):
    r=json.loads(l); k=r['key'].split('|'); g['|'.join(k[:-1])].append((int(k[-1]),r,k[-2]))
out={}
for key,v in sorted(g.items()):
    nf=sum(failed(r,sc) for s,r,sc in v); nd=sum(r['diverged'] for s,r,sc in v)
    ok=[r['rmse'] for s,r,sc in v if not failed(r,sc)]
    out[key]=(len(v),nd,nf,np.mean(ok) if ok else None,np.std(ok,ddof=1) if len(ok)>1 else None)
    print(f'{key:38s} n={len(v)} div={nd} fail={nf} rmse_ok={out[key][3] if ok is None else (round(out[key][3],4) if ok else None)}')
json.dump(out,open('fail_table.json','w'))
