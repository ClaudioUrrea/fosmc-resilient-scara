import json, sys, numpy as np, collections
def num(r,k):
    v=r.get(k); return v if isinstance(v,(int,float)) and not isinstance(v,bool) else None
def table(name, ks=('rmse','emax_post','tv','rec_1','sat_th1_post','Ie_max_post')):
    g=collections.defaultdict(list)
    for l in open(f'results2/camp_{name}.jsonl'):
        r=json.loads(l); k=r['key'].split('|'); g['|'.join(k[:-1])].append(r)
    print('==',name)
    for grp,rs in sorted(g.items()):
        out=[]
        for k in ks:
            v=[num(r,k) for r in rs if num(r,k) is not None and not r['diverged']]
            out.append(f'{np.median(v):.3g}' if v else '-')
        print(f'{grp:40s} n={len(rs)} div={sum(r["diverged"] for r in rs)}', *out)
for n in sys.argv[1:]: table(n)
