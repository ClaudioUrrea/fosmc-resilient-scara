import json,numpy as np,collections
from scipy.stats import wilcoxon
from failfn import failed
rr=[json.loads(l) for l in open('results2/camp_R2T2.jsonl')]
g=collections.defaultdict(dict)
for r in rr:
    k=r['key'].split('|'); g[(k[1],k[2],k[3])][int(k[4])]=r
out={}
for cond in ('nom','K1','D2'):
    for sc in ('S1','S2','S3'):
        if (('ISMC-K',cond,sc) not in g): continue
        row={}
        for c in ('ISMC-K','FOSMC-K'):
            d=g[(c,cond,sc)]; f=[s for s,r in d.items() if failed(r,sc)]
            ok=[s for s in d if s not in f]
            row[c]=dict(n=len(d),fail=len(f),rmse=float(np.mean([d[s]['rmse'] for s in ok])) if ok else None,
                        tv=float(np.mean([r['tv'] for r in d.values()])),emax=float(np.mean([d[s]['emax'] for s in ok])) if ok else None)
        com=[s for s in g[('ISMC-K',cond,sc)] if s in g[('FOSMC-K',cond,sc)] and not failed(g[('ISMC-K',cond,sc)][s],sc) and not failed(g[('FOSMC-K',cond,sc)][s],sc)]
        if len(com)>5:
            x=np.array([g[('FOSMC-K',cond,sc)][s]['rmse'] for s in com]);y=np.array([g[('ISMC-K',cond,sc)][s]['rmse'] for s in com])
            row['rel']=float(x.mean()/y.mean()-1);row['p']=float(wilcoxon(x,y).pvalue);row['npair']=len(com)
        out[f'{cond}|{sc}']=row
        print(cond,sc,json.dumps({k:(v if not isinstance(v,dict) else {a:(round(b,4) if isinstance(b,float) else b) for a,b in v.items()}) for k,v in row.items()}))
json.dump(out,open('results2/an_T.json','w'),indent=1)
