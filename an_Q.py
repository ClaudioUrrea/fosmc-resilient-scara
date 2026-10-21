import json, collections, numpy as np
from scipy.stats import wilcoxon
from failfn import failed
def L(n): return [json.loads(l) for l in open(f'results2/camp_{n}.jsonl')]
nom = {}; 
for r in L('A'):
    k = r['key'].split('|'); nom[(k[1], k[2], int(k[3]))] = r
q = {}
for r in L('Q'):
    k = r['key'].split('|'); q[(k[1], k[3], int(k[4]))] = r
ctrls = ('ISMC-A','ISMC-M','ISMC-B','ISMC-J','FOSMC','FOSMC-J')
print('ctrl    sc  nom_rmse  Q_rmse   nomfail Qfail  nomTV  QTV   Dmax(rail,th1,th2: pre/loe/late)')
for c in ctrls:
    for sc in ('S1','S2','S3'):
        a=[nom[(c,sc,s)] for s in range(100,120) if (c,sc,s) in nom]; b=[q[(c,sc,s)] for s in range(100,120) if (c,sc,s) in q]
        if len(b)<20: continue
        fa=[failed(r,sc) for r in a]; fb=[failed(r,sc) for r in b]
        ra=np.mean([r['rmse'] for r,f in zip(a,fa) if not f]); rb=np.mean([r['rmse'] for r,f in zip(b,fb) if not f]) if not all(fb) else float('nan')
        tv=lambda rs: np.mean([r['tv'] for r in rs if isinstance(r.get('tv'),(int,float))] or [np.nan])
        print(f'{c:8s}{sc} {ra:8.3f} {rb:8.3f} {sum(fa):4d} {sum(fb):4d} {tv(a):7.0f} {tv(b):7.0f}')
for sc in ('S1','S2'):
    d={}
    for c in ('FOSMC-J','ISMC-J','FOSMC','ISMC-A'):
        d[c]=[q[(c,sc,s)]['rmse'] for s in range(100,120) if (c,sc,s) in q]
    if all(len(v)==20 for v in d.values()):
        print(sc,'FOSMC-J vs ISMC-J:',1-np.mean(d['FOSMC-J'])/np.mean(d['ISMC-J']),'p=',wilcoxon(d['FOSMC-J'],d['ISMC-J']).pvalue,
              ' FOSMC vs ISMC-A:',1-np.mean(d['FOSMC'])/np.mean(d['ISMC-A']),'p=',wilcoxon(d['FOSMC'],d['ISMC-A']).pvalue)
# margins
for c in ('FOSMC','ISMC-A'):
    for k in ('Dmax_pre','Dmax_loe','Dmax_late'):
        v=[q[(c,'S2',s)][k] for s in range(100,120) if (c,'S2',s) in q]
        if v: print(c,k,np.max(np.array(v),axis=0).round(2))
