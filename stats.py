import json, numpy as np, collections, glob
from scipy.stats import wilcoxon, fisher_exact
def load(n): return [json.loads(l) for l in open(f'results2/{n}.jsonl')]
def grp(rows):
    g=collections.defaultdict(dict)
    for r in rows:
        k=r['key'].split('|'); g['|'.join(k[1:-1])][int(k[-1])]=r
    return g
A=grp(load('camp_A')); F=grp(load('camp_F')+load('camp_X'))
def ms(d,k):
    v=[r[k] for r in d.values() if not r['diverged'] and r.get(k) is not None]
    return np.mean(v),np.std(v,ddof=1),len(v)
C=['CT-PID','ISMC-A','ISMC-B','ISMC-M','ISMC-J','FOSMC','FOSMC-J']
for sc in ('S1','S2','S3'):
  for k in ('rmse','emax','tv'):
    print(sc,k,' & '.join(f'{ms(A[c+"|"+sc],k)[0]:.4g}±{ms(A[c+"|"+sc],k)[1]:.2g}' for c in C), '| div', [sum(r['diverged'] for r in A[c+'|'+sc].values()) for c in C])
def pair(d1,d2,k='rmse'):
    ok=[s for s in d1 if s in d2 and not d1[s]['diverged'] and not d2[s]['diverged']]
    x=np.array([d1[s][k] for s in ok]); y=np.array([d2[s][k] for s in ok])
    p=wilcoxon(x,y).pvalue if len(ok)>=6 and np.any(x!=y) else float('nan')
    return len(ok), 100*(np.mean(x)/np.mean(y)-1), p, ok
print()
for sc in ('S1','S2','S3'):
  for a,b in (('FOSMC-J','ISMC-J'),('FOSMC','ISMC-A'),('FOSMC','ISMC-M'),('FOSMC','ISMC-J'),('FOSMC-J','ISMC-B'),('FOSMC','ISMC-B')):
    n,d,p,ok=pair(A[a+'|'+sc],A[b+'|'+sc]); print(sc,a,'vs',b,'n',n,'Δmean%',round(d,1),'p',f'{p:.2g}')
  for a,b in (('FOSMC-J','ISMC-J'),('FOSMC','ISMC-A')):
    n,d,p,ok=pair(A[a+'|'+sc],A[b+'|'+sc],'emax'); print(sc,'EMAX',a,'vs',b,n,round(d,1),f'{p:.2g}')
print('div seeds', {c:[s for s,r in A[c+'|S3'].items() if r['diverged']] for c in C})
print('\nAnti-windup S3')
for aw in ('cond','backcalc'):
  for c in ('ISMC-A','ISMC-B','ISMC-M','ISMC-J','FOSMC','FOSMC-J'):
    d=F[f'{c}|aw={aw}|S3']; m,s,n=ms(d,'rmse'); m2,s2,_=ms(d,'emax'); r1=ms(d,'rec_1')
    print(aw,c,f'{m:.3f}±{s:.3f}',f'{m2:.2f}±{s2:.2f}','rec1',f'{r1[0]:.3f}', 'div',sum(r['diverged'] for r in d.values()),'tv',f"{ms(d,'tv')[0]:.0f}")
  for a,b in (('FOSMC-J','ISMC-J'),('FOSMC','ISMC-A'),('FOSMC','ISMC-M')):
    n,dd,p,ok=pair(F[f'{a}|aw={aw}|S3'],F[f'{b}|aw={aw}|S3']); print(' ',aw,a,b,n,round(dd,1),f'{p:.2g}')
print('\nrecovery S3 (mean±sd) band 1mm / 0.5mm; sat_th1_post; det delay; Ie')
for c in C:
  d=A[c+'|S3']; print(c, *[f"{ms(d,k)[0]:.3f}±{ms(d,k)[1]:.3f}" for k in ('rec_1','rec_0.5','sat_th1_post','det_delay_ms')])
# FDI over all campaigns
tot=collections.Counter()
for f in glob.glob('results2/camp_*.jsonl'):
  for r in load(f.split('/')[-1][:-6]):
    sc=r['key'].split('|')[-2]
    if 'CT-PID' in r['key']: pass
    tot[(sc,'n')]+=1; tot[(sc,'fa')]+=bool(r.get('fa')); 
    if sc=='S3': tot[(sc,'md')]+=bool(r.get('md'))
print(dict(tot))
