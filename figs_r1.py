import json, numpy as np, collections, matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
import fontsetup; fontsetup.apply()
from math import gamma
R='reps'; F='figs'
STY={'CT-PID':dict(color='0.45',ls=':'),'ISMC-A':dict(color='tab:blue',ls='--'),'ISMC-M':dict(color='tab:cyan',ls=(0,(3,1,1,1))),
     'ISMC-B':dict(color='tab:green',ls='-.'),'ISMC-J':dict(color='tab:purple',ls='--'),'FOSMC':dict(color='tab:red',ls='-'),'FOSMC-J':dict(color='tab:orange',ls='-')}
_BB=dict(boxstyle='round,pad=0.15',fc='white',ec='none',alpha=0.85)
def save(fig,n): fig.tight_layout(); fig.savefig(f'{F}/{n}.pdf'); fig.savefig(f'{F}/{n}.png',dpi=200); plt.close(fig)
def shade(ax,sc='S3'):
    if sc in ('S2','S3'): ax.axvline(3,color='orange',lw=0.8); ax.text(3.03,0.05,'LOE',color='darkorange',transform=ax.get_xaxis_transform(),fontsize=7,bbox=_BB)
    if sc=='S3': ax.axvline(4,color='r',lw=0.8); ax.text(4.03,0.05,'Lock',color='r',transform=ax.get_xaxis_transform(),fontsize=7,bbox=_BB)
    ax.axvline(6,color='purple',lw=0.6,ls='--'); ax.text(6.03,0.05,'Step disturbance',color='purple',transform=ax.get_xaxis_transform(),fontsize=7,bbox=_BB)
L=lambda n: np.load(f'{R}/{n}.npz')
ex=lambda z: np.linalg.norm(z['x']-z['xd'],axis=1)*1e3
# ---------------- S3 tracking
def fig_s3():
    names=['CT-PID','ISMC-A','ISMC-J','FOSMC','FOSMC-J']
    fig=plt.figure(figsize=(7.0,4.6)); a1=fig.add_subplot(2,2,1); a2=fig.add_subplot(2,2,2); a3=fig.add_subplot(2,1,2)
    d=L('S3_FOSMC_100')
    a1.plot(d['xd'][:,0],d['xd'][:,1],'g--',lw=1.2,label='Desired'); a1.plot(d['x'][:,0],d['x'][:,1],'r-',lw=0.7,label='Executed (FOSMC)')
    a1.set_ylim(0.26,0.46); a1.set_xlabel('$x$ (m)'); a1.set_ylabel('$y$ (m)'); a1.legend(loc='lower center'); a1.set_title('(a) Task space, scenario S3')
    m=(d['t']>=3.9)&(d['t']<4.6)
    for n in names:
        e=L(f'S3_{n}_100'); a2.plot(e['x'][m,0],e['x'][m,1]*1e3,lw=1,label=n,**STY[n])
    a2.plot(d['xd'][m,0],d['xd'][m,1]*1e3,'k',lw=0.6,alpha=0.6,label='Desired'); a2.set_xlabel('$x$ (m)'); a2.set_ylabel('$y$ (mm)'); a2.set_title('(b) Zoom: 3.9–4.6 s (elbow lock)'); a2.legend(ncol=2,fontsize=6)
    for n in names:
        e=L(f'S3_{n}_100'); a3.semilogy(e['t'],ex(e)+1e-4,lw=0.9,label=n,**STY[n])
    shade(a3); a3.set_ylim(1e-3,1e2); a3.set_xlim(0,10); a3.set_xlabel('Time (s)'); a3.set_ylabel(r'$\|\mathbf{x}-\mathbf{x}_d\|$ (mm)'); a3.legend(ncol=5,loc='upper right',framealpha=0.95,fontsize=6.5); a3.set_title('(c) Cartesian tracking error (evaluation seed 100)')
    save(fig,'fig_s3_tracking')
def fig_internals():
    d=L('S3_FOSMC_100'); t=d['t']
    fig,ax=plt.subplots(3,1,figsize=(7.0,5.4),sharex=True); cols=['tab:orange','tab:blue','tab:red']
    lab=['$d$ (m)',r'$\theta_1$ (rad)',r'$\theta_2$ (rad)']
    for i,c in enumerate(cols): ax[0].plot(t,d['q'][:,i],color=c,lw=1,label=lab[i]); ax[0].plot(t,d['qr'][:,i],color=c,lw=0.8,ls='--')
    ax[0].set_ylabel('Joint position'); ax[0].legend(ncol=3,loc='upper right',framealpha=0.95); ax[0].set_title('(a) Joint positions (solid) and references (dashed), FOSMC')
    tm=np.array([600,60,30])
    for i,c in enumerate(cols): ax[1].plot(t,d['tau'][:,i]/tm[i],color=c,lw=0.6,label=[r'$f_d/600$',r'$\tau_1/60$',r'$\tau_2/30$'][i])
    ax[1].set_ylabel('Normalised effort'); ax[1].legend(ncol=3,loc='upper right',framealpha=0.95); ax[1].set_ylim(-1.1,1.1); ax[1].set_title('(b) Actuator commands normalised by the saturation limits')
    rth=np.array([60,8,4])
    for i,c in enumerate(cols): ax[2].plot(t,np.abs(d['r'][:,i])/rth[i],color=c,lw=0.8,label=[r'$|r_d|/r_{\mathrm{th},d}$',r'$|r_1|/r_{\mathrm{th},1}$',r'$|r_2|/r_{\mathrm{th},2}$'][i])
    ax[2].axhline(1,color='k',lw=0.6,ls=':'); ax[2].set_yscale('log'); ax[2].set_ylim(1e-3,1e2); ax[2].set_ylabel('Residual / threshold'); ax[2].legend(ncol=3,loc='upper right',framealpha=0.95)
    ax[2].set_title('(c) Generalised-momentum residuals normalised by their thresholds'); ax[2].set_xlabel('Time (s)')
    for a in ax: shade(a)
    save(fig,'fig_s3_internals')
def grp(files):
    g=collections.defaultdict(dict)
    for f in files:
        for l in open(f'results2/{f}.jsonl'):
            r=json.loads(l); k=r['key'].split('|'); g['|'.join(k[1:-1])][int(k[-1])]=r
    return g
def failed(r,sc): return r['diverged'] or (sc=='S3' and r.get('rec_1') is None) or (sc!='S3' and r['rmse']>3)
def fig_mc():
    g=grp(['camp_A']); C=['CT-PID','ISMC-A','ISMC-M','ISMC-B','ISMC-J','FOSMC','FOSMC-J']
    fig,ax=plt.subplots(1,3,figsize=(7.2,2.8))
    for k,sc in enumerate(('S1','S2','S3')):
        data=[[r['rmse'] for r in g[f'{c}|{sc}'].values() if not failed(r,sc)] for c in C]
        bp=ax[k].boxplot(data,widths=0.6,patch_artist=True,showfliers=True,flierprops=dict(ms=2))
        for p,c in zip(bp['boxes'],C): p.set_facecolor(STY[c]['color']); p.set_alpha(0.55)
        for i,c in enumerate(C):
            nf=sum(failed(r,sc) for r in g[f'{c}|{sc}'].values())
            if nf: ax[k].text(i+1,0.97,f'{nf}/20\nfailed',ha='center',va='top',fontsize=6,color='r',transform=ax[k].get_xaxis_transform())
        ax[k].set_xticks(range(1,8)); ax[k].set_xticklabels(C,rotation=60,fontsize=6.5); ax[k].set_title(f'({"abc"[k]}) {sc}'); 
        if sc!='S3': ax[k].set_yscale('log')
        else: ax[k].set_ylim(1.2,2.0)
    ax[0].set_ylabel('RMSE (mm)')
    save(fig,'fig_mc')
def fig_recovery():
    g=grp(['camp_D'])
    fig,ax=plt.subplots(1,3,figsize=(7.2,2.6))
    Ls=[0.05,0.1,0.25,0.5,1.0,2.0]
    for c,mk in (('FOSMC','o'),('ISMC-M','s')):
        m=[];s=[];nf=[];rc=[]
        for Lm in Ls:
            rs=list(g[f'{c}|L={Lm}|S3'].values()); ok=[r for r in rs if not failed(r,'S3')]
            m.append(np.mean([r['rmse'] for r in ok])); s.append(np.std([r['rmse'] for r in ok],ddof=1)); nf.append(len(rs)-len(ok)); rc.append(np.mean([r['rec_1'] for r in ok]))
        lab='FOSMC ($\\lambda=0.7$)' if c=='FOSMC' else r'ISMC-M($L$), $\Lambda_2=800\kappa_L$'
        ax[0].errorbar(Ls,m,yerr=s,marker=mk,ms=3.5,lw=1,capsize=2,color=STY[c]['color'],label=lab)
        ax[1].plot(Ls,rc,marker=mk,ms=3.5,lw=1,color=STY[c]['color'],label=lab)
        for x,y,n in zip(Ls,m,nf):
            if n: ax[0].annotate(f'{n}/20 failed',(x,y),textcoords='offset points',xytext=(4,6),fontsize=6,color='r')
    a0=ax[0].twinx(); a0.plot(Ls,[Lm**(-0.3)/gamma(0.7) for Lm in Ls],'k:',lw=0.8); a0.set_ylabel(r'$\kappa_L$',fontsize=7); a0.grid(False)
    for a in ax[:2]: a.set_xscale('log'); a.set_xlabel('Memory length $L$ (s)')
    ax[0].set_ylabel('S3 RMSE (mm)'); ax[0].set_title('(a) Memory length, S3'); ax[0].legend(fontsize=6,loc='upper center')
    ax[1].set_ylabel('Recovery time (s)'); ax[1].set_title('(b) Time to re-enter 1-mm band')
    for n,lab,kw in (('S3_ISMC-J_112','ISMC-J',STY['ISMC-J']),('S3_FOSMC-J_112','FOSMC-J',STY['FOSMC-J']),('S3_ISMC-J-cond_112','ISMC-J + AW',dict(color='tab:purple',ls=':')),('S3_FOSMC-J-cond_112','FOSMC-J + AW',dict(color='k',ls=':')),('S3_FOSMC_112','FOSMC',STY['FOSMC'])):
        z=L(n); t=z['t']; e=np.nan_to_num(ex(z),nan=1e3); w=50; m=(t>=3.8)&(t<7.5)
        env=np.array([e[max(0,i-w):i+1].max() for i in range(len(e))])
        ax[2].semilogy(t[m],env[m]+1e-4,lw=1.0,label=lab,**kw)
    ax[2].axvline(4,color='r',lw=0.6); ax[2].set_ylim(1e-2,1e3); ax[2].set_xlabel('Time (s)'); ax[2].set_ylabel('Cartesian error (mm)'); ax[2].set_title('(c) Seed 112, error envelope'); ax[2].legend(fontsize=5.6,loc='upper right',ncol=1)
    save(fig,'fig_recovery')
def fig_delay():
    g=grp(['camp_J','camp_X','camp_A','camp_Z3','camp_Z3b'])
    for c in ('ISMC-A','ISMC-B','ISMC-J','FOSMC','FOSMC-J'): g[f'{c}|dly=0.0|S2']={s:r for s,r in g[f'{c}|S2'].items() }
    fig,ax=plt.subplots(1,2,figsize=(7.0,2.6))
    ds=[0,0.003,0.005,0.007,0.009,0.011]
    for c in ('ISMC-A','ISMC-B','ISMC-J','FOSMC','FOSMC-J'):
        xs=[];ys=[];fs=[]
        for d in ds:
            rs=list(g.get(f'{c}|dly={d}|S2',g.get(f'{c}|dly={float(d)}|S2',{})).values())
            if not rs: continue
            ok=[r['rmse'] for r in rs if not failed(r,'S2')]
            fs.append((d*1e3,len(rs)-len(ok)))
            if ok: xs.append(d*1e3); ys.append(np.median(ok))
        ax[0].semilogy(xs,ys,marker='o',ms=3,lw=1,label=c,**STY[c])
        ax[1].plot([a for a,b in fs],[b for a,b in fs],marker='o',ms=3,lw=1,label=c,**STY[c])
    ax[0].axhline(2,color='k',lw=0.6,ls=':'); ax[0].set_xlabel('Input delay (ms)'); ax[0].set_ylabel('Median RMSE, non-failed runs (mm)'); ax[0].set_title('(a) Accuracy vs. delay (S2, 20 seeds)'); ax[0].legend(fontsize=6)
    ax[1].set_xlabel('Input delay (ms)'); ax[1].set_ylabel('Failed runs (of 20)'); ax[1].set_title('(b) Failures vs. delay'); ax[1].set_ylim(-0.5,20.5)
    save(fig,'fig_delay')
def fig_ablation():
    fig,ax=plt.subplots(1,2,figsize=(7.0,2.5))
    for n,lab,st in (('S3_FOSMC_100','Fault-aware null space + reconfiguration',dict(color='tab:red')),('AB_norec_100','Without reconfiguration',dict(color='tab:blue',ls='--')),('AB_weak_100','Weak null-space objective',dict(color='0.3',ls=':'))):
        z=L(n); t=z['t']; e=ex(z); ax[0].semilogy(t,e+1e-4,lw=0.9,label=lab,**st); ax[1].plot(t,np.degrees(z['q'][:,2]),lw=0.9,label=lab,**st)
    ax[1].axhline(98.3,color='k',lw=0.6,ls=':'); ax[1].text(0.2,100,r'$\theta_2^{\max}=98.3^\circ$',fontsize=6.5)
    for a in ax: a.axvline(4,color='r',lw=0.6); a.set_xlabel('Time (s)'); a.set_xlim(0,10)
    ax[0].set_ylabel('Cartesian error (mm)'); ax[0].set_ylim(1e-3,1e3); ax[0].set_title('(a) Cartesian error'); ax[0].legend(fontsize=6)
    ax[1].set_ylabel(r'$\theta_2$ (deg)'); ax[1].set_title('(b) Elbow angle')
    save(fig,'fig_ablation')
for f in (fig_s3,fig_internals,fig_mc,fig_recovery,fig_delay,fig_ablation): f()
