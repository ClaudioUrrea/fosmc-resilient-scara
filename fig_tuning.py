import json, numpy as np, matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
import fontsetup; fontsetup.apply()
import ff_campaigns as F
d=json.load(open('results2/selection2.json')); B=F.BUDGET; sel=d['selection']
rows=F.load('stage1a')+F.load('stage1c'); ct=sorted(F.tuning_table(rows,'CTPID'),key=lambda x:x['w'])
fig,ax=plt.subplots(1,3,figsize=(7.2,2.6),gridspec_kw=dict(width_ratios=[1.15,1,1]))
L1s=[10,15,20,30,40,60]; L2s=[50,100,200,400,800,1600,2400,3200]
Z=np.full((len(L1s),len(L2s)),np.nan); mark=[]
for t in d['ISMC']:
    if t['L1'] in L1s and t['L2'] in L2s:
        i,j=L1s.index(t['L1']),L2s.index(t['L2'])
        if t['diverged']: mark.append((j,i,'x'))
        else:
            Z[i,j]=t['rmse']
            if t['tv']>B: mark.append((j,i,'o'))
im=ax[0].imshow(np.log10(Z),origin='lower',cmap='viridis_r',aspect='auto')
for j,i,m in mark: ax[0].plot(j,i,m,color='k' if m=='x' else 'w',ms=6 if m=='x' else 7,mfc='none',mew=1.2)
ax[0].set_xticks(range(len(L2s))); ax[0].set_xticklabels(L2s,rotation=60,fontsize=6.5); ax[0].set_yticks(range(len(L1s))); ax[0].set_yticklabels(L1s)
ax[0].set_xlabel(r'$\Lambda_2$'); ax[0].set_ylabel(r'$\Lambda_1$'); ax[0].grid(False)
cb=fig.colorbar(im,ax=ax[0],fraction=0.05,pad=0.02); cb.set_label(r'$\log_{10}$ RMSE (mm)',fontsize=7)
ax[0].set_title('(a) ISMC grid (tuning seeds, S2)',fontsize=8)
for fam,c,mk in (('ISMC','tab:blue','s'),('FOSMC','tab:red','o')):
    ok=[t for t in d[fam] if not t['diverged']]
    ax[1].scatter([t['tv'] for t in ok],[t['rmse'] for t in ok],s=9,c=c,marker=mk,alpha=0.6,label=fam+' configurations')
ax[1].scatter([t['tv'] for t in ct],[t['rmse'] for t in ct],s=12,c='0.4',marker='^',label=r'CT-PID, $\omega\in[20,100]$')
for k,c in (('ISMC-J','tab:blue'),('FOSMC-J','tab:red'),('CT-PID','0.2')):
    ax[1].scatter(sel[k]['tv'],sel[k]['rmse'],s=80,marker='*',c=c,edgecolors='k',linewidths=0.5,zorder=5)
ax[1].axvline(B,color='k',ls='--',lw=0.8); ax[1].set_xscale('log'); ax[1].set_yscale('log')
ax[1].set_xlabel(r'TV (N$\,$m$\,$s$^{-1}$)'); ax[1].set_ylabel('RMSE (mm)'); ax[1].set_ylim(0.008,12); ax[1].legend(fontsize=5.8,loc='upper right',framealpha=0.95); from matplotlib.ticker import FixedLocator,NullFormatter; ax[1].xaxis.set_major_locator(FixedLocator([3e3,5e3,1e4,2e4])); ax[1].xaxis.set_minor_formatter(NullFormatter()); ax[1].set_xticklabels(['3000','5000','10$^4$','2$\\times$10$^4$']); ax[1].set_title('(b) Accuracy vs. control activity',fontsize=8)
w=[t['w'] for t in ct]
ax[2].semilogy(w,[t['rmse'] for t in ct],'k^-',ms=3,lw=0.9,label='RMSE')
a2=ax[2].twinx(); a2.plot(w,[t['tv'] for t in ct],'s--',color='tab:purple',ms=3,lw=0.9,label='TV'); a2.axhline(B,color='tab:purple',ls=':',lw=0.8)
a2.set_ylabel(r'TV (N$\,$m$\,$s$^{-1}$)',color='tab:purple',fontsize=7); a2.grid(False)
ax[2].axvline(28,color='0.5',lw=0.6); ax[2].set_xlabel(r'$\omega$ (rad/s)'); ax[2].set_ylabel('RMSE (mm)'); ax[2].set_title('(c) CT-PID grid',fontsize=8)
fig.tight_layout(); fig.savefig('figs/fig_tuning.pdf'); fig.savefig('figs/fig_tuning.png',dpi=200)
print('ISMC configs',len(d['ISMC']),'FOSMC',len(d['FOSMC']),'CTPID',len(ct))
print('ISMC diverged',[ (t['L1'],t['L2']) for t in d['ISMC'] if t['diverged']])
print('ISMC over budget',[ (t['L1'],t['L2']) for t in d['ISMC'] if not t['diverged'] and t['tv']>B])
print('FOSMC diverged',[ (t['L1'],t['lam'],t['L2']) for t in d['FOSMC'] if t['diverged']])
print('FOSMC over budget n',sum(1 for t in d['FOSMC'] if not t['diverged'] and t['tv']>B))
