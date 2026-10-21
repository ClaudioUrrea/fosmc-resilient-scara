import numpy as np, matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
from math import gamma
import fontsetup; fontsetup.apply()
from theory_num import l1norm
lam=0.7
fig,ax=plt.subplots(1,3,figsize=(7.2,2.5))
t=np.linspace(1e-3,3,2000)
ax[0].plot(t,t**(lam-1)/gamma(lam),'k',lw=1.2,label=r'$k(\tau)$ (full memory)')
for L,c in ((0.25,'tab:orange'),(1.0,'tab:red')):
    ax[0].plot(t,np.minimum(t,L)**(lam-1)/gamma(lam),color=c,ls='--',lw=1,label=rf'$k_L(\tau)$, $L={L:g}$ s')
ax[0].plot(t,np.ones_like(t),color='tab:blue',ls=':',lw=1,label=r'$\lambda=1$')
ax[0].set_ylim(0,3); ax[0].set_xlabel(r'$\tau$ (s)'); ax[0].set_title(r'(a) Kernels, $\lambda=0.7$'); ax[0].legend(fontsize=6)
w=np.logspace(-1,3,500); s=1j*w
def KL(sv,L,lam=0.7):
    u=np.linspace(0,L**lam,200001); out=[]
    for x in sv:
        f=np.exp(-x*u**(1/lam))/lam
        val=np.trapezoid(f,u)/gamma(lam)+(L**(lam-1)/gamma(lam))*np.exp(-x*L)/x
        out.append(val)
    return np.array(out)
for name,(lm,L1,L2,L),st in [('ISMC-A',(1,20,800,None),dict(color='tab:blue',ls='--')),('FOSMC, full memory',(0.7,20,800,None),dict(color='k',ls='-')),
                             ('FOSMC, $L=1$ s',(0.7,20,800,1.0),dict(color='tab:red',ls='-.'))]:
    Kv=1/s if lm==1 else (s**(-lm) if L is None else KL(s,L))
    G=1/(s+L1+L2*Kv)
    ax[1].loglog(w/(2*np.pi),np.abs(G),lw=1,label=name,**st)
ax[1].set_xlabel('Frequency (Hz)'); ax[1].set_ylabel(r'$|G(\mathrm{i}\omega)|$'); ax[1].set_title(r'(b) $s\to e$ gain, $\Lambda_1=20$, $\Lambda_2=800$'); ax[1].legend(fontsize=6)
h=2e-4
for name,args,st in [('ISMC-A',(1.0,20,800),dict(color='tab:blue',ls='--')),('FOSMC, $L=1$ s',(0.7,20,800,1.0),dict(color='tab:red',ls='-'))]:
    v,g=l1norm(*args,T=1.0); tt=np.arange(len(g))*h
    ax[2].plot(tt,g,lw=1,label=name+rf', $\|g\|_1={v:.3f}$',**st)
ax[2].set_xlabel('Time (s)'); ax[2].set_title('(c) Impulse response $g$'); ax[2].legend(fontsize=6); ax[2].set_xlim(0,0.6)
fig.tight_layout(); fig.savefig('figs/fig_kernel.pdf'); fig.savefig('figs/fig_kernel.png',dpi=200)
