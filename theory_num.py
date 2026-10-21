import numpy as np, json
from math import gamma
from scipy.integrate import quad
lam=0.7
def _hdr():
    pass
def l1norm(lam,L1,L2,L=None,T=8.0,h=2e-4):
    n=int(T/h); g=np.zeros(n); g[0]=1.0
    if lam==1.0:
        z=0.0
        for i in range(n-1):
            z+=g[i]*h; g[i+1]=g[i]+h*(-L1*g[i]-L2*z)
        return np.sum(np.abs(g))*h, g
    # product-integration weights for kernel k_L on grid: k_L(t)=min(t,L)^(lam-1)/Gamma(lam)
    tt=np.arange(n+1)*h
    def Kint(a,b):  # integral of k_L over [a,b]
        f=lambda t: (np.minimum(t,L)**lam/lam + np.maximum(t-L,0)*L**(lam-1))/gamma(lam) if L else t**lam/(lam*gamma(lam))
        return f(b)-f(a)
    W=Kint(tt[:-1],tt[1:])     # weight for g at lag j (rectangle rule on g)
    Nw=n if L is None else int(round(L/h))
    tailsum=0.0; kap= 0 if L is None else L**(lam-1)/gamma(lam)
    for i in range(n-1):
        j0=max(0,i-Nw+1)
        conv=np.dot(W[:i-j0+1], g[i::-1][:i-j0+1])
        if L is not None and i-Nw>=0: tailsum+=g[i-Nw]*h
        conv+=kap*tailsum
        g[i+1]=g[i]+h*(-L1*g[i]-L2*conv)
    return np.sum(np.abs(g))*h, g
if __name__=="__main__":
  pass
if __name__=="__main__":
    print('L kappa L2k c_L L2c (L2=800)')
    for L in (0.05,0.1,0.25,0.5,1,2):
        k=L**(lam-1)/gamma(lam); c=L**lam*(1-lam)/gamma(lam+1)
        print(L, round(k,3), round(800*k), round(c,4), round(800*c,1))
    # positivity of Re K_L(iw) = int_0^L ktilde cos
    def ReK(w,L,lam=0.7):
        kt=lambda t:(max(t,1e-14)**(lam-1)-L**(lam-1))/gamma(lam)
        return quad(kt,0,L,weight='cos',wvar=w,limit=400)[0]
    ws=np.logspace(-2,4,400); m=min(ReK(w,1.0) for w in ws); print('min Re K_L(iw), L=1:',m)
    m2=min(ReK(w,0.05) for w in ws); print('min Re K_L(iw), L=0.05:',m2)
    # impulse response L1 norms: g' + L1 g + L2 (k_L * g) = 0, g(0)=1
    res={} if False else {}
    for name,args in [('ISMC-A (1,20,800)',(1.0,20,800)),('FOSMC full (0.7,20,800)',(0.7,20,800,None)),('FOSMC L=1 (0.7,20,800)',(0.7,20,800,1.0)),
                      ('ISMC-M (1,20,616)',(1.0,20,616)),('ISMC-B (1,20,1600)',(1.0,20,1600)),('ISMC-J (1,24,2000)',(1.0,24,2000)),('FOSMC-J L=1 (0.7,12.5,2000)',(0.7,12.5,2000,1.0)),
                      ('FOSMC L=0.05',(0.7,20,800,0.05)),('FOSMC L=2',(0.7,20,800,2.0))]:
        v,g=l1norm(*args); res[name]=v; print(name,'||g||_1 =',round(v,5), ' int g =', round(np.sum(g)*2e-4,6))
    json.dump(res,open('l1norms.json','w'),indent=1)
