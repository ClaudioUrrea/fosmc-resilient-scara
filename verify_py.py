import numpy as np, sim2
pt=dict(sim2.NOMINAL); pt['fv']=1.2*pt['fv']; pt['fc']=1.2*pt['fc']; pt=sim2.with_payload(pt,1.5)
for sc in ('S2','S3'):
    for nm,kind,lam in (('FOSMC','FOSMC',0.7),('ISMC','ISMC',1.0)):
        o=sim2.run(ctrl_kind=kind,lam=lam,L1=20.0,L2=1600.0,scenario=sc,noise=False,p_true=pt,fdi='generic')
        A=np.column_stack([o['t'],o['q'],o['qr'],o['x']])
        np.savetxt(f'results/verify_{sc}_{nm}.csv',A,delimiter=',',header='t,d,th1,th2,dr,th1r,th2r,x,y',comments='')
        print(sc,nm,'det',o['detected_at'],'alarms',o['alarms'])
