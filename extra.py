import ff_campaigns as F
sel=F.sel_load(); sp=[]
for aw in ('cond','backcalc'):
    for s in F.EV: sp.append(F.S(f'F|FOSMC-J|aw={aw}','S3',s,F.ck('FOSMC-J',sel),antiwindup=aw))
for b1 in (0.7,0.5,0.35):
    for s in F.EV[:10]: sp.append(F.S(f'G|FOSMC-J|b1={b1}|ts=1.0','S3',s,F.ck('FOSMC-J',sel),b1=b1,tau_scale=1.0))
for d in (0.003,0.005,0.007,0.009): 
    for s in F.EV[:10]: sp.append(F.S(f'J|FOSMC-J|dly={d}','S2',s,F.ck('FOSMC-J',sel),delay=d))
F.run_jobs(sp,'camp_X')
