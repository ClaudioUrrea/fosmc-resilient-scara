import ff_campaigns as F
sel=F.sel_load(); sp=[]
for d in (0.0,0.011):
    for s in F.EV[:10]: sp.append(F.S(f'J|FOSMC-J|dly={d}','S2',s,F.ck('FOSMC-J',sel),delay=d))
F.run_jobs(sp,'camp_Z3b')
