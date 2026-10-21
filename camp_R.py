import ff_campaigns as F
sel=F.sel_load(); sp=[]
for c in ('FOSMC','ISMC-A'):
    for sc in ('S1','S2','S3'):
        for s in F.EV: sp.append(F.S(f'R|{c}|rho=60-25-3',sc,s,F.ck(c,sel),rho=(60.,25.,3.),log_delta=True))
F.run_jobs(sp,'camp_R')
