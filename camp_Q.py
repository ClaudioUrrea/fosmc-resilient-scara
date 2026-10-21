import ff_campaigns as F
sel=F.sel_load(); sp=[]
for c in ('ISMC-A','ISMC-B','ISMC-M','ISMC-J','FOSMC','FOSMC-J'):
    for sc in ('S1','S2','S3'):
        for s in F.EV: sp.append(F.S(f'Q|{c}|rho=60-25-3',sc,s,F.ck(c,sel),rho=(60.,25.,3.),log_delta=True))
F.run_jobs(sp,'camp_Q')
print('Q DONE',flush=True)
