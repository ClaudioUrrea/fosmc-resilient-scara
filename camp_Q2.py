import ff_campaigns as F
sel=F.sel_load(); sp=[]
for c in ('ISMC-J','FOSMC-J'):
    for s in range(120,200): sp.append(F.S(f'Q|{c}|rho=60-25-3','S3',s,F.ck(c,sel),rho=(60.,25.,3.)))
F.run_jobs(sp,'camp_Q2'); print('Q2 DONE',flush=True)
