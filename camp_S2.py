import ff_campaigns as F
sel=F.sel_load(); sp=[]
for tag,extra in (('nom',{}),('P',dict(rho=(60.,25.,3.)))):
    for s in range(100,200):
        sp.append(F.S(f'S|ISMC-J|sched|{tag}','S3',s,F.ck('ISMC-J',sel),gains_post=(20.0,800.0),**extra))
F.run_jobs(sp,'camp_S'); print('S2 DONE',flush=True)
