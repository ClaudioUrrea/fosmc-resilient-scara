import math, sys
import ff_campaigns as F
sel = F.sel_load(); S = F.S; ck = F.ck
EV100 = list(range(100, 200))
def z1():
    sp = []
    for c in ('ISMC-J', 'FOSMC-J'):
        for s in EV100:
            if s >= 120: sp.append(S(f'Z1|{c}|aw=none', 'S3', s, ck(c, sel), antiwindup='none'))
            sp.append(S(f'Z1|{c}|aw=cond', 'S3', s, ck(c, sel), antiwindup='cond'))
            sp.append(S(f'Z1|{c}|aw=backcalc', 'S3', s, ck(c, sel), antiwindup='backcalc'))
    return sp
def z2():
    sp = []
    for c in ('ISMC-M', 'FOSMC-J'):
        for sc in ('S2', 'S3'):
            for s in F.EV: sp.append(S(f'H|{c}', sc, s, ck(c, sel), loe_ramp=2.0, b1=0.5))
    lv = {'K1': dict(act_tc=0.002), 'K2': dict(act_tc=0.002, enc_res=2*math.pi/2**17, unc_scale=1.5),
          'K3': dict(act_tc=0.004, enc_res=2*math.pi/2**16, unc_scale=2.0)}
    for k, v in lv.items():
        for c in ('ISMC-M', 'FOSMC-J'):
            for sc in ('S2', 'S3'):
                for s in F.EV[:10]: sp.append(S(f'K|{c}|{k}', sc, s, ck(c, sel), **v))
    for s in F.EV[:10]: sp.append(S('G|FOSMC-J|b1=0.5|ts=0.8', 'S3', s, ck('FOSMC-J', sel), b1=0.5, tau_scale=0.8))
    return sp
def z3():
    sp = []
    for d in (0.0, 0.003, 0.005, 0.007, 0.009, 0.011):
        for c in ('ISMC-A', 'ISMC-B', 'ISMC-J', 'FOSMC', 'FOSMC-J'):
            for s in F.EV[10:]: sp.append(S(f'J|{c}|dly={d}', 'S2', s, ck(c, sel), delay=d))
    return sp
if __name__ == '__main__':
    for n, f in (('Z1', z1), ('Z2', z2), ('Z3', z3)):
        F.run_jobs(f(), 'camp_' + n)
    print('ALLZ DONE', flush=True)
