"""Reference trajectories for the cross-implementation check (deterministic: no noise,
fixed true plant = nominal with friction x1.2 and a 1.5-kg payload)."""
import numpy as np, sim, os
os.makedirs('results', exist_ok=True)
pt = dict(sim.NOMINAL); pt['fv'] = 1.2 * pt['fv']; pt['fc'] = 1.2 * pt['fc']; pt = sim.with_payload(pt, 1.5)
for name, kind, lam in (('FOSMC', 'FOSMC', 0.7), ('ISMC', 'ISMC', 1.0)):
    for sc in ('S2', 'S3'):
        o = sim.run(kind, lam, sc, seed=0, L2=1600.0, noise=False, p_true=pt)
        np.savetxt(f'results/verify_{sc}_{name}.csv', np.column_stack([o['t'], o['q'], o['tau'], o['x']]),
                   delimiter=',', header='t,d,theta1,theta2,tau_d,tau_1,tau_2,x,y', comments='')
        m = sim.metrics(o); print(sc, name, 'rmse %.6f mm, detected %s' % (m['rmse'], o['detected_at']))
