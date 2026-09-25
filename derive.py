import sympy as sp
t=sp.symbols('t')
d,t1,t2=[sp.Function(n)(t) for n in ('d','t1','t2')]
m0,m1,m2,l1,lc1,lc2,I1,I2=sp.symbols('m0 m1 m2 l1 lc1 lc2 I1 I2',positive=True)
q=[d,t1,t2]; qd=[sp.diff(x,t) for x in q]
x1=d+lc1*sp.cos(t1); y1=lc1*sp.sin(t1)
x2=d+l1*sp.cos(t1)+lc2*sp.cos(t1+t2); y2=l1*sp.sin(t1)+lc2*sp.sin(t1+t2)
T=sp.Rational(1,2)*m0*sp.diff(d,t)**2+sp.Rational(1,2)*m1*(sp.diff(x1,t)**2+sp.diff(y1,t)**2)+sp.Rational(1,2)*I1*sp.diff(t1,t)**2 \
 +sp.Rational(1,2)*m2*(sp.diff(x2,t)**2+sp.diff(y2,t)**2)+sp.Rational(1,2)*I2*(sp.diff(t1,t)+sp.diff(t2,t))**2
v=sp.symbols('v0:3')
Ts=T.subs({qd[i]:v[i] for i in range(3)})
M=sp.Matrix(3,3,lambda i,j: sp.simplify(sp.diff(Ts,v[i],v[j])))
qs=sp.symbols('q0:3')
Mq=M.subs({q[i]:qs[i] for i in range(3)})
Mq=sp.simplify(Mq)
print(Mq)
C=sp.zeros(3,3)
for k in range(3):
  for j in range(3):
    C[k,j]=sp.simplify(sum(sp.Rational(1,2)*(sp.diff(Mq[k,j],qs[i])+sp.diff(Mq[k,i],qs[j])-sp.diff(Mq[i,j],qs[k]))*v[i] for i in range(3)))
print(C)
# skew check
Md=sp.Matrix(3,3,lambda i,j: sum(sp.diff(Mq[i,j],qs[k])*v[k] for k in range(3)))
print(sp.simplify(Md-2*C+ (Md-2*C).T))
