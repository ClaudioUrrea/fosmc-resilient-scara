#include <stdio.h>
#include <time.h>
#include <math.h>
#define N 1000
#define C 3
int main(){ double w[N+1], buf[C][N+1]; double a=0.3; w[0]=1; for(int j=1;j<=N;j++) w[j]=(1-(a+1)/j)*w[j-1];
 for(int c=0;c<C;c++) for(int j=0;j<=N;j++) buf[c][j]=0; int head=0; double out[C], acc=0; long steps=2000000;
 struct timespec t0,t1; clock_gettime(CLOCK_MONOTONIC,&t0);
 for(long k=0;k<steps;k++){ head=(head+1)%(N+1);
   for(int c=0;c<C;c++){ buf[c][head]=sin(1e-3*k+c); double s=0; int idx=head;
     for(int j=0;j<=N;j++){ s+=w[j]*buf[c][idx]; idx = idx? idx-1 : N; }
     out[c]=s*pow(1e-3,-a); } acc+=out[0]; }
 clock_gettime(CLOCK_MONOTONIC,&t1);
 double us=((t1.tv_sec-t0.tv_sec)*1e9+(t1.tv_nsec-t0.tv_nsec))/1e3/steps; printf("per step %.3f us (acc %g)\n",us,acc); }
