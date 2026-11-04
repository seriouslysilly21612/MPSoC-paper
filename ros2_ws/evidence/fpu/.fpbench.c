#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <time.h>
typedef double v2 __attribute__((vector_size(16)));
typedef float  v4 __attribute__((vector_size(16)));
static double el(struct timespec a,struct timespec b){
  return (b.tv_sec-a.tv_sec)*1e9+(b.tv_nsec-a.tv_nsec); }
#define N 20000000
static volatile double g_sink;
/* Four independent FMA chains: enough ILP to reach the issue rate rather
   than measuring FMA latency. Flop counts assume FMA = 2 flop. */
static double f64_scalar(void){ struct timespec t0,t1;
  double a=1.0000001,b=0.9999999,c=1.0000002,d=0.9999998;
  clock_gettime(CLOCK_MONOTONIC,&t0);
  for(long i=0;i<N;i++){a=a*b+.1;b=b*c+.1;c=c*d+.1;d=d*a+.1;}
  clock_gettime(CLOCK_MONOTONIC,&t1); g_sink=a+b+c+d;
  return (8.0*N)/(el(t0,t1)/1e3); }
static double f64_neon(void){ struct timespec t0,t1;
  v2 a={1.0000001,1.0000003},b={0.9999999,0.9999997},
     c={1.0000002,1.0000004},d={0.9999998,0.9999996},k={.1,.1};
  clock_gettime(CLOCK_MONOTONIC,&t0);
  for(long i=0;i<N;i++){a=a*b+k;b=b*c+k;c=c*d+k;d=d*a+k;}
  clock_gettime(CLOCK_MONOTONIC,&t1); g_sink=a[0]+b[0]+c[0]+d[0];
  return (16.0*N)/(el(t0,t1)/1e3); }
static double f32_neon(void){ struct timespec t0,t1;
  v4 a={1,1,1,1},b={1,1,1,1},c={1,1,1,1},d={1,1,1,1},k={.1f,.1f,.1f,.1f};
  clock_gettime(CLOCK_MONOTONIC,&t0);
  for(long i=0;i<N;i++){a=a*b+k;b=b*c+k;c=c*d+k;d=d*a+k;}
  clock_gettime(CLOCK_MONOTONIC,&t1); g_sink=a[0]+b[0]+c[0]+d[0];
  return (32.0*N)/(el(t0,t1)/1e3); }
/* Dependent integer adds, one per cycle on A53 -> approximate core clock.
   asm volatile so the optimiser cannot fold the chain into a single add;
   unrolled x8 so the loop's own increment/compare/branch does not dominate. */
static double clock_mhz(void){ struct timespec t0,t1; long x=0;
  clock_gettime(CLOCK_MONOTONIC,&t0);
  for(long i=0;i<N/8;i++) __asm__ volatile(
    "add %0,%0,#1\n\tadd %0,%0,#1\n\tadd %0,%0,#1\n\tadd %0,%0,#1\n\t"
    "add %0,%0,#1\n\tadd %0,%0,#1\n\tadd %0,%0,#1\n\tadd %0,%0,#1":"+r"(x));
  clock_gettime(CLOCK_MONOTONIC,&t1); g_sink=(double)x;
  return (N/8*8)/(el(t0,t1)/1e3); }
/* Subnormal penalty: the classic x86 RT-jitter trap. Operands are refreshed
   from memory every iteration so the maths cannot be hoisted out of the loop. */
static double buf[64];
static double subnorm_ns(double v){ struct timespec t0,t1; double acc=0;
  for(int w=0;w<64;w++) buf[w]=v;
  clock_gettime(CLOCK_MONOTONIC,&t0);
  for(long i=0;i<2000000;i++){ acc+=buf[i&63]*buf[(i+1)&63]+buf[i&63]/3.0; buf[i&63]=v; }
  clock_gettime(CLOCK_MONOTONIC,&t1); g_sink=acc;
  return el(t0,t1)/2000000.0; }
int main(int argc,char**argv){
  uint64_t f; __asm__ volatile("mrs %0, fpcr":"=r"(f));
  if(argc>1&&!strcmp(argv[1],"--fpcr")){
    printf("0x%08llx\n",(unsigned long long)f); return 0; }
  f64_scalar();                                     /* warm */
  printf("clock_mhz %.0f\n",       clock_mhz());
  printf("f64_scalar %.0f\n",      f64_scalar());
  printf("f64_neon %.0f\n",        f64_neon());
  printf("f32_neon %.0f\n",        f32_neon());
  double n=subnorm_ns(1.5e-3), d=subnorm_ns(5.0e-319);
  printf("normal_ns %.3f\nsubnormal_ns %.3f\nsubnormal_ratio %.2f\n",n,d,d/n);
  return 0; }
