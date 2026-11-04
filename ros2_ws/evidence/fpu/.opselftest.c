/* Known-answer check: exactly 2000 FP instructions inside the traced region,
   written in asm so the count does not depend on the optimiser. */
#include <signal.h>
#include <stdio.h>
int main(void){ double a=1.5; raise(SIGUSR1);
  for(int i=0;i<1000;i++) __asm__ volatile("fmadd d0,d0,d0,d0":::"d0");
  for(int i=0;i<1000;i++) __asm__ volatile("fmla v1.2d,v1.2d,v1.2d":::"v1");
  raise(SIGUSR2); printf("%g\n",a); return 0; }
