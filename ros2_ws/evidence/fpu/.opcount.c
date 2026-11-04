#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <signal.h>
#include <sys/ptrace.h>
#include <sys/wait.h>
#include <sys/user.h>
#include <sys/uio.h>
#include <elf.h>
#include <stdint.h>
#include <stdlib.h>
static int getpc(pid_t p,uint64_t*pc){ struct user_regs_struct r;
  struct iovec io={&r,sizeof(r)};
  if(ptrace(PTRACE_GETREGSET,p,NT_PRSTATUS,&io)<0) return -1; *pc=r.pc; return 0; }
static FILE* dumpf=NULL;
int main(int argc,char**argv){
  if(getenv("OPDUMP")) dumpf=fopen(getenv("OPDUMP"),"w");
  if(argc<2){fprintf(stderr,"usage: %s <prog> [args...]\n",argv[0]);return 2;}
  pid_t pid=fork();
  if(pid==0){ ptrace(PTRACE_TRACEME,0,0,0);
    if(!freopen("/dev/null","w",stdout)) _exit(126);
    execv(argv[1],&argv[1]); _exit(127); }
  int st; waitpid(pid,&st,0);
  ptrace(PTRACE_SETOPTIONS,pid,0,PTRACE_O_EXITKILL);
  /* Phase 1: run free. URDF parsing and warm-up are millions of instructions
     and single-stepping them would take minutes. */
  int sig=0;
  for(;;){ if(ptrace(PTRACE_CONT,pid,0,sig)<0){fprintf(stderr,"cont failed\n");return 1;}
    sig=0;
    if(waitpid(pid,&st,0)<0||WIFEXITED(st)||WIFSIGNALED(st)){
      fprintf(stderr,"child gone before SIGUSR1\n"); return 1; }
    int s=WSTOPSIG(st); if(s==SIGUSR1) break; if(s!=SIGTRAP) sig=s; }
  /* Phase 2: single-step the region of interest, decoding each instruction. */
  unsigned long long total=0,fpsimd=0,ldst=0,branch=0;
  for(;;){
    if(ptrace(PTRACE_SINGLESTEP,pid,0,sig)<0) break; sig=0;
    if(waitpid(pid,&st,0)<0) break;
    if(WIFEXITED(st)||WIFSIGNALED(st)||!WIFSTOPPED(st)) break;
    int s=WSTOPSIG(st);
    if(s==SIGUSR2) break;
    if(s!=SIGTRAP){ sig=s; continue; }
    uint64_t pc; if(getpc(pid,&pc)<0) break;
    errno=0; long w=ptrace(PTRACE_PEEKTEXT,pid,(void*)pc,0); if(errno) continue;
    uint32_t insn=(uint32_t)w, op0=(insn>>25)&0xF;
    total++;
    /* x111 is the FP/SIMD *unit*, which is wider than "floating-point
       arithmetic": it also holds NEON integer ops and the crypto extensions.
       The bit test cannot tell them apart, so every word in this bucket is
       dumped and handed to objdump for classification by real mnemonic. */
    if((op0&0x7)==0x7){ fpsimd++; if(dumpf) fprintf(dumpf,"%08x\n",insn); }
    else if((op0&0x5)==0x4) ldst++;     /* x1x0 loads and stores          */
    else if((op0&0xE)==0xA) branch++;   /* 101x branch / system           */ }
  kill(pid,SIGKILL); waitpid(pid,&st,0);
  if(dumpf) fclose(dumpf);
  printf("%llu %llu %llu %llu\n",total,fpsimd,ldst,branch); return 0; }
