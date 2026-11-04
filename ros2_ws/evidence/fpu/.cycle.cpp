/* Times the exact RBDL calls FullDynControllerRT issues every 1 kHz cycle.
   Same URDF, same 6 DOF, same call sequence — see ComputeComputedTorque()
   and ComputeFullDynamics() in FullDynControllerRT.cpp. */
#include <rbdl/rbdl.h>
#include <rbdl/addons/urdfreader/urdfreader.h>
#include <cstdio>
#include <ctime>
using namespace RigidBodyDynamics; using namespace RigidBodyDynamics::Math;
static double el(timespec a,timespec b){return (b.tv_sec-a.tv_sec)*1e9+(b.tv_nsec-a.tv_nsec);}
#define REP 20000
/* q is nudged each iteration so no call can be cached or hoisted. */
#define TIME(key,expr) { timespec t0,t1; for(int i=0;i<200;i++){expr;} \
  clock_gettime(CLOCK_MONOTONIC,&t0); for(int i=0;i<REP;i++){q[0]+=1e-9;expr;} \
  clock_gettime(CLOCK_MONOTONIC,&t1); double us=el(t0,t1)/REP/1000.0; \
  printf("%s %.3f\n",key,us); }
int main(int argc,char**argv){
  Model m; if(!Addons::URDFReadFromFile(argv[1],&m,false)){fprintf(stderr,"urdf fail\n");return 1;}
  int dof=m.dof_count;
  VectorNd q=VectorNd::Zero(dof),qd=VectorNd::Zero(dof),z=VectorNd::Zero(dof),
           h=VectorNd::Zero(dof),g=VectorNd::Zero(dof);
  MatrixNd M=MatrixNd::Zero(dof,dof),J=MatrixNd::Zero(6,dof);
  q<<0.45,-0.43,-2.13,-0.04,0.67,-0.56; qd.setConstant(0.1);
  unsigned bid=m.GetBodyId(m.GetBodyName(m.mBodies.size()-1).c_str());
  Vector3d tcp(0,0,0.06),p; Matrix3d R;
  printf("dof %d\n",dof);
  TIME("crba_us",   CompositeRigidBodyAlgorithm(m,q,M))
  TIME("nle_us",    NonlinearEffects(m,q,qd,h))
  TIME("grav_us",   InverseDynamics(m,q,z,z,g))
  TIME("fk_us",     p=CalcBodyToBaseCoordinates(m,q,bid,tcp))
  TIME("jac6d_us",  CalcPointJacobian6D(m,q,bid,tcp,J))
  TIME("bwo_us",    R=CalcBodyWorldOrientation(m,q,bid))
  return 0; }
