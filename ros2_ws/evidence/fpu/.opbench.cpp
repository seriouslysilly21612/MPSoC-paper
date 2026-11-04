#include <rbdl/rbdl.h>
#include <rbdl/addons/urdfreader/urdfreader.h>
#include <csignal>
#include <cstdio>
#include <cstdlib>
using namespace RigidBodyDynamics; using namespace RigidBodyDynamics::Math;
int main(int argc,char**argv){
  Model m; if(!Addons::URDFReadFromFile(argv[1],&m,false))return 1;
  int dof=m.dof_count, which=atoi(argv[2]);
  VectorNd q=VectorNd::Zero(dof),qd=VectorNd::Zero(dof),z=VectorNd::Zero(dof),
           h=VectorNd::Zero(dof),g=VectorNd::Zero(dof);
  MatrixNd M=MatrixNd::Zero(dof,dof),J=MatrixNd::Zero(6,dof);
  q<<0.45,-0.43,-2.13,-0.04,0.67,-0.56; qd.setConstant(0.1);
  unsigned bid=m.GetBodyId(m.GetBodyName(m.mBodies.size()-1).c_str());
  Vector3d tcp(0,0,0.06),p; Matrix3d R;
  for(int i=0;i<50;i++){ CompositeRigidBodyAlgorithm(m,q,M); NonlinearEffects(m,q,qd,h);
    InverseDynamics(m,q,z,z,g); p=CalcBodyToBaseCoordinates(m,q,bid,tcp);
    CalcPointJacobian6D(m,q,bid,tcp,J);
    R=CalcBodyWorldOrientation(m,q,bid); }   /* warm: fault pages, resolve PLT */
  raise(SIGUSR1);
  switch(which){
    case 0: CompositeRigidBodyAlgorithm(m,q,M); break;
    case 1: NonlinearEffects(m,q,qd,h); break;
    case 2: InverseDynamics(m,q,z,z,g); break;
    case 3: p=CalcBodyToBaseCoordinates(m,q,bid,tcp); break;
    case 4: CalcPointJacobian6D(m,q,bid,tcp,J); break;
    case 5: R=CalcBodyWorldOrientation(m,q,bid); break; }
  raise(SIGUSR2);
  printf("%g\n",p[0]+M(0,0)+h[0]+g[0]+J(0,0)+R(0,0)); return 0; }
