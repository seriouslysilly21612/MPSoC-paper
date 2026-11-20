/*
 * check_model.cpp  --  the model that the firmware builds from numbers (src/ctrl/ctrl_model.cpp) is the model that
 * the URDF reader of the application builds from the file.  Guards the controller on the R5 against computing
 * with another arm than the controller on the APU: a stale ctrl_model_indy7.h, a reader that changed, a body
 * added in another way than the reader adds it.  On a PC, with the RBDL of the application:
 *
 *   g++ -std=gnu++17 -O2 -Isrc/ctrl -isystem $RBDL_DIR/include -isystem /usr/include/eigen3 \
 *       test/check_model.cpp src/ctrl/ctrl_model.cpp -L$RBDL_DIR/lib -lrbdl -lrbdl_urdfreader \
 *       -Wl,-rpath,$RBDL_DIR/lib -o /tmp/check_model
 *   /tmp/check_model ../../goal-to-torque_MPSoC/indy7/models/indy7.urdf
 *
 * Every number is compared with ==: both models are made by the same calls from the same numbers (a zero may
 * differ in sign).  Exit 0 if no check failed.
 */
#include <cstdio>
#include <random>

#include <rbdl/rbdl.h>
#include <rbdl/addons/urdfreader/urdfreader.h>

#include "ctrl_model.h"

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

static int failed = 0;

static void expect(bool ok, const char *what)
{
    if (ok)
        return;
    failed++;
    printf("  FAILED: %s\n", what);
}

static bool same_frame(const SpatialTransform &a, const SpatialTransform &b) { return a.E == b.E && a.r == b.r; }

int main(int argc, char **argv)
{
    if (argc != 2)
    {
        fprintf(stderr, "usage: %s URDF\n", argv[0]);
        return 2;
    }
    Model read, built;
    if (!Addons::URDFReadFromFile(argv[1], &read, false, false))
    {
        fprintf(stderr, "%s: the URDF reader cannot read it\n", argv[1]);
        return 2;
    }
    expect(ctrl_model_build(&built), "the numbers go into an empty model");
    expect(!ctrl_model_build(&built), "and not into one that has bodies");

    /* what the reader built, body by body */
    bool bodies = read.mBodies.size() == built.mBodies.size() && read.dof_count == built.dof_count &&
                  read.q_size == built.q_size && read.qdot_size == built.qdot_size;
    for (size_t i = 1; bodies && i < read.mBodies.size(); i++)
        bodies = read.lambda[i] == built.lambda[i] && same_frame(read.X_T[i], built.X_T[i]) &&
                 read.mJoints[i].mJointType == built.mJoints[i].mJointType &&
                 read.mJoints[i].q_index == built.mJoints[i].q_index && read.S[i] == built.S[i] &&
                 read.mBodies[i].mMass == built.mBodies[i].mMass &&
                 read.mBodies[i].mCenterOfMass == built.mBodies[i].mCenterOfMass &&
                 read.mBodies[i].mInertia == built.mBodies[i].mInertia && read.I[i].m == built.I[i].m &&
                 read.I[i].h == built.I[i].h && read.I[i].toMatrix() == built.I[i].toMatrix();
    expect(bodies, "the moving bodies: parent, joint frame, joint, mass, centre of mass, inertia");
    bool fixed = read.mFixedBodies.size() == built.mFixedBodies.size();
    for (size_t i = 0; fixed && i < read.mFixedBodies.size(); i++)
        fixed = read.mFixedBodies[i].mMovableParent == built.mFixedBodies[i].mMovableParent &&
                same_frame(read.mFixedBodies[i].mParentTransform, built.mFixedBodies[i].mParentTransform);
    expect(fixed, "the fixed bodies: moving parent and frame");
    expect(read.mBodyNameMap == built.mBodyNameMap, "every name has the same body");

    /* and what the controller computes with it, at 1000 states */
    const unsigned tcp = read.GetBodyId("tcp");
    expect(tcp == built.GetBodyId("tcp") && tcp != std::numeric_limits<unsigned>::max(), "the tool frame \"tcp\"");
    read.gravity = built.gravity = Vector3d(0., 0., -9.81);
    const int n = (int)read.dof_count;
    std::mt19937_64 rng(20261118);
    std::uniform_real_distribution<double> angle(-3.0, 3.0), speed(-2.0, 2.0);
    VectorNd q(n), qd(n), zero = VectorNd::Zero(n), g1(n), g2(n), h1(n), h2(n);
    MatrixNd M1 = MatrixNd::Zero(n, n), M2 = MatrixNd::Zero(n, n), J1 = MatrixNd::Zero(3, n), J2 = MatrixNd::Zero(3, n);
    const Vector3d point(0., 0., 0.);
    bool gravity = true, dynamics = true, tool = true;
    double largest = 0.0;
    for (int k = 0; k < 1000; k++)
    {
        for (int i = 0; i < n; i++)
        {
            q[i] = angle(rng);
            qd[i] = speed(rng);
        }
        InverseDynamics(read, q, zero, zero, g1);
        InverseDynamics(built, q, zero, zero, g2);
        gravity = gravity && g1 == g2;
        UpdateKinematics(read, q, qd, zero);
        UpdateKinematics(built, q, qd, zero);
        M1.setZero();
        M2.setZero();
        CompositeRigidBodyAlgorithm(read, q, M1, false);
        CompositeRigidBodyAlgorithm(built, q, M2, false);
        NonlinearEffects(read, q, qd, h1);
        NonlinearEffects(built, q, qd, h2);
        dynamics = dynamics && M1 == M2 && h1 == h2;
        J1.setZero();
        J2.setZero();
        CalcPointJacobian(read, q, tcp, point, J1, false);
        CalcPointJacobian(built, q, tcp, point, J2, false);
        tool = tool && J1 == J2 &&
               CalcBodyToBaseCoordinates(read, q, tcp, point, false) == CalcBodyToBaseCoordinates(built, q, tcp, point, false) &&
               CalcBodyWorldOrientation(read, q, tcp, false) == CalcBodyWorldOrientation(built, q, tcp, false);
        largest = std::max(largest, g1.cwiseAbs().maxCoeff());
    }
    expect(gravity, "g(q) at 1000 states");
    expect(dynamics, "M(q) and h(q, qd) at 1000 states");
    expect(tool, "position, orientation and Jacobian of the tool at 1000 states");
    expect(largest > 10.0, "the states load the arm (a gravity torque above 10 N m among them)");
    printf("%s: %d checks failed (%d joints, %zu fixed bodies)\n", failed == 0 ? "PASS" : "FAIL", failed, n,
           read.mFixedBodies.size());
    return failed == 0 ? 0 : 1;
}
