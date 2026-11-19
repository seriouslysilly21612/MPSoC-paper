/* The RBDL model of indy7.urdf as the URDF reader of the controller builds it. Written by rbdl_model_code:
 * not to be edited. Per body: parent, E and r of the joint frame, joint axis, mass, centre of mass,
 * inertia at the centre of mass, name. Per fixed body: moving parent, E and r of its frame, name. */
#ifndef CTRL_MODEL_DATA_H
#define CTRL_MODEL_DATA_H

#include "ctrl_model.h"

#define CTRL_MODEL_CRC    0x573BA374u	/* of the names below and of the numbers, each on a grid of 2^-40 */
#define CTRL_MODEL_BODIES 6
#define CTRL_MODEL_FIXED  3

static const ctrl_model_body_t CTRL_MODEL_BODY[CTRL_MODEL_BODIES] =
{
	{ 0u, { 0x1p+0, 0x0p+0, 0x0p+0, 0x0p+0, 0x1p+0, 0x0p+0, 0x0p+0, 0x0p+0, 0x1p+0 }, { 0x0p+0, 0x0p+0, 0x1.3d70a3d70a3d7p-4 }, { 0x0p+0, 0x0p+0, 0x1p+0, 0x0p+0, 0x0p+0, 0x0p+0 }, { 0x1.6e38e574637f3p+3 }, { 0x1.0c6f7a0b5ed8dp-20, -0x1.3c968943e1006p-5, 0x1.34b51372a38b9p-3 }, { 0x1.dc75cf1630516p-4, 0x1.7def6946280ap-26, -0x1.f8a5caf0341cap-22, 0x1.7def6946280ap-26, 0x1.bd95a3ce66c75p-4, 0x1.d2f85124c41ebp-6, -0x1.f8a5caf0341cap-22, 0x1.d2f85124c41ebp-6, 0x1.cd3ff7348dd8fp-5 }, "link1" },
	{ 1u, { -0x1.1a62633145b51p-54, 0x1.408f068397065p-76, -0x1p+0, 0x1p+0, -0x1.a79398p-53, -0x1.1a62633145b51p-54, -0x1.a79398p-53, -0x1p+0, -0x1.408f067ff08f9p-76 }, { 0x0p+0, -0x1.be76c8b439581p-4, 0x1.c6a7ef9db22d1p-3 }, { 0x0p+0, 0x0p+0, 0x1p+0, 0x0p+0, 0x0p+0, 0x0p+0 }, { 0x1.764026ec52f9fp+2 }, { -0x1.fc6a7ef9db22dp-3, -0x1.0c6f7a0b5ed8dp-19, 0x1.39756c93a7115p-4 }, { 0x1.3641c57284683p-6, 0x1.cb0ffcd9c6adp-19, -0x1.116cf51721f84p-7, 0x1.cb0ffcd9c6adp-19, 0x1.61bb1158f345ap-3, 0x1.312e26b08904ap-20, -0x1.116cf51721f84p-7, 0x1.312e26b08904ap-20, 0x1.5d0c7d5eb08d3p-3 }, "link2" },
	{ 2u, { 0x1p+0, 0x0p+0, 0x0p+0, 0x0p+0, 0x1p+0, 0x0p+0, 0x0p+0, 0x0p+0, 0x1p+0 }, { -0x1.ccccccccccccdp-2, 0x0p+0, -0x1.f3b645a1cac08p-6 }, { 0x0p+0, 0x0p+0, 0x1p+0, 0x0p+0, 0x0p+0, 0x0p+0 }, { 0x1.574dc357582b9p+1 }, { -0x1.08d08919ef955p-3, -0x1.339b3e65b0c46p-33, -0x1.27c49fd7a13c2p-4 }, { 0x1.3c877f375d45bp-8, -0x1.a6e326f95f792p-24, -0x1.06df406c0f0cap-9, -0x1.a6e326f95f792p-24, 0x1.08c4af969ab88p-5, -0x1.4c5b970f46dd9p-28, -0x1.06df406c0f0cap-9, -0x1.4c5b970f46dd9p-28, 0x1.070f47cd6ed52p-5 }, "link3" },
	{ 3u, { -0x1.c307172cece67p-33, 0x1p+0, -0x1p-54, 0x1.c307172cece67p-33, -0x1.ff9cabc084addp-55, -0x1p+0, -0x1p+0, -0x1.c307172cece67p-33, -0x1.c307172cece67p-33 }, { -0x1.116872b020c4ap-2, 0x0p+0, -0x1.3333333333333p-4 }, { 0x0p+0, 0x0p+0, 0x1p+0, 0x0p+0, 0x0p+0, 0x0p+0 }, { 0x1.109fb3a487b44p+1 }, { -0x1.0c6f7a0b5ed8dp-19, -0x1.24af0bf1a5ca3p-5, 0x1.a98eda22f6a51p-5 }, { 0x1.10880dacc8ffap-7, 0x1.6321ddab355ep-23, 0x1.4e419f71e3664p-24, 0x1.6321ddab355ep-23, 0x1.1dd80af885abap-8, 0x1.34f7fd11ad73ap-9, 0x1.4e419f71e3664p-24, 0x1.34f7fd11ad73ap-9, 0x1.bb0cf3aabde86p-8 }, "link4" },
	{ 4u, { -0x1.1a62633145b51p-54, 0x1.408f068397065p-76, -0x1p+0, 0x1p+0, -0x1.a79398p-53, -0x1.1a62633145b51p-54, -0x1.a79398p-53, -0x1p+0, -0x1.408f067ff08f9p-76 }, { 0x0p+0, -0x1.d2f1a9fbe76c9p-4, 0x1.53f7ced916873p-4 }, { 0x0p+0, 0x0p+0, 0x1p+0, 0x0p+0, 0x0p+0, 0x0p+0 }, { 0x1.1cb00d8eded59p+1 }, { -0x1.742fe82517e78p-5, 0x1.92a737110e454p-19, 0x1.fec3547e06962p-5 }, { 0x1.141e38c9dc2fbp-8, 0x1.30fc2bd8d11f2p-20, 0x1.5d16266f9ec59p-10, 0x1.30fc2bd8d11f2p-20, 0x1.5560505ce5231p-7, -0x1.c4ee73774ed82p-21, 0x1.5d16266f9ec59p-10, -0x1.c4ee73774ed82p-21, 0x1.495d22612d2e9p-7 }, "link5" },
	{ 5u, { -0x1.c307172cece67p-33, 0x1p+0, -0x1p-54, 0x1.c307172cece67p-33, -0x1.ff9cabc084addp-55, -0x1p+0, -0x1p+0, -0x1.c307172cece67p-33, -0x1.c307172cece67p-33 }, { -0x1.5810624dd2f1bp-3, 0x0p+0, 0x1.1a9fbe76c8b44p-4 }, { 0x0p+0, 0x0p+0, 0x1p+0, 0x0p+0, 0x0p+0, 0x0p+0 }, { 0x1.87bb024a07353p-2 }, { -0x1.4bbae7f37edc7p-33, -0x1.b97353b4b2fa9p-12, 0x1.01a79fec99f1bp-5 }, { 0x1.3c6e1273d5243p-12, 0x1.fe8eb3c59eadfp-25, 0x1.0d2296d042b95p-48, 0x1.fe8eb3c59eadfp-25, 0x1.3b11824a5c9d8p-12, 0x1.be44022775ebcp-25, 0x1.0d2296d042b95p-48, 0x1.be44022775ebcp-25, 0x1.9b0f35b66a098p-12 }, "link6" },
};

static const ctrl_model_fixed_t CTRL_MODEL_FIXED_BODY[CTRL_MODEL_FIXED] =
{
	{ 0u, { 0x1p+0, 0x0p+0, 0x0p+0, 0x0p+0, 0x1p+0, 0x0p+0, 0x0p+0, 0x0p+0, 0x1p+0 }, { 0x0p+0, 0x0p+0, 0x0p+0 }, "world" },
	{ 0u, { 0x1p+0, 0x0p+0, 0x0p+0, 0x0p+0, 0x1p+0, 0x0p+0, 0x0p+0, 0x0p+0, 0x1p+0 }, { 0x0p+0, 0x0p+0, 0x0p+0 }, "link0" },
	{ 6u, { 0x1p+0, 0x0p+0, 0x0p+0, 0x0p+0, 0x1p+0, 0x0p+0, 0x0p+0, 0x0p+0, 0x1p+0 }, { 0x0p+0, 0x0p+0, 0x1.eb851eb851eb8p-5 }, "tcp" },
};

#endif /* CTRL_MODEL_DATA_H */
