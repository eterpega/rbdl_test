/*
 * RBDL - Rigid Body Dynamics Library
 * Copyright (c) 2011-2016 Martin Felis <martin@fysx.org>
 *
 * Licensed under the zlib license. See LICENSE for more details.
 */

#include <iostream>
#include <rbdl/rbdl.h>

using namespace std;
using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

const double TEST_PREC = 1.0e-14;


int main (int argc, char* argv[]) {
	// rbdl_check_api_version (RBDL_API_VERSION);

	Model* model = NULL;

	unsigned int 	body_1_id, 
					body_2_id, 
					body_3_id, 
					body_4_id, 
					body_e_id;//endeffector
	Body 	body_1, 
			body_2, 
			body_3, 
			body_4, 
			body_e; //end
	
	Joint 	joint_1, 
			joint_2, 
			joint_3, 
			joint_4, 
			joint_e; //end

	model = new Model();
	model->gravity = Vector3d (0., 0., -9.81);




/////////////////////////////////////////////////////////////////////////////////////////

//////OK_BASIC
	// body_a = Body (1., Vector3d (0.1, 0., 0.0), Vector3d (1., 1., 1.));
	// 	joint_a = Joint (JointTypeRevolute,Vector3d (0., 0., 1.));
	// 	body_a_id = model->AddBody(0,  Xtrans(Vector3d(0., 0., 0.)), joint_a, body_a);
	
	// body_b = Body (1., Vector3d (0., 0.5, 0.), Vector3d (1., 1., 1.));
	// 	joint_b =Joint (JointTypeRevolute,Vector3d (0., 0., 1.));
	// 	body_b_id = model->AddBody(body_a_id,  Xtrans(Vector3d(0., 2., 0.)), joint_b, body_b);
	
	// body_c = Body (1., Vector3d (0.5, 0., 0.), Vector3d (1., 1., 1.));
	// 	joint_c = Joint (JointTypeRevolute,Vector3d (0., 0., 1.));
	// 	body_c_id = model->AddBody(body_b_id,  Xtrans(Vector3d(0., 3., 0.)), joint_c, body_c);




//////OK_TEST
	// body_1 = Body (1., Vector3d (0.1, 0., 0.0), Vector3d (1., 1., 1.));
	// 	joint_1 = Joint (JointTypeRevolute,Vector3d (0., 0., 1.));
	// 	body_1_id = model->AddBody(0,  Xtrans(Vector3d(0., 0., 0.)), joint_1, body_1);
	
	// body_2 = Body (1., Vector3d (0., 0.5, 0.), Vector3d (1., 1., 1.));
	// 	joint_2 =Joint (JointTypeRevolute,Vector3d (0., -1., 0.));
	// 	body_2_id = model->AddBody(body_1_id,  Xtrans(Vector3d(0., 0., -0.2)), joint_2, body_2);
	
	// body_3 = Body (1., Vector3d (0.5, 0., 0.), Vector3d (1., 1., 1.));
	// 	joint_3 = Joint (JointTypeRevolute,Vector3d (0., -1., 0.));
	// 	body_3_id = model->AddBody(body_2_id,  Xtrans(Vector3d(0., 0., -0.4)), joint_3, body_3);

	// body_4 = Body (1., Vector3d (0.5, 0., 0.), Vector3d (1., 1., 1.));
	// 	joint_4 = Joint (JointTypeRevolute,Vector3d (0., -1., 0.));
	// 	body_4_id = model->AddBody(body_3_id,  Xtrans(Vector3d(0., 0., -0.3)), joint_4, body_4);

	// body_e = Body (1., Vector3d (0.5, 0., 0.), Vector3d (1., 1., 1.));
	// 	joint_e = Joint (JointTypeFixed);
	// 	body_e_id = model->AddBody(body_4_id,  Xtrans(Vector3d(0., 0., -0.1)), joint_5, body_5);





////OK_TEST
SpatialTransform rbdl_joint_frame1 = SpatialTransform( 
		Matrix3d(	1., 0., 0.,		// w.r.t prev_coord. (axis_old_xyz against new_frame)
        			0., 1., 0.,
        			0., 0., 1.	),
		Vector3d(	0., 0., 0.	)	// w.r.t prev_coord. (translation about old_xyz)
);	   

SpatialTransform rbdl_joint_frame2 = SpatialTransform( 
		Matrix3d(	1., 0., 0.,       
        			0., 0., 1.,
        			0., -1., 0.	),   
		Vector3d(	0., 0,  -0.2	) 
);	   

SpatialTransform rbdl_joint_frame3 = SpatialTransform( 
		Matrix3d(	1., 0., 0.,
        			0., 1., 0.,
        			0., 0., 1.	),
		Vector3d(	0., -0.4, 0.	)
);	   

SpatialTransform rbdl_joint_frame4 = SpatialTransform( 
		Matrix3d(	1., 0., 0.,
        			0., 1., 0.,
        			0., 0., 1.	),
		Vector3d(	0.,  -0.3, 0	)
);	   

SpatialTransform rbdl_joint_frame5 = SpatialTransform( 
		Matrix3d(	1., 0., 0.,
        			0., 1., 0.,
        			0., 0., 1.	),
		Vector3d(	0., -0.1, 0.	)
);	   

std::cout << "frame1: \n" << rbdl_joint_frame1 << std::endl;
std::cout << "frame2: \n" << rbdl_joint_frame2 << std::endl;
std::cout << "frame3: \n" << rbdl_joint_frame3 << std::endl << std::endl;


	body_1 = Body (1., Vector3d (0.1, 0., 0.0), Matrix3d (0.1, 0., 0.,  0., 0.1, 0.,  0., 0., 0.1));
		joint_1 = Joint (JointTypeRevolute,Vector3d (0., 0., 1.));
		body_1_id = model->AddBody(0, rbdl_joint_frame1, joint_1, body_1);
	
	body_2 = Body (1., Vector3d (0., 0.5, 0.), Vector3d (1., 1., 1.));
		joint_2 =Joint (JointTypeRevolute,Vector3d (0., 0., 1.));
		body_2_id = model->AddBody(body_1_id, rbdl_joint_frame2, joint_2, body_2);
	
	body_3 = Body (1., Vector3d (0.5, 0., 0.), Vector3d (1., 1., 1.));
		joint_3 = Joint (JointTypeRevolute,Vector3d (0., 0., 1.));
		body_3_id = model->AddBody(body_2_id, rbdl_joint_frame3, joint_3, body_3);
	
	body_4 = Body (1., Vector3d (0.5, 0., 0.), Vector3d (1., 1., 1.));
		joint_4 = Joint (JointTypeRevolute,Vector3d (0., 0., 1.));
		body_4_id = model->AddBody(body_3_id, rbdl_joint_frame4, joint_4, body_4);

	body_e = Body (1., Vector3d (0.5, 0., 0.), Vector3d (1., 1., 1.));
		joint_e = Joint (JointTypeFixed);
		body_e_id = model->AddBody(body_4_id, rbdl_joint_frame5, joint_e, body_e);









////////////////////////////////////////


/////////////////////////////////////////////////////////////////////////////////
///// calc Kin
/////////////////////////////////////////////////////////////////////////////////

	VectorNd Q = VectorNd::Zero (model->q_size);
	VectorNd QDot = VectorNd::Zero (model->q_size);
	VectorNd QDDot = VectorNd::Zero (model->q_size);
	VectorNd Tau = VectorNd::Zero (model->q_size);
	
	VectorNd Vel = VectorNd::Zero(3);
	VectorNd Vel6d = VectorNd::Zero(6);
	VectorNd Accel = VectorNd::Zero(3);
	VectorNd Accel6d = VectorNd::Zero(6);	
	VectorNd FKin = VectorNd::Zero(3);
	VectorNd tmpVec = VectorNd::Zero(3);

	Vector3d position = Vector3d(0., 0., 0.);
	VectorNd pos = VectorNd::Zero(3);
	VectorNd w_xyz = VectorNd::Zero(3);
	Matrix3d RotM;
	Quaternion quat;
	double qw,qx,qy,qz;


Q(0)=0.*M_PI/180.;
Q(1)=0.0*M_PI/180.;
Q(2)=0.*M_PI/180.;
Q(3)=0.0*M_PI/180.;
std::cout << "Q: " << Q.transpose() << std::endl;

QDot[0]=0.*M_PI/180.;
QDot[1]=0.*M_PI/180.;
QDot[2]=0.*M_PI/180.;
QDot[3]=0.*M_PI/180.;
std::cout << "QDot: " << QDot.transpose() << std::endl;


	// pos = CalcBaseToBodyCoordinates ( *model, Q, body_b_id, tmpVec, true);
	// std::cout << "pos: " << pos.transpose() << std::endl;

	pos = CalcBodyToBaseCoordinates (*model, Q, body_e_id, tmpVec, true);
	std::cout << "pos: " << pos.transpose() << std::endl;
	
	RotM = CalcBodyWorldOrientation(*model, Q, body_e_id, true);
	std:cout << "Rot: \n" << RotM << std::endl;

	quat = Quaternion::fromMatrix(RotM);
	std::cout << "Quat: " << quat.transpose() << std::endl;

	w_xyz = CalcAngularVelocityfromMatrix (RotM);
	std::cout << "w_xyz: " << w_xyz.transpose() << std::endl;


	Vel=CalcPointVelocity(*model, Q, QDot, body_e_id, pos);
	std::cout << "Vel: " << Vel.transpose() << std::endl;
	Vel6d = CalcPointVelocity6D(*model, Q, QDot, body_e_id, pos, true);
	std::cout << "Vel6d: " << Vel6d.transpose() << std::endl;

	Accel = CalcPointAcceleration (*model, Q, QDot, QDDot, body_e_id, pos, true);
	std::cout << "Accel: " << Accel.transpose() << std::endl;
	Accel6d = CalcPointAcceleration6D(*model, Q, QDot, QDDot, body_e_id, pos, true);
	std::cout << "Accel6d: " << Accel6d.transpose() << std::endl;


MatrixNd JacoM3Xn = MatrixNd::Zero(3, model->q_size);
	CalcPointJacobian (*model, Q, body_e_id, pos, JacoM3Xn, true);
	std::cout << "JacoM3Xn: \n" << JacoM3Xn << std::endl;
MatrixNd JacoM6Xn = MatrixNd::Zero(6, model->q_size);	
	CalcPointJacobian6D(*model, Q, body_e_id, pos, JacoM6Xn, true);
	std::cout << "JacoM6Xn: \n" << JacoM6Xn << std::endl;

MatrixNd SJacoM6Xn = MatrixNd::Zero(6, model->q_size);	
	CalcBodySpatialJacobian (*model, Q, body_e_id, SJacoM6Xn, true);
	std::cout << "SJacoM6Xn: \n" << SJacoM6Xn << std::endl;



VectorNd Qinit = VectorNd::Zero(model->q_size);
VectorNd Qres  = VectorNd::Zero(model->q_size);

InverseKinematicsConstraintSet cs;
cs.num_steps=1000;
cs.max_steps=5000;
cs.step_tol=1.0e-20;
cs.lambda=0.9;
cs.AddPointConstraint (body_e_id, pos, pos);
InverseKinematics (*model, Qinit, cs ,Qres);
std::cout << "ik: " << Qres.transpose() << std::endl;


/////////////////////////////////////////////////////////////////////////////////
///// calc Dyn
/////////////////////////////////////////////////////////////////////////////////
Tau(0)=0.;
Tau(1)=0.;
Tau(2)=0.;
Tau(3)=0.;
 	ForwardDynamics (*model, Q, QDot, Tau, QDDot);
	std::cout << "QDDot: " << QDDot.transpose() << std::endl;
	ForwardDynamicsLagrangian(*model, Q, QDot, Tau, QDDot);
	std::cout << "QDDot: " << QDDot.transpose() << std::endl;

VectorNd Tau_inv = VectorNd::Zero (model->q_size);
std::vector<SpatialVector> *f_ext;
	InverseDynamics (*model, Q, QDot, QDDot, Tau_inv);
	std::cout << "Tau    : " << Tau.transpose() << std::endl;
	std::cout << "Tau_inv: " << Tau_inv.transpose() << std::endl;

MatrixNd H = MatrixNd::Zero(model->q_size, model->q_size);
	// H : Jointspace inertia matrix 
	CompositeRigidBodyAlgorithm(*model, Q, H,true);
	std::cout << "H_mat: \n" << H << std::endl;

	// C_tq : Coriolis, Centrifugal torque
VectorNd C_tq = VectorNd::Zero (model->q_size);
	NonlinearEffects(*model, Q, QDot, C_tq);
	std::cout << "C_tq: " << C_tq.transpose() << std::endl;

	// ID_QDDot = M^-1 * (Tau)
VectorNd ID_QDDot = VectorNd::Zero (model->q_size);
	CalcMInvTimesTau (*model, Q, Tau, ID_QDDot, true);
	std::cout << "   QDDot: " << QDDot.transpose() << std::endl;
	std::cout << "ID_QDDot: " << ID_QDDot.transpose() << std::endl;




	delete model;

 	return 0;
}

