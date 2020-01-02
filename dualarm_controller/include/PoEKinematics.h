/*
 * Kinematics.h
 *
 *  Created on: Aug 7, 2018
 *      Author: spec
 */

#ifndef POEKINEMATICS_H_
#define POEKINEMATICS_H_

#include <cmath>
#include <Eigen/Dense>

#include "LieOperator.h"
#include "PropertyDefinition.h"
#include "iostream"

using namespace Eigen;

typedef Matrix<double, ROBOT_DOF, 1> Jointd;
typedef Matrix<double, 6, ROBOT_DOF> Jaco;
typedef Matrix<double,ROBOT_DOF,6> InvJaco;

namespace HYUMotionKinematics {

class PoEKinematics : public HYUMotionBase::LieOperator {
public:
	PoEKinematics();
	PoEKinematics(int DoF);
	virtual ~PoEKinematics();

public:
	void UpdateKinematicInfo(Vector3d _w, Vector3d _p, Vector3d _l, int _link_num);
	void UpdateKinematicInfo_R(Vector3d _w, Vector3d _p, Vector3d _l, int _link_num);
	Vector3d GetV(Vector3d _w, Vector3d _p);
	SE3 GetM(Vector3d _link);
	se3 GetTwist(Vector3d _w, Vector3d _v);
	void HTransMatrix(double q[]);
	se3 Twistout(int i);

	Jaco SpaceJacobian(void);
	Jaco BodyJacobian(void);
	Matrix<double, 3, 6> AnalyticJacobian(void);

	InvJaco Pinv(Jaco _j);
	Matrix<double,6,3> Pinv(Matrix<double,3,6> _j);

	Vector3d ForwardKinematics(void);
	Vector3d GetEulerAngle(void);
	Matrix3d Rot(void);
//	Vector4d InverseKinematics(Matrix4d _td, )
	void Unflag_isInfoupdate();

private:
	int RobotDoF;
	Jaco Jacobian;
	Jaco _SpaceJacobian;
	Jaco _BodyJacobian;
	Matrix<double, 3, 6> _AnalyticJacobian;



protected:
	SE3 T[ROBOT_DOF+1][ROBOT_DOF+1];
	SE3 M[ROBOT_DOF+1];
	SE3 _M[ROBOT_DOF+1][ROBOT_DOF+1];
	SE3 Exp_S[ROBOT_DOF+1];
	Matrix3d RotMat;

	se3 A[ROBOT_DOF+1];
	se3 v_se3[ROBOT_DOF+1];

	int isInfoUpdated;
};

} /* namespace HYUSpesA */

#endif /* POEKINEMATICS_H_ */
