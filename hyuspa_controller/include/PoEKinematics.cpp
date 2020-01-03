/*
 * Kinematics.cpp
 *
 *  Created on: Aug 7, 2018
 *      Author: spec
 */

#include "PoEKinematics.h"

namespace HYUMotionKinematics {

PoEKinematics::PoEKinematics()
{
	isInfoUpdated = 0;
	RobotDoF = 6;
}

PoEKinematics::PoEKinematics(int DoF)
:isInfoUpdated(0)
{
	RobotDoF = DoF;
}

PoEKinematics::~PoEKinematics() {
}
// LEFT //
void PoEKinematics::UpdateKinematicInfo(Vector3d _w, Vector3d _p, Vector3d _l, int _link_num)
{
	M[0] << 1, 0, 0, 163e-3,
			0, 1, 0, 0,
			0, 0, 1, 0,
			0, 0, 0, 1;

	_M[_link_num - 1][_link_num] = this->GetM(_l);
	v_se3[_link_num - 1] = this->GetTwist(_w, this->GetV(_w, _p));

	if(_link_num == 1){
		M[_link_num] = _M[_link_num-1][_link_num];
	}
	else{
		M[_link_num] = M[_link_num-1]*_M[_link_num-1][_link_num];
	}

	A[_link_num] = AdjointMatrix(inverse_SE3(M[_link_num]))*v_se3[_link_num-1];

	isInfoUpdated = 1;
}

// RIGHT //
void PoEKinematics::UpdateKinematicInfo_R(Vector3d _w, Vector3d _p, Vector3d _l, int _link_num) //_l : length of link
{
	M[0].setIdentity();
	v_se3[0].setZero(); Exp_S[0].setZero();


	M[_link_num] = this->GetM(_l);
	v_se3[_link_num] = this->GetTwist(_w, this->GetV(_w, _p));  //twist

	_M[_link_num - 1][_link_num] = M[_link_num - 1].inverse()*M[_link_num]; //MR p.291
	_M[_link_num][_link_num - 1] = M[_link_num].inverse()*M[_link_num - 1];

	A[_link_num] = AdjointMatrix(inverse_SE3(M[_link_num]))*v_se3[_link_num];

	isInfoUpdated = 1;
}

Vector3d PoEKinematics::GetV(Vector3d _w, Vector3d _p)
{
	return (-SkewMatrix(_w))*_p;    //MR p.139
}

SE3 PoEKinematics::GetM(Vector3d _link)
{
	SE3 res;
	res.setIdentity();
	res.block<3, 1>(0, 3) = _link;
	return res;
}

se3 PoEKinematics::GetTwist(Vector3d _w, Vector3d _v)
{
	se3 twist;

	twist.head(3) = _w;
	twist.tail(3) = _v;

	return twist;
}

void PoEKinematics::HTransMatrix(double q[])
{

	for (int end = 1; end <= ROBOT_DOF; ++end){
		Exp_S[end] = SE3Matrix(v_se3[end], q[end-1]);

	}
	/*
	T[0][1] = M[0]*Exp_S[0]*M[1];
	T[0][2] = M[0]*Exp_S[0]*Exp_S[1]*M[2];
	T[0][3] = M[0]*Exp_S[0]*Exp_S[1]*Exp_S[2]*M[3];
	T[0][4] = M[0]*Exp_S[0]*Exp_S[1]*Exp_S[2]*Exp_S[3]*M[4];
	T[0][5] = M[0]*Exp_S[0]*Exp_S[1]*Exp_S[2]*Exp_S[3]*Exp_S[4]*M[5];
	T[0][6] = M[0]*Exp_S[0]*Exp_S[1]*Exp_S[2]*Exp_S[3]*Exp_S[4]*Exp_S[5]*M[6];
	*/
	T[0][1] = Exp_S[1]*M[1];
	T[0][2] = Exp_S[1]*Exp_S[2]*M[2];
	T[0][3] = Exp_S[1]*Exp_S[2]*Exp_S[3]*M[3];
	T[0][4] = Exp_S[1]*Exp_S[2]*Exp_S[3]*Exp_S[4]*M[4];
	T[0][5] = Exp_S[1]*Exp_S[2]*Exp_S[3]*Exp_S[4]*Exp_S[5]*M[5];

//	T[0][6] = Exp_S[0]*Exp_S[1]*Exp_S[2]*Exp_S[3]*Exp_S[4]*Exp_S[5]*M[6];



	for(int i=1; i<ROBOT_DOF; ++i){              //MR P.92
		for(int j=i+1; j<=ROBOT_DOF;++j)
			T[i][j] = inverse_SE3(T[0][i])*T[0][j];
	}
	isInfoUpdated = 1;
}
se3 PoEKinematics::Twistout(int i)
{
	se3 res;
	res = v_se3[i];
	return res;
}

Jaco PoEKinematics::SpaceJacobian(void)
{
	Jacobian.setZero();

	Jacobian.block<6,1>(0,0) = v_se3[1];

	Jacobian.block<6,1>(0,1) = AdjointMatrix(Exp_S[1])*v_se3[2];

	Jacobian.block<6,1>(0,2) = AdjointMatrix(Exp_S[1]*Exp_S[2])*v_se3[3];

	Jacobian.block<6,1>(0,3) = AdjointMatrix(Exp_S[1]*Exp_S[2]*Exp_S[3])*v_se3[4];

	Jacobian.block<6,1>(0,4) = AdjointMatrix(Exp_S[1]*Exp_S[2]*Exp_S[3]*Exp_S[4])*v_se3[5];

	//Jacobian.block<ROBOT_DOF,1>(0,5) = AdjointMatrix(Exp_S[0]*Exp_S[1]*Exp_S[2]*Exp_S[3]*Exp_S[4])*v_se3[5];

	return Jacobian;
}

Jaco PoEKinematics::BodyJacobian(void)
{
	_SpaceJacobian = SpaceJacobian();
	_BodyJacobian = AdjointMatrix(inverse_SE3(T[0][ROBOT_DOF]))*_SpaceJacobian;
	return _BodyJacobian;
}

Matrix<double, 3, 6> PoEKinematics::AnalyticJacobian(void)
{
	_AnalyticJacobian = T[0][ROBOT_DOF].block<3,3>(0,0)*BodyJacobian().block<3,6>(3,0);
	return _AnalyticJacobian;
}

InvJaco PoEKinematics::Pinv(Jaco _j)
{
	InvJaco PJ;
	Jaco J; J << _j;

	//moore-penrose inverse
	int m = 0, n = 0;
	m = J.rows(); n = J.cols();

	if (n > m)//Fat
	{
		PJ = J.transpose()*((J*J.transpose()).inverse());
	}
	else if (m > n)//tall
	{
		PJ = ((J.transpose()*J).inverse())*J.transpose();
	}
	else if (m = n)//Square
	{
		PJ << J.inverse();
	}

	//QR-Decomposition
/*	CompleteOrthogonalDecomposition<MatrixXf> cod(J);
	cod.setThreshold(1e-5);
	PJ = cod.pseudoInverse();*/

	//SVD


	return PJ;
}
Matrix<double,6,3> PoEKinematics::Pinv(Matrix<double,3,6> _j)
{
	Matrix<double,6,3> PJ;
	Matrix<double,3,6> J; J << _j;
	int m = 0, n = 0;
	m = J.rows(); n = J.cols();

	if (n > m)//Fat
	{
		PJ = J.transpose()*((J*J.transpose()).inverse());
	}
	else if (m > n)//tall
	{
		PJ = ((J.transpose()*J).inverse())*J.transpose();
	}
	else if (m = n)//Square
	{
		PJ << J.inverse();
	}
	return PJ;
}
Vector3d PoEKinematics::ForwardKinematics(void)
{
	if(isInfoUpdated)
		RotMat = T[0][ROBOT_DOF].block<3,3>(0,0);
		return T[0][ROBOT_DOF].block<3,1>(0,3);
}

Vector3d PoEKinematics::GetEulerAngle(void)
{
	Vector3d rpy;

	rpy(0) = atan2(RotMat(2,1),RotMat(2,2));
	rpy(1) = -asin(RotMat(2,0));
	rpy(2) = atan2(RotMat(1, 0) / cos(rpy(1)), RotMat(0, 0) / cos(rpy(1)));

	if(isInfoUpdated)
		return rpy;
}

Matrix3d PoEKinematics::Rot(void)
{
	if(isInfoUpdated)
		return T[0][ROBOT_DOF].block<3,3>(0,0);
}

void PoEKinematics::Unflag_isInfoupdate(){
	if(isInfoUpdated)
		isInfoUpdated = 0;
}

} /* namespace HYUSpesA */
