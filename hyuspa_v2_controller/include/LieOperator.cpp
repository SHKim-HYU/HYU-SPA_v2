/*
 * LieOperator.cpp
 *
 *  Created on: Aug 13, 2018
 *      Author: spec
 */

#include "LieOperator.h"

namespace HYUMotionBase {

LieOperator::LieOperator() {

}

LieOperator::~LieOperator() {

}

bool LieOperator::NearZero(double near){
		if (near < 0.000001)
			return true;
		else
			return false;
}

SE3 LieOperator::inverse_SE3( SE3 _SE3 )
{
	SE3 res;
	res << _SE3.block<3, 3>(0, 0).transpose(), -_SE3.block<3, 3>(0, 0).transpose()*_SE3.block<3, 1>(0, 3),
		0, 0, 0, 1;
	return res;
}

Matrix3d LieOperator::SkewMatrix(Vector3d _Vec3)
{
	Matrix3d res;
	res << 	0, 			-_Vec3(2), 	_Vec3(1),
			_Vec3(2), 	0, 			-_Vec3(0),
			-_Vec3(1), 	_Vec3(0), 	0;
	return res;
}

Matrix3d LieOperator::SkewMatrixSquare(Vector3d _Vec3)
{
	Matrix3d res;
	res << -_Vec3(2)*_Vec3(2)-_Vec3(1)*_Vec3(1), 	_Vec3(0)*_Vec3(1), 						_Vec3(0)*_Vec3(2),
			_Vec3(0)*_Vec3(1), 						-_Vec3(0)*_Vec3(0)-_Vec3(2)*_Vec3(2), 	_Vec3(1)*_Vec3(2),
			_Vec3(0)*_Vec3(2), 						_Vec3(1)*_Vec3(2), 						-_Vec3(0)*_Vec3(0)-_Vec3(1)*_Vec3(1);
	return res;
}

Adjoint LieOperator::AdjointMatrix( SE3 _SE3 )
{
	Adjoint res;
	res.setZero();

	res.block<3,3>(0,0) = _SE3.block(0, 0, 3, 3);
	res.block<3,3>(3,3) = _SE3.block(0, 0, 3, 3);
	res.block<3,3>(3,0) = SkewMatrix(_SE3.block(0, 3, 3, 1))*_SE3.block(0, 0, 3, 3);

	return res;
}

Adjoint LieOperator::AdjointDualMatrix(SE3 _SE3)
{
	Adjoint res;
	res.setZero();

	res.block(0, 0, 3, 3) = _SE3.block(0, 0, 3, 3).transpose();
	res.block(0, 3, 3, 3) = _SE3.block(0, 0, 3, 3).transpose()*SkewMatrix(_SE3.block(0, 3, 3, 1)).transpose();
	res.block(3, 3, 3, 3) = _SE3.block(0, 0, 3, 3).transpose();
	return res;
}

adjoint LieOperator::adjointMatrix(se3 _se3)
{
	adjoint res;
	res.setZero();

	res.block(0, 0, 3, 3) = SkewMatrix(_se3.head(3));
	res.block(3, 0, 3, 3) = SkewMatrix(_se3.tail(3));
	res.block(3, 3, 3, 3) = SkewMatrix(_se3.head(3));
	return res;
}

adjoint LieOperator::adjointDualMatrix(se3 _se3)
{
	adjoint res;
	res.setZero();

	res.block(0, 0, 3, 3) = -SkewMatrix(_se3.head(3));
	res.block(0, 3, 3, 3) = -SkewMatrix(_se3.tail(3));
	res.block(3, 3, 3, 3) = -SkewMatrix(_se3.head(3));

	return res;
}

SE3 LieOperator::SE3Matrix(se3 _Twist, double _q)
{
	Matrix4d res = Matrix4d::Identity();
	Matrix3d i = Matrix3d::Identity();

	//Rodrigues' formula
	res.block(0, 0, 3, 3) = i + sinf(_q)*SkewMatrix(_Twist.head(3)) + (1 - cosf(_q))*SkewMatrixSquare(_Twist.head(3));
	res.block(0, 3, 3, 1) = (i*_q + (1 - cosf(_q))*SkewMatrix(_Twist.head(3))
			+ (_q - sinf(_q))*SkewMatrixSquare(_Twist.head(3)))*_Twist.tail(3);

	return res;
}
Matrix3d LieOperator::MatrixLog3(const Matrix3d & R)
{
    double acosinput = (R.trace() - 1) / 2.0;
    MatrixXd m_ret = MatrixXd::Zero(3, 3);
    if (acosinput >= 1)
        return m_ret;
    else if (acosinput <= -1) {
        Vector3d omg;
        if (!NearZero(1 + R(2, 2)))
            omg = (1.0 / std::sqrt(2 * (1 + R(2, 2))))*Eigen::Vector3d(R(0, 2), R(1, 2), 1 + R(2, 2));
        else if (!NearZero(1 + R(1, 1)))
            omg = (1.0 / std::sqrt(2 * (1 + R(1, 1))))*Eigen::Vector3d(R(0, 1), 1 + R(1, 1), R(2, 1));
        else
            omg = (1.0 / std::sqrt(2 * (1 + R(0, 0))))*Eigen::Vector3d(1 + R(0, 0), R(1, 0), R(2, 0));
        m_ret = SkewMatrix(M_PI * omg);
        return m_ret;
    }
    else {
        double theta = std::acos(acosinput);
        m_ret = theta / 2.0 / sin(theta)*(R - R.transpose());
        return m_ret;
    }
}
MatrixXd LieOperator::MatrixLog6(const MatrixXd& T) {
    MatrixXd m_ret(4, 4);
    auto rp = TransToRp(T);
    Matrix3d omgmat = MatrixLog3(rp.at(0));
    Matrix3d zeros3d = Eigen::Matrix3d::Zero(3, 3);
    if (NearZero(omgmat.norm())) {
        m_ret << zeros3d, rp.at(1),
                0, 0, 0, 0;
    }
    else {
        double theta = std::acos((rp.at(0).trace() - 1) / 2.0);
        Eigen::Matrix3d logExpand1 = Eigen::MatrixXd::Identity(3, 3) - omgmat / 2.0;
        Eigen::Matrix3d logExpand2 = (1.0 / theta - 1.0 / std::tan(theta / 2.0) / 2)*omgmat*omgmat / theta;
        Eigen::Matrix3d logExpand = logExpand1 + logExpand2;
        m_ret << omgmat, logExpand*rp.at(1),
                0, 0, 0, 0;
    }
    return m_ret;
}
Vector3d LieOperator::so3ToVec(const Matrix3d& so3mat) {
    Eigen::Vector3d v_ret;
    v_ret << so3mat(2, 1), so3mat(0, 2), so3mat(1, 0);
    return v_ret;
}
/*
Matrix<double, 7, 1> LieOperator::AxisAng6(se3 expc6) {
	Matrix<double, 7, 1> S_theta;
	Vector3f omg_theta,vec_theta;
	double theta;

	omg_theta << expc6(0,0), expc6(1,0), expc6(2,0);
	vec_theta << expc6(3, 0), expc6(4, 0), expc6(5, 0);

	theta = omg_theta.norm();

	if (NearZero(theta)) {
		theta = vec_theta.norm();
		S_theta << expc6, theta;
		return S_theta;
	}
	else {
		S_theta << expc6 / theta, theta;
		return S_theta;
	}
}
 */
MatrixXd LieOperator::Normalize(Eigen::MatrixXd V) {
    V.normalize();
    return V;
}

Vector4d LieOperator::AxisAng3(Vector3d& expc3) {
    Eigen::Vector4d v_ret;
    v_ret << Normalize(expc3), expc3.norm();
    return v_ret;
}
MatrixXd LieOperator::MatrixExp6(const MatrixXd& se3mat) {
    Matrix3d se3mat_cut = se3mat.block<3, 3>(0, 0);
    Vector3d omgtheta = so3ToVec(se3mat_cut);

    MatrixXd m_ret(4, 4);

    // If negligible rotation, m_Ret = [[Identity, angular velocty ]]
    //									[	0	 ,		1		   ]]
    if (NearZero(omgtheta.norm())) {
        // Reuse previous variables that have our required size
        se3mat_cut = Eigen::MatrixXd::Identity(3, 3);
        omgtheta << se3mat(0, 3), se3mat(1, 3), se3mat(2, 3);
        m_ret << se3mat_cut, omgtheta,
                0, 0, 0, 1;
        return m_ret;
    }
        // If not negligible, MR page 105
    else {
        double theta = (AxisAng3(omgtheta))(3);
        Eigen::Matrix3d omgmat = se3mat.block<3, 3>(0, 0) / theta;
        Eigen::Matrix3d expExpand = Eigen::MatrixXd::Identity(3, 3) * theta + (1 - std::cos(theta)) * omgmat + ((theta - std::sin(theta)) * (omgmat * omgmat));
        Eigen::Vector3d linear(se3mat(0, 3), se3mat(1, 3), se3mat(2, 3));
        Eigen::Vector3d GThetaV = (expExpand*linear) / theta;
        m_ret << MatrixExp3(se3mat_cut), GThetaV,
                0, 0, 0, 1;
        return m_ret;
    }
}
Matrix3d LieOperator::MatrixExp3(Matrix3d& so3mat) {
    Eigen::Vector3d omgtheta = so3ToVec(so3mat);

    Eigen::Matrix3d m_ret = Eigen::Matrix3d::Identity();
    if (NearZero(so3mat.norm())) {
        return m_ret;
    }
    else {
        double theta = (AxisAng3(omgtheta))(3);
        Eigen::Matrix3d omgmat = so3mat * (1 / theta);
        return m_ret + std::sin(theta) * omgmat + ((1 - std::cos(theta)) * (omgmat * omgmat));
    }
}
vector<Eigen::MatrixXd> LieOperator::TransToRp(const MatrixXd& T) {
    vector<Eigen::MatrixXd> Rp_ret;
    Eigen::Matrix3d R_ret;
    // Get top left 3x3 corner
    R_ret = T.block<3, 3>(0, 0);

    Eigen::Vector3d p_ret(T(0, 3), T(1, 3), T(2, 3));

    Rp_ret.push_back(R_ret);
    Rp_ret.push_back(p_ret);

    return Rp_ret;
}
MatrixXd LieOperator::TransInv(const MatrixXd& transform) {
    auto rp = TransToRp(transform);
    auto Rt = rp.at(0).transpose();
    auto t = -(Rt * rp.at(1));
    Eigen::MatrixXd inv(4, 4);
    inv = Eigen::MatrixXd::Zero(4,4);
    inv.block(0, 0, 3, 3) = Rt;
    inv.block(0, 3, 3, 1) = t;
    inv(3, 3) = 1;
    return inv;
}
} /* namespace HYUMotion */
