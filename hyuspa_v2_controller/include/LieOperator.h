/*
 * LieOperator.h
 *
 *  Created on: Aug 13, 2018
 *      Author: spec
 */

#ifndef LIEOPERATOR_H_
#define LIEOPERATOR_H_

#include <Eigen/Dense>
#include <vector>
using namespace Eigen;
using namespace std;

typedef Matrix<double, 6, 1> se3;
typedef Matrix<double, 4, 4> SE3;
typedef Matrix<double, 6, 6> Adjoint;
typedef Matrix<double, 6, 6> adjoint;

namespace HYUMotionBase {

class LieOperator {
public:
	LieOperator();
	virtual ~LieOperator();

public:
	bool NearZero(double near);
	SE3 inverse_SE3( SE3 _SE3 );
	Matrix3d SkewMatrix( Vector3d _Vec3 );
	Matrix3d SkewMatrixSquare( Vector3d _Vec3 );
	Adjoint AdjointMatrix( SE3 _SE3 );
	Adjoint AdjointDualMatrix( SE3 _SE3 );
	adjoint adjointMatrix( se3 _se3 );
	adjoint adjointDualMatrix( se3 _se3 );
	Matrix3d MatrixLog3(const Matrix3d& R);
    MatrixXd MatrixLog6(const MatrixXd& T);
	Vector3d so3ToVec(const Matrix3d& so3mat);

	SE3 SE3Matrix(se3 _Twist, double _q);
    MatrixXd Normalize(Eigen::MatrixXd V);
    Vector4d AxisAng3(Vector3d& expc3);
    MatrixXd MatrixExp6(const MatrixXd& se3mat);
    Matrix3d MatrixExp3(Matrix3d& so3mat);
    vector<Eigen::MatrixXd> TransToRp(const MatrixXd& T);
    MatrixXd TransInv(const MatrixXd& transform);

//	Matrix<double, 7, 1> AxisAng6(se3 expc6);
//	Matrix4d MatrixExp6(se3 _s, double _q);
private:


};

} /* namespace HYUMotion */

#endif /* LIEOPERATOR_H_ */
