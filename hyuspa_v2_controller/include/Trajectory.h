/*
 * Trajectory.h
 *
 *  Created on: Nov 20, 2018
 *      Author: spec
 */

#ifndef CONTROL_TRAJECTORY_H_
#define CONTROL_TRAJECTORY_H_

#include <Eigen/Dense>
#include <iostream>
#include <cmath>
#include "SerialRobot.h"
#include "LieOperator.h"

namespace hyuCtrl {



class Trajectory : public HYUMotionBase::LieOperator {
public:
	Trajectory();
	virtual ~Trajectory();
	bool isReady(){return m_isReady;};

	int isReady(int NumJoint){return m_isReady[NumJoint];};
	void SetPolynomial5th(int NumJoint, double startPos, double FinalPos, double InitTime, double Duration);
	void SetPolynomial5th(int NumJoint, state *act, double FinalPos, double InitTime, double Duration, double *q_);

	void SetPolynomial5th(int NumJoint, double startPos, double FinalPos, double InitTime, double Duration, double *q_);

    double Polynomial5th(int NumJoint, double CurrentTime, int *Flag);
    double Polynomial5th(int NumJoint, double CurrentTime, int *Flag, double *q_);

    void SetTrans5th(const VectorXd& startPos, VectorXd& FinalPos, double InitTime, double Duration, VectorXd& x_);
    void Trans5th(const VectorXd& startPos, VectorXd& FinalPos, double Duration, double CurrentTime, int * Flag, VectorXd& x_);
private:
	int m_isReady[6];

	Eigen::Matrix<double, 6, 6> m_cof;
    MatrixXd Xres, Xst, Xfin;
    VectorXd vst,vfin;
    double TrajDuration[16];
    double TrajTime[16];
    double TrajInitTime[16];
	Eigen::Matrix<double, 6, 1> StateVec[16], Coefficient[16];

    double dq, dq_dot, dq_ddot, s_;
};

} /* namespace HYUDA */

#endif /* CONTROL_TRAJECTORY_H_ */
