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
#include <math.h>
#include "SerialRobot.h"

namespace hyuCtrl {



class Trajectory {
public:
	Trajectory();
	virtual ~Trajectory();
	bool isReady(void){return m_isReady;};

	int isReady(int NumJoint){return m_isReady[NumJoint];};
	void SetPolynomial5th(int NumJoint, float startPos, float FinalPos, float InitTime, float Duration);
	void SetPolynomial5th(int NumJoint, state *act, float FinalPos, float InitTime, float Duration, float *q_);
	float Polynomial5th(int NumJoint, float CurrentTime, int *Flag);
	float Polynomial5th(int NumJoint, float CurrentTime, int *Flag, float *q_);

private:
	int m_isReady[6];

	Eigen::Matrix<float, 6, 6> m_cof;

	float TrajDuration[16];
	float TrajTime[16];
	float TrajInitTime[16];
	Eigen::Matrix<float, 6, 1> StateVec[16], Coefficient[16];
	float dq, dq_dot, dq_ddot;
};

} /* namespace HYUDA */

#endif /* CONTROL_TRAJECTORY_H_ */
