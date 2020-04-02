/*
 * robot.h
 *
 *  Created on: Oct 23, 2017
 *      Author: root
 */

#ifndef SERIALROBOT_H_
#define SERIALROBOT_H_

#include <cmath>

#include "LieDynamics.h"
#include "PropertyDefinition.h"

#define LEFT_ARM 0
#define RIGHT_ARM 1
#define MAX_CURRENT_ELMO 1200

typedef Matrix<double, ROBOT_DOF, 1> Jointd;
typedef Matrix<double, 6, 1> Taskd;
typedef Matrix<double, 3, 1> Carteciand;
typedef Matrix<double, 3, 1> Orientationd;
typedef Matrix<double, 3, 1> Axisd;
typedef Matrix<Vector3d, 6, 1> Axisv;
typedef Matrix<Matrix3d, 6, 1> Axism;

typedef struct MOTOR_INFO{
    double toq_const[ROBOT_DOF];
    double gear_ratio[ROBOT_DOF];
    double rate_current[ROBOT_DOF];
}MotorInfo;

#define RADtoDEG 180/M_PI
#define DEGtoRAD M_PI/180

#define sign(a) (((a)<0) ? -1 : ((a)>0))

typedef struct STATE{
    double q[ROBOT_DOF];
    double q_dot[ROBOT_DOF];
    double q_ddot[ROBOT_DOF];
    double torque[ROBOT_DOF];

	Jointd j_q;
	Jointd j_q_d;
	Jointd j_q_dd;
	Jointd j_torque;

	Taskd x;                           //Task space
	Taskd x_dot;
	Taskd x_ddot;
    double s_time;
}state;

class robot : public HYUMotionDynamics::Liedynamics
{
public:
	robot();
	virtual ~robot();

protected:
    Vector3d w[6], p[6], L[6], CoM[6];
    Matrix3d inertia[6];
    double mass[6];

public:

    void robot_update_R(void);
	void motor_update_R(MOTOR_INFO *motor);
	void ENCtoRAD_R(int *enc, Jointd& q);
	void ENCtoRAD_R(int *enc, Jointd& q, Jointd& q_dot, double s_time);
	void ELMO_OUTPUT_R(MOTOR_INFO *motor, Jointd& torque, short *output);

	HYUMotionDynamics::Liedynamics *pDyn;
	HYUMotionKinematics::PoEKinematics *pCoM;
	HYUMotionKinematics::PoEKinematics *pKin;


private:
	Jointd q_p;
};

#endif /* SERIALROBOT_H_ */
