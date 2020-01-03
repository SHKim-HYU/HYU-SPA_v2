/*
 * Controller.h
 *
 *  Created on: 2019. 5. 15.
 *      Author: Administrator
 */

#ifndef CONTROLLER_H_
#define CONTROLLER_H_

#include <vector>
#include <cmath>
#include <Eigen/Dense>
#include "PropertyDefinition.h"
#include "SerialRobot.h"

#define KpBase 10
#define KdBase 0.01
#define KiBase 30


/**
 * @brief
 * @details
 * @author Junho Park
 * @date   20119-05-16
 * @version 1.0.0
 *
 */

namespace HYUControl {

class Controller : public robot {
public:
	Controller();
	Controller(int JointNum);
	virtual ~Controller(){};

	//void ClearError(void);

	void SetPIDGain(double _Kp, double _Kd, double _Ki, int _JointNum);
	void PDController_gravity(double *q, double *q_dot, double *dq, double *dq_dot, double *toq, Jointd &g_mat);
	void PDController(Jointd &q, Jointd &q_dot, double *dq, double *dq_dot, double *toq);
//	void Impedance(float *q, float *q_dot, float *q_ddot, float *dq, float *dq_dot, float *dq_ddot, float *toq, Matrixf &m_mat, Matrixf &c_mat, Jointf &g_mat);
	void Impedance(Jointd &q, Jointd &q_dot, Jointd &q_ddot, double *dq, double *dq_dot, double *dq_ddot, double *toq, Matrixd &m_mat, Jointd &g_mat);
	void Impedance(Jointd &q, Jointd &q_dot, Jointd &q_ddot, double *dq, double *dq_dot, double *dq_ddot, double *toq, Matrixd &m_mat, Jointd &g_mat, Matrixd &c_mat);
	void Inverse_Dynamics_Control(Jointd &q, Jointd &q_dot, Jointd &q_ddot, double *dq, double *dq_dot, double *dq_ddot, double *toq, Matrixd &m_mat, Jointd &g_mat, Matrixd &c_mat);


	void TorqueOutput(double *p_toq, int maxtoq, int *p_dir);
	//void TorqueOutput(double *p_toq , int maxtoq);
	Jointd return_u0(void);

	//void IKAccel(state *des, state *act);

	int int_flag=0;

private:
	Matrix<double,ROBOT_DOF,1> Kp;
	Matrix<double,ROBOT_DOF,1> Kd;
	Matrix<double,ROBOT_DOF,1> Ki;

	Matrix<double,ROBOT_DOF,1> Damp;
	Matrix<double,ROBOT_DOF,1> Stiff;

	Matrix<double,ROBOT_DOF,1> e;
	Matrix<double,ROBOT_DOF,1> e_dev;
	Matrix<double,ROBOT_DOF,1> e_int;
	Matrix<double,ROBOT_DOF,1> e_old;
    VectorXd eTask, edotTask;
    VectorXd KpTask, KdTask;
    VectorXd KiTask;

    Jointd u0;
	int m_Jnum;

    double m_KpBase, m_KdBase, m_KiBase;
};

} /* namespace HYUCtrl */

#endif /* CONTROLLER_H_ */
