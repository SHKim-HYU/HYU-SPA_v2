/*
 * Controller.cpp
 *
 *  Created on: 2019. 5. 15.
 *      Author: Administrator
 */

#include "Controller.h"

namespace HYUControl {

Controller::Controller():m_Jnum(6)
{
	m_KpBase = KpBase;
	m_KdBase = KdBase;
	m_KiBase = KiBase;
	Kp.setZero();
	Kd.setZero();
	Ki.setZero();

	e.setZero();
	e_dev.setZero();
	e_int.setZero();

}

Controller::Controller(int JointNum)
{

    m_Jnum = JointNum;
    m_KpBase = KpBase;
    m_KdBase = KdBase;
    m_KiBase = KiBase;
    Kp.setZero();
    Kd.setZero();
    Ki.setZero();
    KpTask.resize(6);
    KdTask.resize(6);
    KiTask.resize(6);

    e.setZero();
    e_dev.setZero();
    e_int.setZero();
    e_old.setZero();
    eTask.resize(6);
    edotTask.resize(6);


}
Controller::Controller(robot *pManipulator, int JointNum)
{
    this->pManipulator=pManipulator;

    m_Jnum = JointNum;
    m_KpBase = KpBase;
    m_KdBase = KdBase;
    m_KiBase = KiBase;
    Kp.setZero();
    Kd.setZero();
    Ki.setZero();
    KpTask.resize(6);
    KdTask.resize(6);
    KiTask.resize(6);

    e.setZero();
    e_dev.setZero();
    e_int.setZero();
    e_old.setZero();
    eTask.resize(6);
    edotTask.resize(6);


}

void Controller::SetPIDGain(double _Kp, double _Kd, double _Ki, int _JointNum){
    Kp(_JointNum) = _Kp;
    Kd(_JointNum) = _Kd;
    Ki(_JointNum) = _Ki;
    return;
}

void Controller::PDController_gravity(double *q, double *q_dot, double *dq, double *dq_dot, double *toq, Jointd &g_mat)
{
	//Real
	Kp << 118.7931,		119.5279,	59.764,		160.2564,		19.1946;
	//Ki={860.8196,		866.1442,	4.1716,		427.5721,	135.3359};
	Kd << 4.0984,			4.1237,		2.0619,		2.5641,		0.691;
	//Kp={0.0,};
	//Kd={0.0,};
	//Kp << 5,		3,		3,		3,		3;
	//Kd << 0.05,		0.03,	0.03,	0.03,	0.03;


//		int i = 1; //Joint Number
	for(int i=0; i<m_Jnum; ++i)
	{

		e(i) = dq[i] - q[i];
		e_dev(i) = dq_dot[i] - q_dot[i];

/*		if(i==0)
			toq[i] = (g_mat(i))/(float)(TORQUE_CONST_1*MAX_CURRENT_1*HARMONIC_100)*1000*1.66;
		else if(i==1)
			toq[i] = (g_mat(i))/(float)(TORQUE_CONST_2*MAX_CURRENT_2*HARMONIC_100)*1000*1.66;
		else if(i==2)
			toq[i] = (g_mat(i))/(float)(TORQUE_CONST_3*MAX_CURRENT_3*HARMONIC_100)*1000*1.66;
		else if(i==3)
			toq[i] = (g_mat(i))/(float)(TORQUE_CONST_4*MAX_CURRENT_4*HARMONIC_100)*1000*1.66;
		else if(i==4)
			toq[i] = (g_mat(i))/(float)(TORQUE_CONST_5*MAX_CURRENT_5*HARMONIC_100)*1000*1.66;
*/
		if(i==0)
			toq[i] = (Kp(i)*e(i) + Kd(i)*e_dev(i)+g_mat(i))/(double)(TORQUE_CONST_1*MAX_CURRENT_1*HARMONIC_100)*1000*1.66;
		else if(i==1)
			toq[i] = (Kp(i)*e(i) + Kd(i)*e_dev(i)+g_mat(i))/(double)(TORQUE_CONST_2*MAX_CURRENT_2*HARMONIC_100)*1000*1.66;
		else if(i==2)
			toq[i] = (Kp(i)*e(i) + Kd(i)*e_dev(i)+g_mat(i))/(double)(TORQUE_CONST_3*MAX_CURRENT_3*HARMONIC_100)*1000*1.66;
		else if(i==3)
			toq[i] = (Kp(i)*e(i) + Kd(i)*e_dev(i)+g_mat(i))/(double)(TORQUE_CONST_4*MAX_CURRENT_4*HARMONIC_100)*1000*1.66;
		else if(i==4)
			toq[i] = (Kp(i)*e(i) + Kd(i)*e_dev(i)+g_mat(i))/(double)(TORQUE_CONST_5*MAX_CURRENT_5*HARMONIC_100)*1000*1.66;

		//For Simulation
/*		if(i==0)
			toq[i] = (Kp(i)*e(i) + Kd(i)*e_dev(i)+g_mat(i));
		else if(i==1)
			toq[i] = (Kp(i)*e(i) + Kd(i)*e_dev(i)+g_mat(i));
		else if(i==2)
			toq[i] = (Kp(i)*e(i) + Kd(i)*e_dev(i)+g_mat(i));
		else if(i==3)
			toq[i] = (Kp(i)*e(i) + Kd(i)*e_dev(i)+g_mat(i));
		else if(i==4)
			toq[i] = (Kp(i)*e(i) + Kd(i)*e_dev(i)+g_mat(i));
*/
			else
		/*	int_flag++;
			if(int_flag>10000000){
				int_flag=0;
				ClearError();
			}*/
			return;
	}

}

void Controller::PD_Gravity(double * q, double * q_dot, Matrix<double,5,1>& dq, Matrix<double,5,1>& dq_dot, double * toq)
{
    //Kp << 40,        40,	    30,      35,      15;
    Kp << 45,        45,	    35,      40,      15;
    //Ki={860.8196,		866.1442,	4.1716,		427.5721,	135.3359};
    //Kd << 0.23,	    0.23,   0.18,    0.16,   0.12;
    Kd << 0.03375482 * Kp(0), 0.034499895*Kp(1), 0.0345007*Kp(2), 0.015999985*Kp(3), 0.03599971*Kp(4);
    G.resize(ROBOT_DOF,1);
    G=pManipulator->pDyn->G_Matrix();

    for(int i=0; i<m_Jnum; ++i) {

        e(i) = dq[i] - q[i];
        e_dev(i) = dq_dot[i] - q_dot[i];

            //For Simulation
/*		if(i==0)
			toq[i] = G(i);
		else if(i==1)
			toq[i] = G(i);
		else if(i==2)
			toq[i] = G(i);
		else if(i==3)
			toq[i] = G(i);
		else if(i==4)
			toq[i] = G(i);*/
        //For Simulation
        if(i==0)
            toq[i] = (Kp(i)*e(i) + Kd(i)*e_dev(i)+G(i));
        else if(i==1)
            toq[i] = (Kp(i)*e(i) + Kd(i)*e_dev(i)+G(i));
        else if(i==2)
            toq[i] = (Kp(i)*e(i) + Kd(i)*e_dev(i)+G(i));
        else if(i==3)
            toq[i] = (Kp(i)*e(i) + Kd(i)*e_dev(i)+G(i));
        else if(i==4)
            toq[i] = (Kp(i)*e(i) + Kd(i)*e_dev(i)+G(i));
        else
            return;
    }

}

void Controller::Gravity(double * q, double * q_dot, double * toq)
{
    G.resize(ROBOT_DOF,1);
    G=pManipulator->pDyn->G_Matrix();

    for(int i=0; i<m_Jnum; ++i) {

        //For Simulation
		if(i==0)
			toq[i] = G(i);
		else if(i==1)
			toq[i] = G(i);
		else if(i==2)
			toq[i] = G(i);
		else if(i==3)
			toq[i] = G(i);
		else if(i==4)
			toq[i] = G(i);
        else
            return;
    }
}

    void Controller::Inverse_Dynamics_Control(double *q_, double *q_dot, Matrix<double,5,1>& dq_, Matrix<double,5,1>& dq_dot, Matrix<double,5,1>& dq_ddot, double * toq)
    {
        //Kp << 118.7931,		119.5279,	59.764,		160.2564,		39.1946;
        //Ki={860.8196,		866.1442,	4.1716,		427.5721,	135.3359};
        //Kd << 4.0984,			4.1237,		2.0619,		2.5641,		0.691;
        //Kp << 11.7931,		11.5279,	5.764,		16.2564,		3.1946;
        //Kd << 2*sqrt(Kp(0)), 2*sqrt(Kp(1)), 2*sqrt(Kp(2)), 2*sqrt(Kp(3)), 2*sqrt(Kp(4));
        Kp << 100,        100,	    75,      100,      60;
        Kd << 0.03375482 * Kp(0), 0.034499895*Kp(1), 0.0345007*Kp(2), 0.015999985*Kp(3), 0.03599971*Kp(4);

        G.resize(ROBOT_DOF,1);
        G=pManipulator->pDyn->G_Matrix();
        M.resize(ROBOT_DOF,ROBOT_DOF);
        M=pManipulator->pDyn->M_Matrix();
        C.resize(ROBOT_DOF,ROBOT_DOF);
        C=pManipulator->pDyn->C_Matrix();

        q = Map<VectorXd>(q_, this->m_Jnum);
        qdot = Map<VectorXd>(q_dot, this->m_Jnum);

        //dq = Map<VectorXd>(dq_, this->m_Jnum);
        //dqdot = Map<VectorXd>(dq_dot, this->m_Jnum);
        //dqddot = Map<VectorXd>(dq_ddot, this->m_Jnum);
        dq=dq_;
        dqdot=dq_dot;
        dqddot=dq_ddot;


        Jointd u, dq_dd;

        e = dq - q;
        e_dev= dqdot - qdot;

        u0=dqddot + Kd.cwiseProduct(e_dev) + Kp.cwiseProduct(e);
        u= M * u0 + C * qdot + G;


        for(int i=0; i<m_Jnum; ++i)
        {

            if(i==0){

                toq[i] = u(i);
            }
            else if(i==1){

                toq[i] = u(i);
            }
            else if(i==2){

                toq[i] = u(i);
            }
            else if(i==3){

                toq[i] = u(i);
            }
            else if(i==4){

                toq[i] = u(i);
            }
            else
                /*	int_flag++;
                    if(int_flag>10000000){
                        int_flag=0;
                        ClearError();
                    }*/
                return;
        }
    }
    void Controller::ComputedTorque(double *q_, double *q_dot, Matrix<double,5,1>& dq_, Matrix<double,5,1>& dq_dot, Matrix<double,5,1>& dq_ddot, double * toq)
    {
        Kp << 20,        20,	    10,      20,      10;
        Kd << 0.03375482 * Kp(0), 0.034499895*Kp(1), 0.0345007*Kp(2), 0.015999985*Kp(3), 0.03599971*Kp(4);

        G.resize(ROBOT_DOF,1);
        G=pManipulator->pDyn->G_Matrix();
        M.resize(ROBOT_DOF,ROBOT_DOF);
        M=pManipulator->pDyn->M_Matrix();
        C.resize(ROBOT_DOF,ROBOT_DOF);
        C=pManipulator->pDyn->C_Matrix();

        q = Map<VectorXd>(q_, this->m_Jnum);
        qdot = Map<VectorXd>(q_dot, this->m_Jnum);

        //dq = Map<VectorXd>(dq_, this->m_Jnum);
        //dqdot = Map<VectorXd>(dq_dot, this->m_Jnum);
        //dqddot = Map<VectorXd>(dq_ddot, this->m_Jnum);
        dq=dq_;
        dqdot=dq_dot;
        dqddot=dq_ddot;


        Jointd u, uff, dq_dd;

        e = dq - q;
        e_dev= dqdot - qdot;

        uff = M * dqddot + C * dqdot + G;
        u0=Kd.cwiseProduct(e_dev) + Kp.cwiseProduct(e);
        u= uff +u0;


        for(int i=0; i<m_Jnum; ++i)
        {

            if(i==0){

                toq[i] = u(i);
            }
            else if(i==1){

                toq[i] = u(i);
            }
            else if(i==2){

                toq[i] = u(i);
            }
            else if(i==3){

                toq[i] = u(i);
            }
            else if(i==4){

                toq[i] = u(i);
            }
            else
                /*	int_flag++;
                    if(int_flag>10000000){
                        int_flag=0;
                        ClearError();
                    }*/
                return;
        }
    }
void Controller::PDController(Jointd &q, Jointd &q_dot, double *dq, double *dq_dot, double *toq)
{
	Kp << 118.7931,		119.5279,	59.764,		160.2564,		19.1946;
	//Ki={860.8196,		866.1442,	4.1716,		427.5721,	135.3359};
	Kd << 4.0984,			4.1237,		2.0619,		2.5641,		0.691;

	int i = 1;
//	for(int i=0; i<m_Jnum; ++i)
//	{

		e(i) = dq[i] - q(i);
		e_dev(i) = dq_dot[i] - q_dot(i);

	//	e_int.at(i) = e.at(i)+e_old.at(i);
	//	e_old.at(i)=e.at(i);
		if(i==0)
			toq[i] = (Kp(i)*e(i) + Kd(i)*e_dev(i))/(double)(TORQUE_CONST_1*MAX_CURRENT_1*HARMONIC_100)*1000;
		else if(i==1)
			toq[i] = (Kp(i)*e(i) + Kd(i)*e_dev(i))/(double)(TORQUE_CONST_2*MAX_CURRENT_2*HARMONIC_100)*1000;
		else if(i==2)
			toq[i] = (Kp(i)*e(i) + Kd(i)*e_dev(i))/(float)(TORQUE_CONST_3*MAX_CURRENT_3*HARMONIC_100)*1000;
		else if(i==3)
			toq[i] = (Kp(i)*e(i) + Kd(i)*e_dev(i))/(float)(TORQUE_CONST_4*MAX_CURRENT_4*HARMONIC_100)*1000;
		else if(i==4)
			toq[i] = (Kp(i)*e(i) + Kd(i)*e_dev(i))/(float)(TORQUE_CONST_5*MAX_CURRENT_5*HARMONIC_100)*1000;
		else
		/*	int_flag++;
			if(int_flag>10000000){
				int_flag=0;
				ClearError();
			}*/
			return;
	/*	if(i==0)
			toq[i] = (Kp.at(i)*e.at(i) + Kd.at(i)*e_dev.at(i) + Ki.at(i)*e_int.at(i))/(float)(TORQUE_CONST_1*MAX_CURRENT_1*HARMONIC_100)*1000;
		else if(i==1)
			toq[i] = (Kp.at(i)*e.at(i) + Kd.at(i)*e_dev.at(i) + Ki.at(i)*e_int.at(i))/(float)(TORQUE_CONST_2*MAX_CURRENT_2*HARMONIC_100)*1000;
		else if(i==2)
			toq[i] = (Kp.at(i)*e.at(i) + Kd.at(i)*e_dev.at(i) + Ki.at(i)*e_int.at(i))/(float)(TORQUE_CONST_3*MAX_CURRENT_3*HARMONIC_100)*1000;
		else if(i==3)
			toq[i] = (Kp.at(i)*e.at(i) + Kd.at(i)*e_dev.at(i) + Ki.at(i)*e_int.at(i))/(float)(TORQUE_CONST_4*MAX_CURRENT_4*HARMONIC_100)*1000;
		else if(i==4)
			toq[i] = (Kp.at(i)*e.at(i) + Kd.at(i)*e_dev.at(i) + Ki.at(i)*e_int.at(i))/(float)(TORQUE_CONST_5*MAX_CURRENT_5*HARMONIC_100)*1000;
		else
		/*	int_flag++;
			if(int_flag>10000000){
				int_flag=0;
				ClearError();
			}
			return;*/
//	}


}
/*
void Controller::Impedance(double *_q_dot, Matrix<double, 6, 1> & _x,Matrix<double, 6, 1> & _x_dot, Matrix<double, 6, 1> & _dx, Matrix<double, 6, 1> & _dx_dot, Matrix<double, 6, 1> & _dx_ddot, double * toq)
{
    M=pManipulator->pDyn->M_Matrix();
    C=pManipulator->pDyn->C_Matrix();
    G=pManipulator->pDyn->G_Matrix();
    qdot = Map<VectorXd>(_q_dot, this->m_Jnum);

    _a_jaco.resize(6,ROBOT_DOF);
    _a_jaco=pManipulator->pKin->AnalyticJacobian();
    _pinv_jaco.resize(ROBOT_DOF,6);
    _pinv_jaco=pManipulator->pKin->DPI_l(_a_jaco);
    _jaco_dot.resize(6,ROBOT_DOF);
    _jaco_dot=pManipulator->pKin->Jacobian_dot();

    Matrix<double,6,6> Md,Kd,Bd;
    Md = _pinv_jaco.transpose()*M*_pinv_jaco;
    Kd=100*Kd.setIdentity();
    Bd=0.7*Bd.setIdentity();

    ax.resize(6,1);
    ax=_dx_ddot-Md.inverse()*(Bd*(_x_dot-_dx_dot)+Kd*(_x-_dx));

    u0=M*_pinv_jaco*(ax-_jaco_dot*qdot)+C*qdot+G;
    for(int i=0; i<m_Jnum; ++i) {

        if (i == 0) {

            toq[i] = u0(i);
        } else if (i == 1) {

            toq[i] = u0(i);
        } else if (i == 2) {

            toq[i] = u0(i);
        } else if (i == 3) {

            toq[i] = u0(i);
        } else if (i == 4) {

            toq[i] = u0(i);
        } else

            return;
    }
}*/
    /*
void Controller::Impedance(Jointd &q, Jointd &q_dot, Jointd &q_ddot, double *dq, double *dq_dot, double *dq_ddot, double *toq, Matrixd &m_mat, Jointd &g_mat)
{
	Damp << 0.5, 0.5, 0.5, 0.5, 0.5;
	Stiff << 8.0,8.0,8.0,8.0,8.0;

	Jointd u;

	for(int i=0; i<m_Jnum; ++i)
	{

		e(i) = dq[i] - q(i);
		e_dev(i) = dq_dot[i] - q_dot(i);

	}

	//	int i = 0; //Joint Number
	u = m_mat*q_ddot +g_mat + Damp.cwiseProduct(e_dev) + Stiff.cwiseProduct(e);

	for(int i=0; i<m_Jnum; ++i)
	{

		if(i==0){

			toq[i] = u(i)/(double)(TORQUE_CONST_1*MAX_CURRENT_1*HARMONIC_100)*1000;
		}
		else if(i==1){

			toq[i] = u(i)/(double)(TORQUE_CONST_2*MAX_CURRENT_2*HARMONIC_100)*1000;
		}
		else if(i==2){

			toq[i] = u(i)/(double)(TORQUE_CONST_3*MAX_CURRENT_3*HARMONIC_100)*1000;
		}
		else if(i==3){

			toq[i] = u(i)/(double)(TORQUE_CONST_4*MAX_CURRENT_4*HARMONIC_100)*1000;
		}
		else if(i==4){

			toq[i] = u(i)/(double)(TORQUE_CONST_5*MAX_CURRENT_5*HARMONIC_100)*1000;
		}
		else

			return;
	}
}
void Controller::Impedance(Jointd &q, Jointd &q_dot, Jointd &q_ddot, double *dq, double *dq_dot, double *dq_ddot, double *toq, Matrixd &m_mat, Jointd &g_mat, Matrixd &c_mat)
{
//	Damp << 3.0, 3.0, 1.4, 2.8, 1.4;
//	Stiff << 40.0,40.0,20.0,40.0,20.0;
	Damp << 1.0, 1.0, 1.0, 1.0, 1.0;
	Stiff << 30.0,30.0,30.0,30.0,30.0;

	Jointd u, dq_ddot_vec;

	for(int i=0; i<m_Jnum; ++i)
	{

		e(i) = dq[i] - q(i);
		e_dev(i) = dq_dot[i] - q_dot(i);
		dq_ddot_vec(i) = dq_ddot[i];
	}

	//	int i = 0; //Joint Number
	u = m_mat * dq_ddot_vec + c_mat* q_dot + g_mat + Damp.cwiseProduct(e_dev) + Stiff.cwiseProduct(e);

	for(int i=0; i<m_Jnum; ++i)
	{

		if(i==0){

			toq[i] = u(i)/(double)(TORQUE_CONST_1*MAX_CURRENT_1*HARMONIC_100)*1000;
		}
		else if(i==1){

			toq[i] = u(i)/(double)(TORQUE_CONST_2*MAX_CURRENT_2*HARMONIC_100)*1000;
		}
		else if(i==2){

			toq[i] = u(i)/(double)(TORQUE_CONST_3*MAX_CURRENT_3*HARMONIC_100)*1000;
		}
		else if(i==3){

			toq[i] = u(i)/(double)(TORQUE_CONST_4*MAX_CURRENT_4*HARMONIC_100)*1000;
		}
		else if(i==4){

			toq[i] = u(i)/(double)(TORQUE_CONST_5*MAX_CURRENT_5*HARMONIC_100)*1000;
		}
		else

			return;
	}
}*/

/*
void Controller::Impedance(double *q, double *q_dot, double *q_ddot, double *dq, double *dq_dot, double *dq_ddot, double *toq, Matrix6f &m_mat, Matrix6f &c_mat, Jointf &g_mat)
{
	Kp << 118.7931,		119.5279,	59.764,		160.2564,		19.1946;
	//Ki={860.8196,		866.1442,	4.1716,		427.5721,	135.3359};
	Kd << 4.0984,			4.1237,		2.0619,		2.5641,		0.691;
	Damp={0.5, 0.5, 0.5, 0.5, 0.5};
	Stiff={8.0,8.0,8.0,8.0,8.0};

	//	int i = 0; //Joint Number
	for(int i=0; i<m_Jnum; ++i)
	{

		e.at(i) = dq[i] - q[i];
		e_dev.at(i) = dq_dot[i] - q_dot[i];
		//Jointf g = Liedynamics::G_Matrix();

		if(i==0){
			u0=q_ddot[i] + m_mat(i).inverse()*(Damp.at(i)*e_dev.at(i)+Stiff.at(i)*e.at(i));
			toq[i] = (m_mat(i)*u0 + c_mat(i)*q_dot + g_mat(i))/(float)(TORQUE_CONST_1*MAX_CURRENT_1*HARMONIC_100)*1000;
		}
		else if(i==1){
			u0=q_ddot[i] + m_mat(i).inverse()*(Damp.at(i)*e_dev.at(i)+Stiff.at(i)*e.at(i));
			toq[i] = (m_mat(i)*u0 + c_mat(i)*q_dot + g_mat(i))/(float)(TORQUE_CONST_2*MAX_CURRENT_2*HARMONIC_100)*1000;
		}
		else if(i==2){
			u0=q_ddot[i] + m_mat(i).inverse()*(Damp.at(i)*e_dev.at(i)+Stiff.at(i)*e.at(i));
			toq[i] = (m_mat(i)*u0 + c_mat(i)*q_dot + g_mat(i))/(float)(TORQUE_CONST_3*MAX_CURRENT_3*HARMONIC_100)*1000;
		}
		else if(i==3){
			u0=q_ddot[i] + m_mat(i).inverse()*(Damp.at(i)*e_dev.at(i)+Stiff.at(i)*e.at(i));
			toq[i] = (m_mat(i)*u0 + c_mat(i)*q_dot + g_mat(i))/(float)(TORQUE_CONST_4*MAX_CURRENT_4*HARMONIC_100)*1000;
		}
		else if(i==4){
			u0=q_ddot[i] + m_mat(i).inverse()*(Damp.at(i)*e_dev.at(i)+Stiff.at(i)*e.at(i));
			toq[i] = (m_mat(i)*u0 + c_mat(i)*q_dot + g_mat(i))/(float)(TORQUE_CONST_5*MAX_CURRENT_5*HARMONIC_100)*1000;
		}
		else
		/*	int_flag++;
			if(int_flag>10000000){
				int_flag=0;
				ClearError();
			}*/
/*			return;
	}
}*/


void Controller::TorqueOutput(double *p_toq , int maxtoq, int *p_dir)
{
    double toq_tmp=0;
	for(int i=0; i<m_Jnum; ++i)
	{
		toq_tmp = p_toq[i];
		if(toq_tmp <= -maxtoq)
		{
			p_toq[i] = p_dir[i]*-maxtoq;
		}
		else if(toq_tmp >= maxtoq)
		{
			p_toq[i] = p_dir[i]*maxtoq;
		}
		else
		{
			p_toq[i] = p_dir[i]*toq_tmp;
		}
	}
	return;
}

void Controller::CLIKController(double * _q, double * _qdot, double * _dq, double * _dqdot, const Eigen::VectorXd * _dx, const Eigen::VectorXd * _dxdot, const Eigen::VectorXd & _dqdotNull, double * p_Toq, double & _dt)
{
    G=pManipulator->pDyn->G_Matrix();
    q = Map<VectorXd>(_q, this->m_Jnum);
    qdot = Map<VectorXd>(_qdot, this->m_Jnum);

    LinearJacobian=pManipulator->pKin->LinearJacobian();

    eTask.setZero();
    edotTmp.setZero();
    edotTask.setZero();

}

void Controller::CLIKController_2nd(double *_q, double *_qdot, Matrix<double,5,1>& dq, Matrix<double,5,1>& dq_dot, Matrix<double,5,1>& dq_ddot)
{

}

Jointd Controller::return_u0(void)
{
	return u0;
}

} /* namespace HYUCtrl */
