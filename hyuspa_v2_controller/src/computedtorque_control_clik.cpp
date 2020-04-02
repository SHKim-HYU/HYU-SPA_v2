//
// Created by june on 19. 12. 5..
//


// from ros-control meta packages
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>

#include <pluginlib/class_list_macros.h>
#include <std_msgs/Float64MultiArray.h>

#include <urdf/model.h>

// from kdl packages
#include <kdl/tree.hpp>
#include <kdl/kdl.hpp>
#include <kdl/chain.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chaindynparam.hpp>              // inverse dynamics
#include <kdl/chainjnttojacsolver.hpp>        // jacobian
#include <kdl/chainfksolverpos_recursive.hpp> // forward kinematics
#include <kdl/chainfksolvervel_recursive.hpp> // forward kinematics


#include <boost/scoped_ptr.hpp>
#include <boost/lexical_cast.hpp>

#include <cmath>
#include <Controller.h>

#define _USE_MATH_DEFINES

#include "SerialRobot.h"
#include "Trajectory.h"

#define R2D 180/M_PI
#define D2R M_PI/180
#define num_taskspace 6
#define SaveDataMax 97

namespace  hyuspa_v2_controller
{
    class ComputedTorque_Control_CLIK : public controller_interface::Controller<hardware_interface::EffortJointInterface>
    {
    public:
        bool init(hardware_interface::EffortJointInterface *hw, ros::NodeHandle &n)
        {
            // ********* 1. Get joint name / gain from the parameter server *********
            // 1.0 Control objective & Inverse Kinematics mode
            ROS_INFO("#######Initialize start##########");
            if (!n.getParam("ctr_obj", ctr_obj_))
            {
                ROS_ERROR("Could not find control objective");
                return false;
            }

            if (!n.getParam("ik_mode", ik_mode_))
            {
                ROS_ERROR("Could not find control objective");
                return false;
            }

            // 1.1 Joint Name
            if (!n.getParam("joints", joint_names_))
            {
                ROS_ERROR("Could not find joint name");
                return false;
            }
            n_joints_ = joint_names_.size();
            ROS_INFO("#########joint info is obtained########");
            if (n_joints_ == 0)
            {
                ROS_ERROR("List of joint names is empty.");
                return false;
            }
            else
            {
                ROS_INFO("Found %d joint names", n_joints_);
                for (int i = 0; i < n_joints_; i++)
                {
                    ROS_INFO("%s", joint_names_[i].c_str());
                }
            }

            // 1.2 Gain
            // 1.2.1 Joint Controller
            Kp_.resize(n_joints_);
            Kd_.resize(n_joints_);
            Ki_.resize(n_joints_);


            std::vector<double> Kp(n_joints_), Ki(n_joints_), Kd(n_joints_);

            for (size_t i = 0; i < n_joints_; i++)
            {
                std::string si = std::to_string(i + 1);
                if (n.getParam("/hyuspa_v2/computedtorque_control_clik/gains/Arm_Joint_" + si + "/pid/p", Kp[i]))
                {
                    Kp_(i) = Kp[i];
                }
                else
                {
                    std::cout << "/hyuspa_v2/computedtorque_control_clik/gains/Arm_Joint_" + si + "/pid/p" << std::endl;
                    ROS_ERROR("Cannot find pid/p gain");
                    return false;
                }

                if (n.getParam("/hyuspa_v2/computedtorque_control_clik/gains/Arm_Joint_" + si + "/pid/i", Ki[i]))
                {
                    Ki_(i) = Ki[i];
                }
                else
                {
                    ROS_ERROR("Cannot find pid/i gain");
                    return false;
                }

                if (n.getParam("/hyuspa_v2/computedtorque_control_clik/gains/Arm_Joint_" + si + "/pid/d", Kd[i]))
                {
                    Kd_(i) = Kd[i];
                }
                else
                {
                    ROS_ERROR("Cannot find pid/d gain");
                    return false;
                }
            }

            // 1.2.2 Closed-loop Inverse Kinematics Controller
            if (ctr_obj_ == 1)
            {
                if (!n.getParam("/hyuspa_v2/computedtorque_control_clik/clik_gain/K_regulation", K_regulation_))
                {
                    ROS_ERROR("Cannot find clik regulation gain");
                    return false;
                }
            }

            else if (ctr_obj_ == 2)
            {
                if (!n.getParam("/hyuspa_v2/computedtorque_control_clik/clik_gain/K_tracking", K_tracking_))
                {
                    ROS_ERROR("Cannot find clik tracking gain");
                    return false;
                }
            }

            // 2. ********* urdf *********
            urdf::Model urdf;
            if (!urdf.initParam("robot_description"))
            {
                ROS_ERROR("Failed to parse urdf file");
                return false;
            }
            else
            {
                ROS_INFO("Found robot_description");
            }

            // 3. ********* Get the joint object to use in the realtime loop [Joint Handle, URDF] *********
            for (int i = 0; i < n_joints_; i++)
            {
                try
                {
                    joints_.push_back(hw->getHandle(joint_names_[i]));
                }
                catch (const hardware_interface::HardwareInterfaceException &e)
                {
                    ROS_ERROR_STREAM("Exception thrown: " << e.what());
                    return false;
                }

                urdf::JointConstSharedPtr joint_urdf = urdf.getJoint(joint_names_[i]);
                if (!joint_urdf)
                {
                    ROS_ERROR("Could not find joint '%s' in urdf", joint_names_[i].c_str());
                    return false;
                }
                joint_urdfs_.push_back(joint_urdf);
            }

            // 4. ********* KDL *********
            // 4.1 kdl parser
            if (!kdl_parser::treeFromUrdfModel(urdf, kdl_tree_))
            {
                ROS_ERROR("Failed to construct kdl tree");
                return false;
            }
            else
            {
                ROS_INFO("Constructed kdl tree");
            }

            // 4.2 kdl chain
            std::string root_name, tip_name1;
            if (!n.getParam("root_link", root_name))
            {
                ROS_ERROR("Could not find root link name");
                return false;
            }
            if (!n.getParam("tip_link1", tip_name1))
            {
                ROS_ERROR("Could not find tip link name");
                return false;
            }
            if (!kdl_tree_.getChain(root_name, tip_name1, kdl_chain_))
            {
                ROS_ERROR_STREAM("Failed to get KDL chain from tree: ");
                ROS_ERROR_STREAM("  " << root_name << " --> " << tip_name1);
                ROS_ERROR_STREAM("  Tree has " << kdl_tree_.getNrOfJoints() << " joints");
                ROS_ERROR_STREAM("  Tree has " << kdl_tree_.getNrOfSegments() << " segments");
                ROS_ERROR_STREAM("  The segments are:");

                KDL::SegmentMap segment_map = kdl_tree_.getSegments();
                KDL::SegmentMap::iterator it;

                for (it = segment_map.begin(); it != segment_map.end(); it++)
                    ROS_ERROR_STREAM("    " << (*it).first);

                return false;
            }
            else
            {
                ROS_INFO("Got kdl chain");
            }
            gravity_ = KDL::Vector::Zero();
            gravity_(2) = -9.81; // 0: x-axis 1: y-axis 2: z-axis

            M_.resize(kdl_chain_.getNrOfJoints());
            C_.resize(kdl_chain_.getNrOfJoints());
            G_.resize(kdl_chain_.getNrOfJoints());

            id_solver_.reset(new KDL::ChainDynParam(kdl_chain_, gravity_));

            jnt_to_jac_solver_.reset(new KDL::ChainJntToJacSolver(kdl_chain_));
            fk_pos_solver_.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_));

            // ********* 5. 각종 변수 초기화 *********

            // 5.1 KDL Vector 초기화 (사이즈 정의 및 값 0)
            x_cmd_.data = Eigen::VectorXd::Zero(num_taskspace);

            qd_.data = Eigen::VectorXd::Zero(n_joints_);
            qd_dot_.data = Eigen::VectorXd::Zero(n_joints_);
            qd_ddot_.data = Eigen::VectorXd::Zero(n_joints_);
            qd_old_.data = Eigen::VectorXd::Zero(n_joints_);

            q_.data = Eigen::VectorXd::Zero(n_joints_);
            qdot_.data = Eigen::VectorXd::Zero(n_joints_);

            q_chain[0].data = Eigen::VectorXd::Zero(kdl_chain_.getNrOfJoints());

            J_.resize(kdl_chain_.getNrOfJoints());

            TrajFlag_j.resize(n_joints_);
            TrajFlag_j.setZero();
            TrajFlag_t.resize(n_joints_);
            TrajFlag_t.setZero();
            TrajFlag_td.resize(num_taskspace);
            TrajFlag_td.setZero();
            startPose.resize(4,4);
            startPose.setIdentity();
            finalPose.resize(4,4);
            finalPose.setIdentity();

            xd.resize(6);
            xa.resize(6);
            xda.resize(6);
            traj_xd.resize(6);
            res_trad.resize(6);

            // ********* 6. ROS 명령어 *********
            // 6.1 publisher
            pub_qd_ = n.advertise<std_msgs::Float64MultiArray>("qd", 1000);
            pub_q_ = n.advertise<std_msgs::Float64MultiArray>("q", 1000);
            pub_e_ = n.advertise<std_msgs::Float64MultiArray>("e", 1000);

            pub_xd_ = n.advertise<std_msgs::Float64MultiArray>("xd", 1000);
            pub_x_ = n.advertise<std_msgs::Float64MultiArray>("x", 1000);
            pub_ex_ = n.advertise<std_msgs::Float64MultiArray>("ex", 1000);

            pub_SaveData_ = n.advertise<std_msgs::Float64MultiArray>("SaveData", 1000);

            // 6.2 subsriber
            sub_x_cmd_ = n.subscribe<std_msgs::Float64MultiArray>("command",
                    1, &ComputedTorque_Control_CLIK::commandCB, this);
            event = 0; // subscribe 받기 전: 0
            // subscribe 받은 후: 1

            return true;
        }

        void commandCB(const std_msgs::Float64MultiArrayConstPtr &msg)
        {
            if (msg->data.size() != num_taskspace)
            {
                ROS_ERROR_STREAM("Dimension of command (" << msg->data.size() << ") does not match DOF of Task Space (" << 2 << ")! Not executing!");
                return;
            }

            for (int i = 0; i < num_taskspace; i++)
            {
                x_cmd_(i) = msg->data[i];
            }

            event = 1;  // subscribe 받기 전: 0
                        // subscribe 받은 후: 1
        }

        void starting(const ros::Time &time) override
        {
            t = 0.0;
            ROS_INFO("Starting Computed Torque Controller with Closed-Loop Inverse Kinematics");
            //xd={0.048723,-0.30684,1.23653};
            cManipulator = new robot;
            Control = new HYUControl::Controller(cManipulator,n_joints_);
            traj5th_joint = new hyuCtrl::Trajectory();
            traj5th_task = new hyuCtrl::Trajectory();
            qd.setZero();
            qd_dot.setZero();
            qd_old.setZero();

            q_flag=0;
            cManipulator->robot_update_R();
            for(int i=0; i<n_joints_; i++)
            {
                Control->SetPIDGain(Kp_(i), Kd_(i), Ki_(i), i);
            }

            //ROS_INFO("Starting is done");
        }

        void update(const ros::Time &time, const ros::Duration &period) override
        {
            // ********* 0. Get states from gazebo *********
            // 0.1 sampling time
            double dt = period.toSec();
            t = t + 0.001;

            // 0.2 joint state
            for (int i = 0; i < n_joints_; i++)
            {
                q_(i) = joints_[i].getPosition();
                qdot_(i) = joints_[i].getVelocity();
                //torque_(i) = joints_[i].getEffort();
            }

            //Implement the control code
            Eigen::Map<VectorXd>(q, n_joints_) = q_.data;
            Eigen::Map<VectorXd>(qdot, n_joints_) = qdot_.data;

            //ROS_INFO("Controller start");
            cManipulator->pKin->Unflag_isInfoupdate();
            //ROS_INFO("PKIN info");
            cManipulator->pKin->HTransMatrix(q);
            //ROS_INFO("PKIN HTransMatrix");
            rpy = cManipulator->pKin->GetEulerAngle();
            quat = cManipulator->pKin->GetQuaternion();
            //cManipulator->pDyn->Prepare_Dynamics(q,qdot);////////////////////////
            //ROS_INFO("PDyn Prepare");

            b_jaco=cManipulator->pKin->BodyJacobian();
            s_jaco=cManipulator->pKin->SpaceJacobian();
            l_jaco=cManipulator->pKin->LinearJacobian();
            l_jaco_dot=cManipulator->pKin->Jacobian_l_dot();
            a_jaco=cManipulator->pKin->AnalyticJacobian();
            DPI_jaco=cManipulator->pKin->DPI(l_jaco);
            jnt_to_jac_solver_->JntToJac(q_, J_);
            fk_pos_solver_->JntToCart(q_, x_);



            //Control->InvDynController(q, qdot, dq, dqdot, dqddot, torque, dt);
            //torque[0]=-5.0;torque[1]=3.0;torque[2]=0.0;torque[3]=2.0;torque[4]=0.0;

            xdot=l_jaco*qdot_.data;
            x=cManipulator->pKin->ForwardKinematics();

            xdota=a_jaco*qdot_.data;
            ROT=cManipulator->pKin->Rot();
            xa.block<3,1>(0,0)=cManipulator->pKin->so3ToVec(ROT);
            xa.block<3,1>(3,0)=x;

            id_solver_->JntToMass(q_, M_);
            id_solver_->JntToCoriolis(q_, qdot_, C_);
            id_solver_->JntToGravity(q_, G_); // output은 머지? , id_solver는 어디에서?
            /////////////Trajectory for Joint Space//////////////
 /*           if(Motion==1 &&TrajFlag_j(0)==0)
            {
                traj_q[0]=0.01; traj_q[1]=-1.57; traj_q[2]= 0.01; traj_q[3]=1.57; traj_q[4]=0.01;
                Motion++;
                for(int i=0;i<n_joints_;i++)
                {
                    TrajFlag_j(i)=1;
                }
            }
            else if(Motion==2 &&TrajFlag_j(0)==0)
            {
                traj_q[0]=0.732177; traj_q[1]=-1.231181; traj_q[2]= -1.284267; traj_q[3] = 1.05148; traj_q[4] = -0.407245;
                Motion++;
                for(int i=0;i<n_joints_;i++)
                {
                    TrajFlag_j(i)=1;
                }
            }
            else if(Motion==3 &&TrajFlag_j(0)==0)
            {
                traj_q[0]=0.741427; traj_q[1]=-1.961923; traj_q[2]= -1.348277; traj_q[3] = 0.493244; traj_q[4] = -0.407213;
                Motion=1;
                for(int i=0;i<n_joints_;i++)
                {
                    TrajFlag_j(i)=1;
                }
            }

            for (int i = 0; i < n_joints_; i++) {

                if (TrajFlag_j(i)==2)
                {
                    traj5th_joint->Polynomial5th(i, t, &TrajFlag_j(i), res_tra);
                    qd[i] = res_tra[0];
                    qd_dot[i] = res_tra[1];
                    qd_ddot[i]=res_tra[2];
                }
                else if(TrajFlag_j(i)==1) {
                    traj5th_joint->SetPolynomial5th(i, q[i], traj_q(i), t, 2.0, res_tra);
                    qd[i] = res_tra[0];
                    qd_dot[i] = res_tra[1];
                    qd_ddot[i]=res_tra[2];
                    TrajFlag_j(i)=2;
                }

            }
            */
            ////////////////////////


           if(t<=5)
           {
               /////////////Trajectory for Joint Space//////////////
               if(TrajFlag_j[0]==0)
               {
                   traj_q[0]=-0.732177; traj_q[1]=0.3; traj_q[2]= 1.084267; traj_q[3]=-1.05148; traj_q[4]=-0.407245; traj_q[5]=0.6;
                   //traj_q[0]=0.0; traj_q[1]=-1.57; traj_q[2]= 0.0; traj_q[3]=1.57; traj_q[4]=0.0;
                   //traj_q[0]=0.0; traj_q[1]=0.0; traj_q[2]= 0.0; traj_q[3]=0.0; traj_q[4]=0.0;
                   for(int i=0;i<n_joints_;i++)
                   {
                       TrajFlag_j(i)=1;
                   }
               }

                for (int i = 0; i < n_joints_; i++) {

                    if (TrajFlag_j(i)==2)
                    {
                        traj5th_joint->Polynomial5th(i, t, &TrajFlag_j(i), res_tra);
                        qd[i] = res_tra[0];
                        qd_dot[i] = res_tra[1];
                        qd_ddot[i]=res_tra[2];
                        if(TrajFlag_j[i]==0)
                        {
                            TrajFlag_j[i] = 3;
                        }
                    }
                    else if(TrajFlag_j(i)==1) {
                        traj5th_joint->SetPolynomial5th(i, q[i], traj_q(i), t, 2.0, res_tra);
                        qd[i] = res_tra[0];
                        qd_dot[i] = res_tra[1];
                        qd_ddot[i]=res_tra[2];
                        TrajFlag_j(i)=2;
                    }

                }

               ////////////////////////
            }
            else
            {
                traj_flag=1;
                /////////////Trajectory for Task Space//////////////
                if(Motion==1 &&TrajFlag_t(0)==0)
                {
                    traj_x[0]=0.308723; traj_x[1]=-0.30684; traj_x[2]= 1.23653;
                    Motion++;
                    for(int i=0;i<3;i++)
                    {
                        TrajFlag_t(i)=1;
                    }
                }
                else if(Motion==2 &&TrajFlag_t(0)==0)
                {
                    traj_x[0]=0.308723; traj_x[1]=-0.00684; traj_x[2]= 1.23653;
                    Motion++;
                    for(int i=0;i<3;i++)
                    {
                        TrajFlag_t(i)=1;
                    }
                }
                else if(Motion==3 &&TrajFlag_t(0)==0)
                {
                    //traj_x[0]=0.148723; traj_x[1]=-0.30684; traj_x[2]= 1.23653;
                    traj_x[0]=0.308723; traj_x[1]=-0.40684; traj_x[2]= 1.23653;
                    Motion++;
                    for(int i=0;i<3;i++)
                    {
                        TrajFlag_t(i)=1;
                    }
                }
                else if(Motion==4 &&TrajFlag_t(0)==0)
                {
                    //traj_x[0]=0.148723; traj_x[1]=-0.30684; traj_x[2]= 1.33653;
                    traj_x[0]=0.308723; traj_x[1]=-0.030684; traj_x[2]= 1.23653;
                    Motion++;
                    for(int i=0;i<3;i++)
                    {
                        TrajFlag_t(i)=1;
                    }
                }
                else if(Motion==5 &&TrajFlag_t(0)==0)
                {
                    //traj_x[0]=0.108723; traj_x[1]=-0.70684; traj_x[2]= 1.43653;
                    traj_x[0]=0.308723; traj_x[1]=-0.40684; traj_x[2]= 1.23653;
                    Motion++;
                    for(int i=0;i<3;i++)
                    {
                        TrajFlag_t(i)=1;
                    }
                }
                else if(Motion==6 &&TrajFlag_t(0)==0)
                {
                    //traj_x[0]=0.508723; traj_x[1]=-0.60684; traj_x[2]= 1.567653;
                    traj_x[0]=0.308723; traj_x[1]=-0.00684; traj_x[2]= 1.23653;
                    Motion++;
                    for(int i=0;i<3;i++)
                    {
                        TrajFlag_t(i)=1;
                    }
                }
                else if(Motion==7 &&TrajFlag_t(0)==0)
                {
                    //traj_x[0]=0.508723; traj_x[1]=-0.60684; traj_x[2]= 1.567653;
                    traj_x[0]=0.308723; traj_x[1]=-0.40684; traj_x[2]= 1.33653;
                    Motion++;
                    for(int i=0;i<3;i++)
                    {
                        TrajFlag_t(i)=1;
                    }
                }
                else if(Motion==8 &&TrajFlag_t(0)==0)
                {
                    //traj_x[0]=0.508723; traj_x[1]=-0.60684; traj_x[2]= 1.567653;
                    traj_x[0]=0.308723; traj_x[1]=-0.40684; traj_x[2]= 1.43653;
                    Motion++;
                    for(int i=0;i<3;i++)
                    {
                        TrajFlag_t(i)=1;
                    }
                }
                else if(Motion==9 &&TrajFlag_t(0)==0)
                {
                    //traj_x[0]=0.508723; traj_x[1]=-0.60684; traj_x[2]= 1.567653;
                    traj_x[0]=0.308723; traj_x[1]=-0.40684; traj_x[2]= 1.23653;
                    Motion++;
                    for(int i=0;i<3;i++)
                    {
                        TrajFlag_t(i)=1;
                    }
                }
                else if(Motion==10 &&TrajFlag_t(0)==0)
                {
                    //traj_x[0]=0.508723; traj_x[1]=-0.60684; traj_x[2]= 1.567653;
                    traj_x[0]=0.308723; traj_x[1]=-0.20684; traj_x[2]= 1.43653;
                    Motion=1;
                    for(int i=0;i<3;i++)
                    {
                        TrajFlag_t(i)=1;
                    }
                }

                for (int i = 0; i < 3; i++) {

                    if (TrajFlag_t(i)==2)
                    {
                        traj5th_task->Polynomial5th(i, t, &TrajFlag_t(i), res_tra);
                        xd[i] = res_tra[0];
                        xddot[i]=res_tra[1];
                        xdddot[i]=res_tra[2];
                    }
                    else {
                        traj5th_task->SetPolynomial5th(i, x[i], traj_x(i), t, 2.0, res_tra);
                        xd[i] = res_tra[0];
                        xddot[i]=res_tra[1];
                        xdddot[i]=res_tra[2];
                        TrajFlag_t(i)=2;
                    }

                }
                //Circle Trajectory
                if(t_flag==0)
                {
                    t_buf=t;
                    t_flag=1;
                }
                xd[0]=0.308723;
                xd[1]=-0.20684+0.2*sin(t_buf-t);
                xd[2]=1.23653+0.2*(1-cos(t_buf-t));

                /////////////Trajectory for Task Space//////////////
  /*            if(Motion==1 &&TrajFlag_td(0)==0)
                {
                    Matrix3d Rd;
                    Rd<<0.650,-0.43,-0.6268,0.735,0.144,0.6625,0.1941,-0.8918,0.4083;
                    ROTD=cManipulator->pKin->MatrixLog3(Rd);
                    traj_xd[3]=0.2805; traj_xd[4]=-0.2179; traj_xd[5]= 1.31607;
                    traj_xd.head(3)=cManipulator->pKin->so3ToVec(ROTD);
                    Motion++;
                    TrajFlag_td(0)=1;
                }
               if(Motion==2 &&TrajFlag_td(0)==0)
               {
                   Matrix3d Rd;
                   Rd<<0.75176,-0.446541,-0.485239,0.444412,-0.200569,0.873081,-0.48719,-0.871994,0.0476685;
                   ROTD=cManipulator->pKin->MatrixLog3(Rd);
                   traj_xd[3]=0.307105; traj_xd[4]=-0.412607; traj_xd[5]= 1.32142;
                   traj_xd.head(3)=cManipulator->pKin->so3ToVec(ROTD);
                   Motion=1;
                   TrajFlag_td(0)=1;
               }
*/
                if (TrajFlag_td(0)==2)
                {
                    traj5th_task->Trans5th(xa, traj_xd, 2.0, t, &TrajFlag_td(0), res_trad);
                    xda = res_trad;
                }
                else if(TrajFlag_td(0)==1){
                    traj5th_task->SetTrans5th(xa, traj_xd, t, 2.0, res_trad);
                    xda = res_trad;
                    TrajFlag_td(0)=2;
                }


                ////////////////////////
                // CLIK Algorithm
                /////general CLIK
/*                qd_dot=DPI_jaco*(xddot + K_regulation_*(xd-x));
                //qd_dot= DPI_jaco*(K_regulation_*(xda-xa));
                qd = qd_old + qd_dot*dt;

                if(q_flag==0)
                    qd_old=Map<VectorXd>(q,5);
                else
                {
                    qd_old=qd;
                    q_flag=1;
                }
*/
                ///////////////////
                ///// qd CLIK
/*                if(q_flag==0)
                {
                    qd_dot_old = Map<VectorXd>(qdot, 5);
                    qd_old = Map<VectorXd>(q, 5);

                    q_flag=1;
                }
                qd_ddot=DPI_jaco*(xdddot - l_jaco_dot*qdot_.data+250000*(xd-x)+1000*(xddot-xdot));
                qd_dot = qd_dot_old + qd_ddot*dt;
                qd = qd_old + qd_dot*dt - qd_ddot*pow(dt,2)/2;
                if(q_flag==1)
                {
                    qd_dot_old=qd_dot;
                    qd_old=qd;
                }*/
                ////////////////////
            }
               for(int i=0;i<n_joints_;i++)
               {
                   dq[i]=qd(i);
                   dqdot[i]=qd_dot(i);
               }

            cManipulator->pDyn->Prepare_Dynamics(q,qdot);

            if(traj_flag==0)
                Control->PD_Gravity(q, qdot, qd, qd_dot, torque);
            else
               //C0=Control->VSD(q,qdot,traj_x,torque);
               C0=Control->VSD(q,qdot,xd,torque);
              //Control->VSD_ori(q,qdot,xda,torque);


            //Control->ComputedTorque(q, qdot, qd, qd_dot, qd_ddot, torque);
            //Control->PD_Gravity(q, qdot, qd, qd_dot, torque);
            //Control->Inverse_Dynamics_Control(q, qdot, qd, qd_dot, qd_ddot, torque);
            //Control->Gravity(q,qdot,torque);
            //cManipulator->pDyn->Mdot_Matrix(_Mdot);
            for (int i = 0; i < n_joints_; i++)
            {
                joints_[i].setCommand(torque[i]);
                //joints_[i].setCommand(0.0);
            }

            // ********* 4. data 저장 *********
            save_data();

            // ********* 5. state 출력 *********
            print_state();
        }

        void stopping(const ros::Time &time) override
        {
            delete Control;
            delete cManipulator;
            delete traj5th_joint;
            delete traj5th_task;
        }

        static void save_data()
        {

        }

        void print_state()
        {
            static int count = 0;
            if (count > 99)
            {
                printf("*********************************************************\n\n");
                printf("*** Simulation Time (unit: sec)  ***\n");
                printf("t = %f\n", t);
                printf("\n");

                printf("*** Command from Subscriber in Task Space (unit: m) ***\n");
                if (event == 0)
                {
                    printf("No Active!!!\n");
                }
                else
                {
                    printf("Active!!!\n");
                }

                printf("*** States in Joint Space (unit: deg) ***\n");
                for(int i=0; i < n_joints_; i++)
                {
                    printf("Joint ID:%d \t", i);
//                    printf("q: %0.3f, ", q_(i) * R2D);
//                    printf("dq: %0.3f, ", qd_(i) * R2D);
//                    printf("qdot: %0.3f, ", qdot_(i) * R2D);
//                    printf("dqdot: %0.3f, ", qd_dot_(i) * R2D);
                    printf("q: %0.3f, ", q_(i));
                    printf("dq: %0.3f, ", qd(i));
                    printf("qdot: %0.3f, ", qdot_(i));
                    printf("dqdot: %0.3f, ", qd_dot(i));
                    printf("torque: %0.3f", torque[i]);
                    printf("\n");
                }
                cout<<"FK"<<endl<<cManipulator->pKin->ForwardKinematics()<<endl;
                cout<<"KDL FK"<<endl<<x_.p(0)<<endl<<x_.p(1)<<endl<<x_.p(2)<<endl;
                cout<<"Euler Angle"<<endl<<rpy(0)<<", "<<rpy(1)<<", "<<rpy(2)<<endl;

                //cout<<"Linear Velocity Jacobian"<<endl<<l_jaco<<endl;
                //JacobiSVD<MatrixXd> svd(l_jaco,ComputeThinU|ComputeThinV);
                //cout<<"w: "<<cManipulator->pKin->Manipulability(l_jaco)<<endl;
                //cout<<"k: "<<cManipulator->pKin->Condition_Number(l_jaco)<<endl;
                //cout<<"Singular Value"<<endl<<svd.singularValues()<<endl;
                //cout<<"Rotation Matrix"<<endl<<cManipulator->pKin->GetTMat(0,5)<<endl;
                //cout<<"Xd: "<<xda<<"  X: "<<xa<<endl;
                //cout<<"KDL Jacobian"<<endl<<J_.data.block<3,5>(0,0)<<endl;
                //cout<<"Analytic Jacobian"<<endl<<l_jaco<<endl;
                //cout<<"KDL Jacobian"<<endl<<J_.data<<endl;
                //cout<<"Body Jacobian"<<endl<<b_jaco<<endl;
                //cout<<"PI"<<endl<<cManipulator->pKin->Pinv(l_jaco)<<endl;
                //cout<<"DPI"<<endl<<cManipulator->pKin->DPI(l_jaco)<<endl;
                //cout<<"xdot"<<endl<<xdot<<endl;
                //cout<<"KDL xdot"<<endl<<(J_.data*qdot_.data).block<3,1>(0,0)<<endl;
                //cout<<"Gravity"<<endl<<cManipulator->pDyn->G_Matrix()<<endl;
                //cout<<"Verify"<<endl<<_Mdot-2*cManipulator->pDyn->C_Matrix()<<endl;
                //cout<<"M"<<endl<<cManipulator->pDyn->M_Matrix()<<endl;
                cout<<C0<<endl;
                //cout<<"KDL M"<<endl<<M_.data<<endl;
                //cout<<"C"<<endl<<cManipulator->pDyn->C_out()*qdot_.data<<endl;
                //cout<<"KDL C"<<endl<<C_.data<<endl;
                //cout<<"Jaco_dot"<<endl<<cManipulator->pKin->Jacobian_l_dot()<<endl;

                count = 0;
            }
            count++;
        }

    private:
        // others
        double t;
        int ctr_obj_;
        int ik_mode_;
        int event;

        //Joint handles
        unsigned int n_joints_;                               // joint 숫자
        std::vector<std::string> joint_names_;                // joint name ??
        std::vector<hardware_interface::JointHandle> joints_; // ??
        std::vector<urdf::JointConstSharedPtr> joint_urdfs_;  // ??

        // kdl
        KDL::Tree kdl_tree_;   // tree?
        KDL::Chain kdl_chain_; // chain?

        // kdl M,C,G
        KDL::JntSpaceInertiaMatrix M_; // intertia matrix
        KDL::JntArray C_;              // coriolis
        KDL::JntArray G_;              // gravity torque vector
        KDL::Vector gravity_;

        // kdl and Eigen Jacobian
        KDL::Jacobian J_;
        //KDL::Jacobian J_inv_;
        //Eigen::Matrix<double, num_taskspace, num_taskspace> J_inv_;
        Eigen::MatrixXd J_inv_;
        Eigen::Matrix<double, num_taskspace, num_taskspace> J_transpose_;

        // kdl solver
        boost::scoped_ptr<KDL::ChainFkSolverPos_recursive> fk_pos_solver_; //Solver to compute the forward kinematics (position)
        boost::scoped_ptr<KDL::ChainFkSolverVel_recursive> fk_vel_solver_; //Solver to compute the forward kinematics (velocity)
        boost::scoped_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver_; //Solver to compute the jacobian
        boost::scoped_ptr<KDL::ChainDynParam> id_solver_;               // Solver To compute the inverse dynamics

        // Joint Space State
        KDL::JntArray qd_, qd_dot_, qd_ddot_;
        KDL::JntArray qd_old_;
        KDL::JntArray q_, qdot_;

        KDL::JntArray q_chain[2];
        KDL::JntArray qdot_chain[2];

        MatrixXd q_dot;

        double q[6], qdot[6];
        double dq[6] = {0.0,};
        double dqdot[6] = {0.0,};
        double dqddot[6] = {0.0,};
        double torque[6] = {0.0,};

        Matrix<double,1,6> C0;

        Jaco b_jaco;
        Jaco s_jaco;
        Jaco a_jaco;
        LinJaco l_jaco;
        LinJaco l_jaco_dot;
        PinvLJaco DPI_jaco;

        MatrixXd _Mdot;

        // Task Space State
        // ver. 01
        Vector3d xd, xddot, xdddot;
        VectorXd xa, xda;

        Vector3d x;
        Vector3d rpy;
        Vector4d quat;
        Matrix<double,6,1> qd;
        Matrix<double,6,1> qd_old;
        Matrix<double,6,1> qd_dot;
        Matrix<double,6,1> qd_dot_old;
        Matrix<double,6,1> qd_ddot;
        KDL::Frame xd_; // x.p: frame position(3x1), x.m: frame orientation (3x3)
        KDL::Frame x_;
        KDL::Twist ex_temp_;
        int q_flag;

        // KDL::Twist xd_dot_, xd_ddot_;
        Eigen::Matrix<double, num_taskspace, 1> ex_;
        Eigen::Matrix<double, num_taskspace, 1> xd_dot_, xd_ddot_;
        Eigen::Matrix<double, num_taskspace, 1> xdot_, xdota;
        Eigen::Matrix<double, num_taskspace, 1> ex_dot_, ex_int_;
        Eigen::Matrix<double, 3,1> xdot;
        // Input
        KDL::JntArray x_cmd_;

        //Trajectory
        int Motion = 1;
        VectorXi TrajFlag_j;
        VectorXi TrajFlag_t;
        VectorXi TrajFlag_td;
        Matrix<double,6,1> traj_q;
        Vector3d traj_x;
        VectorXd traj_xd, res_trad;
        double res_tra[3]={0.0,};
        Matrix3d ROT, ROTD;
        int traj_flag=0;
        double t_buf;
        int t_flag=0;
        MatrixXd startPose,finalPose;

        // gains
        KDL::JntArray Kp_, Ki_, Kd_;
        double K_regulation_, K_tracking_;

        // save the data
        double SaveData_[SaveDataMax];

        // ros subscriber
        ros::Subscriber sub_x_cmd_;

        // ros publisher
        ros::Publisher pub_qd_, pub_q_, pub_e_;
        ros::Publisher pub_xd_, pub_x_, pub_ex_;
        ros::Publisher pub_SaveData_;

        // ros message
        std_msgs::Float64MultiArray msg_qd_, msg_q_, msg_e_;
        std_msgs::Float64MultiArray msg_xd_, msg_x_, msg_ex_;
        std_msgs::Float64MultiArray msg_SaveData_;

        robot *cManipulator;
        HYUControl::Controller *Control;
        hyuCtrl::Trajectory *traj5th_joint;
        hyuCtrl::Trajectory *traj5th_task;

    };
}

PLUGINLIB_EXPORT_CLASS(hyuspa_v2_controller::ComputedTorque_Control_CLIK,controller_interface::ControllerBase)
