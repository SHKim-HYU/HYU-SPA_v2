#pragma once


#define ROBOT_DOF 5


// Right ARM

#define BASE_Y 0.016
#define BASE_Z 0.908

#define LINK_12 0.05055 //m
#define LINK_23	0.18470
#define LINK_34 0.1153
#define LINK_45 0.1289
#define LINK_56 0.1555

//#define LINK_6e 11e-3

#define MASS_1 0.723  //kg
#define MASS_2 1.426
#define MASS_3 0.243
#define MASS_4 1.189
#define MASS_5 0.690

//#define MASS_6 0.0612

/*
#define MASS_MOTOR_1 0.34
#define MASS_MOTOR_2 0.34
#define MASS_MOTOR_3 0.26
#define MASS_MOTOR_4 0.21
#define MASS_MOTOR_5 0.114
#define MASS_MOTOR_6 0.114
*/

//kgm^2
#define J_Ixx_1 0.000514412
#define J_Ixy_1 0
#define J_Ixz_1 0
#define J_Iyy_1 0.000346530
#define J_Iyz_1 0
#define J_Izz_1 0.000557816

#define J_Ixx_2 0.003681825
#define J_Ixy_2 0
#define J_Ixz_2 0
#define J_Iyy_2 0.001048794
#define J_Iyz_2 0
#define J_Izz_2 0.004071229

#define J_Ixx_3 0.00001152581
#define J_Ixy_3 0
#define J_Ixz_3 0
#define J_Iyy_3 0.00002210614
#define J_Iyz_3 0
#define J_Izz_3 0.00001130552

#define J_Ixx_4 0.00324899669
#define J_Ixy_4 0
#define J_Ixz_4 0
#define J_Iyy_4 0.00054844144
#define J_Iyz_4 0
#define J_Izz_4 0.00324613078

#define J_Ixx_5 0.000454119
#define J_Ixy_5 0
#define J_Ixz_5 0
#define J_Iyy_5 0.00015170768
#define J_Iyz_5 0
#define J_Izz_5 0.00044727983

/*
#define J_Ixx_6 17.4509
#define J_Ixy_6 0
#define J_Ixz_6 0
#define J_Iyy_6 17.4509
#define J_Iyz_6 0
#define J_Izz_6 33.7129
*/

#define HARMONIC_120 120
#define HARMONIC_100 100
#define HARMONIC_50 50
#define ENC_2048 2048
#define ENC_1024 1024
#define ENC_1000 1000
#define ENC_512 512
#define ABS_ENC_19 524288

#define MAX_CURRENT_1 2.55
#define MAX_CURRENT_2 2.55
#define MAX_CURRENT_3 2.83

#define MAX_CURRENT_4 2.83
#define MAX_CURRENT_5 1.84

//#define MAX_CURRENT_6 0.56

#define TORQUE_CONST_1 0.183 //Nm/A
#define TORQUE_CONST_2 0.183
#define TORQUE_CONST_3 0.091

#define TORQUE_CONST_4 0.091
#define TORQUE_CONST_5 0.094

//#define TORQUE_CONST_6 0.1166

#define EFFICIENCY 90.0
