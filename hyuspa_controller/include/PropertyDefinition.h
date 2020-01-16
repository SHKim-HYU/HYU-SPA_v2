#pragma once


#define ROBOT_DOF 5


// Right ARM

#define BASE_Y -0.2305
#define BASE_Z 1.567

#define LINK_12 0.05 //m
#define LINK_23	0.18525
#define LINK_34 0.1168
#define LINK_45 0.1274
#define LINK_56 0.23004

//#define LINK_6e 11e-3

#define MASS_1 0.70151  //kg
#define MASS_2 1.42162
#define MASS_3 0.2435
#define MASS_4 1.1894
#define MASS_5 0.36826

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
#define J_Ixx_1 0.00050315737
#define J_Ixy_1 0.00000655128
#define J_Ixz_1 0.00000042443
#define J_Iyy_1 0.00031839677
#define J_Iyz_1 0.00000023010
#define J_Izz_1 0.00052445446

#define J_Ixx_2 0.00368181349
#define J_Ixy_2 -0.00018327606
#define J_Ixz_2 -0.00000268131
#define J_Iyy_2 0.00104405709
#define J_Iyz_2 0.00000334569
#define J_Izz_2 0.00406650369

#define J_Ixx_3 0.00068284
#define J_Ixy_3 -8.5183E-06
#define J_Ixz_3 -4.7514E-07
#define J_Iyy_3 0.00022011
#define J_Iyz_3 8.8649E-06
#define J_Izz_3 0.00063402

#define J_Ixx_4 0.00324613078
#define J_Ixy_4 -0.00000202519
#define J_Ixz_4 0.00000011493
#define J_Iyy_4 0.00054844144
#define J_Iyz_4 0.00023013877
#define J_Izz_4 0.00324899669

#define J_Ixx_5 0.0029078647
#define J_Ixy_5 0.00008047904
#define J_Ixz_5 -0.00001276791
#define J_Iyy_5 0.00028211059
#define J_Iyz_5 -0.00014451678
#define J_Izz_5 0.00306362830

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
