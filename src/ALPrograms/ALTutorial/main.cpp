#include "BasicFiles/BasicSetting.h"

//for QP
#include <QuadProg++.hh>
#include <js_qp.h>

//for rbdl
#include "rbdl/rbdl.h"

//for fcl
#include "test_fcl_utility.h"
#include "fcl_resources/config.h.in"
#include "fcl/narrowphase/distance.h"

using namespace fcl;
using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

#define PI			3.141592653589793

// Basic --------
pRBCORE_SHM             sharedData;
pUSER_SHM               userData;
JointControlClass       *jCon;


int     __IS_WORKING = false;
int     __IS_GAZEBO = false;
int     PODO_NO = -1;


enum TUTORIAL_COMMAND
{
    TUTORIAL_NO_ACT = 100,
    TUTORIAL_FINGER_CONTROL
};

enum JS_QP_COMMAND
{
    JS_NO_ACT = 500,
    JS_QP_START,
    JS_QP_STOP,
    JS_TEST,
    JS_INIT_POS,
    JS_TCP_CONNECT

};



//for QP, START
//value

unsigned int idx_save = 0;
unsigned int saveFlag;
float DataBuf[50][100000];
int js_rt_flag = 0;
int time_init_to_motion = 0;
JS_QP js_qp;

int n_dof = 31; // 6*4+6+1 = 31
int n_col = 18; //18;
MatrixNd qmin_hubo2 =   MatrixNd::Zero(n_dof,1);
MatrixNd qmax_hubo2 =   MatrixNd::Zero(n_dof,1);
MatrixNd dqmin_hubo2 =  MatrixNd::Zero(n_dof,1);
MatrixNd dqmax_hubo2 =  MatrixNd::Zero(n_dof,1);
MatrixNd ddqmin_hubo2 = MatrixNd::Zero(n_dof,1);
MatrixNd ddqmax_hubo2 = MatrixNd::Zero(n_dof,1);

MatrixNd l_hubo2 = MatrixNd::Zero(4,1);
MatrixNd q_hubo2 = MatrixNd::Zero(n_dof+1,1); // last term is quaternion
MatrixNd q_hubo2_init = MatrixNd::Zero(n_dof+1,1); // last term is quaternion
MatrixNd q_hubo2_filtered = MatrixNd::Zero(n_dof+1,1); // last term is quaternion

MatrixNd dq_hubo2 = MatrixNd::Zero(n_dof,1);

Math::Vector3d wp2ee = Math::Vector3d(0,0,-85);
Math::Vector3d ar2ee = Math::Vector3d(0,0,-94.5);

MatrixNd Rd_hubo2_WST = MatrixNd::Zero(3,3);
MatrixNd Rf_hubo2_WST = MatrixNd::Zero(3,3);

MatrixNd Rd_hubo2_PEL = MatrixNd::Zero(3,3);
MatrixNd Rf_hubo2_PEL = MatrixNd::Zero(3,3);

MatrixNd Rd_hubo2_RH = MatrixNd::Zero(3,3);
MatrixNd Td_hubo2_RH = MatrixNd::Zero(4,4);
MatrixNd Xd_hubo2_RH = MatrixNd::Zero(3,1);

MatrixNd Rd_hubo2_LH = MatrixNd::Zero(3,3);
MatrixNd Td_hubo2_LH = MatrixNd::Zero(4,4);
MatrixNd Xd_hubo2_LH = MatrixNd::Zero(3,1);

MatrixNd Rf_hubo2_RH = MatrixNd::Zero(3,3);
MatrixNd Tf_hubo2_RH = MatrixNd::Zero(4,4);
MatrixNd Xf_hubo2_RH = MatrixNd::Zero(3,1);

MatrixNd Rf_hubo2_LH = MatrixNd::Zero(3,3);
MatrixNd Tf_hubo2_LH = MatrixNd::Zero(4,4);
MatrixNd Xf_hubo2_LH = MatrixNd::Zero(3,1);


MatrixNd Rd_hubo2_RF = MatrixNd::Zero(3,3);
MatrixNd Td_hubo2_RF = MatrixNd::Zero(4,4);
MatrixNd Xd_hubo2_RF = MatrixNd::Zero(3,1);

MatrixNd Rd_hubo2_LF = MatrixNd::Zero(3,3);
MatrixNd Td_hubo2_LF = MatrixNd::Zero(4,4);
MatrixNd Xd_hubo2_LF = MatrixNd::Zero(3,1);

MatrixNd Rf_hubo2_RF = MatrixNd::Zero(3,3);
MatrixNd Tf_hubo2_RF = MatrixNd::Zero(4,4);
MatrixNd Xf_hubo2_RF = MatrixNd::Zero(3,1);

MatrixNd Rf_hubo2_LF = MatrixNd::Zero(3,3);
MatrixNd Tf_hubo2_LF = MatrixNd::Zero(4,4);
MatrixNd Xf_hubo2_LF = MatrixNd::Zero(3,1);

double m_robot = 0;
Math::Vector3d COM;


MatrixNd J_hubo2 = MatrixNd::Zero(6,n_dof);

double dt_hubo2 = 1/200.0;

MatrixNd l_da = MatrixNd::Zero(7,1);

MatrixNd q_da_right = MatrixNd::Zero(6,1);

MatrixNd Rf_da_right = MatrixNd::Zero(3,3);
MatrixNd Tf_da_right = MatrixNd::Zero(4,4);
MatrixNd Xf_da_right = MatrixNd::Zero(3,1);

MatrixNd q_da_left = MatrixNd::Zero(6,1);

MatrixNd Rf_da_left = MatrixNd::Zero(3,3);
MatrixNd Tf_da_left = MatrixNd::Zero(4,4);
MatrixNd Xf_da_left = MatrixNd::Zero(3,1);

Math::Vector3d pel2lhy      = Math::Vector3d(0,88.5,0);
Math::Vector3d pel2rhy      = Math::Vector3d(0,-88.5,0);
Math::Vector3d hp2kn        = Math::Vector3d(0,0,-280);
Math::Vector3d kn2ap        = Math::Vector3d(0,0,-278);
Math::Vector3d ap2ar        = Math::Vector3d(0,0,-94.5);
Math::Vector3d sy2eb        = Math::Vector3d(22.0,0.0,-182.0);
Math::Vector3d eb2wy        = Math::Vector3d(-22.0,0.0,-164.0);
Math::Vector3d pel2torso      = Math::Vector3d(0,0,172.5);
Math::Vector3d zv           = Math::Vector3d(0.0,0.0,0.0);
Math::Vector3d torso2lsp    = Math::Vector3d(0,215,195);
Math::Vector3d torso2rsp    = Math::Vector3d(0,-215,195);

MatrixNd err_M_WST = MatrixNd::Zero(3,3);
MatrixNd err_ori_WST = MatrixNd::Zero(3,1);
MatrixNd err_M_PEL = MatrixNd::Zero(3,3);
MatrixNd err_ori_PEL = MatrixNd::Zero(3,1);

MatrixNd err_M_RH = MatrixNd::Zero(3,3);
MatrixNd err_M_LH = MatrixNd::Zero(3,3);
MatrixNd err_M_RF = MatrixNd::Zero(3,3);
MatrixNd err_M_LF = MatrixNd::Zero(3,3);

MatrixNd err_ori_RH = MatrixNd::Zero(3,1);
MatrixNd err_pos_RH = MatrixNd::Zero(3,1);

MatrixNd err_ori_LH = MatrixNd::Zero(3,1);
MatrixNd err_pos_LH = MatrixNd::Zero(3,1);

MatrixNd err_ori_RF = MatrixNd::Zero(3,1);
MatrixNd err_pos_RF = MatrixNd::Zero(3,1);

MatrixNd err_ori_LF = MatrixNd::Zero(3,1);
MatrixNd err_pos_LF = MatrixNd::Zero(3,1);

MatrixNd err_pos_PELZ = MatrixNd::Zero(1,1);

MatrixNd dqlb_hubo2 = MatrixNd::Zero(n_dof,1);
MatrixNd dqub_hubo2 = MatrixNd::Zero(n_dof,1);
MatrixNd Aineq_ub = MatrixNd::Zero(n_dof,n_dof);
MatrixNd Bineq_ub = MatrixNd::Zero(n_dof,1);
MatrixNd Aineq_lb = MatrixNd::Zero(n_dof,n_dof);
MatrixNd Bineq_lb = MatrixNd::Zero(n_dof,1);

MatrixNd temp_jacob = MatrixNd::Zero(6,n_dof);

MatrixNd D = MatrixNd::Zero(n_col,1);
MatrixNd N = MatrixNd::Zero(n_col,n_dof);

double d1, d2, d3, d4, d5, d6, d7, d8, d9; //d1~3-> body with right arm, d4 -> right arm link1 and 3, d5~7-> body with left arm, d8 left arm link1 and 3, d9 right and left end-effector
double d10, d11, d12, d13, d14, d15, d16, d17, d18;

MatrixNd N1, N2, N3, N4, N5, N6, N7, N8, N9;
MatrixNd N10, N11, N12, N13, N14, N15, N16, N17, N18;

MatrixNd Aineq_a(n_dof*2+n_col,n_dof);
MatrixNd Bineq_a(n_dof*2+n_col,1);

MatrixNd A_behavior = MatrixNd::Zero(33,n_dof);
MatrixNd B_behavior = MatrixNd::Zero(33,1);

MatrixNd n = MatrixNd::Zero(3,1);

double dt = 1/200.0;
double dist_margin = 50;

MatrixNd body_center = MatrixNd::Zero(3,1);
MatrixNd body_rot = MatrixNd::Zero(3,3);
MatrixNd body_length = MatrixNd::Zero(3,1);

MatrixNd temp_link_center = MatrixNd::Zero(3,1);
MatrixNd temp_link_rot = MatrixNd::Zero(3,3);

double temp_link_length;
double temp_link_radi;

MatrixNd temp_joint_to_linkc = MatrixNd::Zero(3,1);

MatrixNd temp_link_center_ = MatrixNd::Zero(3,1);
MatrixNd temp_link_rot_ = MatrixNd::Zero(3,3);

double temp_link_length_;
double temp_link_radi_;

MatrixNd temp_joint_to_linkc_ = MatrixNd::Zero(3,1);

MatrixNd temp_cp1 = MatrixNd::Zero(3,1);
MatrixNd temp_cp2 = MatrixNd::Zero(3,1);

MatrixNd temp_J1 = MatrixNd::Zero(6,n_dof);
MatrixNd temp_J2 = MatrixNd::Zero(6,n_dof);

MatrixNd box_center1_local = MatrixNd::Zero(3,1);
MatrixNd box_center1_global = MatrixNd::Zero(3,1);
MatrixNd box_rot1 = MatrixNd::Zero(3,3);
MatrixNd box_length1 = MatrixNd::Zero(3,1);

MatrixNd box_center2_local = MatrixNd::Zero(3,1);
MatrixNd box_center2_global = MatrixNd::Zero(3,1);
MatrixNd box_rot2 = MatrixNd::Zero(3,3);
MatrixNd box_length2 = MatrixNd::Zero(3,1);


MatrixNd cylinder_center1_local = MatrixNd::Zero(3,1);
MatrixNd cylinder_center1_global = MatrixNd::Zero(3,1);

MatrixNd cylinder_rot1 = MatrixNd::Zero(3,3);
double cylinder_length1;
double cylinder_radi1;

MatrixNd cylinder_center2_local = MatrixNd::Zero(3,1);
MatrixNd cylinder_center2_global = MatrixNd::Zero(3,1);
MatrixNd cylinder_rot2 = MatrixNd::Zero(3,3);
double cylinder_length2;
double cylinder_radi2;

double temp_double;
MatrixNd desired_COM = MatrixNd::Zero(3,1);
int rt_count = 0;
int _rt_count = 0;
int count_end_phase = 0;

double t_start = 0;
double t1 = 0;
//function

void SaveFile(void)
{
    FILE* fp;
    unsigned int i, j;
    fp = fopen("/home/rainbow/Desktop/podo_hubo_new/exe/hubo2_telepresence_data.txt", "w");

    for(i=0 ; i<idx_save ; i++)
    {
        for(j=0 ; j<40 ; j++)
            fprintf(fp, "%f\t", DataBuf[j][i]);
        fprintf(fp, "\n");
    }
    fclose(fp);

    idx_save=0;
    saveFlag=0;
}


double dot_eigen(MatrixNd a, MatrixNd b)
{
    double c=0;

    if(a.rows() != b.rows())
        return c;

    for(int i=0;i<a.rows();i++)
    {
        c+= a(i,0)*b(i,0);
    }

    return c;
}

MatrixNd Rx(double th)
{
    MatrixNd _Rx(3,3);

    _Rx(0,0) = 1.0;
    _Rx(0,1) = 0.0;
    _Rx(0,2) = 0.0;

    _Rx(1,0) = 0.0;
    _Rx(1,1) = cos(th);
    _Rx(1,2) = -sin(th);

    _Rx(2,0) = 0.0;
    _Rx(2,1) = sin(th);
    _Rx(2,2) = cos(th);


    return _Rx;
}

MatrixNd Ry(double th)
{
    MatrixNd _Ry(3,3);

    _Ry(0,0) = cos(th);
    _Ry(0,1) = 0.0;
    _Ry(0,2) = sin(th);

    _Ry(1,0) = 0.0;
    _Ry(1,1) = 1.0;
    _Ry(1,2) = 0.0;

    _Ry(2,0) = -sin(th);
    _Ry(2,1) = 0.0;
    _Ry(2,2) = cos(th);


    return _Ry;
}

MatrixNd Rz(double th)
{
    MatrixNd _Rz(3,3);

    _Rz(0,0) = cos(th);
    _Rz(0,1) = -sin(th);
    _Rz(0,2) = 0.0;

    _Rz(1,0) = sin(th);
    _Rz(1,1) = cos(th);
    _Rz(1,2) = 0.0;

    _Rz(2,0) = 0.0;
    _Rz(2,1) = 0.0;
    _Rz(2,2) = 1.0;


    return _Rz;
}

MatrixNd kine_hubo2(MatrixNd q, MatrixNd l)
{
    MatrixNd T = MatrixNd::Zero(4,4);
    MatrixNd tempT = MatrixNd::Zero(4,4);
    MatrixNd temp_tran = MatrixNd::Zero(3,1);

    //joint RSP, joint 1
    tempT(3,3) = 1;

    tempT.block(0,0,3,3) = Ry(q(0,0));

    temp_tran(0,0) = 0;
    temp_tran(1,0) = 0;
    temp_tran(2,0) = 0;

    tempT.block(0,3,3,1) = temp_tran;

    T = MatrixNd::Identity(4,4) * tempT;

    //joint RSR, joint 2
    tempT.block(0,0,3,3) = Rx(q(1,0));

    temp_tran(0,0) = 0;
    temp_tran(1,0) = 0;
    temp_tran(2,0) = 0;

    tempT.block(0,3,3,1) = temp_tran;

    T = T * tempT;

    //joint RSY, joint 3
    tempT.block(0,0,3,3) = Rz(q(2,0));

    temp_tran(0,0) = 0;
    temp_tran(1,0) = 0;
    temp_tran(2,0) = 0;

    tempT.block(0,3,3,1) = temp_tran;

    T = T * tempT;

    //joint REB, joint 4
    tempT.block(0,0,3,3) = Ry(q(3,0));

    temp_tran(0,0) = l(3,0);
    temp_tran(1,0) = 0;
    temp_tran(2,0) = -l(0,0);

    tempT.block(0,3,3,1) = temp_tran;

    T = T * tempT;

    //joint RWY, joint 5
    tempT.block(0,0,3,3) = Rz(q(4,0));

    temp_tran(0,0) = -l(3,0);
    temp_tran(1,0) = 0;
    temp_tran(2,0) = -l(1,0);

    tempT.block(0,3,3,1) = temp_tran;

    T = T * tempT;

    //joint RWP, joint 6
    tempT.block(0,0,3,3) = Ry(q(5,0));

    temp_tran(0,0) = 0;
    temp_tran(1,0) = 0;
    temp_tran(2,0) = 0;

    tempT.block(0,3,3,1) = temp_tran;

    T = T * tempT;

    //joint End-Effector
    tempT.block(0,0,3,3) = MatrixNd::Identity(3,3);

    temp_tran(0,0) = 0;
    temp_tran(1,0) = 0;
    temp_tran(2,0) = -l(2,0);

    tempT.block(0,3,3,1) = temp_tran;

    T = T * tempT;


    return T;
}

MatrixNd Jacob_hubo2_arm(MatrixNd q, MatrixNd l)
{
    MatrixNd J = MatrixNd::Zero(6,6);

    double l_ua = l(0,0);
    double l_la = l(1,0);
    double l_tip = l(2,0);
    double l_offset = l(3,0);

    J(0,0) = (l_la)*(sin(q(3,0))*(cos(q(2,0))*sin(q(0,0)) - cos(q(0,0))*sin(q(1,0))*sin(q(2,0))) - cos(q(0,0))*cos(q(1,0))*cos(q(3,0))) +  (l_offset)*(cos(q(3,0))*(cos(q(2,0))*sin(q(0,0)) - cos(q(0,0))*sin(q(1,0))*sin(q(2,0))) + cos(q(0,0))*cos(q(1,0))*sin(q(3,0))) +  (l_tip)*(cos(q(5,0))*(sin(q(3,0))*(cos(q(2,0))*sin(q(0,0)) - cos(q(0,0))*sin(q(1,0))*sin(q(2,0))) - cos(q(0,0))*cos(q(1,0))*cos(q(3,0))) + sin(q(5,0))*(cos(q(4,0))*(cos(q(3,0))*(cos(q(2,0))*sin(q(0,0)) - cos(q(0,0))*sin(q(1,0))*sin(q(2,0))) + cos(q(0,0))*cos(q(1,0))*sin(q(3,0))) - sin(q(4,0))*(sin(q(0,0))*sin(q(2,0)) + cos(q(0,0))*cos(q(2,0))*sin(q(1,0))))) -  (l_offset)*(cos(q(2,0))*sin(q(0,0)) - cos(q(0,0))*sin(q(1,0))*sin(q(2,0))) -  (l_ua)*cos(q(0,0))*cos(q(1,0));
    J(0,1) = (l_la)*(cos(q(3,0))*sin(q(0,0))*sin(q(1,0)) - cos(q(1,0))*sin(q(0,0))*sin(q(2,0))*sin(q(3,0))) -  (l_offset)*(sin(q(0,0))*sin(q(1,0))*sin(q(3,0)) + cos(q(1,0))*cos(q(3,0))*sin(q(0,0))*sin(q(2,0))) +  (l_tip)*(cos(q(5,0))*(cos(q(3,0))*sin(q(0,0))*sin(q(1,0)) - cos(q(1,0))*sin(q(0,0))*sin(q(2,0))*sin(q(3,0))) - sin(q(5,0))*(cos(q(4,0))*(sin(q(0,0))*sin(q(1,0))*sin(q(3,0)) + cos(q(1,0))*cos(q(3,0))*sin(q(0,0))*sin(q(2,0))) + cos(q(1,0))*cos(q(2,0))*sin(q(0,0))*sin(q(4,0)))) +  (l_ua)*sin(q(0,0))*sin(q(1,0)) +  (l_offset)*cos(q(1,0))*sin(q(0,0))*sin(q(2,0));
    J(0,2) = (l_tip)*(sin(q(5,0))*(sin(q(4,0))*(cos(q(0,0))*cos(q(2,0)) + sin(q(0,0))*sin(q(1,0))*sin(q(2,0))) + cos(q(3,0))*cos(q(4,0))*(cos(q(0,0))*sin(q(2,0)) - cos(q(2,0))*sin(q(0,0))*sin(q(1,0)))) + cos(q(5,0))*sin(q(3,0))*(cos(q(0,0))*sin(q(2,0)) - cos(q(2,0))*sin(q(0,0))*sin(q(1,0)))) -  (l_offset)*(cos(q(0,0))*sin(q(2,0)) - cos(q(2,0))*sin(q(0,0))*sin(q(1,0))) +  (l_offset)*cos(q(3,0))*(cos(q(0,0))*sin(q(2,0)) - cos(q(2,0))*sin(q(0,0))*sin(q(1,0))) +  (l_la)*sin(q(3,0))*(cos(q(0,0))*sin(q(2,0)) - cos(q(2,0))*sin(q(0,0))*sin(q(1,0)));
    J(0,3) = (l_offset)*(sin(q(3,0))*(cos(q(0,0))*cos(q(2,0)) + sin(q(0,0))*sin(q(1,0))*sin(q(2,0))) + cos(q(1,0))*cos(q(3,0))*sin(q(0,0))) -  (l_la)*(cos(q(3,0))*(cos(q(0,0))*cos(q(2,0)) + sin(q(0,0))*sin(q(1,0))*sin(q(2,0))) - cos(q(1,0))*sin(q(0,0))*sin(q(3,0))) -  (l_tip)*(cos(q(5,0))*(cos(q(3,0))*(cos(q(0,0))*cos(q(2,0)) + sin(q(0,0))*sin(q(1,0))*sin(q(2,0))) - cos(q(1,0))*sin(q(0,0))*sin(q(3,0))) - cos(q(4,0))*sin(q(5,0))*(sin(q(3,0))*(cos(q(0,0))*cos(q(2,0)) + sin(q(0,0))*sin(q(1,0))*sin(q(2,0))) + cos(q(1,0))*cos(q(3,0))*sin(q(0,0))));
    J(0,4) = (l_tip)*sin(q(5,0))*(sin(q(4,0))*(cos(q(3,0))*(cos(q(0,0))*cos(q(2,0)) + sin(q(0,0))*sin(q(1,0))*sin(q(2,0))) - cos(q(1,0))*sin(q(0,0))*sin(q(3,0))) + cos(q(4,0))*(cos(q(0,0))*sin(q(2,0)) - cos(q(2,0))*sin(q(0,0))*sin(q(1,0))));
    J(0,5) = (l_tip)*(sin(q(5,0))*(sin(q(3,0))*(cos(q(0,0))*cos(q(2,0)) + sin(q(0,0))*sin(q(1,0))*sin(q(2,0))) + cos(q(1,0))*cos(q(3,0))*sin(q(0,0))) - cos(q(5,0))*(cos(q(4,0))*(cos(q(3,0))*(cos(q(0,0))*cos(q(2,0)) + sin(q(0,0))*sin(q(1,0))*sin(q(2,0))) - cos(q(1,0))*sin(q(0,0))*sin(q(3,0))) - sin(q(4,0))*(cos(q(0,0))*sin(q(2,0)) - cos(q(2,0))*sin(q(0,0))*sin(q(1,0)))));

    J(1,0) = 0;
    J(1,1) = (l_ua)*cos(q(1,0)) -  (l_tip)*(sin(q(5,0))*(cos(q(4,0))*(cos(q(1,0))*sin(q(3,0)) - cos(q(3,0))*sin(q(1,0))*sin(q(2,0))) - cos(q(2,0))*sin(q(1,0))*sin(q(4,0))) - cos(q(5,0))*(cos(q(1,0))*cos(q(3,0)) + sin(q(1,0))*sin(q(2,0))*sin(q(3,0)))) +  (l_la)*(cos(q(1,0))*cos(q(3,0)) + sin(q(1,0))*sin(q(2,0))*sin(q(3,0))) -  (l_offset)*(cos(q(1,0))*sin(q(3,0)) - cos(q(3,0))*sin(q(1,0))*sin(q(2,0))) -  (l_offset)*sin(q(1,0))*sin(q(2,0));
    J(1,2) = (l_tip)*(sin(q(5,0))*(cos(q(1,0))*sin(q(2,0))*sin(q(4,0)) - cos(q(1,0))*cos(q(2,0))*cos(q(3,0))*cos(q(4,0))) - cos(q(1,0))*cos(q(2,0))*cos(q(5,0))*sin(q(3,0))) +  (l_offset)*cos(q(1,0))*cos(q(2,0)) -  (l_offset)*cos(q(1,0))*cos(q(2,0))*cos(q(3,0)) -  (l_la)*cos(q(1,0))*cos(q(2,0))*sin(q(3,0));
    J(1,3) = -  (l_la)*(sin(q(1,0))*sin(q(3,0)) + cos(q(1,0))*cos(q(3,0))*sin(q(2,0))) -  (l_offset)*(cos(q(3,0))*sin(q(1,0)) - cos(q(1,0))*sin(q(2,0))*sin(q(3,0))) -  (l_tip)*(cos(q(5,0))*(sin(q(1,0))*sin(q(3,0)) + cos(q(1,0))*cos(q(3,0))*sin(q(2,0))) + cos(q(4,0))*sin(q(5,0))*(cos(q(3,0))*sin(q(1,0)) - cos(q(1,0))*sin(q(2,0))*sin(q(3,0))));
    J(1,4) = (l_tip)*sin(q(5,0))*(sin(q(4,0))*(sin(q(1,0))*sin(q(3,0)) + cos(q(1,0))*cos(q(3,0))*sin(q(2,0))) - cos(q(1,0))*cos(q(2,0))*cos(q(4,0)));
    J(1,5) = - (l_tip)*(cos(q(5,0))*(cos(q(4,0))*(sin(q(1,0))*sin(q(3,0)) + cos(q(1,0))*cos(q(3,0))*sin(q(2,0))) + cos(q(1,0))*cos(q(2,0))*sin(q(4,0))) + sin(q(5,0))*(cos(q(3,0))*sin(q(1,0)) - cos(q(1,0))*sin(q(2,0))*sin(q(3,0))));


    J(2,0) = (l_la)*(sin(q(3,0))*(cos(q(0,0))*cos(q(2,0)) + sin(q(0,0))*sin(q(1,0))*sin(q(2,0))) + cos(q(1,0))*cos(q(3,0))*sin(q(0,0))) +  (l_offset)*(cos(q(3,0))*(cos(q(0,0))*cos(q(2,0)) + sin(q(0,0))*sin(q(1,0))*sin(q(2,0))) - cos(q(1,0))*sin(q(0,0))*sin(q(3,0))) +  (l_tip)*(cos(q(5,0))*(sin(q(3,0))*(cos(q(0,0))*cos(q(2,0)) + sin(q(0,0))*sin(q(1,0))*sin(q(2,0))) + cos(q(1,0))*cos(q(3,0))*sin(q(0,0))) + sin(q(5,0))*(cos(q(4,0))*(cos(q(3,0))*(cos(q(0,0))*cos(q(2,0)) + sin(q(0,0))*sin(q(1,0))*sin(q(2,0))) - cos(q(1,0))*sin(q(0,0))*sin(q(3,0))) - sin(q(4,0))*(cos(q(0,0))*sin(q(2,0)) - cos(q(2,0))*sin(q(0,0))*sin(q(1,0))))) -  (l_offset)*(cos(q(0,0))*cos(q(2,0)) + sin(q(0,0))*sin(q(1,0))*sin(q(2,0))) +  (l_ua)*cos(q(1,0))*sin(q(0,0));
    J(2,1) = (l_la)*(cos(q(0,0))*cos(q(3,0))*sin(q(1,0)) - cos(q(0,0))*cos(q(1,0))*sin(q(2,0))*sin(q(3,0))) -  (l_offset)*(cos(q(0,0))*sin(q(1,0))*sin(q(3,0)) + cos(q(0,0))*cos(q(1,0))*cos(q(3,0))*sin(q(2,0))) +  (l_tip)*(cos(q(5,0))*(cos(q(0,0))*cos(q(3,0))*sin(q(1,0)) - cos(q(0,0))*cos(q(1,0))*sin(q(2,0))*sin(q(3,0))) - sin(q(5,0))*(cos(q(4,0))*(cos(q(0,0))*sin(q(1,0))*sin(q(3,0)) + cos(q(0,0))*cos(q(1,0))*cos(q(3,0))*sin(q(2,0))) + cos(q(0,0))*cos(q(1,0))*cos(q(2,0))*sin(q(4,0)))) +  (l_ua)*cos(q(0,0))*sin(q(1,0)) +  (l_offset)*cos(q(0,0))*cos(q(1,0))*sin(q(2,0));
    J(2,2) = (l_offset)*(sin(q(0,0))*sin(q(2,0)) + cos(q(0,0))*cos(q(2,0))*sin(q(1,0))) -  (l_tip)*(sin(q(5,0))*(sin(q(4,0))*(cos(q(2,0))*sin(q(0,0)) - cos(q(0,0))*sin(q(1,0))*sin(q(2,0))) + cos(q(3,0))*cos(q(4,0))*(sin(q(0,0))*sin(q(2,0)) + cos(q(0,0))*cos(q(2,0))*sin(q(1,0)))) + cos(q(5,0))*sin(q(3,0))*(sin(q(0,0))*sin(q(2,0)) + cos(q(0,0))*cos(q(2,0))*sin(q(1,0)))) -  (l_offset)*cos(q(3,0))*(sin(q(0,0))*sin(q(2,0)) + cos(q(0,0))*cos(q(2,0))*sin(q(1,0))) -  (l_la)*sin(q(3,0))*(sin(q(0,0))*sin(q(2,0)) + cos(q(0,0))*cos(q(2,0))*sin(q(1,0)));
    J(2,3) = (l_la)*(cos(q(3,0))*(cos(q(2,0))*sin(q(0,0)) - cos(q(0,0))*sin(q(1,0))*sin(q(2,0))) + cos(q(0,0))*cos(q(1,0))*sin(q(3,0))) -  (l_offset)*(sin(q(3,0))*(cos(q(2,0))*sin(q(0,0)) - cos(q(0,0))*sin(q(1,0))*sin(q(2,0))) - cos(q(0,0))*cos(q(1,0))*cos(q(3,0))) +  (l_tip)*(cos(q(5,0))*(cos(q(3,0))*(cos(q(2,0))*sin(q(0,0)) - cos(q(0,0))*sin(q(1,0))*sin(q(2,0))) + cos(q(0,0))*cos(q(1,0))*sin(q(3,0))) - cos(q(4,0))*sin(q(5,0))*(sin(q(3,0))*(cos(q(2,0))*sin(q(0,0)) - cos(q(0,0))*sin(q(1,0))*sin(q(2,0))) - cos(q(0,0))*cos(q(1,0))*cos(q(3,0))));
    J(2,4) = - (l_tip)*sin(q(5,0))*(sin(q(4,0))*(cos(q(3,0))*(cos(q(2,0))*sin(q(0,0)) - cos(q(0,0))*sin(q(1,0))*sin(q(2,0))) + cos(q(0,0))*cos(q(1,0))*sin(q(3,0))) + cos(q(4,0))*(sin(q(0,0))*sin(q(2,0)) + cos(q(0,0))*cos(q(2,0))*sin(q(1,0))));
    J(2,5) = - (l_tip)*(sin(q(5,0))*(sin(q(3,0))*(cos(q(2,0))*sin(q(0,0)) - cos(q(0,0))*sin(q(1,0))*sin(q(2,0))) - cos(q(0,0))*cos(q(1,0))*cos(q(3,0))) - cos(q(5,0))*(cos(q(4,0))*(cos(q(3,0))*(cos(q(2,0))*sin(q(0,0)) - cos(q(0,0))*sin(q(1,0))*sin(q(2,0))) + cos(q(0,0))*cos(q(1,0))*sin(q(3,0))) - sin(q(4,0))*(sin(q(0,0))*sin(q(2,0)) + cos(q(0,0))*cos(q(2,0))*sin(q(1,0)))));


    J(3,0) = 0;
    J(3,1) = cos(q(0,0));
    J(3,2) = cos(q(1,0))*sin(q(0,0));
    J(3,3) = cos(q(2,0))*sin(q(0,0))*sin(q(1,0)) - cos(q(0,0))*sin(q(2,0));
    J(3,4) = sin(q(3,0))*(cos(q(0,0))*cos(q(2,0)) + sin(q(0,0))*sin(q(1,0))*sin(q(2,0))) + cos(q(1,0))*cos(q(3,0))*sin(q(0,0));
    J(3,5) = - sin(q(4,0))*(cos(q(3,0))*(cos(q(0,0))*cos(q(2,0)) + sin(q(0,0))*sin(q(1,0))*sin(q(2,0))) - cos(q(1,0))*sin(q(0,0))*sin(q(3,0))) - cos(q(4,0))*(cos(q(0,0))*sin(q(2,0)) - cos(q(2,0))*sin(q(0,0))*sin(q(1,0)));


    J(4,0) = 1;
    J(4,1) = 0;
    J(4,2) = -sin(q(1,0));
    J(4,3) = cos(q(1,0))*cos(q(2,0));
    J(4,4) = cos(q(1,0))*sin(q(2,0))*sin(q(3,0)) - cos(q(3,0))*sin(q(1,0));
    J(4,5) = cos(q(1,0))*cos(q(2,0))*cos(q(4,0)) - sin(q(4,0))*(sin(q(1,0))*sin(q(3,0)) + cos(q(1,0))*cos(q(3,0))*sin(q(2,0)));


    J(5,0) = 0;
    J(5,1) = -sin(q(0,0));
    J(5,2) = cos(q(0,0))*cos(q(1,0));
    J(5,3) = sin(q(0,0))*sin(q(2,0)) + cos(q(0,0))*cos(q(2,0))*sin(q(1,0));
    J(5,4) = cos(q(0,0))*cos(q(1,0))*cos(q(3,0)) - sin(q(3,0))*(cos(q(2,0))*sin(q(0,0)) - cos(q(0,0))*sin(q(1,0))*sin(q(2,0)));
    J(5,5) = sin(q(4,0))*(cos(q(3,0))*(cos(q(2,0))*sin(q(0,0)) - cos(q(0,0))*sin(q(1,0))*sin(q(2,0))) + cos(q(0,0))*cos(q(1,0))*sin(q(3,0))) + cos(q(4,0))*(sin(q(0,0))*sin(q(2,0)) + cos(q(0,0))*cos(q(2,0))*sin(q(1,0)));


    return J;
}

void generate_jointlimit_hubo2(MatrixNd q, MatrixNd qmin, MatrixNd qmax, MatrixNd dqmax, MatrixNd ddqmax, MatrixNd &lb, MatrixNd &ub)
{
    MatrixNd tempM = MatrixNd::Zero(3,1);
    double dt = 1/200.0;
    double temp_min, temp_max;

    for(int i=0;i<n_dof;i++)
    {
        if(i>5 || i<3)
        {
            tempM(0,0) = dqmax(i,0);
            tempM(1,0) = (qmax(i,0) - q(i,0))/dt;
            tempM(2,0) = sqrt(2*ddqmax(i,0)*(qmax(i,0) - q(i,0)));

            temp_min = tempM.minCoeff();

            tempM(0,0) = -dqmax(i,0);
            tempM(1,0) = (qmin(i,0) - q(i,0))/dt;
            tempM(2,0) = -sqrt(2*ddqmax(i,0)*(q(i,0) - qmin(i,0)));

            temp_max = tempM.maxCoeff();

            ub(i,0) = temp_min;
            lb(i,0) = temp_max;
        }
        else
        {
            ub(i,0) = PI/16.0;
            lb(i,0) = -PI/16.0;
        }
    }
}

double find_cloest_points_box_cylinder(MatrixNd box_center, MatrixNd box_rot, double box_x, double box_y, double box_z, MatrixNd cylinder_center, MatrixNd cylinder_rot, double cylinder_z, double cylinder_r, MatrixNd &cp_box, MatrixNd &cp_cylinder)
{
    DistanceResult<double> result;
    double dist;

    fcl::Matrix3d rotCylinder;
    fcl::Vector3d trCylinder;
    fcl::Matrix3d rotBox;
    fcl::Vector3d trBox;
    using CollisionGeometryPtr_t = shared_ptr<CollisionGeometry<double>>;
    CollisionGeometryPtr_t cgeomCylinder_ (new Cylinder<double> (cylinder_r,cylinder_z));

    CollisionGeometryPtr_t cgeomBox_ (new Box<double> (box_x,box_y,box_z));



    rotCylinder = cylinder_rot;
    trCylinder = cylinder_center;

    rotBox = box_rot;
    trBox= box_center;

    CollisionObject<double> cylinder (cgeomCylinder_, rotCylinder, trCylinder);
    CollisionObject<double> box (cgeomBox_, rotBox, trBox);
    //    //cout << "cgecgeomBox_omCylinder_ : " << cgeomCylinder_ << endl;
    //    //cout << "cgeomBox_ : " << cgeomBox_ << endl;

    result.clear();

    DistanceRequest<double> request(true);

    request.gjk_solver_type = fcl::GJKSolverType::GST_INDEP;
    request.distance_tolerance = 1e-6;

    dist = fcl::distance(&cylinder, &box, request, result) ;

    cp_cylinder = result.nearest_points[0];
    cp_box = result.nearest_points[1];

    //  //cout << "distance: " << dist << endl;
    //  //cout << "cp_box: " << rotBox*cp_box + box_center << endl;
    //  //cout << "cp_cylinder: " << rotCylinder*cp_cylinder + cylinder_center << endl;


    return dist;

}

double find_cloest_points_cylinder_cylinder(MatrixNd cylinder_center1, MatrixNd cylinder_rot1, double cylinder_z1, double cylinder_r1, MatrixNd cylinder_center2, MatrixNd cylinder_rot2, double cylinder_z2, double cylinder_r2, MatrixNd &cp_cylinder1, MatrixNd &cp_cylinder2)
{
    DistanceResult<double> result;
    double dist;

    fcl::Matrix3d rotCylinder1;
    fcl::Vector3d trCylinder1;
    fcl::Matrix3d rotCylinder2;
    fcl::Vector3d trCylinder2;

    using CollisionGeometryPtr_t = shared_ptr<CollisionGeometry<double>>;
    CollisionGeometryPtr_t cgeomCylinder_1 (new Cylinder<double> (cylinder_r1,cylinder_z1));

    CollisionGeometryPtr_t cgeomCylinder_2 (new Cylinder<double> (cylinder_r2,cylinder_z2));



    rotCylinder1 = cylinder_rot1;
    trCylinder1 = cylinder_center1;

    rotCylinder2 = cylinder_rot2;
    trCylinder2 = cylinder_center2;



    CollisionObject<double> cylinder1 (cgeomCylinder_1, rotCylinder1, trCylinder1);
    CollisionObject<double> cylinder2 (cgeomCylinder_2, rotCylinder2, trCylinder2);




    result.clear();

    DistanceRequest<double> request(true);

    request.gjk_solver_type = fcl::GJKSolverType::GST_INDEP;
    request.distance_tolerance = 1e-6;

    dist = fcl::distance(&cylinder1, &cylinder2, request, result);

    cp_cylinder1 = result.nearest_points[0];
    cp_cylinder2 = result.nearest_points[1];

    //    //cout << "distance: " << dist << endl;
    //    //cout << "cp_cylinder1 : " << rotCylinder1*cp_cylinder1 + cylinder_center1 << endl;
    //    //cout << "cp_cylinder2 : " << rotCylinder2*cp_cylinder2 + cylinder_center2 << endl;



    return dist;
    //    return;


}

double find_cloest_points_box_box(MatrixNd box_center1, MatrixNd box_rot1, double box_x1, double box_y1, double box_z1, MatrixNd box_center2, MatrixNd box_rot2, double box_x2, double box_y2, double box_z2,  MatrixNd &cp_box1, MatrixNd &cp_box2)
{
    DistanceResult<double> result;
    double dist;

    fcl::Matrix3d rotbox1;
    fcl::Vector3d trbox1;
    fcl::Matrix3d rotbox2;
    fcl::Vector3d trbox2;

    using CollisionGeometryPtr_t = shared_ptr<CollisionGeometry<double>>;
    CollisionGeometryPtr_t cgeombox_1 (new Box<double> (box_x1,box_y1,box_z1));

    CollisionGeometryPtr_t cgeombox_2 (new Box<double> (box_x2,box_y2,box_z2));

    rotbox1 = box_rot1;
    trbox1 = box_center1;

    rotbox2 = box_rot2;
    trbox2 = box_center2;

    CollisionObject<double> box1 (cgeombox_1, rotbox1, trbox1);
    CollisionObject<double> box2 (cgeombox_2, rotbox2, trbox2);

    result.clear();

    DistanceRequest<double> request(true);

    request.gjk_solver_type = fcl::GJKSolverType::GST_INDEP;
    request.distance_tolerance = 1e-6;

    dist = fcl::distance(&box1, &box2, request, result);

    cp_box1 = result.nearest_points[0];
    cp_box2 = result.nearest_points[1];

    //    //cout << "distance: " << dist << endl;
    //    //cout << "cp_box1 : " << rotbox1*cp_box1 + box_center1 << endl;
    //    //cout << "cp_box2 : " << rotbox2*cp_box2 + box_center2 << endl;



    return dist;
    //    return;

}


void find_closest_points_line_to_line(MatrixNd P0, MatrixNd P1, MatrixNd Q0, MatrixNd Q1, MatrixNd &Pc, MatrixNd &Qc)
{

    MatrixNd u;
    MatrixNd v;
    MatrixNd W0;

    u = (P1 - P0);
    v = (Q1 - Q0);

    W0 = P0 - Q0;

    double a = dot_eigen(u,u);
    double b = dot_eigen(u,v);
    double c = dot_eigen(v,v);
    double d = dot_eigen(u,W0);
    double e = dot_eigen(v,W0);

    if(a*c - b*b == 0)
    {
        Qc = Q0;
        Pc = P0;
        return;
    }

    double uc = (b*e - c*d) / (a*c - b*b);
    double vc = (a*e - b*d) / (a*c - b*b);


    if(uc > 1)
        uc = 1;
    else if(uc < 0)
        uc = 0;

    if(vc > 1)
        vc =1;
    else if(vc < 0)
        vc = 0;

    Pc = P0 + u*uc;
    Qc = Q0 + v*vc;

}

MatrixNd kine_data_arm_right(MatrixNd q, MatrixNd l)
{
    MatrixNd T = MatrixNd::Zero(4,4);
    MatrixNd tempT = MatrixNd::Zero(4,4);
    MatrixNd temp_tran = MatrixNd::Zero(3,1);

    //joint 1
    tempT(3,3) = 1;

    tempT.block(0,0,3,3) = Rz(q(0,0));

    temp_tran(0,0) = 0;
    temp_tran(1,0) = 0;
    temp_tran(2,0) = 0;

    tempT.block(0,3,3,1) = temp_tran;

    T = MatrixNd::Identity(4,4) * tempT;

    //joint 2
    tempT.block(0,0,3,3) = Ry(q(1,0));

    temp_tran(0,0) = 0;
    temp_tran(1,0) = -l(0,0);
    temp_tran(2,0) = 0;

    tempT.block(0,3,3,1) = temp_tran;

    T = T * tempT;

    //joint 3

    tempT.block(0,0,3,3) = Ry(q(2,0));

    temp_tran(0,0) = 0;
    temp_tran(1,0) = 0;
    temp_tran(2,0) = l(1,0);

    tempT.block(0,3,3,1) = temp_tran;

    T = T * tempT;

    //joint 4

    tempT.block(0,0,3,3) = Rz(q(3,0));

    temp_tran(0,0) = 0;
    temp_tran(1,0) = -l(2,0);
    temp_tran(2,0) = 0;

    tempT.block(0,3,3,1) = temp_tran;

    T = T * tempT;

    //joint 5
    tempT.block(0,0,3,3) = Ry(q(4,0));

    temp_tran(0,0) = 0;
    temp_tran(1,0) = 0;
    temp_tran(2,0) = -l(3,0);

    tempT.block(0,3,3,1) = temp_tran;

    T = T * tempT;


    //joint 6
    tempT.block(0,0,3,3) = Rz(q(5,0));

    temp_tran(0,0) = 0;
    temp_tran(1,0) = l(4,0);
    temp_tran(2,0) = 0;

    tempT.block(0,3,3,1) = temp_tran;

    T = T * tempT;

    //handller
    tempT.block(0,0,3,3) = MatrixNd::Identity(3,3);

    temp_tran(0,0) = -l(5,0);
    temp_tran(1,0) = 0;
    temp_tran(2,0) = -l(6,0);

    tempT.block(0,3,3,1) = temp_tran;

    T = T * tempT;

    //adjust orientation
    tempT = T;
    tempT.block(0,0,3,3) = T.block(0,0,3,3)*Ry(-PI/2.0);
    T = tempT;

    return T;


}


MatrixNd kine_data_arm_left(MatrixNd q, MatrixNd l)
{
    MatrixNd T = MatrixNd::Zero(4,4);
    MatrixNd tempT = MatrixNd::Zero(4,4);
    MatrixNd temp_tran = MatrixNd::Zero(3,1);

    //joint 1
    tempT(3,3) = 1;

    tempT.block(0,0,3,3) = Rz(q(0,0));

    temp_tran(0,0) = 0;
    temp_tran(1,0) = 0;
    temp_tran(2,0) = 0;

    tempT.block(0,3,3,1) = temp_tran;

    T = MatrixNd::Identity(4,4) * tempT;

    //joint 2
    tempT.block(0,0,3,3) = Ry(q(1,0));

    temp_tran(0,0) = 0;
    temp_tran(1,0) = l(0,0);
    temp_tran(2,0) = 0;

    tempT.block(0,3,3,1) = temp_tran;

    T = T * tempT;

    //joint 3

    tempT.block(0,0,3,3) = Ry(q(2,0));

    temp_tran(0,0) = 0;
    temp_tran(1,0) = 0;
    temp_tran(2,0) = l(1,0);

    tempT.block(0,3,3,1) = temp_tran;

    T = T * tempT;

    //joint 4

    tempT.block(0,0,3,3) = Rz(q(3,0));

    temp_tran(0,0) = 0;
    temp_tran(1,0) = l(2,0);
    temp_tran(2,0) = 0;

    tempT.block(0,3,3,1) = temp_tran;

    T = T * tempT;

    //joint 5
    tempT.block(0,0,3,3) = Ry(q(4,0));

    temp_tran(0,0) = 0;
    temp_tran(1,0) = 0;
    temp_tran(2,0) = -l(3,0);

    tempT.block(0,3,3,1) = temp_tran;

    T = T * tempT;


    //joint 6
    tempT.block(0,0,3,3) = Rz(q(5,0));

    temp_tran(0,0) = 0;
    temp_tran(1,0) = -l(4,0);
    temp_tran(2,0) = 0;

    tempT.block(0,3,3,1) = temp_tran;

    T = T * tempT;

    //handller
    tempT.block(0,0,3,3) = MatrixNd::Identity(3,3);

    temp_tran(0,0) = -l(5,0);
    temp_tran(1,0) = 0;
    temp_tran(2,0) = -l(6,0);

    tempT.block(0,3,3,1) = temp_tran;

    T = T * tempT;

    //adjust orientation
    tempT = T;
    tempT.block(0,0,3,3) = T.block(0,0,3,3)*Ry(-PI/2.0);
    T = tempT;

    return T;


}

Math::Quaternion update_quaternion(Math::Quaternion _q, MatrixNd w)
{


    double FOG_GYRO_X_RAD = w(0,0);
    double FOG_GYRO_Y_RAD = w(1,0);
    double FOG_GYRO_Z_RAD = w(2,0);

    double Qdel[4];
    double Qcur[4];
    double Qprev[4];

    Qprev[0] = _q[3];
    Qprev[1] = _q[0];
    Qprev[2] = _q[1];
    Qprev[3] = _q[2];

    double dt = 1.0/200.0;
    double ftemp0 = sqrt(FOG_GYRO_X_RAD*FOG_GYRO_X_RAD + FOG_GYRO_Y_RAD*FOG_GYRO_Y_RAD + FOG_GYRO_Z_RAD*FOG_GYRO_Z_RAD);

    if(fabs(ftemp0) < 1e-12)
        ftemp0 = 1e-12;

    double ftemp1 = cos(dt*ftemp0/2.);
    double ftemp2 = sin(dt*ftemp0/2.)/ftemp0;

    Qdel[0] = ftemp1;
    Qdel[1] = ftemp2*FOG_GYRO_X_RAD;
    Qdel[2] = ftemp2*FOG_GYRO_Y_RAD;
    Qdel[3] = ftemp2*FOG_GYRO_Z_RAD;

    //    Qcur[0] = Qprev[0]*Qdel[0] - Qprev[1]*Qdel[1] - Qprev[2]*Qdel[2] - Qprev[3]*Qdel[3];
    //    Qcur[1] = Qprev[0]*Qdel[1] + Qprev[1]*Qdel[0] + Qprev[2]*Qdel[3] - Qprev[3]*Qdel[2];
    //    Qcur[2] = Qprev[0]*Qdel[2] - Qprev[1]*Qdel[3] + Qprev[2]*Qdel[0] + Qprev[3]*Qdel[1];
    //    Qcur[3] = Qprev[0]*Qdel[3] + Qprev[1]*Qdel[2] - Qprev[2]*Qdel[1] + Qprev[3]*Qdel[0];

    Qcur[0] = Qdel[0]*Qprev[0] - Qdel[1]*Qprev[1] - Qdel[2]*Qprev[2] - Qdel[3]*Qprev[3];
    Qcur[1] = Qdel[0]*Qprev[1] + Qdel[1]*Qprev[0] + Qdel[2]*Qprev[3] - Qdel[3]*Qprev[2];
    Qcur[2] = Qdel[0]*Qprev[2] - Qdel[1]*Qprev[3] + Qdel[2]*Qprev[0] + Qdel[3]*Qprev[1];
    Qcur[3] = Qdel[0]*Qprev[3] + Qdel[1]*Qprev[2] - Qdel[2]*Qprev[1] + Qdel[3]*Qprev[0];

    Math::Quaternion q_cur = Math::Quaternion(Qcur[1],Qcur[2],Qcur[3],Qcur[0]);

    return q_cur;
}
//for QP, END




//for RBDL, START
JS_RBDL js_rbdl;


//for RBDL, END

//void FingerControl(char right_left, char finger, char current);

int main(int argc, char *argv[])
{
    // Termination signal ---------------------------------
    signal(SIGTERM, CatchSignals);   // "kill" from shell
    signal(SIGINT,  CatchSignals);    // Ctrl-c
    signal(SIGHUP,  CatchSignals);    // shell termination
    signal(SIGKILL, CatchSignals);
    signal(SIGSEGV, CatchSignals);

    // Block memory swapping ------------------------------
    mlockall(MCL_CURRENT|MCL_FUTURE);

    // Setting the AL Name <-- Must be a unique name!!
    sprintf(__AL_NAME, "ALTutorial");


    CheckArguments(argc, argv);
    if(PODO_NO == -1){
        FILE_LOG(logERROR) << "Please check the AL Number";
        FILE_LOG(logERROR) << "Terminate this AL..";
        return 0;
    }

    // Initialize RBCore -----------------------------------
    if(RBInitialize() == false)
        __IS_WORKING = false;

    jCon->SetMotionOwner(0);
    while(__IS_WORKING){
        usleep(100*1000);

        switch(sharedData->COMMAND[PODO_NO].USER_COMMAND){

        case JS_TCP_CONNECT:
        {

            FILE_LOG(logSUCCESS) << "Command JS_TCP_CONNECT received..";


            sharedData->COMMAND[PODO_NO].USER_COMMAND = JS_NO_ACT;
            break;

        }
        case JS_TEST:
        {

            FILE_LOG(logSUCCESS) << "Command JS_TEST received..";
            jCon->RefreshToCurrentReference();
            jCon->SetAllMotionOwner();


            sharedData->COMMAND[PODO_NO].USER_COMMAND = JS_NO_ACT;
            break;

        }

        case JS_INIT_POS:
        {
            FILE_LOG(logSUCCESS) << "Command JS_INIT_POS received..";

            jCon->RefreshToCurrentReference();
            jCon->SetAllMotionOwner();

            js_rt_flag = 0;

            usleep(1000*10);

            q_hubo2 <<  0,0,0,0,0,0,
                    0,
                    20, -10, 0, -90, 0, -10,
                    20, 10, 0, -90, 0, -10,
                    0,-1.446,-38.344,70.239,-31.925,1.446,
                    0,1.446,-38.344,70.239,-31.925,-1.446,
                    0;

            q_hubo2 *= PI/180.0;

            jCon->SetMoveJoint(WST, q_hubo2(6,0)*180.0/PI, 2000.0, MOVE_ABSOLUTE);

            jCon->SetMoveJoint(RSP, q_hubo2(7,0)*180.0/PI, 2000.0, MOVE_ABSOLUTE);
            jCon->SetMoveJoint(RSR, q_hubo2(8,0)*180.0/PI, 2000.0, MOVE_ABSOLUTE);
            jCon->SetMoveJoint(RSY, q_hubo2(9,0)*180.0/PI, 2000.0, MOVE_ABSOLUTE);
            jCon->SetMoveJoint(REB, q_hubo2(10,0)*180.0/PI, 2000.0, MOVE_ABSOLUTE);
            jCon->SetMoveJoint(RWY, q_hubo2(11,0)*180.0/PI, 2000.0, MOVE_ABSOLUTE);
            jCon->SetMoveJoint(RWP, q_hubo2(12,0)*180.0/PI, 2000.0, MOVE_ABSOLUTE);

            jCon->SetMoveJoint(LSP, q_hubo2(13,0)*180.0/PI, 2000.0, MOVE_ABSOLUTE);
            jCon->SetMoveJoint(LSR, q_hubo2(14,0)*180.0/PI, 2000.0, MOVE_ABSOLUTE);
            jCon->SetMoveJoint(LSY, q_hubo2(15,0)*180.0/PI, 2000.0, MOVE_ABSOLUTE);
            jCon->SetMoveJoint(LEB, q_hubo2(16,0)*180.0/PI, 2000.0, MOVE_ABSOLUTE);
            jCon->SetMoveJoint(LWY, q_hubo2(17,0)*180.0/PI, 2000.0, MOVE_ABSOLUTE);
            jCon->SetMoveJoint(LWP, q_hubo2(18,0)*180.0/PI, 2000.0, MOVE_ABSOLUTE);

            jCon->SetMoveJoint(RHY, q_hubo2(19,0)*180.0/PI, 2000.0, MOVE_ABSOLUTE);
            jCon->SetMoveJoint(RHR, q_hubo2(20,0)*180.0/PI, 2000.0, MOVE_ABSOLUTE);
            jCon->SetMoveJoint(RHP, q_hubo2(21,0)*180.0/PI, 2000.0, MOVE_ABSOLUTE);
            jCon->SetMoveJoint(RKN, q_hubo2(22,0)*180.0/PI, 2000.0, MOVE_ABSOLUTE);
            jCon->SetMoveJoint(RAP, q_hubo2(23,0)*180.0/PI, 2000.0, MOVE_ABSOLUTE);
            jCon->SetMoveJoint(RAR, q_hubo2(24,0)*180.0/PI, 2000.0, MOVE_ABSOLUTE);

            jCon->SetMoveJoint(LHY, q_hubo2(25,0)*180.0/PI, 2000.0, MOVE_ABSOLUTE);
            jCon->SetMoveJoint(LHR, q_hubo2(26,0)*180.0/PI, 2000.0, MOVE_ABSOLUTE);
            jCon->SetMoveJoint(LHP, q_hubo2(27,0)*180.0/PI, 2000.0, MOVE_ABSOLUTE);
            jCon->SetMoveJoint(LKN, q_hubo2(28,0)*180.0/PI, 2000.0, MOVE_ABSOLUTE);
            jCon->SetMoveJoint(LAP, q_hubo2(29,0)*180.0/PI, 2000.0, MOVE_ABSOLUTE);
            jCon->SetMoveJoint(LAR, q_hubo2(30,0)*180.0/PI, 2000.0, MOVE_ABSOLUTE);


            for(int i=0;i<25;i++)
            {
                sharedData->q[i] = q_hubo2(i+6,0);
            }

            sharedData->q[25] = 0;
            sharedData->q[26] = 0;


            sharedData->COMMAND[PODO_NO].USER_COMMAND = JS_NO_ACT;
            break;

        }

        case JS_QP_START:
        {

            FILE_LOG(logSUCCESS) << "Command JS_QP_START received..";

            jCon->RefreshToCurrentReference();
            jCon->SetAllMotionOwner();

            l_da << 85.0, 535.0, 50.0, 465.0, 65.0, 60, 150.0;
            l_hubo2 << 182,164,85,22;

            //            sharedData->ENCODER[MC_GetID(RSP)][MC_GetCH(RSP)].CurrentReference;

            qmin_hubo2 <<   0,0,0,0,0,0,
                    -30,
                    -100,-170,-85,-135,-85,-80,
                    -100,2,-85,-135,-85,-80,
                    -30,-40,-70,5,-70,-20,
                    -30,-40,-70,5,-70,-20;

            qmin_hubo2 *= PI/180.0;

            qmin_hubo2(0,0) = -200;
            qmin_hubo2(1,0) = -200;
            qmin_hubo2(2,0) = -200;


            qmax_hubo2 <<   0,0,0,0,0,0,
                    30,                     //WST
                    80,-2,85,-20,85,40,     //RH
                    80,170,85,-20,85,40,     //LH
                    30,40,70,145,70,20,     //RF
                    30,40,70,145,70,20;    //LF // for avoid singularity, elbow have to be lower thatn q_eb < PI - atan(l_ua/l_offset) - atan(l_la/l_offset), in this case, it is 14.54

            qmax_hubo2 *= PI/180.0;

            qmax_hubo2(0,0) = 200;
            qmax_hubo2(1,0) = 200;
            qmax_hubo2(2,0) = 200;


            dqmax_hubo2 <<  0,0,0,0,0,0,
                    2.4673/4.0,
                    2.4673, 2.4673, 2.4673, 2.4673, 2.4673, 2.4673,
                    2.4673, 2.4673, 2.4673, 2.4673, 2.4673, 2.4673,
                    2.4673, 2.4673, 2.4673, 2.4673, 2.4673, 2.4673,
                    2.4673, 2.4673, 2.4673, 2.4673, 2.4673, 2.4673;

            dqmax_hubo2(0,0) = 50;
            dqmax_hubo2(1,0) = 50;
            dqmax_hubo2(2,0) = 50;

            ddqmax_hubo2 << 0,0,0,0,0,0,
                    7.7505/4.0,
                    7.7505, 7.7505, 7.7505, 7.7505, 7.7505, 7.7505,
                    7.7505, 7.7505, 7.7505, 7.7505, 7.7505, 7.7505,
                    7.7505, 7.7505, 7.7505, 7.7505, 7.7505, 7.7505,
                    7.7505, 7.7505, 7.7505, 7.7505, 7.7505, 7.7505;

            ddqmax_hubo2 *= 1/4.0;

            ddqmax_hubo2(0,0) = 50;
            ddqmax_hubo2(1,0) = 50;
            ddqmax_hubo2(2,0) = 50;

            //            sharedData->ENCODER[MC_GetID(RSP)][MC_GetCH(RSP)].CurrentReference

            q_hubo2 <<  0,0,0,0,0,0,
                    0,
                    20, -10, 0, -90, 0, -10,
                    20, 10, 0, -90, 0, -10,
                    0,-1.371,-33.084,60.836,-27.782,1.371,
                    0,1.371,-33.084,60.836,-27.782,-1.371,
                    0;

            q_hubo2_init(6,0) = sharedData->ENCODER[MC_GetID(WST)][MC_GetCH(WST)].CurrentReference;
            int idx_joint = 7;
            for(int i=RSP;i<RSP+6;i++)
                q_hubo2_init(idx_joint++,0) = sharedData->ENCODER[MC_GetID(i)][MC_GetCH(i)].CurrentReference;

            idx_joint = 13;

            for(int i=LSP;i<LSP+6;i++)
                q_hubo2_init(idx_joint++,0) = sharedData->ENCODER[MC_GetID(i)][MC_GetCH(i)].CurrentReference;

            idx_joint = 19;

            for(int i=RHY;i<RHY+6;i++)
                q_hubo2_init(idx_joint++,0) = sharedData->ENCODER[MC_GetID(i)][MC_GetCH(i)].CurrentReference;

            idx_joint = 25;

            for(int i=LHY;i<LHY+6;i++)
                q_hubo2_init(idx_joint++,0) = sharedData->ENCODER[MC_GetID(i)][MC_GetCH(i)].CurrentReference;


            q_hubo2_init(qp_RSR) += -15.0;
            q_hubo2_init(qp_LSR) += 15.0;
            q_hubo2_init(qp_REB) += -20.0;
            q_hubo2_init(qp_LEB) += -20.0;

//            //cout << "q_hubo2_init" << endl << q_hubo2_init << endl;
            q_hubo2_init *= PI/180.0;
            q_hubo2 = q_hubo2_init;
            q_hubo2_filtered = q_hubo2_init;
//            q_hubo2 *= PI/180.0;

            Math::Vector3d rot_vec(0,1,0);
            Math::Quaternion qPel = Math::Quaternion::fromAxisAngle(rot_vec,-0/16);
            Math::VectorNd tempq = q_hubo2;
            js_rbdl.Robot->SetQuaternion(js_rbdl.n_pel,qPel,tempq);
            q_hubo2 = tempq;

            q_hubo2(0,0) = 0;
            q_hubo2(1,0) = 0;
            q_hubo2(2,0) = 0;

            idx_save = 0;
            time_init_to_motion = 0;
            rt_count = 0;
            _rt_count = 0;

            js_rt_flag = 2;
            sharedData->COMMAND[PODO_NO].USER_COMMAND = JS_NO_ACT;
            break;

        }
        case JS_QP_STOP:
        {
            FILE_LOG(logSUCCESS) << "Command JS_QP_STOP received..";
//            //cout << "am i here? " << endl;

            jCon->RefreshToCurrentReference();
            jCon->SetAllMotionOwner();

            js_rt_flag = 0;
            SaveFile();

            sharedData->COMMAND[PODO_NO].USER_COMMAND = JS_NO_ACT;
            break;

        }


        case 999:
            FILE_LOG(logSUCCESS) << "Command 999 received..";
            jCon->RefreshToCurrentReference();
            jCon->SetAllMotionOwner();
            jCon->SetMoveJoint(WST, 30.0, 2000.0, MOVE_ABSOLUTE);
            sharedData->COMMAND[PODO_NO].USER_COMMAND = NO_ACT;
            break;
        case TUTORIAL_FINGER_CONTROL:
            FILE_LOG(logSUCCESS) << "Command TUTORIAL_FINGER_CONTROL received..";
            //            FingerControl(sharedData->COMMAND[PODO_NO].USER_PARA_CHAR[0],
            //                    sharedData->COMMAND[PODO_NO].USER_PARA_CHAR[1],
            //                    sharedData->COMMAND[PODO_NO].USER_PARA_CHAR[2]);
            sharedData->COMMAND[PODO_NO].USER_COMMAND = TUTORIAL_NO_ACT;
            break;
        default:
            break;
        }
    }

    FILE_LOG(logERROR) << "Process \"" << __AL_NAME << "\" is terminated" << endl;
    return 0;
}




//==============================//
// Task Thread
//==============================//


void RBTaskThread(void *)
{
    while(__IS_WORKING)
    {

        if(js_rt_flag == 2)
        {
            t_start = rt_timer_read();

            //cout << "rt start" << endl;

            for(int i=0;i<6;i++)
            {
                q_da_right(i,0) = sharedData->data_armR[6-i].cur_angle*PI/180.0;
                q_da_left(i,0) = sharedData->data_armL[6-i].cur_angle*PI/180.0;
            }

            Tf_da_right = kine_data_arm_right(q_da_right,l_da);
            Rf_da_right = Tf_da_right.block(0,0,3,3);
            Xf_da_right = Tf_da_right.block(0,3,3,1);

            MatrixNd pos_offset = MatrixNd::Zero(3,1);

            Xf_da_right *= 1./1.35;
            Xf_da_right(1,0) *= 1.2;
            pos_offset << 50,50,0;

            Xf_da_right -= pos_offset;
            Tf_da_left = kine_data_arm_left(q_da_left,l_da);
            Rf_da_left = Tf_da_left.block(0,0,3,3);
            Xf_da_left = Tf_da_left.block(0,3,3,1);

            Xf_da_left *= 1./1.35;
            Xf_da_left(1,0) *= 1.2;
            pos_offset << 50,-50,0;
            Xf_da_left -= pos_offset;


            //set desired task (RH, LH, RF, LF)
            int hand_global = 0;

            Xd_hubo2_RH = Xf_da_right;
            Rd_hubo2_RH = Rf_da_right;

            Xd_hubo2_LH = Xf_da_left;
            Rd_hubo2_LH = Rf_da_left;

//                        Xd_hubo2_RH<< 250, -50 + sin(idx_save/400.0*PI)*50, 300 + cos(idx_save/400.0*PI)*50;
//            Xd_hubo2_RH << 150, 50, -100;
//            Rd_hubo2_RH = Ry(-90/180.0*PI)*Rx(0/180.0*PI);

//            //            Xd_hubo2_LH<< 250, 50 + sin(idx_save/400.0*PI)*50, 300 + cos(idx_save/400.0*PI)*50;
//            Xd_hubo2_LH << 150, -50, -100;
//            Rd_hubo2_LH = Ry(-90/180.0*PI)*Rx(-0/180.0*PI);

            //have to modify JSJSJSJSJS
            Xd_hubo2_RF << 0, -100, -575;
            Rd_hubo2_RF = MatrixNd::Identity(3,3);
            //            Rd_hubo2_RF = Rx(-PI/1.5);

            Xd_hubo2_LF << 0, 100, -575;
            Rd_hubo2_LF = MatrixNd::Identity(3,3);

            //have to modify JSJSJSJSJS

//            Xd_hubo2_RF << 0, -150, -500;
//            Rd_hubo2_RF = MatrixNd::Identity(3,3);
//            //            Rd_hubo2_RF = Rx(-PI/1.5);

//            Xd_hubo2_LF << 0, 0, -575;
//            Rd_hubo2_LF = MatrixNd::Identity(3,3);

            Rd_hubo2_WST = Rz(0/4.0);

            Math::Vector3d _rot_vec(cos(idx_save/400.0*PI),sin(idx_save/400.0*PI),0);
            Math::Quaternion temp_quat = Math::Quaternion::fromAxisAngle(_rot_vec,-PI/8);
            Rd_hubo2_PEL = temp_quat.toMatrix();
            Rd_hubo2_PEL = Rx(-0/8);

            double imu_yaw = 0;
            imu_yaw = sharedData->IMU[0].Yaw;
            if(imu_yaw > 30.0)
                imu_yaw = 30.0;
            if(imu_yaw < -30.0)
                imu_yaw = -30.0;

            double imu_roll = 0;
            imu_roll = sharedData->IMU[0].Roll;
            if(imu_roll > 30.0)
                imu_roll = 30.0;
            if(imu_roll < -30.0)
                imu_roll = -30.0;

            double imu_pitch = 0;
            imu_pitch = sharedData->IMU[0].Pitch;
            if(imu_pitch > 30.0)
                imu_pitch = 30.0;
            if(imu_pitch < -30.0)
                imu_pitch = -30.0;


//            Rd_hubo2_WST = Rz(sharedData->IMU[0].Yaw * PI/180.0);
//            Rd_hubo2_PEL = Ry(-sharedData->IMU[0].Pitch  * PI/180.0) * Rx(-sharedData->IMU[0].Roll  * PI/180.0);
            Rd_hubo2_WST = Rz(imu_yaw * PI/180.0);
            Rd_hubo2_PEL = Ry(-imu_pitch  * PI/180.0) * Rx(-imu_roll  * PI/180.0);

//            //cout << "yaw : " << sharedData->IMU[0].Yaw << ", pitch : " << sharedData->IMU[0].Pitch << ", roll : " << sharedData->IMU[0].Roll << endl;
            double Xd_hubo2_Pelz = 0;

//            if(rt_count < 200*5)
//            {
//                //cout << "phase 1 " << endl;
//                _rt_count = rt_count-0.;
//                desired_COM <<0,150. / 200. / 5. * _rt_count,400;

//            }
//            else if(rt_count < 200*4 + 200*5)
//            {
//                _rt_count = rt_count - 200*5;
//                //cout << "phase 2 " << endl;
//                desired_COM <<0,150,400;
//                Rd_hubo2_PEL = Rx(-_rt_count/200./4.*PI/4.);
//                Xd_hubo2_RF << 0, -150+_rt_count/200./4.*-350, -450 + _rt_count/200./4.*750.;
//                Rd_hubo2_RF = Rx(-_rt_count/200./4.*PI/2.0);
//                Xd_hubo2_LF << 0, 150, -450-200*_rt_count/200./4.;

//                Rd_hubo2_WST = Rz(-_rt_count/200./4.*PI/4.);

//            }
//            else
//            {
//                desired_COM <<0,150,400;
//                Rd_hubo2_PEL = Rx(-PI/4.);
//                Xd_hubo2_RF << 0, -500, 300;
//                Rd_hubo2_RF = Rx(-PI/2.0);
//                Rd_hubo2_WST = Rz(-PI/4.0);
//                Xd_hubo2_LF << 0, 150, -450-200;
//            }

//            //cout << "CalcBodyToBaseCoordinates(*js_rbdl.Robot, q_hubo2, js_rbdl.n_torso,zv,1) : " << endl << CalcBodyToBaseCoordinates(*js_rbdl.Robot, q_hubo2, js_rbdl.n_pel,zv,1) << endl;
//            //cout << "CalcBodyWorldOrientation(*js_rbdl.Robot, q_hubo2, js_rbdl.n_pel,1) : " << endl << CalcBodyWorldOrientation(*js_rbdl.Robot, q_hubo2, js_rbdl.n_pel,1) << endl;


            if(hand_global == 1)
            {
                Xf_hubo2_RH = CalcBodyToBaseCoordinates(*js_rbdl.Robot, q_hubo2, js_rbdl.n_rwp,wp2ee,1);
                Rf_hubo2_RH = CalcBodyWorldOrientation(*js_rbdl.Robot, q_hubo2, js_rbdl.n_rwp, 1);

            }
            else
            {
                Xf_hubo2_RH = CalcBodyToBaseCoordinates(*js_rbdl.Robot, q_hubo2, js_rbdl.n_rwp,wp2ee,1) - CalcBodyToBaseCoordinates(*js_rbdl.Robot, q_hubo2, js_rbdl.n_torso,torso2rsp,1);
                Xf_hubo2_RH = CalcBodyWorldOrientation(*js_rbdl.Robot, q_hubo2, js_rbdl.n_torso,1)* Xf_hubo2_RH;
                Rf_hubo2_RH = CalcBodyWorldOrientation(*js_rbdl.Robot, q_hubo2, js_rbdl.n_rwp, 1) * CalcBodyWorldOrientation(*js_rbdl.Robot, q_hubo2, js_rbdl.n_torso,1).inverse();
            }

//            //cout << "Xd_hubo2_RH : " << endl << Xd_hubo2_RH << endl;
//            //cout << "Xf_hubo2_RH : " << endl << Xf_hubo2_RH << endl;

//            //cout << "Rd_hubo2_RH : " << endl << Rd_hubo2_RH << endl;
//            //cout << "Rf_hubo2_RH : " << endl << Rf_hubo2_RH << endl;

            if(hand_global == 1)
            {
                Xf_hubo2_LH = CalcBodyToBaseCoordinates(*js_rbdl.Robot, q_hubo2, js_rbdl.n_lwp,wp2ee,1);
                Rf_hubo2_LH = CalcBodyWorldOrientation(*js_rbdl.Robot, q_hubo2, js_rbdl.n_lwp, 1);

            }
            else
            {
                Xf_hubo2_LH = CalcBodyToBaseCoordinates(*js_rbdl.Robot, q_hubo2, js_rbdl.n_lwp,wp2ee,1) - CalcBodyToBaseCoordinates(*js_rbdl.Robot, q_hubo2, js_rbdl.n_torso,torso2lsp,1);
                Xf_hubo2_LH = CalcBodyWorldOrientation(*js_rbdl.Robot, q_hubo2, js_rbdl.n_torso,1) * Xf_hubo2_LH;
                Rf_hubo2_LH = CalcBodyWorldOrientation(*js_rbdl.Robot, q_hubo2, js_rbdl.n_lwp, 1) * CalcBodyWorldOrientation(*js_rbdl.Robot, q_hubo2, js_rbdl.n_torso,1).inverse();
            }

//            //cout << "Xd_hubo2_LH : " << endl << Xd_hubo2_LH << endl;
//            //cout << "Xf_hubo2_LH : " << endl << Xf_hubo2_LH << endl;

//            //cout << "Rd_hubo2_LH : " << endl << Rd_hubo2_LH << endl;
//            //cout << "Rf_hubo2_LH : " << endl << Rf_hubo2_LH << endl;



            ////


            Xf_hubo2_RF = CalcBodyToBaseCoordinates(*js_rbdl.Robot, q_hubo2, js_rbdl.n_rar,ar2ee,1);
            Rf_hubo2_RF = CalcBodyWorldOrientation(*js_rbdl.Robot, q_hubo2, js_rbdl.n_rar, 1);


            Xf_hubo2_LF = CalcBodyToBaseCoordinates(*js_rbdl.Robot, q_hubo2, js_rbdl.n_lar,ar2ee,1);
            Rf_hubo2_LF = CalcBodyWorldOrientation(*js_rbdl.Robot, q_hubo2, js_rbdl.n_lar, 1);

//            //cout << "Xf_hubo2_RF : " << endl << Xf_hubo2_RF << endl;
//            //cout << "Rf_hubo2_RF : " << endl << Rf_hubo2_RF << endl;


//            //cout << "Xf_hubo2_LF : " << endl << Xf_hubo2_LF << endl;
//            //cout << "Rf_hubo2_LF : " << endl << Rf_hubo2_LF << endl;

            /////


            //pelvis z
            err_pos_PELZ(0,0) = Xd_hubo2_Pelz-q_hubo2(2,0);

            //pelvis orientation error
            Rf_hubo2_PEL = CalcBodyWorldOrientation(*js_rbdl.Robot, q_hubo2, js_rbdl.n_pel,1).inverse();

//            //cout <<"Rd_hubo2_PEL : " << endl << Rd_hubo2_PEL << endl;
//            //cout <<"Rf_hubo2_PEL : " << endl << Rf_hubo2_PEL << endl;

            err_M_PEL = Rd_hubo2_PEL*Rf_hubo2_PEL.inverse();

            temp_double = (err_M_PEL.trace() - 1.0) / 2.0;

            if(temp_double >= 1)
                temp_double = 1-1e-6;
            else if(temp_double <= -1)
                temp_double = -1+1e-6;


            double th_PEL = acos(temp_double);

            if(fabs(sin(th_PEL)) < 1e-6)
            {
                err_ori_PEL = MatrixNd::Zero(3,1);

            }
            else
            {
                err_ori_PEL(0,0) = th_PEL/2./sin(th_PEL)*(err_M_PEL(2,1) - err_M_PEL(1,2));
                err_ori_PEL(1,0) = th_PEL/2./sin(th_PEL)*(err_M_PEL(0,2) - err_M_PEL(2,0));
                err_ori_PEL(2,0) = th_PEL/2./sin(th_PEL)*(err_M_PEL(1,0) - err_M_PEL(0,1));
            }

            //wasit joint error
            //            Rd_hubo2_WST = CalcBodyWorldOrientation(*js_rbdl.Robot, q_hubo2, js_rbdl.n_pel,1).inverse() * Rz(0/16);
            //            Rf_hubo2_WST = CalcBodyWorldOrientation(*js_rbdl.Robot, q_hubo2, js_rbdl.n_torso,1).inverse();

            Rf_hubo2_WST = CalcBodyWorldOrientation(*js_rbdl.Robot, q_hubo2, js_rbdl.n_torso, 1) * CalcBodyWorldOrientation(*js_rbdl.Robot, q_hubo2, js_rbdl.n_pel,1).inverse();
            err_M_WST = Rd_hubo2_WST*Rf_hubo2_WST;

            temp_double = (err_M_WST.trace() - 1.0) / 2.0;

            if(temp_double >= 1)
                temp_double = 1-1e-6;
            else if(temp_double <= -1)
                temp_double = -1+1e-6;


            double th_WST = acos(temp_double);

            if(fabs(sin(th_WST)) < 1e-6)
            {
                err_ori_WST = MatrixNd::Zero(3,1);

            }
            else
            {
                err_ori_WST(0,0) = th_WST/2./sin(th_WST)*(err_M_WST(2,1) - err_M_WST(1,2));
                err_ori_WST(1,0) = th_WST/2./sin(th_WST)*(err_M_WST(0,2) - err_M_WST(2,0));
                err_ori_WST(2,0) = th_WST/2./sin(th_WST)*(err_M_WST(1,0) - err_M_WST(0,1));
            }

            ///////////

            err_M_RH = Rd_hubo2_RH*Rf_hubo2_RH;

            temp_double = (err_M_RH.trace() - 1.0) / 2.0;

            if(temp_double >= 1)
                temp_double = 1-1e-6;
            else if(temp_double <= -1)
                temp_double = -1+1e-6;


            double th_RH = acos(temp_double);

            err_M_LH = Rd_hubo2_LH*Rf_hubo2_LH;

            temp_double = (err_M_LH.trace() - 1.0) / 2.0;

            if(temp_double >= 1)
                temp_double = 1-1e-6;
            else if(temp_double <= -1)
                temp_double = -1+1e-6;

            double th_LH = acos(temp_double);

            err_M_RF = Rd_hubo2_RF*Rf_hubo2_RF;
            //            //cout << " Rd_hubo2_RF : " << endl << Rd_hubo2_RF << endl;
            //            //cout << " Rf_hubo2_RF : " << endl << Rf_hubo2_RF << endl;
            //            //cout << " err_M_RF : " << endl << err_M_RF << endl;

            temp_double = (err_M_RF.trace() - 1.0) / 2.0;

            if(temp_double >= 1)
                temp_double = 1-1e-6;
            else if(temp_double <= -1)
                temp_double = -1+1e-6;

            //            //cout << "temp double : " << endl << temp_double << endl;
            double th_RF = acos(temp_double);

            err_M_LF = Rd_hubo2_LF*Rf_hubo2_LF;

            temp_double = (err_M_LF.trace() - 1.0) / 2.0;

            if(temp_double >= 1)
                temp_double = 1-1e-6;
            else if(temp_double <= -1)
                temp_double = -1+1e-6;


            double th_LF = acos(temp_double);




            if(fabs(sin(th_RH)) < 1e-6)
            {
                err_ori_RH = MatrixNd::Zero(3,1);

            }
            else
            {
                err_ori_RH(0,0) = th_RH/2./sin(th_RH)*(err_M_RH(2,1) - err_M_RH(1,2));
                err_ori_RH(1,0) = th_RH/2./sin(th_RH)*(err_M_RH(0,2) - err_M_RH(2,0));
                err_ori_RH(2,0) = th_RH/2./sin(th_RH)*(err_M_RH(1,0) - err_M_RH(0,1));
            }

            err_pos_RH = Xd_hubo2_RH - Xf_hubo2_RH;

            //            //cout << "err_ori_RH : " << endl << err_ori_RH << endl;
            //            //cout << "err_pos_RH : " << endl << err_pos_RH << endl;

            if(fabs(sin(th_LH)) < 1e-6)
            {
                err_ori_LH = MatrixNd::Zero(3,1);

            }
            else
            {
                err_ori_LH(0,0) = th_LH/2./sin(th_LH)*(err_M_LH(2,1) - err_M_LH(1,2));
                err_ori_LH(1,0) = th_LH/2./sin(th_LH)*(err_M_LH(0,2) - err_M_LH(2,0));
                err_ori_LH(2,0) = th_LH/2./sin(th_LH)*(err_M_LH(1,0) - err_M_LH(0,1));
            }

            err_pos_LH = Xd_hubo2_LH - Xf_hubo2_LH;

            //            //cout << "err_ori_LH : " << endl << err_ori_LH << endl;
            //            //cout << "err_pos_LH : " << endl << err_pos_LH << endl;

            if(fabs(sin(th_RF)) < 1e-6)
            {
                err_ori_RF = MatrixNd::Zero(3,1);

            }
            else
            {
                err_ori_RF(0,0) = th_RF/2./sin(th_RF)*(err_M_RF(2,1) - err_M_RF(1,2));
                err_ori_RF(1,0) = th_RF/2./sin(th_RF)*(err_M_RF(0,2) - err_M_RF(2,0));
                err_ori_RF(2,0) = th_RF/2./sin(th_RF)*(err_M_RF(1,0) - err_M_RF(0,1));
            }

            err_pos_RF = Xd_hubo2_RF - Xf_hubo2_RF;

            //            //cout << " aaa : " << endl << err_M_RF << endl;
            //            //cout << " bbb : " << endl << th_RF << endl;

            //            //cout << "err_ori_RF : " << endl << err_ori_RF << endl;
            //            //cout << "err_pos_RF : " << endl << err_pos_RF << endl;

            if(fabs(sin(th_LF)) < 1e-6)
            {
                err_ori_LF = MatrixNd::Zero(3,1);

            }
            else
            {
                err_ori_LF(0,0) = th_LF/2./sin(th_LF)*(err_M_LF(2,1) - err_M_LF(1,2));
                err_ori_LF(1,0) = th_LF/2./sin(th_LF)*(err_M_LF(0,2) - err_M_LF(2,0));
                err_ori_LF(2,0) = th_LF/2./sin(th_LF)*(err_M_LF(1,0) - err_M_LF(0,1));
            }

            err_pos_LF = Xd_hubo2_LF - Xf_hubo2_LF;

            js_rbdl.Calc_Jacob_COM(q_hubo2,m_robot,COM);    //calc com jacobian

            MatrixNd err_com = MatrixNd::Zero(3,1);
            err_com = desired_COM-COM;

            t1 = rt_timer_read();

            //for self-collision

            //for right link 1 and body

            body_center = CalcBodyToBaseCoordinates(*js_rbdl.Robot, q_hubo2, js_rbdl.n_torso,zv,1);
            body_rot = CalcBodyWorldOrientation(*js_rbdl.Robot, q_hubo2, js_rbdl.n_torso, 1).inverse();
            body_length << 200,250,700;

            temp_joint_to_linkc <<0,0,-l_hubo2(0,0)/2.0; // local
            temp_link_center = CalcBodyToBaseCoordinates(*js_rbdl.Robot, q_hubo2, js_rbdl.n_rsy,temp_joint_to_linkc,1);
            temp_link_rot = CalcBodyWorldOrientation(*js_rbdl.Robot, q_hubo2, js_rbdl.n_rsy, 1).inverse();
            temp_link_length = l_hubo2(0,0);
            temp_link_radi = 40;

            t1 = rt_timer_read();
            find_cloest_points_box_cylinder(body_center,body_rot,body_length(0,0),body_length(1,0),body_length(2,0),temp_link_center,temp_link_rot,temp_link_length,temp_link_radi,temp_cp1,temp_cp2); //cp1: body, cp2: arm link1
//            //cout << "FCL 11 time : " << (rt_timer_read() - t1)/1000./1000. << endl;

            temp_J1 = MatrixNd::Zero(6,n_dof);
            temp_J2 = MatrixNd::Zero(6,n_dof);

            CalcPointJacobian6D(*js_rbdl.Robot,q_hubo2,js_rbdl.n_torso,body_center + temp_cp1,temp_J1,1); // temp_cp2 -> local coordinate
            CalcPointJacobian6D(*js_rbdl.Robot,q_hubo2,js_rbdl.n_rsy,temp_joint_to_linkc + temp_cp2,temp_J2,1); // temp_cp2 -> local coordinate

            temp_cp1 = body_rot*temp_cp1 + body_center;
            temp_cp2 = temp_link_rot*temp_cp2 + temp_link_center; // temp_cp1, temp_cp2 --> global coordinate

            d1 = ((temp_cp1 - temp_cp2).norm() - dist_margin)/dt;
            n = (temp_cp1 - temp_cp2) / (temp_cp1 - temp_cp2).norm();
            N1 = n.transpose()*(temp_J2.block(3,0,3,n_dof) - temp_J1.block(3,0,3,n_dof));

            //for right link 2 and body

            temp_joint_to_linkc <<-l_hubo2(3,0),0,-l_hubo2(1,0)/2.0; // local
            temp_link_center = CalcBodyToBaseCoordinates(*js_rbdl.Robot, q_hubo2, js_rbdl.n_reb,temp_joint_to_linkc,1);

            temp_link_rot = CalcBodyWorldOrientation(*js_rbdl.Robot, q_hubo2, js_rbdl.n_reb, 1).inverse();
            temp_link_length = l_hubo2(1,0);
            temp_link_radi = 40;

            find_cloest_points_box_cylinder(body_center,body_rot,body_length(0,0),body_length(1,0),body_length(2,0),temp_link_center,temp_link_rot,temp_link_length,temp_link_radi,temp_cp1,temp_cp2); //cp1: body, cp2: arm link1

            temp_J1 = MatrixNd::Zero(6,n_dof);
            temp_J2 = MatrixNd::Zero(6,n_dof);

            CalcPointJacobian6D(*js_rbdl.Robot,q_hubo2,js_rbdl.n_torso,body_center + temp_cp1,temp_J1,1); // temp_cp2 -> local coordinate
            CalcPointJacobian6D(*js_rbdl.Robot,q_hubo2,js_rbdl.n_reb,temp_joint_to_linkc + temp_cp2,temp_J2,1); // temp_cp2 -> local coordinate

            temp_cp1 = body_rot*temp_cp1 + body_center;
            temp_cp2 = temp_link_rot*temp_cp2 + temp_link_center; // temp_cp1, temp_cp2 --> global coordinate

            d2 = ((temp_cp1 - temp_cp2).norm() - dist_margin)/dt;
            n = (temp_cp1 - temp_cp2) / (temp_cp1 - temp_cp2).norm();
            N2 = n.transpose()*(temp_J2.block(3,0,3,n_dof) - temp_J1.block(3,0,3,n_dof));

            //for right link 3 and body
            temp_joint_to_linkc <<0,0,-l_hubo2(2,0)/2.0; // local
            temp_link_center = CalcBodyToBaseCoordinates(*js_rbdl.Robot, q_hubo2, js_rbdl.n_rwp,temp_joint_to_linkc,1);

            temp_link_rot = CalcBodyWorldOrientation(*js_rbdl.Robot, q_hubo2, js_rbdl.n_rwp, 1).inverse();
            temp_link_length = l_hubo2(2,0);
            temp_link_radi = 50;

            find_cloest_points_box_cylinder(body_center,body_rot,body_length(0,0),body_length(1,0),body_length(2,0),temp_link_center,temp_link_rot,temp_link_length,temp_link_radi,temp_cp1,temp_cp2); //cp1: body, cp2: arm link1

            temp_J1 = MatrixNd::Zero(6,n_dof);
            temp_J2 = MatrixNd::Zero(6,n_dof);

            CalcPointJacobian6D(*js_rbdl.Robot,q_hubo2,js_rbdl.n_torso,body_center + temp_cp1,temp_J1,1); // temp_cp2 -> local coordinate
            CalcPointJacobian6D(*js_rbdl.Robot,q_hubo2,js_rbdl.n_rwp,temp_joint_to_linkc + temp_cp2,temp_J2,1); // temp_cp2 -> local coordinate

            temp_cp1 = body_rot*temp_cp1 + body_center;
            temp_cp2 = temp_link_rot*temp_cp2 + temp_link_center; // temp_cp1, temp_cp2 --> global coordinate

            d3 = ((temp_cp1 - temp_cp2).norm() - dist_margin)/dt;
            n = (temp_cp1 - temp_cp2) / (temp_cp1 - temp_cp2).norm();
            N3 = n.transpose()*(temp_J2.block(3,0,3,n_dof) - temp_J1.block(3,0,3,n_dof));

            //for right link 1 and link 3
            temp_joint_to_linkc_ <<0,0,-l_hubo2(0,0)/2.0; // local
            temp_link_center_ = CalcBodyToBaseCoordinates(*js_rbdl.Robot, q_hubo2, js_rbdl.n_rsy,temp_joint_to_linkc_,1);
            temp_link_rot_ = CalcBodyWorldOrientation(*js_rbdl.Robot, q_hubo2, js_rbdl.n_rsy, 1).inverse();
            temp_link_length_ = l_hubo2(0,0);
            temp_link_radi_ = 40;

            temp_joint_to_linkc <<0,0,-l_hubo2(2,0)/2.0; // local
            temp_link_center = CalcBodyToBaseCoordinates(*js_rbdl.Robot, q_hubo2, js_rbdl.n_rwp,temp_joint_to_linkc,1);
            temp_link_rot = CalcBodyWorldOrientation(*js_rbdl.Robot, q_hubo2, js_rbdl.n_rwp, 1).inverse();
            temp_link_length = l_hubo2(2,0);
            temp_link_radi = 50;

            find_cloest_points_cylinder_cylinder(temp_link_center,temp_link_rot,temp_link_length,temp_link_radi,temp_link_center_,temp_link_rot_,temp_link_length_,temp_link_radi_,temp_cp1,temp_cp2); //cp1: body, cp2: arm link1

            temp_J1 = MatrixNd::Zero(6,n_dof);
            temp_J2 = MatrixNd::Zero(6,n_dof);

            CalcPointJacobian6D(*js_rbdl.Robot,q_hubo2,js_rbdl.n_rwp,temp_joint_to_linkc + temp_cp1,temp_J1,1); // temp_cp1 -> local coordinate, link3
            CalcPointJacobian6D(*js_rbdl.Robot,q_hubo2,js_rbdl.n_rsy,temp_joint_to_linkc_ + temp_cp2,temp_J2,1); // temp_cp2 -> local coordinate, link1

            temp_cp1 = temp_link_rot*temp_cp1 + temp_link_center;
            temp_cp2 = temp_link_rot_*temp_cp2 + temp_link_center_; // temp_cp1, temp_cp2 --> global coordinate

            d4 = ((temp_cp1 - temp_cp2).norm() - dist_margin)/dt;
            n = (temp_cp1 - temp_cp2) / (temp_cp1 - temp_cp2).norm();
            N4 = n.transpose()*(temp_J2.block(3,0,3,n_dof) - temp_J1.block(3,0,3,n_dof));




            //////


            //for left link 1 and body

            temp_joint_to_linkc <<0,0,-l_hubo2(0,0)/2.0; // local
            temp_link_center = CalcBodyToBaseCoordinates(*js_rbdl.Robot, q_hubo2, js_rbdl.n_lsy,temp_joint_to_linkc,1);
            temp_link_rot = CalcBodyWorldOrientation(*js_rbdl.Robot, q_hubo2, js_rbdl.n_lsy, 1).inverse();
            temp_link_length = l_hubo2(0,0);
            temp_link_radi = 40;

            find_cloest_points_box_cylinder(body_center,body_rot,body_length(0,0),body_length(1,0),body_length(2,0),temp_link_center,temp_link_rot,temp_link_length,temp_link_radi,temp_cp1,temp_cp2); //cp1: body, cp2: arm link1

            temp_J1 = MatrixNd::Zero(6,n_dof);
            temp_J2 = MatrixNd::Zero(6,n_dof);

            CalcPointJacobian6D(*js_rbdl.Robot,q_hubo2,js_rbdl.n_torso,body_center + temp_cp1,temp_J1,1); // temp_cp2 -> local coordinate
            CalcPointJacobian6D(*js_rbdl.Robot,q_hubo2,js_rbdl.n_lsy,temp_joint_to_linkc + temp_cp2,temp_J2,1); // temp_cp2 -> local coordinate

            temp_cp1 = body_rot*temp_cp1 + body_center;
            temp_cp2 = temp_link_rot*temp_cp2 + temp_link_center; // temp_cp1, temp_cp2 --> global coordinate

            d5 = ((temp_cp1 - temp_cp2).norm() - dist_margin)/dt;
            n = (temp_cp1 - temp_cp2) / (temp_cp1 - temp_cp2).norm();
            N5 = n.transpose()*(temp_J2.block(3,0,3,n_dof) - temp_J1.block(3,0,3,n_dof));

            //for left link 2 and body

            temp_joint_to_linkc <<-l_hubo2(3,0),0,-l_hubo2(1,0)/2.0; // local
            temp_link_center = CalcBodyToBaseCoordinates(*js_rbdl.Robot, q_hubo2, js_rbdl.n_leb,temp_joint_to_linkc,1);
            temp_link_rot = CalcBodyWorldOrientation(*js_rbdl.Robot, q_hubo2, js_rbdl.n_leb, 1).inverse();
            temp_link_length = l_hubo2(1,0);
            temp_link_radi = 40;

            find_cloest_points_box_cylinder(body_center,body_rot,body_length(0,0),body_length(1,0),body_length(2,0),temp_link_center,temp_link_rot,temp_link_length,temp_link_radi,temp_cp1,temp_cp2); //cp1: body, cp2: arm link1

            temp_J1 = MatrixNd::Zero(6,n_dof);
            temp_J2 = MatrixNd::Zero(6,n_dof);

            CalcPointJacobian6D(*js_rbdl.Robot,q_hubo2,js_rbdl.n_torso,body_center + temp_cp1,temp_J1,1); // temp_cp2 -> local coordinate
            CalcPointJacobian6D(*js_rbdl.Robot,q_hubo2,js_rbdl.n_leb,temp_joint_to_linkc + temp_cp2,temp_J2,1); // temp_cp2 -> local coordinate

            temp_cp1 = body_rot*temp_cp1 + body_center;
            temp_cp2 = temp_link_rot*temp_cp2 + temp_link_center; // temp_cp1, temp_cp2 --> global coordinate

            d6 = ((temp_cp1 - temp_cp2).norm() - dist_margin)/dt;
            n = (temp_cp1 - temp_cp2) / (temp_cp1 - temp_cp2).norm();
            N6 = n.transpose()*(temp_J2.block(3,0,3,n_dof) - temp_J1.block(3,0,3,n_dof));

            //for left link 3 and body
            temp_joint_to_linkc <<0,0,-l_hubo2(2,0)/2.0; // local
            temp_link_center = CalcBodyToBaseCoordinates(*js_rbdl.Robot, q_hubo2, js_rbdl.n_lwp,temp_joint_to_linkc,1);
            //            temp_link_center += CalcBodyToBaseCoordinates(*js_rbdl.Robot, q_hubo2, js_rbdl.n_pel,zv,1);

            temp_link_rot = CalcBodyWorldOrientation(*js_rbdl.Robot, q_hubo2, js_rbdl.n_lwp, 1).inverse();
            temp_link_length = l_hubo2(2,0);
            temp_link_radi = 50;

            find_cloest_points_box_cylinder(body_center,body_rot,body_length(0,0),body_length(1,0),body_length(2,0),temp_link_center,temp_link_rot,temp_link_length,temp_link_radi,temp_cp1,temp_cp2); //cp1: body, cp2: arm link1

            temp_J1 = MatrixNd::Zero(6,n_dof);
            temp_J2 = MatrixNd::Zero(6,n_dof);

            CalcPointJacobian6D(*js_rbdl.Robot,q_hubo2,js_rbdl.n_torso,body_center + temp_cp1,temp_J1,1); // temp_cp2 -> local coordinate
            CalcPointJacobian6D(*js_rbdl.Robot,q_hubo2,js_rbdl.n_lwp,temp_joint_to_linkc + temp_cp2,temp_J2,1); // temp_cp2 -> local coordinate

            temp_cp1 = body_rot*temp_cp1 + body_center;
            temp_cp2 = temp_link_rot*temp_cp2 + temp_link_center; // temp_cp1, temp_cp2 --> global coordinate

            d7 = ((temp_cp1 - temp_cp2).norm() - dist_margin)/dt;
            n = (temp_cp1 - temp_cp2) / (temp_cp1 - temp_cp2).norm();
            N7 = n.transpose()*(temp_J2.block(3,0,3,n_dof) - temp_J1.block(3,0,3,n_dof));

            //for left link 1 and link 3
            temp_joint_to_linkc_ <<0,0,-l_hubo2(0,0)/2.0; // local
            temp_link_center_ = CalcBodyToBaseCoordinates(*js_rbdl.Robot, q_hubo2, js_rbdl.n_lsy,temp_joint_to_linkc_,1);
            //            temp_link_center_ += CalcBodyToBaseCoordinates(*js_rbdl.Robot, q_hubo2, js_rbdl.n_pel,zv,1);

            temp_link_rot_ = CalcBodyWorldOrientation(*js_rbdl.Robot, q_hubo2, js_rbdl.n_lsy, 1).inverse();
            temp_link_length_ = l_hubo2(0,0);
            temp_link_radi_ = 40;

            temp_joint_to_linkc <<0,0,-l_hubo2(2,0)/2.0; // local
            temp_link_center = CalcBodyToBaseCoordinates(*js_rbdl.Robot, q_hubo2, js_rbdl.n_lwp,temp_joint_to_linkc,1);
            //            temp_link_center += CalcBodyToBaseCoordinates(*js_rbdl.Robot, q_hubo2, js_rbdl.n_pel,zv,1);

            temp_link_rot = CalcBodyWorldOrientation(*js_rbdl.Robot, q_hubo2, js_rbdl.n_lwp, 1).inverse();
            temp_link_length = l_hubo2(2,0);
            temp_link_radi = 50;

            find_cloest_points_cylinder_cylinder(temp_link_center,temp_link_rot,temp_link_length,temp_link_radi,temp_link_center_,temp_link_rot_,temp_link_length_,temp_link_radi_,temp_cp1,temp_cp2); //cp1: body, cp2: arm link1

            temp_J1 = MatrixNd::Zero(6,n_dof);
            temp_J2 = MatrixNd::Zero(6,n_dof);

            CalcPointJacobian6D(*js_rbdl.Robot,q_hubo2,js_rbdl.n_lwp,temp_joint_to_linkc + temp_cp1,temp_J1,1); // temp_cp1 -> local coordinate, link3
            CalcPointJacobian6D(*js_rbdl.Robot,q_hubo2,js_rbdl.n_lsy,temp_joint_to_linkc_ + temp_cp2,temp_J2,1); // temp_cp2 -> local coordinate, link1

            temp_cp1 = temp_link_rot*temp_cp1 + temp_link_center;
            temp_cp2 = temp_link_rot_*temp_cp2 + temp_link_center_; // temp_cp1, temp_cp2 --> global coordinate

            d8 = ((temp_cp1 - temp_cp2).norm() - dist_margin)/dt;
            n = (temp_cp1 - temp_cp2) / (temp_cp1 - temp_cp2).norm();
            N8 = n.transpose()*(temp_J2.block(3,0,3,n_dof) - temp_J1.block(3,0,3,n_dof));

            //for right and left ee
            temp_joint_to_linkc_ <<0,0,-l_hubo2(2,0)/2.0; // local
            temp_link_center_ = CalcBodyToBaseCoordinates(*js_rbdl.Robot, q_hubo2, js_rbdl.n_rwp,temp_joint_to_linkc_,1);
            //            temp_link_center_ += CalcBodyToBaseCoordinates(*js_rbdl.Robot, q_hubo2, js_rbdl.n_pel,zv,1);

            temp_link_rot_ = CalcBodyWorldOrientation(*js_rbdl.Robot, q_hubo2, js_rbdl.n_rwp, 1).inverse();
            temp_link_length_ = l_hubo2(2,0);
            temp_link_radi_ = 75;

            temp_joint_to_linkc <<0,0,-l_hubo2(2,0)/2.0; // local
            temp_link_center = CalcBodyToBaseCoordinates(*js_rbdl.Robot, q_hubo2, js_rbdl.n_lwp,temp_joint_to_linkc,1);
            //            temp_link_center += CalcBodyToBaseCoordinates(*js_rbdl.Robot, q_hubo2, js_rbdl.n_pel,zv,1);

            temp_link_rot = CalcBodyWorldOrientation(*js_rbdl.Robot, q_hubo2, js_rbdl.n_lwp, 1).inverse();
            temp_link_length = l_hubo2(2,0);
            temp_link_radi = 75;

            find_cloest_points_cylinder_cylinder(temp_link_center,temp_link_rot,temp_link_length,temp_link_radi,temp_link_center_,temp_link_rot_,temp_link_length_,temp_link_radi_,temp_cp1,temp_cp2); //cp1: body, cp2: arm link1

            temp_J1 = MatrixNd::Zero(6,n_dof);
            temp_J2 = MatrixNd::Zero(6,n_dof);

            CalcPointJacobian6D(*js_rbdl.Robot,q_hubo2,js_rbdl.n_lwp,temp_joint_to_linkc + temp_cp1,temp_J1,1); // temp_cp1 -> local coordinate, link3
            CalcPointJacobian6D(*js_rbdl.Robot,q_hubo2,js_rbdl.n_rwp,temp_joint_to_linkc_ + temp_cp2,temp_J2,1); // temp_cp2 -> local coordinate, link1

            temp_cp1 = temp_link_rot*temp_cp1 + temp_link_center;
            temp_cp2 = temp_link_rot_*temp_cp2 + temp_link_center_; // temp_cp1, temp_cp2 --> global coordinate

            d9 = ((temp_cp1 - temp_cp2).norm() - dist_margin*1.5)/dt;
            n = (temp_cp1 - temp_cp2) / (temp_cp1 - temp_cp2).norm();
            N9 = n.transpose()*(temp_J2.block(3,0,3,n_dof) - temp_J1.block(3,0,3,n_dof));


            //right arm link 3 (cylinder)<-> right leg link 1 (box)

            box_center1_local = hp2kn/2;
            box_center1_global = CalcBodyToBaseCoordinates(*js_rbdl.Robot, q_hubo2, js_rbdl.n_rhp,box_center1_local,1);
            box_rot1 = CalcBodyWorldOrientation(*js_rbdl.Robot, q_hubo2, js_rbdl.n_rhp, 1).inverse();
            box_length1 << 100,100,-hp2kn(2,0);

            cylinder_center1_local = wp2ee/2;
            cylinder_center1_global = CalcBodyToBaseCoordinates(*js_rbdl.Robot, q_hubo2, js_rbdl.n_rwp,cylinder_center1_local,1);
            cylinder_rot1 = CalcBodyWorldOrientation(*js_rbdl.Robot, q_hubo2, js_rbdl.n_rwp, 1).inverse();
            cylinder_length1 = -wp2ee(2,0);
            cylinder_radi1 = 75;

            find_cloest_points_box_cylinder(box_center1_global,box_rot1,box_length1(0,0),box_length1(1,0),box_length1(2,0),cylinder_center1_global,cylinder_rot1,cylinder_length1,cylinder_radi1,temp_cp1,temp_cp2); //cp1: body, cp2: arm link1

            temp_J1 = MatrixNd::Zero(6,n_dof);
            temp_J2 = MatrixNd::Zero(6,n_dof);

            CalcPointJacobian6D(*js_rbdl.Robot,q_hubo2,js_rbdl.n_rhp,box_center1_local + temp_cp1,temp_J1,1); // temp_cp2 -> local coordinate
            CalcPointJacobian6D(*js_rbdl.Robot,q_hubo2,js_rbdl.n_rwp,cylinder_center1_local + temp_cp2,temp_J2,1); // temp_cp2 -> local coordinate

            temp_cp1 = box_rot1*temp_cp1 + box_center1_global;
            temp_cp2 = cylinder_rot1*temp_cp2 + cylinder_center1_global; // temp_cp1, temp_cp2 --> global coordinate

            d10 = ((temp_cp1 - temp_cp2).norm() - dist_margin)/dt;
            n = (temp_cp1 - temp_cp2) / (temp_cp1 - temp_cp2).norm();
            N10 = n.transpose()*(temp_J2.block(3,0,3,n_dof) - temp_J1.block(3,0,3,n_dof));

            //left arm link 3 (cylinder) <-> left leg link 1 (box)

            box_center1_local = hp2kn/2;
            box_center1_global = CalcBodyToBaseCoordinates(*js_rbdl.Robot, q_hubo2, js_rbdl.n_lhp,box_center1_local,1);
            box_rot1 = CalcBodyWorldOrientation(*js_rbdl.Robot, q_hubo2, js_rbdl.n_lhp, 1).inverse();
            box_length1 << 100,100,-hp2kn(2,0);

            cylinder_center1_local = wp2ee/2;
            cylinder_center1_global = CalcBodyToBaseCoordinates(*js_rbdl.Robot, q_hubo2, js_rbdl.n_lwp,cylinder_center1_local,1);
            cylinder_rot1 = CalcBodyWorldOrientation(*js_rbdl.Robot, q_hubo2, js_rbdl.n_lwp, 1).inverse();
            cylinder_length1 = -wp2ee(2,0);
            cylinder_radi1 = 75;

            find_cloest_points_box_cylinder(box_center1_global,box_rot1,box_length1(0,0),box_length1(1,0),box_length1(2,0),cylinder_center1_global,cylinder_rot1,cylinder_length1,cylinder_radi1,temp_cp1,temp_cp2); //cp1: body, cp2: arm link1

            temp_J1 = MatrixNd::Zero(6,n_dof);
            temp_J2 = MatrixNd::Zero(6,n_dof);

            CalcPointJacobian6D(*js_rbdl.Robot,q_hubo2,js_rbdl.n_lhp,box_center1_local + temp_cp1,temp_J1,1); // temp_cp2 -> local coordinate
            CalcPointJacobian6D(*js_rbdl.Robot,q_hubo2,js_rbdl.n_lwp,cylinder_center1_local + temp_cp2,temp_J2,1); // temp_cp2 -> local coordinate

            temp_cp1 = box_rot1*temp_cp1 + box_center1_global;
            temp_cp2 = cylinder_rot1*temp_cp2 + cylinder_center1_global; // temp_cp1, temp_cp2 --> global coordinate

            d11 = ((temp_cp1 - temp_cp2).norm() - dist_margin)/dt;
            n = (temp_cp1 - temp_cp2) / (temp_cp1 - temp_cp2).norm();
            N11 = n.transpose()*(temp_J2.block(3,0,3,n_dof) - temp_J1.block(3,0,3,n_dof));

            //right leg link 1 <-> left leg link 1

            box_center1_local = hp2kn/2;
            box_center1_global = CalcBodyToBaseCoordinates(*js_rbdl.Robot, q_hubo2, js_rbdl.n_rhp,box_center1_local,1);
            box_rot1 = CalcBodyWorldOrientation(*js_rbdl.Robot, q_hubo2, js_rbdl.n_rhp, 1).inverse();
            box_length1 << 100,100,-hp2kn(2,0);

            box_center2_local = hp2kn/2;
            box_center2_global = CalcBodyToBaseCoordinates(*js_rbdl.Robot, q_hubo2, js_rbdl.n_lhp,box_center1_local,1);
            box_rot2 = CalcBodyWorldOrientation(*js_rbdl.Robot, q_hubo2, js_rbdl.n_lhp, 1).inverse();
            box_length2 << 100,100,-hp2kn(2,0);

            find_cloest_points_box_box(box_center1_global,box_rot1,box_length1(0,0),box_length1(1,0),box_length1(2,0),box_center2_global,box_rot2,box_length2(0,0),box_length2(1,0),box_length2(2,0),temp_cp1,temp_cp2);

            temp_J1 = MatrixNd::Zero(6,n_dof);
            temp_J2 = MatrixNd::Zero(6,n_dof);

            CalcPointJacobian6D(*js_rbdl.Robot,q_hubo2,js_rbdl.n_rhp,box_center1_local + temp_cp1,temp_J1,1); // temp_cp2 -> local coordinate
            CalcPointJacobian6D(*js_rbdl.Robot,q_hubo2,js_rbdl.n_lhp,box_center2_local + temp_cp2,temp_J2,1); // temp_cp2 -> local coordinate

            temp_cp1 = box_rot1*temp_cp1 + box_center1_global;
            temp_cp2 = box_rot2*temp_cp2 + box_center2_global; // temp_cp1, temp_cp2 --> global coordinate

            d12 = ((temp_cp1 - temp_cp2).norm() - dist_margin/1.25)/dt;
            n = (temp_cp1 - temp_cp2) / (temp_cp1 - temp_cp2).norm();
            N12 = n.transpose()*(temp_J2.block(3,0,3,n_dof) - temp_J1.block(3,0,3,n_dof));

            //right leg link 1 <-> left leg link 2

            box_center1_local = hp2kn/2;
            box_center1_global = CalcBodyToBaseCoordinates(*js_rbdl.Robot, q_hubo2, js_rbdl.n_rhp,box_center1_local,1);
            box_rot1 = CalcBodyWorldOrientation(*js_rbdl.Robot, q_hubo2, js_rbdl.n_rhp, 1).inverse();
            box_length1 << 100,100,-hp2kn(2,0);

            box_center2_local = kn2ap/2;
            box_center2_global = CalcBodyToBaseCoordinates(*js_rbdl.Robot, q_hubo2, js_rbdl.n_lkn,box_center1_local,1);
            box_rot2 = CalcBodyWorldOrientation(*js_rbdl.Robot, q_hubo2, js_rbdl.n_lkn, 1).inverse();
            box_length2 << 80,80,-kn2ap(2,0);

            find_cloest_points_box_box(box_center1_global,box_rot1,box_length1(0,0),box_length1(1,0),box_length1(2,0),box_center2_global,box_rot2,box_length2(0,0),box_length2(1,0),box_length2(2,0),temp_cp1,temp_cp2);

            temp_J1 = MatrixNd::Zero(6,n_dof);
            temp_J2 = MatrixNd::Zero(6,n_dof);

            CalcPointJacobian6D(*js_rbdl.Robot,q_hubo2,js_rbdl.n_rhp,box_center1_local + temp_cp1,temp_J1,1); // temp_cp2 -> local coordinate
            CalcPointJacobian6D(*js_rbdl.Robot,q_hubo2,js_rbdl.n_lkn,box_center2_local + temp_cp2,temp_J2,1); // temp_cp2 -> local coordinate

            temp_cp1 = box_rot1*temp_cp1 + box_center1_global;
            temp_cp2 = box_rot2*temp_cp2 + box_center2_global; // temp_cp1, temp_cp2 --> global coordinate

            d13 = ((temp_cp1 - temp_cp2).norm() - dist_margin)/dt;
            n = (temp_cp1 - temp_cp2) / (temp_cp1 - temp_cp2).norm();
            N13 = n.transpose()*(temp_J2.block(3,0,3,n_dof) - temp_J1.block(3,0,3,n_dof));

            //right leg link 2 <-> left leg link 1

            box_center1_local = kn2ap/2;
            box_center1_global = CalcBodyToBaseCoordinates(*js_rbdl.Robot, q_hubo2, js_rbdl.n_rkn,box_center1_local,1);
            box_rot1 = CalcBodyWorldOrientation(*js_rbdl.Robot, q_hubo2, js_rbdl.n_rkn, 1).inverse();
            box_length1 << 80,80,-kn2ap(2,0);

            box_center2_local = hp2kn/2;
            box_center2_global = CalcBodyToBaseCoordinates(*js_rbdl.Robot, q_hubo2, js_rbdl.n_lhp,box_center1_local,1);
            box_rot2 = CalcBodyWorldOrientation(*js_rbdl.Robot, q_hubo2, js_rbdl.n_lhp, 1).inverse();
            box_length2 << 100,100,-hp2kn(2,0);

            find_cloest_points_box_box(box_center1_global,box_rot1,box_length1(0,0),box_length1(1,0),box_length1(2,0),box_center2_global,box_rot2,box_length2(0,0),box_length2(1,0),box_length2(2,0),temp_cp1,temp_cp2);

            temp_J1 = MatrixNd::Zero(6,n_dof);
            temp_J2 = MatrixNd::Zero(6,n_dof);

            CalcPointJacobian6D(*js_rbdl.Robot,q_hubo2,js_rbdl.n_rkn,box_center1_local + temp_cp1,temp_J1,1); // temp_cp2 -> local coordinate
            CalcPointJacobian6D(*js_rbdl.Robot,q_hubo2,js_rbdl.n_lhp,box_center2_local + temp_cp2,temp_J2,1); // temp_cp2 -> local coordinate

            temp_cp1 = box_rot1*temp_cp1 + box_center1_global;
            temp_cp2 = box_rot2*temp_cp2 + box_center2_global; // temp_cp1, temp_cp2 --> global coordinate

            d14 = ((temp_cp1 - temp_cp2).norm() - dist_margin)/dt;
            n = (temp_cp1 - temp_cp2) / (temp_cp1 - temp_cp2).norm();
            N14 = n.transpose()*(temp_J2.block(3,0,3,n_dof) - temp_J1.block(3,0,3,n_dof));

            //right leg link 2 <-> left leg link 2

            box_center1_local = kn2ap/2;
            box_center1_global = CalcBodyToBaseCoordinates(*js_rbdl.Robot, q_hubo2, js_rbdl.n_rkn,box_center1_local,1);
            box_rot1 = CalcBodyWorldOrientation(*js_rbdl.Robot, q_hubo2, js_rbdl.n_rkn, 1).inverse();
            box_length1 << 80,80,-kn2ap(2,0);

            box_center2_local = kn2ap/2;
            box_center2_global = CalcBodyToBaseCoordinates(*js_rbdl.Robot, q_hubo2, js_rbdl.n_lkn,box_center1_local,1);
            box_rot2 = CalcBodyWorldOrientation(*js_rbdl.Robot, q_hubo2, js_rbdl.n_lkn, 1).inverse();
            box_length2 << 80,80,-kn2ap(2,0);

            find_cloest_points_box_box(box_center1_global,box_rot1,box_length1(0,0),box_length1(1,0),box_length1(2,0),box_center2_global,box_rot2,box_length2(0,0),box_length2(1,0),box_length2(2,0),temp_cp1,temp_cp2);

            temp_J1 = MatrixNd::Zero(6,n_dof);
            temp_J2 = MatrixNd::Zero(6,n_dof);

            CalcPointJacobian6D(*js_rbdl.Robot,q_hubo2,js_rbdl.n_rkn,box_center1_local + temp_cp1,temp_J1,1); // temp_cp2 -> local coordinate
            CalcPointJacobian6D(*js_rbdl.Robot,q_hubo2,js_rbdl.n_lkn,box_center2_local + temp_cp2,temp_J2,1); // temp_cp2 -> local coordinate

            temp_cp1 = box_rot1*temp_cp1 + box_center1_global;
            temp_cp2 = box_rot2*temp_cp2 + box_center2_global; // temp_cp1, temp_cp2 --> global coordinate

            d15 = ((temp_cp1 - temp_cp2).norm() - dist_margin)/dt;
            n = (temp_cp1 - temp_cp2) / (temp_cp1 - temp_cp2).norm();
            N15 = n.transpose()*(temp_J2.block(3,0,3,n_dof) - temp_J1.block(3,0,3,n_dof));


            //right leg link 2 <-> left leg link 3

            box_center1_local = kn2ap/2;
            box_center1_global = CalcBodyToBaseCoordinates(*js_rbdl.Robot, q_hubo2, js_rbdl.n_rkn,box_center1_local,1);
            box_rot1 = CalcBodyWorldOrientation(*js_rbdl.Robot, q_hubo2, js_rbdl.n_rkn, 1).inverse();
            box_length1 << 80,80,-kn2ap(2,0);

            box_center2_local << 30,0,ar2ee(2,0);
            box_center2_global = CalcBodyToBaseCoordinates(*js_rbdl.Robot, q_hubo2, js_rbdl.n_lar,box_center1_local,1);
            box_rot2 = CalcBodyWorldOrientation(*js_rbdl.Robot, q_hubo2, js_rbdl.n_lar, 1).inverse();
            box_length2 << 220,130,-ar2ee(2,0);

            find_cloest_points_box_box(box_center1_global,box_rot1,box_length1(0,0),box_length1(1,0),box_length1(2,0),box_center2_global,box_rot2,box_length2(0,0),box_length2(1,0),box_length2(2,0),temp_cp1,temp_cp2);

            temp_J1 = MatrixNd::Zero(6,n_dof);
            temp_J2 = MatrixNd::Zero(6,n_dof);

            CalcPointJacobian6D(*js_rbdl.Robot,q_hubo2,js_rbdl.n_rkn,box_center1_local + temp_cp1,temp_J1,1); // temp_cp2 -> local coordinate
            CalcPointJacobian6D(*js_rbdl.Robot,q_hubo2,js_rbdl.n_lar,box_center2_local + temp_cp2,temp_J2,1); // temp_cp2 -> local coordinate

            temp_cp1 = box_rot1*temp_cp1 + box_center1_global;
            temp_cp2 = box_rot2*temp_cp2 + box_center2_global; // temp_cp1, temp_cp2 --> global coordinate

            d16 = ((temp_cp1 - temp_cp2).norm() - dist_margin)/dt;
            n = (temp_cp1 - temp_cp2) / (temp_cp1 - temp_cp2).norm();
            N16 = n.transpose()*(temp_J2.block(3,0,3,n_dof) - temp_J1.block(3,0,3,n_dof));

            //right leg link 3 <-> left leg link 2

            box_center1_local << 30,0,ar2ee(2,0);
            box_center1_global = CalcBodyToBaseCoordinates(*js_rbdl.Robot, q_hubo2, js_rbdl.n_rar,box_center1_local,1);
            box_rot1 = CalcBodyWorldOrientation(*js_rbdl.Robot, q_hubo2, js_rbdl.n_rar, 1).inverse();
            box_length1 << 220,130,-ar2ee(2,0);

            box_center2_local = kn2ap/2;
            box_center2_global = CalcBodyToBaseCoordinates(*js_rbdl.Robot, q_hubo2, js_rbdl.n_lkn,box_center1_local,1);
            box_rot2 = CalcBodyWorldOrientation(*js_rbdl.Robot, q_hubo2, js_rbdl.n_lkn, 1).inverse();
            box_length2 << 80,80,-kn2ap(2,0);


            find_cloest_points_box_box(box_center1_global,box_rot1,box_length1(0,0),box_length1(1,0),box_length1(2,0),box_center2_global,box_rot2,box_length2(0,0),box_length2(1,0),box_length2(2,0),temp_cp1,temp_cp2);

            temp_J1 = MatrixNd::Zero(6,n_dof);
            temp_J2 = MatrixNd::Zero(6,n_dof);

            CalcPointJacobian6D(*js_rbdl.Robot,q_hubo2,js_rbdl.n_rar,box_center1_local + temp_cp1,temp_J1,1); // temp_cp2 -> local coordinate
            CalcPointJacobian6D(*js_rbdl.Robot,q_hubo2,js_rbdl.n_lkn,box_center2_local + temp_cp2,temp_J2,1); // temp_cp2 -> local coordinate

            temp_cp1 = box_rot1*temp_cp1 + box_center1_global;
            temp_cp2 = box_rot2*temp_cp2 + box_center2_global; // temp_cp1, temp_cp2 --> global coordinate

            d17 = ((temp_cp1 - temp_cp2).norm() - dist_margin)/dt;
            n = (temp_cp1 - temp_cp2) / (temp_cp1 - temp_cp2).norm();
            N17 = n.transpose()*(temp_J2.block(3,0,3,n_dof) - temp_J1.block(3,0,3,n_dof));


            //right leg link 3 <-> left leg link 3

            box_center1_local << 30,0,ar2ee(2,0);
            box_center1_global = CalcBodyToBaseCoordinates(*js_rbdl.Robot, q_hubo2, js_rbdl.n_rar,box_center1_local,1);
            box_rot1 = CalcBodyWorldOrientation(*js_rbdl.Robot, q_hubo2, js_rbdl.n_rar, 1).inverse();
            box_length1 << 220,130,-ar2ee(2,0);

            box_center2_local << 30,0,ar2ee(2,0);
            box_center2_global = CalcBodyToBaseCoordinates(*js_rbdl.Robot, q_hubo2, js_rbdl.n_lar,box_center1_local,1);
            box_rot2 = CalcBodyWorldOrientation(*js_rbdl.Robot, q_hubo2, js_rbdl.n_lar, 1).inverse();
            box_length2 << 220,130,-ar2ee(2,0);


            find_cloest_points_box_box(box_center1_global,box_rot1,box_length1(0,0),box_length1(1,0),box_length1(2,0),box_center2_global,box_rot2,box_length2(0,0),box_length2(1,0),box_length2(2,0),temp_cp1,temp_cp2);

            temp_J1 = MatrixNd::Zero(6,n_dof);
            temp_J2 = MatrixNd::Zero(6,n_dof);

            CalcPointJacobian6D(*js_rbdl.Robot,q_hubo2,js_rbdl.n_rar,box_center1_local + temp_cp1,temp_J1,1); // temp_cp2 -> local coordinate
            CalcPointJacobian6D(*js_rbdl.Robot,q_hubo2,js_rbdl.n_lar,box_center2_local + temp_cp2,temp_J2,1); // temp_cp2 -> local coordinate

            temp_cp1 = box_rot1*temp_cp1 + box_center1_global;
            temp_cp2 = box_rot2*temp_cp2 + box_center2_global; // temp_cp1, temp_cp2 --> global coordinate

            d18 = ((temp_cp1 - temp_cp2).norm() - dist_margin)/dt;
            n = (temp_cp1 - temp_cp2) / (temp_cp1 - temp_cp2).norm();
            N18 = n.transpose()*(temp_J2.block(3,0,3,n_dof) - temp_J1.block(3,0,3,n_dof));
            //////
            //make D and N matrix

            D(0,0) = d1;
            D(1,0) = d2;
            D(2,0) = d3;
            D(3,0) = d4;
            D(4,0) = d5;
            D(5,0) = d6;
            D(6,0) = d7;
            D(7,0) = d8;
            D(8,0) = d9;
            D(9,0) = d10;
            D(10,0) = d11;
            D(11,0) = d12;
            D(12,0) = d13;
            D(13,0) = d14;
            D(14,0) = d15;
            D(15,0) = d16;
            D(16,0) = d17;
            D(17,0) = d18;

            N.block(0,0,1,n_dof) = N1;
            N.block(1,0,1,n_dof) = N2;
            N.block(2,0,1,n_dof) = N3;
            N.block(3,0,1,n_dof) = N4;
            N.block(4,0,1,n_dof) = N5;
            N.block(5,0,1,n_dof) = N6;
            N.block(6,0,1,n_dof) = N7;
            N.block(7,0,1,n_dof) = N8;
            N.block(8,0,1,n_dof) = N9;
            N.block(9,0,1,n_dof) = N10;
            N.block(10,0,1,n_dof) = N11;
            N.block(11,0,1,n_dof) = N12;
            N.block(12,0,1,n_dof) = N13;
            N.block(13,0,1,n_dof) = N14;
            N.block(14,0,1,n_dof) = N15;
            N.block(15,0,1,n_dof) = N16;
            N.block(16,0,1,n_dof) = N17;
            N.block(17,0,1,n_dof) = N18;

            //cout << "FCL time : " << (rt_timer_read() - t1)/1000./1000. << endl;

            //////////////////////////
            //for limit limits, upper and lower limit, joint velocity level

            generate_jointlimit_hubo2(q_hubo2,qmin_hubo2,qmax_hubo2,dqmax_hubo2,ddqmax_hubo2,dqlb_hubo2,dqub_hubo2);

            //            //cout << " lb : " << endl << dqlb_hubo2 << endl;
            //            //cout << " ub : " << endl << dqub_hubo2 << endl;
            //            //cout << "test //cout,, finish generate joint limit" << endl;


            //            Aineq_ub.block(6,6,n_dof-6-1,n_dof-6-1) = MatrixNd::Identity(n_dof-6-1,n_dof-6-1);
            Aineq_ub = MatrixNd::Identity(n_dof,n_dof);
            Bineq_ub = dqub_hubo2;


            //            Aineq_lb.block(6,6,n_dof-6-1,n_dof-6-1) = -1*MatrixNd::Identity(n_dof-6-1,n_dof-6-1);
            Aineq_lb = -1*MatrixNd::Identity(n_dof,n_dof);
            Bineq_lb = -1.0*dqlb_hubo2;

            //            //cout << "test //cout,, make inequality ub lb" << endl;


            Aineq_a.block(0,0,n_dof,n_dof) = Aineq_ub;
            Aineq_a.block(n_dof,0,n_dof,n_dof) = Aineq_lb;
            Aineq_a.block(n_dof*2,0,n_col,n_dof) = N;

            Bineq_a.block(0,0,n_dof,1) = Bineq_ub;
            Bineq_a.block(n_dof,0,n_dof,1) = Bineq_lb;
            Bineq_a.block(n_dof*2,0,n_col,1) = D;

            //            //cout << "test //cout,, make inequality total" << endl;

            //Make A matrix

            CalcPointJacobian6D(*js_rbdl.Robot,q_hubo2,js_rbdl.n_rwp,wp2ee,js_rbdl.jacob_arm_right,1);
            CalcPointJacobian6D(*js_rbdl.Robot,q_hubo2,js_rbdl.n_lwp,wp2ee,js_rbdl.jacob_arm_left,1);
            CalcPointJacobian6D(*js_rbdl.Robot,q_hubo2,js_rbdl.n_rar,ar2ee,js_rbdl.jacob_leg_right,1);
            CalcPointJacobian6D(*js_rbdl.Robot,q_hubo2,js_rbdl.n_lar,ar2ee,js_rbdl.jacob_leg_left,1);
            CalcPointJacobian6D(*js_rbdl.Robot,q_hubo2,js_rbdl.n_torso,zv,js_rbdl.jacob_torso,1);
            CalcPointJacobian6D(*js_rbdl.Robot,q_hubo2,js_rbdl.n_pel,zv,js_rbdl.jacob_pel,1);

            CalcPointJacobian6D(*js_rbdl.Robot,q_hubo2,js_rbdl.n_torso,torso2rsp,temp_jacob,1);
            A_behavior.block(0,0,6,n_dof) = js_rbdl.jacob_arm_right - temp_jacob;

            CalcPointJacobian6D(*js_rbdl.Robot,q_hubo2,js_rbdl.n_torso,torso2lsp,temp_jacob,1);
            A_behavior.block(6,0,6,n_dof) = js_rbdl.jacob_arm_left - temp_jacob;

            A_behavior.block(12,0,6,n_dof) = js_rbdl.jacob_leg_right;
            A_behavior.block(18,0,6,n_dof) = js_rbdl.jacob_leg_left;
            //            //cout << "jacob leg left : " << endl << js_rbdl.jacob_leg_left << endl;

            A_behavior.block(24,0,2,n_dof) = js_rbdl.jacob_com.block(0,0,2,n_dof);
            A_behavior.block(26,0,3,n_dof) = js_rbdl.jacob_torso.block(0,0,3,n_dof);
            A_behavior.block(29,0,3,n_dof) = js_rbdl.jacob_pel.block(0,0,3,n_dof);
            A_behavior.block(32,0,1,n_dof) = js_rbdl.jacob_pel.block(5,0,1,n_dof);

            A_behavior.block(0,0,3,n_dof) *= 5;     //right arm ori
            A_behavior.block(3,0,3,n_dof) *= 0.25;     //right arm pos
            A_behavior.block(6,0,3,n_dof) *= 5;     //left arm ori
            A_behavior.block(9,0,3,n_dof) *= 0.25;     //left arm pos
            A_behavior.block(12,0,3,n_dof) *= 1000;    //right leg ori
            A_behavior.block(15,0,3,n_dof) *= 10;    //right leg pos
            A_behavior.block(18,0,3,n_dof) *= 1000;    //left leg ori
            A_behavior.block(21,0,3,n_dof) *= 10;    //left leg pos
            A_behavior.block(24,0,2,n_dof) *= 1.0;    //COM
            A_behavior.block(26,0,3,n_dof) *= 500;    //WST orientation
            A_behavior.block(29,0,3,n_dof) *= 1000;    //PEL orientation
            A_behavior.block(32,0,1,n_dof) *= 0.5;    //pel z position

            //Make B matrix

            B_behavior.block(0,0,3,1) = err_ori_RH*5;   //right arm ori
            B_behavior.block(3,0,3,1) = err_pos_RH*0.25;   //right arm pos
            B_behavior.block(6,0,3,1) = err_ori_LH*5;   //left arm ori
            B_behavior.block(9,0,3,1) = err_pos_LH*0.25;   //left arm pos
            B_behavior.block(12,0,3,1) = err_ori_RF*1000; //right leg ori
            B_behavior.block(15,0,3,1) = err_pos_RF*10;  //right leg pos
            B_behavior.block(18,0,3,1) = err_ori_LF*1000; //left leg ori
            B_behavior.block(21,0,3,1) = err_pos_LF*10;  //left leg pos
            B_behavior.block(24,0,2,1) = err_com.block(0,0,2,1)*1.0;    //COM
            B_behavior.block(26,0,3,1) = err_ori_WST*500;                //WST orientation
            B_behavior.block(29,0,3,1) = err_ori_PEL*1000;                //PEL orientation
            B_behavior.block(32,0,1,1) = err_pos_PELZ*0.5;                //pel z position

            double Kp = 10;

            B_behavior *= Kp;

            t1 = rt_timer_read();
            //Solve QP
            js_qp.setNums(n_dof,0,n_dof*2+n_col);
            js_qp.make_HF(A_behavior,B_behavior);
            js_qp.make_IEQ(Aineq_a,Bineq_a);

            //            //cout << "test //cout,, make HJ and IEQ" << endl;

            dq_hubo2 = js_qp.solve_QP();

            //cout << "QP time : " << (rt_timer_read() - t1)/1000./1000. << endl;

            //            //cout << "err : " << endl << err << endl;
            //            //cout << "solve of QP: " << endl << dq_hubo2 << endl;
            //            //cout << "idx: " << idx_save << ", hubo2 q : " << endl << q_hubo2 << endl;
//            //cout << "COM : " << endl << COM << endl;
//            //cout << "pel pos : " << endl << CalcBodyToBaseCoordinates(*js_rbdl.Robot, q_hubo2, js_rbdl.n_pel,zv,1) << endl;

            //            //cout << "m_robot : " << endl << m_robot << endl;
            //            //cout << "com jacob : " << endl << js_rbdl.jacob_com << endl;

            //            //cout << "js_rbdl.jacob_leg_right : " << js_rbdl.jacob_leg_right.rows() << " x "<< js_rbdl.jacob_leg_right.cols() << endl << js_rbdl.jacob_leg_right << endl;
            //            //cout << "js_rbdl.jacob_leg_left : " << js_rbdl.jacob_leg_left.rows() << " x "<< js_rbdl.jacob_leg_left.cols() << endl << js_rbdl.jacob_leg_left << endl;

            for(int i=0;i<n_dof;i++)
                if(isnan(dq_hubo2(i,0)))
                {
                    //cout << "NAN! NAN! NAN! NAN! " << endl;
                    exit(0);
                }




            //motion smooth
            if(time_init_to_motion < 600)
            {
                for(int i=6;i<6+24+1;i++)
                {
                    dq_hubo2(i,0) *= time_init_to_motion/600.0;
                }

                time_init_to_motion++;
            }


            //update joint angle
            q_hubo2.block(0,0,3,1)  += dq_hubo2.block(0,0,3,1)*dt_hubo2;
            q_hubo2.block(6,0,25,1) += dq_hubo2.block(6,0,25,1)*dt_hubo2;

            Math::Quaternion qPel = update_quaternion(Math::Quaternion(q_hubo2(3,0),q_hubo2(4,0),q_hubo2(5,0),q_hubo2(js_rbdl.Robot->q_size-1)),dq_hubo2.block(3,0,3,1));
            Math::VectorNd tempq = q_hubo2;
            js_rbdl.Robot->SetQuaternion(js_rbdl.n_pel,qPel,tempq);
            q_hubo2 = tempq;

            //low pass filtering

            double alpha_lfp = 0.5;
            q_hubo2_filtered.block(0,0,3,1) *= alpha_lfp;
            q_hubo2_filtered.block(0,0,3,1) += (1.0 - alpha_lfp)*q_hubo2.block(0,0,3,1);
            q_hubo2.block(0,0,3,1) = q_hubo2_filtered.block(0,0,3,1);

            q_hubo2_filtered.block(6,0,25,1) *= alpha_lfp;
            q_hubo2_filtered.block(6,0,25,1) += (1.0 - alpha_lfp)*q_hubo2.block(6,0,25,1);
            q_hubo2.block(6,0,25,1) = q_hubo2_filtered.block(6,0,25,1);


            jCon->SetMoveJoint(WST, q_hubo2(6,0)*180.0/PI, RT_TIMER_PERIOD_MS, MOVE_ABSOLUTE);

            jCon->SetMoveJoint(RSP, q_hubo2(7,0)*180.0/PI, RT_TIMER_PERIOD_MS, MOVE_ABSOLUTE);
            jCon->SetMoveJoint(RSR, 15.0 + q_hubo2(8,0)*180.0/PI, RT_TIMER_PERIOD_MS, MOVE_ABSOLUTE);
            jCon->SetMoveJoint(RSY, q_hubo2(9,0)*180.0/PI, RT_TIMER_PERIOD_MS, MOVE_ABSOLUTE);
            jCon->SetMoveJoint(REB, 20.0 + q_hubo2(10,0)*180.0/PI, RT_TIMER_PERIOD_MS, MOVE_ABSOLUTE);
            jCon->SetMoveJoint(RWY, q_hubo2(11,0)*180.0/PI, RT_TIMER_PERIOD_MS, MOVE_ABSOLUTE);
            jCon->SetMoveJoint(RWP, q_hubo2(12,0)*180.0/PI, RT_TIMER_PERIOD_MS, MOVE_ABSOLUTE);

            jCon->SetMoveJoint(LSP, q_hubo2(13,0)*180.0/PI, RT_TIMER_PERIOD_MS, MOVE_ABSOLUTE);
            jCon->SetMoveJoint(LSR, -15.0 + q_hubo2(14,0)*180.0/PI, RT_TIMER_PERIOD_MS, MOVE_ABSOLUTE);
            jCon->SetMoveJoint(LSY, q_hubo2(15,0)*180.0/PI, RT_TIMER_PERIOD_MS, MOVE_ABSOLUTE);
            jCon->SetMoveJoint(LEB, 20.0 + q_hubo2(16,0)*180.0/PI, RT_TIMER_PERIOD_MS, MOVE_ABSOLUTE);
            jCon->SetMoveJoint(LWY, q_hubo2(17,0)*180.0/PI, RT_TIMER_PERIOD_MS, MOVE_ABSOLUTE);
            jCon->SetMoveJoint(LWP, q_hubo2(18,0)*180.0/PI, RT_TIMER_PERIOD_MS, MOVE_ABSOLUTE);

            jCon->SetMoveJoint(RHY, q_hubo2(19,0)*180.0/PI, RT_TIMER_PERIOD_MS, MOVE_ABSOLUTE);
            jCon->SetMoveJoint(RHR, q_hubo2(20,0)*180.0/PI, RT_TIMER_PERIOD_MS, MOVE_ABSOLUTE);
            jCon->SetMoveJoint(RHP, q_hubo2(21,0)*180.0/PI, RT_TIMER_PERIOD_MS, MOVE_ABSOLUTE);
            jCon->SetMoveJoint(RKN, q_hubo2(22,0)*180.0/PI, RT_TIMER_PERIOD_MS, MOVE_ABSOLUTE);
            jCon->SetMoveJoint(RAP, q_hubo2(23,0)*180.0/PI, RT_TIMER_PERIOD_MS, MOVE_ABSOLUTE);
            jCon->SetMoveJoint(RAR, q_hubo2(24,0)*180.0/PI, RT_TIMER_PERIOD_MS, MOVE_ABSOLUTE);

            jCon->SetMoveJoint(LHY, q_hubo2(25,0)*180.0/PI, RT_TIMER_PERIOD_MS, MOVE_ABSOLUTE);
            jCon->SetMoveJoint(LHR, q_hubo2(26,0)*180.0/PI, RT_TIMER_PERIOD_MS, MOVE_ABSOLUTE);
            jCon->SetMoveJoint(LHP, q_hubo2(27,0)*180.0/PI, RT_TIMER_PERIOD_MS, MOVE_ABSOLUTE);
            jCon->SetMoveJoint(LKN, q_hubo2(28,0)*180.0/PI, RT_TIMER_PERIOD_MS, MOVE_ABSOLUTE);
            jCon->SetMoveJoint(LAP, q_hubo2(29,0)*180.0/PI, RT_TIMER_PERIOD_MS, MOVE_ABSOLUTE);
            jCon->SetMoveJoint(LAR, q_hubo2(30,0)*180.0/PI, RT_TIMER_PERIOD_MS, MOVE_ABSOLUTE);

//            //cout << "q_hubo2 " << endl << q_hubo2 << endl;

            for(int i=0;i<25;i++)
            {
                sharedData->q[i] = q_hubo2(i+6,0);
            }

            sharedData->q[25] = 0;
            sharedData->q[26] = 0;


            for(int i=0;i<n_dof;i++)
                DataBuf[i][idx_save] = q_hubo2(i,0);

            //DataBuf[n_dof][idx_save] = sharedData->FT[0].Pitch;
            //DataBuf[n_dof+1][idx_save] = sharedData->FT[0].Roll;

            idx_save++;

            if(idx_save >= 200*30)
                idx_save = 0;

            //cout << "rt end" << endl;

            //cout << "loop time : " << rt_timer_read() - t_start << endl;

        }


        rt_count++;

        if(rt_count > 10000000)
            rt_count = 10000000;


        jCon->MoveAllJoint();
        rt_task_suspend(&rtTaskCon);
    }
}
//==============================//



//==============================//
// Flag Thread
//==============================//
void RBFlagThread(void *)
{
    rt_task_set_periodic(NULL, TM_NOW, 300*1000);        // 300 usec

    while(__IS_WORKING)
    {
        rt_task_wait_period(NULL);

        //if(1){
        if(HasAnyOwnership()){
            if(sharedData->SYNC_SIGNAL[PODO_NO] == true){
                jCon->JointUpdate();
                rt_task_resume(&rtTaskCon);
            }
        }
    }
}
//==============================//
