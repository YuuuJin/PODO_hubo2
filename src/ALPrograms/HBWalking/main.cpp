
#include "BasicFiles/BasicSetting.h"

#include <iostream>
#include <string>
#include <stdio.h>

#include "HB_functions.h"
#include "taskmotion.h"
#include "HB_inverse.h"
#include "HB_PreviewWalk.h"
#include <timer.h>


#define SAVEN       280


#define PODO_AL_NAME       "HBWalking"

using namespace std;


// Basic --------
pRBCORE_SHM             sharedData;
pUSER_SHM               userData;
JointControlClass       *joint;


int     __IS_WORKING = false;
int     __IS_GAZEBO = false;
int     PODO_NO = -1;

//HB_DynamicWalk  HBD;
//HB_DynamicWalk2 HBD2;
//DesiredStates   DS;
//RobotStates     RST, RST_ini;
//RobotSensor     RSEN;
//REFERENCE       REF;
//BP_RBDL         bp_rbdl;
HB_PreviewWalk  HBPW;
HB_inverse      Hi;


TaskMotion      *WBmotion;
int             WB_FLAG = 0;
static const double dt = 0.005;

//Gazelle_Kine GK;

RState RS_temp;



//SYS ID
double      InputFreq;
double      InputAmp;
vec3        Initial_COM;
vec3        Initial_pRF;
vec3        Initial_pLF;
vec3        ctrl_COM;
double      z_ctrl;
unsigned int SysIDcnt;
unsigned int SysIDcnt_init;
double Fz_diff_error_old = 0;

int input_Amp;
vec4 input_torque_filtered = vec4(0,0,0,0);

bool AnkleTorqueControl_flag = false;
bool Choreonoid_flag = false;

// Hip roll vib control
double RHR_con_deg = 0;
double LHR_con_deg = 0;
double RHY_con_deg = 0;
double LHY_con_deg = 0;
double LHP_con_deg = 0;
double RHP_con_deg = 0;
double LKN_con_deg = 0;
double RKN_con_deg = 0;

//Ankle angle
double RAR_ang_old = 0;
double RAP_ang_old = 0;
double LAR_ang_old = 0;
double LAP_ang_old = 0;

//torso Orientation control
double RHR_comp_Ang = 0;
double LHR_comp_Ang = 0;

//Ankle angle
double RA1_ref_deg, RA2_ref_deg, LA1_ref_deg, LA2_ref_deg;

// waste momentum compensation
double WST_ref_deg = 0;
double NeckYaw_ref_deg = 0;
double RSP_ref_deg = 0;
double LSP_ref_deg = 0;
double RSP_ref_deg_offset = 0;
double LSP_ref_deg_offset = 0;

// Ready To walk mode
vec3 COM_ini_global, pRF_ini_global, pLF_ini_global;

//Ankle angle control
vec3 RF_angle_con_deg, LF_angle_con_deg;

// State Estimator
vec3 pel_estimated;

//SAVE
int Scnt = 0;
double SAVE[SAVEN][300000];

// Functions
void ResetGlobalCoord(int RF_OR_LF_OR_PC_MidFoot);
void SensorInput();
double getEnc(int in);
double getPosRef(int in);
double getEncVel(int in);
void save_onestep(int cnt);
void save_all();
void ArmGainOverride(void);
void ArmGainOverride_Off(void);

doubles quat2doubles(quat _quat);

enum HBWalking_COMMAND
{
    HBWalking_NO_ACT = 100,
    HBWalking_DATA_SAVE,
    HBWalking_STOP,
    HBWalking_PrevWalk,
    HBWalking_Test,
    HBWalking_JoyStick_Walk_Stop,
    HBWalking_Ready_To_Walk,
    HBWalking_Clear_Controller,
};

enum task_thread
{
    _task_Idle = 0,
    _task_HB_PrevWalk,
    _task_HB_test,
    _task_Ready_To_Walk,

}_task_thread;

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
    sprintf(__AL_NAME, "HBWalking");


    CheckArguments(argc, argv);
//    cout<<"podo no: "<<PODO_NO<<endl;
    if(PODO_NO == -1){
        FILE_LOG(logERROR) << "Please check the AL Number";
        FILE_LOG(logERROR) << "Terminate this AL..";
        return 0;
    }
//    PODO_NO = 7; // for debug mode

    // Initialize RBCore -----------------------------------
    if(RBInitialize() == false)
        __IS_WORKING = false;

    joint->SetMotionOwner(0);

//    HBPW.sharedCMD = sharedCMD;
//    HBPW.sharedREF = sharedREF;
//    HBPW.sharedSEN = sharedSEN;
    HBPW.sharedData = sharedData;
    HBPW.userData = userData;



////point1: 0.108881, -0.230622       q
////point2: -0.105808, -0.192767      p
////point3: -0.0801077, -0.0470154
////point4: 0.100406, -0.278684       r
////point5: 0.074706, -0.424436
////point6: -0.139982, -0.38658
////point7: -0.114282, -0.240829
////point8: -0.0848707, 3.46044e-320

//    Point points[] = {{0.108881, -0.230622}, {-0.105808, -0.192767}, {-0.0801077, -0.0470154}, {0.100406, -0.278684},
//                            {0.074706, -0.424436}, {-0.139982, -0.38658}, {-0.114282, -0.240829}, {-0.0848707, 3.46044e-320}};

//    int n_final = 8;
//    stack<Point> SP = convexHull(points,8, n_final);

//    cout<<"done"<<endl;



    // WBIK Initialize
    WBmotion = new TaskMotion(sharedData, joint);

    while(__IS_WORKING){
        usleep(100*1000);

        switch(sharedData->COMMAND[PODO_NO].USER_COMMAND){
        case HBWalking_DATA_SAVE:
        {
            sharedData->COMMAND[PODO_NO].USER_COMMAND = HBWalking_NO_ACT;
            save_all();

            break;
        }

        case HBWalking_STOP:
        {
            sharedData->COMMAND[PODO_NO].USER_COMMAND = HBWalking_NO_ACT;
            _task_thread = _task_Idle;

            ArmGainOverride_Off();

            WB_FLAG = 0;
            save_all();

            break;
        }

        case HBWalking_Test:
        {
            sharedData->COMMAND[PODO_NO].USER_COMMAND = HBWalking_NO_ACT;
            _task_thread = _task_Idle;

            cout<<"Logging start!"<<endl;

            joint->RefreshToCurrentReference();
            joint->SetAllMotionOwner();

            WBmotion->MomentFlag = false;
            WB_FLAG = 0;

            WBmotion->StopAll();
            WBmotion->RefreshToCurrentReference_HB(RF_angle_con_deg.x, RF_angle_con_deg.y, LF_angle_con_deg.x, LF_angle_con_deg.y);

            cout<<"Initial Task position: "<<endl;
            cout<<"COM.x : "<<WBmotion->pCOM_3x1[0]<<"   COM.y : "<<WBmotion->pCOM_3x1[1]<<"  COM.z : "<<WBmotion->pCOM_3x1[2]<<endl;
            cout<<"pel.x : "<<WBmotion->pPel_3x1[0]<<"   pel.y : "<<WBmotion->pPel_3x1[1]<<"  pel.z : "<<WBmotion->pPel_3x1[2]<<endl;
            cout<<"LF.x : "<<WBmotion->pLF_3x1[0]<<"   LF.y : "<<WBmotion->pLF_3x1[1]<<"  LF.z : "<<WBmotion->pLF_3x1[2]<<endl;
            cout<<"RF.x : "<<WBmotion->pRF_3x1[0]<<"   RF.y : "<<WBmotion->pRF_3x1[1]<<"  RF.z : "<<WBmotion->pRF_3x1[2]<<endl;

            ResetGlobalCoord(1);
            usleep(200*1000);

            cout<<"After ResetGlobal"<<endl;
            cout<<"COM.x : "<<WBmotion->pCOM_3x1[0]<<"   COM.y : "<<WBmotion->pCOM_3x1[1]<<"  COM.z : "<<WBmotion->pCOM_3x1[2]<<endl;
            cout<<"pel.x : "<<WBmotion->pPel_3x1[0]<<"   pel.y : "<<WBmotion->pPel_3x1[1]<<"  pel.z : "<<WBmotion->pPel_3x1[2]<<endl;
            cout<<"LF.x : "<<WBmotion->pLF_3x1[0]<<"   LF.y : "<<WBmotion->pLF_3x1[1]<<"  LF.z : "<<WBmotion->pLF_3x1[2]<<endl;
            cout<<"RF.x : "<<WBmotion->pRF_3x1[0]<<"   RF.y : "<<WBmotion->pRF_3x1[1]<<"  RF.z : "<<WBmotion->pRF_3x1[2]<<endl;

            vec3 COM_ini = vec3(WBmotion->pCOM_3x1);
            vec3 pRF = vec3(WBmotion->pRF_3x1);
            vec3 pLF = vec3(WBmotion->pLF_3x1);
            quat qRF = quat(WBmotion->qRF_4x1);
            quat qLF = quat(WBmotion->qLF_4x1);



            HBPW.Test_init(COM_ini, pRF, qRF, pLF, qLF);

            ArmGainOverride();

            SensorInput();
            HBPW.MeasurementInput(RS_temp);

            _task_thread = _task_HB_test;
            WB_FLAG = 1;

            break;

        }
        case HBWalking_PrevWalk:
        {
            sharedData->COMMAND[PODO_NO].USER_COMMAND = HBWalking_NO_ACT;
            _task_thread = _task_Idle;

            //IMU Reset
            sharedData->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[0] = 3;
            sharedData->COMMAND[RBCORE_PODO_NO].USER_COMMAND = DAEMON_SENSOR_IMU_NULL;

//            joint->RefreshToCurrentPosition();
            joint->RefreshToCurrentReference();
            joint->SetAllMotionOwner();

            WBmotion->MomentFlag = false;
            WB_FLAG = 0;

            WBmotion->StopAll();
            WBmotion->RefreshToCurrentReference_HB(RF_angle_con_deg.x, RF_angle_con_deg.y, LF_angle_con_deg.x, LF_angle_con_deg.y);

            cout<<"Initial Task position: "<<endl;
            cout<<"COM.x : "<<WBmotion->pCOM_3x1[0]<<"   COM.y : "<<WBmotion->pCOM_3x1[1]<<"  COM.z : "<<WBmotion->pCOM_3x1[2]<<endl;
            cout<<"pel.x : "<<WBmotion->pPel_3x1[0]<<"   pel.y : "<<WBmotion->pPel_3x1[1]<<"  pel.z : "<<WBmotion->pPel_3x1[2]<<endl;
            cout<<"LF.x : "<<WBmotion->pLF_3x1[0]<<"   LF.y : "<<WBmotion->pLF_3x1[1]<<"  LF.z : "<<WBmotion->pLF_3x1[2]<<endl;
            cout<<"RF.x : "<<WBmotion->pRF_3x1[0]<<"   RF.y : "<<WBmotion->pRF_3x1[1]<<"  RF.z : "<<WBmotion->pRF_3x1[2]<<endl;

            ArmGainOverride();
            ResetGlobalCoord(1);
            usleep(200*1000);

            cout<<"After ResetGlobal"<<endl;
            cout<<"COM.x : "<<WBmotion->pCOM_3x1[0]<<"   COM.y : "<<WBmotion->pCOM_3x1[1]<<"  COM.z : "<<WBmotion->pCOM_3x1[2]<<endl;
            cout<<"pel.x : "<<WBmotion->pPel_3x1[0]<<"   pel.y : "<<WBmotion->pPel_3x1[1]<<"  pel.z : "<<WBmotion->pPel_3x1[2]<<endl;
            cout<<"LF.x : "<<WBmotion->pLF_3x1[0]<<"   LF.y : "<<WBmotion->pLF_3x1[1]<<"  LF.z : "<<WBmotion->pLF_3x1[2]<<endl;
            cout<<"RF.x : "<<WBmotion->pRF_3x1[0]<<"   RF.y : "<<WBmotion->pRF_3x1[1]<<"  RF.z : "<<WBmotion->pRF_3x1[2]<<endl;

            vec3 COM_ini = vec3(WBmotion->pCOM_3x1);
            vec3 pRF = vec3(WBmotion->pRF_3x1);
            vec3 pLF = vec3(WBmotion->pLF_3x1);

            quat qPel_ini = quat(WBmotion->qPEL_4x1);
            quat qRF = quat(WBmotion->qRF_4x1);
            quat qLF = quat(WBmotion->qLF_4x1);

            int no_of_step = sharedData->COMMAND[PODO_NO].USER_PARA_INT[0];
            double t_step = sharedData->COMMAND[PODO_NO].USER_PARA_DOUBLE[0];
            double step_stride = sharedData->COMMAND[PODO_NO].USER_PARA_DOUBLE[1];



            double postime = 400.0;

            double RSP_ref,RSR_ref,RSY_ref,REB_ref,RWY_ref,RWP_ref,LSP_ref,LSR_ref,LSY_ref,LEB_ref,LWY_ref,LWP_ref;
            double WST_ref, RWY2_ref, LWY2_ref;

            RSP_ref = 30.0;
            RSR_ref = 0.0;
            RSY_ref = 0.0;
            REB_ref = -90;
            RWY_ref = 0.0;
            RWP_ref = 0.0;

            LSP_ref = 30.0;
            LSR_ref = 0.0;
            LSY_ref = 0.0;
            LEB_ref = -90;
            LWY_ref = 0.0;
            LWP_ref = 0.0;

            WST_ref = 0.0;

            RWY2_ref = 0.0;
            LWY2_ref = 0.0;

            joint->SetMoveJoint(RSP, RSP_ref, postime, MOVE_ABSOLUTE);
            joint->SetMoveJoint(RSR, RSR_ref, postime, MOVE_ABSOLUTE);
            joint->SetMoveJoint(RSY, RSY_ref, postime, MOVE_ABSOLUTE);
            joint->SetMoveJoint(REB, REB_ref, postime, MOVE_ABSOLUTE);
            joint->SetMoveJoint(RWY, RWY_ref, postime, MOVE_ABSOLUTE);
            joint->SetMoveJoint(RWP, RWP_ref, postime, MOVE_ABSOLUTE);

            joint->SetMoveJoint(LSP, LSP_ref, postime, MOVE_ABSOLUTE);
            joint->SetMoveJoint(LSR, LSR_ref, postime, MOVE_ABSOLUTE);
            joint->SetMoveJoint(LSY, LSY_ref, postime, MOVE_ABSOLUTE);
            joint->SetMoveJoint(LEB, LEB_ref, postime, MOVE_ABSOLUTE);
            joint->SetMoveJoint(LWY, LWY_ref, postime, MOVE_ABSOLUTE);
            joint->SetMoveJoint(LWP, LWP_ref, postime, MOVE_ABSOLUTE);

            // preview Gain load
            HBPW.PreviewGainLoad(HBPW.zc);

            HBPW.HB_set_step(COM_ini, qPel_ini, pRF, qRF, pLF, qLF, WST_ref_deg, t_step, no_of_step, step_stride, 1);

            cout<<"foot print & timing : total "<<HBPW.SDB.size()<<"steps "<<endl;

            for(int i=HBPW.SDB.size()-1;i>=0;i--){
                cout<<"phase: "<<i<<"  "<<HBPW.SDB[i].Fpos.x<<", "<<HBPW.SDB[i].Fpos.y<<" ("<<HBPW.SDB[i].t<<"s)"<<endl;
            }

            SensorInput();
            HBPW.MeasurementInput(RS_temp);

            WB_FLAG = 1;


            if(sharedData->COMMAND[PODO_NO].USER_PARA_CHAR[15] == 10){
                HBPW.Joystick_walk_flag = true;
                HBPW.Joystick_on_signal = true;

                sharedData->COMMAND[PODO_NO].USER_PARA_INT[0] = 0;
                sharedData->COMMAND[PODO_NO].USER_PARA_INT[1] = 0;
                sharedData->COMMAND[PODO_NO].USER_PARA_INT[2] = 0;
                sharedData->COMMAND[PODO_NO].USER_PARA_INT[3] = 0;
                sharedData->COMMAND[PODO_NO].USER_PARA_INT[4] = 0;
                sharedData->COMMAND[PODO_NO].USER_PARA_INT[5] = 0;
            }

            userData->M2G.Walking_state = true;

            // Waste angle initialize
            WST_ref_deg = sharedData->ENCODER[MC_GetID(WST)][MC_GetCH(WST)].CurrentReference;
            NeckYaw_ref_deg = sharedData->ENCODER[MC_GetID(NKY)][MC_GetCH(NKY)].CurrentReference;
            RSP_ref_deg_offset = 30;//sharedData->ENCODER[MC_GetID(RSP)][MC_GetCH(RSP)].CurrentReference;
            LSP_ref_deg_offset = 30;//sharedData->ENCODER[MC_GetID(LSP)][MC_GetCH(LSP)].CurrentReference;

            _task_thread = _task_HB_PrevWalk;

            break;
        }
        case HBWalking_Ready_To_Walk:
        {
            sharedData->COMMAND[PODO_NO].USER_COMMAND = HBWalking_NO_ACT;
            _task_thread = _task_Idle;

            cout<<"Logging start!"<<endl;

            joint->RefreshToCurrentReference();
            joint->SetAllMotionOwner();

            cout<<"State Estimation Start"<<endl;

            WBmotion->MomentFlag = false;
            WB_FLAG = 0;

            WBmotion->StopAll();
            WBmotion->RefreshToCurrentReference_HB(RF_angle_con_deg.x, RF_angle_con_deg.y, LF_angle_con_deg.x, LF_angle_con_deg.y);

            cout<<"Initial Task position: "<<endl;
            cout<<"COM.x : "<<WBmotion->pCOM_3x1[0]<<"   COM.y : "<<WBmotion->pCOM_3x1[1]<<"  COM.z : "<<WBmotion->pCOM_3x1[2]<<endl;
            cout<<"pel.x : "<<WBmotion->pPel_3x1[0]<<"   pel.y : "<<WBmotion->pPel_3x1[1]<<"  pel.z : "<<WBmotion->pPel_3x1[2]<<endl;
            cout<<"LF.x : "<<WBmotion->pLF_3x1[0]<<"   LF.y : "<<WBmotion->pLF_3x1[1]<<"  LF.z : "<<WBmotion->pLF_3x1[2]<<endl;
            cout<<"RF.x : "<<WBmotion->pRF_3x1[0]<<"   RF.y : "<<WBmotion->pRF_3x1[1]<<"  RF.z : "<<WBmotion->pRF_3x1[2]<<endl;

            ResetGlobalCoord(1);
            usleep(200*1000);

            cout<<"After ResetGlobal"<<endl;
            cout<<"COM.x : "<<WBmotion->pCOM_3x1[0]<<"   COM.y : "<<WBmotion->pCOM_3x1[1]<<"  COM.z : "<<WBmotion->pCOM_3x1[2]<<endl;
            cout<<"pel.x : "<<WBmotion->pPel_3x1[0]<<"   pel.y : "<<WBmotion->pPel_3x1[1]<<"  pel.z : "<<WBmotion->pPel_3x1[2]<<endl;
            cout<<"LF.x : "<<WBmotion->pLF_3x1[0]<<"   LF.y : "<<WBmotion->pLF_3x1[1]<<"  LF.z : "<<WBmotion->pLF_3x1[2]<<endl;
            cout<<"RF.x : "<<WBmotion->pRF_3x1[0]<<"   RF.y : "<<WBmotion->pRF_3x1[1]<<"  RF.z : "<<WBmotion->pRF_3x1[2]<<endl;

            vec3 COM_ini = vec3(WBmotion->pCOM_3x1);
            vec3 pRF = vec3(WBmotion->pRF_3x1);
            vec3 pLF = vec3(WBmotion->pLF_3x1);

            quat qPel_ini = quat(WBmotion->qPEL_4x1);
            quat qRF = quat(WBmotion->qRF_4x1);
            quat qLF = quat(WBmotion->qLF_4x1);

            COM_ini_global = COM_ini;
            pRF_ini_global = pRF;
            pLF_ini_global = pLF;

            int no_of_step = sharedData->COMMAND[PODO_NO].USER_PARA_INT[0];
            double t_step = sharedData->COMMAND[PODO_NO].USER_PARA_DOUBLE[0];
            double step_stride = sharedData->COMMAND[PODO_NO].USER_PARA_DOUBLE[1];

            HBPW.HB_set_step(COM_ini, qPel_ini, pRF, qRF, pLF, qLF, WST_ref_deg, t_step, no_of_step, step_stride, 1);

            // Waste angle initialize
            WST_ref_deg = sharedData->ENCODER[MC_GetID(WST)][MC_GetCH(WST)].CurrentPosition;

            // preview Gain load
            HBPW.PreviewGainLoad(HBPW.zc);

            SensorInput();
            HBPW.MeasurementInput(RS_temp);

            WB_FLAG = 1;

            _task_thread = _task_Ready_To_Walk;

            break;
        }


        case HBWalking_JoyStick_Walk_Stop:
        {
            sharedData->COMMAND[PODO_NO].USER_COMMAND = HBWalking_NO_ACT;
            HBPW.Joystick_off_signal = true;
            HBPW.del_pos = vec3();

            break;
        }

        case HBWalking_Clear_Controller:
        {
            sharedData->COMMAND[PODO_NO].USER_COMMAND = HBWalking_NO_ACT;

            WB_FLAG = 0;

            WST_ref_deg = 0;
            RSP_ref_deg = 0;
            LSP_ref_deg = 0;
            NeckYaw_ref_deg = 0;
            RF_angle_con_deg = vec3();
            LF_angle_con_deg = vec3();
            HBPW.RF_angle_ctrl = vec3();
            HBPW.LF_angle_ctrl = vec3();
            RHR_con_deg = 0;
            LHR_con_deg = 0;
            RHY_con_deg = 0;
            LHY_con_deg = 0;
            LHP_con_deg = 0;
            RHP_con_deg = 0;
            LKN_con_deg = 0;
            RKN_con_deg = 0;


            cout<<"Control Valure are Cleared"<<endl;
            break;
        }

        case 999:
            FILE_LOG(logSUCCESS) << "Command 999 received..";
            joint->RefreshToCurrentReference();
            joint->SetAllMotionOwner();
            joint->SetMoveJoint(WST, 30.0, 2000.0, MOVE_ABSOLUTE);
            sharedData->COMMAND[PODO_NO].USER_COMMAND = HBWalking_NO_ACT;
            break;
        default:
            sharedData->COMMAND[PODO_NO].USER_COMMAND = HBWalking_NO_ACT;
            break;
        }
    }

    FILE_LOG(logERROR) << "Process \"" << __AL_NAME << "\" is terminated" << endl;
    return 0;
}



////========================================================================================================================////
//// Task Thread
////========================================================================================================================//
////========================================================================================================================//
////========================================================================================================================//
////========================================================================================================================//
doubles dbs_qRF(4), dbs_qLF(4), dbs_qPel(4);
void RBTaskThread(void *)
{
    while(__IS_WORKING)
    {

        switch(_task_thread)
        {
        case _task_Ready_To_Walk:
        {
            SensorInput();
            HBPW.MeasurementInput(RS_temp);

            HBPW.k++;
            save_onestep(HBPW.k);


            //// -----------------------COM Damping Control
            HBPW.COM_ref = COM_ini_global;

            HBPW.DampingControl3();

            HBPW.uCOM = HBPW.COM_ref + HBPW.COM_damping_con;//HBPW.COM_ref;

//            if(HBPW.k % 10 == 0) cout<<"uCOMx: "<<HBPW.uCOM.x<<" uCOMy: "<<HBPW.uCOM.y<<endl;

//            cout<<"uCOMy: "<<HBPW.uCOM.y<<"  ZMPy : "<<HBPW.ZMP_global.y<<endl;

            HBPW.Standing_mode_flag = true;

            //// ---------------------- DSP_FZ control
            HBPW.t_now = 0;
            HBPW.RF_landing_flag = true;
            HBPW.LF_landing_flag = true;
            HBPW.step_phase = 0;

            HBPW.cZMP_dsp_global = vec3(0,0,0) + (1 + 4.0/HBPW.w)*((HBPW.COM_m_filtered + HBPW.dCOM_m_diff_filtered/HBPW.w));

            HBPW.cZMP_dsp_global_proj = zmpProjectionToSP_offset(HBPW.cZMP_dsp_global, HBPW.pRF_ref, HBPW.pLF_ref, HBPW.qRF_ref, HBPW.qLF_ref, 100, 100, -0.002, -0.002);

            HBPW.cZMP_dsp_local_proj = global2local_point(HBPW.qRF_ref, HBPW.qLF_ref, HBPW.pRF_ref, HBPW.pLF_ref, HBPW.cZMP_dsp_global_proj);  //-> not using

            HBPW.DSP_Fz_controller(0,HBPW.RS.F_RF.z, HBPW.RS.F_LF.z);

            HBPW.pRF_ref.z = pRF_ini_global.z + 0.5*HBPW.z_ctrl;
            HBPW.pLF_ref.z = pLF_ini_global.z - 0.5*HBPW.z_ctrl;


            //// --------------------- Ankle Torque Control
            HBPW.step_phase = 0;
            HBPW.Pos_Ankle_torque_control_flag = true;
            HBPW.Standing_mode_flag = false;

//            vec3 temp_cZMP_TC_local = (1 + 10.0/w)*(HBPW.COM_e_imu_local + 1.0*HBPW.dCOM_e_imu_local/HBPW.w);

//            HBPW.cZMP_TC_local.x = temp_cZMP_TC_local.x - 0.000;
//            HBPW.cZMP_TC_local.y = temp_cZMP_TC_local.y;

//            HBPW.cZMP_TC_global = local2global_point(HBPW.qRF_ref, HBPW.qLF_ref,HBPW.pRF_ref, HBPW.pLF_ref, HBPW.cZMP_TC_local);

//            HBPW.cZMP_TC_proj = zmpProjectionToSP_offset(HBPW.cZMP_TC_global, HBPW.pRF_ref, HBPW.pLF_ref, HBPW.qRF_ref, HBPW.qLF_ref, HBPW.RS.F_RF, HBPW.RS.F_LF, +0.00, +0.00);

            HBPW.cZMP_TC_local = (1 + 4/HBPW.w)*((HBPW.COM_m_filtered + 1.5*HBPW.dCOM_m_diff_filtered/HBPW.w));

            HBPW.cZMP_TC_global = local2global_point(HBPW.qRF_ref, HBPW.qLF_ref, HBPW.pRF_ref, HBPW.pLF_ref, HBPW.cZMP_TC_local);


            HBPW.cZMP_TC_proj =  zmpProjectionToSP_offset(HBPW.cZMP_TC_local, HBPW.pRF_ref, HBPW.pLF_ref, HBPW.qRF_ref, HBPW.qLF_ref, HBPW.RS.F_RF, HBPW.RS.F_LF, -0.002, +0.02);



            HBPW.AnkleTorque_ref = HBPW.Ankle_Torque_from_cZMP(HBPW.cZMP_TC_proj, HBPW.ZMP_global, HBPW.qRF_ref, HBPW.qLF_ref, HBPW.pRF_ref, HBPW.pLF_ref, HBPW.RS.F_RF, HBPW.RS.F_LF);


            HBPW.AnkleTorqueController_pos(HBPW.AnkleTorque_ref[0], HBPW.AnkleTorque_ref[1], HBPW.AnkleTorque_ref[2], HBPW.AnkleTorque_ref[3],
                                        HBPW.RS.F_RF, HBPW.RS.F_LF, HBPW.RS.M_RF, HBPW.RS.M_LF);

            HBPW.Standing_mode_flag = false;


            //// ---------------- CP error & Walking Trigger

            vec3 CP_error = HBPW.COM_m_filtered + 1.4*HBPW.dCOM_m_diff_filtered/HBPW.w;

            if(HBPW.k % 10 == 0) cout<<"CP_error_x: "<<CP_error.x<<" CP_error_y: "<<CP_error.y<<endl;

            if(fabs(CP_error.x) > 0.06 && HBPW.k > 1000) _task_thread = _task_HB_PrevWalk;
            else _task_thread = _task_Ready_To_Walk;



            //// ------------------- Reference Trajectory

            WBmotion->addCOMInfo_xy_pelz_HB(HBPW.uCOM.x, HBPW.uCOM.y, WBmotion->pPel_3x1[2]);
            WBmotion->addRFPosInfo_HB(HBPW.pRF_ref.x, HBPW.pRF_ref.y, HBPW.pRF_ref.z);
            WBmotion->addLFPosInfo_HB(HBPW.pLF_ref.x, HBPW.pLF_ref.y, HBPW.pLF_ref.z);

            WBmotion->addRFOriInfo_HB(HBPW.qRF_ref);
            WBmotion->addLFOriInfo_HB(HBPW.qLF_ref);


            break;

        }
        case _task_HB_PrevWalk:
        {
//            RTIME begin, end;
//            begin = rt_timer_read();
            if(HBPW.step_phase <= 1){
              joint->SetJointRefAngle(RF1, 5.f);
              joint->SetJointRefAngle(RF3, 5.f);
              joint->SetJointRefAngle(RF4, 5.f);
              joint->SetJointRefAngle(RF2, 5.f);
              joint->SetJointRefAngle(RF5, 5.f);

              joint->SetJointRefAngle(LF1, 5.f);
              joint->SetJointRefAngle(LF2, 5.f);
              joint->SetJointRefAngle(LF3, 5.f);
              joint->SetJointRefAngle(LF4, 5.f);
              joint->SetJointRefAngle(LF5, 5.f);
            }
            else{
              joint->SetJointRefAngle(RF1, 0.f);
              joint->SetJointRefAngle(RF3, 0.f);
              joint->SetJointRefAngle(RF4, 0.f);
              joint->SetJointRefAngle(RF2, 0.f);
              joint->SetJointRefAngle(RF5, 0.f);

              joint->SetJointRefAngle(LF1, 0.f);
              joint->SetJointRefAngle(LF2, 0.f);
              joint->SetJointRefAngle(LF3, 0.f);
              joint->SetJointRefAngle(LF4, 0.f);
              joint->SetJointRefAngle(LF5, 0.f);
            }

            userData->M2G.valveMode = 1;

            //// if JoyStick on, calc del_pos from Joystick input
            if(HBPW.Joystick_walk_flag == true){
                int JOY_RJOG_RL = sharedData->COMMAND[PODO_NO].USER_PARA_INT[0];
                int JOY_RJOG_UD = sharedData->COMMAND[PODO_NO].USER_PARA_INT[1];
                int JOY_LJOG_RL = sharedData->COMMAND[PODO_NO].USER_PARA_INT[2];
                int JOY_LJOG_UD = sharedData->COMMAND[PODO_NO].USER_PARA_INT[3];
                int JOY_RB = sharedData->COMMAND[PODO_NO].USER_PARA_INT[4];
                int JOY_LB = sharedData->COMMAND[PODO_NO].USER_PARA_INT[5];
                double des_t_step = 0.7;//sharedData->COMMAND[PODO_NO].USER_PARA_DOUBLE[15];

                //cout<<"Ljog RL : "<<JOY_LJOG_RL<<endl;

                HBPW.Calc_del_pos_from_Joystick(JOY_RJOG_RL, JOY_RJOG_UD, JOY_LJOG_RL, JOY_LJOG_UD, des_t_step);

                if(JOY_RB == 1 && JOY_LB == 1){ // Walking Emergency Stop
                    _task_thread = _task_Idle;
                    save_all();
                }
            }

            userData->M2G.valveMode = 2;


            //// Put sensor value into RS object
            SensorInput();

            userData->M2G.valveMode = 3;

            HBPW.MeasurementInput(RS_temp);


            userData->M2G.valveMode = 4;

//            if(HBPW.step_phase == HBPW.N_step && (HBPW.t_now > 0.6-HBPW.dt/2 && HBPW.t_now < 0.6+HBPW.dt/2)){
            if(HBPW.step_phase == HBPW.N_step + 1){
                //Init Hand
                sharedData->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[0] = 1; //right
                sharedData->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[1] = 1; //left
                sharedData->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[2] = 0; //head
                sharedData->COMMAND[RBCORE_PODO_NO].USER_COMMAND = DAEMON_INIT_FIND_HOME_HAND_HEAD;
            }

            //// Main Walking code
            if(HBPW.Preveiw_walking() == -1){
                _task_thread = _task_Idle;
                WB_FLAG = 0;
                //WB_FLAG = 0;
                save_all();
                //HBPW.save_WD();
                userData->M2G.Walking_state = false;
                ArmGainOverride_Off();
                joint->SetJointRefAngle(RF1, 0.f);
                joint->SetJointRefAngle(RF2, 0.f);
                joint->SetJointRefAngle(RF3, 0.f);
                joint->SetJointRefAngle(RF4, 0.f);
                joint->SetJointRefAngle(RF5, 0.f);

                joint->SetJointRefAngle(LF1, 0.f);
                joint->SetJointRefAngle(LF2, 0.f);
                joint->SetJointRefAngle(LF3, 0.f);
                joint->SetJointRefAngle(LF4, 0.f);
                joint->SetJointRefAngle(LF5, 0.f);



                double postime = 800.0;
                joint->SetMoveJoint(RSP, 10.0, postime, MOVE_ABSOLUTE);
                joint->SetMoveJoint(RSR, -10.0, postime, MOVE_ABSOLUTE);
                joint->SetMoveJoint(RSY, 0.0, postime, MOVE_ABSOLUTE);
                joint->SetMoveJoint(REB, -30, postime, MOVE_ABSOLUTE);
                joint->SetMoveJoint(RWY, 0.0, postime, MOVE_ABSOLUTE);
                joint->SetMoveJoint(RWP, 0.0, postime, MOVE_ABSOLUTE);

                joint->SetMoveJoint(LSP, 10.0, postime, MOVE_ABSOLUTE);
                joint->SetMoveJoint(LSR, 10.0, postime, MOVE_ABSOLUTE);
                joint->SetMoveJoint(LSY, 0.0, postime, MOVE_ABSOLUTE);
                joint->SetMoveJoint(LEB, -30.0, postime, MOVE_ABSOLUTE);
                joint->SetMoveJoint(LWY, 0.0, postime, MOVE_ABSOLUTE);
                joint->SetMoveJoint(LWP, 0.0, postime, MOVE_ABSOLUTE);

                joint->SetMoveJoint(RHY, 0, postime, MOVE_ABSOLUTE);
                joint->SetMoveJoint(RHR, 0, postime, MOVE_ABSOLUTE);
                joint->SetMoveJoint(RHP, -26.403, postime, MOVE_ABSOLUTE);
                joint->SetMoveJoint(RKN, 49.705, postime, MOVE_ABSOLUTE);
                joint->SetMoveJoint(RAP, -23.302, postime, MOVE_ABSOLUTE);
                joint->SetMoveJoint(RAR, 0, postime, MOVE_ABSOLUTE);

                joint->SetMoveJoint(LHY, 0, postime, MOVE_ABSOLUTE);
                joint->SetMoveJoint(LHR, 0, postime, MOVE_ABSOLUTE);
                joint->SetMoveJoint(LHP, -26.403, postime, MOVE_ABSOLUTE);
                joint->SetMoveJoint(LKN, 49.705, postime, MOVE_ABSOLUTE);
                joint->SetMoveJoint(LAP, -23.302, postime, MOVE_ABSOLUTE);
                joint->SetMoveJoint(LAR, 0, postime, MOVE_ABSOLUTE);

                RSP_ref_deg_offset = 10;//sharedData->ENCODER[MC_GetID(RSP)][MC_GetCH(RSP)].CurrentReference;
                LSP_ref_deg_offset = 10;//sharedData->ENCODER[MC_GetID(LSP)][MC_GetCH(LSP)].CurrentReference;

                WST_ref_deg = 0;
                RSP_ref_deg = 0;
                LSP_ref_deg = 0;
                NeckYaw_ref_deg = 0;
                RF_angle_con_deg = vec3();
                LF_angle_con_deg = vec3();
                HBPW.RF_angle_ctrl = vec3();
                HBPW.LF_angle_ctrl = vec3();
                RHR_con_deg = 0;
                LHR_con_deg = 0;
                RHY_con_deg = 0;
                LHY_con_deg = 0;
                LHP_con_deg = 0;
                RHP_con_deg = 0;
                LKN_con_deg = 0;
                RKN_con_deg = 0;

                cout<<"Preview Walk finished"<<endl;
                //cout<<RHR_con_deg<<", "<<LHR_con_deg<<", "<<RHY_con_deg<<", "<<LHY_con_deg<<", "<<LHP_con_deg<<", "<<RHP_con_deg<<endl;
            }

            userData->M2G.step_phase = HBPW.step_phase;

            //// disturbance command (Choreonoid)
//            if(HBPW.step_phase == 2 && (HBPW.t_now >= 0.65 && HBPW.t_now < 0.655)){
//                sharedData->COMMAND[MAX_AL-1].USER_COMMAND = 222;
//                sharedData->COMMAND[MAX_AL-1].USER_PARA_DOUBLE[0] = 25;
//                sharedData->COMMAND[MAX_AL-1].USER_PARA_DOUBLE[1] = 570;
//                // 330 : Ankle
//                // 400 : Ankle + hip
//                // 570 : Ankle + hip + stepping
//            }



            userData->M2G.valveMode = 5;

            vec3 COM_total = HBPW.uCOM;


            //// Leg Vibration Control
            LHY_con_deg = -HBPW.LHY_con_deg;
            RHY_con_deg = -HBPW.RHY_con_deg;

            LHR_con_deg = 1.0*HBPW.LHR_con_deg + HBPW.L_roll_compen_deg;
            RHR_con_deg = 1.0*HBPW.RHR_con_deg + HBPW.R_roll_compen_deg;

//            LHR_con_deg =  HBPW.L_roll_compen_deg;
//            RHR_con_deg =  HBPW.R_roll_compen_deg;

            LHP_con_deg =  1.0*HBPW.LHP_con_deg*0.5;
            RHP_con_deg =  1.0*HBPW.RHP_con_deg*0.5;
            LKN_con_deg = HBPW.L_knee_compen_deg;
            RKN_con_deg = HBPW.R_knee_compen_deg;

            //cout<<"LHR_con_deg :"<<HBPW.LHR_con_deg<<"  RHR_con_deg: "<<HBPW.RHR_con_deg<<endl;


            //// Pos Ankle torque control
            RF_angle_con_deg.x = HBPW.RF_angle_ctrl.x*R2D + HBPW.RF_landing_angle.r*R2D;
            RF_angle_con_deg.y = HBPW.RF_angle_ctrl.y*R2D + HBPW.RF_landing_angle.p*R2D;
            LF_angle_con_deg.x = HBPW.LF_angle_ctrl.x*R2D + HBPW.LF_landing_angle.r*R2D;
            LF_angle_con_deg.y = HBPW.LF_angle_ctrl.y*R2D + HBPW.LF_landing_angle.p*R2D;


            ////Foot and Pelv Orientation
            //foot Orientation

            if(AnkleTorqueControl_flag == true){
                quat total_qRF = HBPW.qRF_ref*HBPW.RF_del_quat;
                quat total_qLF = HBPW.qLF_ref*HBPW.LF_del_quat;

                for(int i=0;i<4;i++){
                    dbs_qRF[i] = total_qRF[i];
                    dbs_qLF[i] = total_qLF[i];
                }
            }
            else{
                for(int i=0;i<4;i++){
                    dbs_qRF[i] = HBPW.qRF_ref[i];
                    dbs_qLF[i] = HBPW.qLF_ref[i];
                }
            }

            // Pelvis Orientation
            for(int i=0;i<4;i++){
                dbs_qPel[i] = HBPW.qPel_ref[i];
            }


            userData->M2G.valveMode = 6;

            //// Put reference Task to Trajectory Handler
//            WBmotion->addCOMInfo(COM_total.x, COM_total.y, 0.62 + HBPW.del_COMz_con, 0.005);
//            WBmotion->addRFPosInfo(HBPW.pRF_ref.x, HBPW.pRF_ref.y, HBPW.pRF_ref.z,0.005);
//            WBmotion->addLFPosInfo(HBPW.pLF_ref.x, HBPW.pLF_ref.y, HBPW.pLF_ref.z,0.005);
//            WBmotion->addRFOriInfo(dbs_qRF,0.005);
//            WBmotion->addLFOriInfo(dbs_qLF,0.005);
//            WBmotion->addPELOriInfo(dbs_qPel,0.005);

            //WBmotion->addCOMInfo_HB(COM_total.x, COM_total.y, 0.62 + HBPW.del_COMz_con);
            WBmotion->addCOMInfo_xy_pelz_HB(COM_total.x, COM_total.y, WBmotion->pPel_3x1[2]);
            WBmotion->addRFPosInfo_HB(HBPW.pRF_ref.x, HBPW.pRF_ref.y, HBPW.pRF_ref.z);
            WBmotion->addLFPosInfo_HB(HBPW.pLF_ref.x, HBPW.pLF_ref.y, HBPW.pLF_ref.z);
            WBmotion->addRFOriInfo_HB(HBPW.qRF_ref);
            WBmotion->addLFOriInfo_HB(HBPW.qLF_ref);
            WBmotion->addPELOriInfo_HB(HBPW.qPel_ref);

            userData->M2G.valveMode = 7;

            WST_ref_deg = HBPW.WST_ref_deg;
            NeckYaw_ref_deg = -HBPW.WST_ref_deg;
            RSP_ref_deg = HBPW.RSP_des_deg;
            LSP_ref_deg = HBPW.LSP_des_deg;


            save_onestep(HBPW.k);

//            end = rt_timer_read();
//            cout<<"time : "<<(long)(end - begin)/100000.0<<endl; // 0.1ms unit

            userData->M2G.valveMode = 8;

            break;
        }
        case _task_HB_test:
        {
            SensorInput();
            HBPW.MeasurementInput(RS_temp);

            HBPW.k++;
            save_onestep(HBPW.k);


            ////-----------------------------temp
//            if(HBPW.k % 500 == 0){
//                cout<<"11sec"<<endl;
//                cout<<"COM : "<<WBmotion->des_pCOM_3x1_HB[0]<<", "<<WBmotion->des_pCOM_3x1_HB[1]<<", "<<WBmotion->des_pCOM_3x1_HB[2]<<endl;
//                cout<<"pPEL : "<<WBmotion->des_pPELz_HB<<endl;
//                cout<<"pRF : "<<WBmotion->des_pRF_3x1_HB[0]<<", "<<WBmotion->des_pRF_3x1_HB[1]<<", "<<WBmotion->des_pRF_3x1_HB[2]<<endl;
//                cout<<"pLF : "<<WBmotion->des_pLF_3x1_HB[0]<<", "<<WBmotion->des_pLF_3x1_HB[1]<<", "<<WBmotion->des_pLF_3x1_HB[2]<<endl;
//                cout<<"qPEL : "<<WBmotion->des_qPEL_4x1_HB[0]<<", "<<WBmotion->des_qPEL_4x1_HB[1]<<", "<<WBmotion->des_qPEL_4x1_HB[2]<<endl;
//                cout<<"qRF : "<<WBmotion->des_qRF_4x1_HB[0]<<", "<<WBmotion->des_qRF_4x1_HB[1]<<", "<<WBmotion->des_qRF_4x1_HB[2]<<endl;
//                cout<<"qLF : "<<WBmotion->des_qLF_4x1_HB[0]<<", "<<WBmotion->des_qLF_4x1_HB[1]<<", "<<WBmotion->des_qLF_4x1_HB[2]<<endl;
//                cout<<endl;


//            }

//            WBmotion->addCOMInfo_xy_pelz_HB(0, 0, WBmotion->pPel_3x1[2]);
//            WBmotion->addRFPosInfo_HB(0, WBmotion->pRF_3x1[1], 0);
//            WBmotion->addLFPosInfo_HB(0, 0.1091, 0);
//            WBmotion->addRFOriInfo_HB(quat());
//            WBmotion->addLFOriInfo_HB(quat());
//            WBmotion->addPELOriInfo_HB(quat());

//            HBPW.k++;


            //// -----------------------COM Damping Control
//            HBPW.p_ref[0] = vec3(0, 0.0,0);
            HBPW.COM_ref = vec3(0, 0.0,0);
            HBPW.p_ref[0] = vec3(0, 0.0,0);

            HBPW.DampingControl3();
            //HBPW.ZMP_Tracking_controller();



            HBPW.uCOM = HBPW.COM_ref + HBPW.COM_damping_con;//HBPW.COM_ref;
//            HBPW.uCOM = HBPW.COM_ref + HBPW.COM_zmp_con;//HBPW.COM_ref;

//            cout<<"uCOMy: "<<HBPW.uCOM.y<<endl;
            cout<<"damping con uCOMy: "<<HBPW.COM_damping_con.y<<"  ZMPy : "<<HBPW.ZMP_global.y<<endl;


            //WBmotion->addCOMInfo_HB(HBPW.uCOM.x, HBPW.uCOM.y, WBmotion->pCOM_3x1[2]);
            WBmotion->addCOMInfo_xy_pelz_HB(HBPW.uCOM.x, HBPW.uCOM.y, WBmotion->pPel_3x1[2]);
            //WBmotion->addCOMInfo_xy_pelz_HB(WBmotion->pCOM_3x1[0], WBmotion->pCOM_3x1[1], WBmotion->pPel_3x1[2]);

            //// ----------------------Standing Test



//            // using only imu
////            HBPW.cZMP_dsp_local = (1 + 2/HBPW.w)*(HBPW.COM_e_imu_local + 1.0*HBPW.dCOM_e_imu_local/HBPW.w);
////            HBPW.cZMP_dsp_global = local2global_point(HBPW.qRF_ref, HBPW.qLF_ref, HBPW.pRF_ref, HBPW.pLF_ref, HBPW.cZMP_dsp_local);

//            // using imu + damping con input
//            double k_gain = 6;
//            HBPW.cZMP_dsp_global = (1 + 1.5/HBPW.w)*((HBPW.COM_m_filtered + HBPW.dCOM_m_diff_filtered/HBPW.w));


//            HBPW.cZMP_dsp_global_proj = zmpProjectionToSP_offset(HBPW.cZMP_dsp_global, HBPW.pRF_ref, HBPW.pLF_ref, HBPW.qRF_ref, HBPW.qLF_ref, 100, 100, -0.005, -0.005);

//            HBPW.cZMP_dsp_local_proj = global2local_point(HBPW.qRF_ref, HBPW.qLF_ref, HBPW.pRF_ref, HBPW.pLF_ref, HBPW.cZMP_dsp_global_proj);

//            //HBPW.DSP_Fz_controller2(0);
//            HBPW.DSP_Fz_controller(0,HBPW.RS.F_RF.z, HBPW.RS.F_LF.z);

//            double RFz = HBPW.pRF_ref.z + 0.5*HBPW.z_ctrl;
//            double LFz = HBPW.pLF_ref.z - 0.5*HBPW.z_ctrl;

//            //cout<<"z ctrl: "<<HBPW.z_ctrl<<endl;
//            //cout<<"cZMP_dsp_local_proj.y : "<<HBPW.cZMP_dsp_local_proj.y<<endl;
//            //cout<<"Alpha dsp : "<<HBPW.Alpha_dsp<<"z_ctrl : "<<HBPW.z_ctrl<<endl;

//            // Ankle Torque
//            //HBPW.cZMP_TC_local = (1 + 6/HBPW.w)*(1.1*HBPW.COM_e_imu_local + 1*HBPW.dCOM_e_imu_local/HBPW.w);

//            //
//            HBPW.cZMP_TC_local = (1 + 4/HBPW.w)*((HBPW.COM_m_filtered + HBPW.dCOM_m_diff_filtered/HBPW.w));

//            HBPW.cZMP_TC_global = local2global_point(HBPW.qRF_ref, HBPW.qLF_ref, HBPW.pRF_ref, HBPW.pLF_ref, HBPW.cZMP_TC_local);


//            HBPW.cZMP_TC_proj =  zmpProjectionToSP_offset(HBPW.cZMP_TC_local, HBPW.pRF_ref, HBPW.pLF_ref, HBPW.qRF_ref, HBPW.qLF_ref, HBPW.RS.F_RF, HBPW.RS.F_LF, -0.002, +0.02);

//            HBPW.AnkleTorque_ref = HBPW.Ankle_Torque_from_cZMP(HBPW.cZMP_TC_proj, HBPW.ZMP_global, HBPW.qRF_ref, HBPW.qLF_ref, HBPW.pRF_ref, HBPW.pLF_ref, HBPW.RS.F_RF, HBPW.RS.F_LF);

//            HBPW.AnkleTorqueController_pos(HBPW.AnkleTorque_ref[0], HBPW.AnkleTorque_ref[1], HBPW.AnkleTorque_ref[2], HBPW.AnkleTorque_ref[3],
//                                        HBPW.RS.F_RF, HBPW.RS.F_LF, HBPW.RS.M_RF, HBPW.RS.M_LF);

//            HBPW.qRF_ref = quat()*HBPW.RF_quat_ctrl;
//            HBPW.qLF_ref = quat()*HBPW.LF_quat_ctrl;



//            // Damping Control
//            HBPW.DampingControl3();
//            HBPW.uCOM = HBPW.COM_ref + HBPW.COM_damping_con;// + HBPW.COM_con;//HBPW.COM_ref;

//            //cout<<"com_con : "<<HBPW.COM_con.y<<endl;


//            WBmotion->addCOMInfo_xy_pelz_HB(HBPW.uCOM.x, HBPW.uCOM.y, WBmotion->pPel_3x1[2]);
//            WBmotion->addRFPosInfo_HB(HBPW.pRF_ref.x, HBPW.pRF_ref.y, RFz);
//            WBmotion->addLFPosInfo_HB(HBPW.pLF_ref.x, HBPW.pLF_ref.y, LFz);

//            WBmotion->addRFOriInfo_HB(HBPW.qRF_ref);
//            WBmotion->addLFOriInfo_HB(HBPW.qLF_ref);

//            save_onestep(HBPW.k);
//            HBPW.k++;


            ////------------------------- Left Swing Leg Vibration Control


//            LHR_con_deg = HBPW.LSwingLeg_Vib_Control(HBPW.ACC_LF_filtered, false);

//            if(LHR_con_deg > 0.3) LHR_con_deg = 0.3;
//            if(LHR_con_deg < -0.3) LHR_con_deg = -0.3;

////            LHR_con_deg = HBPW.LSwingLeg_Vib_Control2(HBPW.ACC_LF_filtered, HBPW.RS.IMULocalW.x, false);

//            //LHP_con_deg = 1.2*HBPW.LSwingLeg_Pitch_Vib_Control(HBPW.ACC_LF_filtered, false);

//            cout<<"ACC_LFy_bpf: "<<HBPW.ACC_LF_filtered.y<<"   con deg : "<<LHR_con_deg<<endl;
//            cout<<"torso _rollvel: "<<HBPW.RS.IMULocalW.x<<endl;

//            //RHR_con_deg = HBPW.LTorso_Roll_Vib_Control(HBPW.RS.IMULocalW, false);

//            RHY_con_deg = HBPW.LTorso_Yaw_Vib_Control(HBPW.RS.IMULocalW, false);
//            cout<<"IMU yaw vel: "<<HBPW.RS.IMULocalW.z<<"   con deg : "<<RHY_con_deg<<endl;

            //// -------------------------Right Swing Leg Vibration Control
//            RHR_con_deg = HBPW.RSwingLeg_Vib_Control(HBPW.ACC_RF_filtered, false);
//            //RHP_con_deg = 1*HBPW.RSwingLeg_Pitch_Vib_Control(HBPW.ACC_RF_filtered);

//            //LHR_con_deg = HBPW.RTorso_Roll_Vib_Control(HBPW.RS.IMULocalW);

//            LHY_con_deg = HBPW.RTorso_Yaw_Vib_Control(HBPW.RS.IMULocalW, false);

//            cout<<"ACC_RFy_bpf: "<<HBPW.ACC_RF_filtered.y<<"   con deg : "<<RHR_con_deg<<endl;


            //// ----------------Ankle Torque Control (by position)
//            HBPW.R_SSP_ZMP_Control(vec3(),vec3(),RSEN.F_RF, RSEN.M_RF);
//            HBPW.L_SSP_ZMP_Control(vec3(),vec3(),RSEN.F_LF, RSEN.M_LF);
//            cout<<"angle x: "<<HBPW.LF_del_angle.x*R2D<<endl;

//            quat total_qRF = HBPW.RF_del_quat;
//            quat total_qLF = HBPW.LF_del_quat;
//            for(int i=0;i<4;i++){
//                WBmotion->des_qRF_4x1[i] = total_qRF[i];
//                WBmotion->des_qLF_4x1[i] = total_qLF[i];
//            }

            //// -------------------Arial Phase Ankle Torque Control (by pwm)
/*            LegJoints LJ_original = WBmotion->WBIK2(HBPW.qRF_ref, HBPW.qLF_ref);

            vec4 AnkleToque =  HBPW.Ankle_Torque_from_PositionFB(2, HBPW.RS.F_RF, HBPW.RS.F_LF,
                                          LJ_original.RAP*R2D, LJ_original.RAR*R2D, LJ_original.LAP*R2D, LJ_original.LAR*R2D,

                                          HBPW.RS.JSP.RAP*R2D, HBPW.RS.JSP.RAR*R2D, HBPW.RS.JSP.LAP*R2D, HBPW.RS.JSP.LAR*R2D,
                                          0, 0, 0, 0,
                                          HBPW.RS.JSV.dRAP*R2D, HBPW.RS.JSV.dRAR*R2D, HBPW.RS.JSV.dLAP*R2D, HBPW.RS.JSV.dLAR*R2D);

//            vec4 AnkleToque = HBPW.Ankle_Torque_from_cZMP(vec3(0.1, 0, 0), vec3(0,0,0), HBPW.qRF_ref, HBPW.qLF_ref,
//                                                        vec3(WBmotion->pRF_3x1), vec3(WBmotion->pLF_3x1), vec3(0,0,100), vec3(0,0,100));

            double Tx_ref, Ty_ref;

            Tx_ref = 5;
            Ty_ref = 0;
            HBPW.AnkleTorqueController(-AnkleToque[0], -AnkleToque[1], -AnkleToque[2], -AnkleToque[3],
                                   HBPW.RS.JSP.RA1*R2D, HBPW.RS.JSP.RA2*R2D, HBPW.RS.JSP.LA1*R2D, HBPW.RS.JSP.LA2*R2D,
                                   HBPW.RS.JSP.RAP*R2D, HBPW.RS.JSP.RAR*R2D, HBPW.RS.JSP.LAP*R2D, HBPW.RS.JSP.LAR*R2D,
                                   HBPW.RS.JSV.dRA1*R2D, HBPW.RS.JSV.dRA2*R2D, HBPW.RS.JSV.dLA1*R2D, HBPW.RS.JSV.dLA2*R2D,
                                   HBPW.RS.F_RF, HBPW.RS.F_LF, HBPW.RS.M_RF, HBPW.RS.M_LF);



            HBPW.AnkleQuatCalculaltor(HBPW.RS.JSP.RAP*R2D - LJ_original.RAP*R2D, HBPW.RS.JSP.RAR*R2D - LJ_original.RAR*R2D,
                                      HBPW.RS.JSP.LAP*R2D - LJ_original.LAP*R2D, HBPW.RS.JSP.LAR*R2D - LJ_original.LAR*R2D);

            quat total_qRF = HBPW.qRF_ref*HBPW.RF_del_quat;
            quat total_qLF = HBPW.qLF_ref*HBPW.LF_del_quat;
            doubles dbs_qRF(4), dbs_qLF(4);
            for(int i=0;i<4;i++){
                dbs_qRF[i] = total_qRF[i];
                dbs_qLF[i] = total_qLF[i];

            }

            WBmotion->addRFOriInfo(dbs_qRF,0.005);
            WBmotion->addLFOriInfo(dbs_qLF,0.005);



//            cout<<"ref Tx: "<<Tx_ref<<", Tx: "<<HBPW.RS.M_RF.x<<endl;
            cout<<"T_RAR: "<<AnkleToque[0]<<" T_RAP: "<<AnkleToque[1]<<" T_LAR: "<<AnkleToque[2]<<" T_LAP: "<<AnkleToque[3]<<endl;
//            cout<<"E_RAR: "<<HBPW.RS.JSP.RAP*R2D - WBmotion->LJ.RAP*R2D<<" E_RAP: "<<HBPW.RS.JSP.RAR*R2D - WBmotion->LJ.RAR*R2D
//               <<" E_LAR: "<<HBPW.RS.JSP.LAP*R2D - WBmotion->LJ.LAP*R2D<<" E_LAP: "<<HBPW.RS.JSP.LAR*R2D - WBmotion->LJ.LAR*R2D<<endl;
//            cout<<"q_RF: "<<total_qRF.w<<", "<<total_qRF.x<<", "<<total_qRF.y<<", "<<total_qRF.z<<" ::: "
//                <<"q_LF: "<<total_qLF.w<<", "<<total_qLF.x<<", "<<total_qLF.y<<", "<<total_qLF.z<<endl;
//            cout<<"E_RAP: "<<HBPW.RS.JSP.RAP*R2D - LJ_original.RAP*R2D<<" E_RAR: "<<HBPW.RS.JSP.RAR*R2D - LJ_original.RAR*R2D
//               <<" E_LAP: "<<HBPW.RS.JSP.LAP*R2D - LJ_original.LAP*R2D<<" E_LAR: "<<HBPW.RS.JSP.LAR*R2D - LJ_original.LAR*R2D<<endl;
*/
            ////----------------------- Standing Test
/*           // original LJ ref
            LegJoints LJ_original = WBmotion->WBIK2(HBPW.qRF_ref, HBPW.qLF_ref);

            //cZMP generation
            double dt_gain1 = 25;
            vec3 CP_ref = HBPW.COM_ref;
            //HBPW.cZMP = 1/(1 - exp(HBPW.w*HBPW.dt*dt_gain1))*CP_ref - exp(HBPW.w*HBPW.dt*dt_gain1)/(1 - exp(HBPW.w*HBPW.dt*dt_gain1))*HBPW.CP_m;// - CP_ref + p_ref[0];

            //cZMP from DCM Tracking Controller
            double DCM_gain = 8;
            HBPW.cZMP = HBPW.COM_ref + (1 + DCM_gain/HBPW.w)*(HBPW.CP_m - (HBPW.COM_ref + HBPW.dCOM_ref/HBPW.w));

            HBPW.cZMP_proj = zmpProjectionToSP_offset(HBPW.cZMP, HBPW.pRF_ref, HBPW.pLF_ref, HBPW.qRF_ref, HBPW.qLF_ref, HBPW.RS.F_RF, HBPW.RS.F_LF, +0.00, 0.00);

            vec4 cZMP_FF_torque = HBPW.Ankle_Torque_from_cZMP(HBPW.cZMP_proj, HBPW.ZMP_global, HBPW.qRF_ref, HBPW.qLF_ref, HBPW.pRF_ref, HBPW.pLF_ref, HBPW.RS.F_RF, HBPW.RS.F_LF);

            vec4 Angle_FB_torque =  HBPW.Ankle_Torque_from_PositionFB(2, HBPW.RS.F_RF, HBPW.RS.F_LF,
                                          LJ_original.RAP*R2D, LJ_original.RAR*R2D, LJ_original.LAP*R2D, LJ_original.LAR*R2D,

                                          HBPW.RS.JSP.RAP*R2D, HBPW.RS.JSP.RAR*R2D, HBPW.RS.JSP.LAP*R2D, HBPW.RS.JSP.LAR*R2D,
                                          0, 0, 0, 0,
                                          HBPW.RS.JSV.dRAP*R2D, HBPW.RS.JSV.dRAR*R2D, HBPW.RS.JSV.dLAP*R2D, HBPW.RS.JSV.dLAR*R2D);

            vec4 AnkleToque;

            AnkleToque[0] = cZMP_FF_torque[0];// - Angle_FB_torque[0];
            AnkleToque[1] = cZMP_FF_torque[1];// - Angle_FB_torque[1];
            AnkleToque[2] = cZMP_FF_torque[2];// - Angle_FB_torque[2];
            AnkleToque[3] = cZMP_FF_torque[3];// - Angle_FB_torque[3];

            HBPW.AnkleTorqueController(AnkleToque[0], AnkleToque[1], AnkleToque[2], AnkleToque[3],
                                   HBPW.RS.JSP.RA1*R2D, HBPW.RS.JSP.RA2*R2D, HBPW.RS.JSP.LA1*R2D, HBPW.RS.JSP.LA2*R2D,
                                   HBPW.RS.JSP.RAP*R2D, HBPW.RS.JSP.RAR*R2D, HBPW.RS.JSP.LAP*R2D, HBPW.RS.JSP.LAR*R2D,
                                   HBPW.RS.JSV.dRA1*R2D, HBPW.RS.JSV.dRA2*R2D, HBPW.RS.JSV.dLA1*R2D, HBPW.RS.JSV.dLA2*R2D,
                                   HBPW.RS.F_RF, HBPW.RS.F_LF, HBPW.RS.M_RF, HBPW.RS.M_LF);



            HBPW.AnkleQuatCalculaltor(HBPW.RS.JSP.RAP*R2D - LJ_original.RAP*R2D, HBPW.RS.JSP.RAR*R2D - LJ_original.RAR*R2D,
                                      HBPW.RS.JSP.LAP*R2D - LJ_original.LAP*R2D, HBPW.RS.JSP.LAR*R2D - LJ_original.LAR*R2D);

            quat total_qRF = HBPW.qRF_ref*HBPW.RF_del_quat;
            quat total_qLF = HBPW.qLF_ref*HBPW.LF_del_quat;
            doubles dbs_qRF(4), dbs_qLF(4);
            for(int i=0;i<4;i++){
                dbs_qRF[i] = total_qRF[i];
                dbs_qLF[i] = total_qLF[i];

            }

            WBmotion->addRFOriInfo(dbs_qRF,0.005);
            WBmotion->addLFOriInfo(dbs_qLF,0.005);

            //cout<<"ref RAP: "<<AnkleToque[1]<<" RAP: "<<HBPW.RS.M_RF.y<<" ref LAP: "<<AnkleToque[3]<<" LAP: "<<HBPW.RS.M_LF.y<<endl;
*/
            //// -------------------------Sensor agreement test with Choreonoid
//            double Torso_ori_deg = 0;

//            Torso_ori_deg = 10*sin(2*3.141*0.5*HBPW.k*HBPW.dt);

//            quat Torso_quat = quat(vec3(1,0,0),Torso_ori_deg*D2R);

//            doubles Torso_quat_dbs(4);
//            for(int i=0;i<4;i++){
//                Torso_quat_dbs[i] = Torso_quat[i];
//            }

//            WBmotion->addPELOriInfo(Torso_quat_dbs, 0.005);

//            if(HBPW.k > 10*200){
//                _task_thread = _task_Idle;
//                HBPW.save_all();
//            }

            ////----------------------- Kinematics calibration
//            cout<<"Solution: 0.1091    "<<"ZMP y: "<<HBPW.ZMP_global.y<<endl;

            ////---------------- DSP Fz Controller & Ankle PWM Torque Control Test
//            double dT_cZMP = 0.1;
//            HBPW.cZMP = vec3(0,0,0) + exp(HBPW.w*dT_cZMP)/(exp(HBPW.w*dT_cZMP) - 1)*((HBPW.COM_m_filtered + HBPW.dCOM_m_diff_filtered/HBPW.w) - 0);

//            HBPW.cZMP_proj = zmpProjectionToSP_offset(HBPW.cZMP, HBPW.pRF_ref, HBPW.pLF_ref, HBPW.qRF_ref, HBPW.qLF_ref, HBPW.RS.F_RF, HBPW.RS.F_LF, +0.00, 0.00);

//            // get alpha_dsp and Ankle Torque Control

//            // Calc Original leg joint for kinematics update during Ankle Torque Control
//            HBPW.LJ_ref = WBmotion->WBIK2(HBPW.qRF_ref, HBPW.qLF_ref);


//            vec4 cZMP_FF_torque = HBPW.Ankle_Torque_from_cZMP(HBPW.cZMP_proj, HBPW.ZMP_global, HBPW.qRF_ref, HBPW.qLF_ref, HBPW.pRF_ref, HBPW.pLF_ref, HBPW.RS.F_RF, HBPW.RS.F_LF);

//            vec4 Angle_FB_torque = HBPW.Ankle_Torque_from_PositionFB(2, HBPW.RS.F_RF, HBPW.RS.F_LF,
//                                                                HBPW.LJ_ref.RAP*R2D, HBPW.LJ_ref.RAR*R2D, HBPW.LJ_ref.LAP*R2D, HBPW.LJ_ref.LAR*R2D,
//                                                                HBPW.RS.JSP.RAP*R2D, HBPW.RS.JSP.RAR*R2D, HBPW.RS.JSP.LAP*R2D, HBPW.RS.JSP.LAR*R2D,
//                                                                0*R2D, 0*R2D, 0*R2D, 0*R2D,
//                                                                HBPW.RS.JSV.dRAP*R2D, HBPW.RS.JSV.dRAR*R2D, HBPW.RS.JSV.dLAP*R2D, HBPW.RS.JSV.dLAR*R2D);

//            HBPW.AnkleTorque_ref[0] = cZMP_FF_torque[0] - Angle_FB_torque[0];
//            HBPW.AnkleTorque_ref[1] = cZMP_FF_torque[1] - Angle_FB_torque[1];
//            HBPW.AnkleTorque_ref[2] = cZMP_FF_torque[2] - Angle_FB_torque[2];
//            HBPW.AnkleTorque_ref[3] = cZMP_FF_torque[3] - Angle_FB_torque[3];

//            HBPW.AnkleTorqueController(HBPW.AnkleTorque_ref[0], HBPW.AnkleTorque_ref[1], HBPW.AnkleTorque_ref[2], HBPW.AnkleTorque_ref[3],
//                                       HBPW.RS.JSP.RA1*R2D, HBPW.RS.JSP.RA2*R2D, HBPW.RS.JSP.LA1*R2D, HBPW.RS.JSP.LA2*R2D,
//                                       HBPW.RS.JSP.RAP*R2D, HBPW.RS.JSP.RAR*R2D, HBPW.RS.JSP.LAP*R2D, HBPW.RS.JSP.LAR*R2D,
//                                       HBPW.RS.JSV.dRA1*R2D, HBPW.RS.JSV.dRA2*R2D, HBPW.RS.JSV.dLA1*R2D, HBPW.RS.JSV.dLA2*R2D,
//                                       HBPW.RS.F_RF, HBPW.RS.F_LF, HBPW.RS.M_RF, HBPW.RS.M_LF);

//            //Ankle quaternion compensation( Calc RF_LF_del_quat )
//            HBPW.AnkleQuatCalculaltor(HBPW.RS.JSP.RAP*R2D - HBPW.LJ_ref.RAP*R2D, HBPW.RS.JSP.RAR*R2D - HBPW.LJ_ref.RAR*R2D,
//                                      HBPW.RS.JSP.LAP*R2D - HBPW.LJ_ref.LAP*R2D, HBPW.RS.JSP.LAR*R2D - HBPW.LJ_ref.LAR*R2D);


//            // get Alpha dsp
//            vec3 L = HBPW.pRF_ref - HBPW.pLF_ref;
//            L.z = 0;
//            vec3 L_G = HBPW.G_R_g_pitroll*L;

//            if(L_G.z >= 0.005) L_G.z = 0.005;
//            if(L_G.z <= -0.005) L_G.z = -0.005;

//            HBPW.Alpha_dsp = 0.5 - L_G.z/0.005*0.5;


//            // calc ref Fz differnece
//            double Fz_total = 320;//HBPW.RS.F_LF.z + HBPW.RS.F_RF.z ; //32kg

//            static double Fz_total_filtered = 0;

//            double alpha_Fz = 1/(1 + 2*PI*dt*3.0);

//            Fz_total_filtered = alpha_Fz*Fz_total_filtered + (1 - alpha_Fz)*Fz_total;

//            HBPW.RF_Fz_ref = HBPW.Alpha_dsp*Fz_total;
//            HBPW.LF_Fz_ref = (1.0 - HBPW.Alpha_dsp)*Fz_total;

//            HBPW.Fz_diff_ref = HBPW.LF_Fz_ref - HBPW.RF_Fz_ref;

//            HBPW.Fz_diff = HBPW.RS.F_LF.z - HBPW.RS.F_RF.z;

//            // Control
//            double dsp_ctrl_gain = 0.0006;

//            HBPW.dz_ctrl = dsp_ctrl_gain*(HBPW.Fz_diff_ref - HBPW.Fz_diff);// - HBPW.z_ctrl/5;


//            double alpha_dz = 1/(1 + 2*PI*dt*5);
//            HBPW.dz_ctrl_filtered = alpha_dz*HBPW.dz_ctrl_filtered + (1.0 - alpha_dz)*HBPW.dz_ctrl;


//            HBPW.z_ctrl += HBPW.dz_ctrl_filtered*HBPW.dt;

//            if(HBPW.z_ctrl > 0.03) HBPW.z_ctrl = 0.03;
//            if(HBPW.z_ctrl < -0.03) HBPW.z_ctrl = -0.03;

//            double alpha_z = 1/(1 + 2*PI*dt*10.0);
//            HBPW.z_ctrl_filtered = alpha_z*HBPW.z_ctrl_filtered + (1.0 - alpha_z)*HBPW.z_ctrl;

//            HBPW.pRF_ref.z = + 0.5*HBPW.z_ctrl;
//            HBPW.pLF_ref.z = - 0.5*HBPW.z_ctrl;


//            cout<<"z_ctrl: "<<HBPW.z_ctrl<<" cZMP: "<<HBPW.cZMP_proj.y<<"  alpha dsp : "<<HBPW.Alpha_dsp<<endl;


//            doubles dbs_qRF(4), dbs_qLF(4);
//            if(AnkleTorqueControl_flag == true){
//                quat total_qRF = HBPW.qRF_ref*HBPW.RF_del_quat;
//                quat total_qLF = HBPW.qLF_ref*HBPW.LF_del_quat;

//                for(int i=0;i<4;i++){
//                    dbs_qRF[i] = total_qRF[i];
//                    dbs_qLF[i] = total_qLF[i];
//                }
//            }
//            else{
//                for(int i=0;i<4;i++){
//                    dbs_qRF[i] = HBPW.qRF_ref[i];
//                    dbs_qLF[i] = HBPW.qLF_ref[i];
//                }
//            }

//            // Ankle Torque to Choreonoid
//            if(AnkleTorqueControl_flag == true){
//                sharedData->Choreonoid.T_RAR = -HBPW.InputTorque[0];
//                sharedData->Choreonoid.T_RAP = -HBPW.InputTorque[1];
//                sharedData->Choreonoid.T_LAR = -HBPW.InputTorque[2];
//                sharedData->Choreonoid.T_LAP = -HBPW.InputTorque[3];
//            }

//            // Put reference Task to Trajectory Handler
//            WBmotion->addRFPosInfo(HBPW.pRF_ref.x, HBPW.pRF_ref.y, HBPW.pRF_ref.z,0.005);
//            WBmotion->addLFPosInfo(HBPW.pLF_ref.x, HBPW.pLF_ref.y, HBPW.pLF_ref.z,0.005);
//            WBmotion->addRFOriInfo(dbs_qRF,0.005);
//            WBmotion->addLFOriInfo(dbs_qLF,0.005);

//            HBPW.k++;

//            save_onestep(HBPW.k);


            ////--------------- DSP Fz Controller & Ankle "angle" Torque Control Test

            //--------Ankle angle control ---------------------

//            double dT_cZMP = 0.2;
//            HBPW.cZMP = exp(HBPW.w*dT_cZMP)/(exp(HBPW.w*dT_cZMP) - 1)*HBPW.CP_e_imu;

//            HBPW.cZMP_proj = zmpProjectionToSP_offset(HBPW.cZMP, HBPW.pRF_ref, HBPW.pLF_ref, HBPW.qRF_ref, HBPW.qLF_ref, HBPW.RS.F_RF, HBPW.RS.F_LF, +0.00, 0.00);

//            HBPW.AnkleTorque_ref = HBPW.Ankle_Torque_from_cZMP(HBPW.cZMP_proj, HBPW.ZMP_global, HBPW.qRF_ref, HBPW.qLF_ref, HBPW.pRF_ref, HBPW.pLF_ref, HBPW.RS.F_RF, HBPW.RS.F_LF);

//            // calc 'XF_qaut_ctrl' according to torque reference
////            HBPW.AnkleTorqueController_pos(HBPW.AnkleTorque_ref[0], HBPW.AnkleTorque_ref[1], HBPW.AnkleTorque_ref[2], HBPW.AnkleTorque_ref[3],
////                                        HBPW.RS.F_RF, HBPW.RS.F_LF, HBPW.RS.M_RF, HBPW.RS.M_LF);
//            HBPW.AnkleTorqueController_pos(0, 0, 0, 0,
//                                        HBPW.RS.F_RF, HBPW.RS.F_LF, HBPW.RS.M_RF, HBPW.RS.M_LF);

//            HBPW.qRF_ref = HBPW.RF_quat_ctrl;
//            HBPW.qLF_ref = HBPW.LF_quat_ctrl;

//            // -----------DSP Fz control--------------------

//            // get Alpha dsp
//            vec3 L = HBPW.pRF_ref - HBPW.pLF_ref;
//            L.z = 0;
//            vec3 L_G = HBPW.G_R_g_pitroll*L;

//            if(L_G.z >= 0.005) L_G.z = 0.005;
//            if(L_G.z <= -0.005) L_G.z = -0.005;

//            HBPW.Alpha_dsp = 0.5 - L_G.z/0.005*0.5;

//            // Calc ref Fz difference
//            //robot mass
//            double Fz_total = 320; //32kg
//            HBPW.RF_Fz_ref = HBPW.Alpha_dsp*Fz_total;
//            HBPW.LF_Fz_ref = (1.0 - HBPW.Alpha_dsp)*Fz_total;

//            HBPW.Fz_diff_ref = HBPW.LF_Fz_ref - HBPW.RF_Fz_ref;

//            // get Fz difference Error
//            HBPW.Fz_diff = HBPW.RS.F_LF.z - HBPW.RS.F_RF.z;


//            // Control
//            double dsp_ctrl_gain;

//            dsp_ctrl_gain = 0.0003;

//            HBPW.dz_ctrl = dsp_ctrl_gain*(HBPW.Fz_diff_ref - HBPW.Fz_diff);


//            double alpha_dz = 1/(1 + 2*PI*dt*5);
//            HBPW.dz_ctrl_filtered = alpha_dz*HBPW.dz_ctrl_filtered + (1.0 - alpha_dz)*HBPW.dz_ctrl;

//            HBPW.z_ctrl += HBPW.dz_ctrl_filtered*HBPW.dt;

//            if(HBPW.z_ctrl > 0.05) HBPW.z_ctrl = 0.05;
//            if(HBPW.z_ctrl < -0.05) HBPW.z_ctrl = -0.05;

//            HBPW.pRF_ref.z = + 0.5*HBPW.z_ctrl;
//            HBPW.pLF_ref.z = - 0.5*HBPW.z_ctrl;

            // Put reference Task to Trajectory Handler

//            doubles dbs_qRF(4), dbs_qLF(4);
//            for(int i=0;i<4;i++){
//                dbs_qRF[i] = HBPW.qRF_ref[i];
//                dbs_qLF[i] = HBPW.qLF_ref[i];
//            }

//            WBmotion->addRFPosInfo(HBPW.pRF_ref.x, HBPW.pRF_ref.y, HBPW.pRF_ref.z,0.005);
//            WBmotion->addLFPosInfo(HBPW.pLF_ref.x, HBPW.pLF_ref.y, HBPW.pLF_ref.z,0.005);
//            WBmotion->addRFOriInfo(dbs_qRF,0.005);
//            WBmotion->addLFOriInfo(dbs_qLF,0.005);

//            HBPW.k++;

//            save_onestep(HBPW.k);


            //// --------------------------temp

//            quat Qu1 = quat(vec3(0,0,1),D2R*181)*quat(vec3(0,1,0),D2R*20)*quat(vec3(1,0,0),D2R*-10);
//            quat Qu2 = quat(vec3(0,0,1),D2R*-179)*quat(vec3(0,1,0),D2R*20)*quat(vec3(1,0,0),D2R*-10);
//            quat Qu3 = quat(vec3(0,0,1),D2R*-90)*quat(vec3(0,1,0),D2R*20)*quat(vec3(1,0,0),D2R*-10);

//            rpy Euler_Qu1 = rpy(Qu1);
//            rpy Euler_Qu2 = rpy(Qu2);
//            rpy Euler_Qu3 = rpy(Qu3);

//            //if(Euler_Qu.y < 0) Euler_Qu.y = Euler_Qu.y + 360*D2R;

//            cout<<"Qu1: "<<Qu1.w<<", "<<Qu1.x<<", "<<Qu1.y<<", "<<Qu1.z<<endl;
//            cout<<"Qu2: "<<Qu2.w<<", "<<Qu2.x<<", "<<Qu2.y<<", "<<Qu2.z<<endl;
//            cout<<"Qu3: "<<Qu3.w<<", "<<Qu3.x<<", "<<Qu3.y<<", "<<Qu3.z<<endl;

//            cout<<"yaw : "<<Euler_Qu1.y*R2D<<", pitch: "<<Euler_Qu1.p*R2D<<", roll: "<<Euler_Qu1.r*R2D<<endl;
//            cout<<"yaw : "<<Euler_Qu2.y*R2D<<", pitch: "<<Euler_Qu2.p*R2D<<", roll: "<<Euler_Qu2.r*R2D<<endl;
//            cout<<"yaw : "<<Euler_Qu3.y*R2D<<", pitch: "<<Euler_Qu3.p*R2D<<", roll: "<<Euler_Qu3.r*R2D<<endl<<endl;

            ////---------------------------- Logging
            HBPW.Logging();

            break;
        }

        case _task_Idle:


            break;
        }


    if(WB_FLAG == 1){
        WBmotion->updateAll_HB();

        userData->M2G.valveMode = 9;

        //WBmotion->WBIK();
        WBmotion->WBIK_xy_pelz();

        userData->M2G.valveMode = 10;

        joint->SetJointRefAngle(WST, WST_ref_deg);
        joint->SetJointRefAngle(NKY, NeckYaw_ref_deg);
        joint->SetJointRefAngle(RSP, RSP_ref_deg_offset + RSP_ref_deg);
        joint->SetJointRefAngle(LSP, LSP_ref_deg_offset + LSP_ref_deg);

        //Right leg
        double RHY_ref_deg = WBmotion->LJ.RHY*R2D + RHY_con_deg;
        if(RHY_ref_deg > 30.0) RHY_ref_deg == 30.0;
        if(RHY_ref_deg < -30.0) RHY_ref_deg == -30.0;

        joint->SetJointRefAngle(RHY,RHY_ref_deg);
        joint->SetJointRefAngle(RHR,WBmotion->LJ.RHR*R2D + RHR_con_deg);
        joint->SetJointRefAngle(RHP,WBmotion->LJ.RHP*R2D + RHP_con_deg);
        joint->SetJointRefAngle(RKN,WBmotion->LJ.RKN*R2D + RKN_con_deg);
//        //for gazelle2
//        double Ankle_gear_ratio = 140;
//        double RKN_rot_compen_deg = WBmotion->LJ.RKN*R2D/Ankle_gear_ratio;

//        GK.IK_Ankle_right(WBmotion->LJ.RAP*R2D + HBPW.RF_angle_ctrl.y*R2D, WBmotion->LJ.RAR*R2D + HBPW.RF_angle_ctrl.x*R2D, RA1_ref_deg, RA2_ref_deg);
//        joint->SetJointRefAngle(RAP, RA1_ref_deg - RKN_rot_compen_deg);
//        joint->SetJointRefAngle(RAR, RA2_ref_deg - RKN_rot_compen_deg);
        joint->SetJointRefAngle(RAP,WBmotion->LJ.RAP*R2D + RF_angle_con_deg.y);
        joint->SetJointRefAngle(RAR,WBmotion->LJ.RAR*R2D + RF_angle_con_deg.x);

        // Left Leg
        double LHY_ref_deg = WBmotion->LJ.LHY*R2D + LHY_con_deg;
        if(LHY_ref_deg > 30.0) LHY_ref_deg == 30.0;
        if(LHY_ref_deg < -30.0) LHY_ref_deg == -30.0;

        joint->SetJointRefAngle(LHY,LHY_ref_deg);
        joint->SetJointRefAngle(LHR,WBmotion->LJ.LHR*R2D + LHR_con_deg);
        joint->SetJointRefAngle(LHP,WBmotion->LJ.LHP*R2D + LHP_con_deg);
        joint->SetJointRefAngle(LKN,WBmotion->LJ.LKN*R2D + LKN_con_deg);
//        //for gazelle
//        double LKN_rot_compen_deg = WBmotion->LJ.LKN*R2D/Ankle_gear_ratio;

//        GK.IK_Ankle_left(WBmotion->LJ.LAP*R2D + HBPW.LF_angle_ctrl.y*R2D, WBmotion->LJ.LAR*R2D + HBPW.LF_angle_ctrl.x*R2D, LA1_ref_deg, LA2_ref_deg);
//        joint->SetJointRefAngle(LAP, LA1_ref_deg - LKN_rot_compen_deg);
//        joint->SetJointRefAngle(LAR, LA2_ref_deg - LKN_rot_compen_deg);
        joint->SetJointRefAngle(LAP,WBmotion->LJ.LAP*R2D + LF_angle_con_deg.y);
        joint->SetJointRefAngle(LAR,WBmotion->LJ.LAR*R2D + LF_angle_con_deg.x);

        userData->M2G.valveMode = 11;


    }

        joint->MoveAllJoint();

        userData->M2G.valveMode = 12;
        rt_task_suspend(&rtTaskCon);

        userData->M2G.valveMode = 13;
    }
}
//==============================//



//==============================//
// Flag Thread
//==============================//
void RBFlagThread(void *)
{
    rt_task_set_periodic(NULL, TM_NOW, 200*1000);        // 5 usec

    while(__IS_WORKING)
    {
        rt_task_wait_period(NULL);

        //if(HasAnyOwnership()){
            if(sharedData->SYNC_SIGNAL[PODO_NO] == true){
                joint->JointUpdate();
                rt_task_resume(&rtTaskCon);
            }
        //}
    }
}
//==============================//

void ResetGlobalCoord(int RF_OR_LF_OR_PC_MidFoot){
    switch(RF_OR_LF_OR_PC_MidFoot)
    {
    case 1: //MidFoot
    {
        double LowerFootz;
        if(WBmotion->pRF_3x1[2] < WBmotion->pLF_3x1[2] - 0.0001){
            LowerFootz = WBmotion->pRF_3x1[2];
        }
        else if(WBmotion->pRF_3x1[2] > WBmotion->pLF_3x1[2] + 0.0001){
            LowerFootz = WBmotion->pLF_3x1[2];
        }
        else{
            LowerFootz = (WBmotion->pRF_3x1[2] + WBmotion->pLF_3x1[2])/2;
        }
        vec3 midfoot = vec3((WBmotion->pRF_3x1[0] + WBmotion->pLF_3x1[0])/2, (WBmotion->pRF_3x1[1] + WBmotion->pLF_3x1[1])/2, LowerFootz);
        cout<<"midFoot : ("<<midfoot.x<<", "<<midfoot.y<<", "<<midfoot.z<<")"<<endl;

        WBmotion->pCOM_3x1[0] = WBmotion->pCOM_3x1[0] - midfoot.x;
        WBmotion->pCOM_3x1[1] = WBmotion->pCOM_3x1[1] - midfoot.y;
        WBmotion->pCOM_3x1[2] = WBmotion->pCOM_3x1[2]- midfoot.z;

        WBmotion->pRF_3x1[0] = WBmotion->pRF_3x1[0] - midfoot.x;
        WBmotion->pRF_3x1[1] = WBmotion->pRF_3x1[1] - midfoot.y;
        WBmotion->pRF_3x1[2] = WBmotion->pRF_3x1[2] - midfoot.z;

        WBmotion->pLF_3x1[0] = WBmotion->pLF_3x1[0] - midfoot.x;
        WBmotion->pLF_3x1[1] = WBmotion->pLF_3x1[1] - midfoot.y;
        WBmotion->pLF_3x1[2] = WBmotion->pLF_3x1[2] - midfoot.z;

        WBmotion->pPel_3x1[0] = WBmotion->pPel_3x1[0] - midfoot.x;
        WBmotion->pPel_3x1[1] = WBmotion->pPel_3x1[1] - midfoot.y;
        WBmotion->pPel_3x1[2] = WBmotion->pPel_3x1[2] - midfoot.z;

//        WBmotion->addCOMInfo(WBmotion->pCOM_3x1[0], WBmotion->pCOM_3x1[1], WBmotion->pCOM_3x1[2],0.005);
//        WBmotion->addRFPosInfo(WBmotion->pRF_3x1[0], WBmotion->pRF_3x1[1], WBmotion->pRF_3x1[2],0.005);
//        WBmotion->addLFPosInfo(WBmotion->pLF_3x1[0], WBmotion->pLF_3x1[1], WBmotion->pLF_3x1[2],0.005);
//        WBmotion->updateAll();
        WBmotion->addCOMInfo_HB(WBmotion->pCOM_3x1[0], WBmotion->pCOM_3x1[1], WBmotion->pCOM_3x1[2]);
        WBmotion->addRFPosInfo_HB(WBmotion->pRF_3x1[0], WBmotion->pRF_3x1[1], WBmotion->pRF_3x1[2]);
        WBmotion->addLFPosInfo_HB(WBmotion->pLF_3x1[0], WBmotion->pLF_3x1[1], WBmotion->pLF_3x1[2]);
        WBmotion->addPELPosInfo_HB(WBmotion->pPel_3x1[2]);
        quat qDefalt = quat();
        WBmotion->addPELOriInfo_HB(qDefalt);
        WBmotion->addRFOriInfo_HB(qDefalt);
        WBmotion->addLFOriInfo_HB(qDefalt);
        WBmotion->updateAll_HB();

        WBmotion->WBIK();

        break;
    }
    }
}

double getEnc(int in)
{

    double a = sharedData->ENCODER[MC_ID_CH_Pairs[in].id][MC_ID_CH_Pairs[in].ch].CurrentPosition*M_PI/180.;

    return a;
}
double getPosRef(int in)
{

    double a = sharedData->ENCODER[MC_ID_CH_Pairs[in].id][MC_ID_CH_Pairs[in].ch].CurrentReference*M_PI/180.;

    return a;
}
double getEncVel(int in)
{

    double a = sharedData->ENCODER[MC_ID_CH_Pairs[in].id][MC_ID_CH_Pairs[in].ch].CurrentVelocity*M_PI/180.;

    return a;
}

void SensorInput(){
    // Fill in the ROBOTSEN struct
    vec3 F_RF = vec3(sharedData->FT[0].Fx,sharedData->FT[0].Fy,sharedData->FT[0].Fz);
    vec3 F_LF = vec3(sharedData->FT[1].Fx,sharedData->FT[1].Fy,sharedData->FT[1].Fz);
//    vec3 F_RF = vec3(0,0,sharedSEN->FT[0].Fz);
//    vec3 F_LF = vec3(0,0,sharedSEN->FT[1].Fz);
    vec3 M_RF = vec3(sharedData->FT[0].Mx,sharedData->FT[0].My,0);
    vec3 M_LF = vec3(sharedData->FT[1].Mx,sharedData->FT[1].My,0);

    vec3 IMUangle(sharedData->IMU[0].Roll,sharedData->IMU[0].Pitch,sharedData->IMU[0].Yaw);   // Gyro Integration
    //vec3 IMUangle_comp(sharedData->IMU[0].Roll_Comp,sharedData->IMU[0].Pitch_Comp,sharedData->IMU[0].Yaw);   // Complementary filter
    //vec3 IMUangle_comp = IMUangle;

    quat IMUquat(sharedData->IMU[0].Q[0], sharedData->IMU[0].Q[1], sharedData->IMU[0].Q[2], sharedData->IMU[0].Q[3]);
    vec3 IMUvel = vec3(sharedData->IMU[0].RollVel,sharedData->IMU[0].PitchVel,sharedData->IMU[0].YawVel);
    vec3 IMUacc(sharedData->IMU[0].AccX,sharedData->IMU[0].AccY,sharedData->IMU[0].AccZ);
    vec3 ACC_RF = vec3(sharedData->FT[0].Pitch*4.0, -sharedData->FT[0].Roll*4.0,0);
    vec3 ACC_LF = vec3(sharedData->FT[1].Pitch*4.0, -sharedData->FT[1].Roll*4.0,0);


    RS_temp.F_RF = F_RF;
    RS_temp.F_LF = F_LF;
    RS_temp.M_RF = M_RF;
    RS_temp.M_LF = M_LF;

    RS_temp.ACC_RF = ACC_RF;
    RS_temp.ACC_LF = ACC_LF;

    RS_temp.IMUangle = IMUangle*D2R;
//    RS_temp.IMUangle_comp = IMUangle_comp*D2R;

    RS_temp.IMUquat = IMUquat;
    RS_temp.IMULocalW = IMUvel*D2R;
    RS_temp.IMUomega = mat3(RS_temp.IMUquat)*RS_temp.IMULocalW;

        
}


void save_onestep(int cnt){
    SAVE[0][cnt] = HBPW.COM_ref.x;
    SAVE[1][cnt] = HBPW.CP_ref.x;
    SAVE[2][cnt] = HBPW.COM_ref.y;
    SAVE[3][cnt] = HBPW.CP_ref.y;
    SAVE[4][cnt] = HBPW.p_ref[0].x;
    SAVE[5][cnt] = HBPW.p_ref[0].y;
    SAVE[6][cnt] = HBPW.p_out.x;
    SAVE[7][cnt] = HBPW.p_out.y;
    SAVE[8][cnt] = HBPW.COM_LIPM.x;
    SAVE[9][cnt] = HBPW.COM_LIPM.y;

    SAVE[10][cnt] = HBPW.dCOM_ref.x;
    SAVE[11][cnt] = HBPW.dCOM_ref.y;
    SAVE[12][cnt] = HBPW.cZMP.x;
    SAVE[13][cnt] = HBPW.cZMP.y;
    SAVE[14][cnt] = HBPW.ZMP_global.x;
    SAVE[15][cnt] = HBPW.ZMP_global.y;
    SAVE[16][cnt] = HBPW.cZMP_proj.x;
    SAVE[17][cnt] = HBPW.cZMP_proj.y;

    SAVE[18][cnt] = HBPW.RS.IMUangle.x;
    SAVE[19][cnt] = HBPW.RS.IMUangle.y;
    SAVE[20][cnt] = HBPW.RS.IMUangle.z;

    SAVE[21][cnt] = HBPW.X_obs[0];
    SAVE[22][cnt] = HBPW.Y_obs[0];
    SAVE[23][cnt] = HBPW.X_obs[1];
    SAVE[24][cnt] = HBPW.Y_obs[1];
    SAVE[25][cnt] = HBPW.RS.IMULocalW.x;
    SAVE[26][cnt] = HBPW.RS.IMULocalW.y;

    SAVE[27][cnt] = HBPW.dT;//dT_est;//dT_buf[0];
    SAVE[28][cnt] = HBPW.pRF_ref.x;
    SAVE[29][cnt] = HBPW.pLF_ref.x;

    SAVE[30][cnt] = HBPW.pRF_ref.z;
    SAVE[31][cnt] = HBPW.pLF_ref.z;
    SAVE[32][cnt] = HBPW.ZMP_error_local.y;

    SAVE[33][cnt] = 0; // COM_measure x
    SAVE[34][cnt] = 0; // COM_measure y

    SAVE[35][cnt] = HBPW.uCOM.x;
    SAVE[36][cnt] = HBPW.uCOM.y;

    SAVE[37][cnt] = HBPW.CP_m_filtered.x; // temp x
    SAVE[38][cnt] = HBPW.CP_m_filtered.y; // temp y

    SAVE[39][cnt] = HBPW.COM_error_local.y;
    SAVE[40][cnt] = HBPW.SDB[HBPW.step_phase].swingFoot;

    SAVE[41][cnt] = HBPW.COM_m.x;
    SAVE[42][cnt] = HBPW.COM_m.y;
    SAVE[43][cnt] = HBPW.COM_damping_con.x;
    SAVE[44][cnt] = HBPW.COM_damping_con.y;

    SAVE[45][cnt] = HBPW.CP_m.y; // ZMP x estimation
    SAVE[46][cnt] = HBPW.CP_m.x;
    SAVE[47][cnt] = 0;
    SAVE[48][cnt] = HBPW.pRF_ref.x;
    SAVE[49][cnt] = HBPW.pRF_ref.y;

    SAVE[50][cnt] = HBPW.RS.F_RF.z;
    SAVE[51][cnt] = HBPW.RS.F_LF.z;
    SAVE[52][cnt] = HBPW.pLF_ref.x;
    SAVE[53][cnt] = HBPW.pLF_ref.y;
    SAVE[54][cnt] = 0;

    SAVE[55][cnt] = 0;
    SAVE[56][cnt] = 0;
    SAVE[57][cnt] = HBPW.Fz_diff_error;

    SAVE[58][cnt] = HBPW.ACC_RF_filtered.x;
    SAVE[59][cnt] = HBPW.ACC_RF_filtered.y;
    SAVE[60][cnt] = HBPW.ACC_LF_filtered.x;
    SAVE[61][cnt] = HBPW.ACC_LF_filtered.y;

    SAVE[62][cnt] = HBPW.Ye_obs[0];
    SAVE[63][cnt] = HBPW.Ye_obs[1];
    SAVE[64][cnt] = HBPW.Xe_obs[0];
    SAVE[65][cnt] = HBPW.Xe_obs[1];
    SAVE[66][cnt] = HBPW.L_roll_obs[0];
    SAVE[67][cnt] = HBPW.LHR_con_deg;

    SAVE[68][cnt] = HBPW.dCOM_m_diff.x;
    SAVE[69][cnt] = HBPW.dCOM_m_diff.y;
    SAVE[70][cnt] = HBPW.dCOM_m_imu.x;
    SAVE[71][cnt] = HBPW.dCOM_m_imu.y;
    SAVE[72][cnt] = 0;
    SAVE[73][cnt] = 0;

    SAVE[74][cnt] = HBPW.RS.M_RF.x;
    SAVE[75][cnt] = HBPW.RS.M_RF.y;
    SAVE[76][cnt] = HBPW.RS.F_RF.z;
    SAVE[77][cnt] = HBPW.RS.M_LF.x;
    SAVE[78][cnt] = HBPW.RS.M_LF.y;
    SAVE[79][cnt] = HBPW.RS.F_LF.z;

    SAVE[80][cnt] = HBPW.RF_z_dz_ddz[0];
    SAVE[81][cnt] = HBPW.RF_z_dz_ddz[1];
    SAVE[82][cnt] = HBPW.LF_z_dz_ddz[0];
    SAVE[83][cnt] = HBPW.LF_z_dz_ddz[1];
    SAVE[84][cnt] = HBPW.RF_landing_flag;
    SAVE[85][cnt] = HBPW.LF_landing_flag;

    SAVE[86][cnt] = HBPW.pRF_landing.z;
    SAVE[87][cnt] = HBPW.pLF_landing.z;
    SAVE[88][cnt] = 0;
    SAVE[89][cnt] = 0;

    SAVE[90][cnt] = HBPW.COM_y_dy_ddy_SA[0];
    SAVE[91][cnt] = HBPW.COM_y_dy_ddy_SA[1];
    SAVE[92][cnt] = HBPW.COM_y_dy_ddy_SA[2];
    SAVE[93][cnt] = HBPW.COM_x_dx_ddx_SA[0];
    SAVE[94][cnt] = HBPW.COM_x_dx_ddx_SA[1];
    SAVE[95][cnt] = HBPW.COM_x_dx_ddx_SA[2];
    SAVE[96][cnt] = HBPW.p_out_SA.y;
    SAVE[97][cnt] = HBPW.p_out_SA.x;
    SAVE[98][cnt] = HBPW.p_ref_SA[0].y;
    SAVE[99][cnt] = HBPW.p_ref_SA[0].x;

    SAVE[100][cnt] = HBPW.COM_SA_ref.y;
    SAVE[101][cnt] = HBPW.COM_SA_ref.x;
    SAVE[102][cnt] = HBPW.dCOM_SA_ref.y;
    SAVE[103][cnt] = HBPW.dCOM_SA_ref.x;
    SAVE[104][cnt] = HBPW.t_now;
    SAVE[105][cnt] = HBPW.CP_SA_ref_local.x;
    SAVE[106][cnt] = HBPW.CP_SA_ref_local.y;
    SAVE[107][cnt] = 0;
    SAVE[108][cnt] = HBPW.Landing_delXY.x;
    SAVE[109][cnt] = HBPW.Landing_delXY.y;

    SAVE[110][cnt] = HBPW.CP_error_lf.x;
    SAVE[111][cnt] = HBPW.CP_error_lf.y;
    SAVE[112][cnt] = HBPW.cZMP_SA_lf.x;
    SAVE[113][cnt] = HBPW.cZMP_SA_lf.y;
    SAVE[114][cnt] = HBPW.RF_y_dy_ddy[0];
    SAVE[115][cnt] = HBPW.RF_y_dy_ddy[1];
    SAVE[116][cnt] = HBPW.pRF_landing.x;
    SAVE[117][cnt] = HBPW.pRF_landing.y;
    SAVE[118][cnt] = 0;
    SAVE[119][cnt] = 0;

    SAVE[120][cnt] = HBPW.CP_error_rf.x;
    SAVE[121][cnt] = HBPW.CP_error_rf.y;
    SAVE[122][cnt] = HBPW.cZMP_SA_rf.x;
    SAVE[123][cnt] = HBPW.cZMP_SA_rf.y;
    SAVE[124][cnt] = HBPW.LF_y_dy_ddy[0];
    SAVE[125][cnt] = HBPW.LF_y_dy_ddy[1];
    SAVE[126][cnt] = HBPW.pLF_landing.x;
    SAVE[127][cnt] = HBPW.pLF_landing.y;
    SAVE[128][cnt] = 0;
    SAVE[129][cnt] = 0;

    SAVE[130][cnt] = HBPW.del_u_f.x;
    SAVE[131][cnt] = HBPW.del_u_f.y;
    SAVE[132][cnt] = HBPW.del_b_f.x;
    SAVE[133][cnt] = HBPW.del_b_f.y;
    SAVE[134][cnt] = HBPW.new_T;
    SAVE[135][cnt] = HBPW.del_u_g.x;
    SAVE[136][cnt] = HBPW.del_u_g.y;
    SAVE[137][cnt] = HBPW.del_b_g.x;
    SAVE[138][cnt] = HBPW.del_b_g.y;
    SAVE[139][cnt] = HBPW.SA_Enable_flag;

    SAVE[140][cnt] = HBPW.del_u_f_filtered.x;
    SAVE[141][cnt] = HBPW.del_u_f_filtered.y;
    SAVE[142][cnt] = HBPW.del_b_f_filtered.x;
    SAVE[143][cnt] = HBPW.del_b_f_filtered.y;
    SAVE[144][cnt] = HBPW.new_T_filtered;
    SAVE[145][cnt] = HBPW.UD_flag;
    SAVE[146][cnt] = HBPW.LandingZ_des;
    SAVE[147][cnt] = 0;
    SAVE[148][cnt] = HBPW.Pelv_roll_ref;
    SAVE[149][cnt] = HBPW.Pelv_pitch_ref;

    SAVE[150][cnt] = HBPW.Omega_pitch;
    SAVE[151][cnt] = HBPW.Omega_roll;
    SAVE[152][cnt] = HBPW.Omega_pitch_filtered;
    SAVE[153][cnt] = HBPW.Omega_roll_filtered;
    SAVE[154][cnt] = HBPW.del_b0_Nf.x;
    SAVE[155][cnt] = HBPW.del_b0_Nf.y;
    SAVE[156][cnt] = HBPW.b0_Nf.x;
    SAVE[157][cnt] = HBPW.b0_Nf.y;
    SAVE[158][cnt] = HBPW.SDB[HBPW.step_phase].t;
    SAVE[159][cnt] = HBPW.T_nom;

    SAVE[160][cnt] = HBPW.del_u_Nf.x;
    SAVE[161][cnt] = HBPW.del_u_Nf.y;
    SAVE[162][cnt] = HBPW.del_b_Nf.x;
    SAVE[163][cnt] = HBPW.del_b_Nf.y;
    SAVE[164][cnt] = HBPW.new_T_Nf;
    SAVE[165][cnt] = HBPW.del_u_Nf_filtered.x;
    SAVE[166][cnt] = HBPW.del_u_Nf_filtered.y;
    SAVE[167][cnt] = HBPW.del_b_Nf_filtered.x;
    SAVE[168][cnt] = HBPW.del_b_Nf_filtered.y;
    SAVE[169][cnt] = HBPW.new_T_Nf_filtered;

    SAVE[170][cnt] = HBPW.dz_com_ctrl;
    SAVE[171][cnt] = HBPW.z_com_ctrl;
    SAVE[172][cnt] = 0;
    SAVE[173][cnt] = 0;
    SAVE[174][cnt] = 0;
    SAVE[175][cnt] = 0;
    SAVE[176][cnt] = 0;
    SAVE[177][cnt] = HBPW.G_R_g_pitroll_rpy.r;
    SAVE[178][cnt] = HBPW.G_R_g_pitroll_rpy.p;
    SAVE[179][cnt] = HBPW.G_R_g_pitroll_rpy.y;

    SAVE[180][cnt] = sharedData->ENCODER[MC_ID_CH_Pairs[RHY].id][MC_ID_CH_Pairs[RHY].ch].CurrentPosition;
    SAVE[181][cnt] = sharedData->ENCODER[MC_ID_CH_Pairs[RHR].id][MC_ID_CH_Pairs[RHR].ch].CurrentPosition;
    SAVE[182][cnt] = sharedData->ENCODER[MC_ID_CH_Pairs[RHP].id][MC_ID_CH_Pairs[RHP].ch].CurrentPosition;
    SAVE[183][cnt] = sharedData->ENCODER[MC_ID_CH_Pairs[RKN].id][MC_ID_CH_Pairs[RKN].ch].CurrentPosition;
    SAVE[184][cnt] = sharedData->ENCODER[MC_ID_CH_Pairs[RAP].id][MC_ID_CH_Pairs[RAP].ch].CurrentPosition;
    SAVE[185][cnt] = sharedData->ENCODER[MC_ID_CH_Pairs[RAR].id][MC_ID_CH_Pairs[RAR].ch].CurrentPosition;
    SAVE[186][cnt] = 0;
    SAVE[187][cnt] = 0;
    SAVE[188][cnt] = 0;
    SAVE[189][cnt] = 0;

    SAVE[190][cnt] = sharedData->ENCODER[MC_ID_CH_Pairs[LHY].id][MC_ID_CH_Pairs[LHY].ch].CurrentPosition;
    SAVE[191][cnt] = sharedData->ENCODER[MC_ID_CH_Pairs[LHR].id][MC_ID_CH_Pairs[LHR].ch].CurrentPosition;
    SAVE[192][cnt] = sharedData->ENCODER[MC_ID_CH_Pairs[LHP].id][MC_ID_CH_Pairs[LHP].ch].CurrentPosition;
    SAVE[193][cnt] = sharedData->ENCODER[MC_ID_CH_Pairs[LKN].id][MC_ID_CH_Pairs[LKN].ch].CurrentPosition;
    SAVE[194][cnt] = sharedData->ENCODER[MC_ID_CH_Pairs[LAP].id][MC_ID_CH_Pairs[LAP].ch].CurrentPosition;
    SAVE[195][cnt] = sharedData->ENCODER[MC_ID_CH_Pairs[LAR].id][MC_ID_CH_Pairs[LAR].ch].CurrentPosition;
    SAVE[196][cnt] = 0;
    SAVE[197][cnt] = 0;
    SAVE[198][cnt] = 0;
    SAVE[199][cnt] = 0;

    SAVE[200][cnt] = WBmotion->LJ.RHY*R2D + RHY_con_deg;
    SAVE[201][cnt] = WBmotion->LJ.RHR*R2D + RHR_con_deg;
    SAVE[202][cnt] = WBmotion->LJ.RHP*R2D + RHP_con_deg;
    SAVE[203][cnt] = WBmotion->LJ.RKN*R2D;
    SAVE[204][cnt] = RA1_ref_deg;  //RAP board
    SAVE[205][cnt] = RA2_ref_deg;  //RAR board
    SAVE[206][cnt] = 0;
    SAVE[207][cnt] = RHY_con_deg;
    SAVE[208][cnt] = RHR_con_deg;
    SAVE[209][cnt] = RHP_con_deg;

    SAVE[210][cnt] = WBmotion->LJ.LHY*R2D + LHY_con_deg;
    SAVE[211][cnt] = WBmotion->LJ.LHR*R2D + LHR_con_deg;
    SAVE[212][cnt] = WBmotion->LJ.LHP*R2D + LHP_con_deg;
    SAVE[213][cnt] = WBmotion->LJ.LKN*R2D;
    SAVE[214][cnt] = LA1_ref_deg; //LAP board
    SAVE[215][cnt] = LA2_ref_deg; //LAR board
    SAVE[216][cnt] = 0;
    SAVE[217][cnt] = LHY_con_deg;
    SAVE[218][cnt] = LHR_con_deg;
    SAVE[219][cnt] = LHP_con_deg;

    SAVE[220][cnt] = HBPW.AnkleTorque_ref[0]; //RAR ref torque
    SAVE[221][cnt] = HBPW.AnkleTorque_ref[1]; //RAP ref torque
    SAVE[222][cnt] = HBPW.AnkleTorque_ref[2]; //LAR ref torque
    SAVE[223][cnt] = HBPW.AnkleTorque_ref[3]; //LAP ref torque
    SAVE[224][cnt] = HBPW.RF_Fz_ref;
    SAVE[225][cnt] = HBPW.LF_Fz_ref;
    SAVE[226][cnt] = 0;
    SAVE[227][cnt] = 0;
    SAVE[228][cnt] = 0;
    SAVE[229][cnt] = HBPW.Alpha_dsp;

    SAVE[230][cnt] = HBPW.DSP_time_flag;
    SAVE[231][cnt] = HBPW.DSP_force_flag;
    SAVE[232][cnt] = HBPW.RF_Fz_ref;
    SAVE[233][cnt] = HBPW.LF_Fz_ref;
    SAVE[234][cnt] = HBPW.Fz_diff_ref;
    SAVE[235][cnt] = HBPW.Fz_diff;
    SAVE[236][cnt] = HBPW.dz_ctrl;
    SAVE[237][cnt] = HBPW.dz_ctrl_filtered;
    SAVE[238][cnt] = HBPW.z_ctrl;
    SAVE[239][cnt] = HBPW.z_ctrl_filtered;

    SAVE[240][cnt] = HBPW.dRF_angle_ctrl.x;
    SAVE[241][cnt] = HBPW.dRF_angle_ctrl.y;
    SAVE[242][cnt] = HBPW.RF_angle_ctrl.x;
    SAVE[243][cnt] = HBPW.RF_angle_ctrl.y;
    SAVE[244][cnt] = HBPW.dLF_angle_ctrl.x;
    SAVE[245][cnt] = HBPW.dLF_angle_ctrl.y;
    SAVE[246][cnt] = HBPW.LF_angle_ctrl.x;
    SAVE[247][cnt] = HBPW.LF_angle_ctrl.y;
    SAVE[248][cnt] = 0;
    SAVE[249][cnt] = 0;

    SAVE[250][cnt] = HBPW.RS.IMUangle_comp.x;
    SAVE[251][cnt] = HBPW.RS.IMUangle_comp.y;
    SAVE[252][cnt] = HBPW.COM_m_comp.x;
    SAVE[253][cnt] = HBPW.COM_m_comp.y;
    SAVE[254][cnt] = HBPW.dCOM_m_comp.x;
    SAVE[255][cnt] = HBPW.dCOM_m_comp.y;
    SAVE[256][cnt] = HBPW.CP_m_comp.x;
    SAVE[257][cnt] = HBPW.CP_m_comp.y;
    SAVE[258][cnt] = HBPW.cZMP_TC_proj.x;
    SAVE[259][cnt] = HBPW.cZMP_TC_proj.y;

    SAVE[260][cnt] = HBPW.COM_e_imu_local.x;
    SAVE[261][cnt] = HBPW.COM_e_imu_local.y;
    SAVE[262][cnt] = HBPW.dCOM_e_imu_local.x;
    SAVE[263][cnt] = HBPW.dCOM_e_imu_local.y;
    SAVE[264][cnt] = HBPW.dsp_ctrl_gain;
    SAVE[265][cnt] = HBPW.dsp_tilt_gain;
    SAVE[266][cnt] = HBPW.cZMP_dsp_global.y;
    SAVE[267][cnt] = HBPW.cZMP_dsp_global.x;
    SAVE[268][cnt] = HBPW.cZMP_dsp_global_proj.y;
    SAVE[269][cnt] = HBPW.cZMP_dsp_global_proj.x;

    SAVE[270][cnt] = HBPW.COM_m_comp_filtered.x;
    SAVE[271][cnt] = HBPW.COM_m_comp_filtered.y;
    SAVE[272][cnt] = HBPW.dCOM_m_comp_filtered.x;
    SAVE[273][cnt] = HBPW.dCOM_m_comp_filtered.y;
    SAVE[274][cnt] = HBPW.Pelv_pitch_acc_ref;
    SAVE[275][cnt] = HBPW.Pelv_pitch_vel_ref;
    SAVE[276][cnt] = HBPW.Pelv_pitch_ref;
    SAVE[277][cnt] = HBPW.Pelv_roll_acc_ref;
    SAVE[278][cnt] = HBPW.Pelv_roll_vel_ref;
    SAVE[279][cnt] = HBPW.Pelv_roll_ref;
}


void save_all()
{
    printf("walk finished and saved%d\n",HBPW.k);
    FILE* ffp2 = fopen("/home/rainbow/Desktop/HBtest_Walking_Data_prev1.txt","w");
    for(int i=0;i<HBPW.k;i++)
    {
        for(int j=0;j<SAVEN;j++)
        {
            fprintf(ffp2,"%f\t",SAVE[j][i]);
        }
        fprintf(ffp2,"\n");
    }

    fclose(ffp2);
    printf("save done\n");
}


doubles quat2doubles(quat _quat){
    doubles q;
    for(int i=0;i<4;i++){
        q[i] = _quat[i];
    }
    return q;
}

void ArmGainOverride(void){

    cout<<"arm gain over"<<endl;
    //RSR
    MCsetFrictionParameter(JOINT_INFO[RSR].canch, JOINT_INFO[RSR].bno, JOINT_INFO[RSR].mch, 1000, 7, 0);
    MCBoardSetSwitchingMode(JOINT_INFO[RSR].canch, JOINT_INFO[RSR].bno, SW_MODE_NON_COMPLEMENTARY);
    MCenableFrictionCompensation(JOINT_INFO[RSR].canch, JOINT_INFO[RSR].bno, JOINT_INFO[RSR].mch, ENABLE);
    MCJointGainOverride(JOINT_INFO[RSR].canch, JOINT_INFO[RSR].bno, JOINT_INFO[RSR].mch, 35, 10);


    MCsetFrictionParameter(JOINT_INFO[REB].canch, JOINT_INFO[REB].bno, JOINT_INFO[REB].mch, 1000, 7, 0);
    MCBoardSetSwitchingMode(JOINT_INFO[REB].canch, JOINT_INFO[REB].bno, SW_MODE_NON_COMPLEMENTARY);
    MCenableFrictionCompensation(JOINT_INFO[REB].canch, JOINT_INFO[REB].bno, JOINT_INFO[REB].mch, ENABLE);
    MCJointGainOverride(JOINT_INFO[REB].canch, JOINT_INFO[REB].bno, JOINT_INFO[REB].mch, 35, 10);
    usleep(5000);

    //LSR
    MCsetFrictionParameter(JOINT_INFO[LSR].canch, JOINT_INFO[LSR].bno, JOINT_INFO[LSR].mch, 1000, 7, 0);
    MCBoardSetSwitchingMode(JOINT_INFO[LSR].canch, JOINT_INFO[LSR].bno, SW_MODE_NON_COMPLEMENTARY);
    MCenableFrictionCompensation(JOINT_INFO[LSR].canch, JOINT_INFO[LSR].bno, JOINT_INFO[LSR].mch, ENABLE);
    MCJointGainOverride(JOINT_INFO[LSR].canch, JOINT_INFO[LSR].bno, JOINT_INFO[LSR].mch, 35, 10);


    MCsetFrictionParameter(JOINT_INFO[LEB].canch, JOINT_INFO[LEB].bno, JOINT_INFO[LEB].mch, 1000, 7, 0);
    MCBoardSetSwitchingMode(JOINT_INFO[LEB].canch, JOINT_INFO[LEB].bno, SW_MODE_NON_COMPLEMENTARY);
    MCenableFrictionCompensation(JOINT_INFO[LEB].canch, JOINT_INFO[LEB].bno, JOINT_INFO[LEB].mch, ENABLE);
    MCJointGainOverride(JOINT_INFO[LEB].canch, JOINT_INFO[LEB].bno, JOINT_INFO[LEB].mch, 35, 10);
    usleep(5000);
}

void ArmGainOverride_Off(void){
    MCenableFrictionCompensation(JOINT_INFO[LSR].canch, JOINT_INFO[LSR].bno, JOINT_INFO[LSR].mch, DISABLE);
    MCBoardSetSwitchingMode(JOINT_INFO[LSR].canch,JOINT_INFO[LSR].bno, SW_MODE_COMPLEMENTARY);

    MCJointGainOverride(JOINT_INFO[LSR].canch, JOINT_INFO[LSR].bno, JOINT_INFO[LSR].mch, 0,1500); //--LSR


    MCenableFrictionCompensation(JOINT_INFO[RSR].canch, JOINT_INFO[RSR].bno, JOINT_INFO[RSR].mch, DISABLE);
    MCBoardSetSwitchingMode(JOINT_INFO[RSR].canch,JOINT_INFO[RSR].bno, SW_MODE_COMPLEMENTARY);

    MCJointGainOverride(JOINT_INFO[RSR].canch, JOINT_INFO[RSR].bno, JOINT_INFO[RSR].mch, 0,1500); //--RSR

    usleep(5000);

    MCenableFrictionCompensation(JOINT_INFO[LEB].canch, JOINT_INFO[LEB].bno, JOINT_INFO[LEB].mch, DISABLE);
    MCBoardSetSwitchingMode(JOINT_INFO[LEB].canch,JOINT_INFO[LEB].bno, SW_MODE_COMPLEMENTARY);

    MCJointGainOverride(JOINT_INFO[LEB].canch, JOINT_INFO[LEB].bno, JOINT_INFO[LEB].mch, 0,1500); //--LSR



    MCenableFrictionCompensation(JOINT_INFO[REB].canch, JOINT_INFO[REB].bno, JOINT_INFO[REB].mch, DISABLE);
    MCBoardSetSwitchingMode(JOINT_INFO[REB].canch,JOINT_INFO[REB].bno, SW_MODE_COMPLEMENTARY);

    MCJointGainOverride(JOINT_INFO[REB].canch, JOINT_INFO[REB].bno, JOINT_INFO[REB].mch, 0,1500); //--RSR

    usleep(5000);
}
