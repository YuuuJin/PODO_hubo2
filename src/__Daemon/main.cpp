#include <iostream>
#include <sys/mman.h>
#include <unistd.h>
#include <signal.h>
#include <fcntl.h>

#include <QSettings>

#include "RBCAN.h"
#include "RBDataBase.h"
#include "../../share/Headers/LANData/GazeboLANData.h"

#include "RBProcessManager.h"

#include "RBMotorController.h"
#include "RBFTSensor.h"
#include "RBIMUSensor.h"
#include "RBSmartPower.h"


using namespace std;

QString     settingFile;


// Basic --------
int     IS_WORKING = false;
int     IS_CHILD = false;
int     IS_CAN_OK = false;

pRBCORE_SHM             sharedData;
pRBCAN                  canHandler;
RBProcessManager        *pmHandler;

float   EXF_R_Modifier[5] = {0.0, };
float   EXF_L_Modifier[5] = {0.0, };


// Initialize --------
int     RBCore_Initialize();
int     RBCore_SMInitialize();
int     RBCore_DBInitialize();
int     RBCore_CANInitialize();
int     RBCore_ThreadInitialize();
int     RBCore_PMInitialize();
int     RBCore_FingerInitialize();
int     RBCore_Termination();
void    RBCore_RTThreadCon(void *);
RT_TASK rtTaskCon;

// Database --------
DB_GENERAL      RBDataBase::_DB_GENERAL;
DB_MC           RBDataBase::_DB_MC[MAX_MC];
DB_FT           RBDataBase::_DB_FT[MAX_FT];
DB_IMU          RBDataBase::_DB_IMU[MAX_IMU];
DB_SP           RBDataBase::_DB_SP[MAX_SP];
DB_AL           RBDataBase::_DB_AL[MAX_AL];

int     _VERSION;
int     _NO_OF_AL;
int     _NO_OF_COMM_CH;
int     _NO_OF_MC;
int     _NO_OF_FT;
int     _NO_OF_IMU;
int     _NO_OF_SP;


// Devices --------
RBMotorController   _DEV_MC[MAX_MC];
RBFTSensor          _DEV_FT[MAX_FT];
RBIMUSensor         _DEV_IMU[MAX_IMU];
RBSmartPower        _DEV_SP[MAX_SP];


int _CANOUT_ENABLED = false;
int _ENCODER_ENABLED = false;
int _SENSOR_ENABLED = false;
int _REFERENCE_ENABLED = false;


int _ALCommandCnt[MAX_AL] = {0,};
long _ThreadCnt = 0;

bool StatusReadFlag[MAX_MC] = {0,};
bool ErrorClearStart = false;


// Thread Functions
void    THREAD_ReadNewIMU();
void    THREAD_ReadSensor();
void    THREAD_ReadEncoder();
void    THREAD_ReadTemperature();
void    THREAD_ReadVoltage();
void    THREAD_ReadHomeError();

void    THREAD_RequestSensor();
void    THREAD_RequestEncoder();
void    THREAD_RequestTemperature();
void    THREAD_RequestVoltage();

// Command Functions
void    RBCMD_InitCheckDevice();
void    RBCMD_InitFindHome();
void    RBCMD_InitFindHomeHandHead();
void    RBCMD_InitFetOnOff();
void    RBCMD_InitControlOnOff();
void    RBCMD_InitSetFingerModifier();

void    RBCMD_AttrControlMode();

void    RBCMD_SensorEncoderReset();
void    RBCMD_SensorEncoderOnOff();
void    RBCMD_SensorSensorOnOff();
void    RBCMD_SensorFTNull();
void    RBCMD_SensorIMUNull();
void    RBCMD_SensorIMUOffsetSet();

void    RBCMD_MotionRefOnOff();
void    RBCMD_MotionMove();
void    RBCMD_MotionErrorClear();

void    RBCMD_CANEnableDisable();
const double D2R = 0.0174533;

// Debugging data
#define ROW_data_debug 6000
#define COL_data_debug 50
float   JW_Data_Debug[COL_data_debug][ROW_data_debug];
int sJW_Data_Debug_Index = 0;
FILE *fpDebug;

void JW_save()
{
    if(sJW_Data_Debug_Index < ROW_data_debug)
    {
        JW_Data_Debug[0][sJW_Data_Debug_Index] = _DEV_MC[MC_ID_CH_Pairs[LHR].id].Joints[MC_ID_CH_Pairs[LHR].ch].Reference;
        JW_Data_Debug[1][sJW_Data_Debug_Index] = _DEV_MC[MC_ID_CH_Pairs[LHR].id].Joints[MC_ID_CH_Pairs[LHR].ch].CurrentPosition;
        JW_Data_Debug[2][sJW_Data_Debug_Index] = _DEV_MC[MC_ID_CH_Pairs[LHR].id].Joints[MC_ID_CH_Pairs[LHR].ch].CurrentStatus.B[1];

        JW_Data_Debug[4][sJW_Data_Debug_Index] = _DEV_MC[MC_ID_CH_Pairs[LAP].id].Joints[MC_ID_CH_Pairs[LAP].ch].Reference;
        JW_Data_Debug[5][sJW_Data_Debug_Index] = _DEV_MC[MC_ID_CH_Pairs[LAP].id].Joints[MC_ID_CH_Pairs[LAP].ch].CurrentPosition;
        JW_Data_Debug[6][sJW_Data_Debug_Index] = _DEV_MC[MC_ID_CH_Pairs[LAP].id].Joints[MC_ID_CH_Pairs[LAP].ch].CurrentStatus.B[1];

        JW_Data_Debug[8][sJW_Data_Debug_Index] = _DEV_MC[MC_ID_CH_Pairs[RWP].id].Joints[MC_ID_CH_Pairs[RWP].ch].Reference;
        JW_Data_Debug[9][sJW_Data_Debug_Index] = _DEV_MC[MC_ID_CH_Pairs[RWP].id].Joints[MC_ID_CH_Pairs[RWP].ch].CurrentPosition;
        JW_Data_Debug[10][sJW_Data_Debug_Index] = _DEV_MC[MC_ID_CH_Pairs[RWP].id].Joints[MC_ID_CH_Pairs[RWP].ch].CurrentStatus.B[1];

        JW_Data_Debug[12][sJW_Data_Debug_Index] = _DEV_MC[MC_ID_CH_Pairs[LWP].id].Joints[MC_ID_CH_Pairs[LWP].ch].Reference;
        JW_Data_Debug[13][sJW_Data_Debug_Index] = _DEV_MC[MC_ID_CH_Pairs[LWP].id].Joints[MC_ID_CH_Pairs[LWP].ch].CurrentPosition;
        JW_Data_Debug[14][sJW_Data_Debug_Index] = _DEV_MC[MC_ID_CH_Pairs[LWP].id].Joints[MC_ID_CH_Pairs[LWP].ch].CurrentStatus.B[1];

        sJW_Data_Debug_Index++;
        if(sJW_Data_Debug_Index >= ROW_data_debug) sJW_Data_Debug_Index = 0;
    }
}



void CatchSignals(int _signal)
{
    switch(_signal)
    {
    case SIGHUP:     // shell termination
    case SIGINT:     // Ctrl-c
    case SIGTERM:    // "kill" from shell
    case SIGKILL:
    case SIGSEGV:
        canHandler->Finish();
        usleep(1000*1000);

        for(int i=1; i<_NO_OF_AL; i++){
            pmHandler->CloseAL(i);
        }
        IS_WORKING = false;
        break;
    }
    usleep(1000*1000);
}


int main(int argc, char *argv[])
{
    // Copyright
    cout << endl;
    cout << " \033[31m######################################################################\n";
    cout << " #                                                                    #\n";
    cout << " #  PODO Version 3.1 (New HUBO2)                                      #\n";
    cout << " #  Copyright 2017 Rainbow Robotics Co.                               #\n";
    cout << " #                                                                    #\n";
    cout << " #  Main developer: Jeongsoo Lim                                      #\n";
    cout << " #  E-mail: jslim@rainbow.re.kr                                       #\n";
    cout << " #                                                                    #\n";
    cout << " #  We touch the core!                                                #\n";
    cout << " #                                                                    #\n";
    cout << " ######################################################################\n";

    // Termination signal
    signal(SIGTERM, CatchSignals);       // "kill" from shell
    signal(SIGINT,  CatchSignals);       // Ctrl-c
    signal(SIGHUP,  CatchSignals);       // shell termination
    signal(SIGKILL, CatchSignals);
    signal(SIGSEGV, CatchSignals);

    // Block memory swapping
    mlockall(MCL_CURRENT|MCL_FUTURE);

    if(RBCore_Initialize() == false){
        FILE_LOG(logERROR) << "Core Initialization Failed..";
        return 0;
    }


    while(IS_WORKING){
        usleep(100*1000);

        switch(sharedData->COMMAND[RBCORE_PODO_NO].USER_COMMAND){
        case DAEMON_PROCESS_CREATE:
            FILE_LOG(logINFO) << "CMD: DAEMON_PROCESS_CREATE";
            pmHandler->OpenAL(sharedData->COMMAND[RBCORE_PODO_NO].USER_PARA_INT[0]);
            sharedData->COMMAND[RBCORE_PODO_NO].USER_COMMAND = NO_ACT;
            break;

        case DAEMON_PROCESS_KILL:
            FILE_LOG(logINFO) << "CMD: DAEMON_PROCESS_KILL";
            pmHandler->CloseAL(sharedData->COMMAND[RBCORE_PODO_NO].USER_PARA_INT[0]);
            sharedData->COMMAND[RBCORE_PODO_NO].USER_COMMAND = NO_ACT;
            break;

        case DAEMON_INIT_CHECK_DEVICE:
            FILE_LOG(logINFO) << "CMD: DAEMON_INIT_CHECK_DEVICE";
            if(IS_CAN_OK)   {RBCMD_InitCheckDevice();}
            else            {FILE_LOG(logWARNING) << "CAN device not set";}
            sharedData->COMMAND[RBCORE_PODO_NO].USER_COMMAND = NO_ACT;
            break;

        case DAEMON_INIT_FIND_HOME:
            FILE_LOG(logINFO) << "CMD: DAEMON_INIT_FIND_HOME";
            if(IS_CAN_OK)   {RBCMD_InitFindHome();}
            else            {FILE_LOG(logWARNING) << "CAN device not set";}
            sharedData->COMMAND[RBCORE_PODO_NO].USER_COMMAND = NO_ACT;
            break;

        case DAEMON_INIT_FIND_HOME_HAND_HEAD:
            FILE_LOG(logINFO) << "CMD: DAEMON_INIT_FIND_HOME_HAND_HEAD";
            if(IS_CAN_OK)   {RBCMD_InitFindHomeHandHead();}
            else            {FILE_LOG(logWARNING) << "CAN device not set";}
            sharedData->COMMAND[RBCORE_PODO_NO].USER_COMMAND = NO_ACT;
            break;

        case DAEMON_INIT_FET_ONOFF:
            FILE_LOG(logINFO) << "CMD: DAEMON_INIT_FET_ONOFF";
            if(IS_CAN_OK)   {RBCMD_InitFetOnOff();}
            else            {FILE_LOG(logWARNING) << "CAN device not set";}
            sharedData->COMMAND[RBCORE_PODO_NO].USER_COMMAND = NO_ACT;
            break;

        case DAEMON_INIT_CONTROL_ONOFF:
            FILE_LOG(logINFO) << "CMD: DAEMON_INIT_CONTROL_ONOFF";
            if(IS_CAN_OK)   {RBCMD_InitControlOnOff();}
            else            {FILE_LOG(logWARNING) << "CAN device not set";}
            sharedData->COMMAND[RBCORE_PODO_NO].USER_COMMAND = NO_ACT;
            break;

        case DAEMON_INIT_SET_FINGER_MODIFIER:
            FILE_LOG(logINFO) << "CMD: DAEMON_INIT_SET_FINGER_MODIFIER";

            RBCMD_InitSetFingerModifier();

            sharedData->COMMAND[RBCORE_PODO_NO].USER_COMMAND = NO_ACT;
            break;

        case DAEMON_ATTR_CONTROL_MODE:
            FILE_LOG(logINFO) << "CMD: DAEMON_ATTR_CONTROL_MODE";
            if(IS_CAN_OK)   {RBCMD_AttrControlMode();}
            else            {FILE_LOG(logWARNING) << "CAN device not set";}
            sharedData->COMMAND[RBCORE_PODO_NO].USER_COMMAND = NO_ACT;
            break;

        case DAEMON_SENSOR_ENCODER_RESET:
            FILE_LOG(logINFO) << "CMD: DAEMON_SENSOR_ENCODER_RESET";
            if(IS_CAN_OK)   {RBCMD_SensorEncoderReset();}
            else            {FILE_LOG(logWARNING) << "CAN device not set";}
            sharedData->COMMAND[RBCORE_PODO_NO].USER_COMMAND = NO_ACT;
            break;

        case DAEMON_SENSOR_ENCODER_ONOFF:
            FILE_LOG(logINFO) << "CMD: DAEMON_SENSOR_ENCODER_ONOFF";
            if(IS_CAN_OK)   {RBCMD_SensorEncoderOnOff();}
            else            {FILE_LOG(logWARNING) << "CAN device not set";}
            sharedData->COMMAND[RBCORE_PODO_NO].USER_COMMAND = NO_ACT;
            break;

        case DAEMON_SENSOR_SENSOR_ONOFF:
            FILE_LOG(logINFO) << "CMD: DAEMON_SENSOR_SENSOR_ONOFF";
            if(IS_CAN_OK)   {RBCMD_SensorSensorOnOff();}
            else            {FILE_LOG(logWARNING) << "CAN device not set";}
            sharedData->COMMAND[RBCORE_PODO_NO].USER_COMMAND = NO_ACT;
            break;

        case DAEMON_SENSOR_FT_NULL:
            FILE_LOG(logINFO) << "CMD: DAEMON_SENSOR_FT_NULL";
            if(IS_CAN_OK)   {RBCMD_SensorFTNull();}
            else            {FILE_LOG(logWARNING) << "CAN device not set";}
            sharedData->COMMAND[RBCORE_PODO_NO].USER_COMMAND = NO_ACT;
            break;

        case DAEMON_SENSOR_IMU_NULL:
            //   FILE_LOG(logINFO) << "CMD: DAEMON_SENSOR_IMU_NULL";
           {
               char imu_cmd_type = sharedData->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[0];
               if(imu_cmd_type == 1){
                   RBCAN_MB mb;
                   mb.channel = 0;
                   mb.id = 150;
                   mb.data[0] = 1;
                   mb.data[1] = 1;
                   mb.dlc = 2;
                   canHandler->RBCAN_WriteData(mb);
                   FILE_LOG(logINFO) << "CMD: DAEMON_SENSOR_IMU_ENABLE";
               }else if(imu_cmd_type == 2){
                   RBCAN_MB mb;
                   mb.channel = 0;
                   mb.id = 150;
                   mb.data[0] = 3;
                   mb.dlc = 1;
                   canHandler->RBCAN_WriteData(mb);
                   FILE_LOG(logINFO) << "CMD: DAEMON_SENSOR_IMU_NULLING";
               }else if(imu_cmd_type == 3){
                   RBCAN_MB mb;
                   mb.channel = 0;
                   mb.id = 150;
                   mb.data[0] = 2;
                   mb.data[1] = 0;
                   mb.dlc = 2;
                   canHandler->RBCAN_WriteData(mb);
                   FILE_LOG(logINFO) << "CMD: DAEMON_SENSOR_IMU_RESET";
               }else if(imu_cmd_type == 4){
                   RBCAN_MB mb;
                   mb.channel = 0;
                   mb.id = 150;
                   mb.data[0] = 2;
                   mb.data[1] = 1;
                   mb.dlc = 2;
                   canHandler->RBCAN_WriteData(mb);
                   FILE_LOG(logINFO) << "CMD: DAEMON_SENSOR_IMU_RESET_ZERO";
               }

           }
            sharedData->COMMAND[RBCORE_PODO_NO].USER_COMMAND = NO_ACT;
               break;
//            FILE_LOG(logINFO) << "CMD: DAEMON_SENSOR_IMU_NULL";
//            if(IS_CAN_OK)   {RBCMD_SensorIMUNull();}
//            else            {FILE_LOG(logWARNING) << "CAN device not set";}
//            sharedData->COMMAND[RBCORE_PODO_NO].USER_COMMAND = NO_ACT;
//            break;

        case DAEMON_SENSOR_IMU_OFFSET_SET:
            FILE_LOG(logINFO) << "CMD: DAEMON_SENSOR_IMU_OFFSET_SET";
            if(IS_CAN_OK)   {RBCMD_SensorIMUOffsetSet();}
            else            {FILE_LOG(logWARNING) << "CAN device not set";}
            sharedData->COMMAND[RBCORE_PODO_NO].USER_COMMAND = NO_ACT;
            break;

        case DAEMON_MOTION_REF_ONOFF:
            FILE_LOG(logINFO) << "CMD: DAEMON_MOTION_REF_ONOFF";
            RBCMD_MotionRefOnOff();

            sharedData->COMMAND[RBCORE_PODO_NO].USER_COMMAND = NO_ACT;
            break;

        case DAEMON_MOTION_MOVE:
            FILE_LOG(logINFO) << "CMD: DAEMON_MOTION_MOVE";
            RBCMD_MotionMove();

            sharedData->COMMAND[RBCORE_PODO_NO].USER_COMMAND = NO_ACT;
            break;

        case DAEMON_MOTION_ERROR_CLEAR:
            FILE_LOG(logINFO) << "CMD: DAEMON_MOTION_ERROR_CLEAR";
            if(IS_CAN_OK)   {RBCMD_MotionErrorClear();}
            else            {FILE_LOG(logWARNING) << "CAN device not set";}
            sharedData->COMMAND[RBCORE_PODO_NO].USER_COMMAND = NO_ACT;
            break;

        case DAEMON_CAN_ENABLE_DISABLE:
            FILE_LOG(logINFO) << "CMD: DAEMON_CAN_ENABLE_DISABLE";
            if(IS_CAN_OK)   {RBCMD_CANEnableDisable();}
            else            {FILE_LOG(logWARNING) << "CAN device not set";}
            sharedData->COMMAND[RBCORE_PODO_NO].USER_COMMAND = NO_ACT;
            break;

        case 888:
            FILE_LOG(logINFO) << "Command: SAVE_DEBUG_DATA";
            fpDebug = fopen("dataDaemon.txt","w");
            for(int i=0;i<ROW_data_debug;i++){
                for(int j=0;j<COL_data_debug;j++)fprintf(fpDebug,"%g\t", JW_Data_Debug[j][i]);
                fprintf(fpDebug,"\n");
            }
            fclose(fpDebug);
            sharedData->COMMAND[RBCORE_PODO_NO].USER_COMMAND = NO_ACT;
            break;
        }
    }


    RBCore_Termination();

    usleep(1000*1000);
    return 0;
}



void RBCore_RTThreadCon(void *)
{
    rt_task_set_periodic(NULL, TM_NOW, (RT_TIMER_PERIOD_MS)*1000000);

    RTIME   th_start, th_stop;
    int     waitCnt, waitOK;
    double  timeGap = 0.0;
    double  finalJointRef[MOTOR_2CH];

    while(IS_WORKING)
    {
        rt_task_wait_period(NULL);
        JW_save();
        // Read Sensor & Encoder =====================================
        if(canHandler->IsWorking()){
            THREAD_ReadNewIMU();
            THREAD_ReadSensor();
            THREAD_ReadEncoder();
            //THREAD_ReadTemperature();
            //THREAD_ReadVoltage();
            THREAD_ReadHomeError();

            sharedData->CAN_Enabled = _CANOUT_ENABLED;
            sharedData->ENC_Enabled = _ENCODER_ENABLED;
            sharedData->SEN_Enabled = _SENSOR_ENABLED;
            sharedData->REF_Enabled = _REFERENCE_ENABLED;
        }
        // ===========================================================

        // Change Flag ===============================================
        for(int i=0; i<_NO_OF_MC; i++){
            for(int j=0; j<_DEV_MC[i].MOTOR_CHANNEL; j++){
                sharedData->ACK_SIGNAL[i][j] = false;
            }
        }
        for(int i=0; i<_NO_OF_AL; i++){
            sharedData->SYNC_SIGNAL[i] = true;
        }
        // ===========================================================

        // Wait Reference ============================================
        th_start = rt_timer_read();
        waitCnt = 0;
        waitOK = false;
        while(1){
            // check the all WriteDoneFlag are enabled
            int notRead = 0;
            for(int i=0; i<_NO_OF_MC; i++){
                for(int j=0; j<_DEV_MC[i].MOTOR_CHANNEL; j++){
                    if(sharedData->ACK_SIGNAL[i][j] == false && sharedData->MotionOwner[i][j] != RBCORE_PODO_NO){
                        notRead++;
                    }
                }
            }
            if(notRead == 0){
                th_stop = rt_timer_read();
                timeGap = (double)(th_stop - th_start) / (double)1000000.0;
                waitOK = true;
                break;
            }

            if(waitCnt%500 == 0){
                th_stop = rt_timer_read();
                timeGap = (double)(th_stop - th_start) / (double)1000000.0;
                if(timeGap > 2.5){
                    waitOK = false;
                    FILE_LOG(logWARNING) << "Over 2.5msec";
                    break;
                }
            }
            waitCnt++;
            usleep(2);
        }
        // ===========================================================


        // Write CAN Reference =======================================
        for(int i=0; i<_NO_OF_MC; i++){
            for(int j=0; j<_DEV_MC[i].MOTOR_CHANNEL; j++){
                int motionOwner = sharedData->MotionOwner[i][j];

                _DEV_MC[i].RBJoint_MoveJoint(j);
                sharedData->JointReference[RBCORE_PODO_NO][i][j] = _DEV_MC[i].MoveJoints[j].RefAngleCurrent;

                finalJointRef[j] = sharedData->JointReference[motionOwner][i][j];
                _DEV_MC[i].Joints[j].Reference = finalJointRef[j];
                sharedData->ENCODER[i][j].CurrentReference = finalJointRef[j];
            }
            if(canHandler->IsWorking() && _CANOUT_ENABLED){ // CANOUT
                if(_DEV_MC[i].BOARD_ID < 32){
                    // New Firmware
                    _DEV_MC[i].RBBoard_SendReference();
                }else{
                    // Old Firmware
                    _DEV_MC[i].RBBoard_SendReference_old();
                }
            }
        }
        // ===========================================================


        // Manual CAN ================================================
        if(canHandler->IsWorking()){
            for(int i=0; i<MAX_MANUAL_CAN; i++){
                if(sharedData->ManualCAN[i].status == MANUALCAN_NEW){
                    sharedData->ManualCAN[i].status = MANUALCAN_WRITING;
                    RBCAN_MB mb;
                    mb.channel = sharedData->ManualCAN[i].channel;
                    mb.id = sharedData->ManualCAN[i].id;
                    mb.dlc = sharedData->ManualCAN[i].dlc;
                    for(int j=0; j<mb.dlc; j++)
                        mb.data[j] = sharedData->ManualCAN[i].data[j];

                    if(mb.data[0] != 0x6F || _CANOUT_ENABLED)  // CANOUT
                        canHandler->RBCAN_WriteData(mb);

                    //FILE_LOG(logINFO) << "MANUAL_CAN: " << mb.id << " == " << (int)mb.data[0] << ", " << (int)mb.data[1] << ", " << (int)mb.data[2] << ", " << (int)mb.data[3] << ", " << (int)mb.data[4] << ", " << (int)mb.data[5] << ", " << (int)mb.data[6] << ", " << (int)mb.data[7];
                    sharedData->ManualCAN[i].status = MANUALCAN_EMPTY;
                }
            }
        }
        // ===========================================================


        _ThreadCnt++;
        // Request Sensor & Encoder ==================================
        if(canHandler->IsWorking()){
            THREAD_RequestSensor();
            THREAD_RequestEncoder();
            THREAD_RequestTemperature();
            //THREAD_RequestVoltage();
        }
        // ===========================================================
    }
}

void RBCMD_InitCheckDevice(){
    const int MAX_CAN = 2;
//    void *tempHandler[MAX_CAN];
//    for(int i=0; i<MAX_CAN; i++){
//        tempHandler[i] = canHandler->canHandler[i];
//    }

//    int fail, ok;
//    int chok[MAX_CAN] = {0,};
//    for(int canch=0; canch<MAX_CAN; canch++){
//        for(int trynum=0; trynum<MAX_CAN; trynum++){
//            fail = ok = 0;

//            for(int i=0; i<_NO_OF_MC; i++){
//                if(_DEV_MC[i].CAN_CHANNEL == canch){
//                    if(_DEV_MC[i].RBBoard_CANCheck(RT_TIMER_PERIOD_MS)){
//                        ok++;
//                    }else{
//                        fail++;
//                    }
//                }
//            }

//            for(int i=0; i<_NO_OF_FT; i++){
//                if(_DEV_FT[i].CAN_CHANNEL == canch){
//                    if(_DEV_FT[i].RBBoard_CANCheck(RT_TIMER_PERIOD_MS)){
//                        ok++;
//                    }else{
//                        fail++;
//                    }
//                }
//            }

//            for(int i=0; i<_NO_OF_IMU; i++){
//                if(_DEV_IMU[i].CAN_CHANNEL == canch){
//                    if(_DEV_IMU[i].RBBoard_CANCheck(RT_TIMER_PERIOD_MS)){
//                        ok++;
//                    }else{
//                        fail++;
//                    }
//                }
//            }

//            for(int i=0; i<_NO_OF_SP; i++){
//                if(_DEV_SP[i].CAN_CHANNEL == canch){
//                    if(_DEV_SP[i].RBBoard_CANCheck(RT_TIMER_PERIOD_MS)){
//                        ok++;
//                    }else{
//                        fail++;
//                    }
//                }
//            }

//            if(ok > fail){
//                if(trynum != 0)
//                    FILE_LOG(logWARNING) << "Recommand to move CAN channel " << canch << "to " << (canch+trynum)%MAX_CAN;
//                else
//                    FILE_LOG(logSUCCESS) << "Channel " << canch << " is on the right port";
//                chok[canch] = 1;
//                break;
//            }else{
//                FILE_LOG(logINFO) << "Try again after changing the channel";
//                canHandler->canHandler[canch] = tempHandler[(canch+trynum+1)%MAX_CAN];
//            }
//        }
//    }


    for(int i=0; i<_NO_OF_MC; i++){
        _DEV_MC[i].RBBoard_CANCheck(RT_TIMER_PERIOD_MS);
        for(int j=0; j<_DEV_MC[i].TOTAL_CHANNEL; j++){
            sharedData->ENCODER[i][j].BoardConnection = _DEV_MC[i].ConnectionStatus;
        }
    }
    for(int i=0; i<_NO_OF_FT; i++){
        _DEV_FT[i].RBBoard_CANCheck(RT_TIMER_PERIOD_MS);
        sharedData->FT[i].BoardConnection = _DEV_FT[i].ConnectionStatus;
    }
    for(int i=0; i<_NO_OF_IMU; i++){
        _DEV_IMU[i].RBBoard_CANCheck(RT_TIMER_PERIOD_MS);
        sharedData->IMU[i].BoardConnection = _DEV_IMU[i].ConnectionStatus;
    }
    for(int i=0; i<_NO_OF_SP; i++){
        _DEV_SP[i].RBBoard_CANCheck(RT_TIMER_PERIOD_MS);
        sharedData->SP[i].BoardConnection = _DEV_SP[i].ConnectionStatus;
    }

//    _DEV_MC[MC_GetID(RF1)].RBBoard_SetControlMode(0x01);
//    _DEV_MC[MC_GetID(LF1)].RBBoard_SetControlMode(0x01);
}


void RBCMD_InitFindHome(){
    int id = sharedData->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[0];
    int ch = sharedData->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[1];

    int i;
    if(id == -2){

        i = 8;
        for(int j=0; j<_DEV_MC[i].MOTOR_CHANNEL; j++){
            _DEV_MC[i].RBJoint_ResetEncoder(j+1);
            sharedData->MotionOwner[i][j] = RBCORE_PODO_NO;

            for(int k=0; k<_NO_OF_AL; k++){
                sharedData->JointReference[k][i][j] = 0.0;
            }
            _DEV_MC[i].Joints[j].Reference = 0.0;
            _DEV_MC[i].MoveJoints[j].RefAngleCurrent = 0.0;

            _DEV_MC[i].RBJoint_EnableFETDriver(j+1, 1);
            usleep(100);
            _DEV_MC[i].RBJoint_FindHome(j+1);
        }

        i = 9;
        for(int j=0; j<_DEV_MC[i].MOTOR_CHANNEL; j++){
            _DEV_MC[i].RBJoint_ResetEncoder(j+1);
            sharedData->MotionOwner[i][j] = RBCORE_PODO_NO;

            for(int k=0; k<_NO_OF_AL; k++){
                sharedData->JointReference[k][i][j] = 0.0;
            }
            _DEV_MC[i].Joints[j].Reference = 0.0;
            _DEV_MC[i].MoveJoints[j].RefAngleCurrent = 0.0;

            _DEV_MC[i].RBJoint_EnableFETDriver(j+1, 1);
            usleep(100);
            _DEV_MC[i].RBJoint_FindHome(j+1);
        }

        i = 10;
        for(int j=0; j<_DEV_MC[i].MOTOR_CHANNEL; j++){
            _DEV_MC[i].RBJoint_ResetEncoder(j+1);
            sharedData->MotionOwner[i][j] = RBCORE_PODO_NO;

            for(int k=0; k<_NO_OF_AL; k++){
                sharedData->JointReference[k][i][j] = 0.0;
            }
            _DEV_MC[i].Joints[j].Reference = 0.0;
            _DEV_MC[i].MoveJoints[j].RefAngleCurrent = 0.0;

            _DEV_MC[i].RBJoint_EnableFETDriver(j+1, 1);
            usleep(100);
            _DEV_MC[i].RBJoint_FindHome(j+1);
        }
        i = 11;
        for(int j=0; j<_DEV_MC[i].MOTOR_CHANNEL; j++){
            _DEV_MC[i].RBJoint_ResetEncoder(j+1);
            sharedData->MotionOwner[i][j] = RBCORE_PODO_NO;

            for(int k=0; k<_NO_OF_AL; k++){
                sharedData->JointReference[k][i][j] = 0.0;
            }
            _DEV_MC[i].Joints[j].Reference = 0.0;
            _DEV_MC[i].MoveJoints[j].RefAngleCurrent = 0.0;

            _DEV_MC[i].RBJoint_EnableFETDriver(j+1, 1);
            usleep(100);
            _DEV_MC[i].RBJoint_FindHome(j+1);
        }

        i = 12;
        for(int j=0; j<_DEV_MC[i].MOTOR_CHANNEL; j++){
            _DEV_MC[i].RBJoint_ResetEncoder(j+1);
            sharedData->MotionOwner[i][j] = RBCORE_PODO_NO;

            for(int k=0; k<_NO_OF_AL; k++){
                sharedData->JointReference[k][i][j] = 0.0;
            }
            _DEV_MC[i].Joints[j].Reference = 0.0;
            _DEV_MC[i].MoveJoints[j].RefAngleCurrent = 0.0;

            _DEV_MC[i].RBJoint_EnableFETDriver(j+1, 1);
            usleep(100);
            _DEV_MC[i].RBJoint_FindHome(j+1);
        }


        i = 13;

        _DEV_MC[i].RBJoint_ResetEncoder(0x0F);
        for(int j=0; j<_DEV_MC[j].MOTOR_CHANNEL; j++){
            sharedData->MotionOwner[i][j] = RBCORE_PODO_NO;
            for(int k=0; k<_NO_OF_AL; k++){
                sharedData->JointReference[k][i][j] = 0.0;
            }
            _DEV_MC[i].Joints[j].Reference = 0.0;
            _DEV_MC[i].MoveJoints[j].RefAngleCurrent = 0.0;
        }
        _DEV_MC[i].RBJoint_FindHome(0x0F);
        i = 14;

        _DEV_MC[i].RBJoint_ResetEncoder(0x0F);
        for(int j=0; j<_DEV_MC[j].MOTOR_CHANNEL; j++){
            sharedData->MotionOwner[i][j] = RBCORE_PODO_NO;
            for(int k=0; k<_NO_OF_AL; k++){
                sharedData->JointReference[k][i][j] = 0.0;
            }
            _DEV_MC[i].Joints[j].Reference = 0.0;
            _DEV_MC[i].MoveJoints[j].RefAngleCurrent = 0.0;
        }
        _DEV_MC[i].RBJoint_FindHome(0x0F);

        i = 15;

        _DEV_MC[i].RBJoint_ResetEncoder(0x0F);
        for(int j=0; j<_DEV_MC[j].MOTOR_CHANNEL; j++){
            sharedData->MotionOwner[i][j] = RBCORE_PODO_NO;
            for(int k=0; k<_NO_OF_AL; k++){
                sharedData->JointReference[k][i][j] = 0.0;
            }
            _DEV_MC[i].Joints[j].Reference = 0.0;
            _DEV_MC[i].MoveJoints[j].RefAngleCurrent = 0.0;
        }
        _DEV_MC[i].RBJoint_FindHome(0x0F);

        i = 16;

        _DEV_MC[i].RBJoint_ResetEncoder(0x0F);
        for(int j=0; j<_DEV_MC[j].MOTOR_CHANNEL; j++){
            sharedData->MotionOwner[i][j] = RBCORE_PODO_NO;
            for(int k=0; k<_NO_OF_AL; k++){
                sharedData->JointReference[k][i][j] = 0.0;
            }
            _DEV_MC[i].Joints[j].Reference = 0.0;
            _DEV_MC[i].MoveJoints[j].RefAngleCurrent = 0.0;
        }
        _DEV_MC[i].RBJoint_FindHome(0x0F);

        i = 17;

        _DEV_MC[i].RBJoint_ResetEncoder(0x0F);
        for(int j=0; j<_DEV_MC[j].MOTOR_CHANNEL; j++){
            sharedData->MotionOwner[i][j] = RBCORE_PODO_NO;
            for(int k=0; k<_NO_OF_AL; k++){
                sharedData->JointReference[k][i][j] = 0.0;
            }
            _DEV_MC[i].Joints[j].Reference = 0.0;
            _DEV_MC[i].MoveJoints[j].RefAngleCurrent = 0.0;
        }
        _DEV_MC[i].RBJoint_FindHome(0x0F);
    }



/*
        for(int i=8; i<_NO_OF_MC; i++){
            if(_DEV_MC[i].BOARD_ID < 32){
                for(int j=0; j<_DEV_MC[i].MOTOR_CHANNEL; j++){
                    _DEV_MC[i].RBJoint_ResetEncoder(j+1);
                    sharedData->MotionOwner[i][j] = RBCORE_PODO_NO;

                    for(int k=0; k<_NO_OF_AL; k++){
                        sharedData->JointReference[k][i][j] = 0.0;
                    }
                    _DEV_MC[i].Joints[j].Reference = 0.0;
                    _DEV_MC[i].MoveJoints[j].RefAngleCurrent = 0.0;

                    _DEV_MC[i].RBJoint_EnableFETDriver(j+1, 1);
                    usleep(100);
                    _DEV_MC[i].RBJoint_FindHome(j+1);
                }
            }else{
                _DEV_MC[i].RBJoint_ResetEncoder(0x0F);
                for(int j=0; j<_DEV_MC[j].MOTOR_CHANNEL; j++){
                    sharedData->MotionOwner[i][j] = RBCORE_PODO_NO;
                    for(int k=0; k<_NO_OF_AL; k++){
                        sharedData->JointReference[k][i][j] = 0.0;
                    }
                    _DEV_MC[i].Joints[j].Reference = 0.0;
                    _DEV_MC[i].MoveJoints[j].RefAngleCurrent = 0.0;
                }
                _DEV_MC[i].RBJoint_FindHome(0x0F);
            }
        }
    }

*/
    if(id == -1){ // All
        for(int i=0; i<_NO_OF_MC; i++){
            if(_DEV_MC[i].BOARD_ID < 32){
                for(int j=0; j<_DEV_MC[i].MOTOR_CHANNEL; j++){
                    _DEV_MC[i].RBJoint_ResetEncoder(j+1);
                    sharedData->MotionOwner[i][j] = RBCORE_PODO_NO;

                    for(int k=0; k<_NO_OF_AL; k++){
                        sharedData->JointReference[k][i][j] = 0.0;
                    }
                    _DEV_MC[i].Joints[j].Reference = 0.0;
                    _DEV_MC[i].MoveJoints[j].RefAngleCurrent = 0.0;

                    _DEV_MC[i].RBJoint_EnableFETDriver(j+1, 1);
                    usleep(100);
                    _DEV_MC[i].RBJoint_FindHome(j+1);
                }
            }else{
                _DEV_MC[i].RBJoint_ResetEncoder(0x0F);
                for(int j=0; j<_DEV_MC[j].MOTOR_CHANNEL; j++){
                    sharedData->MotionOwner[i][j] = RBCORE_PODO_NO;
                    for(int k=0; k<_NO_OF_AL; k++){
                        sharedData->JointReference[k][i][j] = 0.0;
                    }
                    _DEV_MC[i].Joints[j].Reference = 0.0;
                    _DEV_MC[i].MoveJoints[j].RefAngleCurrent = 0.0;
                }
                _DEV_MC[i].RBJoint_FindHome(0x0F);
            }
        }
    }else{  // Each
        _DEV_MC[id].RBJoint_ResetEncoder(ch+1);
        sharedData->MotionOwner[id][ch] = RBCORE_PODO_NO;
        for(int k=0; k<_NO_OF_AL; k++){
            sharedData->JointReference[k][id][ch] = 0.0;
        }
        _DEV_MC[id].Joints[ch].Reference = 0.0;
        _DEV_MC[id].MoveJoints[ch].RefAngleCurrent = 0.0;

        if(_DEV_MC[id].BOARD_ID < 32){
            _DEV_MC[id].RBJoint_EnableFETDriver(ch+1, 1);
        }
        usleep(100);
        _DEV_MC[id].RBJoint_FindHome(ch+1);
    }
}

void RBCMD_InitFindHomeHandHead(){
    int right = sharedData->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[0];
    int left  = sharedData->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[1];
    int head  = sharedData->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[2];

    if(right == 1){
        //_DEV_MC[MC_GetID(RF1)].RBBoard_SetControlMode(0x01);
        //usleep(50*1000);
        //_DEV_MC[MC_GetID(RF1)].RBJoint_ResetEncoder(0x0F);
        usleep(50*1000);
        _DEV_MC[MC_GetID(RF1)].RBJoint_FindHome(0x0F);
        int id = MC_GetID(RF1);
        for(int j=0; j<5; j++){
            for(int k=0; k<_NO_OF_AL; k++){
                sharedData->JointReference[k][id][j] = 0.0;
            }
            _DEV_MC[id].Joints[j].Reference = 0.0;
            _DEV_MC[id].MoveJoints[j].RefAngleCurrent = 0.0;
        }
    }
    if(left == 1){
        //_DEV_MC[MC_GetID(LF1)].RBBoard_SetControlMode(0x01);
        //usleep(50*1000);
        //_DEV_MC[MC_GetID(LF1)].RBJoint_ResetEncoder(0x0F);
        usleep(50*1000);
        _DEV_MC[MC_GetID(LF1)].RBJoint_FindHome(0x0F);
        int id = MC_GetID(LF1);
        for(int j=0; j<5; j++){
            for(int k=0; k<_NO_OF_AL; k++){
                sharedData->JointReference[k][id][j] = 0.0;
            }
            _DEV_MC[id].Joints[j].Reference = 0.0;
            _DEV_MC[id].MoveJoints[j].RefAngleCurrent = 0.0;
        }
    }
    if(head == 1){
//        _DEV_MC[MC_GetID(NKY)].RBJoint_ResetEncoder(0x0F);
//        usleep(50*1000);
        _DEV_MC[MC_GetID(NKY)].RBJoint_FindHome(0x0F);
        int id = MC_GetID(NKY);
        for(int j=0; j<3; j++){
            for(int k=0; k<_NO_OF_AL; k++){
                sharedData->JointReference[k][id][j] = 0.0;
            }
            _DEV_MC[id].Joints[j].Reference = 0.0;
            _DEV_MC[id].MoveJoints[j].RefAngleCurrent = 0.0;
        }
    }
}

void RBCMD_InitFetOnOff(){
    int id = sharedData->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[0];
    int ch = sharedData->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[1];
    int onoff = sharedData->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[2];   // 0->off
    if(onoff != 0) onoff = 1;

    if(id == -1){ // All
        for(int i=0; i<_NO_OF_MC; i++){
            for(int j=0; j<_DEV_MC[i].MOTOR_CHANNEL; j++){
                _DEV_MC[i].RBJoint_EnableFETDriver(j+1, onoff);
            }
        }
    }else{  // Each
        _DEV_MC[id].RBJoint_EnableFETDriver(ch+1, onoff);
    }
}

void RBCMD_InitControlOnOff(){
    int id = sharedData->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[0];
    int ch = sharedData->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[1];
    int onoff = sharedData->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[2];   // 0->off
    if(onoff != 0) onoff = 1;

    if(id == -1){ // All
        for(int i=0; i<_NO_OF_MC; i++){
            for(int j=0; j<_DEV_MC[i].MOTOR_CHANNEL; j++){
                _DEV_MC[i].RBJoint_EnableFeedbackControl(j+1, onoff);
            }
        }
    }else{  // Each
        _DEV_MC[id].RBJoint_EnableFeedbackControl(ch+1, onoff);
    }
}

void RBCMD_InitSetFingerModifier(){
    QSettings settings(settingFile, QSettings::NativeFormat);
    if(sharedData->COMMAND[RBCORE_PODO_NO].USER_PARA_INT[0] == 1){
        // just read current value
    }else{
        // set with new values (only available if the Daemon is turned on for extra finger
        for(int i=0; i<5; i++){
            settings.setValue(QString().sprintf("exfr%d",i), sharedData->COMMAND[RBCORE_PODO_NO].USER_PARA_FLOAT[i]);
            settings.setValue(QString().sprintf("exfl%d",i), sharedData->COMMAND[RBCORE_PODO_NO].USER_PARA_FLOAT[i+5]);
        }
    }

    EXF_R_Modifier[0] = settings.value("exfr0", "").toFloat();
    EXF_R_Modifier[1] = settings.value("exfr1", "").toFloat();
    EXF_R_Modifier[2] = settings.value("exfr2", "").toFloat();
    EXF_R_Modifier[3] = settings.value("exfr3", "").toFloat();
    EXF_R_Modifier[4] = settings.value("exfr4", "").toFloat();

    EXF_L_Modifier[0] = settings.value("exfl0", "").toFloat();
    EXF_L_Modifier[1] = settings.value("exfl1", "").toFloat();
    EXF_L_Modifier[2] = settings.value("exfl2", "").toFloat();
    EXF_L_Modifier[3] = settings.value("exfl3", "").toFloat();
    EXF_L_Modifier[4] = settings.value("exfl4", "").toFloat();

    for(int i=0; i<5; i++){
        sharedData->EXF_R_Modifier[i] = EXF_R_Modifier[i];
        sharedData->EXF_L_Modifier[i] = EXF_L_Modifier[i];
    }
}

void RBCMD_AttrControlMode(){
    int id = sharedData->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[0];
    int ch = sharedData->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[1];
    int mode = sharedData->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[2];   // 0->pos 2->cur 3->pwm 4->pos+pwm

    if(id == -1){ // All
        for(int i=0; i<_NO_OF_MC; i++){
            for(int j=0; j<_DEV_MC[i].MOTOR_CHANNEL; j++){
                _DEV_MC[i].Joints[j].ControlMode = mode;
            }
        }
    }else{  // Each
        _DEV_MC[id].Joints[ch].ControlMode = mode;
    }
}

void RBCMD_SensorEncoderReset(){
    int id = sharedData->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[0];
    int ch = sharedData->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[1];

    if(id == -1){ // All
        for(int i=0; i<_NO_OF_MC; i++){
            for(int j=0; j<_DEV_MC[i].MOTOR_CHANNEL; j++){
                _DEV_MC[i].RBJoint_ResetEncoder(j+1);
                sharedData->MotionOwner[i][j] = RBCORE_PODO_NO;

                for(int k=0; k<_NO_OF_AL; k++){
                    sharedData->JointReference[k][i][j] = 0.0;
                }
                _DEV_MC[i].Joints[j].Reference = 0.0;
                _DEV_MC[i].MoveJoints[j].RefAngleCurrent = 0.0;
            }
        }
    }else{  // Each
        _DEV_MC[id].RBJoint_ResetEncoder(ch+1);
        sharedData->MotionOwner[id][ch] = RBCORE_PODO_NO;
        for(int k=0; k<_NO_OF_AL; k++){
            sharedData->JointReference[k][id][ch] = 0.0;
        }
        _DEV_MC[id].Joints[ch].Reference = 0.0;
        _DEV_MC[id].MoveJoints[ch].RefAngleCurrent = 0.0;
    }
}

void RBCMD_SensorEncoderOnOff(){
    int onoff = sharedData->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[0];
    if(onoff != 0) onoff = 1;

    if(onoff == 1){ //on
        for(int i=0; i<_NO_OF_MC; i++){
            _DEV_MC[i].RBBoard_RequestEncoder(1);       //continuous
            _ENCODER_ENABLED = true;
        }
    }else{  //off
        for(int i=0; i<_NO_OF_MC; i++){
            _DEV_MC[i].RBBoard_RequestEncoder(0);       //oneshot
            _ENCODER_ENABLED = false;
        }
    }
}

void RBCMD_SensorSensorOnOff(){
    int onoff = sharedData->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[0];
    if(onoff != 0) onoff = 1;

    if(onoff == 1){ //on
        _SENSOR_ENABLED = true;
    }else{  //off
        _SENSOR_ENABLED = false;
    }
}

void RBCMD_SensorFTNull(){
    int id = sharedData->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[0];

    if(id == -1){   //All
        for(int i=0; i<_NO_OF_FT; i++){
            _DEV_FT[i].RBFT_Nulling(0);
            _DEV_FT[i].RBFT_Nulling(4);
        }
    }else{  //Each
        _DEV_FT[id].RBFT_Nulling(0);
    }
}

void RBCMD_SensorIMUNull(){
    int id = sharedData->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[0];

    if(id == -1){   //All
        for(int i=0; i<_NO_OF_IMU; i++){
            _DEV_IMU[i].RBIMU_RequestNulling();
        }
    }else{  //Each
        _DEV_IMU[id].RBIMU_RequestNulling();
    }
}

void RBCMD_SensorIMUOffsetSet(){
    int imu = sharedData->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[0];
    float roll = sharedData->COMMAND[RBCORE_PODO_NO].USER_PARA_FLOAT[0];
    float pitch = sharedData->COMMAND[RBCORE_PODO_NO].USER_PARA_FLOAT[1];

    _DEV_IMU[imu].ROLL_OFFSET = roll;
    _DEV_IMU[imu].PITCH_OFFSET = pitch;
}


void RBCMD_MotionRefOnOff(){
    int onoff = sharedData->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[0];
    _REFERENCE_ENABLED = onoff;

    for(int i=0; i<_NO_OF_MC; i++){
        _DEV_MC[i].RBBoard_ReferenceOutEnable(onoff);
    }
}

void RBCMD_MotionMove(){
    int id = sharedData->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[0];
    int ch = sharedData->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[1];
    int mode = sharedData->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[2];
    float time = sharedData->COMMAND[RBCORE_PODO_NO].USER_PARA_FLOAT[0];
    float ang = sharedData->COMMAND[RBCORE_PODO_NO].USER_PARA_FLOAT[1];

    _DEV_MC[id].MoveJoints[ch].MoveFlag = false;
    _DEV_MC[id].MoveJoints[ch].RefAngleCurrent = sharedData->JointReference[RBCORE_PODO_NO][id][ch] = sharedData->ENCODER[id][ch].CurrentReference;
    _DEV_MC[id].RBJoint_SetMoveJoint(ch, ang, time, mode);
    sharedData->MotionOwner[id][ch] = RBCORE_PODO_NO;
}

void RBCMD_MotionErrorClear(){
    int id = sharedData->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[0];
    int ch = sharedData->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[1];
    int mode = sharedData->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[2];
    if(mode != 0) mode = 1;


    ErrorClearStart = true;
    if(mode == 0){  // just error clear
        if(id == -1){ // All
            for(int i=0; i<_NO_OF_MC; i++){
                for(int j=0; j<_DEV_MC[i].MOTOR_CHANNEL; j++){
                    _DEV_MC[i].RBJoint_ClearErrorFlag(j+1);
                }
            }
        }else{  // Each
            _DEV_MC[id].RBJoint_ClearErrorFlag(ch+1);
        }
    }else{          // error clear + joint recovery
        // reference out disable & get motion ownership for all joints
        for(int i=0; i<_NO_OF_MC; i++){
            _DEV_MC[i].RBBoard_ReferenceOutEnable(false);
            for(int j=0; j<_DEV_MC[i].MOTOR_CHANNEL; j++){
                sharedData->MotionOwner[i][j] = RBCORE_PODO_NO;
            }
        }

        // encoder enable
        for(int i=0; i<_NO_OF_MC; i++){
            _DEV_MC[i].RBBoard_RequestEncoder(1);
        }

        // sleep for encoder read
        usleep(30*1000);

        for(int i=0; i<_NO_OF_MC; i++){
            for(int j=0; j<_DEV_MC[i].MOTOR_CHANNEL; j++){
                // update reference with encoder (exception RWH LWH RHAND LHAND)
//                if((i == 4 && j == 1) || (i == 10 && j == 1) || (i == 21 && j == 1) || (i == 22 && j == 1)){
//                    _DEV_MC[i].MoveJoints[j].RefAngleCurrent = 0.0;
//                }else{
                    _DEV_MC[i].MoveJoints[j].RefAngleCurrent = sharedData->ENCODER[i][j].CurrentPosition;
                //}
            }
        }

        if(id == -1){ // All
            for(int i=0; i<_NO_OF_MC; i++){
                for(int j=0; j<_DEV_MC[i].MOTOR_CHANNEL; j++){
                    // error clear
                    _DEV_MC[i].RBJoint_ClearErrorFlag(j+1);

                    // FET on & CTRL on
                    _DEV_MC[i].RBJoint_EnableFETDriver(j+1, true);
                    _DEV_MC[i].RBJoint_EnableFeedbackControl(j+1, true);
                }
            }
        }else{  // Each
            // error clear
            _DEV_MC[id].RBJoint_ClearErrorFlag(ch+1);

            // FET on & CTRL on
            _DEV_MC[id].RBJoint_EnableFETDriver(ch+1, true);
            _DEV_MC[id].RBJoint_EnableFeedbackControl(ch+1, true);
        }

        // wait for settling
        usleep(10*1000);

        // reference out enable
        for(int i=0; i<_NO_OF_MC; i++){
            for(int j=0; j<_DEV_MC[i].MOTOR_CHANNEL; j++){
                _DEV_MC[i].RBBoard_ReferenceOutEnable(true);
            }
        }
    }
    ErrorClearStart = false;
}

void RBCMD_CANEnableDisable(){
    _CANOUT_ENABLED = sharedData->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[0];
}

//==============================================================================
int RBCore_Initialize(void){
    cout << endl;
    FILE_LOG(logERROR) << "==========Initializing===========";

    IS_WORKING = true;

    // Close Every ALs
    pmHandler = new RBProcessManager();

    // Shared Memory initialize
    if(RBCore_SMInitialize() == false)
        return false;

    // Load RBCore configuration file
    if(RBCore_DBInitialize() == false)
        return false;

    // CAN Communcation initialize
    RBCore_CANInitialize();

    // Real-time thread initialize
    if(RBCore_ThreadInitialize() == false)
        return false;

    // Process Manager initialize
    if(RBCore_PMInitialize() == false)
        return false;

    RBCore_FingerInitialize();

    FILE_LOG(logERROR) << "=================================";
    cout << endl;

    sharedData->PODO_AL_WORKING[RBCORE_PODO_NO] = true;
    IS_WORKING = true;
    return true;
}
//---------------
int RBCore_SMInitialize(){
    shm_unlink(RBCORE_SHM_NAME);

    int shmFD;
    // Core Shared Memory Creation ==============================================
    shmFD = shm_open(RBCORE_SHM_NAME, O_CREAT|O_RDWR, 0666);
    if(shmFD == -1){
        FILE_LOG(logERROR) << "Fail to open core shared memory";
        return -1;
    }else{
        if(ftruncate(shmFD, sizeof(RBCORE_SHM)) == -1){
            FILE_LOG(logERROR) << "Fail to truncate core shared memory";
            return -1;
        }else{
            sharedData = (pRBCORE_SHM)mmap(0, sizeof(RBCORE_SHM), PROT_READ|PROT_WRITE, MAP_SHARED, shmFD, 0);
            if(sharedData == (void*)-1){
                FILE_LOG(logERROR) << "Fail to mapping core shared memory";
                return -1;
            }
        }
    }
    FILE_LOG(logSUCCESS) << "Core shared memory creation = OK";
    // =========================================================================

    return true;
}
//---------------
int RBCore_DBInitialize(){
    RBDataBase DB;
    DB.SetFilename("Core_Config.db");
    if(DB.OpenDB() == false){
        FILE_LOG(logERROR) << "Fail to load database file";
        return false;
    }

    _VERSION        = RBDataBase::_DB_GENERAL.VERSION;
    _NO_OF_AL       = RBDataBase::_DB_GENERAL.NO_OF_AL;
    _NO_OF_COMM_CH  = RBDataBase::_DB_GENERAL.NO_OF_COMM_CH;
    _NO_OF_MC       = RBDataBase::_DB_GENERAL.NO_OF_MC;
    _NO_OF_FT       = RBDataBase::_DB_GENERAL.NO_OF_FT;
    _NO_OF_IMU      = RBDataBase::_DB_GENERAL.NO_OF_IMU;
    _NO_OF_SP       = RBDataBase::_DB_GENERAL.NO_OF_SP;

    FILE_LOG(logSUCCESS) << "Core load database = OK";

    std::cout << "----------------------------------" << std::endl;
    std::cout << "     VERSION       : " << _VERSION << std::endl;
    std::cout << "     NO_OF_AL      : " << _NO_OF_AL << std::endl;
    std::cout << "     NO_OF_COMM_CH : " << _NO_OF_COMM_CH << std::endl;
    std::cout << "     NO_OF_MC      : " << _NO_OF_MC << std::endl;
    std::cout << "     NO_OF_FT      : " << _NO_OF_FT << std::endl;
    std::cout << "     NO_OF_IMU     : " << _NO_OF_IMU << std::endl;
    std::cout << "     NO_OF_SP      : " << _NO_OF_SP << std::endl;
    std::cout << "----------------------------------" << std::endl;

    for(int i=0; i<_NO_OF_MC; i++)   _DEV_MC[i].RBBoard_GetDBData(RBDataBase::_DB_MC[i]);
    for(int i=0; i<_NO_OF_FT; i++)   _DEV_FT[i].RBBoard_GetDBData(RBDataBase::_DB_FT[i]);
    for(int i=0; i<_NO_OF_IMU; i++)  _DEV_IMU[i].RBBoard_GetDBData(RBDataBase::_DB_IMU[i]);
    for(int i=0; i<_NO_OF_SP; i++)   _DEV_SP[i].RBBoard_GetDBData(RBDataBase::_DB_SP[i]);

    return true;
}
//---------------
int RBCore_CANInitialize(){
    canHandler = new RBCAN(_NO_OF_COMM_CH);

    if(canHandler->IsWorking() == false){
        IS_CAN_OK = false;
        return false;
    }else{
        _CANOUT_ENABLED = true;
        // Add CAN ID for the New-IMU
        canHandler->RBCAN_AddMailBox(83);
        canHandler->RBCAN_AddMailBox(84);
        canHandler->RBCAN_AddMailBox(85);

        for(int i=0; i<_NO_OF_MC; i++)   _DEV_MC[i].RBMC_AddCANMailBox();
        for(int i=0; i<_NO_OF_FT; i++)   _DEV_FT[i].RBFT_AddCANMailBox();
        for(int i=0; i<_NO_OF_IMU; i++)  _DEV_IMU[i].RBIMU_AddCANMailBox();
        for(int i=0; i<_NO_OF_SP; i++)   _DEV_SP[i].RBSP_AddCANMailBox();
        IS_CAN_OK = true;
        return true;
    }
}
//---------------
int RBCore_ThreadInitialize(){
    if(rt_task_create(&rtTaskCon, "RBCORE_TASK", 0, 99, 0) == 0){
        cpu_set_t aCPU;
        CPU_ZERO(&aCPU);
        CPU_SET(0, &aCPU);
        if(rt_task_set_affinity(&rtTaskCon, &aCPU) != 0){
            FILE_LOG(logWARNING) << "Core real-time thread set affinity CPU failed..";
        }
        if(rt_task_start(&rtTaskCon, &RBCore_RTThreadCon, NULL) == 0){
            FILE_LOG(logSUCCESS) << "Core real-time thread start = OK";
        }else{
            FILE_LOG(logERROR) << "Core real-time thread start = FAIL";
            return false;
        }
    }else{
        FILE_LOG(logERROR) << "Fail to create core real-time thread";
        return false;
    }
    return true;
}
//---------------
int RBCore_PMInitialize(){
    int ret = pmHandler->OpenAL(1);
    if(ret == -99){
        IS_CHILD = true;
        IS_WORKING = false;
        return false;
    }
    return true;
}
//---------------
int RBCore_FingerInitialize(){
    settingFile = "configs/FingerConfig.ini";
    QSettings settings(settingFile, QSettings::NativeFormat);
    EXF_R_Modifier[0] = settings.value("exfr0", "").toFloat();
    EXF_R_Modifier[1] = settings.value("exfr1", "").toFloat();
    EXF_R_Modifier[2] = settings.value("exfr2", "").toFloat();
    EXF_R_Modifier[3] = settings.value("exfr3", "").toFloat();
    EXF_R_Modifier[4] = settings.value("exfr4", "").toFloat();

    EXF_L_Modifier[0] = settings.value("exfl0", "").toFloat();
    EXF_L_Modifier[1] = settings.value("exfl1", "").toFloat();
    EXF_L_Modifier[2] = settings.value("exfl2", "").toFloat();
    EXF_L_Modifier[3] = settings.value("exfl3", "").toFloat();
    EXF_L_Modifier[4] = settings.value("exfl4", "").toFloat();

    // set defalut 1.0 value if it is not set yet
    for(int i=0; i<5; i++){
        if(EXF_R_Modifier[i] <= 0.01){
            settings.setValue(QString().sprintf("exfr%d", i), 1.0);
            EXF_R_Modifier[i] = 1.0;
        }
        if(EXF_L_Modifier[i] <= 0.01){
            settings.setValue(QString().sprintf("exfl%d", i), 1.0);
            EXF_L_Modifier[i] = 1.0;
        }
    }

    for(int i=0; i<5; i++){
        sharedData->EXF_R_Modifier[i] = EXF_R_Modifier[i];
        sharedData->EXF_L_Modifier[i] = EXF_L_Modifier[i];
    }
    return true;
}
//---------------
int RBCore_Termination(){
    if(IS_CHILD)
        return true;

    rt_task_delete(&rtTaskCon);
    shm_unlink(RBCORE_SHM_NAME);
    FILE_LOG(logERROR) << "RBCore will be terminated..";
    return true;
}
//==============================================================================

void THREAD_ReadNewIMU(void){
    RBCAN_MB mb;
    RBCAN_MB mb2;
    RBCAN_MB mb3;

    mb.channel = 0;
    mb.id = 83;
    canHandler->RBCAN_ReadData(&mb);

    if(mb.status != RBCAN_NODATA){
        double q0 = (double)((short)((mb.data[1] << 8) | mb.data[0])) / 30000.0;
        double q1 = (double)((short)((mb.data[3] << 8) | mb.data[2])) / 30000.0;
        double q2 = (double)((short)((mb.data[5] << 8) | mb.data[4])) / 30000.0;
        double q3 = (double)((short)((mb.data[7] << 8) | mb.data[6])) / 30000.0;


        double phi = atan2(2 * (q2*q3 + q0*q1), 1 - 2 * (q1*q1 + q2*q2))*57.2957914;
        double theta = -1 * asin(2 * (q1*q3 - q0*q2))*57.2957914;
        double psi = atan2(2 * (q1*q2 + q0*q3), 1 - 2 * (q2*q2 + q3*q3))*57.2957914;

        sharedData->IMU[0].Roll = phi +0.6;//+ _DEV_IMU[0].ROLL_OFFSET;
        sharedData->IMU[0].Pitch = theta +0.3;//+ _DEV_IMU[0].PITCH_OFFSET;;
        sharedData->IMU[0].Yaw = psi;

//        sharedSEN->IMU[0].Yaw = psi;
//        sharedSEN->IMU[0].Pitch = theta;
//        sharedSEN->IMU[0].Roll = phi;

        sharedData->IMU[0].Q[0] = q0;
        sharedData->IMU[0].Q[1] = q1;
        sharedData->IMU[0].Q[2] = q2;
        sharedData->IMU[0].Q[3] = q3;

        mb.status = RBCAN_NODATA;
    }

    mb2.channel = 0;
    mb2.id = 84;
    canHandler->RBCAN_ReadData(&mb2);
    if(mb2.status != RBCAN_NODATA){
        double roll_vel = (double)((short)((mb2.data[1] << 8) | mb2.data[0])) / 100.0;
        double pitch_vel = (double)((short)((mb2.data[3] << 8) | mb2.data[2])) / 100.0;
        double yaw_vel = (double)((short)((mb2.data[5] << 8) | mb2.data[4])) / 100.0;
        char imu_limit = mb2.data[6];
        if(imu_limit == 1){
            cout<<"IMU limit exceed...!!!"<<endl;
        }

        sharedData->IMU[0].RollVel = roll_vel;
        sharedData->IMU[0].PitchVel = pitch_vel;
        sharedData->IMU[0].YawVel = yaw_vel;
//        sharedSEN->IMU[0].RollVel = roll_vel;
//        sharedSEN->IMU[0].PitchVel = pitch_vel;
//        sharedSEN->IMU[0].YawVel = yaw_vel;

        mb2.status = RBCAN_NODATA;
    }

//    mb3.channel = 0;
//    mb3.id = 85;
//    canHandler->RBCAN_ReadData(&mb3);
//    if(mb3.status != RBCAN_NODATA){
//        double c_roll = (double)((short)((mb3.data[1] << 8) | mb3.data[0])) / 300.0;
//        double c_pitch = (double)((short)((mb3.data[3] << 8) | mb3.data[2])) / 300.0;
//        double a_roll = (double)((short)((mb3.data[5] << 8) | mb3.data[4])) / 300.0;
//        double a_pitch = (double)((short)((mb3.data[7] << 8) | mb3.data[6])) / 300.0;


//        sharedSEN->IMU[0].Roll_Comp = c_roll;
//        sharedSEN->IMU[0].Pitch_Comp = c_pitch;
//        sharedSEN->IMU[0].Roll_Acc = a_roll;
//        sharedSEN->IMU[0].Pitch_Acc = a_pitch;

//        mb3.status = RBCAN_NODATA;
//    }

}

void THREAD_ReadSensor(){
    if(_SENSOR_ENABLED == false)
        return;

//    for(int i=0; i<_NO_OF_IMU; i++){
//        _DEV_IMU[i].RBIMU_ReadData();
//        sharedData->IMU[i].Roll      = _DEV_IMU[i].ROLL;
//        sharedData->IMU[i].Pitch     = _DEV_IMU[i].PITCH;
//        sharedData->IMU[i].Yaw       = _DEV_IMU[i].YAW;
//        sharedData->IMU[i].RollVel   = _DEV_IMU[i].ROLL_VEL;
//        sharedData->IMU[i].PitchVel  = _DEV_IMU[i].PITCH_VEL;
//        sharedData->IMU[i].YawVel    = _DEV_IMU[i].YAW_VEL;
//        sharedData->IMU[i].AccX      = _DEV_IMU[i].ACC_X;
//        sharedData->IMU[i].AccY      = _DEV_IMU[i].ACC_Y;
//        sharedData->IMU[i].AccZ      = _DEV_IMU[i].ACC_Z;
//    }

    for(int i=0; i<_NO_OF_FT; i++){
        _DEV_FT[i].RBFT_ReadData();
        sharedData->FT[i].Fx     = _DEV_FT[i].FX;
        sharedData->FT[i].Fy     = _DEV_FT[i].FY;
        sharedData->FT[i].Fz     = _DEV_FT[i].FZ;
        sharedData->FT[i].Mx     = _DEV_FT[i].MX;
        sharedData->FT[i].My     = _DEV_FT[i].MY;
        sharedData->FT[i].Mz     = _DEV_FT[i].MZ;

        if(i == 0){
            sharedData->FT[i].Fz = _DEV_FT[i].FZ;// * 8.0/7.0;
        }

        if(_DEV_FT[i].SENSOR_TYPE == 0){
            sharedData->FT[i].Roll       = _DEV_FT[i].ROLL;
            sharedData->FT[i].RollVel    = _DEV_FT[i].VelRoll;
            sharedData->FT[i].Pitch      = _DEV_FT[i].PITCH;
            sharedData->FT[i].PitchVel   = _DEV_FT[i].VelPitch;
        }
    }
}

void THREAD_RequestSensor(){
    if(_SENSOR_ENABLED){
        for(int i=0; i<_NO_OF_FT; i++){
            if(_DEV_FT[i].SENSOR_TYPE == 0x00){
                _DEV_FT[i].RBFT_RequestData(0x03);

                _DEV_FT[i].RBFT_RequestData(0x04);
                _DEV_FT[i].RBFT_RequestData(0x00);
            }else{
                _DEV_FT[i].RBFT_RequestData(0x00);
            }
        }

        for(int i=0; i<_NO_OF_IMU; i++){
            _DEV_IMU[i].RBIMU_RequestData(0x00);
        }
    }
}

void THREAD_ReadEncoder(){
    if(_ENCODER_ENABLED == false)
        return;

    for(int i=0; i<_NO_OF_MC; i++){
        _DEV_MC[i].RBBoard_ReadEncoderData();
        for(int j=0; j<_DEV_MC[i].MOTOR_CHANNEL; j++){
            sharedData->ENCODER[i][j].CurrentPosition = _DEV_MC[i].Joints[j].CurrentPosition;
            sharedData->ENCODER[i][j].CurrentVelocity = _DEV_MC[i].Joints[j].CurrentVelocity;
        }
    }
}

void THREAD_RequestEncoder(){
//    static int finger_toggle = 0;
//    if(_ENCODER_ENABLED){
//        finger_toggle++;
//        if(finger_toggle%4 == 0){
//            _DEV_MC[MC_GetID(RF1)].RBBoard_RequestEncoder(0, 0);    // M0,M1,M2
//            _DEV_MC[MC_GetID(LF1)].RBBoard_RequestEncoder(0, 0);    // M0,M1,M2
//        }else if(finger_toggle%4 == 2){
//            _DEV_MC[MC_GetID(RF1)].RBBoard_RequestEncoder(0, 1);    // M3,M4
//            _DEV_MC[MC_GetID(LF1)].RBBoard_RequestEncoder(0, 1);    // M3,M4
//        }
//    }
}

void THREAD_ReadTemperature(){
    if(_SENSOR_ENABLED){
        int mc = _ThreadCnt%_NO_OF_MC;
        _DEV_MC[mc].RBBoard_ReadTemperature();
        sharedData->MCTemperature[mc] = _DEV_MC[mc].BoardTemperature;
        for(int i=0; i<_DEV_MC[mc].MOTOR_CHANNEL; i++){
            sharedData->MotorTemperature[mc][i] = _DEV_MC[mc].Joints[i].Temperature;
        }
    }
}

void THREAD_RequestTemperature(){
    int mc = _ThreadCnt%_NO_OF_MC;

//    // temperature
//    if(_SENSOR_ENABLED){
//        _DEV_MC[mc].RBBoard_RequestTemperature();
//    }

    // home & error
    if(_DEV_MC[mc].ConnectionStatus == true){
        if(_DEV_MC[mc].BOARD_ID < 32){
            StatusReadFlag[mc] = false;
            _DEV_MC[mc].RBBoard_RequestStatus();
        }
    }
}


void THREAD_ReadVoltage(){
    if(_SENSOR_ENABLED){
        _DEV_SP[0].RBSP_ReadVoltageCurrent();
        sharedData->SP[0].Voltage = _DEV_SP[0].Voltage;
        sharedData->SP[0].Current = _DEV_SP[0].Current;
    }
}

void THREAD_RequestVoltage(){
    if(_SENSOR_ENABLED){
        _DEV_SP[0].RBSP_RequestVoltageAndCurrent();
    }
}


void THREAD_ReadHomeError(){
    static unsigned int StatusErrorCnt[MAX_MC] = {0,};

    //if(_SENSOR_ENABLED){
    int mc = _ThreadCnt%_NO_OF_MC;


    if(_DEV_MC[mc].ConnectionStatus == true){
        if(_DEV_MC[mc].RBBoard_GetStatus() == true){
            StatusErrorCnt[mc] = 0;
            StatusReadFlag[mc] = true;

            for(int j=0; j<_DEV_MC[mc].MOTOR_CHANNEL; j++){
                sharedData->MCStatus[mc][j] = _DEV_MC[mc].Joints[j].CurrentStatus;
                sharedData->MCStatus[mc][j].b.CUR = 0;
                if(sharedData->MCStatus[mc][j].b.BIG == 1 || sharedData->MCStatus[mc][j].b.INP == 1 ||
                        sharedData->MCStatus[mc][j].b.ENC == 1 || sharedData->MCStatus[mc][j].b.JAM == 1 )
                {
                    if(ErrorClearStart == false){
                        //FILE_LOG(logERROR) << "Need Recover Activated.. Reason: Servo[" << mc << ", " << j << "] Off with Error(" << (int)(sharedData->MCStatus[mc][j].B[1]) << ")";
                        //NeedJointRecover = true;
                    }
                }
            }
        }else if(_SENSOR_ENABLED && StatusReadFlag[mc] == false){
            StatusErrorCnt[mc]++;
            if(StatusErrorCnt[mc] > 3 && _DEV_MC[mc].BOARD_ID < 32){
                if(ErrorClearStart == false){
                    for(int j=0; j<_DEV_MC[mc].MOTOR_CHANNEL; j++){
                        sharedData->MCStatus[mc][j].b.CUR = 1;
                    }
                    //FILE_LOG(logERROR) << "Need Recover Activated.. Reason: No Status Return";
                    //NeedCANRecover = true;
                }
            }

            if(StatusErrorCnt[mc]%3 == 0 && _DEV_MC[mc].BOARD_ID < 32)
                FILE_LOG(logERROR) << "Status Get Error from Board " << mc << " Cnt : " << StatusErrorCnt[mc];

        }
    }

    //}
}
