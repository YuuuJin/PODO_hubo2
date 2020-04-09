

#include "BasicFiles/BasicSetting.h"
#include "BasicFiles/TaskMotion.h"
#include "RBJoystick.h"


#define BIT_PC 90 //90
// Basic --------
pRBCORE_SHM             sharedData;
pUSER_SHM               userData;
JointControlClass       *jCon;
TaskMotion              *mCon;
ControllerControlClass  *cCon;


int     __IS_WORKING = false;
int     __IS_GAZEBO = false;
int     PODO_NO = -1;


// motion -----------------
int  WB_FLAG = false;

void StopWB();
void StartWB(int _mode);

void ReadTextData(char *_filename);
int OpenFile(const char* _filename);
int CloseFile();
// motion -----------------


int hubo_vision_on=DISABLE;

int     motionLength = 0;
float   stepLength = 0.0f;



int     pyeongchangdemo_num;
float   ringData[RING_SIZE][31];
int     ringIndexFront;
int     ringIndexBack;
float   localJoint[NO_OF_JOINTS];
float   localQpel[4];
float   localDeltaPCz;
float   WR_PelcZ;
int     fileReadFlag;
FILE*   fpMocap;


RBJoystick  *joy;
char    JOY_B = 0, JOY_X = 0;
int     JOY_LJOG_RL = 0, JOY_LJOG_UD = 0;
int     JOY_HEAD_MANUAL_MODE_FLAG = false;
float   HEAD_UD_REF = 0.0;
float   HEAD_RL_REF = 0.0;

//Controller
KINE_DRC_HUBO4 kine_drc_hubo4;
double WalkReadyRef[34] = {0.,};
double WBIK_Q0[34] = {0.,};
double WBIK_Q[34] = {0.,},Qub[34]={0.,};

void WBIK_PARA_CHANGE();
void WholebodyInitialize();
void WholebodyFinalize();
void get_zmp2();
void WBIK();

double  X_ZMP_Local,Y_ZMP_Local,X_ZMP_Global,Y_ZMP_Global,X_ZMP_REF_Local,Y_ZMP_REF_Local,X_ZMP_REF_Global,Y_ZMP_REF_Global;
double GLOBAL_ZMP_REF_X,GLOBAL_ZMP_REF_Y;
double com1,com2;
double t_thread=0.,t0,t1,t2;
double lamda;
double T10,T21;
double a1,b1,c1;
double a2,b2,c2;
double zc,xc,yc,xRF,yRF,zRF,xLF,yLF,zLF;
double LeftPitch;
double RSR_demo,LSR_demo;
double AddJoint;

double des_pCOM_3x1[3]={0,}, des_qPEL_4x1[4]={1,0,0,0}, des_pRF_3x1[3]={0,}, des_qRF_4x1[4]={1,0,0,0}, des_pLF_3x1[3]={0,}, des_qLF_4x1[4]={1,0,0,0};
int DemoFlag;

double I_ZMP_CON_X=0.f,I_ZMP_CON_Y=0.f;
double I_ZMP_CON_X_last=0.f,I_ZMP_CON_Y_last=0.f;
double Old_I_ZMP_CON_X=0.f,Old_I_ZMP_CON_Y=0.f;
double  LPF_Del_PC_X_DSP_XZMP_CON = 0., LPF_Del_PC_Y_DSP_YZMP_CON = 0.;
double  LPF_Del_PC_X_SSP_XZMP_CON = 0., LPF_Del_PC_Y_SSP_YZMP_CON = 0.;
double  Del_PC_X_DSP_XZMP_CON = 0., Del_PC_Y_DSP_YZMP_CON = 0., Old_Del_PC_X_DSP_XZMP_CON = 0., Old_Del_PC_Y_DSP_YZMP_CON = 0.,
        Del_PC_X_SSP_XZMP_CON = 0., Del_PC_Y_SSP_YZMP_CON = 0., Old_Del_PC_X_SSP_XZMP_CON = 0., Old_Del_PC_Y_SSP_YZMP_CON = 0.;
double  Old_Del_PC_X_DSP_XZMP_CON2 = 0;
double  Old_Del_PC_Y_DSP_YZMP_CON2 = 0;

double temp1des_qLF_4x1[4],temp2des_qLF_4x1[4],temp3des_qLF_4x1[4],temp4des_qLF_4x1[4],temp5des_qLF_4x1[4];
double temp1des_qPEL_4x1[4],temp2des_qPEL_4x1[4],temp3des_qPEL_4x1[4],temp4des_qPEL_4x1[4],temp5des_qPEL_4x1[4];

double PELx;
bool Control_OnOff_Flag;

const double DEL_T = 0.005;
//const double    OFFSET_ELB = -20.0;
//const double    OFFSET_RSR = -15.0;
//const double    OFFSET_LSR = 15.0;
double curRSR,curLSR;
double  final_gain_DSP_ZMP_CON = 0., final_gain_SSP_ZMP_CON = 0.;
unsigned int CNT_final_gain_DSP_ZMP_CON = 0,  CNT_final_gain_SSP_ZMP_CON = 0;
void Kirk_Control();
void Kirk_Control_ssp();
void ZMP_intergral_control();
// DSP ZMP Controller
double  kirkZMPCon_XP1(double u, double ZMP, int zero);
double  kirkZMPCon_YP1(double u, double ZMP, int zero);
double  kirkZMPCon_XP2(double u, double ZMP, int zero);
double  kirkZMPCon_YP2(double u, double ZMP, int zero);

extern int motion_loop_cnt;
extern int music_offset_2018;
extern int bit_int_pc;
enum FTNAME{
    RAFT = 0,
    LAFT
};


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
    sprintf(__AL_NAME, "PYEONGCHANGDEMO");


    CheckArguments(argc, argv);
    if(PODO_NO == -1){
        FILE_LOG(logERROR) << "Please check the AL Number";
        FILE_LOG(logERROR) << "Terminate this AL..";
        //return 0;
        PODO_NO = 3;
    }

    joy = new RBJoystick();
    int joy_connect_cnt = 0;


    // Initialize RBCore -----------------------------------
    if(RBInitialize() == false)
        __IS_WORKING = false;

    mCon = new TaskMotion(sharedData, jCon);

    jCon->RefreshToCurrentReference();
    mCon->RefreshToCurrentReference();




    if(RBStartThread() == false)
        __IS_WORKING = false;

    //jCon->SetMotionOwner(0);
    while(__IS_WORKING){
        usleep(100*1000);

        switch(sharedData->COMMAND[PODO_NO].USER_COMMAND){
        case PYEONGCHANG_DEMO:
//            FILE_LOG(logSUCCESS) << "Pyeongchang_DEMO: " << sharedData->COMMAND[PODO_NO].USER_PARA_INT[3];

            if(JOY_HEAD_MANUAL_MODE_FLAG){
                FILE_LOG(logERROR) << "Joy Manual Head Mode is Working Now..";
                sharedData->COMMAND[PODO_NO].USER_COMMAND = NO_ACT;
                break;
            }if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[3] == TASK_HANDSHAKE){

                StartWB(0);
                mCon->tmTimeCnt = 0;
                mCon->sciMotionSelector = PYEONGCHANG_DEMO;
                mCon->TaskMotionSelector = PYEONGCHANG_DEMO;
                pyeongchangdemo_num = TASK_HANDSHAKE;
            }if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[3] == TASK_BOUQUET){
                StartWB(0);
                mCon->tmTimeCnt = 0;
                mCon->sciMotionSelector = PYEONGCHANG_DEMO;
                mCon->TaskMotionSelector = PYEONGCHANG_DEMO;
                pyeongchangdemo_num = TASK_BOUQUET;
            }if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[3] == TASK_MOTION3){
                StartWB(0);
                mCon->tmTimeCnt = 0;
                mCon->sciMotionSelector = PYEONGCHANG_DEMO;
                mCon->TaskMotionSelector = PYEONGCHANG_DEMO;
                pyeongchangdemo_num = TASK_MOTION3;
            }if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[3] == TASK_MOTION4){
                StartWB(0);
                mCon->tmTimeCnt = 0;
                mCon->sciMotionSelector = PYEONGCHANG_DEMO;
                mCon->TaskMotionSelector = PYEONGCHANG_DEMO;
                pyeongchangdemo_num = TASK_MOTION4;
            }if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[3] == TASK_MOTION5){
                StartWB(0);
                mCon->tmTimeCnt = 0;
                mCon->sciMotionSelector = PYEONGCHANG_DEMO;
                mCon->TaskMotionSelector = PYEONGCHANG_DEMO;
                pyeongchangdemo_num = TASK_MOTION5;
            }if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[3] == TASK_MOTION6){
                StartWB(0);
                mCon->tmTimeCnt = 0;
                mCon->sciMotionSelector = PYEONGCHANG_DEMO;
                mCon->TaskMotionSelector = PYEONGCHANG_DEMO;
                pyeongchangdemo_num = TASK_MOTION6;
            }if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[3] == TASK_MOTION7){
                StartWB(0);
                mCon->tmTimeCnt = 0;
                mCon->sciMotionSelector = PYEONGCHANG_DEMO;
                mCon->TaskMotionSelector = PYEONGCHANG_DEMO;
                pyeongchangdemo_num = TASK_MOTION7;
            }if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[3] == TASK_MOTION8){
                StartWB(0);
                mCon->tmTimeCnt = 0;
                mCon->sciMotionSelector = PYEONGCHANG_DEMO;
                mCon->TaskMotionSelector = PYEONGCHANG_DEMO;
                pyeongchangdemo_num = TASK_MOTION8;
            }if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[3] == TASK_MOTION9){
                StartWB(0);
                mCon->tmTimeCnt = 0;
                mCon->sciMotionSelector = PYEONGCHANG_DEMO;
                mCon->TaskMotionSelector = PYEONGCHANG_DEMO;
                pyeongchangdemo_num = TASK_MOTION9;
            }if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[3] == TASK_MOTION10){
                StartWB(0);
                mCon->tmTimeCnt = 0;
                mCon->sciMotionSelector = PYEONGCHANG_DEMO;
                mCon->TaskMotionSelector = PYEONGCHANG_DEMO;
                pyeongchangdemo_num = TASK_MOTION10;
            }if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[3] == TEST_1){
                StartWB(0);
                mCon->tmTimeCnt = 0;
                mCon->sciMotionSelector = PYEONGCHANG_DEMO;
                mCon->TaskMotionSelector = PYEONGCHANG_DEMO;
                pyeongchangdemo_num = TEST_1;
            }if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[3] == TEST_2){
                StartWB(0);
                mCon->tmTimeCnt = 0;
                mCon->sciMotionSelector = PYEONGCHANG_DEMO;
                mCon->TaskMotionSelector = PYEONGCHANG_DEMO;
                pyeongchangdemo_num = TEST_2;
            }if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[3] == TEST_3){
                StartWB(0);
                mCon->tmTimeCnt = 0;
                mCon->sciMotionSelector = PYEONGCHANG_DEMO;
                mCon->TaskMotionSelector = PYEONGCHANG_DEMO;
                pyeongchangdemo_num = TEST_3;
            }if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[3] == TEST_4){
                StartWB(0);
                mCon->tmTimeCnt = 0;
                mCon->sciMotionSelector = PYEONGCHANG_DEMO;
                mCon->TaskMotionSelector = PYEONGCHANG_DEMO;
                pyeongchangdemo_num = TEST_4;
            }
            //--------------------------------------------------------------------------------------

            if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[3] == HANDSHAKE_READY){
                StartWB(0);
                //WholebodyInitialize();
                mCon->tmTimeCnt = 0;
                mCon->sciMotionSelector = PYEONGCHANG_DEMO;
                mCon->TaskMotionSelector = PYEONGCHANG_DEMO;
                pyeongchangdemo_num = HANDSHAKE_READY;
            }if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[3] == HANDSHAKE_LOWGAIN){
                StartWB(0);
                mCon->tmTimeCnt = 0;
                mCon->sciMotionSelector = PYEONGCHANG_DEMO;
                mCon->TaskMotionSelector = PYEONGCHANG_DEMO;
                pyeongchangdemo_num = HANDSHAKE_LOWGAIN;
            }if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[3] == HANDSHAKE_GRIP_ON){
                StartWB(0);
                mCon->tmTimeCnt = 0;
                mCon->sciMotionSelector = PYEONGCHANG_DEMO;
                mCon->TaskMotionSelector = PYEONGCHANG_DEMO;
                pyeongchangdemo_num = HANDSHAKE_GRIP_ON;
            }if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[3] == HANDSHAKE_GRIP_OFF){
                StartWB(0);
                mCon->tmTimeCnt = 0;
                mCon->sciMotionSelector = PYEONGCHANG_DEMO;
                mCon->TaskMotionSelector = PYEONGCHANG_DEMO;
                pyeongchangdemo_num = HANDSHAKE_GRIP_OFF;
            }if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[3] == HANDSHAKE_HIGHGAIN){
                StartWB(0);
                mCon->tmTimeCnt = 0;
                mCon->sciMotionSelector = PYEONGCHANG_DEMO;
                mCon->TaskMotionSelector = PYEONGCHANG_DEMO;
                pyeongchangdemo_num = HANDSHAKE_HIGHGAIN;
            }if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[3] == HANDSHAKE_INIT){
                StartWB(0);
                mCon->tmTimeCnt = 0;
                mCon->sciMotionSelector = PYEONGCHANG_DEMO;
                mCon->TaskMotionSelector = PYEONGCHANG_DEMO;
                pyeongchangdemo_num = HANDSHAKE_INIT;
            }
            //--------------------------------------------------------------------------------------

            if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[3] == BOUQUET_READY){
                StartWB(0);
                mCon->tmTimeCnt = 0;
                mCon->sciMotionSelector = PYEONGCHANG_DEMO;
                mCon->TaskMotionSelector = PYEONGCHANG_DEMO;
                pyeongchangdemo_num = BOUQUET_READY;
            }if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[3] == BOUQUET_GIVE){
                StartWB(0);
                mCon->tmTimeCnt = 0;
                mCon->sciMotionSelector = PYEONGCHANG_DEMO;
                mCon->TaskMotionSelector = PYEONGCHANG_DEMO;
                pyeongchangdemo_num = BOUQUET_GIVE;
            }if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[3] == BOUQUET_INIT){
                StartWB(0);
                mCon->tmTimeCnt = 0;
                mCon->sciMotionSelector = PYEONGCHANG_DEMO;
                mCon->TaskMotionSelector = PYEONGCHANG_DEMO;
                pyeongchangdemo_num = BOUQUET_INIT;
            }
            //--------------------------------------------------------------------------------------

            if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[3] == CLAP_READY){
                StartWB(0);
                mCon->tmTimeCnt = 0;
                mCon->sciMotionSelector = PYEONGCHANG_DEMO;
                mCon->TaskMotionSelector = PYEONGCHANG_DEMO;
                mCon->clap_cnt = sharedData->COMMAND[PODO_NO].USER_PARA_CHAR[0];
                pyeongchangdemo_num = CLAP_READY;
            }if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[3] == CLAP_ONE){
                StartWB(0);
                mCon->tmTimeCnt = 0;
                mCon->sciMotionSelector = PYEONGCHANG_DEMO;
                mCon->TaskMotionSelector = PYEONGCHANG_DEMO;
                pyeongchangdemo_num = CLAP_ONE;
            }if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[3] == CLAP_THREE){
                StartWB(0);
                mCon->tmTimeCnt = 0;
                mCon->sciMotionSelector = PYEONGCHANG_DEMO;
                mCon->TaskMotionSelector = PYEONGCHANG_DEMO;
                pyeongchangdemo_num = CLAP_THREE;
            }if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[3] == CLAP_KOREA){
                StartWB(0);
                mCon->tmTimeCnt = 0;
                mCon->sciMotionSelector = PYEONGCHANG_DEMO;
                mCon->TaskMotionSelector = PYEONGCHANG_DEMO;
                pyeongchangdemo_num = CLAP_KOREA;
            }if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[3] == CLAP_INIT){
                StartWB(0);
                mCon->tmTimeCnt = 0;
                mCon->sciMotionSelector = PYEONGCHANG_DEMO;
                mCon->TaskMotionSelector = PYEONGCHANG_DEMO;
                pyeongchangdemo_num = CLAP_INIT;
            }
            //--------------------------------------------------------------------------------------

            if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[3] == HEAD_YES){
                StartWB(0);
                mCon->tmTimeCnt = 0;
                mCon->sciMotionSelector = PYEONGCHANG_DEMO;
                mCon->TaskMotionSelector = PYEONGCHANG_DEMO;
                pyeongchangdemo_num = HEAD_YES;
            }if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[3] == HEAD_NO){
                StartWB(0);
                mCon->tmTimeCnt = 0;
                mCon->sciMotionSelector = PYEONGCHANG_DEMO;
                mCon->TaskMotionSelector = PYEONGCHANG_DEMO;
                pyeongchangdemo_num = HEAD_NO;
            }if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[3] == HEAD_STRETCHING){
                StartWB(0);
                mCon->tmTimeCnt = 0;
                mCon->sciMotionSelector = PYEONGCHANG_DEMO;
                mCon->TaskMotionSelector = PYEONGCHANG_DEMO;
                pyeongchangdemo_num = HEAD_STRETCHING;
            }if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[3] == HEAD_INIT){
                StartWB(0);
                mCon->tmTimeCnt = 0;
                mCon->sciMotionSelector = PYEONGCHANG_DEMO;
                mCon->TaskMotionSelector = PYEONGCHANG_DEMO;
                pyeongchangdemo_num = HEAD_INIT;
            }
            //--------------------------------------------------------------------------------------

            if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[3] == BOW_ONE){
                StartWB(0);
                mCon->tmTimeCnt = 0;
                mCon->sciMotionSelector = PYEONGCHANG_DEMO;
                mCon->TaskMotionSelector = PYEONGCHANG_DEMO;
                pyeongchangdemo_num = BOW_ONE;
            }if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[3] == BOW_TWO){
                StartWB(0);
                mCon->tmTimeCnt = 0;
                mCon->sciMotionSelector = PYEONGCHANG_DEMO;
                mCon->TaskMotionSelector = PYEONGCHANG_DEMO;
                pyeongchangdemo_num = BOW_TWO;
            }if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[3] == BOW_TWO_W){
                StartWB(0);
                mCon->tmTimeCnt = 0;
                sprintf(mCon->filename, "../share/WB_Data/New_UB_ScienceMuseum_2.txt");
                ReadTextData(mCon->filename);
                mCon->sciMotionSelector = PYEONGCHANG_DEMO;
                mCon->TaskMotionSelector = PYEONGCHANG_DEMO;
                pyeongchangdemo_num = BOW_TWO_W;
            }if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[3] == BOW_GREETING){
                StartWB(0);
                mCon->tmTimeCnt = 0;
                sprintf(mCon->filename, "../share/WB_Data/New_UB_ScienceMuseum_12.txt");
                ReadTextData(mCon->filename);
                mCon->sciMotionSelector = PYEONGCHANG_DEMO;
                mCon->TaskMotionSelector = PYEONGCHANG_DEMO;
                pyeongchangdemo_num = BOW_GREETING;
            }
            //--------------------------------------------------------------------------------------

            if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[3] == GUID_LEFT){
                StartWB(0);
                mCon->tmTimeCnt = 0;
                mCon->sciMotionSelector = PYEONGCHANG_DEMO;
                mCon->TaskMotionSelector = PYEONGCHANG_DEMO;
                pyeongchangdemo_num = GUID_LEFT;
            }if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[3] == GUID_RIGHT){
                StartWB(0);
                mCon->tmTimeCnt = 0;
                mCon->sciMotionSelector = PYEONGCHANG_DEMO;
                mCon->TaskMotionSelector = PYEONGCHANG_DEMO;
                pyeongchangdemo_num = GUID_RIGHT;
            }if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[3] == GUID_INIT){
                StartWB(0);
                mCon->tmTimeCnt = 0;
                mCon->sciMotionSelector = PYEONGCHANG_DEMO;
                mCon->TaskMotionSelector = PYEONGCHANG_DEMO;
                pyeongchangdemo_num = GUID_INIT;
            }
            //--------------------------------------------------------------------------------------
            if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[3] == 2018){
                StartWB(0);
                mCon->tmTimeCnt = 0;
                mCon->sciMotionSelector = PYEONGCHANG_DEMO;
                mCon->TaskMotionSelector = PYEONGCHANG_DEMO;
                pyeongchangdemo_num=2018;
                motion_loop_cnt = 0;
                bit_int_pc = BIT_PC;
                music_offset_2018 = 620;
            }if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[3] == 2017){
                StartWB(0);
                mCon->tmTimeCnt = 0;
                mCon->sciMotionSelector = PYEONGCHANG_DEMO;
                mCon->TaskMotionSelector = PYEONGCHANG_DEMO;
                pyeongchangdemo_num=2017;
            }









            sharedData->COMMAND[PODO_NO].USER_COMMAND = NO_ACT;
        default:
            break;
        }


        // Joystick Connection Control-----
        if(joy->fdJoy == 0){
            if(joy_connect_cnt%20 == 10){
                joy->ConnectJoy("/dev/input/js0");
            }else if(joy_connect_cnt%20 == 19){
                joy->ConnectJoy("/dev/input/js1");
                joy_connect_cnt = 0;
            }else{
                joy_connect_cnt++;
            }
        }
        if(joy->connection == true){
            sharedData->JOY_STATUS = true;

            JOY_LJOG_RL = joy->JoyAxis[0];
            JOY_LJOG_UD = joy->JoyAxis[1];
            JOY_B = joy->JoyButton[1];
            JOY_X = joy->JoyButton[2];
        }else{
            sharedData->JOY_STATUS = false;
            JOY_LJOG_RL = 0;
            JOY_LJOG_UD = 0;
            JOY_B = 0;
            JOY_X = 0;
        }
        sharedData->JOY_HEAD_JOG[0] = JOY_LJOG_UD;
        sharedData->JOY_HEAD_JOG[1] = JOY_LJOG_RL;
        sharedData->JOY_HEAD_BTN[0] = JOY_B;
        sharedData->JOY_HEAD_BTN[1] = JOY_X;


        if(JOY_B == 1 && JOY_HEAD_MANUAL_MODE_FLAG == false){
            if(pyeongchangdemo_num == NO_ACT){
                FILE_LOG(logSUCCESS) << "Joy Manual Head Mode is Now Working..";
                JOY_HEAD_MANUAL_MODE_FLAG = true;
                HEAD_UD_REF = sharedData->ENCODER[MC_GetID(NK1)][MC_GetCH(NK1)].CurrentPosition;
                HEAD_RL_REF = sharedData->ENCODER[MC_GetID(NKY)][MC_GetCH(NKY)].CurrentPosition;
            }else{
                FILE_LOG(logERROR) << "PyeongChang Mode is Working now..";
            }
        }else if(JOY_X == 1 && JOY_HEAD_MANUAL_MODE_FLAG == true){
            FILE_LOG(logSUCCESS) << "PyeongChang Mode is Now Working..";
            for(int i=NKY; i<=NK2; i++){
                mCon->wbUBJoint->motionJoint[i]->RefAngleCurrent = sharedData->ENCODER[MC_GetID(i)][MC_GetCH(i)].CurrentPosition;
                mCon->wbUBJoint->setMoveJointAngle(mCon->wbUBJoint->motionJoint[i], 0.0, 500, MOVE_ABSOLUTE);
            }
            usleep(500*1000);
            JOY_HEAD_MANUAL_MODE_FLAG = false;
        }
        // ----------------------------------


        //FILE_LOG(logINFO) << (int)(pyeongchangdemo_num);

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

        switch(mCon->TaskMotionSelector)
        {
        case PYEONGCHANG_DEMO:
            mCon->pyeongchangDemo();
            break;
        default:
            break;
        }
        get_zmp2();
        WBIK();
//        if (pyeongchangdemo_num == NO_ACT)
//            WholebodyInitialize();
//        printf("X_ZMP_Global = %f, %f\n",X_ZMP_Global,Y_ZMP_Global);
        //WholebodyInitialize();


        if(WB_FLAG == true){
            mCon->taskUpdate();
            mCon->WBIK();

            for(int i=RHY; i<=LAR; i++) jCon->SetJointRefAngle(i, mCon->Q_filt_34x1[idRHY+i-RHY]*R2D);

            jCon->SetJointRefAngle(WST, mCon->Q_filt_34x1[idWST]*R2D);

            jCon->SetJointRefAngle(RSP, mCon->Q_filt_34x1[idRSP]*R2D);
            jCon->SetJointRefAngle(RSR, mCon->Q_filt_34x1[idRSR]*R2D - OFFSET_RSR);
            jCon->SetJointRefAngle(RSY, mCon->Q_filt_34x1[idRSY]*R2D);
            jCon->SetJointRefAngle(REB, mCon->Q_filt_34x1[idREB]*R2D - OFFSET_ELB);
            jCon->SetJointRefAngle(RWY, mCon->Q_filt_34x1[idRWY]*R2D);
            jCon->SetJointRefAngle(RWP, mCon->Q_filt_34x1[idRWP]*R2D);
            //jCon->SetJointRefAngle(RWY2, mCon->Q_filt_34x1[idRWY2]*R2D);

            jCon->SetJointRefAngle(LSP, mCon->Q_filt_34x1[idLSP]*R2D);
            jCon->SetJointRefAngle(LSR, mCon->Q_filt_34x1[idLSR]*R2D - OFFSET_LSR);
            jCon->SetJointRefAngle(LSY, mCon->Q_filt_34x1[idLSY]*R2D);
            jCon->SetJointRefAngle(LEB, mCon->Q_filt_34x1[idLEB]*R2D - OFFSET_ELB);
            jCon->SetJointRefAngle(LWY, mCon->Q_filt_34x1[idLWY]*R2D);
            jCon->SetJointRefAngle(LWP, mCon->Q_filt_34x1[idLWP]*R2D);
            //jCon->SetJointRefAngle(LWY2, mCon->Q_filt_34x1[idLWY2]*R2D);

            if(!HasAnyOwnership())
                WB_FLAG = false;
        }

        // for Neck
        if(JOY_HEAD_MANUAL_MODE_FLAG == true){
            HEAD_RL_REF -= (float)JOY_LJOG_RL/32767.0 * 0.1;
            HEAD_UD_REF -= (float)JOY_LJOG_UD/32767.0 * 0.1;
            if(HEAD_RL_REF > 40.0)
                HEAD_RL_REF = 40.0;
            if(HEAD_RL_REF < -40.0)
                HEAD_RL_REF = -40.0;
            if(HEAD_UD_REF > 20.0)
                HEAD_UD_REF = 20.0;
            if(HEAD_UD_REF < -20.0)
                HEAD_UD_REF = -20.0;
            jCon->SetJointRefAngle(NKY, HEAD_RL_REF);
            jCon->SetJointRefAngle(NK1, HEAD_UD_REF);
            jCon->SetJointRefAngle(NK2, HEAD_UD_REF);
            //FILE_LOG(logWARNING) << HEAD_RL_REF << ", " << HEAD_UD_REF;
        }else{
            for(int i=NKY; i<=NK2; i++){
                mCon->wbUBJoint->moveJointAngle(mCon->wbUBJoint->motionJoint[i]);
                jCon->SetJointRefAngle(i, mCon->wbUBJoint->motionJoint[i]->RefAngleCurrent);
            }
        }

        // for hand
        for(int i=RF1; i<=LF5; i++){
            mCon->wbUBJoint->moveFinger(mCon->wbUBJoint->motionJoint[i]);
            jCon->SetJointRefAngle(i, mCon->wbUBJoint->motionJoint[i]->RefAngleCurrent);
        }
//        for(int i=RF1; i<=RF5; i++){
//            FILE_LOG(logERROR) << mCon->wbUBJoint->motionJoint[i]->RefAngleCurrent;
//        }

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

        if(HasAnyOwnership()){
            if(sharedData->SYNC_SIGNAL[PODO_NO] == true){
                jCon->JointUpdate();
                rt_task_resume(&rtTaskCon);
            }
        }
    }
}
//==============================//


void StopWB(){
    WB_FLAG = false;
}

void StartWB(int _mode){
    StopWB();
    jCon->RefreshToCurrentReference();
    mCon->ResetGlobalCoord(0);  // pelvis center
    mCon->RefreshToCurrentReference();
    mCon->Arm_Joint_mode_flag = ENABLE;
    if(_mode == 0)
    jCon->SetAllMotionOwner();
    else if(_mode == 1)
    {
        for(int i=12; i<NO_OF_JOINTS; i++){
            jCon->SetMotionOwner(i);
        }
    }
    WB_FLAG = true;
}


void ReadTextData(char *_filename)
{
    int i;
    fileReadFlag = DISABLE;
    //sci_mode = mocap_mode;
    if(OpenFile(_filename) == HUBO2_FAIL)
    {
        sharedData->COMMAND[PODO_NO].USER_COMMAND = NO_ACT;
        //printf(">>> Fail to open motion capture data file..!!\n");
    }
    else
    {
//        i = HUBO2UtilControllerBind("CON_WB");
//        Hubo2Command->ControllerOnOff[i] = ENABLE;

        while (fgetc(fpMocap) != '\n');

        fscanf(fpMocap,"%d",&motionLength);
        fgetc(fpMocap);	// move to the next line from the scalar
        fgetc(fpMocap);	// move to the next line from the blank line

        while (fgetc(fpMocap) != '\n');

        fscanf(fpMocap,"%f",&stepLength);
        fgetc(fpMocap);	// move to the next line from the scalar
        fgetc(fpMocap);	// move to the next line from the blank line

        while (fgetc(fpMocap) != '\n');

        //printf("Motion Length: %d\t Step Length: %f\n", motionLength, stepLength);


        // get the current motion reference of Hubo2
        for(i=0; i<NO_OF_JOINTS; i++) localJoint[i] = sharedData->ENCODER[MC_GetID(i)][MC_GetCH(i)].CurrentReference;
            localQpel[0] = mCon->qPEL_4x1[0];
            localQpel[1] = mCon->qPEL_4x1[1];
            localQpel[2] = mCon->qPEL_4x1[2];
            localQpel[3] = mCon->qPEL_4x1[3];
            WR_PelcZ = mCon->pPelZ;
            localDeltaPCz = 0.0f;


        // change motion type & unlock the motion protection
        // for(i=0; i<NO_OF_JOINTS; i++) Hubo2->MotionOwned[i] = MotionNumber;
        // change control mode as "velocity control"
       // for(i=NKY; i<=NK2; i++) Hubo2->Joint[i].JointControlMode = JOINT_CONTROL_MODE_VEL;
       //  for(i=NKY; i<=NK2; i++) Hubo2->Joint[i].JointControlMode = JOINT_CONTROL_MODE_POS;
       // for(i=RF1; i<=LF5; i++) Hubo2->Joint[i].JointControlMode = JOINT_CONTROL_MODE_CUR;

        // parameter change
        fileReadFlag = ENABLE;
    }
}

// --------------------------------------------------------------------------------------------- //
int OpenFile(const char *_filename)
{
    //printf("%s", _filename);
    if(fpMocap != NULL) // if another file pointer is allocated
    {
        if(CloseFile() == HUBO2_FAIL)
        {
            //printf("Cannot open file..!!\n");
            return HUBO2_FAIL;
        }
    }

    fpMocap = fopen(_filename, "r");
    if(fpMocap == NULL)
    {
        //printf("Cannot open file..!!\n");
        return HUBO2_FAIL;
    }
    return HUBO2_SUCCESS;
}
// --------------------------------------------------------------------------------------------- //

// --------------------------------------------------------------------------------------------- //
int CloseFile()
{
    if(fclose(fpMocap) == HUBO2_SUCCESS)
    {
        fpMocap = NULL;
        return HUBO2_SUCCESS;
    }
    else
    {
        return HUBO2_FAIL;
    }
}
// --------------------------------------------------------------------------------------------- //
void WholebodyFinalize()
{
//    Control_OnOff_Flag = false;
}

void WholebodyInitialize()
{
    WBIK_PARA_CHANGE();
    CNT_final_gain_DSP_ZMP_CON = 0;
//    Control_OnOff_Flag = true;

    Qub[idRSP] = 10.*D2R;
    Qub[idRSR] = -10.*D2R ;//+ OFFSET_RSR*D2R;
    Qub[idRSY] = -0.*D2R;
    Qub[idREB] = -30.*D2R;// + OFFSET_ELB*D2R;

    Qub[idLSP] = 10.*D2R;
    Qub[idLSR] = 10.*D2R;//+ OFFSET_RSR*D2R;
    Qub[idLSY] =  0.*D2R;
    Qub[idLEB] = -30.*D2R;//+ OFFSET_ELB*D2R;

    Qub[idRWP] =  0.*D2R;
    Qub[idLWP] =  0.*D2R;

    WBIK_Q0[idRHY] = 0.;
    WBIK_Q0[idRHR] = -2.78*D2R;
    WBIK_Q0[idRHP] = -43.9*D2R;
    WBIK_Q0[idRKN] = 80.58*D2R; //77.4*D2R;
    WBIK_Q0[idRAP] = -36.68*D2R;
    WBIK_Q0[idRAR] = 2.78*D2R;

    WBIK_Q0[idLHY] = 0.;
    WBIK_Q0[idLHR] = 2.78*D2R;
    WBIK_Q0[idLHP] = -43.9*D2R;
    WBIK_Q0[idLKN] = 80.58*D2R;//77.4*D2R;
    WBIK_Q0[idLAP] = -36.68*D2R;
    WBIK_Q0[idLAR] = -2.78*D2R;
    WBIK_Q0[idRSP] = -5.*D2R;
    WBIK_Q0[idLSP] = -5.*D2R;

    WBIK_Q0[idRSR] = 10.*D2R;
    WBIK_Q0[idLSR] = -10.*D2R;

    WBIK_Q0[idRSY] = 0.*D2R;
    WBIK_Q0[idLSY] = 0.*D2R;

    WBIK_Q0[idREB] = -130.*D2R;
    WBIK_Q0[idLEB] = -130.*D2R;

    WBIK_Q0[idRWY] = 0.*D2R;
    WBIK_Q0[idLWY] = 0.*D2R;

    WBIK_Q0[idRWP] = 20.*D2R;
    WBIK_Q0[idLWP] = 20.*D2R;
    WBIK_Q0[idRWY2] = 0;//20.*D2R;
    WBIK_Q0[idLWY2] = 0;//20.*D2R;
    WBIK_Q0[idWST] = 0;//-180*D2R;//20.*D2R;

    for(int i=0; i<=LAR; i++)
    {
    WalkReadyRef[i] = jCon->Joints[i]->RefAngleCurrent;
    printf("%f\n",WalkReadyRef[i]);
    }

}
void WBIK()
{
    Kirk_Control();
    des_pCOM_3x1[0] = userData->WalkReadyCOM[0] - Del_PC_X_DSP_XZMP_CON*0.001- Del_PC_X_SSP_XZMP_CON*0.001;
    des_pCOM_3x1[1] = userData->WalkReadyCOM[1] - Del_PC_Y_DSP_YZMP_CON*0.001- Del_PC_Y_SSP_YZMP_CON*0.001;
    des_pCOM_3x1[2] = userData->WalkReadyCOM[2];

    des_qPEL_4x1[0] = 1.;
    des_qPEL_4x1[1] = 0.;
    des_qPEL_4x1[2] = 0.;
    des_qPEL_4x1[3] = 0.;

    des_pRF_3x1[0] = 0.;
    des_pRF_3x1[1] = -kine_drc_hubo4.L_PEL2PEL/2.;//-0.13;//-kine_drc_hubo.L_PEL2PEL/2;//-0.135;//
    des_pRF_3x1[2] = 0.0;//0.05

    des_qRF_4x1[0] = 1.;
    des_qRF_4x1[1] = 0.;
    des_qRF_4x1[2] = 0.;
    des_qRF_4x1[3] = 0.;

    des_pLF_3x1[0] = 0.;
    des_pLF_3x1[1] = kine_drc_hubo4.L_PEL2PEL/2.;//0.13;//kine_drc_hubo.L_PEL2PEL/2;//0.135;//
    des_pLF_3x1[2] = 0.;

    des_qLF_4x1[0] = 1.;
    des_qLF_4x1[1] = 0.;
    des_qLF_4x1[2] = 0.;
    des_qLF_4x1[3] = 0.;

    memcpy(WBIK_Q0,WBIK_Q,34*sizeof(double));

    Qub[idRSP] = 40.*D2R;
    Qub[idLSP] = 40.*D2R;

    Qub[idRSR] = (10.)*D2R;
    Qub[idLSR] = (-10.)*D2R;

    Qub[idRSY] = 0.*D2R;
    Qub[idLSY] = 0.*D2R;

    Qub[idREB] = (-130. )*D2R;
    Qub[idLEB] = (-130. )*D2R;

    Qub[idRWY] = 0.*D2R;
    Qub[idLWY] = 0.*D2R;

    Qub[idRWP] = 20.*D2R;
    Qub[idLWP] = 20.*D2R;

    WBIK_Q0[idRHY] = 0.;
    WBIK_Q0[idRHR] = -2.78*D2R;
    WBIK_Q0[idRHP] = -43.9*D2R;
    WBIK_Q0[idRKN] = 80.58*D2R; //77.4*D2R;
    WBIK_Q0[idRAP] = -36.68*D2R;
    WBIK_Q0[idRAR] = 2.78*D2R;

    WBIK_Q0[idLHY] = 0.;
    WBIK_Q0[idLHR] = 2.78*D2R;
    WBIK_Q0[idLHP] = -43.9*D2R;
    WBIK_Q0[idLKN] = 80.58*D2R;//77.4*D2R;
    WBIK_Q0[idLAP] = -36.68*D2R;
    WBIK_Q0[idLAR] = -2.78*D2R;
    WBIK_Q0[idRSP] = 40.*D2R;
    WBIK_Q0[idLSP] = 40.*D2R;

    WBIK_Q0[idRSR] = 10.*D2R;
    WBIK_Q0[idLSR] = -10.*D2R;

    WBIK_Q0[idRSY] = 0.*D2R;
    WBIK_Q0[idLSY] = 0.*D2R;

    WBIK_Q0[idREB] = -130.*D2R;
    WBIK_Q0[idLEB] = -130.*D2R;

    WBIK_Q0[idRWY] = 0.*D2R;
    WBIK_Q0[idLWY] = 0.*D2R;

    WBIK_Q0[idRWP] = 20.*D2R;
    WBIK_Q0[idLWP] = 20.*D2R;
    WBIK_Q0[idRWY2] = 0;//20.*D2R;
    WBIK_Q0[idLWY2] = 0;//20.*D2R;
    WBIK_Q0[idWST] = 0;//-180*D2R;//20.*D2R;

    kine_drc_hubo4.IK_LowerBody_Global(WBIK_Q0,Qub,des_pCOM_3x1, des_qPEL_4x1, des_pRF_3x1, des_qRF_4x1, des_pLF_3x1, des_qLF_4x1,WBIK_Q);
    for(int i=0; i<=LAR; i++)
    {
        cCon->AddJoints[i]->RefAngleCurrent = -WalkReadyRef[i]+WBIK_Q[i+7]*R2D;
    }
}

void get_zmp2()
{
        // ZMP 읽기 //
        double M_LF[3],M_RF[3],M_LF_Global[3],M_RF_Global[3],F_LF[3],F_RF[3],F_LF_Global[3],F_RF_Global[3];
        double pCenter[3],qCenter[4],qCenter_bar[4];
        double zmp[3],zmp_local[3],zmp_ref_local[3];

        // Foot Center in Global Coord.
        pCenter[0] = (des_pRF_3x1[0] + des_pLF_3x1[0])/2.;
        pCenter[1] = (des_pRF_3x1[1] + des_pLF_3x1[1])/2.;
        pCenter[2] = (des_pRF_3x1[2] + des_pLF_3x1[2])/2.;

//        one_cos_orientation2(0.5,des_qRF_4x1,des_qLF_4x1,0,1,qCenter);
        qtRZ(0.,qCenter);


        if(sharedData->FT[RAFT].Fz + sharedData->FT[LAFT].Fz > 50.)
        {
            M_LF[0] =  sharedData->FT[LAFT].Mx;
            M_LF[1] =  sharedData->FT[LAFT].My;
            M_LF[2] =  sharedData->FT[LAFT].Mz;

            QTtransform(des_qLF_4x1,M_LF,M_LF_Global);

            M_RF[0] =  sharedData->FT[RAFT].Mx;
            M_RF[1] =  sharedData->FT[RAFT].My;
            M_RF[2] =  sharedData->FT[RAFT].Mz;

            QTtransform(des_qRF_4x1,M_RF,M_RF_Global);

            F_LF[0] = sharedData->FT[LAFT].Fx;
            F_LF[1] = sharedData->FT[LAFT].Fy;
            F_LF[2] = sharedData->FT[LAFT].Fz;

            QTtransform(des_qLF_4x1,F_LF,F_LF_Global);

            F_RF[0] = sharedData->FT[RAFT].Fx;
            F_RF[1] = sharedData->FT[RAFT].Fy;
            F_RF[2] = sharedData->FT[RAFT].Fz;

            QTtransform(des_qRF_4x1,F_RF,F_RF_Global);

             double temp1[3],temp2[3],temp3[3],temp4[3];

             diff_vv(des_pRF_3x1,3,pCenter,temp1);// (despRF - pCenter)
             diff_vv(des_pLF_3x1,3,pCenter,temp2);// (despLF - pCenter)

             cross(1,temp1,F_RF_Global,temp3);// (despRF - pCenter)x(F_RF_Global)
             cross(1,temp2,F_LF_Global,temp4);// (despLF - pCenter)x(F_LF_Global)

             sum_vv(temp3,3,temp4,temp3); // (despRF - pCenter)x(F_RF_Global) + (despLF - pCenter)x(F_LF_Global)
             sum_vv(temp3,3,M_RF_Global,temp3); // (despRF - pCenter)x(F_RF_Global) + (despLF - pCenter)x(F_LF_Global) + M_RF_Global
             sum_vv(temp3,3,M_LF_Global,temp3); // (despRF - pCenter)x(F_RF_Global) + (despLF - pCenter)x(F_LF_Global) + M_RF_Global + M_LF_Global

             zmp[0] = (-temp3[1])/(F_RF_Global[2] + F_LF_Global[2]) + pCenter[0];
             zmp[1] = (temp3[0])/(F_RF_Global[2] + F_LF_Global[2]) + pCenter[1];
             zmp[2] = 0.;

             diff_vv(zmp,3,pCenter,temp1); // zmp - pCenter

             qCenter_bar[0] = qCenter[0];
             qCenter_bar[1] = -qCenter[1];
             qCenter_bar[2] = -qCenter[2];
             qCenter_bar[3] = -qCenter[3];

             QTtransform(qCenter_bar, temp1, zmp_local); // qCenter_bar*(zmp-pCenter)

             temp2[0] = GLOBAL_ZMP_REF_X;
             temp2[1] = GLOBAL_ZMP_REF_Y;
             temp2[2] = 0;
             diff_vv(temp2,3,pCenter,temp1);
             QTtransform(qCenter_bar, temp1, zmp_ref_local);


            X_ZMP_Local = 1000.*zmp_local[0];
            Y_ZMP_Local = 1000.*zmp_local[1];

            X_ZMP_Global = 1000.*zmp[0];
            Y_ZMP_Global = 1000.*zmp[1];

            X_ZMP_REF_Local = zmp_ref_local[0];
            Y_ZMP_REF_Local = zmp_ref_local[1];

            X_ZMP_REF_Global = GLOBAL_ZMP_REF_X;
            Y_ZMP_REF_Global = GLOBAL_ZMP_REF_Y;


//            X_ZMP_n =(1.f-2.f*PI*7.0*(double)RT_TIMER_PERIOD_MS*0.001f)*X_ZMP_n + 2.f*PI*7.0*(double)RT_TIMER_PERIOD_MS*0.001f*X_ZMP;
//            Y_ZMP_n =(1.f-2.f*PI*7.0*(double)RT_TIMER_PERIOD_MS*0.001f)*Y_ZMP_n + 2.f*PI*7.0*(double)RT_TIMER_PERIOD_MS*0.001f*Y_ZMP;
        }
}
void Kirk_Control()
{
    ZMP_intergral_control();
    final_gain_DSP_ZMP_CON = 0.5*(1-cos(PI*CNT_final_gain_DSP_ZMP_CON/40));

    Del_PC_X_DSP_XZMP_CON = final_gain_DSP_ZMP_CON*kirkZMPCon_XP2(-Del_PC_X_DSP_XZMP_CON, X_ZMP_Local - (X_ZMP_REF_Local)*1000 , 1);
    Del_PC_Y_DSP_YZMP_CON = final_gain_DSP_ZMP_CON*kirkZMPCon_YP2(-Del_PC_Y_DSP_YZMP_CON, Y_ZMP_Local - (Y_ZMP_REF_Local)*1000 , 1);

    LPF_Del_PC_X_DSP_XZMP_CON = (1.f-2.f*PI*3.0*(double)RT_TIMER_PERIOD_MS*0.001f)*LPF_Del_PC_X_DSP_XZMP_CON + 2.f*PI*3.0*(double)RT_TIMER_PERIOD_MS*0.001f*Del_PC_X_DSP_XZMP_CON;
    LPF_Del_PC_Y_DSP_YZMP_CON = (1.f-2.f*PI*3.0*(double)RT_TIMER_PERIOD_MS*0.001f)*LPF_Del_PC_Y_DSP_YZMP_CON + 2.f*PI*3.0*(double)RT_TIMER_PERIOD_MS*0.001f*Del_PC_Y_DSP_YZMP_CON;

    if(CNT_final_gain_DSP_ZMP_CON < 40 && Control_OnOff_Flag == true) CNT_final_gain_DSP_ZMP_CON++;
    else if(CNT_final_gain_DSP_ZMP_CON < 80 && Control_OnOff_Flag == false) CNT_final_gain_DSP_ZMP_CON++;

    Old_Del_PC_X_DSP_XZMP_CON = Del_PC_X_DSP_XZMP_CON;
    Old_Del_PC_Y_DSP_YZMP_CON = Del_PC_Y_DSP_YZMP_CON;
    Old_Del_PC_X_DSP_XZMP_CON2 = Del_PC_X_DSP_XZMP_CON;
    Old_Del_PC_Y_DSP_YZMP_CON2 = Del_PC_Y_DSP_YZMP_CON;
    Old_I_ZMP_CON_X = I_ZMP_CON_X;
    Old_I_ZMP_CON_Y = I_ZMP_CON_Y;

//    printf("",final_gain_DSP_ZMP_CON,Del_PC_X_DSP_XZMP_CON)
}
void Kirk_Control_ssp()
{
    final_gain_SSP_ZMP_CON = 0.5*(1-cos(PI*CNT_final_gain_SSP_ZMP_CON/40));

    Del_PC_X_SSP_XZMP_CON = final_gain_SSP_ZMP_CON*kirkZMPCon_XP1(-Del_PC_X_SSP_XZMP_CON, X_ZMP_Local - (X_ZMP_REF_Local)*1000 , 1);
    Del_PC_Y_SSP_YZMP_CON = final_gain_SSP_ZMP_CON*kirkZMPCon_YP1(-Del_PC_Y_SSP_YZMP_CON, Y_ZMP_Local - (Y_ZMP_REF_Local)*1000 , 1);
    //printf(" Del_PC_Y_SSP_YZMP_CON = %f\n",Del_PC_Y_SSP_YZMP_CON);
    LPF_Del_PC_X_SSP_XZMP_CON = (1.f-2.f*PI*3.0*(double)RT_TIMER_PERIOD_MS*0.001f)*LPF_Del_PC_X_SSP_XZMP_CON + 2.f*PI*3.0*(double)RT_TIMER_PERIOD_MS*0.001f*Del_PC_X_SSP_XZMP_CON;
    LPF_Del_PC_Y_SSP_YZMP_CON = (1.f-2.f*PI*3.0*(double)RT_TIMER_PERIOD_MS*0.001f)*LPF_Del_PC_Y_SSP_YZMP_CON + 2.f*PI*3.0*(double)RT_TIMER_PERIOD_MS*0.001f*Del_PC_Y_SSP_YZMP_CON;

    if(CNT_final_gain_SSP_ZMP_CON < 40) CNT_final_gain_SSP_ZMP_CON++;

    Old_Del_PC_X_SSP_XZMP_CON = Del_PC_X_SSP_XZMP_CON;
    Old_Del_PC_Y_SSP_YZMP_CON = Del_PC_Y_SSP_YZMP_CON;
}
void ZMP_intergral_control()
{
    I_ZMP_CON_X += -0.001*(0.001*X_ZMP_Local - (X_ZMP_REF_Local ));//-X_ZMP_n_OFFSET_BP
    I_ZMP_CON_Y += -0.001*(0.001*Y_ZMP_Local - (Y_ZMP_REF_Local ));

    if(I_ZMP_CON_X > 0.04)I_ZMP_CON_X=0.04;
    else if(I_ZMP_CON_X < -0.04)I_ZMP_CON_X=-0.04;
    if(I_ZMP_CON_Y > 0.04)I_ZMP_CON_Y=0.04;
    else if(I_ZMP_CON_Y < -0.04)I_ZMP_CON_Y=-0.04;

}
double kirkZMPCon_YP1(double u, double ZMP, int zero)
{
    int i;
    const double A[2][2] = {{0,1},{-14.9504066667389	-0.616370808678501}};
    const double B[2] = {0,35.2914483893102};
    const double C[2] = {1.70719953272619,0.0382262996941896};
    const double D = -2.18871734964897;
//    const double Kg[2] = {1.87183004235299,0.492573413240444};
    const double Kg[2] = {0.596733608123605,0.322560555343192};
//    const double Og[2] = {10.2903310784662,47.5060572719318};
    const double Og[2] = {8.40760843975646,26.9490900208963};

    static double x_old[2], x_new[2], Temp_1[2], Temp_2[2], Temp_3[2];
    double y;

    Temp_1[0] = A[0][0]*x_old[0] + A[0][1]*x_old[1];
    Temp_1[1] = A[1][0]*x_old[0] + A[1][1]*x_old[1];

    Temp_2[0] = B[0]*u;
    Temp_2[1] = B[1]*u;

    Temp_3[0] = Og[0]*(ZMP - C[0]*x_old[0] - C[1]*x_old[1] -D*u);
    Temp_3[1] = Og[1]*(ZMP - C[0]*x_old[0] - C[1]*x_old[1] -D*u);

    if(zero == 0)
    {
        for(i=0; i<2; i++)
        {
            x_old[i] = 0.;
            x_new[i] = 0.;
            Temp_1[i] = 0.;
            Temp_2[i] = 0.;
            Temp_3[i] = 0.;
        }
    }

    x_new[0] = x_old[0] + DEL_T*(Temp_1[0] + Temp_2[0] + Temp_3[0]);
    x_new[1] = x_old[1] + DEL_T*(Temp_1[1] + Temp_2[1] + Temp_3[1]);

    y = Kg[0]*x_new[0] + Kg[1]*x_new[1];

    x_old[0] = x_new[0];
    x_old[1] = x_new[1];

    if(y > 60.0) y = 60.0;
    else if(y < -60.0) y = -60.0;

    return y ;
}
// ----------------------------------------------------------------------------------- //
double kirkZMPCon_XP1(double u, double ZMP, int zero)
{
    int i;
    const double A[2][2] = {{0,1},{-42.8368246575059,-0.719099276791585}};
    const double B[2] = {0,71.0432663261910};
    const double C[2] = {3.43666912554807,0.0445973496432212};
    const double D = -4.40598605839496;
    const double Kg[2] = {0.537322920475309,	0.243244738267863};
    const double Og[2] = {5.40216194635022,	16.0426024573112};

    static double x_old[2], x_new[2], Temp_1[2], Temp_2[2], Temp_3[2];
    double y;

    Temp_1[0] = A[0][0]*x_old[0] + A[0][1]*x_old[1];
    Temp_1[1] = A[1][0]*x_old[0] + A[1][1]*x_old[1];

    Temp_2[0] = B[0]*u;
    Temp_2[1] = B[1]*u;

    Temp_3[0] = Og[0]*(ZMP - C[0]*x_old[0] - C[1]*x_old[1] -D*u);
    Temp_3[1] = Og[1]*(ZMP - C[0]*x_old[0] - C[1]*x_old[1] -D*u);

    if(zero == 0)
    {
        for(i=0; i<2; i++)
        {
            x_old[i] = 0.;
            x_new[i] = 0.;
            Temp_1[i] = 0.;
            Temp_2[i] = 0.;
            Temp_3[i] = 0.;
        }
    }

    x_new[0] = x_old[0] + DEL_T*(Temp_1[0] + Temp_2[0] + Temp_3[0]);
    x_new[1] = x_old[1] + DEL_T*(Temp_1[1] + Temp_2[1] + Temp_3[1]);

    y = Kg[0]*x_new[0] + Kg[1]*x_new[1];

    x_old[0] = x_new[0];
    x_old[1] = x_new[1];

    if(y > 60.0) y = 60.0;
    else if(y < -60.0) y = -60.0;

    return y ;
}
double kirkZMPCon_XP2(double u, double ZMP, int zero)
{
    int i;
    const double A[2][2] = {{0.000000000000, 1.000000000000}, {-93.44,-1.0204}};
    const double B[2] = {0,153.5062};
    const double C[2] = {5.3672,0.0510};
    const double D = -7.6675;
    const double Kg[2] = {-0.1917,0.0976};
    const double Og[2] = {3.5220,1.4988};
//    const double A[2][2] = {{0.000000000000, 1.000000000000}, {-93.44,-0.8218}};
//    const double B[2] = {0,135.9192};
//    const double C[2] = {6.5750,0.0510};
//    const double D = -8.4295;
//    const double Kg[2] = {-0.2165,0.1117};
//    const double Og[2] = {2.9066,1.3227};
//    const double A[2][2] = {{0.000000000000, 1.000000000000}, {-121.509441687773,-0.686900801711364}};
//    const double B[2] = {0,171.905595852174};
//    const double C[2] = {8.31581491568207,0.0426004533905397};
//    const double D = -10.6613011739514;
//    const double Kg[2] = {-0.561351369682899,0.0541756604962248};
//    const double Og[2] = {1.87687421562180,-6.91634420265651};

    static double x_old[2], x_new[2], Temp_1[2], Temp_2[2], Temp_3[2];
    double y;

    Temp_1[0] = A[0][0]*x_old[0] + A[0][1]*x_old[1];
    Temp_1[1] = A[1][0]*x_old[0] + A[1][1]*x_old[1];

    Temp_2[0] = B[0]*u;
    Temp_2[1] = B[1]*u;

    Temp_3[0] = Og[0]*(ZMP - C[0]*x_old[0] - C[1]*x_old[1] -D*u);
    Temp_3[1] = Og[1]*(ZMP - C[0]*x_old[0] - C[1]*x_old[1] -D*u);

    if(zero == 0)
    {
        for(i=0; i<2; i++)
        {
            x_old[i] = 0.;
            x_new[i] = 0.;
            Temp_1[i] = 0.;
            Temp_2[i] = 0.;
            Temp_3[i] = 0.;
        }
    }

    x_new[0] = x_old[0] + DEL_T*(Temp_1[0] + Temp_2[0] + Temp_3[0]);
    x_new[1] = x_old[1] + DEL_T*(Temp_1[1] + Temp_2[1] + Temp_3[1]);

    y = Kg[0]*x_new[0] + Kg[1]*x_new[1];

    x_old[0] = x_new[0];
    x_old[1] = x_new[1];

    if(y > 60.0) y = 60.0;
    else if(y < -60.0) y = -60.0;

    return y ;
}
// ----------------------------------------------------------------------------------- //
double kirkZMPCon_YP2(double u, double ZMP, int zero)
{
    int i;
//    const double A[2][2] = {{0.000000000000, 1.000000000000}, {-93.44,-1.2327}};
//    const double B[2] = {0,135.9192};
//    const double C[2] = {6.5750,0.0765};
//    const double D = -8.4295;
//    const double Kg[2] = {-0.0915,0.1234};
//    const double Og[2] = {2.8458,0.7336};
//    const double Kg[2] = {-0.4152,0.0792};
//

//    const double Og[2] = {2.8405,1.1907};
//

    const double A[2][2] = {{0.000000000000, 1.000000000000}, {-93.44,-1.5306}};
    const double B[2] = {0,153.5062};
    const double C[2] = {5.3672,0.0765};
    const double D = -7.6675;

    const double Kg[2] = {-0.0809742090339354,	0.107288107510564};
    const double Og[2] = {3.43082537995055,0.723663240519607};

//    const double A[2][2] = {{0.000000000000, 1.000000000000}, {-119.405421865731,-0.441579086814448}};
//    const double B[2] = {0,169.208134541865};
//    const double C[2] = {8.18532708084719,0.0273860057510612};
//    const double D = -10.4940090780092;
//    const double Kg[2] = {-0.552014961447503,0.0564891335695250};
//    const double Og[2] = {3.56807911031747,12.8724994839785};

    static double x_old[2], x_new[2], Temp_1[2], Temp_2[2], Temp_3[2];
    double y;

    Temp_1[0] = A[0][0]*x_old[0] + A[0][1]*x_old[1];
    Temp_1[1] = A[1][0]*x_old[0] + A[1][1]*x_old[1];

    Temp_2[0] = B[0]*u;
    Temp_2[1] = B[1]*u;

    Temp_3[0] = Og[0]*(ZMP - C[0]*x_old[0] - C[1]*x_old[1] -D*u);
    Temp_3[1] = Og[1]*(ZMP - C[0]*x_old[0] - C[1]*x_old[1] -D*u);

    if(zero == 0)
    {
        for(i=0; i<2; i++)
        {
            x_old[i] = 0.;
            x_new[i] = 0.;
            Temp_1[i] = 0.;
            Temp_2[i] = 0.;
            Temp_3[i] = 0.;
        }
    }

    x_new[0] = x_old[0] + DEL_T*(Temp_1[0] + Temp_2[0] + Temp_3[0]);
    x_new[1] = x_old[1] + DEL_T*(Temp_1[1] + Temp_2[1] + Temp_3[1]);

   // y = Kg[0]*(atan2(GLOBAL_Y_LIPM_n,des_pCOM_3x1[Zdir])*1000-x_new[0]) + Kg[1]*(atan2(GLOBAL_Y_LIPM_d_n,des_pCOM_3x1[Zdir])*1000-x_new[1]);
    y = Kg[0]*(x_new[0]) + Kg[1]*(x_new[1]);

    x_old[0] = x_new[0];
    x_old[1] = x_new[1];

    if(y > 80.0) y = 80.0;
    else if(y < -80.0) y = -80.0;

    return y ;
}

void WBIK_PARA_CHANGE(){

///////////////////////////////////////////////////////////////////
    // Original without the head
//        kine_drc_hubo4.C_Torso[0] = 0.000941-0.065;
//        kine_drc_hubo4.m_Torso = 24.98723;
//        kine_drc_hubo4.m_RightWrist = 4.5;
//        kine_drc_hubo4.m_LeftWrist = 4.5;
//        kine_drc_hubo4.m_Pelvis = 11.867886 +2.;
//        kine_drc_hubo4.C_Torso[1] =0;
//        kine_drc_hubo4.L_FOOT = 0.113;
//        kine_drc_hubo4.iter_limit = 100;
//        kine_drc_hubo4.converge_criterium = 1e-6;




    // Tuned value with the head
//    kine_drc_hubo4.C_Torso[0] = 0.000941-0.065;

    float distributed_mass = 3.3/14;

    kine_drc_hubo4.L_ANKLE = 0.001;
    kine_drc_hubo4.L_ELB_OFFSET = 0.022;
    kine_drc_hubo4.L_FOOT = 0.095;
    kine_drc_hubo4.L_HAND = 0.08;
    kine_drc_hubo4.L_LOWER_ARM = 0.1222+0.04041;
    kine_drc_hubo4.L_LOWER_LEG = 0.3;
    kine_drc_hubo4.L_PC2WST = 0.17287;
    kine_drc_hubo4.L_PEL2PEL = 0.0885*2;
    kine_drc_hubo4.L_SHOULDER2SHOULDER = 0.2145*2;
    kine_drc_hubo4.L_UPPER_ARM = 0.17914;
    kine_drc_hubo4.L_UPPER_LEG = 0.3;
    kine_drc_hubo4.L_WST2SHOULDER = 0.196;

    kine_drc_hubo4.m_LeftFoot=2.6626 + distributed_mass;
    kine_drc_hubo4.C_LeftFoot[0]=0.0216;
    kine_drc_hubo4.C_LeftFoot[1]=0.0014;
    kine_drc_hubo4.C_LeftFoot[2]=-0.0160;

    kine_drc_hubo4.m_LeftHand=0.3 + distributed_mass;
    kine_drc_hubo4.C_LeftHand[0]=0.0033;
    kine_drc_hubo4.C_LeftHand[1]=0.0015;
    kine_drc_hubo4.C_LeftHand[2]=-0.0635;

    kine_drc_hubo4.m_LeftLowerArm=0.5542 + distributed_mass;
    kine_drc_hubo4.C_LeftLowerArm[0]=0.0003;
    kine_drc_hubo4.C_LeftLowerArm[1]=0.0006;
    kine_drc_hubo4.C_LeftLowerArm[2]=-0.047;

    kine_drc_hubo4.m_LeftLowerLeg=1.96 + distributed_mass;
    kine_drc_hubo4.C_LeftLowerLeg[0]=0.0146;
    kine_drc_hubo4.C_LeftLowerLeg[1]=0.0146;
    kine_drc_hubo4.C_LeftLowerLeg[2]=-0.1845;

    kine_drc_hubo4.m_LeftUpperArm=2.3147 + distributed_mass;
    kine_drc_hubo4.C_LeftUpperArm[0]=0.0062;
    kine_drc_hubo4.C_LeftUpperArm[1]=-0.0178;
    kine_drc_hubo4.C_LeftUpperArm[2]=-0.0451;

    kine_drc_hubo4.m_LeftUpperLeg=6.3512 + distributed_mass;
    kine_drc_hubo4.C_LeftUpperLeg[0]=0.0175;
    kine_drc_hubo4.C_LeftUpperLeg[1]=0.0099;
    kine_drc_hubo4.C_LeftUpperLeg[2]=-0.0995;

    kine_drc_hubo4.m_RightFoot=2.6626 + distributed_mass;
    kine_drc_hubo4.C_RightFoot[0]=0.0216;
    kine_drc_hubo4.C_RightFoot[1]=-0.0014;
    kine_drc_hubo4.C_RightFoot[2]=-0.0160;

    kine_drc_hubo4.m_RightHand=0.3 + distributed_mass;
    kine_drc_hubo4.C_RightHand[0]=0.0033;
    kine_drc_hubo4.C_RightHand[1]=-0.0015;
    kine_drc_hubo4.C_RightHand[2]=-0.0635;

    kine_drc_hubo4.m_RightLowerArm=0.5542 + distributed_mass;
    kine_drc_hubo4.C_RightLowerArm[0]=0.0003;
    kine_drc_hubo4.C_RightLowerArm[1]=-0.0006;
    kine_drc_hubo4.C_RightLowerArm[2]=-0.047;

    kine_drc_hubo4.m_RightLowerLeg=1.96 + distributed_mass;
    kine_drc_hubo4.C_RightLowerLeg[0]=0.0146;
    kine_drc_hubo4.C_RightLowerLeg[1]=-0.0146;
    kine_drc_hubo4.C_RightLowerLeg[2]=-0.1845;

    kine_drc_hubo4.m_RightUpperArm=2.3147 + distributed_mass;
    kine_drc_hubo4.C_RightUpperArm[0]=0.0062;
    kine_drc_hubo4.C_RightUpperArm[1]=0.0178;
    kine_drc_hubo4.C_RightUpperArm[2]=-0.0451;

    kine_drc_hubo4.m_RightUpperLeg=6.3512 + distributed_mass;
    kine_drc_hubo4.C_RightUpperLeg[0]=0.0175;
    kine_drc_hubo4.C_RightUpperLeg[1]=-0.0099;
    kine_drc_hubo4.C_RightUpperLeg[2]=-0.0995;

    kine_drc_hubo4.m_Pelvis=3.88 + distributed_mass;
    kine_drc_hubo4.C_Pelvis[0]=-0.0119;
    kine_drc_hubo4.C_Pelvis[1]=0;
    kine_drc_hubo4.C_Pelvis[2]=0.1323;

    kine_drc_hubo4.m_Torso=7.3 + 1.5 + distributed_mass;
    kine_drc_hubo4.C_Torso[0]=-0.0115;
    kine_drc_hubo4.C_Torso[1]=0.0;
    kine_drc_hubo4.C_Torso[2]=0.1347+0.01;

    kine_drc_hubo4.m_LeftWrist = 1.;
    kine_drc_hubo4.m_RightWrist = 1.;
    // weight of Torso is increased ( Torso + Head)
   /*ine_drc_hubo4.m_Torso = 24.98723 + 0.0; // kg
    kine_drc_hubo4.m_RightWrist = 4.5;
    kine_drc_hubo4.m_LeftWrist = 4.5;
    kine_drc_hubo4.m_Pelvis = 11.867886 +2.;
    kine_drc_hubo4.C_Torso[1] =0;
    kine_drc_hubo4.L_FOOT = 0.113;
    kine_drc_hubo4.iter_limit = 100;
    kine_drc_hubo4.converge_criterium = 1e-6;*/

///////////////////////////////////////////////////////////////////


}

