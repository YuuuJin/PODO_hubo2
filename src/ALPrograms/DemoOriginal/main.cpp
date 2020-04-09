

#include "BasicFiles/BasicSetting.h"
#include "BasicFiles/TaskMotion.h"

#define BIT_PC 89 //90

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
int ownwershinp_state =1;

int hubo_vision_on=DISABLE;

int     motionLength = 0;
float   stepLength = 0.0f;



int     scidemo_num;
float   ringData[RING_SIZE][31];
int     ringIndexFront;
int     ringIndexBack;
float   localJoint[NO_OF_JOINTS];
float   localQpel[4];
float   localDeltaPCz;
float   WR_PelcZ;
int     fileReadFlag;
FILE*   fpMocap;


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
int Control_OnOff_Flag =0;

const double DEL_T = 0.005;
//const double    OFFSET_ELB = -20.0;
//const double    OFFSET_RSR = -15.0;
//const double    OFFSET_LSR = 15.0;
double curRSR,curLSR;
double  final_gain_DSP_ZMP_CON = 0., final_gain_SSP_ZMP_CON = 0.;
double  final_gain_DSP_ZMP_CON2 = 0., final_gain_SSP_ZMP_CON2 = 0.;
unsigned int CNT_final_gain_DSP_ZMP_CON = 0,  CNT_final_gain_SSP_ZMP_CON = 0;
unsigned int CNT_final_gain_DSP_ZMP_CON2 = 0,  CNT_final_gain_SSP_ZMP_CON2 = 0;
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


extern int motion_time_check;
extern int motion_sequence_loop;

enum FTNAME{
    RAFT = 0,
    LAFT
};


// Dave data
void JW_save();
void JW_save2();
#define ROW_data_debug 20000
#define COL_data_debug 100
FILE *fp;
FILE *fp2;
double   JW_Data_Debug[COL_data_debug][ROW_data_debug];
double   JW_Data_Debug2[COL_data_debug][ROW_data_debug];
int sJW_Data_Debug_Index;
int sJW_Data_Debug_Index2;

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
    sprintf(__AL_NAME, "DemoOriginal");


    CheckArguments(argc, argv);
    if(PODO_NO == -1){
        FILE_LOG(logERROR) << "Please check the AL Number";
        FILE_LOG(logERROR) << "Terminate this AL..";
        //return 0;
        PODO_NO = 3;
    }



    // Initialize RBCore -----------------------------------
    if(RBInitialize() == false)
        __IS_WORKING = false;

    WholebodyInitialize();
    for(int i=0; i<=LAR; i++)
    {
        cCon->AddJoints[i]->RefAngleCurrent = 0.;
    }
    mCon = new TaskMotion(sharedData, jCon);

    jCon->RefreshToCurrentReference();
    mCon->RefreshToCurrentReference();

    if(RBStartThread() == false)
        __IS_WORKING = false;


//    jCon->RefreshToCurrentReference();
    Control_OnOff_Flag = 0;
//    mCon->ResetGlobalCoord(0);
//    mCon->RefreshToCurrentReference();

//    jCon->SetAllMotionOwner();



    //jCon->SetMotionOwner(0);
    while(__IS_WORKING){
        usleep(100*1000);

        switch(sharedData->COMMAND[PODO_NO].USER_COMMAND){
        case SCI_DEMO:
            FILE_LOG(logSUCCESS) << "SCI_DEMO: " << sharedData->COMMAND[PODO_NO].USER_PARA_INT[3];

            if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[3] == 1)
            {
                StartWB(0);
                mCon->sciTimeCnt = 0;
                mCon->sciMotionSelector = SCI_DEMO;
                mCon->TaskMotionSelector = SCI_DEMO;
                scidemo_num=1;
            }
            else if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[3] == 2)
            {
                printf("Hello I am here...!!!\n");
                StartWB(0);
                sprintf(mCon->filename, "../share/WB_Data/New_UB_ScienceMuseum_2.txt");
                ReadTextData(mCon->filename);
                mCon->sciTimeCnt = 0;
                mCon->sciMotionSelector = SCI_DEMO;
                mCon->TaskMotionSelector = SCI_DEMO;
                scidemo_num=2;
            }
            else if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[3] == 201)
            {
                sleep(1);
                StartWB(0);
                mCon->sciTimeCnt = 0;
                mCon->sciMotionSelector = SCI_DEMO;
                mCon->TaskMotionSelector = SCI_DEMO;
                scidemo_num=201;
            }
            else if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[3] == 3)
            {
                StartWB(0);
                sprintf(mCon->filename, "../share/WB_Data/New_UB_ScienceMuseum_12.txt");
                ReadTextData(mCon->filename);
                mCon->sciTimeCnt = 0;
                mCon->sciMotionSelector = SCI_DEMO;
                mCon->TaskMotionSelector = SCI_DEMO;
                scidemo_num=3;
            }
            else if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[3] == 4)
            {
                StartWB(0);
                sprintf(mCon->filename, "../share/WB_Data/New_UB_ScienceMuseum_5.txt");
                ReadTextData(mCon->filename);
                mCon->sciTimeCnt = 0;
                mCon->sciMotionSelector = SCI_DEMO;
                mCon->TaskMotionSelector = SCI_DEMO;
                scidemo_num=4;
            }
            else if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[3] == 5)
            {
                StartWB(0);
                sprintf(mCon->filename, "../share/WB_Data/New_UB_ScienceMuseum_8.txt");
                ReadTextData(mCon->filename);
                mCon->sciTimeCnt = 0;
                mCon->sciMotionSelector = SCI_DEMO;
                mCon->TaskMotionSelector = SCI_DEMO;
                scidemo_num=5;
            }
            else if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[3] == 6)
            {
                StartWB(0);
                mCon->sciTimeCnt = 0;
                mCon->sciMotionSelector = SCI_DEMO;
                mCon->TaskMotionSelector = SCI_DEMO;
                scidemo_num=6;
            }
            else if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[3] == 701)//vision on
            {
                hubo_vision_on=ENABLE;
                printf("Vision command activate\n");
                StartWB(0);
                mCon->sciTimeCnt = 0;
                mCon->sciMotionSelector = SCI_DEMO;
                mCon->TaskMotionSelector = SCI_DEMO;
                scidemo_num=701;
            }
            else if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[3] == 702)//vision off
            {
                hubo_vision_on=DISABLE;
                StartWB(0);
                mCon->sciTimeCnt = 0;
                mCon->sciMotionSelector = SCI_DEMO;
                mCon->TaskMotionSelector = SCI_DEMO;
                scidemo_num=702;
            }
            else if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[3] == 703)
            {
                StartWB(0);
                sprintf(mCon->filename, "../share/WB_Data/New_UB_ScienceMuseum_32.txt");
                ReadTextData(mCon->filename);
                mCon->sciTimeCnt = 0;
                mCon->sciMotionSelector = SCI_DEMO;
                mCon->TaskMotionSelector = SCI_DEMO;
                scidemo_num=703;
            }
            else if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[3] == 801)//ctrl off pos
            {
                StartWB(0);
                mCon->sciTimeCnt = 0;
                mCon->sciMotionSelector = SCI_DEMO;
                mCon->TaskMotionSelector = SCI_DEMO;
                scidemo_num=801;
            }
            else if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[3] == 802)//ctrl on pos
            {
                StartWB(0);
                mCon->sciTimeCnt = 0;
                mCon->sciMotionSelector = SCI_DEMO;
                mCon->TaskMotionSelector = SCI_DEMO;
                scidemo_num=802;
            }
            else if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[3] == 9)
            {
                StartWB(0);
                sprintf(mCon->filename, "../share/WB_Data/New_UB_ScienceMuseum_24.txt");
                ReadTextData(mCon->filename);
                mCon->sciTimeCnt = 0;
                mCon->sciMotionSelector = SCI_DEMO;
                mCon->TaskMotionSelector = SCI_DEMO;
                scidemo_num=9;
            }
            else if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[3] == 10)
            {
                StartWB(0);
                sprintf(mCon->filename, "../share/WB_Data/New_UB_ScienceMuseum_37.txt");
                ReadTextData(mCon->filename);
                mCon->sciTimeCnt = 0;
                mCon->sciMotionSelector = SCI_DEMO;
                mCon->TaskMotionSelector = SCI_DEMO;
                scidemo_num=10;
            }
            else if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[3] == 1001)
            {
                sleep(1);
                StartWB(0);
                mCon->sciTimeCnt = 0;
                mCon->sciMotionSelector = SCI_DEMO;
                mCon->TaskMotionSelector = SCI_DEMO;
                scidemo_num=1001;
            }
            else if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[3] == 11)
            {
                StartWB(0);
                sprintf(mCon->filename, "../share/WB_Data/New_UB_ScienceMuseum_26.txt");
                ReadTextData(mCon->filename);
                mCon->sciTimeCnt = 0;
                mCon->sciMotionSelector = SCI_DEMO;
                mCon->TaskMotionSelector = SCI_DEMO;
                scidemo_num=11;
            }
            else if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[3] == 12)
            {
                StartWB(0);
                sprintf(mCon->filename, "../share/WB_Data/New_UB_ScienceMuseum_18.txt");
                ReadTextData(mCon->filename);
                mCon->sciTimeCnt = 0;
                mCon->sciMotionSelector = SCI_DEMO;
                mCon->TaskMotionSelector = SCI_DEMO;
                scidemo_num=12;
            }
            else if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[3] == 13)
            {
                StartWB(0);
                mCon->sciTimeCnt = 0;
                mCon->sciMotionSelector = SCI_DEMO;
                mCon->TaskMotionSelector = SCI_DEMO;
                scidemo_num=13;
            }
            else if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[3] == 1401)
            {
                StartWB(0);
                mCon->sciTimeCnt = 0;
                mCon->sciMotionSelector = SCI_DEMO;
                mCon->TaskMotionSelector = SCI_DEMO;
                scidemo_num=1401;
            }
            else if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[3] == 1402)
            {
                StartWB(0);
                mCon->sciTimeCnt = 0;
                mCon->sciMotionSelector = SCI_DEMO;
                mCon->TaskMotionSelector = SCI_DEMO;
                scidemo_num=1402;
            }
            else if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[3] == 15)
            {
                StartWB(0);
                mCon->sciTimeCnt = 0;
                mCon->sciMotionSelector = SCI_DEMO;
                mCon->TaskMotionSelector = SCI_DEMO;
                scidemo_num=15;
            }
            else if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[3] == 16)
            {
                StartWB(0);
                sprintf(mCon->filename, "../share/WB_Data/New_UB_ScienceMuseum_32.txt");
                ReadTextData(mCon->filename);
                mCon->sciTimeCnt = 0;
                mCon->sciMotionSelector = SCI_DEMO;
                mCon->TaskMotionSelector = SCI_DEMO;
                scidemo_num=16;
            }
            else if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[3] == 17)
            {
                StartWB(0);
                sprintf(mCon->filename, "../share/WB_Data/New_UB_ScienceMuseum_24.txt");
                ReadTextData(mCon->filename);
                mCon->sciTimeCnt = 0;
                mCon->sciMotionSelector = SCI_DEMO;
                mCon->TaskMotionSelector = SCI_DEMO;
                scidemo_num=17;
            }
            else if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[3] == 1801)
            {
                StartWB(0);
                mCon->sciTimeCnt = 0;
                mCon->sciMotionSelector = SCI_DEMO;
                mCon->TaskMotionSelector = SCI_DEMO;
                scidemo_num=1801;
            }
            else if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[3] == 1802)
            {
                StartWB(0);
                mCon->sciTimeCnt = 0;
                mCon->sciMotionSelector = SCI_DEMO;
                mCon->TaskMotionSelector = SCI_DEMO;
                scidemo_num=1802;
            }
            else if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[3] == 1804)
            {
                StartWB(0);
                mCon->sciTimeCnt = 0;
                mCon->sciMotionSelector = SCI_DEMO;
                mCon->TaskMotionSelector = SCI_DEMO;
                scidemo_num=1804;
            }
            else if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[3] == 19)
            {
                StartWB(0);
                sprintf(mCon->filename, "../share/WB_Data/New_UB_ScienceMuseum_10.txt");
                ReadTextData(mCon->filename);
                mCon->sciTimeCnt = 0;
                mCon->sciMotionSelector = SCI_DEMO;
                mCon->TaskMotionSelector = SCI_DEMO;
                scidemo_num=19;
            }
            else if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[3] == 1901)
            {
                sleep(1);// finger motion delay
                StartWB(0);
                sprintf(mCon->filename, "../share/WB_Data/New_UB_ScienceMuseum_7.txt");
                ReadTextData(mCon->filename);
                mCon->sciTimeCnt = 0;
                mCon->sciMotionSelector = SCI_DEMO;
                mCon->TaskMotionSelector = SCI_DEMO;
                scidemo_num=1901;
            }
            else if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[3] == 1902)
            {
                sleep(1);
                StartWB(0);
                mCon->sciTimeCnt = 0;
                mCon->sciMotionSelector = SCI_DEMO;
                mCon->TaskMotionSelector = SCI_DEMO;
                scidemo_num=1902;
            }
            else if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[3] == 20)
            {
                StartWB(0);
                sprintf(mCon->filename, "../share/WB_Data/New_UB_ScienceMuseum_13.txt");
                ReadTextData(mCon->filename);
                mCon->sciTimeCnt = 0;
                mCon->sciMotionSelector = SCI_DEMO;
                mCon->TaskMotionSelector = SCI_DEMO;
                scidemo_num=20;
            }
            else if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[3] == 2001)
            {
                sleep(1);
                StartWB(0);
                mCon->sciTimeCnt = 0;
                mCon->sciMotionSelector = SCI_DEMO;
                mCon->TaskMotionSelector = SCI_DEMO;
                scidemo_num=2001;
            }
            else if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[3] == 21)
            {
                StartWB(0);
                mCon->sciTimeCnt = 0;
                mCon->sciMotionSelector = SCI_DEMO;
                mCon->TaskMotionSelector = SCI_DEMO;
                scidemo_num=21;
            }
            else if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[3] == 22)
            {
                StartWB(0);
                mCon->sciTimeCnt = 0;
                mCon->sciMotionSelector = SCI_DEMO;
                mCon->TaskMotionSelector = SCI_DEMO;
                scidemo_num=22;
            }
            else if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[3] == 23)
            {
                StartWB(0);
                mCon->sciTimeCnt = 0;
                mCon->sciMotionSelector = SCI_DEMO;
                mCon->TaskMotionSelector = SCI_DEMO;
                scidemo_num=23;
            }
            else if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[3] == 24)
            {
                StartWB(0);
                mCon->sciTimeCnt = 0;
                mCon->sciMotionSelector = SCI_DEMO;
                mCon->TaskMotionSelector = SCI_DEMO;
                scidemo_num=24;
            }
            else if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[3] == 25)
            {
                StartWB(0);
                sprintf(mCon->filename, "../share/WB_Data/New_UB_gangnam_style_5ms.txt");
                //sprintf(mCon->filename, "../share/WB_Data/New_UB_ScienceMuseum_42_5ms.txt");

                ReadTextData(mCon->filename);
                mCon->sciTimeCnt = 0;
                mCon->sciMotionSelector = SCI_DEMO;
                mCon->TaskMotionSelector = SCI_DEMO;
                scidemo_num=25;

            }
            else if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[3] == 26)
            {
                StartWB(0);
                mCon->sciTimeCnt = 0;
                mCon->sciMotionSelector = SCI_DEMO;
                mCon->TaskMotionSelector = SCI_DEMO;
                scidemo_num=26;

//                StartWB(0);
//                sprintf(mCon->filename, "../share/WB_Data/New_UB_ScienceMuseum_35.txt");
//                ReadTextData(mCon->filename);
//                mCon->sciTimeCnt = 0;
//                mCon->sciMotionSelector = SCI_DEMO;
//                mCon->TaskMotionSelector = SCI_DEMO;
//                scidemo_num=26;
            }
            // Scene 27 is walking
            else if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[3] == 28)
            {
                StartWB(0);
                //sprintf(mCon->filename, "../share/WB_Data/New_UB_ScienceMuseum_37.txt");
//                ReadTextData(mCon->filename);
                mCon->sciTimeCnt = 0;
                mCon->sciMotionSelector = SCI_DEMO;
                mCon->TaskMotionSelector = SCI_DEMO;
                scidemo_num=28;
            }
            else if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[3] == 2801)
            {
                StartWB(0);
                mCon->sciTimeCnt = 0;
                mCon->sciMotionSelector = SCI_DEMO;
                mCon->TaskMotionSelector = SCI_DEMO;
                scidemo_num=2801;
            }
            else if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[3] == 29)
            {
                StartWB(0);
                mCon->sciTimeCnt = 0;
                mCon->sciMotionSelector = SCI_DEMO;
                mCon->TaskMotionSelector = SCI_DEMO;
                scidemo_num=29;
            }
            else if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[3] == 30)
            {
                StartWB(0);
                mCon->sciTimeCnt = 0;
                mCon->sciMotionSelector = SCI_DEMO;
                mCon->TaskMotionSelector = SCI_DEMO;
                scidemo_num=30;
            }
            else if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[3] == 31)
            {
                StartWB(0);
                mCon->sciTimeCnt = 0;
                mCon->sciMotionSelector = SCI_DEMO;
                mCon->TaskMotionSelector = SCI_DEMO;
                scidemo_num=31;
            }
            else if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[3] == 32)
            {
                StartWB(0);
                mCon->sciTimeCnt = 0;
                mCon->sciMotionSelector = SCI_DEMO;
                mCon->TaskMotionSelector = SCI_DEMO;
                scidemo_num=32;
            }
            else if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[3] == 33)
            {
                StartWB(0);
                sprintf(mCon->filename, "../share/WB_Data/New_UB_ScienceMuseum_2.txt");
                ReadTextData(mCon->filename);
                mCon->sciTimeCnt = 0;
                mCon->sciMotionSelector = SCI_DEMO;
                mCon->TaskMotionSelector = SCI_DEMO;
                scidemo_num=33;
            }
            else if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[3] == 8801)
            {
                StartWB(0);
                mCon->sciTimeCnt = 0;
                mCon->sciMotionSelector = SCI_DEMO;
                mCon->TaskMotionSelector = SCI_DEMO;
                scidemo_num=8801;
            }
            else if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[3] == 8802)
            {
                StartWB(0);
                mCon->sciTimeCnt = 0;
                mCon->sciMotionSelector = SCI_DEMO;
                mCon->TaskMotionSelector = SCI_DEMO;
                scidemo_num=8802;
            }
            else if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[3] == 8803)
            {
                StartWB(0);
                mCon->sciTimeCnt = 0;
                mCon->sciMotionSelector = SCI_DEMO;
                mCon->TaskMotionSelector = SCI_DEMO;
                scidemo_num=8803;
            }
            else if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[3] == 101)
            {
                StartWB(0);
                mCon->sciTimeCnt = 0;
                mCon->sciMotionSelector = SCI_DEMO;
                mCon->TaskMotionSelector = SCI_DEMO;
                scidemo_num=101;
            }
            else if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[3] == 102)
            {
                StartWB(0);
                mCon->sciTimeCnt = 0;
                mCon->sciMotionSelector = SCI_DEMO;
                mCon->TaskMotionSelector = SCI_DEMO;
                scidemo_num=102;
            }
            else if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[3] == 103)
            {
                StartWB(0);
                mCon->sciTimeCnt = 0;
                mCon->sciMotionSelector = SCI_DEMO;
                mCon->TaskMotionSelector = SCI_DEMO;
                scidemo_num=103;
            }
            else if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[3] == 104)
            {
                StartWB(0);
                mCon->sciTimeCnt = 0;
                mCon->sciMotionSelector = SCI_DEMO;
                mCon->TaskMotionSelector = SCI_DEMO;
                scidemo_num=104;
            }
            else if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[3] == 105)//One
            {
                StartWB(0);
                mCon->sciTimeCnt = 0;
                mCon->sciMotionSelector = SCI_DEMO;
                mCon->TaskMotionSelector = SCI_DEMO;
                scidemo_num=105;
            }
            else if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[3] == 106)//Two
            {
                StartWB(0);
                mCon->sciTimeCnt = 0;
                mCon->sciMotionSelector = SCI_DEMO;
                mCon->TaskMotionSelector = SCI_DEMO;
                scidemo_num=106;
            }
            else if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[3] == 107)//Three
            {
                StartWB(0);
                mCon->sciTimeCnt = 0;
                mCon->sciMotionSelector = SCI_DEMO;
                mCon->TaskMotionSelector = SCI_DEMO;
                scidemo_num=107;
            }
            else if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[3] == 108)//
            {
                StartWB(0);
                sprintf(mCon->filename, "../share/WB_Data/New_UB_ScienceMuseum_26.txt");
                ReadTextData(mCon->filename);
                mCon->sciTimeCnt = 0;
                mCon->sciMotionSelector = SCI_DEMO;
                mCon->TaskMotionSelector = SCI_DEMO;
                scidemo_num=108;
            }
            else if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[3] == 815)
            {
                StartWB(0);
                mCon->sciTimeCnt = 0;
                mCon->sciMotionSelector = SCI_DEMO;
                mCon->TaskMotionSelector = SCI_DEMO;
                scidemo_num=815;
            }
            else if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[3] == 816)
            {
                StartWB(0);
                mCon->sciTimeCnt = 0;
                mCon->sciMotionSelector = SCI_DEMO;
                mCon->TaskMotionSelector = SCI_DEMO;
                scidemo_num=816;
            }
            else if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[3] == 817)
            {
                StartWB(0);
                mCon->sciTimeCnt = 0;
                mCon->sciMotionSelector = SCI_DEMO;
                mCon->TaskMotionSelector = SCI_DEMO;
                scidemo_num=817;
            }
            else if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[3] == 818)
            {
                StartWB(0);
                mCon->sciTimeCnt = 0;
                mCon->sciMotionSelector = SCI_DEMO;
                mCon->TaskMotionSelector = SCI_DEMO;
                scidemo_num=818;
            }
/*
            if(JOY_HEAD_MANUAL_MODE_FLAG){
                FILE_LOG(logERROR) << "Joy Manual Head Mode is Working Now..";
                sharedData->COMMAND[PODO_NO].USER_COMMAND = NO_ACT;
                break;
            }*/
            //--------------------------------------------------------------------------------------

            if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[3] == HANDSHAKE_READY){
                StartWB(0);
                //WholebodyInitialize();
                mCon->sciTimeCnt = 0;
                mCon->sciMotionSelector = SCI_DEMO;
                mCon->TaskMotionSelector = SCI_DEMO;
                scidemo_num = HANDSHAKE_READY;
            }if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[3] == HANDSHAKE_LOWGAIN){
                StartWB(0);
                mCon->sciTimeCnt = 0;
                mCon->sciMotionSelector = SCI_DEMO;
                mCon->TaskMotionSelector = SCI_DEMO;
                scidemo_num = HANDSHAKE_LOWGAIN;
            }if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[3] == HANDSHAKE_GRIP_ON){
                StartWB(0);
                mCon->sciTimeCnt = 0;
                mCon->sciMotionSelector = SCI_DEMO;
                mCon->TaskMotionSelector = SCI_DEMO;
                scidemo_num = HANDSHAKE_GRIP_ON;
            }if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[3] == HANDSHAKE_GRIP_OFF){
                StartWB(0);
                mCon->sciTimeCnt = 0;
                mCon->sciMotionSelector = SCI_DEMO;
                mCon->TaskMotionSelector = SCI_DEMO;
                scidemo_num = HANDSHAKE_GRIP_OFF;
            }if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[3] == HANDSHAKE_HIGHGAIN){
                StartWB(0);
                mCon->sciTimeCnt = 0;
                mCon->sciMotionSelector = SCI_DEMO;
                mCon->TaskMotionSelector = SCI_DEMO;
                scidemo_num = HANDSHAKE_HIGHGAIN;
            }if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[3] == HANDSHAKE_INIT){
                StartWB(0);
                mCon->sciTimeCnt = 0;
                mCon->sciMotionSelector = SCI_DEMO;
                mCon->TaskMotionSelector = SCI_DEMO;
                scidemo_num = HANDSHAKE_INIT;
            }
            //--------------------------------------------------------------------------------------

            if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[3] == BOUQUET_READY){
                StartWB(0);
                mCon->sciTimeCnt = 0;
                mCon->sciMotionSelector = SCI_DEMO;
                mCon->TaskMotionSelector = SCI_DEMO;
                scidemo_num = BOUQUET_READY;
            }if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[3] == BOUQUET_GIVE){
                StartWB(0);
                mCon->sciTimeCnt = 0;
                mCon->sciMotionSelector = SCI_DEMO;
                mCon->TaskMotionSelector = SCI_DEMO;
                scidemo_num = BOUQUET_GIVE;
            }if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[3] == BOUQUET_INIT){
                StartWB(0);
                mCon->sciTimeCnt = 0;
                mCon->sciMotionSelector = SCI_DEMO;
                mCon->TaskMotionSelector = SCI_DEMO;
                scidemo_num = BOUQUET_INIT;
            }
            //--------------------------------------------------------------------------------------

            if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[3] == CLAP_READY){
                StartWB(0);
                mCon->sciTimeCnt = 0;
                mCon->sciMotionSelector = SCI_DEMO;
                mCon->TaskMotionSelector = SCI_DEMO;
                mCon->clap_cnt = sharedData->COMMAND[PODO_NO].USER_PARA_CHAR[0];
                scidemo_num = CLAP_READY;
            }if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[3] == CLAP_INIT){
                StartWB(0);
                mCon->sciTimeCnt = 0;
                mCon->sciMotionSelector = SCI_DEMO;
                mCon->TaskMotionSelector = SCI_DEMO;
                scidemo_num = CLAP_INIT;
            }
            //--------------------------------------------------------------------------------------

            if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[3] == HEAD_YES){
                StartWB(0);
                mCon->sciTimeCnt = 0;
                mCon->sciMotionSelector = SCI_DEMO;
                mCon->TaskMotionSelector = SCI_DEMO;
                scidemo_num = HEAD_YES;
            }if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[3] == HEAD_NO){
                StartWB(0);
                mCon->sciTimeCnt = 0;
                mCon->sciMotionSelector = SCI_DEMO;
                mCon->TaskMotionSelector = SCI_DEMO;
                scidemo_num = HEAD_NO;
            }if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[3] == HEAD_STRETCHING){
                StartWB(0);
                mCon->sciTimeCnt = 0;
                mCon->sciMotionSelector = SCI_DEMO;
                mCon->TaskMotionSelector = SCI_DEMO;
                scidemo_num = HEAD_STRETCHING;
            }if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[3] == HEAD_INIT){
                StartWB(0);
                mCon->sciTimeCnt = 0;
                mCon->sciMotionSelector = SCI_DEMO;
                mCon->TaskMotionSelector = SCI_DEMO;
                scidemo_num = HEAD_INIT;
            }
            //--------------------------------------------------------------------------------------

            if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[3] == BOW_ONE){
                StartWB(0);
                mCon->sciTimeCnt = 0;
                mCon->sciMotionSelector = SCI_DEMO;
                mCon->TaskMotionSelector = SCI_DEMO;
                scidemo_num = BOW_ONE;
            }if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[3] == BOW_TWO){
                StartWB(0);
                mCon->sciTimeCnt = 0;
                mCon->sciMotionSelector = SCI_DEMO;
                mCon->TaskMotionSelector = SCI_DEMO;
                scidemo_num = BOW_TWO;
            }if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[3] == BOW_TWO_W){
                StartWB(0);
                mCon->sciTimeCnt = 0;
                sprintf(mCon->filename, "../share/WB_Data/New_UB_ScienceMuseum_2.txt");
                ReadTextData(mCon->filename);
                mCon->sciMotionSelector = SCI_DEMO;
                mCon->TaskMotionSelector = SCI_DEMO;
                scidemo_num = BOW_TWO_W;
            }if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[3] == BOW_GREETING){
                StartWB(0);
                mCon->sciTimeCnt = 0;
                sprintf(mCon->filename, "../share/WB_Data/New_UB_ScienceMuseum_12.txt");
                ReadTextData(mCon->filename);
                mCon->sciMotionSelector = SCI_DEMO;
                mCon->TaskMotionSelector = SCI_DEMO;
                scidemo_num = BOW_GREETING;
            }
            //--------------------------------------------------------------------------------------

            if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[3] == WST_TWIST){
                StartWB(0);
                mCon->sciTimeCnt = 0;
                mCon->sciMotionSelector = SCI_DEMO;
                mCon->TaskMotionSelector = SCI_DEMO;
                scidemo_num = WST_TWIST;
            }
            if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[3] == WST_ZERO){
                StartWB(0);
                mCon->sciTimeCnt = 0;
                mCon->sciMotionSelector = SCI_DEMO;
                mCon->TaskMotionSelector = SCI_DEMO;
                scidemo_num = WST_ZERO;
            }

            if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[3] == GUID_LEFT){
                StartWB(0);
                mCon->sciTimeCnt = 0;
                mCon->sciMotionSelector = SCI_DEMO;
                mCon->TaskMotionSelector = SCI_DEMO;
                scidemo_num = GUID_LEFT;
            }if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[3] == GUID_RIGHT){
                StartWB(0);
                mCon->sciTimeCnt = 0;
                mCon->sciMotionSelector = SCI_DEMO;
                mCon->TaskMotionSelector = SCI_DEMO;
                scidemo_num = GUID_RIGHT;
            }if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[3] == GUID_INIT){
                StartWB(0);
                mCon->sciTimeCnt = 0;
                mCon->sciMotionSelector = SCI_DEMO;
                mCon->TaskMotionSelector = SCI_DEMO;
                scidemo_num = GUID_INIT;
            }

            if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[3] == DEMO_SCENE_1){
                StartWB(0);
                mCon->sciTimeCnt = 0;
                mCon->sciMotionSelector = SCI_DEMO;
                mCon->TaskMotionSelector = SCI_DEMO;
                scidemo_num = DEMO_SCENE_1;
            }

            if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[3] == DEMO_SCENE_2){
                StartWB(0);
                mCon->sciTimeCnt = 0;
                mCon->sciMotionSelector = SCI_DEMO;
                mCon->TaskMotionSelector = SCI_DEMO;
                scidemo_num = DEMO_SCENE_2;
            }

            if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[3] == DEMO_SCENE_3){
                StartWB(0);
                mCon->sciTimeCnt = 0;
                mCon->sciMotionSelector = SCI_DEMO;
                mCon->TaskMotionSelector = SCI_DEMO;
                scidemo_num = DEMO_SCENE_3;
            }

            if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[3] == DEMO_SCENE_4){
                StartWB(0);
                mCon->sciTimeCnt = 0;
                mCon->sciMotionSelector = SCI_DEMO;
                mCon->TaskMotionSelector = SCI_DEMO;
                scidemo_num = DEMO_SCENE_4;
            }


            //--------------------------------------------------------------------------------------
            if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[3] == DANCE_GOGOGO){
                if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[10] == 0)StartWB(0);
                else StartWB(1);
                mCon->sciTimeCnt = 0;
                mCon->sciMotionSelector = SCI_DEMO;
                mCon->TaskMotionSelector = SCI_DEMO;
                scidemo_num=DANCE_GOGOGO;
                motion_loop_cnt = 0;
                bit_int_pc = BIT_PC;
                motion_time_check = 0;
                motion_sequence_loop = 0;
                if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[11] == 0) {
                    music_offset_2018 = 680;//120;
                }
                if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[11] == 1) {
                    music_offset_2018 = 480;
                    FILE_LOG(logSUCCESS) << "GOGOGO Walking ";

                }
            }if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[3] == DANCE_GOGOGO_READY){
//                StartWB(1);
                StartWB(0);
                mCon->sciTimeCnt = 0;
                mCon->sciMotionSelector = SCI_DEMO;
                mCon->TaskMotionSelector = SCI_DEMO;
                scidemo_num=DANCE_GOGOGO_READY;
            }


            sharedData->COMMAND[PODO_NO].USER_COMMAND = NO_ACT;
            break;
        case SCI_DEMO_Control_on:
            FILE_LOG(logSUCCESS) << "SCI_DEMO_control_on: " << sharedData->COMMAND[PODO_NO].USER_PARA_INT[3];
            StartWB(0);
            WholebodyInitialize();
            CNT_final_gain_DSP_ZMP_CON = 0;
            Control_OnOff_Flag = 1;
            sharedData->COMMAND[PODO_NO].USER_COMMAND = NO_ACT;
            break;
        case SCI_DEMO_Control_off:
            FILE_LOG(logSUCCESS) << "SCI_DEMO_control_off: " << sharedData->COMMAND[PODO_NO].USER_PARA_INT[3];
//            StartWB(0);
            WholebodyInitialize();
            CNT_final_gain_DSP_ZMP_CON2 = 0;
            Control_OnOff_Flag = 2;
            sharedData->COMMAND[PODO_NO].USER_COMMAND = NO_ACT;
            break;
        case SCI_DEMO_DATA_SAVE:
            if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[0] == 0){

                unsigned int i,j;

                fp = fopen("data_demo.txt","w");
                for(i=0;i<ROW_data_debug;i++)
                {
                    for(j=0;j<COL_data_debug;j++)fprintf(fp,"%g\t", JW_Data_Debug[j][i]);
                    fprintf(fp,"\n");
                }
                fclose(fp);

                fp2 = fopen("data2_demo.txt","w");
                for(i=0;i<ROW_data_debug;i++)
                {
                    for(j=0;j<COL_data_debug;j++)fprintf(fp2,"%g\t", JW_Data_Debug2[j][i]);
                    fprintf(fp2,"\n");
                }
                fclose(fp2);
                //cout << ">>> Data txt save~!!" << endl;
            }else{

                sJW_Data_Debug_Index=0;
                sJW_Data_Debug_Index2=0;
            }
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

        switch(mCon->TaskMotionSelector)
        {
        case SCI_DEMO:
            mCon->sciDemo();
            break;
        default:
            break;
        }
        get_zmp2();
        WBIK();

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
        for(int i=NKY; i<=NK2; i++){
            mCon->wbUBJoint->moveJointAngle(mCon->wbUBJoint->motionJoint[i]);
            jCon->SetJointRefAngle(i, mCon->wbUBJoint->motionJoint[i]->RefAngleCurrent);
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
    rt_task_set_periodic(NULL, TM_NOW, 600*1000);        // 600 usec

    while(__IS_WORKING)
    {
        rt_task_wait_period(NULL);

        if(HasAnyOwnership()){
            if(sharedData->SYNC_SIGNAL[PODO_NO] == true){
                jCon->JointUpdate();
                rt_task_resume(&rtTaskCon);
            }
        }
        else
        {
            Control_OnOff_Flag = 2;
            WholebodyInitialize();
        }
    }
}
//==============================//


void StopWB(){
    WB_FLAG = false;
}

void StartWB(int _mode){
    StopWB();

//    for(int i=0; i<=LAR; i++)
//    {
//    WalkReadyRef[i] = jCon->Joints[i]->RefAngleCurrent;
//    printf("WR = %f\n",WalkReadyRef[i]);
//    }

    jCon->RefreshToCurrentReference();
    mCon->ResetGlobalCoord(0);
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
    else if(_mode == 2)
    {
        for(int i=12; i<NO_OF_JOINTS; i++){
            if(i != WST){
                jCon->SetMotionOwner(i);
            }
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
// --------------------------------------------------------------------------------------------- //
void WholebodyFinalize()
{
}

void WholebodyInitialize()
{
    WBIK_PARA_CHANGE();

    Qub[idRSP] = 10.*D2R;
    Qub[idRSR] = -10.*D2R + OFFSET_RSR*D2R;
    Qub[idRSY] = -0.*D2R;
    Qub[idREB] = -30.*D2R + OFFSET_ELB*D2R;

    Qub[idLSP] = 10.*D2R;
    Qub[idLSR] = 10.*D2R + OFFSET_RSR*D2R;
    Qub[idLSY] =  0.*D2R;
    Qub[idLEB] = -30.*D2R + OFFSET_ELB*D2R;

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

//    for(int i=0; i<=LAR; i++)
//    {
//    WalkReadyRef[i] = jCon->Joints[i]->RefAngleCurrent;
////    printf("%f\n",WalkReadyRef[i]);
//    }


}
void WBIK()
{
//    if()
    Kirk_Control();
    des_pCOM_3x1[0] = 0.02 - Del_PC_X_DSP_XZMP_CON*0.001;//- Del_PC_X_SSP_XZMP_CON*0.001;
    des_pCOM_3x1[1] = 0.0 - Del_PC_Y_DSP_YZMP_CON*0.001;//- Del_PC_Y_SSP_YZMP_CON*0.001;
    des_pCOM_3x1[2] = 0.57;

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

    Qub[idRSP] = 10.*D2R;
    Qub[idLSP] = 10.*D2R;

    Qub[idRSR] = (-10.+ OFFSET_RSR)*D2R;
    Qub[idLSR] = (10. + OFFSET_LSR)*D2R;

    Qub[idRSY] = 0.*D2R;
    Qub[idLSY] = 0.*D2R;

    Qub[idREB] = (-30. + OFFSET_ELB)*D2R;
    Qub[idLEB] = (-30. + OFFSET_ELB)*D2R;

    Qub[idRWY] = 0.*D2R;
    Qub[idLWY] = 0.*D2R;

    Qub[idRWP] = 0.*D2R;
    Qub[idLWP] = 0.*D2R;

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


//    WalkReadyRef[RHY] = 0.00000;
//    WalkReadyRef[RHR] = 0.00000;
//    WalkReadyRef[RHP] = -28.274122;
//    WalkReadyRef[RKN] = 52.586784;
//    WalkReadyRef[RAP] = -24.312664;
//    WalkReadyRef[RAR] = 0.0000;
//    WalkReadyRef[LHY] = 0.00000;
//    WalkReadyRef[LHR] = 0.000;
//    WalkReadyRef[LHP] = -28.274122;
//    WalkReadyRef[LKN] = 52.586784;
//    WalkReadyRef[LAP] = -24.312664;
//    WalkReadyRef[LAR] = 0.0000;
    WalkReadyRef[RHY] = 0.00000;
    WalkReadyRef[RHR] = 0.00000;
    WalkReadyRef[RHP] = -26.402625;
    WalkReadyRef[RKN] = 49.704794;
    WalkReadyRef[RAP] = -23.302169;
    WalkReadyRef[RAR] = 0.0000;
    WalkReadyRef[LHY] = 0.00000;
    WalkReadyRef[LHR] = 0.000;
    WalkReadyRef[LHP] = -26.402625;
    WalkReadyRef[LKN] = 49.704794;
    WalkReadyRef[LAP] = -23.302169;
    WalkReadyRef[LAR] = 0.0000;

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
    final_gain_DSP_ZMP_CON = 0.5*(1-cos(PI*CNT_final_gain_DSP_ZMP_CON/200));
    final_gain_DSP_ZMP_CON2 = 0.5*(1-cos(PI*CNT_final_gain_DSP_ZMP_CON2/200));

    if(Control_OnOff_Flag == 1)
    {
        Del_PC_X_DSP_XZMP_CON = final_gain_DSP_ZMP_CON*kirkZMPCon_XP2(-Del_PC_X_DSP_XZMP_CON, X_ZMP_Local - (X_ZMP_REF_Local)*1000 , 1);
        Del_PC_Y_DSP_YZMP_CON = final_gain_DSP_ZMP_CON*kirkZMPCon_YP2(-Del_PC_Y_DSP_YZMP_CON, Y_ZMP_Local - (Y_ZMP_REF_Local)*1000 , 1);
        Old_Del_PC_X_DSP_XZMP_CON = Del_PC_X_DSP_XZMP_CON;
        Old_Del_PC_Y_DSP_YZMP_CON = Del_PC_Y_DSP_YZMP_CON;

        //printf("%d,%f,%d,%f, %f\n",CNT_final_gain_DSP_ZMP_CON,final_gain_DSP_ZMP_CON,CNT_final_gain_DSP_ZMP_CON2,final_gain_DSP_ZMP_CON2,Del_PC_X_DSP_XZMP_CON);
    }
    else if(Control_OnOff_Flag==2)
    {
        Del_PC_X_DSP_XZMP_CON = (1-final_gain_DSP_ZMP_CON2)*Old_Del_PC_X_DSP_XZMP_CON;//*kirkZMPCon_XP2(-Del_PC_X_DSP_XZMP_CON, X_ZMP_Local - (X_ZMP_REF_Local)*1000 , 1);
        Del_PC_Y_DSP_YZMP_CON = (1-final_gain_DSP_ZMP_CON2)*Old_Del_PC_Y_DSP_YZMP_CON;//*kirkZMPCon_YP2(-Del_PC_Y_DSP_YZMP_CON, Y_ZMP_Local - (Y_ZMP_REF_Local)*1000 , 1);

        //printf("%d,%f,%d,%f, %f\n",CNT_final_gain_DSP_ZMP_CON,final_gain_DSP_ZMP_CON,CNT_final_gain_DSP_ZMP_CON2,final_gain_DSP_ZMP_CON2,Del_PC_X_DSP_XZMP_CON);
    }
    else if(Control_OnOff_Flag==0)
    {
        Del_PC_X_DSP_XZMP_CON = 0;
        Del_PC_Y_DSP_YZMP_CON = 0;
        Old_Del_PC_X_DSP_XZMP_CON = Del_PC_X_DSP_XZMP_CON;
        Old_Del_PC_Y_DSP_YZMP_CON = Del_PC_Y_DSP_YZMP_CON;
    }

    LPF_Del_PC_X_DSP_XZMP_CON = (1.f-2.f*PI*3.0*(double)RT_TIMER_PERIOD_MS*0.001f)*LPF_Del_PC_X_DSP_XZMP_CON + 2.f*PI*3.0*(double)RT_TIMER_PERIOD_MS*0.001f*Del_PC_X_DSP_XZMP_CON;
    LPF_Del_PC_Y_DSP_YZMP_CON = (1.f-2.f*PI*3.0*(double)RT_TIMER_PERIOD_MS*0.001f)*LPF_Del_PC_Y_DSP_YZMP_CON + 2.f*PI*3.0*(double)RT_TIMER_PERIOD_MS*0.001f*Del_PC_Y_DSP_YZMP_CON;

    if(CNT_final_gain_DSP_ZMP_CON < 200 && Control_OnOff_Flag == 1 ){ CNT_final_gain_DSP_ZMP_CON++;}//printf("%d, final gain = %f, Del_ZMP_CON = %f\n",CNT_final_gain_DSP_ZMP_CON,final_gain_DSP_ZMP_CON,Del_PC_X_DSP_XZMP_CON);}

    if(CNT_final_gain_DSP_ZMP_CON2 <200 && Control_OnOff_Flag == 2) {CNT_final_gain_DSP_ZMP_CON2++;}

    Old_Del_PC_X_DSP_XZMP_CON2 = Del_PC_X_DSP_XZMP_CON;
    Old_Del_PC_Y_DSP_YZMP_CON2 = Del_PC_Y_DSP_YZMP_CON;

    Old_I_ZMP_CON_X = I_ZMP_CON_X;
    Old_I_ZMP_CON_Y = I_ZMP_CON_Y;


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
void JW_save()
{
    if(sJW_Data_Debug_Index < ROW_data_debug)
    {
        JW_Data_Debug[0][sJW_Data_Debug_Index] = jCon->Joints[RHY]->RefAngleCurrent ;
        JW_Data_Debug[1][sJW_Data_Debug_Index] = jCon->Joints[RHR]->RefAngleCurrent ;
        JW_Data_Debug[2][sJW_Data_Debug_Index] = jCon->Joints[RHP]->RefAngleCurrent ;
        JW_Data_Debug[3][sJW_Data_Debug_Index] = jCon->Joints[RKN]->RefAngleCurrent ;
        JW_Data_Debug[4][sJW_Data_Debug_Index] = jCon->Joints[RAP]->RefAngleCurrent ;
        JW_Data_Debug[5][sJW_Data_Debug_Index] = jCon->Joints[RAR]->RefAngleCurrent ;

        JW_Data_Debug[6][sJW_Data_Debug_Index] = jCon->Joints[LHY]->RefAngleCurrent ;
        JW_Data_Debug[7][sJW_Data_Debug_Index] = jCon->Joints[LHR]->RefAngleCurrent ;
        JW_Data_Debug[8][sJW_Data_Debug_Index] = jCon->Joints[LHP]->RefAngleCurrent ;
        JW_Data_Debug[9][sJW_Data_Debug_Index] = jCon->Joints[LKN]->RefAngleCurrent ;
        JW_Data_Debug[10][sJW_Data_Debug_Index] = jCon->Joints[LAP]->RefAngleCurrent ;
        JW_Data_Debug[11][sJW_Data_Debug_Index] = jCon->Joints[LAR]->RefAngleCurrent ;

        JW_Data_Debug[12][sJW_Data_Debug_Index] = cCon->AddJoints[RHY]->RefAngleCurrent ;
        JW_Data_Debug[13][sJW_Data_Debug_Index] = cCon->AddJoints[RHR]->RefAngleCurrent ;
        JW_Data_Debug[14][sJW_Data_Debug_Index] = cCon->AddJoints[RHP]->RefAngleCurrent ;
        JW_Data_Debug[15][sJW_Data_Debug_Index] = cCon->AddJoints[RKN]->RefAngleCurrent ;
        JW_Data_Debug[16][sJW_Data_Debug_Index] = cCon->AddJoints[RAP]->RefAngleCurrent ;
        JW_Data_Debug[17][sJW_Data_Debug_Index] = cCon->AddJoints[RAR]->RefAngleCurrent ;

        JW_Data_Debug[18][sJW_Data_Debug_Index] = cCon->AddJoints[LHY]->RefAngleCurrent ;
        JW_Data_Debug[19][sJW_Data_Debug_Index] = cCon->AddJoints[LHR]->RefAngleCurrent ;
        JW_Data_Debug[20][sJW_Data_Debug_Index] = cCon->AddJoints[LHP]->RefAngleCurrent ;
        JW_Data_Debug[21][sJW_Data_Debug_Index] = cCon->AddJoints[LKN]->RefAngleCurrent ;
        JW_Data_Debug[22][sJW_Data_Debug_Index] = cCon->AddJoints[LAP]->RefAngleCurrent ;
        JW_Data_Debug[23][sJW_Data_Debug_Index] = cCon->AddJoints[LAR]->RefAngleCurrent ;

        sJW_Data_Debug_Index++;
        if(sJW_Data_Debug_Index >= ROW_data_debug) sJW_Data_Debug_Index = 0;

    }
}

// --------------------------------------------------------------------------------------------- //
void JW_save2()
{
    if(sJW_Data_Debug_Index2 < ROW_data_debug)
    {
        JW_Data_Debug2[0][sJW_Data_Debug_Index2] = 0;

        sJW_Data_Debug_Index2++;
        if(sJW_Data_Debug_Index2 >= ROW_data_debug) sJW_Data_Debug_Index2 = 0;
    }
}
