#include "taskmotion.h"
#include "taskmotion.h"


#define RING_SIZE                400
#define HAND_GRASP_VALUE         60
#define HAND_STRETCH_VALUE      -60
#define HAND_STOP_VALUE          0
#define TRIGGER_GRASP_VALUE      14
#define TRIGGER_STRETCH_VALUE   -14

#define MODIFY_Z_VALUE          0.04

int RSPint;
int RSPmode;


extern int scidemo_num;
extern float   ringData[RING_SIZE][31];
extern int     ringIndexFront;
extern int     ringIndexBack;
extern float   localJoint[NO_OF_JOINTS];
extern float   localQpel[4];
extern float   localDeltaPCz;
extern float   WR_PelcZ;
extern int     fileReadFlag;
extern FILE*   fpMocap;

extern int posX;
extern int posY;

float alphaneck5=1/(1.+2.*3.141592*5.*0.005);
float alphaneck2=1/(1.+2.*3.141592*2.*0.005);
float alphaneck7=1/(1.+2.*3.141592*7.*0.005);
float alphaneck10=1/(1.+2.*3.141592*10.*0.005);
float alphaneck15=1/(1.+2.*3.141592*15.*0.005);
float alphaneck30=1/(1.+2.*3.141592*30.*0.005);
float dif_Pos_Neck_Yaw_LPF, dif_Pos_Neck_Pitch_LPF;
float Pos_Neck_Yaw_LPF, Pos_Neck_Pitch_LPF;
float Pos_Neck_Yaw_old, Pos_Neck_Pitch_old;
float Initial_COM[3];
//
float t1_101 = 1500.;
float t1_hapum = 1000.;
int t1_101_int = 0;
float z_101 = 0.04;
int bit_int = 47;
int music_offset = 165;

float MotionTime = 1000.;
int Int_MotionTime = (int)(MotionTime/RT_TIMER_PERIOD_MS);

TaskMotion::TaskMotion()
{
    for(int i=0; i<4; i++)
    {
        wbPosRF[i] = new TaskPos();
        wbPosLF[i] = new TaskPos();
        wbPosRH[i] = new TaskPos();
        wbPosLH[i] = new TaskPos();
    }
    wbPosWST = new TaskPos();
    for(int i=0; i<3; i++)
    {
        wbPosCOM[i] = new TaskPos();
    }
    wbPosPelZ = new TaskPos();

    wbOriRF = new TaskOri();
    wbOriLF = new TaskOri();
    wbOriRH = new TaskOri();
    wbOriLH = new TaskOri();
    wbOriPel = new TaskOri();

    wbUBJoint = new JointControlClass(sharedData, PODO_NO);

    TaskMotionSelector = NO_ACT;
    sciMotionSelector = NO_ACT;
    sciTimeCnt = 0;
    P_gain_yaw_neck = .2;
    P_gain_pitch_neck = .2;
}

void TaskMotion::LeadTextData(void)
{
    while(fileReadFlag == ENABLE)
    {
        if(((ringIndexFront+1)%RING_SIZE) == ringIndexBack%RING_SIZE)
        {
            //printf("ringIndexFront: %d\t ringIndexBack: %d\n", ringIndexFront, ringIndexBack);
            break;
        }

        ringIndexFront %= RING_SIZE;
        if(EOF == fscanf(fpMocap,"%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f"
            , &ringData[ringIndexFront][0], &ringData[ringIndexFront][1], &ringData[ringIndexFront][2], &ringData[ringIndexFront][3], &ringData[ringIndexFront][4]
            , &ringData[ringIndexFront][5], &ringData[ringIndexFront][6], &ringData[ringIndexFront][7], &ringData[ringIndexFront][8], &ringData[ringIndexFront][9]
            , &ringData[ringIndexFront][10], &ringData[ringIndexFront][11], &ringData[ringIndexFront][12], &ringData[ringIndexFront][13], &ringData[ringIndexFront][14]
            , &ringData[ringIndexFront][15], &ringData[ringIndexFront][16], &ringData[ringIndexFront][17], &ringData[ringIndexFront][18], &ringData[ringIndexFront][19]
            , &ringData[ringIndexFront][20], &ringData[ringIndexFront][21], &ringData[ringIndexFront][22], &ringData[ringIndexFront][23], &ringData[ringIndexFront][24]
            , &ringData[ringIndexFront][25], &ringData[ringIndexFront][26], &ringData[ringIndexFront][27], &ringData[ringIndexFront][28], &ringData[ringIndexFront][29]
            , &ringData[ringIndexFront][30]))
        {
            fileReadFlag = DISABLE;
            break;
        }
        else
        {
            ringIndexFront++;
        }
    }
}

void TaskMotion::ChaseTextData(void)
{
    float tempData[31];
    if(ringIndexBack%RING_SIZE != ringIndexFront%RING_SIZE)
    {
        ringIndexBack %= RING_SIZE;
        for(int i=0; i<31; i++) tempData[i] = ringData[ringIndexBack][i];

        ringIndexBack++;

        localJoint[WST] = tempData[5];  localJoint[RSP] = tempData[6];  localJoint[RSR] = tempData[7];  localJoint[RSY] = tempData[8];
        localJoint[REB] = tempData[9];  localJoint[RWY] = tempData[10]; localJoint[RWP] = tempData[11];
        if(scidemo_num==25)
        localJoint[LSP] = tempData[12]*0.9;
        else
        localJoint[LSP] = tempData[12];

        localJoint[LSR] = tempData[13]; localJoint[LSY] = tempData[14]; localJoint[LEB] = tempData[15]; localJoint[LWY] = 0.0;//tempData[16];
        localJoint[LWP] = tempData[17]; localJoint[NKY] = tempData[18]; localJoint[NK1] = tempData[19]; localJoint[NK2] = tempData[20];
//        localJoint[RF1] = tempData[21]; localJoint[RF2] = tempData[22]; localJoint[RF3] = tempData[23]; localJoint[RF4] = tempData[24];
//        localJoint[RF5] = tempData[25]; localJoint[LF1] = tempData[26]; localJoint[LF2] = tempData[27]; localJoint[LF3] = tempData[28];
//        localJoint[LF4] = tempData[29]; localJoint[LF5] = tempData[30];
        if(scidemo_num==108)
        {
        localJoint[RF1] = -1.*tempData[21]; localJoint[RF2] = -1.*tempData[22]; localJoint[RF3] = -1.*tempData[23]; localJoint[RF4] = -1.*tempData[24];
        localJoint[RF5] = -1.*tempData[25]; localJoint[LF1] = -1.*tempData[28]; localJoint[LF2] = -1.*tempData[28]; localJoint[LF3] = -1.*tempData[28];
        localJoint[LF4] = -1.*tempData[29]; localJoint[LF5] = -1.*tempData[30];
        }
        else if(scidemo_num==20)
        {
            localJoint[RF1] = -1.*tempData[21]; localJoint[RF2] = -1.*tempData[22]; localJoint[RF3] = -1.*tempData[23]; localJoint[RF4] = -1.*tempData[24];
            localJoint[RF5] = -1.*tempData[25]; localJoint[LF1] = -1.*tempData[26]; localJoint[LF2] = -1.*tempData[27]; localJoint[LF3] = -1.*tempData[28];
            localJoint[LF4] = -1.*tempData[29]; localJoint[LF5] = -1.*tempData[30];
        }
        else
        {
        localJoint[RF1] = -1.*tempData[21]; localJoint[RF2] = -1.*tempData[22]; localJoint[RF3] = -1.*tempData[23]; localJoint[RF4] = -1.*tempData[24];
        localJoint[RF5] = -1.*tempData[25]; localJoint[LF1] = -1.*tempData[26]; localJoint[LF2] = -1.*tempData[27]; localJoint[LF3] = -1.*tempData[28];
        localJoint[LF4] = -1.*tempData[29]; localJoint[LF5] = -1.*tempData[30];
        }

        localDeltaPCz = tempData[0];
        localQpel[0] = tempData[1];
        localQpel[1] = tempData[2];
        localQpel[2] = tempData[3];
        localQpel[3] = tempData[4];
    }

/*
        wbUBJoint->Joints[LSP]->RefAngleCurrent = localJoint[LSP];
        wbUBJoint->Joints[LSR]->RefAngleCurrent = localJoint[LSR];
        wbUBJoint->Joints[LSY]->RefAngleCurrent = localJoint[LSY];
        wbUBJoint->Joints[LEB]->RefAngleCurrent = localJoint[LEB];
        wbUBJoint->Joints[LWY]->RefAngleCurrent = localJoint[LWY];
        wbUBJoint->Joints[LWP]->RefAngleCurrent = localJoint[LWP];


        wbUBJoint->Joints[RSP]->RefAngleCurrent = localJoint[RSP];
        wbUBJoint->Joints[RSR]->RefAngleCurrent = localJoint[RSR];
        wbUBJoint->Joints[RSY]->RefAngleCurrent = localJoint[RSY];
        wbUBJoint->Joints[REB]->RefAngleCurrent = localJoint[REB];
        wbUBJoint->Joints[RWY]->RefAngleCurrent = localJoint[RWY];
        wbUBJoint->Joints[RWP]->RefAngleCurrent = localJoint[RWP];
        */

    for(int i=RSP; i < NO_OF_JOINTS; i++)
    {
        if(i==NKY || i==NK1 || i==NK2)  wbUBJoint->Joints[i]->RefAngleCurrent = localJoint[i]/100.0f;
        else if(i!=LWY)wbUBJoint->Joints[i]->RefAngleCurrent = localJoint[i];
    }
     //printf(" [LEB]: %f\n",localJoint[LEB]);

    wbPosWST->RefCurrent = localJoint[WST];
    //wbPosPelZ->RefCurrent =  wbPosPelZ->RefCurrent + localDeltaPCz;
    wbPosPelZ->RefCurrent =  WR_PelcZ + localDeltaPCz*0.6;//0.5;


    double tempq0=1.;
    double tempq1=0.;
    double tempq2=0.;
    double tempq3=0.;
    double tempqnorm=1.;

    tempq0=localQpel[0]*2.;//2.f;
    tempq1=localQpel[1];
    tempq2=localQpel[2];
    tempq3=localQpel[3];
    tempqnorm=sqrt(tempq0*tempq0+tempq1*tempq1+tempq2*tempq2+tempq3*tempq3);
    tempq0=tempq0/tempqnorm;
    tempq1=tempq1/tempqnorm;
    tempq2=tempq2/tempqnorm;
    tempq3=tempq3/tempqnorm;
    wbOriPel->RefCurrent[1] = tempq0;
    wbOriPel->RefCurrent[2] = tempq1;
    wbOriPel->RefCurrent[3] = tempq2;
    wbOriPel->RefCurrent[4] = tempq3;
//    printf("Quaternion = %f,%f,%f,%f\n",tempq0,tempq1,tempq2,tempq3);


    //for(int i=1; i<=4; i++) wbOriPel->RefCurrent[i] = localQpel[i-1];
}


unsigned char TaskMotion::refreshToCurrentReference(void)
{
    for(int i=1; i<=2; i++)
    {
        Hubo2->DesTaskPos.pCOM_2x1[i] = wbPosCOM[i]->RefCurrent = Hubo2->CurTaskPos.pCOM_3x1[i];
    }
    Hubo2->DesTaskPos.rWST = wbPosWST->RefCurrent = Hubo2->CurTaskPos.Q_32x1[idWST];
    Hubo2->DesTaskPos.pPELz = wbPosPelZ->RefCurrent = Hubo2->CurTaskPos.pPEL_3x1[3];
    for(int i=1; i<=3; i++)
    {
        Hubo2->DesTaskPos.pRF_3x1[i] = wbPosRF[i]->RefCurrent = (Hubo2->DesTaskPos.GorL_flag_pRF == GLOBAL) ? Hubo2->CurTaskPos.pRF_3x1[i] : Hubo2->CurTaskPos.pRF_LP_3x1[i];
        Hubo2->DesTaskPos.pLF_3x1[i] = wbPosLF[i]->RefCurrent = (Hubo2->DesTaskPos.GorL_flag_pLF == GLOBAL) ? Hubo2->CurTaskPos.pLF_3x1[i] : Hubo2->CurTaskPos.pLF_LP_3x1[i];
        if(Hubo2->DesTaskPos.GorL_flag_pRH == GLOBAL)
            Hubo2->DesTaskPos.pRH_3x1[i] = wbPosRH[i]->RefCurrent = Hubo2->CurTaskPos.pRH_3x1[i];
        else if(Hubo2->DesTaskPos.GorL_flag_pRH == LOCAL_PELV)
            Hubo2->DesTaskPos.pRH_3x1[i] = wbPosRH[i]->RefCurrent = Hubo2->CurTaskPos.pRH_LP_3x1[i];

        if(Hubo2->DesTaskPos.GorL_flag_pLH == GLOBAL)
            Hubo2->DesTaskPos.pLH_3x1[i] = wbPosLH[i]->RefCurrent = Hubo2->CurTaskPos.pLH_3x1[i];
        else if(Hubo2->DesTaskPos.GorL_flag_pLH == LOCAL_PELV)
            Hubo2->DesTaskPos.pLH_3x1[i] = wbPosLH[i]->RefCurrent = Hubo2->CurTaskPos.pLH_LP_3x1[i];

    }
    for(int i=1; i<=4; i++)
    {
        Hubo2->DesTaskPos.qRF_4x1[i] = wbOriRF->RefCurrent[i] = (Hubo2->DesTaskPos.GorL_flag_qRF == GLOBAL) ? Hubo2->CurTaskPos.qRF_4x1[i] : Hubo2->CurTaskPos.qRF_LP_4x1[i];
        Hubo2->DesTaskPos.qLF_4x1[i] = wbOriLF->RefCurrent[i] = (Hubo2->DesTaskPos.GorL_flag_qLF == GLOBAL) ? Hubo2->CurTaskPos.qLF_4x1[i] : Hubo2->CurTaskPos.qLF_LP_4x1[i];
        if(Hubo2->DesTaskPos.qRH_4x1[i] == GLOBAL)
            Hubo2->DesTaskPos.qRH_4x1[i] = wbOriRH->RefCurrent[i] = Hubo2->CurTaskPos.qRH_4x1[i];
        else if(Hubo2->DesTaskPos.qRH_4x1[i] == LOCAL_PELV)
            Hubo2->DesTaskPos.qRH_4x1[i] = wbOriRH->RefCurrent[i] = Hubo2->CurTaskPos.qRH_LP_4x1[i];


        if(Hubo2->DesTaskPos.qLH_4x1[i] == GLOBAL)
            Hubo2->DesTaskPos.qLH_4x1[i] = wbOriLH->RefCurrent[i] = Hubo2->CurTaskPos.qLH_4x1[i];
        else if(Hubo2->DesTaskPos.qLH_4x1[i] == LOCAL_PELV)
            Hubo2->DesTaskPos.qLH_4x1[i] = wbOriLH->RefCurrent[i] = Hubo2->CurTaskPos.qLH_LP_4x1[i];


        Hubo2->DesTaskPos.qPEL_4x1[i] = wbOriPel->RefCurrent[i] = Hubo2->CurTaskPos.qPEL_4x1[i];
    }

    Hubo2Command->GlobalLocalChangeFlag = DISABLE;
    return ERR_OK;
}

unsigned char TaskMotion::changeCoordAndInitialize(unsigned char _pRFflag, unsigned char _qRFflag, unsigned char _pLFflag, unsigned char _qLFflag, unsigned char _pRHflag, unsigned char _qRHflag, unsigned char _pLHflag, unsigned char _qLHflag)
{
    // Hold WBIK
    Hubo2Command->GlobalLocalChangeFlag = ENABLE;
    while(Hubo2Command->GlobalLocalChangeFlag != CONFIRM) usleep(100*1000);

    // Change coordinate system
    Hubo2->DesTaskPos.GorL_flag_pRF = _pRFflag;
    Hubo2->DesTaskPos.GorL_flag_pLF = _pLFflag;
    Hubo2->DesTaskPos.GorL_flag_qRF = _qRFflag;
    Hubo2->DesTaskPos.GorL_flag_qLF = _qLFflag;

    Hubo2->DesTaskPos.GorL_flag_pRH = _pRHflag;
    Hubo2->DesTaskPos.GorL_flag_pLH = _pLHflag;
    Hubo2->DesTaskPos.GorL_flag_qRH = _qRHflag;
    Hubo2->DesTaskPos.GorL_flag_qLH = _qLHflag;

    // Initialize RefCurrent and desired ref. to current WBIK ref.//
    for(int i=1; i<=2; i++)
    {
        Hubo2->DesTaskPos.pCOM_2x1[i] = wbPosCOM[i]->RefCurrent = Hubo2->CurTaskPos.pCOM_3x1[i];
    }
    Hubo2->DesTaskPos.rWST = wbPosWST->RefCurrent = Hubo2->CurTaskPos.Q_32x1[idWST];
    Hubo2->DesTaskPos.pPELz = wbPosPelZ->RefCurrent = Hubo2->CurTaskPos.pPEL_3x1[3];
    for(int i=1; i<=3; i++)
    {
        Hubo2->DesTaskPos.pRF_3x1[i] = wbPosRF[i]->RefCurrent = (Hubo2->DesTaskPos.GorL_flag_pRF == GLOBAL) ? Hubo2->CurTaskPos.pRF_3x1[i] : Hubo2->CurTaskPos.pRF_LP_3x1[i];
        Hubo2->DesTaskPos.pLF_3x1[i] = wbPosLF[i]->RefCurrent = (Hubo2->DesTaskPos.GorL_flag_pLF == GLOBAL) ? Hubo2->CurTaskPos.pLF_3x1[i] : Hubo2->CurTaskPos.pLF_LP_3x1[i];
        if(Hubo2->DesTaskPos.GorL_flag_pRH == GLOBAL)
            Hubo2->DesTaskPos.pRH_3x1[i] = wbPosRH[i]->RefCurrent = Hubo2->CurTaskPos.pRH_3x1[i];
        else if(Hubo2->DesTaskPos.GorL_flag_pRH == LOCAL_PELV)
            Hubo2->DesTaskPos.pRH_3x1[i] = wbPosRH[i]->RefCurrent = Hubo2->CurTaskPos.pRH_LP_3x1[i];

        if(Hubo2->DesTaskPos.GorL_flag_pLH == GLOBAL)
            Hubo2->DesTaskPos.pLH_3x1[i] = wbPosLH[i]->RefCurrent = Hubo2->CurTaskPos.pLH_3x1[i];
        else if(Hubo2->DesTaskPos.GorL_flag_pLH == LOCAL_PELV)
            Hubo2->DesTaskPos.pLH_3x1[i] = wbPosLH[i]->RefCurrent = Hubo2->CurTaskPos.pLH_LP_3x1[i];

    }
    for(int i=1; i<=4; i++)
    {
        Hubo2->DesTaskPos.qRF_4x1[i] = wbOriRF->RefCurrent[i] = (Hubo2->DesTaskPos.GorL_flag_qRF == GLOBAL) ? Hubo2->CurTaskPos.qRF_4x1[i] : Hubo2->CurTaskPos.qRF_LP_4x1[i];
        Hubo2->DesTaskPos.qLF_4x1[i] = wbOriLF->RefCurrent[i] = (Hubo2->DesTaskPos.GorL_flag_qLF == GLOBAL) ? Hubo2->CurTaskPos.qLF_4x1[i] : Hubo2->CurTaskPos.qLF_LP_4x1[i];
        if(Hubo2->DesTaskPos.qRH_4x1[i] == GLOBAL)
            Hubo2->DesTaskPos.qRH_4x1[i] = wbOriRH->RefCurrent[i] = Hubo2->CurTaskPos.qRH_4x1[i];
        else if(Hubo2->DesTaskPos.qRH_4x1[i] == LOCAL_PELV)
            Hubo2->DesTaskPos.qRH_4x1[i] = wbOriRH->RefCurrent[i] = Hubo2->CurTaskPos.qRH_LP_4x1[i];


        if(Hubo2->DesTaskPos.qLH_4x1[i] == GLOBAL)
            Hubo2->DesTaskPos.qLH_4x1[i] = wbOriLH->RefCurrent[i] = Hubo2->CurTaskPos.qLH_4x1[i];
        else if(Hubo2->DesTaskPos.qLH_4x1[i] == LOCAL_PELV)
            Hubo2->DesTaskPos.qLH_4x1[i] = wbOriLH->RefCurrent[i] = Hubo2->CurTaskPos.qLH_LP_4x1[i];


        Hubo2->DesTaskPos.qPEL_4x1[i] = wbOriPel->RefCurrent[i] = Hubo2->CurTaskPos.qPEL_4x1[i];
    }

    Hubo2Command->GlobalLocalChangeFlag = DISABLE;
    return ERR_OK;
}


unsigned char TaskMotion::setMoveCOM(float _xCOM, float _yCOM, float _msTime, unsigned char _mode)
{
    wbPosCOM[1]->setTaskPos(_xCOM, _msTime, _mode);
    wbPosCOM[2]->setTaskPos(_yCOM, _msTime, _mode);
    return ERR_OK;
}


unsigned char TaskMotion:: setMoveLeg(char _selectLeg, float _xLEG, float _yLEG, float _zLEG, float _ori[5], float _msTime, unsigned char _mode)
{
    switch(_selectLeg)
    {
    case HUBO_RIGHT:
        wbPosRF[1]->setTaskPos(_xLEG, _msTime, _mode);
        wbPosRF[2]->setTaskPos(_yLEG, _msTime, _mode);
        wbPosRF[3]->setTaskPos(_zLEG, _msTime, _mode);
        if(_ori != NULL)
            wbOriRF->setTaskOri(_ori, _msTime, _mode);
        break;
    case HUBO_LEFT:
        wbPosLF[1]->setTaskPos(_xLEG, _msTime, _mode);
        wbPosLF[2]->setTaskPos(_yLEG, _msTime, _mode);
        wbPosLF[3]->setTaskPos(_zLEG, _msTime, _mode);
        if(_ori != NULL)
            wbOriLF->setTaskOri(_ori, _msTime, _mode);
        break;
    default:
        printf(">>> Wrong leg selection..!!(moveLeg)\n");
        return ERR_WRONG_SELECTION;
    }
    return ERR_OK;
}



void TaskMotion:: setMoveFinger(char _selectFinger,int _direction, float _msTime)
{
    _direction=_direction*-1;
    if(_selectFinger>=RF1 && _selectFinger<=LF5)
    {
        if(_direction == 1)
        {
            wbUBJoint->SetMoveJoint(_selectFinger, 10.f, _msTime, MODE_ABSOLUTE);
        }
        else if(_direction == -1)
        {
            wbUBJoint->SetMoveJoint(_selectFinger, -10.f, _msTime, MODE_ABSOLUTE);
        }
    }

    else if(_selectFinger == 0)
    {
        if(_direction == 1)
        {
            for(int i=RF1; i<=RF5; i++) wbUBJoint->SetMoveJoint(i, 10.f, _msTime, MODE_ABSOLUTE);
        }
        else if(_direction == -1)
        {
            for(int i=RF1; i<=RF5; i++) wbUBJoint->SetMoveJoint(i, -10.f, _msTime, MODE_ABSOLUTE);
        }
    }
    else if(_selectFinger == 1)
    {
        if(_direction == 1)
        {
            for(int i=LF1; i<=LF5; i++) wbUBJoint->SetMoveJoint(i, 10.f, _msTime, MODE_ABSOLUTE);
        }
        else if(_direction == -1)
        {
            for(int i=LF1; i<=LF5; i++) wbUBJoint->SetMoveJoint(i, -10.f, _msTime, MODE_ABSOLUTE);
        }
    }
    else if(_selectFinger == 2)
    {
        if(_direction == 1)
        {
            for(int i=RF1; i<=LF5; i++) wbUBJoint->SetMoveJoint(i, 10.f, _msTime, MODE_ABSOLUTE);
        }
        else if(_direction == -1)
        {
            for(int i=RF1; i<=LF5; i++) wbUBJoint->SetMoveJoint(i, -10.f, _msTime, MODE_ABSOLUTE);
        }
    }

}


unsigned char TaskMotion::setMoveHand(char _selectHand, float _xHAND, float _yHAND, float _zHAND, float _ori[5], float _msTime, unsigned char _mode)
{
    switch(_selectHand)
    {
    case HUBO_RIGHT:
        wbPosRH[1]->setTaskPos(_xHAND, _msTime, _mode);
        wbPosRH[2]->setTaskPos(_yHAND, _msTime, _mode);
        wbPosRH[3]->setTaskPos(_zHAND, _msTime, _mode);
        if(_ori != NULL)
            wbOriRH->setTaskOri(_ori, _msTime, _mode);
        break;
    case HUBO_LEFT:
        wbPosLH[1]->setTaskPos(_xHAND, _msTime, _mode);
        wbPosLH[2]->setTaskPos(_yHAND, _msTime, _mode);
        wbPosLH[3]->setTaskPos(_zHAND, _msTime, _mode);
        if(_ori != NULL)
            wbOriLH->setTaskOri(_ori, _msTime, _mode);
        break;
    default:
        //printf(">>> Wrong leg selection..!!(moveHand)\n");
        return ERR_WRONG_SELECTION;
    }
    return ERR_OK;
}

void TaskMotion::initHeadHand(int select)
{
    if(select == 0)
    {
        printf(">>> Command: FIND_INIT_POS_HEAD\n");
        Hubo2Command->Command = INIT_FIND_POS_HEAD;
        usleep(250*1000);
        Hubo2Command->RefOutOnOff = ENABLE;
    }
    else if(select == 1)
    {
        printf(">>> Command: FIND_INIT_POS_HAND\n");
        Hubo2Command->Command = INIT_FIND_POS_RHAND;
        usleep(250*1000);
        Hubo2Command->Command = INIT_FIND_POS_LHAND;
        usleep(250*1000);
        Hubo2Command->RefOutOnOff = ENABLE;
    }
    else if(select == 2)
    {
        printf(">>> Command: FIND_INIT_POS_RHAND_LHAND_NECK\n");
        Hubo2Command->Command = INIT_FIND_POS_RHAND;
        usleep(250*1000);
        Hubo2Command->Command = INIT_FIND_POS_LHAND;
        usleep(250*1000);
        Hubo2Command->Command = INIT_FIND_POS_HEAD;
        usleep(250*1000);
        Hubo2Command->RefOutOnOff = ENABLE;
    }
}

void TaskMotion::moveTaskPosOri(int _taskSpace)
{
    int i;

    switch(_taskSpace)
    {
    case TASK_RF:
        for(i=1; i<=3; i++)
        {
            wbPosRF[i]->moveTaskPos();
            Hubo2->DesTaskPos.pRF_3x1[i] = wbPosRF[i]->RefCurrent;
        }
        wbOriRF->moveTaskOri();
        for(i=1; i<=4; i++) Hubo2->DesTaskPos.qRF_4x1[i] = wbOriRF->RefCurrent[i];
        break;
    case TASK_LF:
        for(i=1; i<=3; i++)
        {
            wbPosLF[i]->moveTaskPos();
            Hubo2->DesTaskPos.pLF_3x1[i] = wbPosLF[i]->RefCurrent;
        }
        wbOriLF->moveTaskOri();
        for(i=1; i<=4; i++) Hubo2->DesTaskPos.qLF_4x1[i] = wbOriLF->RefCurrent[i];
        break;
    case TASK_RH:
        for(i=1; i<=3; i++)
        {
            wbPosRH[i]->moveTaskPos();
            Hubo2->DesTaskPos.pRH_3x1[i] = wbPosRH[i]->RefCurrent;
        }
        wbOriRH->moveTaskOri();
        for(i=1; i<=4; i++) Hubo2->DesTaskPos.qRH_4x1[i] = wbOriRH->RefCurrent[i];
        break;
    case TASK_LH:
        for(i=1; i<=3; i++)
        {
            wbPosLH[i]->moveTaskPos();
            Hubo2->DesTaskPos.pLH_3x1[i] = wbPosLH[i]->RefCurrent;
        }
        wbOriLH->moveTaskOri();
        for(i=1; i<=4; i++) Hubo2->DesTaskPos.qLH_4x1[i] = wbOriLH->RefCurrent[i];
        break;
    case TASK_PEL:
        wbPosPelZ->moveTaskPos();
        Hubo2->DesTaskPos.pPELz = wbPosPelZ->RefCurrent;
        wbOriPel->moveTaskOri();
        for(i=1; i<=4; i++) Hubo2->DesTaskPos.qPEL_4x1[i] = wbOriPel->RefCurrent[i];
        break;
    case TASK_COM:
        for(i=1; i<=2; i++)
        {
            wbPosCOM[i]->moveTaskPos();
            Hubo2->DesTaskPos.pCOM_2x1[i] = wbPosCOM[i]->RefCurrent;
        }
        break;
    case TASK_WST:
        wbPosWST->moveTaskPos();
        Hubo2->DesTaskPos.rWST = wbPosWST->RefCurrent*D2R;
        break;
    }
}


void TaskMotion::taskUpdate(int _motionNum)
{
    for(int i=0; i<NO_OF_MOTION; i++)
    {
        if(Hubo2->WBMotionOwned[i] == _motionNum)
        {
            moveTaskPosOri(i);
        }
    }

    if(Hubo2->DesTaskPos.LA_Joint_mode_flag == ENABLE)
    {
        for(int i=LSP; i<=LWP; i++)
        {
            if(Hubo2->MotionOwned[i] == _motionNum) wbUBJoint->moveJointAngle(wbUBJoint->Joints[i]);
        }

        Hubo2->DesTaskPos.Q_32x1[idLSP] = wbUBJoint->Joints[LSP]->RefAngleCurrent*D2R;
        Hubo2->DesTaskPos.Q_32x1[idLSR] = (wbUBJoint->Joints[LSR]->RefAngleCurrent+OFFSET_LSR)*D2R;
        Hubo2->DesTaskPos.Q_32x1[idLSY] = wbUBJoint->Joints[LSY]->RefAngleCurrent*D2R;
        Hubo2->DesTaskPos.Q_32x1[idLEB] = (wbUBJoint->Joints[LEB]->RefAngleCurrent+OFFSET_ELB)*D2R;
        Hubo2->DesTaskPos.Q_32x1[idLWY] = wbUBJoint->Joints[LWY]->RefAngleCurrent*D2R;
        Hubo2->DesTaskPos.Q_32x1[idLWP] = wbUBJoint->Joints[LWP]->RefAngleCurrent*D2R;

    }
    if(Hubo2->DesTaskPos.RA_Joint_mode_flag == ENABLE)
    {
        for(int i=RSP; i<=RWP; i++)
        {
            if(Hubo2->MotionOwned[i] == _motionNum) wbUBJoint->moveJointAngle(wbUBJoint->Joints[i]);
        }

        Hubo2->DesTaskPos.Q_32x1[idRSP] = wbUBJoint->Joints[RSP]->RefAngleCurrent*D2R;
        Hubo2->DesTaskPos.Q_32x1[idRSR] = (wbUBJoint->Joints[RSR]->RefAngleCurrent+OFFSET_RSR)*D2R;
        Hubo2->DesTaskPos.Q_32x1[idRSY] = wbUBJoint->Joints[RSY]->RefAngleCurrent*D2R;
        Hubo2->DesTaskPos.Q_32x1[idREB] = (wbUBJoint->Joints[REB]->RefAngleCurrent+OFFSET_ELB)*D2R;
        Hubo2->DesTaskPos.Q_32x1[idRWY] = wbUBJoint->Joints[RWY]->RefAngleCurrent*D2R;
        Hubo2->DesTaskPos.Q_32x1[idRWP] = wbUBJoint->Joints[RWP]->RefAngleCurrent*D2R;
    }
}

/**************************************************************************************************************************************************/
void TaskMotion::sciDemo(int _motionNumber, JointControlClass *_wbJoint)
{
    static float temp_ori[5];
    switch(sciMotionSelector)
    {
    case SCI_DEMO:
        //demoindex
        int tempendtime;

        switch(scidemo_num)
        {


        case 1://SCENE #1 - Turn and look
            if(sciTimeCnt == 0)
            {
                wbPosWST->setTaskPos(45.f, 2000.f, MODE_RELATIVE);
                wbUBJoint->SetMoveJoint(NK1, -15.f, 2000.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(NK2, 15.f, 2000.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(NKY, 15.f, 2000.f, MODE_ABSOLUTE);
            }

            else if(sciTimeCnt==700)
            {
                wbPosWST->setTaskPos(-45.f, 2000.f, MODE_RELATIVE);
                wbUBJoint->SetMoveJoint(NK1, 0.f, 2000.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(NK2, 0.f, 2000.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(NKY, 0.f, 2000.f, MODE_ABSOLUTE);
            }
            else if(sciTimeCnt==1300)
            {
                initHeadHand(1);
                sciMotionSelector = NO_ACT;
                printf(">>> Scene-1 motion end\n");
            }
            break;

        case 2://SCENE #2 - Greeting with arm shake
            if(fileReadFlag == ENABLE)
            {
                LeadTextData();
            }
            ChaseTextData();
            if(sciTimeCnt == 0)
            {
                wbUBJoint->SetMoveJoint(LWY, -85.f, 2000.f, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == 1500)
            {
                wbUBJoint->SetMoveJoint(LWY, 0.f, 2000.f, MODE_ABSOLUTE);
            }
            if(ringIndexBack == ringIndexFront)
            {
                sciMotionSelector = NO_ACT;
                //printf(">>> Scene-2 motion end\n");
                printf("\n ========== %f \n",sciTimeCnt*0.005);
                Hubo2Command->CommandIntData[3] = 201;
                Hubo2Command->Command = SCI_DEMO;
            }
            break;
        case 201:
            if(sciTimeCnt == 0)
                initHeadHand(1);
            else if(sciTimeCnt==200)
            {
                sciMotionSelector = NO_ACT;
                printf(">>> Scene-2 motion end\n");
            }
            break;

        case 3://SCENE #3 -Bow and back
            if(fileReadFlag == ENABLE)
            {
                LeadTextData();
            }
            ChaseTextData();
            if(ringIndexBack == ringIndexFront)
            {
                tempendtime=sciTimeCnt;
                sciMotionSelector = NO_ACT;
                //printf(">>> Scene-3-0 motion end\n");
//                sciTimeCnt = 0;
//                sciMotionSelector = SCI_DEMO;
//                TaskMotionSelector = SCI_DEMO;
//                scidemo_num=301;
                printf("\n ========== %f \n",sciTimeCnt*0.005);
            }

            break;

        case 301://SCENE #3-1
            if(sciTimeCnt == 1)//to ready posture
            {
                wbUBJoint->SetMoveJoint(RSP, 50.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RSR, -30.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RSY, 40.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(REB, -45.f, 800.f, MODE_ABSOLUTE);

                setMoveFinger(0,-1,700);//2 both 0 right 1 left RF1

                //wbUBJoint->SetMoveJoint(NK1, 4.f, 800.f, MODE_ABSOLUTE);
                //wbUBJoint->SetMoveJoint(NK2, -4.f, 800.f, MODE_ABSOLUTE);

                wbPosWST->setTaskPos(-10.f, 800.f, MODE_ABSOLUTE);
            }
            else if(sciTimeCnt==161)//to body
            {
                wbUBJoint->SetMoveJoint(RSP, 60.f, 600.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RSR, -15.f, 600.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(REB, -70.f, 600.f, MODE_ABSOLUTE);

            }
            else if(sciTimeCnt==281)//to outside
            {
                wbUBJoint->SetMoveJoint(RSP, 50.f, 600.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RSR, -30.f, 600.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(REB, -40.f, 600.f, MODE_ABSOLUTE);
            }
            else if(sciTimeCnt==401)//to body08
            {
                wbUBJoint->SetMoveJoint(RSP, 60.f, 600.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RSR, -15.f, 600.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(REB, -70.f, 600.f, MODE_ABSOLUTE);
            }
            else if(sciTimeCnt==521)//to ready posture
            {
                wbUBJoint->SetMoveJoint(RSP, 50.f, 600.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RSR, -30.f, 600.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(REB, -45.f, 600.f, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == 650)//to walk ready
            {
                wbUBJoint->SetMoveJoint(RSP, 10.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RSR, -10.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RSY, 0.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(REB, -30.f, 800.f, MODE_ABSOLUTE);

                setMoveFinger(0,1,700);//2 both 0 right 1 left RF1

                //wbUBJoint->SetMoveJoint(NK1, 0.f, 800.f, MODE_ABSOLUTE);
                //wbUBJoint->SetMoveJoint(NK2, 0.f, 800.f, MODE_ABSOLUTE);

                wbPosWST->setTaskPos(0.f, 800.f, MODE_ABSOLUTE);
            }
            else if(sciTimeCnt == 900)
                initHeadHand(1);
            if(sciTimeCnt==1200)
            {
                sciMotionSelector = NO_ACT;
                printf("\n ==========time : %f \n",sciTimeCnt*0.005);
                printf(">>> Scene-3 motion end\n");
            }

            break;

        case 4://SCENE #4 -Not yet
            if(fileReadFlag == ENABLE)
            {
                LeadTextData();
            }
            ChaseTextData();
            if(ringIndexBack == ringIndexFront)
            {
                sciMotionSelector = NO_ACT;
                printf("\n ==========time : %f \n",sciTimeCnt*0.005);
                printf(">>> Scene-4 motion end\n");
            }

            break;

        case 5://SCENE #5 -arm motion + go down up
            if(fileReadFlag == ENABLE)
            {
                LeadTextData();
            }
            ChaseTextData();
            if(ringIndexBack == ringIndexFront)
            {
                tempendtime=sciTimeCnt;
                sciMotionSelector = NO_ACT;
                printf("\n ==========time : %f \n",sciTimeCnt*0.005);
                //printf(">>> Scene-5-0 motion end\n");
//                sciTimeCnt = 0;
//                sciMotionSelector = SCI_DEMO;
//                TaskMotionSelector = SCI_DEMO;
//                scidemo_num=501;
            }

            break;

        case 501://SCENE #5-1
            if(sciTimeCnt==101)
            {
                wbUBJoint->SetMoveJoint(RSP, -70.f, 1500.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(REB, -90.f, 1500.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RWY, 30.f, 1500.f, MODE_ABSOLUTE);

                wbUBJoint->SetMoveJoint(LSP, -70.f, 1500.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LEB, -90.f, 1500.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LWY, -30.f, 1500.f, MODE_ABSOLUTE);
                setMoveFinger(2,-1,700);//2 both 0 right 1 left RF1
            }
            else if(sciTimeCnt==401)
            {
                wbUBJoint->SetMoveJoint(RSP, 30.f, 1500.f, MODE_RELATIVE);
                wbUBJoint->SetMoveJoint(LSP, 30.f, 1500.f, MODE_RELATIVE);
                wbPosPelZ->setTaskPos(-0.10, 1500.f, MODE_RELATIVE);
            }
            else if(sciTimeCnt==701)
            {
                wbUBJoint->SetMoveJoint(RSP, -30.f, 1500.f, MODE_RELATIVE);
                wbUBJoint->SetMoveJoint(LSP, -30.f, 1500.f, MODE_RELATIVE);
                wbPosPelZ->setTaskPos(0.10, 1500.f, MODE_RELATIVE);
            }
            else if(sciTimeCnt==1001)
            {
                wbUBJoint->SetMoveJoint(RSP, 10.f, 1500.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(REB, -30.f, 1500.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RWY, 0.f, 1500.f, MODE_ABSOLUTE);

                wbUBJoint->SetMoveJoint(LSP, 10.f, 1500.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LEB, -30.f, 1500.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LWY, 0.f, 1500.f, MODE_ABSOLUTE);
                setMoveFinger(2,1,700);//2 both 0 right 1 left RF1
            }

            if(sciTimeCnt==401)
            {
                wbUBJoint->SetMoveJoint(RWY, -40.f, 700.f, MODE_RELATIVE);
                wbUBJoint->SetMoveJoint(LWY, 40.f, 700.f, MODE_RELATIVE);

                //wbUBJoint->SetMoveJoint(NK1, 5.f, 700.f, MODE_RELATIVE);
                //wbUBJoint->SetMoveJoint(NK2, -5.f, 700.f, MODE_RELATIVE);
                //wbUBJoint->SetMoveJoint(NKY, 2.f, 700.f, MODE_ABSOLUTE);
            }
            else if(sciTimeCnt==541)
            {
                wbUBJoint->SetMoveJoint(RWY, 40.f, 700.f, MODE_RELATIVE);
                wbUBJoint->SetMoveJoint(LWY, -40.f, 700.f, MODE_RELATIVE);

                //wbUBJoint->SetMoveJoint(NK1, -5.f, 700.f, MODE_RELATIVE);
                //wbUBJoint->SetMoveJoint(NK2, 5.f, 700.f, MODE_RELATIVE);
                //wbUBJoint->SetMoveJoint(NKY, -2.f, 700.f, MODE_ABSOLUTE);
            }
            else if(sciTimeCnt==681)
            {
                wbUBJoint->SetMoveJoint(RWY, -40.f, 700.f, MODE_RELATIVE);
                wbUBJoint->SetMoveJoint(LWY, 40.f, 700.f, MODE_RELATIVE);

                //wbUBJoint->SetMoveJoint(NK1, 5.f, 700.f, MODE_RELATIVE);
                //wbUBJoint->SetMoveJoint(NK2, -5.f, 700.f, MODE_RELATIVE);
                //wbUBJoint->SetMoveJoint(NKY, 2.f, 700.f, MODE_ABSOLUTE);
            }
            else if(sciTimeCnt==821)
            {
                wbUBJoint->SetMoveJoint(RWY, 40.f, 700.f, MODE_RELATIVE);
                wbUBJoint->SetMoveJoint(LWY, -40.f, 700.f, MODE_RELATIVE);

                //wbUBJoint->SetMoveJoint(NK1, -5.f, 700.f, MODE_ABSOLUTE);
                //wbUBJoint->SetMoveJoint(NK2, 5.f, 700.f, MODE_ABSOLUTE);
                //wbUBJoint->SetMoveJoint(NKY, 0.f, 700.f, MODE_ABSOLUTE);
            }

            if(sciTimeCnt==1300)
            {
                sciMotionSelector = NO_ACT;
                printf("\n ==========time : %f \n",sciTimeCnt*0.005);
                printf(">>> Scene-5 motion end\n");
            }
            break;

        case 6://SCENE #6 - Chase
            if(sciTimeCnt == 0)// right arm goto back
            {
                wbPosWST->setTaskPos(-15.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RSP, 50.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RSR, -5.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(REB, -48.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LSP, -30.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LSR, 5.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LEB, -84.f, 800.f, MODE_ABSOLUTE);

                setMoveFinger(2,-1,700);//2 both 0 right 1 left RF1
            }
            else if(sciTimeCnt==180)// right arm goto front
            {
                wbPosWST->setTaskPos(15.f, 1200.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RSP, -30.f, 1200.f, MODE_ABSOLUTE);
                //wbUBJoint->SetMoveJoint(RSR, -5.f, 1200.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(REB, -84.f, 1200.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LSP, 50.f, 1200.f, MODE_ABSOLUTE);
                //wbUBJoint->SetMoveJoint(LSR, 5.f, 1200.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LEB, -48.f, 1200.f, MODE_ABSOLUTE);

                wbUBJoint->SetMoveJoint(NKY, -10.f, 1200.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(NK1, 10.f, 1200.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(NK2, -10.f, 1200.f, MODE_ABSOLUTE);
            }
            else if(sciTimeCnt==420)// right arm goto back
            {
                wbPosWST->setTaskPos(-15.f, 1200.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RSP, 50.f, 1200.f, MODE_ABSOLUTE);
                //wbUBJoint->SetMoveJoint(RSR, -5.f, 1200.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(REB, -48.f, 1200.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LSP, -30.f, 1200.f, MODE_ABSOLUTE);
                //wbUBJoint->SetMoveJoint(LSR, 5.f, 1200.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LEB, -84.f, 1200.f, MODE_ABSOLUTE);

                wbUBJoint->SetMoveJoint(NKY, 10.f, 1200.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(NK1, -10.f, 1200.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(NK2, 10.f, 1200.f, MODE_ABSOLUTE);
            }
            else if(sciTimeCnt==660)// right arm goto front
            {
                wbPosWST->setTaskPos(15.f, 1200.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RSP, -30.f, 1200.f, MODE_ABSOLUTE);
                //wbUBJoint->SetMoveJoint(RSR, -5.f, 1200.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(REB, -84.f, 1200.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LSP, 50.f, 1200.f, MODE_ABSOLUTE);
                //wbUBJoint->SetMoveJoint(LSR, 5.f, 1200.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LEB, -48.f, 1200.f, MODE_ABSOLUTE);

                wbUBJoint->SetMoveJoint(NKY, -10.f, 1200.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(NK1, 10.f, 1200.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(NK2, -10.f, 1200.f, MODE_ABSOLUTE);
            }
            else if(sciTimeCnt==900)
            {
                wbPosWST->setTaskPos(0.f, 1100.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RSP, 10.f, 1100.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RSR, -10.f, 1100.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(REB, -30.f, 1100.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LSP, 10.f, 1100.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LSR, 10.f, 1100.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LEB, -30.f, 1100.f, MODE_ABSOLUTE);

                wbUBJoint->SetMoveJoint(NKY, 0.f, 1100.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(NK1, 0.f, 1100.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(NK2, 0.f, 1100.f, MODE_ABSOLUTE);

                setMoveFinger(2,1,700);//2 both 0 right 1 left RF1
            }

            if(sciTimeCnt==1200)
                initHeadHand(1);//0 head . 1 hand . 2 hand+head

            if(sciTimeCnt==1700)
            {
                sciMotionSelector = NO_ACT;
                printf("\n ==========time : %f \n",sciTimeCnt*0.005);
                printf(">>> Scene-6 motion end\n");
            }
            break;

        case 703://SCENE #703 -Come on~
            if(fileReadFlag == ENABLE)
            {
                LeadTextData();
            }
            ChaseTextData();
            if(ringIndexBack == ringIndexFront)
            {
                sciMotionSelector = NO_ACT;
                printf("\n ==========time : %f \n",sciTimeCnt*0.005);
                printf(">>> Scene-703 motion end\n");
            }

            break;
        case 801://SCENE #801 - ctrl off pos
            if(sciTimeCnt == 0)
            {
                wbPosPelZ->setTaskPos(-0.12, 1800.f, MODE_RELATIVE);
                wbUBJoint->SetMoveJoint(RSR, 5.f, 1800.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(REB, 0.f, 1800.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LSR, -5.f, 1800.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LEB, 0.f, 1800.f, MODE_ABSOLUTE);

                wbUBJoint->SetMoveJoint(NK1, -10.f, 1800.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(NK2, 10.f, 1800.f, MODE_ABSOLUTE);
            }
            else if(sciTimeCnt==400)
            {
                sciMotionSelector = NO_ACT;
                printf("\n ==========time : %f \n",sciTimeCnt*0.005);
                printf(">>> Scene-801 motion end\n");
            }
            break;
        case 802://SCENE #802 - ctrl on pos
            if(sciTimeCnt == 0)
            {
                wbPosPelZ->setTaskPos(0.12, 1800.f, MODE_RELATIVE);
                wbUBJoint->SetMoveJoint(RSR, -10.f, 1800.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(REB, -30.f, 1800.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LSR, 10.f, 1800.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LEB, -30.f, 1800.f, MODE_ABSOLUTE);

                wbUBJoint->SetMoveJoint(NK1, 0.f, 1800.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(NK2, 0.f, 1800.f, MODE_ABSOLUTE);
            }
            else if(sciTimeCnt==400)
            {
                sciMotionSelector = NO_ACT;
                printf("\n ==========time : %f \n",sciTimeCnt*0.005);
                printf(">>> Scene-802 motion end\n");
            }
            break;

//        case 9:
//            if(fileReadFlag == ENABLE)
//            {
//                LeadTextData();
//            }
//            ChaseTextData();
//            if(ringIndexBack == ringIndexFront)
//            {
//                sciMotionSelector = NO_ACT;
//                printf(">>> Scene-9 motion end\n");
//            }

//            break;
//        case 10:
//            if(fileReadFlag == ENABLE)
//            {
//                LeadTextData();
//            }
//            ChaseTextData();
//            if(ringIndexBack == ringIndexFront)
//            {
//                sciMotionSelector = NO_ACT;
//                //printf(">>> Scene-10 motion end\n");
//                Hubo2Command->CommandIntData[3] = 1001;
//                Hubo2Command->Command = SCI_DEMO;
//            }

//            break;
//        case 1001:
//            if(sciTimeCnt == 0)
//                initHeadHand(1);
//            else if(sciTimeCnt==200)
//            {
//                sciMotionSelector = NO_ACT;
//                printf(">>> Scene-10 motion end\n");
//            }
//            break;
        case 11:
            if(fileReadFlag == ENABLE)
            {
                LeadTextData();
            }
            ChaseTextData();
            if(ringIndexBack == ringIndexFront)
            {
                sciMotionSelector = NO_ACT;
                printf("\n ==========time : %f \n",sciTimeCnt*0.005);
                printf(">>> Scene-11 motion end\n");
            }

            break;
        case 12:
            if(fileReadFlag == ENABLE)
            {
                LeadTextData();
            }
            ChaseTextData();
            if(ringIndexBack == ringIndexFront)
            {
                sciMotionSelector = NO_ACT;
                printf("\n ==========time : %f \n",sciTimeCnt*0.005);
                printf(">>> Scene-12 motion end\n");
            }

            break;
        case 13:
            if(sciTimeCnt == 0)
            {
                wbUBJoint->SetMoveJoint(NK1, 25.f, 1000.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(NK2, -10.f, 1000.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(NKY, -20.f, 1000.f, MODE_ABSOLUTE);
            }
            if(sciTimeCnt==600)//after 2+3sec
            {
                wbUBJoint->SetMoveJoint(NK1, 0.f, 1000.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(NK2, 0.f, 1000.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(NKY, 0.f, 1000.f, MODE_ABSOLUTE);
            }
            if(sciTimeCnt==1000)
            {
                sciMotionSelector = NO_ACT;
                printf("\n ==========time : %f \n",sciTimeCnt*0.005);
                printf(">>> Scene-13 motion end\n");
            }
            break;
        case 1401:
            if(sciTimeCnt == 0)
            {

                wbPosWST->setTaskPos(-30.f, 1200.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RSP, 40.f, 1200.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(REB, -80.f, 1200.f, MODE_ABSOLUTE);

                wbUBJoint->SetMoveJoint(NKY, 20.f, 1200.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(NK1, 10.f, 1200.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(NK2, -10.f, 1200.f, MODE_ABSOLUTE);

                wbUBJoint->SetMoveJoint(LSP, -25.f, 1200.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LSR, -5.f, 1200.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LEB, -80.f, 1200.f, MODE_ABSOLUTE);

                RSPint=(int)(10*rand());
                if(RSPint<0)
                    RSPint=-1*RSPint;
                printf("my RSPint is :%d\n",RSPint);
                if(RSPint%3==0)
                    RSPmode=0;//rock
                else if(RSPint%3==1)
                    RSPmode=1;//
                else if(RSPint%3==2)
                    RSPmode=2;//paper
                else
                    RSPmode=3;
                printf("my RSPmode is :%d\n",RSPmode);
            }

            else if(sciTimeCnt==300)
            {
                sciMotionSelector = NO_ACT;
                printf("\n ==========time : %f \n",sciTimeCnt*0.005);
                printf(">>> Scene-1401 motion end\n");
            }
            break;
        case 1402://SCENE #1 - Turn and look
            if(sciTimeCnt == 100)
            {
                wbPosWST->setTaskPos(25.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RSP, -70.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RSR, 10.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(REB, -10.f, 800.f, MODE_ABSOLUTE);

                wbUBJoint->SetMoveJoint(NKY, -25.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(NK1, 0.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(NK2, 0.f, 800.f, MODE_ABSOLUTE);

                wbUBJoint->SetMoveJoint(LSP, 10.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LSR, 10.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LEB, -30.f, 800.f, MODE_ABSOLUTE);
                if(RSPmode==0)
                {
                    setMoveFinger(0,-1,700);//2 both 0 right 1 left RF1
                }
                else if(RSPmode==1)
                {
                    setMoveFinger(RF1,-1,700);//2 both 0 right 1 left RF1
                    setMoveFinger(RF2,1,800);//2 both 0 right 1 left RF1
                    setMoveFinger(RF3,1,800);//2 both 0 right 1 left RF1
                    setMoveFinger(RF4,-1,700);//2 both 0 right 1 left RF1
                    setMoveFinger(RF5,-1,700);//2 both 0 right 1 left RF1
                }
                else if(RSPmode==2)
                {
                    setMoveFinger(0,1,700);//2 both 0 right 1 left RF1
                }
            }
            else if(sciTimeCnt == 660)
            {
                wbPosWST->setTaskPos(0.f, 1000.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RSP, 10.f, 1000.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RSR, -10.f, 1000.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(REB, -30.f, 1000.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(NKY, 0.f, 1000.f, MODE_ABSOLUTE);
                if(RSPmode==0)
                {
                    setMoveFinger(0,1,700);//2 both 0 right 1 left RF1
                }
                else if(RSPmode==1)
                {
                    setMoveFinger(RF1,1,700);
                    setMoveFinger(RF2,-1,700);//2 both 0 right 1 left RF1
                    setMoveFinger(RF3,-1,700);//2 both 0 right 1 left RF1
                    setMoveFinger(RF4,1,700);//2 both 0 right 1 left RF1
                    setMoveFinger(RF5,1,700);//2 both 0 right 1 left RF1
                }
                else if(RSPmode==2)
                {
                    setMoveFinger(0,-1,700);//2 both 0 right 1 left RF1
                }
            }
            else if(sciTimeCnt == 940)
                initHeadHand(1);//0=head 1=hand 2=both

            if(sciTimeCnt==1350)
            {
                sciMotionSelector = NO_ACT;
                printf("\n ==========time : %f \n",sciTimeCnt*0.005);
                printf(">>> Scene-1402 motion end\n");
            }
            break;
        case 15:
            if(sciTimeCnt==0)
            {
                wbUBJoint->SetMoveJoint(RSP, -40.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RSR, 0.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RSY, 25.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(REB, -90.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RWY, -40.f, 800.f, MODE_ABSOLUTE);

                wbUBJoint->SetMoveJoint(LSP, -40.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LSR, 0.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LSY, -25.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LEB, -90.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LWY, 40.f, 800.f, MODE_ABSOLUTE);
                setMoveFinger(2,1,700);//2 both 0 right 1 left RF1
            }
            else if(sciTimeCnt==200)//go up
            {
                wbUBJoint->SetMoveJoint(RSP, -45.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(REB, -110.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LSP, -45.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LEB, -110.f, 400.f, MODE_ABSOLUTE);

                wbUBJoint->SetMoveJoint(NK1, -13.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(NK2, -13.f, 400.f, MODE_ABSOLUTE);
            }
            else if(sciTimeCnt==280)
            {
                wbUBJoint->SetMoveJoint(RSP, -40.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(REB, -90.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LSP, -40.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LEB, -90.f, 400.f, MODE_ABSOLUTE);

                wbUBJoint->SetMoveJoint(NK1, 0.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(NK2, 0.f, 400.f, MODE_ABSOLUTE);
            }
            else if(sciTimeCnt==360)//go up
            {
                wbUBJoint->SetMoveJoint(RSP, -45.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(REB, -110.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LSP, -45.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LEB, -110.f, 400.f, MODE_ABSOLUTE);

                wbUBJoint->SetMoveJoint(NK1, -13.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(NK2, -13.f, 400.f, MODE_ABSOLUTE);
            }
            else if(sciTimeCnt==440)
            {
                wbUBJoint->SetMoveJoint(RSP, -40.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(REB, -90.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LSP, -40.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LEB, -90.f, 400.f, MODE_ABSOLUTE);

                wbUBJoint->SetMoveJoint(NK1, 0.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(NK2, 0.f, 400.f, MODE_ABSOLUTE);
            }
            else if(sciTimeCnt==520)//go up
            {
                wbUBJoint->SetMoveJoint(RSP, -45.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(REB, -110.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LSP, -45.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LEB, -110.f, 400.f, MODE_ABSOLUTE);

                wbUBJoint->SetMoveJoint(NK1, -13.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(NK2, -13.f, 400.f, MODE_ABSOLUTE);
            }
            else if(sciTimeCnt==600)
            {
                wbUBJoint->SetMoveJoint(RSP, -40.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(REB, -90.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LSP, -40.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LEB, -90.f, 400.f, MODE_ABSOLUTE);
            }
            else if(sciTimeCnt==900)
            {
                wbUBJoint->SetMoveJoint(RSP, 10.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RSR, -10.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RSY, 0.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(REB, -30.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RWY, 0.f, 800.f, MODE_ABSOLUTE);

                wbUBJoint->SetMoveJoint(LSP, 10.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LSR, 10.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LSY, 0.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LEB, -30.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LWY, 0.f, 800.f, MODE_ABSOLUTE);

                wbUBJoint->SetMoveJoint(NK1, 0.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(NK2, 0.f, 800.f, MODE_ABSOLUTE);
                setMoveFinger(2,-1,600);//2 both 0 right 1 left RF1
            }


            if(sciTimeCnt == 520)
            {
                wbUBJoint->SetMoveJoint(NKY, 20.f, 600.f, MODE_ABSOLUTE);
            }
            else if(sciTimeCnt==640)
            {
                wbUBJoint->SetMoveJoint(NKY, -20.f, 600.f, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == 760)
            {
                wbUBJoint->SetMoveJoint(NKY, 20.f, 600.f, MODE_ABSOLUTE);
            }
            else if(sciTimeCnt==880)
            {
                wbUBJoint->SetMoveJoint(NKY, 0.f, 600.f, MODE_ABSOLUTE);
            }

            if(sciTimeCnt==1100)
                initHeadHand(1);//0 head . 1 hand . 2 hand+head

            if(sciTimeCnt==1350)
            {
                sciMotionSelector = NO_ACT;
                printf("\n ==========time : %f \n",sciTimeCnt*0.005);
                printf(">>> Scene-15 motion end\n");
            }
            break;

        case 701:

            break;

        case 702:
            if(sciTimeCnt ==0)
            {
                wbUBJoint->SetMoveJoint(NKY, 0.f, 2000.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(NK1, 0.f, 2000.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(NK2, 0.f, 2000.f, MODE_ABSOLUTE);
                initHeadHand(0);
            }
            if(sciTimeCnt == 500)
            {
                sciMotionSelector = NO_ACT;
                printf("\n ==========time : %f \n",sciTimeCnt*0.005);}

            break;

        case 16:
            if(fileReadFlag == ENABLE)
            {
                LeadTextData();
            }
            ChaseTextData();
            if(sciTimeCnt == 100)
            {
                wbUBJoint->SetMoveJoint(LWY, -80.f, 600.f, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == 700)
            {
                wbUBJoint->SetMoveJoint(LWY, 0.f, 400.f, MODE_ABSOLUTE);
            }
            if(ringIndexBack == ringIndexFront)
            {
                sciMotionSelector = NO_ACT;
                printf("\n ==========time : %f \n",sciTimeCnt*0.005);
                printf(">>> Scene-16 motion end\n");
            }

            break;
        case 17:
            if(fileReadFlag == ENABLE)
            {
                LeadTextData();
            }
            ChaseTextData();
            if(ringIndexBack == ringIndexFront)
            {
                sciMotionSelector = NO_ACT;
                printf("\n ==========time : %f \n",sciTimeCnt*0.005);
                printf(">>> Scene-17 motion end\n");
            }

            break;
        case 1801:
            if(sciTimeCnt ==0)
            {
                Initial_COM[1]=Hubo2->CurTaskPos.pCOM_3x1[1];
                Initial_COM[2]=Hubo2->CurTaskPos.pCOM_3x1[2];


                wbUBJoint->SetMoveJoint(RSR, -20.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LSR, 20.f, 400.f, MODE_ABSOLUTE);
            }

            if(sciTimeCnt == 100)
            {
                setMoveCOM(Hubo2->CurTaskPos.pLF_LP_3x1[1], Hubo2->CurTaskPos.pLF_LP_3x1[2]+0.02, 1500, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == 480)
            {
                setMoveLeg(HUBO_RIGHT, 0.10, 0, 0.075, NULL, 1500, MODE_RELATIVE);
            }
            if(sciTimeCnt == 900)
            {
                sciMotionSelector = NO_ACT;
                printf("\n ==========time : %f \n",sciTimeCnt*0.005);
                printf(">>> Scene-1801 motion end\n");
            }
            break;
        case 1802:
            if(sciTimeCnt == 0)
            {
                wbPosPelZ->setTaskPos(-0.06, 2000.f, MODE_RELATIVE);
                setMoveLeg(HUBO_RIGHT, 0.05, 0, 0.02, NULL, 2000, MODE_RELATIVE);
            }
            if(sciTimeCnt == 500)
            {
                wbPosPelZ->setTaskPos(0.06, 2000.f, MODE_RELATIVE);
                setMoveLeg(HUBO_RIGHT, -0.05, 0, -0.02, NULL, 2000, MODE_RELATIVE);
            }
            if(sciTimeCnt == 1150)
            {
                sciMotionSelector = NO_ACT;
                printf("\n ==========time : %f \n",sciTimeCnt*0.005);
                printf(">>> Scene-1802 motion end\n");
            }
            break;

        case 1804:
            if(sciTimeCnt == 0)
            {
                setMoveLeg(HUBO_RIGHT, -0.10, 0, -0.072, NULL, 1600, MODE_RELATIVE);
            }
            if(sciTimeCnt == 350)
            {
                setMoveCOM(Initial_COM[1], Initial_COM[2], 1800, MODE_ABSOLUTE);
                setMoveLeg(HUBO_RIGHT, 0, 0, -0.003, NULL, 1800, MODE_RELATIVE);
            }
            if(sciTimeCnt == 800)
            {
                wbUBJoint->SetMoveJoint(RSR, -10.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LSR, 10.f, 400.f, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == 1000)
            {
                sciMotionSelector = NO_ACT;
                printf("\n ==========time : %f \n",sciTimeCnt*0.005);
                printf(">>> Scene-1804 motion end\n");
            }
            break;

        case 19:
            if(fileReadFlag == ENABLE)
            {
                LeadTextData();
            }
            ChaseTextData();
            if(ringIndexBack == ringIndexFront)
            {
                sciMotionSelector = NO_ACT;
                printf("\n ==========time : %f \n",sciTimeCnt*0.005);
                //printf(">>> Scene-1900 motion end\n");
                Hubo2Command->CommandIntData[3] = 1901;
                Hubo2Command->Command = SCI_DEMO;
            }

            break;

        case 1901:
            if(fileReadFlag == ENABLE)
            {
                LeadTextData();
            }
            ChaseTextData();
            if(ringIndexBack == ringIndexFront)
            {
                sciMotionSelector = NO_ACT;
                printf("\n ==========time : %f \n",sciTimeCnt*0.005);
                Hubo2Command->CommandIntData[3] = 1902;
                Hubo2Command->Command = SCI_DEMO;
            }
            break;
        case 1902:
            if(sciTimeCnt == 0)
                initHeadHand(1);
            else if(sciTimeCnt==180)
            {
                sciMotionSelector = NO_ACT;
                printf("\n ==========time : %f \n",sciTimeCnt*0.005);
                printf(">>> Scene-19 motion end\n");
            }
            break;

        case 20:
            if(fileReadFlag == ENABLE)
            {
                LeadTextData();
            }
            ChaseTextData();

            if(sciTimeCnt == 0)
            {
                wbUBJoint->SetMoveJoint(LWY, -55.f, 700.f, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == 500)
            {
                wbUBJoint->SetMoveJoint(LWY, 0.f, 700.f, MODE_ABSOLUTE);

            }
            if(sciTimeCnt == 700)
            {
                 initHeadHand(1);
            }

            if(ringIndexBack == ringIndexFront)
            {
                sciMotionSelector = NO_ACT;
                printf("\n ==========time : %f , %d\n",sciTimeCnt*0.005,sciTimeCnt);
                //printf(">>> Scene-20 motion end\n");
//                Hubo2Command->CommandIntData[3] = 2001;
//                Hubo2Command->Command = SCI_DEMO;
            }

            break;
        case 2001:
            if(sciTimeCnt == 0)
                initHeadHand(1);
            else if(sciTimeCnt==200)
            {
                sciMotionSelector = NO_ACT;
                printf("\n ==========time : %f \n",sciTimeCnt*0.005);
                printf(">>> Scene-20 motion end\n");
            }
            break;
        case 21:
            if(sciTimeCnt == 0)
            {
                setMoveFinger(LF2,-1,700);//2 both 0 right 1 left RF1
                setMoveFinger(LF3,-1,700);//2 both 0 right 1 left RF1
                setMoveFinger(LF4,-1,700);//2 both 0 right 1 left RF1
                setMoveFinger(LF5,-1,700);//2 both 0 right 1 left RF1

                setMoveFinger(RF2,-1,700);//2 both 0 right 1 left RF1
                setMoveFinger(RF3,-1,700);//2 both 0 right 1 left RF1
                setMoveFinger(RF4,-1,700);//2 both 0 right 1 left RF1
                setMoveFinger(RF5,-1,700);//2 both 0 right 1 left RF1
            }
            if(sciTimeCnt == 140)
            {
                wbUBJoint->SetMoveJoint(RSP, -50.f, 1200.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RSR, -15.f, 1200.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RSY, 5.f, 1200.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(REB, -115.f, 1200.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RWY, 30.f, 1200.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RWP, 0.f, 1200.f, MODE_ABSOLUTE);

                wbUBJoint->SetMoveJoint(LSP, -50.f, 1200.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LSR, 15.f, 1200.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LSY, -5.f, 1200.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LEB, -115.f, 1200.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LWY, -30.f, 1200.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LWP, 0.f, 1200.f, MODE_ABSOLUTE);
                setMoveFinger(LF2,1,1500);//2 both 0 right 1 left RF1
                setMoveFinger(RF2,1,1500);//2 both 0 right 1 left RF1

            }
            // 2000ms = 400step
            if(sciTimeCnt == 380)
            {
                wbUBJoint->SetMoveJoint(LSP, -80.f, 600.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LEB, -125.f, 600.f, MODE_ABSOLUTE);
                wbPosPelZ->setTaskPos(-0.03, 600.f, MODE_RELATIVE);
                wbUBJoint->SetMoveJoint(NK1, -8.f, 600.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(NK2, 8.f, 600.f, MODE_ABSOLUTE);

            }
            // 3000ms = 600step
            if(sciTimeCnt == 500)
            {
                wbUBJoint->SetMoveJoint(LSP, -50.f, 600.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LEB, -115.f, 600.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LWY, -45.f, 600.f, MODE_ABSOLUTE);

                wbUBJoint->SetMoveJoint(RSP, -80.f, 600.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(REB, -125.f, 600.f, MODE_ABSOLUTE);

                wbUBJoint->SetMoveJoint(NK1, 8.f, 600.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(NK2, -8.f, 600.f, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == 580)
            {
                setMoveFinger(LF3,1,1500);//2 both 0 right 1 left RF1
                setMoveFinger(RF3,1,1500);//2 both 0 right 1 left RF1

            }//440+300=740
            // 4000ms = 800step
            if(sciTimeCnt == 620)
            {
                wbUBJoint->SetMoveJoint(RSP, -50.f, 600.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(REB, -115.f, 600.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RWY, 45.f, 600.f, MODE_ABSOLUTE);
                wbPosPelZ->setTaskPos(0.03, 600.f, MODE_RELATIVE);
                wbUBJoint->SetMoveJoint(NK1, 0.f, 600.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(NK2, 0.f, 600.f, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == 700)
            {
                setMoveFinger(LF1,-1,700);//2 both 0 right 1 left RF1
                setMoveFinger(RF1,-1,700);//2 both 0 right 1 left RF1
            }

            //2+2
            // 5000ms = 1000step
            if(sciTimeCnt == 860)
            {
                wbUBJoint->SetMoveJoint(RWP, -50.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RSP, -70.f, 800.f, MODE_ABSOLUTE);

                wbUBJoint->SetMoveJoint(LWP, -50.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LSP, -70.f, 800.f, MODE_ABSOLUTE);
                wbPosPelZ->setTaskPos(-0.04, 800.f, MODE_RELATIVE);
                wbUBJoint->SetMoveJoint(NK1, -10.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(NK2, 10.f, 800.f, MODE_ABSOLUTE);
            }
            // 6000ms = 1200step
            if(sciTimeCnt == 1020)
            {
                wbUBJoint->SetMoveJoint(RWP, 0.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RSP, -50.f, 800.f, MODE_ABSOLUTE);

                wbUBJoint->SetMoveJoint(LWP, 0.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LSP, -50.f, 800.f, MODE_ABSOLUTE);
                wbPosPelZ->setTaskPos(0.04, 800.f, MODE_RELATIVE);
                wbUBJoint->SetMoveJoint(NK1, 0.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(NK2, 0.f, 800.f, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == 1040)
            {
                setMoveFinger(LF2,-1,1500);//2 both 0 right 1 left RF1
                setMoveFinger(LF4,1,1500);//2 both 0 right 1 left RF1
                setMoveFinger(LF5,1,1500);//2 both 0 right 1 left RF1
                setMoveFinger(RF2,-1,1500);//2 both 0 right 1 left RF1
                setMoveFinger(RF4,1,1500);//2 both 0 right 1 left RF1
                setMoveFinger(RF5,1,1500);//2 both 0 right 1 left RF1
            }
            // 7000ms = 1400step
            if(sciTimeCnt == 1180)
            {
                wbUBJoint->SetMoveJoint(LSY, -35.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LWY, -65.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RSY, 35.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RWY, 65.f, 800.f, MODE_ABSOLUTE);
            }
            // 7800ms = 1560step
            if(sciTimeCnt == 1340)
            {
                wbUBJoint->SetMoveJoint(LSY, -5.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LWY, -45.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RSY, 5.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RWY, 45.f, 800.f, MODE_ABSOLUTE);
                wbPosPelZ->setTaskPos(-0.04, 800.f, MODE_RELATIVE);
                wbUBJoint->SetMoveJoint(NK1, 10.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(NK2, -10.f, 800.f, MODE_ABSOLUTE);
            }
            // 8600ms = 1720step
            if(sciTimeCnt == 1500)
            {
                wbUBJoint->SetMoveJoint(LSY, -35.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LWY, -65.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RSY, 35.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RWY, 65.f, 800.f, MODE_ABSOLUTE);
                wbPosPelZ->setTaskPos(0.04, 800.f, MODE_RELATIVE);
                wbUBJoint->SetMoveJoint(NK1, 0.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(NK2, 0.f, 800.f, MODE_ABSOLUTE);
            }
            // 9400ms = 1880step
            if(sciTimeCnt == 1660)
            {
                wbUBJoint->SetMoveJoint(LSY, -5.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LWY, -45.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RSY, 5.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RWY, 45.f, 800.f, MODE_ABSOLUTE);
            }
            //10200ms = 2040
            if(sciTimeCnt == 1860)
            {
                wbUBJoint->SetMoveJoint(RSP, 10.f, 1600.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RSR, -10.f, 1600.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RSY, 0.f, 1600.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(REB, -30.f, 1600.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RWY, 0.f, 1600.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RWP, 0.f, 1600.f, MODE_ABSOLUTE);

                wbUBJoint->SetMoveJoint(LSP, 10.f, 1600.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LSR, 10.f, 1600.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LSY, 0.f, 1600.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LEB, -30.f, 1600.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LWY, 0.f, 1600.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LWP, 0.f, 1600.f, MODE_ABSOLUTE);
                setMoveFinger(LF1,1,700);//2 both 0 right 1 left RF1
                setMoveFinger(LF2,1,700);//2 both 0 right 1 left RF1
                setMoveFinger(LF3,-1,700);//2 both 0 right 1 left RF1
                setMoveFinger(LF4,-1,700);//2 both 0 right 1 left RF1
                setMoveFinger(LF5,-1,700);//2 both 0 right 1 left RF1
                setMoveFinger(RF1,1,700);//2 both 0 right 1 left RF1
                setMoveFinger(RF2,1,700);//2 both 0 right 1 left RF1
                setMoveFinger(RF3,-1,700);//2 both 0 right 1 left RF1
                setMoveFinger(RF4,-1,700);//2 both 0 right 1 left RF1
                setMoveFinger(RF5,-1,700);//2 both 0 right 1 left RF1
            }
            if(sciTimeCnt == 2300)
                initHeadHand(1);
            if(sciTimeCnt == 2800)
            {
                sciMotionSelector = NO_ACT;
                printf("\n ==========time : %f \n",sciTimeCnt*0.005);
                printf(">>> Scene-21 motion end\n");
            }
            break;
        case 22:
            //goto one foot pos
            if(sciTimeCnt ==0)
            {
                Initial_COM[1]=Hubo2->CurTaskPos.pCOM_3x1[1];
                Initial_COM[2]=Hubo2->CurTaskPos.pCOM_3x1[2];

                wbUBJoint->SetMoveJoint(RSR, -20.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LSR, 20.f, 400.f, MODE_ABSOLUTE);
                setMoveFinger(2,1,700);//2 both 0 right 1 left RF1
            }
            if(sciTimeCnt == 100)
            {
                setMoveCOM(Hubo2->CurTaskPos.pLF_LP_3x1[1], Hubo2->CurTaskPos.pLF_LP_3x1[2], 1500, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == 480)
            {
                setMoveLeg(HUBO_RIGHT, 0.10, 0, 0.075, NULL, 1500, MODE_RELATIVE);
            }
            //down up
            if(sciTimeCnt == 900)
            {
                wbPosPelZ->setTaskPos(-0.05, 1300.f, MODE_RELATIVE);
                setMoveLeg(HUBO_RIGHT, 0.05, 0, 0.02, NULL, 1300, MODE_RELATIVE);
            }
            if(sciTimeCnt == 1200)
            {
                wbPosPelZ->setTaskPos(0.05, 1300.f, MODE_RELATIVE);
                setMoveLeg(HUBO_RIGHT, -0.05, 0, -0.02, NULL, 1300, MODE_RELATIVE);
            }
            //go back to ground
            if(sciTimeCnt == 1500)
            {
                setMoveLeg(HUBO_RIGHT, -0.10, 0, -0.072, NULL, 1600, MODE_RELATIVE);
            }
            if(sciTimeCnt == 1850)
            {
                setMoveCOM(Initial_COM[1], Initial_COM[2], 1800, MODE_ABSOLUTE);
                setMoveLeg(HUBO_RIGHT, 0, 0, -0.003, NULL, 1800, MODE_RELATIVE);
            }
            //up arm
            if(sciTimeCnt == 2400)//490
            {
                wbUBJoint->SetMoveJoint(RSP, 0.f, 1500.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RSR, -110.f, 1500.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RSY, -90.f, 1500.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(REB, -85.f, 1500.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RWY, -90.f, 1500.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RWP, 0.f, 1500.f, MODE_ABSOLUTE);

                wbUBJoint->SetMoveJoint(LSP, 0.f, 1500.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LSR, 110.f, 1500.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LSY, 90.f, 1500.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LEB, -85.f, 1500.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LWY, 90.f, 1500.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LWP, 0.f, 1500.f, MODE_ABSOLUTE);

                wbUBJoint->SetMoveJoint(NK1, -10.f, 1500.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(NK2, 10.f, 1500.f, MODE_ABSOLUTE);

                setMoveFinger(2,-1,600);//2 both 0 right 1 left RF1
            }
            if(sciTimeCnt == 2750)
                wbPosWST->setTaskPos(-35.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(NKY, 35.f, 800.f, MODE_ABSOLUTE);
            if(sciTimeCnt==2910)
                wbPosWST->setTaskPos(35.f, 1200.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(NKY, -35.f, 1200.f, MODE_ABSOLUTE);
            if(sciTimeCnt==3150)
                wbPosWST->setTaskPos(0.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(NKY, 0.f, 800.f, MODE_ABSOLUTE);

            if(sciTimeCnt == 3350)
            {
                wbUBJoint->SetMoveJoint(RSP, 10.f, 1600.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RSR, -10.f, 1600.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RSY, 0.f, 1600.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(REB, -30.f, 1600.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RWY, 0.f, 1600.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RWP, 0.f, 1600.f, MODE_ABSOLUTE);

                wbUBJoint->SetMoveJoint(LSP, 10.f, 1600.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LSR, 10.f, 1600.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LSY, 0.f, 1600.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LEB, -30.f, 1600.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LWY, 0.f, 1600.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LWP, 0.f, 1600.f, MODE_ABSOLUTE);

                wbUBJoint->SetMoveJoint(NK1, 0.f, 1600.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(NK2, 0.f, 1600.f, MODE_ABSOLUTE);
            }

            if(sciTimeCnt == 3750)
                initHeadHand(1);//0 head . 1 hand . 2 hand+head

            //end task
            if(sciTimeCnt==4100)
            {
                sciMotionSelector = NO_ACT;
                printf("\n ==========time : %f \n",sciTimeCnt*0.005);
                printf(">>> Scene-22 motion end\n");
            }
            break;


        case 23:
            if(sciTimeCnt == 0)
            {
                wbUBJoint->SetMoveJoint(RSP, 25.f, 700.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RSR, -45.f, 700.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RSY, 40.f, 700.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(REB, -80.f, 700.f, MODE_ABSOLUTE);

                wbUBJoint->SetMoveJoint(LSP, 25.f, 700.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LSR, 45.f, 700.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LSY, -40.f, 700.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LEB, -80.f, 700.f, MODE_ABSOLUTE);
                setMoveFinger(2,-1,700);//2 both 0 right 1 left RF1
            }
            if(sciTimeCnt == 440)
            {
                wbUBJoint->SetMoveJoint(RSP, 10.f, 700.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RSR, -10.f, 700.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RSY, 0.f, 700.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(REB, -30.f, 700.f, MODE_ABSOLUTE);

                wbUBJoint->SetMoveJoint(LSP, 10.f, 700.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LSR, 10.f, 700.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LSY, 0.f, 700.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LEB, -30.f, 700.f, MODE_ABSOLUTE);
                setMoveFinger(2,1,700);//2 both 0 right 1 left RF1
            }

            if(sciTimeCnt == 140)
                wbUBJoint->SetMoveJoint(NKY, 20.f, 500.f, MODE_ABSOLUTE);
            if(sciTimeCnt == 240)
                wbUBJoint->SetMoveJoint(NKY, -20.f, 500.f, MODE_ABSOLUTE);
            if(sciTimeCnt == 340)
                wbUBJoint->SetMoveJoint(NKY, 20.f, 500.f, MODE_ABSOLUTE);
            if(sciTimeCnt == 440)
                wbUBJoint->SetMoveJoint(NKY, 0.f, 500.f, MODE_ABSOLUTE);


            if(sciTimeCnt == 600)
            {
                sciMotionSelector = NO_ACT;
                printf("\n ==========time : %f \n",sciTimeCnt*0.005);
                printf(">>> Scene-23 motion end\n");
            }
            break;
        case 24:
            if(sciTimeCnt==0)
            {
                wbUBJoint->SetMoveJoint(RSP, -40.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RSR, 0.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RSY, 25.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(REB, -90.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RWY, -40.f, 800.f, MODE_ABSOLUTE);

                wbUBJoint->SetMoveJoint(LSP, -40.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LSR, 0.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LSY, -25.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LEB, -90.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LWY, 40.f, 800.f, MODE_ABSOLUTE);
                setMoveFinger(2,1,700);//2 both 0 right 1 left RF1
            }
            else if(sciTimeCnt==200)//go up
            {
                wbUBJoint->SetMoveJoint(RSP, -45.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(REB, -110.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LSP, -45.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LEB, -110.f, 400.f, MODE_ABSOLUTE);

                wbUBJoint->SetMoveJoint(NK1, -13.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(NK2, -13.f, 400.f, MODE_ABSOLUTE);
            }
            else if(sciTimeCnt==280)
            {
                wbUBJoint->SetMoveJoint(RSP, -40.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(REB, -90.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LSP, -40.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LEB, -90.f, 400.f, MODE_ABSOLUTE);

                wbUBJoint->SetMoveJoint(NK1, 0.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(NK2, 0.f, 400.f, MODE_ABSOLUTE);
            }
            else if(sciTimeCnt==360)//go up
            {
                wbUBJoint->SetMoveJoint(RSP, -45.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(REB, -110.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LSP, -45.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LEB, -110.f, 400.f, MODE_ABSOLUTE);

                wbUBJoint->SetMoveJoint(NK1, -13.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(NK2, -13.f, 400.f, MODE_ABSOLUTE);
            }
            else if(sciTimeCnt==440)
            {
                wbUBJoint->SetMoveJoint(RSP, -40.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(REB, -90.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LSP, -40.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LEB, -90.f, 400.f, MODE_ABSOLUTE);

                wbUBJoint->SetMoveJoint(NK1, 0.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(NK2, 0.f, 400.f, MODE_ABSOLUTE);
            }
            else if(sciTimeCnt==520)//go up
            {
                wbUBJoint->SetMoveJoint(RSP, -45.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(REB, -110.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LSP, -45.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LEB, -110.f, 400.f, MODE_ABSOLUTE);

                wbUBJoint->SetMoveJoint(NK1, -13.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(NK2, -13.f, 400.f, MODE_ABSOLUTE);
            }
            else if(sciTimeCnt==600)
            {
                wbUBJoint->SetMoveJoint(RSP, -40.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(REB, -90.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LSP, -40.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LEB, -90.f, 400.f, MODE_ABSOLUTE);
            }
            else if(sciTimeCnt==900)
            {
                wbUBJoint->SetMoveJoint(RSP, 10.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RSR, -10.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RSY, 0.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(REB, -30.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RWY, 0.f, 800.f, MODE_ABSOLUTE);

                wbUBJoint->SetMoveJoint(LSP, 10.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LSR, 10.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LSY, 0.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LEB, -30.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LWY, 0.f, 800.f, MODE_ABSOLUTE);

                wbUBJoint->SetMoveJoint(NK1, 0.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(NK2, 0.f, 800.f, MODE_ABSOLUTE);
                setMoveFinger(2,-1,600);//2 both 0 right 1 left RF1
            }


            if(sciTimeCnt == 520)
            {
                wbUBJoint->SetMoveJoint(NKY, 20.f, 600.f, MODE_ABSOLUTE);
            }
            else if(sciTimeCnt==640)
            {
                wbUBJoint->SetMoveJoint(NKY, -20.f, 600.f, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == 760)
            {
                wbUBJoint->SetMoveJoint(NKY, 20.f, 600.f, MODE_ABSOLUTE);
            }
            else if(sciTimeCnt==880)
            {
                wbUBJoint->SetMoveJoint(NKY, 0.f, 600.f, MODE_ABSOLUTE);
            }

            if(sciTimeCnt==1100)
                initHeadHand(1);//0 head . 1 hand . 2 hand+head

            if(sciTimeCnt==1350)
            {
                sciMotionSelector = NO_ACT;
                printf("\n ==========time : %f \n",sciTimeCnt*0.005);
                printf(">>> Scene-24 motion end\n");
            }
            break;
        case 25:
            if(fileReadFlag == ENABLE)
            {
                LeadTextData();
            }
            ChaseTextData();
            if(ringIndexBack == ringIndexFront)
            {
                sciMotionSelector = NO_ACT;
                printf("\n ==========time : %f \n",sciTimeCnt*0.005);
                printf(">>> Scene-25 motion end\n");
            }
            break;
        case 26:
            if(sciTimeCnt ==0)
                setMoveFinger(2,-1,1000);//2 both 0 right 1 left RF1

            if(sciTimeCnt == 140)
            {
                wbUBJoint->SetMoveJoint(RSP, 20.f, 350.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RSR, -30.f, 350.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RSY, 30.f, 350.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(REB, -60.f, 350.f, MODE_ABSOLUTE);

                wbUBJoint->SetMoveJoint(LSP, 20.f, 350.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LSR, 30.f, 350.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LSY, -30.f, 350.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LEB, -60.f, 350.f, MODE_ABSOLUTE);
            }
            else if(sciTimeCnt == 210)
            {
                wbUBJoint->SetMoveJoint(RSP, 20.f, 350.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RSR, -15.f, 350.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RSY, 20.f, 350.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(REB, -60.f, 350.f, MODE_ABSOLUTE);

                wbUBJoint->SetMoveJoint(LSP, 20.f, 350.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LSR, 15.f, 350.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LSY, -20.f, 350.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LEB, -60.f, 350.f, MODE_ABSOLUTE);
            }
            else if(sciTimeCnt == 280)
            {
                wbUBJoint->SetMoveJoint(RSP, 20.f, 350.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RSR, -30.f, 350.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RSY, 30.f, 350.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(REB, -60.f, 350.f, MODE_ABSOLUTE);

                wbUBJoint->SetMoveJoint(LSP, 20.f, 350.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LSR, 30.f, 350.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LSY, -30.f, 350.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LEB, -60.f, 350.f, MODE_ABSOLUTE);
            }
            else if(sciTimeCnt == 350)
            {
                wbUBJoint->SetMoveJoint(RSP, 10.f, 350.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RSR, -10.f, 350.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RSY, 0.f, 350.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(REB, -30.f, 350.f, MODE_ABSOLUTE);

                wbUBJoint->SetMoveJoint(LSP, 10.f, 350.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LSR, 10.f, 350.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LSY, 0.f, 350.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LEB, -30.f, 350.f, MODE_ABSOLUTE);
                setMoveFinger(2,1,1000);//2 both 0 right 1 left RF1
            }
            if(sciTimeCnt==500)
                initHeadHand(1);//0 head . 1 hand . 2 hand+head
            if(sciTimeCnt==850)
            {
                sciMotionSelector = NO_ACT;
                printf("\n ==========time : %f \n",sciTimeCnt*0.005);
                printf(">>> Scene-26 motion end\n");
            }
            break;
        case 28:
            if(sciTimeCnt == 0)
            {
                temp_ori[0]=wbOriPel->RefCurrent[0];
                temp_ori[1]=wbOriPel->RefCurrent[1];
                temp_ori[2]=wbOriPel->RefCurrent[2];
                temp_ori[3]=wbOriPel->RefCurrent[3];
                temp_ori[4]=wbOriPel->RefCurrent[4];
                float **mat_temp1 = alloc_matrix(4,4);
                float vec_temp[5],vec_temp1[5],vec_temp2[5];
                RY(20*D2R,mat_temp1);

                DC2QT((const float**)mat_temp1,vec_temp);
                QT2RV(vec_temp,vec_temp1);
                vec_temp2[1] = vec_temp1[0];
                vec_temp2[2] = vec_temp1[1];
                vec_temp2[3] = vec_temp1[2];
                vec_temp2[4] = vec_temp1[3];

                wbOriPel->setTaskOri(vec_temp2, 1200, MODE_ABSOLUTE);
                wbPosWST->setTaskPos(-20, 900, MODE_RELATIVE);
                wbUBJoint->SetMoveJoint(LSP, -75.f, 900.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LSR,  10.f, 900.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LSY, -45.f, 900.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LEB, -90.f, 900.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(NKY, -60.f, 900.f, MODE_ABSOLUTE);
                setMoveFinger(1,1,1000);//2 both 0 right 1 left RF1
                delete_matrix(mat_temp1,4,4);
            }
            else if(sciTimeCnt == 500)
            {
                wbOriPel->setTaskOri(temp_ori, 1200, MODE_ABSOLUTE);
                wbPosWST->setTaskPos(20, 900, MODE_RELATIVE);
                Setmove_walk_ready(900);
                setMoveFinger(1,-1,1000);//2 both 0 right 1 left RF1
            }

            if(sciTimeCnt == 1000)
                initHeadHand(1);//0 head . 1 hand . 2 hand+head

            if(sciTimeCnt == 1500)
            {
                sciMotionSelector = NO_ACT;
                printf("\n ==========time : %f \n",sciTimeCnt*0.005);
            }
            break;
        case 2801:
            if(sciTimeCnt == 0)
            {
                wbUBJoint->SetMoveJoint(RSP, -55.f, 900.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RSR, 0.f, 900.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RSY, -30.f, 900.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(REB, -15.f, 900.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RWY, -70.f, 900.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RWP, 0.f, 900.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(NK1, 10.f, 900.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(NK2, -10.f, 900.f, MODE_ABSOLUTE);
                setMoveFinger(0,1,1000);//2 both 0 right 1 left RF1
            }
            else if(sciTimeCnt == 380)
            {
                wbUBJoint->SetMoveJoint(RSP, 10.f, 900.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RSR, -10.f, 900.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RSY, 0.f, 900.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(REB, -30.f, 900.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RWY, 0.f, 900.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RWP, 0.f, 900.f, MODE_ABSOLUTE);
                setMoveFinger(0,-1,1000);//2 both 0 right 1 left RF1
                wbUBJoint->SetMoveJoint(NK1, 0.f, 900.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(NK2, 0.f, 900.f, MODE_ABSOLUTE);
            }

            if(sciTimeCnt == 600)
                initHeadHand(1);//0 head . 1 hand . 2 hand+head

            if(sciTimeCnt==1000)
            {
                sciMotionSelector = NO_ACT;
                printf("\n ==========time : %f \n",sciTimeCnt*0.005);
                printf(">>> Scene-28 motion end\n");
            }
            break;

        case 29:
            if(sciTimeCnt==0)
            {
                wbUBJoint->SetMoveJoint(RSP, -30.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RSR, -15.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RSY, 40.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(REB, -100.f, 800.f, MODE_ABSOLUTE);//-120
                wbUBJoint->SetMoveJoint(RWY, -40.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RWP, 0.f, 800.f, MODE_ABSOLUTE);

                setMoveFinger(0,1,1000);//2 both 0 right 1 left RF1
                wbUBJoint->SetMoveJoint(NK1, -12.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(NK2, 12.f, 800.f, MODE_ABSOLUTE);
            }
            else if(sciTimeCnt == 360)
            {
                wbUBJoint->SetMoveJoint(RSP, -30.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RSR, 0.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RSY, 0.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(REB, -50.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RWY, -55.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RWP, 0.f, 800.f, MODE_ABSOLUTE);

                wbUBJoint->SetMoveJoint(LSP, 0.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LSR, 20.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LSY, -50.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LEB, -70.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LWY, 0.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LWP, 0.f, 800.f, MODE_ABSOLUTE);
                setMoveFinger(1,-1,1000);//2 both 0 right 1 left RF1
                wbUBJoint->SetMoveJoint(NK1, 12.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(NK2, -12.f, 800.f, MODE_ABSOLUTE);
            }
            else if(sciTimeCnt == 620)
            {
                wbUBJoint->SetMoveJoint(RSP, -35.f, 600.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RSR, 10.f, 600.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RSY, 60.f, 600.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(REB, -75.f, 600.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RWY, 35.f, 600.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RWP, 0.f, 600.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(NK1, 0.f, 600.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(NK2, 0.f, 600.f, MODE_ABSOLUTE);

                wbUBJoint->SetMoveJoint(LSP, -30.f, 600.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LSR, -10.f, 600.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LSY, -60.f, 600.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LEB, -50.f, 600.f, MODE_ABSOLUTE);//
                wbUBJoint->SetMoveJoint(LWY, 10.f, 600.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LWP, 5.f, 600.f, MODE_ABSOLUTE);
            }
            // 33 ready finished
            else if(sciTimeCnt == 740)//go out
            {
                wbUBJoint->SetMoveJoint(RSP, -50.f, 550.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RSR, 0.f, 550.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RSY, 65.f, 550.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(REB, -55.f, 550.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RWY, 45.f, 550.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RWP, 0.f, 550.f, MODE_ABSOLUTE);
            }
            else if(sciTimeCnt == 850)//go ready
            {
                wbUBJoint->SetMoveJoint(RSP, -35.f, 550.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RSR, 10.f, 550.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RSY, 60.f, 550.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(REB, -75.f, 550.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RWY, 35.f, 550.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RWP, 0.f, 550.f, MODE_ABSOLUTE);
            }
            else if(sciTimeCnt == 960)//go out
            {
                wbUBJoint->SetMoveJoint(RSP, -50.f, 550.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RSR, 0.f, 550.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RSY, 65.f, 550.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(REB, -55.f, 550.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RWY, 45.f, 550.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RWP, 0.f, 550.f, MODE_ABSOLUTE);
            }
            else if(sciTimeCnt == 1070)//go ready
            {
                wbUBJoint->SetMoveJoint(RSP, -35.f, 550.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RSR, 10.f, 550.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RSY, 60.f, 550.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(REB, -75.f, 550.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RWY, 35.f, 550.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RWP, 0.f, 550.f, MODE_ABSOLUTE);
            }
            else if(sciTimeCnt == 1180)//go out
            {
                wbUBJoint->SetMoveJoint(RSP, -50.f, 550.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RSR, 0.f, 550.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RSY, 65.f, 550.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(REB, -55.f, 550.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RWY, 45.f, 550.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RWP, 0.f, 550.f, MODE_ABSOLUTE);
            }
            else if(sciTimeCnt == 1290)//go ready
            {
                wbUBJoint->SetMoveJoint(RSP, -35.f, 550.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RSR, 10.f, 550.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RSY, 60.f, 550.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(REB, -75.f, 550.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RWY, 35.f, 550.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RWP, 0.f, 550.f, MODE_ABSOLUTE);
            }

            // goto walk ready pos
            else if(sciTimeCnt ==1400)
            {
                wbUBJoint->SetMoveJoint(RSP, 10.f, 1200.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RSR, -10.f, 1200.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RSY, 0.f, 1200.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(REB, -30.f, 1200.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RWY, 0.f, 1200.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RWP, 0.f, 1200.f, MODE_ABSOLUTE);

                wbUBJoint->SetMoveJoint(LSP, 10.f, 1200.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LSR, 10.f, 1200.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LSY, 0.f, 1200.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LEB, -30.f, 1200.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LWY, 0.f, 1200.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LWP, 0.f, 1200.f, MODE_ABSOLUTE);
                setMoveFinger(0,-1,1000);//2 both 0 right 1 left RF1
                setMoveFinger(1,1,1000);//2 both 0 right 1 left RF1
            }
            if(sciTimeCnt==1740)
                initHeadHand(1);//0 head . 1 hand . 2 hand+head

            //end task
            if(sciTimeCnt==2120)
            {
                sciMotionSelector = NO_ACT;
                printf("\n ==========time : %f \n",sciTimeCnt*0.005);
                printf(">>> Scene-29 motion end\n");
            }
            break;
        case 30:
            if(sciTimeCnt == 0)
            {
                wbUBJoint->SetMoveJoint(LSP, -50.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LSR, 0.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LEB, -110.f, 800.f, MODE_ABSOLUTE);
            }
            else if(sciTimeCnt == 360)
            {
                wbUBJoint->SetMoveJoint(LSP, -110.f, 600.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LSR, 0.f, 600.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LEB, -20.f, 600.f, MODE_ABSOLUTE);
                setMoveFinger(1,1,1000);//2 both 0 right 1 left RF1
            }
            else if(sciTimeCnt == 580)
            {
                wbUBJoint->SetMoveJoint(LSP, 10.f, 1000.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LSR, 10.f, 1000.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LEB, -30.f, 1000.f, MODE_ABSOLUTE);
                setMoveFinger(1,-1,700);//2 both 0 right 1 left RF1
            }

            if(sciTimeCnt==800)
                initHeadHand(1);//0 head . 1 hand . 2 hand+head

            if(sciTimeCnt==1100)
            {
                sciMotionSelector = NO_ACT;
                printf("\n ==========time : %f \n",sciTimeCnt*0.005);
                printf(">>> Scene-30 motion end\n");
            }
            break;
        case 31:
            if(sciTimeCnt == 0)
            {
                wbUBJoint->SetMoveJoint(LSP, 10.f, 700.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LSR, 45.f, 700.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LSY, -70.f, 700.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LEB, -70.f, 700.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LWY, -20.f, 700.f, MODE_ABSOLUTE);

                wbUBJoint->SetMoveJoint(RSP, 10.f, 700.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RSR, -45.f, 700.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RSY, 70.f, 700.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(REB, -70.f, 700.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RWY, 20.f, 700.f, MODE_ABSOLUTE);
                setMoveFinger(2,-1,1000);//2 both 0 right 1 left RF1
            }
            else if(sciTimeCnt == 300)
            {
                wbUBJoint->SetMoveJoint(LSP, 10.f, 700.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LSR, 10.f, 700.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LSY, 0.f, 700.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LEB, -30.f, 700.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LWY, 0.f, 700.f, MODE_ABSOLUTE);

                wbUBJoint->SetMoveJoint(RSP, 10.f, 700.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RSR, -10.f, 700.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RSY, 0.f, 700.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(REB, -30.f, 700.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RWY, 0.f, 700.f, MODE_ABSOLUTE);
                setMoveFinger(2,1,1000);//2 both 0 right 1 left RF1
            }

            if(sciTimeCnt == 150)
            {
                wbPosWST->setTaskPos(30.f, 700.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(NKY, 15.f, 700.f, MODE_ABSOLUTE);
            }
            else if(sciTimeCnt == 300)
            {
                wbPosWST->setTaskPos(0.f, 700.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(NKY, 0.f, 700.f, MODE_ABSOLUTE);
            }

            if(sciTimeCnt==500)
            {
                sciMotionSelector = NO_ACT;
                printf("\n ==========time : %f \n",sciTimeCnt*0.005);
                printf(">>> Scene-31 motion end\n");
            }
            break;

        case 32:
            if(sciTimeCnt == 0)
                setMoveFinger(2,-1,700);//2 both 0 right 1 left RF1
            if(sciTimeCnt == 140)
            {
                wbUBJoint->SetMoveJoint(RSP, 20.f, 350.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RSR, -30.f, 350.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RSY, 30.f, 350.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(REB, -60.f, 350.f, MODE_ABSOLUTE);

                wbUBJoint->SetMoveJoint(LSP, 20.f, 350.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LSR, 30.f, 350.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LSY, -30.f, 350.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LEB, -60.f, 350.f, MODE_ABSOLUTE);

            }
            else if(sciTimeCnt == 210)
            {
                wbUBJoint->SetMoveJoint(RSP, 20.f, 350.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RSR, -15.f, 350.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RSY, 20.f, 350.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(REB, -60.f, 350.f, MODE_ABSOLUTE);

                wbUBJoint->SetMoveJoint(LSP, 20.f, 350.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LSR, 15.f, 350.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LSY, -20.f, 350.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LEB, -60.f, 350.f, MODE_ABSOLUTE);
            }
            else if(sciTimeCnt == 280)
            {
                wbUBJoint->SetMoveJoint(RSP, 20.f, 350.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RSR, -30.f, 350.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RSY, 30.f, 350.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(REB, -60.f, 350.f, MODE_ABSOLUTE);

                wbUBJoint->SetMoveJoint(LSP, 20.f, 350.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LSR, 30.f, 350.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LSY, -30.f, 350.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LEB, -60.f, 350.f, MODE_ABSOLUTE);
            }
            else if(sciTimeCnt == 350)
            {
                wbUBJoint->SetMoveJoint(RSP, 10.f, 350.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RSR, -10.f, 350.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RSY, 0.f, 350.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(REB, -30.f, 350.f, MODE_ABSOLUTE);

                wbUBJoint->SetMoveJoint(LSP, 10.f, 350.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LSR, 10.f, 350.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LSY, 0.f, 350.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LEB, -30.f, 350.f, MODE_ABSOLUTE);
            }

            // draw love

            if(sciTimeCnt == 490)
            {
                setMoveFinger(2,1,1000);//2 both 0 right 1 left RF1
                wbUBJoint->SetMoveJoint(RSP, 0.f, 1500.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RSR, -110.f, 1500.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RSY, -90.f, 1500.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(REB, -85.f, 1500.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RWY, -90.f, 1500.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RWP, 0.f, 1500.f, MODE_ABSOLUTE);

                wbUBJoint->SetMoveJoint(LSP, 0.f, 1500.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LSR, 110.f, 1500.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LSY, 90.f, 1500.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LEB, -85.f, 1500.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LWY, 90.f, 1500.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LWP, 0.f, 1500.f, MODE_ABSOLUTE);

                wbUBJoint->SetMoveJoint(NK1, -10.f, 1500.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(NK2, 10.f, 1500.f, MODE_ABSOLUTE);
            }
            else if(sciTimeCnt == 890)
            {
                wbUBJoint->SetMoveJoint(RSP, 10.f, 1600.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RSR, -10.f, 1600.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RSY, 0.f, 1600.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(REB, -30.f, 1600.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RWY, 0.f, 1600.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RWP, 0.f, 1600.f, MODE_ABSOLUTE);

                wbUBJoint->SetMoveJoint(LSP, 10.f, 1600.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LSR, 10.f, 1600.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LSY, 0.f, 1600.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LEB, -30.f, 1600.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LWY, 0.f, 1600.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LWP, 0.f, 1600.f, MODE_ABSOLUTE);

                wbUBJoint->SetMoveJoint(NK1, 0.f, 1600.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(NK2, 0.f, 1600.f, MODE_ABSOLUTE);
            }

            if(sciTimeCnt==1340)
            {
                sciMotionSelector = NO_ACT;
                printf("\n ==========time : %f \n",sciTimeCnt*0.005);
                printf(">>> Scene-32 motion end\n");
            }
            break;
        case 33://SCENE #33 - Greeting with arm shake
            if(fileReadFlag == ENABLE)
            {
                LeadTextData();
            }
            ChaseTextData();
            if(ringIndexBack == ringIndexFront)
            {
                sciMotionSelector = NO_ACT;
                printf("\n ==========time : %f \n",sciTimeCnt*0.005);
                printf(">>> Scene-33 motion end\n");
            }
            break;
        case 8801:
            if(sciTimeCnt == 0)
            {
                wbUBJoint->SetMoveJoint(NK1, 15.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(NK2, 15.f, 400.f, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == 80)
            {
                wbUBJoint->SetMoveJoint(NK1, -15.f, 500.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(NK2, -15.f, 500.f, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == 180)
            {
                wbUBJoint->SetMoveJoint(NK1, 15.f, 500.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(NK2, 15.f, 500.f, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == 280)
            {
                wbUBJoint->SetMoveJoint(NK1, -15.f, 500.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(NK2, -15.f, 500.f, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == 380)
            {
                wbUBJoint->SetMoveJoint(NK1, 0.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(NK2, 0.f, 400.f, MODE_ABSOLUTE);
            }
            if(sciTimeCnt==500)
            {
                sciMotionSelector = NO_ACT;
                printf("\n ==========time : %f \n",sciTimeCnt*0.005);
                printf(">>> Scene-open motion# 1 end\n");
            }
            break;
        case 8802:
            if(sciTimeCnt == 0)
            {
                wbUBJoint->SetMoveJoint(NKY, -25.f, 400.f, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == 80)
            {
                wbUBJoint->SetMoveJoint(NKY, 25.f, 500.f, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == 180)
            {
                wbUBJoint->SetMoveJoint(NKY, -25.f, 500.f, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == 280)
            {
                wbUBJoint->SetMoveJoint(NKY, 25.f, 500.f, MODE_ABSOLUTE);
            }
            if(sciTimeCnt==380)
            {
                wbUBJoint->SetMoveJoint(NKY, 0.f, 400.f, MODE_ABSOLUTE);
            }
            if(sciTimeCnt==500)
            {
                sciMotionSelector = NO_ACT;
                printf("\n ==========time : %f \n",sciTimeCnt*0.005);
                printf(">>> Scene-open motion# 2 end\n");
            }
            break;
        case 8803:
            if(sciTimeCnt == 0)
            {
                wbUBJoint->SetMoveJoint(NK1, 15.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(NK2, 15.f, 400.f, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == 80)
            {
                wbUBJoint->SetMoveJoint(NK1, -15.f, 500.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(NK2, -15.f, 500.f, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == 180)
            {
                wbUBJoint->SetMoveJoint(NK1, 15.f, 500.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(NK2, 15.f, 500.f, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == 280)
            {
                wbUBJoint->SetMoveJoint(NK1, -15.f, 500.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(NK2, -15.f, 500.f, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == 380)
            {
                wbUBJoint->SetMoveJoint(NK1, 0.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(NK2, 0.f, 400.f, MODE_ABSOLUTE);
            }
            if(sciTimeCnt==500)
            {
                sciMotionSelector = NO_ACT;
                printf("\n ==========time : %f \n",sciTimeCnt*0.005);
                printf(">>> Scene-open motion# 3 end\n");
            }
            break;
        case 101:
        {

            t1_101_int = (int)(t1_101/RT_TIMER_PERIOD_MS);
//            if(sciTimeCnt == 0)
//            {
//                z_101 = 0.04;
//                setMoveFinger(RF1,-1,700);//2 both 0 right 1 left
//                setMoveFinger(RF2,1,700);//2 both 0 right 1 left
//                setMoveFinger(RF3,-1,700);//2 both 0 right 1 left
//                setMoveFinger(RF4,-1,700);//2 both 0 right 1 left
//                setMoveFinger(RF5,-1,700);//2 both 0 right 1 left
//                wbUBJoint->SetMoveJoint(RSP, -135.f, t1_101, MODE_ABSOLUTE);
//                wbUBJoint->SetMoveJoint(RSR, 20.f, t1_101, MODE_ABSOLUTE);
////                wbUBJoint->SetMoveJoint(REB, -10.f, t1_101, MODE_ABSOLUTE);
//            }
            //Pelvis move + Left move =========================================================================================================
            if(sciTimeCnt == 0 + music_offset)
            {
                z_101 = 0.05;
                wbPosPelZ->setTaskPos(-z_101, bit_int*RT_TIMER_PERIOD_MS, MODE_RELATIVE);
                wbUBJoint->SetMoveJoint(LSR, 25.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LSY,-25.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LEB,-50.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(NK1, 15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(NK2, 15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == bit_int + music_offset)
            {
                wbPosPelZ->setTaskPos(z_101, bit_int*RT_TIMER_PERIOD_MS, MODE_RELATIVE);
                wbUBJoint->SetMoveJoint(LSR, 10.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LSY,  0.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LEB,-30.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(NK1,-15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(NK2,-15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == bit_int*2 + music_offset)
            {
                wbPosPelZ->setTaskPos(-z_101, bit_int*RT_TIMER_PERIOD_MS, MODE_RELATIVE);
                wbUBJoint->SetMoveJoint(LSR, 25.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LSY,-25.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LEB,-50.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(NK1, 15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(NK2, 15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == bit_int*3 + music_offset)
            {
                wbPosPelZ->setTaskPos(z_101, bit_int*RT_TIMER_PERIOD_MS, MODE_RELATIVE);
                wbUBJoint->SetMoveJoint(LSR, 10.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LSY,  0.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LEB,-30.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(NK1,-15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(NK2,-15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == bit_int*4 + music_offset)
            {
                wbPosPelZ->setTaskPos(-z_101, bit_int*RT_TIMER_PERIOD_MS, MODE_RELATIVE);
                wbUBJoint->SetMoveJoint(LSR, 25.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LSY,-25.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LEB,-50.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(NK1, 15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(NK2, 15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == bit_int*5 + music_offset)
            {
                wbPosPelZ->setTaskPos(z_101, bit_int*RT_TIMER_PERIOD_MS, MODE_RELATIVE);
                wbUBJoint->SetMoveJoint(LSR, 10.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LSY,  0.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LEB,-30.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(NK1,-15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(NK2,-15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == bit_int*6 + music_offset)
            {
                wbPosPelZ->setTaskPos(-z_101, bit_int*RT_TIMER_PERIOD_MS, MODE_RELATIVE);
                wbUBJoint->SetMoveJoint(LSR, 25.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LSY,-25.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LEB,-50.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(NK1, 15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(NK2, 15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == bit_int*7 + music_offset)
            {
                wbPosPelZ->setTaskPos(z_101, bit_int*RT_TIMER_PERIOD_MS, MODE_RELATIVE);
                wbUBJoint->SetMoveJoint(LSR, 10.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LSY,  0.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LEB,-30.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(NK1,-15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(NK2,-15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
            }
            //Pelvis move + Left move + Shake Right=========================================================================================================
            if(sciTimeCnt == bit_int*8 + music_offset)
            {
                wbPosPelZ->setTaskPos(-z_101, bit_int*RT_TIMER_PERIOD_MS, MODE_RELATIVE);
                wbUBJoint->SetMoveJoint(RSP, -120.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
//                wbUBJoint->SetMoveJoint(RSR, -15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(REB,-70.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LSR, 25.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LSY,-25.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LEB,-50.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(NK1, 15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(NK2, 15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == bit_int*9 + music_offset)
            {
                wbPosPelZ->setTaskPos(z_101, bit_int*RT_TIMER_PERIOD_MS, MODE_RELATIVE);
                wbUBJoint->SetMoveJoint(RSP, -130.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
//                wbUBJoint->SetMoveJoint(RSR, 0.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(REB,-10.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LSR, 10.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LSY,  0.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LEB,-30.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(NK1,-15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(NK2,-15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == bit_int*10 + music_offset)
            {
                wbPosPelZ->setTaskPos(-z_101, bit_int*RT_TIMER_PERIOD_MS, MODE_RELATIVE);
                wbUBJoint->SetMoveJoint(RSP, -120.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
//                wbUBJoint->SetMoveJoint(RSR, -15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(REB, -70.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LSR, 25.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LSY,-25.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LEB,-50.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(NK1, 15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(NK2, 15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == bit_int*11 + music_offset)
            {
                wbPosPelZ->setTaskPos(z_101, bit_int*RT_TIMER_PERIOD_MS, MODE_RELATIVE);
                wbUBJoint->SetMoveJoint(RSP, -130.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
//                wbUBJoint->SetMoveJoint(RSR, 0.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(REB, -10.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LSR, 10.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LSY,  0.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LEB,-30.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(NK1,-15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(NK2,-15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == bit_int*12 + music_offset)
            {
                wbPosPelZ->setTaskPos(-z_101, bit_int*RT_TIMER_PERIOD_MS, MODE_RELATIVE);
                wbUBJoint->SetMoveJoint(RSP, -120.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
//                wbUBJoint->SetMoveJoint(RSR, -15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(REB,-70.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LSR, 25.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LSY,-25.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LEB,-50.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(NK1, 15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(NK2, 15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == bit_int*13 + music_offset)
            {
                wbPosPelZ->setTaskPos(z_101, bit_int*RT_TIMER_PERIOD_MS, MODE_RELATIVE);
                wbUBJoint->SetMoveJoint(RSP, -130.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
//                wbUBJoint->SetMoveJoint(RSR, 0.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(REB, -10.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LSR, 10.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LSY,  0.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LEB,-30.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(NK1,-15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(NK2,-15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == bit_int*14 + music_offset)
            {
                wbPosPelZ->setTaskPos(-z_101, bit_int*RT_TIMER_PERIOD_MS, MODE_RELATIVE);
                wbUBJoint->SetMoveJoint(RSP, -120.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
//                wbUBJoint->SetMoveJoint(RSR, -15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(REB, -70.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LSR,  25.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LSY, -25.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LEB, -50.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(NK1, 15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(NK2, 15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == bit_int*15 + music_offset)
            {
                wbPosPelZ->setTaskPos(z_101, bit_int*RT_TIMER_PERIOD_MS, MODE_RELATIVE);
                wbUBJoint->SetMoveJoint(RSP, -130.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
//                wbUBJoint->SetMoveJoint(RSR, 0.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(REB,-10.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LSR, 10.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LSY,  0.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LEB,-30.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(NK1,-15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(NK2,-15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
            }
            //Pelvis move + Left move=========================================================================================================
            if(sciTimeCnt == bit_int*16 + music_offset)
            {
                wbPosPelZ->setTaskPos(-z_101, bit_int*RT_TIMER_PERIOD_MS, MODE_RELATIVE);
                wbUBJoint->SetMoveJoint(LSR, 25.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LSY,-25.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LEB,-40.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(NK1, 15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(NK2, 15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == bit_int*17 + music_offset)
            {
                wbPosPelZ->setTaskPos(z_101, bit_int*RT_TIMER_PERIOD_MS, MODE_RELATIVE);
                wbUBJoint->SetMoveJoint(LSR, 10.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LSY,  0.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LEB,-30.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(NK1,-15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(NK2,-15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == bit_int*18 + music_offset)
            {
                wbPosPelZ->setTaskPos(-z_101, bit_int*RT_TIMER_PERIOD_MS, MODE_RELATIVE);
                wbUBJoint->SetMoveJoint(LSR, 25.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LSY,-25.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LEB,-50.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(NK1, 15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(NK2, 15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == bit_int*19 + music_offset)
            {
                wbPosPelZ->setTaskPos(z_101, bit_int*RT_TIMER_PERIOD_MS, MODE_RELATIVE);
                wbUBJoint->SetMoveJoint(LSR, 10.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LSY,  0.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LEB,-30.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(NK1,-15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(NK2,-15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == bit_int*20 + music_offset)
            {
                wbPosPelZ->setTaskPos(-z_101, bit_int*RT_TIMER_PERIOD_MS, MODE_RELATIVE);
                wbUBJoint->SetMoveJoint(LSR, 25.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LSY,-25.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LEB,-50.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(NK1, 15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(NK2, 15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == bit_int*21 + music_offset)
            {
                wbPosPelZ->setTaskPos(z_101, bit_int*RT_TIMER_PERIOD_MS, MODE_RELATIVE);
                wbUBJoint->SetMoveJoint(LSR, 10.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LSY,  0.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LEB,-30.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(NK1,-15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(NK2,-15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == bit_int*22 + music_offset)
            {
                wbPosPelZ->setTaskPos(-z_101, bit_int*RT_TIMER_PERIOD_MS, MODE_RELATIVE);
                wbUBJoint->SetMoveJoint(LSR, 25.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LSY,-25.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LEB,-50.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(NK1, 15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(NK2, 15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == bit_int*23 + music_offset)
            {
                wbPosPelZ->setTaskPos(z_101, bit_int*RT_TIMER_PERIOD_MS, MODE_RELATIVE);
                wbUBJoint->SetMoveJoint(LSR, 10.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LSY,  0.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LEB,-30.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(NK1,-15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(NK2,-15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
            }
            //Pelvis move + Left move + Shake Right=========================================================================================================
            if(sciTimeCnt == bit_int*24 + music_offset)
            {
                wbPosPelZ->setTaskPos(-z_101, bit_int*RT_TIMER_PERIOD_MS, MODE_RELATIVE);
                wbUBJoint->SetMoveJoint(LSR, 25.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LSY,-25.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LEB,-50.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RSP, -120.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
//                wbUBJoint->SetMoveJoint(RSR, -15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(REB, -70.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(NK1, 15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(NK2, 15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == bit_int*25 + music_offset)
            {
                wbPosPelZ->setTaskPos(z_101, bit_int*RT_TIMER_PERIOD_MS, MODE_RELATIVE);
                wbUBJoint->SetMoveJoint(LSR, 10.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LSY, 0.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LEB,-30.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RSP, -130.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
//                wbUBJoint->SetMoveJoint(RSR, 0.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(REB, -10.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(NK1,-15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(NK2,-15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == bit_int*26 + music_offset)
            {
                wbPosPelZ->setTaskPos(-z_101, bit_int*RT_TIMER_PERIOD_MS, MODE_RELATIVE);
                wbUBJoint->SetMoveJoint(LSR, 25.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LSY,-25.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LEB,-50.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RSP, -120.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
//                wbUBJoint->SetMoveJoint(RSR, -15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(REB, -70.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(NK1, 15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(NK2, 15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == bit_int*27 + music_offset)
            {
                wbPosPelZ->setTaskPos(z_101, bit_int*RT_TIMER_PERIOD_MS, MODE_RELATIVE);
                wbUBJoint->SetMoveJoint(LSR, 10.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LSY, 0.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LEB,-30.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RSP, -130.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
//                wbUBJoint->SetMoveJoint(RSR, 0.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(REB, -10.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(NK1,-15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(NK2,-15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == bit_int*28 + music_offset)
            {
                wbPosPelZ->setTaskPos(-z_101, bit_int*RT_TIMER_PERIOD_MS, MODE_RELATIVE);
                wbUBJoint->SetMoveJoint(LSR, 25.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LSY,-25.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LEB,-50.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RSP, -120.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
//                wbUBJoint->SetMoveJoint(RSR, -15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(REB, -70.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(NK1, 15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(NK2, 15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == bit_int*29 + music_offset)
            {
                wbPosPelZ->setTaskPos(z_101, bit_int*RT_TIMER_PERIOD_MS, MODE_RELATIVE);
                wbUBJoint->SetMoveJoint(LSR, 10.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LSY, 0.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LEB,-30.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RSP, -130.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
//                wbUBJoint->SetMoveJoint(RSR, 0.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(REB, -10.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(NK1,-15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(NK2,-15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                setMoveFinger(RF2,-1,1400);//2 both 0 right 1 left
            }
            if(sciTimeCnt == bit_int*30 + music_offset)
            {
                wbPosPelZ->setTaskPos(-z_101, bit_int*RT_TIMER_PERIOD_MS, MODE_RELATIVE);
                wbUBJoint->SetMoveJoint(LSR, 25.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LSY,-25.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LEB,-50.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RSP, -120.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
//                wbUBJoint->SetMoveJoint(RSR, -15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(REB, -70.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(NK1, 15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(NK2, 15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == bit_int*31 + music_offset)
            {
                wbPosPelZ->setTaskPos(z_101, bit_int*RT_TIMER_PERIOD_MS, MODE_RELATIVE);
                wbUBJoint->SetMoveJoint(LSR, 10.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LSY, 0.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LEB,-30.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RSP, -130.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
//                wbUBJoint->SetMoveJoint(RSR, 0.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(REB, -10.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(NK1,-15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(NK2,-15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);

            }
            //Punch Right + Left=========================================================================================================
            if(sciTimeCnt == bit_int*32 + music_offset)
            {
//                setMoveFinger(RF1,-1,800);//2 both 0 right 1 left
//                setMoveFinger(RF2,-1,1400);//2 both 0 right 1 left
//                setMoveFinger(RF3,-1,800);//2 both 0 right 1 left
//                setMoveFinger(RF4,-1,800);//2 both 0 right 1 left
//                setMoveFinger(RF5,-1,800);//2 both 0 right 1 left
                wbPosPelZ->setTaskPos(-z_101, bit_int*RT_TIMER_PERIOD_MS, MODE_RELATIVE);
                wbUBJoint->SetMoveJoint(RSP, -80.f, bit_int*RT_TIMER_PERIOD_MS*2, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RSR,  10.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(REB, -55.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RWY,  90.f, bit_int*RT_TIMER_PERIOD_MS*2, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(NKY,  -15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(NK1, 0.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(NK2, 0.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbPosWST->setTaskPos(15.f, bit_int*RT_TIMER_PERIOD_MS*2., MODE_ABSOLUTE);
            }
            if(sciTimeCnt == bit_int*33 + music_offset)
            {
                wbPosPelZ->setTaskPos(z_101, bit_int*RT_TIMER_PERIOD_MS, MODE_RELATIVE);
                wbUBJoint->SetMoveJoint(REB, -10.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
//                wbUBJoint->SetMoveJoint(RSR,  10.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == bit_int*34 + music_offset)
            {
                wbPosPelZ->setTaskPos(-z_101, bit_int*RT_TIMER_PERIOD_MS, MODE_RELATIVE);
            }
            if(sciTimeCnt == bit_int*35 + music_offset)
            {
                wbPosPelZ->setTaskPos(z_101, bit_int*RT_TIMER_PERIOD_MS, MODE_RELATIVE);
            }
            if(sciTimeCnt == bit_int*36 + music_offset)
            {
                setMoveFinger(LF1,-1,700);//2 both 0 right 1 left
                setMoveFinger(LF2,-1,700);//2 both 0 right 1 left
                setMoveFinger(LF3,-1,700);//2 both 0 right 1 left
                setMoveFinger(LF4,-1,700);//2 both 0 right 1 left
                setMoveFinger(LF5,-1,700);//2 both 0 right 1 left
                wbPosPelZ->setTaskPos(-z_101, bit_int*RT_TIMER_PERIOD_MS, MODE_RELATIVE);
                wbUBJoint->SetMoveJoint(LSP, -80.f, bit_int*RT_TIMER_PERIOD_MS*2, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LSR,  20.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LEB, -55.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LWY, -90.f, bit_int*RT_TIMER_PERIOD_MS*2, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(NKY,   15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbPosWST->setTaskPos(-15.f, bit_int*RT_TIMER_PERIOD_MS*2., MODE_ABSOLUTE);

                wbUBJoint->SetMoveJoint(RSP,   10.f, bit_int*RT_TIMER_PERIOD_MS*2, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RSR,   20.f, bit_int*RT_TIMER_PERIOD_MS*2, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(REB,  -20.f, bit_int*RT_TIMER_PERIOD_MS*2, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == bit_int*37 + music_offset)
            {
                wbPosPelZ->setTaskPos(z_101, bit_int*RT_TIMER_PERIOD_MS, MODE_RELATIVE);
                wbUBJoint->SetMoveJoint(LEB, -10.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LSR, -10.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == bit_int*38 + music_offset)
            {
                wbPosPelZ->setTaskPos(-z_101, bit_int*RT_TIMER_PERIOD_MS, MODE_RELATIVE);
            }
            if(sciTimeCnt == bit_int*39 + music_offset)
            {
                wbPosPelZ->setTaskPos(z_101, bit_int*RT_TIMER_PERIOD_MS, MODE_RELATIVE);
            }
            //Punch Right + Pelvis Down=========================================================================================================
            if(sciTimeCnt == bit_int*40 + music_offset)
            {
                z_101 = 0.15;
                wbPosPelZ->setTaskPos(-z_101, bit_int*RT_TIMER_PERIOD_MS, MODE_RELATIVE);
                wbUBJoint->SetMoveJoint(RSP, -80.f, bit_int*RT_TIMER_PERIOD_MS*2, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RSR, -20.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(REB, -55.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(NKY,  -15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbPosWST->setTaskPos(15.f, bit_int*RT_TIMER_PERIOD_MS*2., MODE_ABSOLUTE);

                wbUBJoint->SetMoveJoint(LSP,   10.f, bit_int*RT_TIMER_PERIOD_MS*2, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LSR,  -20.f, bit_int*RT_TIMER_PERIOD_MS*2, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LEB,  -20.f, bit_int*RT_TIMER_PERIOD_MS*2, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == bit_int*41 + music_offset)
            {
                wbPosPelZ->setTaskPos(z_101-0.1, bit_int*RT_TIMER_PERIOD_MS, MODE_RELATIVE);
                wbUBJoint->SetMoveJoint(REB, -10.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RSR,  10.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == bit_int*42 + music_offset)
            {
                wbPosPelZ->setTaskPos(-z_101, bit_int*RT_TIMER_PERIOD_MS, MODE_RELATIVE);
            }
            if(sciTimeCnt == bit_int*43 + music_offset)
            {
                wbPosPelZ->setTaskPos(z_101-0.1, bit_int*RT_TIMER_PERIOD_MS, MODE_RELATIVE);
            }
            if(sciTimeCnt == bit_int*44 + music_offset)
            {
                wbPosPelZ->setTaskPos(-(z_101-0.1), bit_int*RT_TIMER_PERIOD_MS, MODE_RELATIVE);
            }
            if(sciTimeCnt == bit_int*45 + music_offset)
            {
                wbPosPelZ->setTaskPos(z_101, bit_int*RT_TIMER_PERIOD_MS, MODE_RELATIVE);
            }
            if(sciTimeCnt == bit_int*46 + music_offset)
            {
                wbPosPelZ->setTaskPos(-(z_101-0.1), bit_int*RT_TIMER_PERIOD_MS, MODE_RELATIVE);
                wbPosWST->setTaskPos(0.f, bit_int*RT_TIMER_PERIOD_MS*2., MODE_ABSOLUTE);
            }
            if(sciTimeCnt == bit_int*47 + music_offset)
            {
                wbPosPelZ->setTaskPos(z_101, bit_int*RT_TIMER_PERIOD_MS, MODE_RELATIVE);
                wbUBJoint->SetMoveJoint(NKY,  0.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
            }
            //WST Rotate + Left Right Move=========================================================================================================
            if(sciTimeCnt == bit_int*48 + music_offset)
            {
                wbUBJoint->SetMoveJoint(RSP, -140.f, bit_int*RT_TIMER_PERIOD_MS*4, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RSY,   70.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(REB, -60.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);

                wbUBJoint->SetMoveJoint(LSP, -130.f, bit_int*RT_TIMER_PERIOD_MS*4, MODE_ABSOLUTE);
//                wbUBJoint->SetMoveJoint(LSY, -70.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LEB, -30.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbPosWST->setTaskPos(20.f, bit_int*RT_TIMER_PERIOD_MS*2, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == bit_int*49 + music_offset)
            {
                wbUBJoint->SetMoveJoint(RSY,   70.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(REB, -30.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);

                wbUBJoint->SetMoveJoint(LSY, -70.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LEB, -60.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == bit_int*50 + music_offset)
            {
                wbUBJoint->SetMoveJoint(RSY,   70.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(REB, -60.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);

                wbUBJoint->SetMoveJoint(LSY, -70.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LEB, -30.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbPosWST->setTaskPos(-20.f, bit_int*RT_TIMER_PERIOD_MS*4, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == bit_int*51 + music_offset)
            {
                wbUBJoint->SetMoveJoint(RSY,   70.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(REB, -30.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);

                wbUBJoint->SetMoveJoint(LSY, -70.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LEB, -60.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == bit_int*52 + music_offset)
            {
                wbUBJoint->SetMoveJoint(RSY,   70.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(REB, -60.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);

                wbUBJoint->SetMoveJoint(LSY, -70.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LEB, -30.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == bit_int*53 + music_offset)
            {
                wbUBJoint->SetMoveJoint(RSY,   70.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(REB, -30.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);

                wbUBJoint->SetMoveJoint(LSY, -70.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LEB, -60.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == bit_int*54 + music_offset)
            {
                wbUBJoint->SetMoveJoint(RSY,   70.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(REB, -60.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);

                wbUBJoint->SetMoveJoint(LSY, -70.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LEB, -30.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbPosWST->setTaskPos(0.f, bit_int*RT_TIMER_PERIOD_MS*2, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == bit_int*55 + music_offset)
            {
                wbUBJoint->SetMoveJoint(RSY,   70.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(REB, -30.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);

                wbUBJoint->SetMoveJoint(LSY, -70.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LEB, -60.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
            }
            //Pelvis move + Left move +  Right move=========================================================================================================
            if(sciTimeCnt == bit_int*56 + music_offset)
            {
                wbUBJoint->SetMoveJoint(RSP,  13.f, bit_int*20, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RSR, -10.f, bit_int*11, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RSY,   0.f, bit_int*9, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(REB, -30.f, bit_int*9, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RWY,   0.f, bit_int*9, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LSP,  13.f, bit_int*20, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LSR,  10.f, bit_int*11, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LSY,   0.f, bit_int*9, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LEB, -30.f, bit_int*9, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LWY,   0.f, bit_int*9, MODE_ABSOLUTE);
            }

            //Pelvis move + Left move +  Right move=========================================================================================================
            if(sciTimeCnt == bit_int*64 + music_offset)
            {
                z_101 = 0.05;
                wbPosPelZ->setTaskPos(-z_101, bit_int*RT_TIMER_PERIOD_MS, MODE_RELATIVE);
                wbUBJoint->SetMoveJoint(RSR,-25.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RSY, 25.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(REB,-50.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);

                wbUBJoint->SetMoveJoint(LSR, 25.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LSY,-25.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LEB,-50.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(NK1, 15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(NK2, 15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == bit_int*65 + music_offset)
            {
                wbPosPelZ->setTaskPos(z_101, bit_int*RT_TIMER_PERIOD_MS, MODE_RELATIVE);
                wbUBJoint->SetMoveJoint(RSR,-10.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RSY,  0.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(REB,-30.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);

                wbUBJoint->SetMoveJoint(LSR, 10.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LSY,  0.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LEB,-30.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(NK1,-15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(NK2,-15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == bit_int*66 + music_offset)
            {
                wbPosPelZ->setTaskPos(-z_101, bit_int*RT_TIMER_PERIOD_MS, MODE_RELATIVE);
                wbUBJoint->SetMoveJoint(RSR,-25.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RSY, 25.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(REB,-50.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);

                wbUBJoint->SetMoveJoint(LSR, 25.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LSY,-25.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LEB,-50.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(NK1, 15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(NK2, 15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == bit_int*67 + music_offset)
            {
                wbPosPelZ->setTaskPos(z_101, bit_int*RT_TIMER_PERIOD_MS, MODE_RELATIVE);
                wbUBJoint->SetMoveJoint(RSR,-10.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RSY,  0.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(REB,-30.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);

                wbUBJoint->SetMoveJoint(LSR, 10.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LSY,  0.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LEB,-30.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(NK1,-15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(NK2,-15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == bit_int*68 + music_offset)
            {
                wbPosPelZ->setTaskPos(-z_101, bit_int*RT_TIMER_PERIOD_MS, MODE_RELATIVE);
                wbUBJoint->SetMoveJoint(RSR,-25.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RSY, 25.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(REB,-50.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);

                wbUBJoint->SetMoveJoint(LSR, 25.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LSY,-25.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LEB,-50.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(NK1, 15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(NK2, 15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == bit_int*69 + music_offset)
            {
                wbPosPelZ->setTaskPos(z_101, bit_int*RT_TIMER_PERIOD_MS, MODE_RELATIVE);
                wbUBJoint->SetMoveJoint(RSR,-10.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RSY,  0.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(REB,-30.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);

                wbUBJoint->SetMoveJoint(LSR, 10.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LSY,  0.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LEB,-30.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(NK1,-15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(NK2,-15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == bit_int*70 + music_offset)
            {
                wbPosPelZ->setTaskPos(-z_101, bit_int*RT_TIMER_PERIOD_MS, MODE_RELATIVE);
                wbUBJoint->SetMoveJoint(RSR,-25.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RSY, 25.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(REB,-50.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);

                wbUBJoint->SetMoveJoint(LSR, 25.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LSY,-25.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LEB,-50.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(NK1, 15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(NK2, 15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == bit_int*71 + music_offset)
            {
                wbPosPelZ->setTaskPos(z_101, bit_int*RT_TIMER_PERIOD_MS, MODE_RELATIVE);
                wbUBJoint->SetMoveJoint(RSR,-10.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RSY,  0.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(REB,-30.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);

                wbUBJoint->SetMoveJoint(LSR, 10.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LSY,  0.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LEB,-30.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(NK1,-15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(NK2,-15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
            }
            //Pelvis move + Left move +  Right move=========================================================================================================
            if(sciTimeCnt == bit_int*72 + music_offset)
            {
                z_101 = 0.05;
                wbPosPelZ->setTaskPos(-z_101, bit_int*RT_TIMER_PERIOD_MS, MODE_RELATIVE);
                wbUBJoint->SetMoveJoint(RSR,-25.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RSY, 25.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(REB,-50.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);

                wbUBJoint->SetMoveJoint(LSR, 25.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LSY,-25.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LEB,-50.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(NK1, 15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(NK2, 15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == bit_int*73 + music_offset)
            {
                wbPosPelZ->setTaskPos(z_101, bit_int*RT_TIMER_PERIOD_MS, MODE_RELATIVE);
                wbUBJoint->SetMoveJoint(RSR,-10.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RSY,  0.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(REB,-30.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);

                wbUBJoint->SetMoveJoint(LSR, 10.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LSY,  0.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LEB,-30.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(NK1,-15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(NK2,-15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == bit_int*74 + music_offset)
            {
                wbPosPelZ->setTaskPos(-z_101, bit_int*RT_TIMER_PERIOD_MS, MODE_RELATIVE);
                wbUBJoint->SetMoveJoint(RSR,-25.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RSY, 25.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(REB,-50.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);

                wbUBJoint->SetMoveJoint(LSR, 25.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LSY,-25.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LEB,-50.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(NK1, 15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(NK2, 15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == bit_int*75 + music_offset)
            {
                wbPosPelZ->setTaskPos(z_101, bit_int*RT_TIMER_PERIOD_MS, MODE_RELATIVE);
                wbUBJoint->SetMoveJoint(RSR,-10.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RSY,  0.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(REB,-30.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);

                wbUBJoint->SetMoveJoint(LSR, 10.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LSY,  0.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LEB,-30.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(NK1,-15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(NK2,-15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == bit_int*76 + music_offset)
            {
                wbPosPelZ->setTaskPos(-z_101, bit_int*RT_TIMER_PERIOD_MS, MODE_RELATIVE);
                wbUBJoint->SetMoveJoint(RSR,-25.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RSY, 25.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(REB,-50.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);

                wbUBJoint->SetMoveJoint(LSR, 25.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LSY,-25.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LEB,-50.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(NK1, 15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(NK2, 15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == bit_int*77 + music_offset)
            {
                wbPosPelZ->setTaskPos(z_101, bit_int*RT_TIMER_PERIOD_MS, MODE_RELATIVE);
                wbUBJoint->SetMoveJoint(RSR,-10.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RSY,  0.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(REB,-30.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);

                wbUBJoint->SetMoveJoint(LSR, 10.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LSY,  0.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LEB,-30.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(NK1,-15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(NK2,-15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == bit_int*78 + music_offset)
            {
                wbPosPelZ->setTaskPos(-z_101, bit_int*RT_TIMER_PERIOD_MS, MODE_RELATIVE);
                wbUBJoint->SetMoveJoint(RSR,-25.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RSY, 25.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(REB,-50.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);

                wbUBJoint->SetMoveJoint(LSR, 25.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LSY,-25.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LEB,-50.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(NK1, 15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(NK2, 15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == bit_int*79 + music_offset)
            {
                wbPosPelZ->setTaskPos(z_101, bit_int*RT_TIMER_PERIOD_MS, MODE_RELATIVE);
                wbUBJoint->SetMoveJoint(RSR,-10.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RSY,  0.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(REB,-30.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);

                wbUBJoint->SetMoveJoint(LSR, 10.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LSY,  0.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LEB,-30.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(NK1,-15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(NK2,-15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
            }
            //Pelvis move + Left move +  Right move=========================================================================================================
            if(sciTimeCnt == bit_int*80 + music_offset)
            {
                z_101 = 0.05;
                wbPosPelZ->setTaskPos(-z_101, bit_int*RT_TIMER_PERIOD_MS, MODE_RELATIVE);
                wbUBJoint->SetMoveJoint(RSR,-25.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RSY, 25.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(REB,-50.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);

                wbUBJoint->SetMoveJoint(LSR, 25.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LSY,-25.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LEB,-50.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(NK1, 15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(NK2, 15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == bit_int*81 + music_offset)
            {
                wbPosPelZ->setTaskPos(z_101, bit_int*RT_TIMER_PERIOD_MS, MODE_RELATIVE);
                wbUBJoint->SetMoveJoint(RSR,-10.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RSY,  0.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(REB,-30.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);

                wbUBJoint->SetMoveJoint(LSR, 10.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LSY,  0.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LEB,-30.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(NK1,-15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(NK2,-15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == bit_int*82 + music_offset)
            {
                wbPosPelZ->setTaskPos(-z_101, bit_int*RT_TIMER_PERIOD_MS, MODE_RELATIVE);
                wbUBJoint->SetMoveJoint(RSR,-25.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RSY, 25.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(REB,-50.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);

                wbUBJoint->SetMoveJoint(LSR, 25.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LSY,-25.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LEB,-50.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(NK1, 15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(NK2, 15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == bit_int*83 + music_offset)
            {
                wbPosPelZ->setTaskPos(z_101, bit_int*RT_TIMER_PERIOD_MS, MODE_RELATIVE);
                wbUBJoint->SetMoveJoint(RSR,-10.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RSY,  0.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(REB,-30.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);

                wbUBJoint->SetMoveJoint(LSR, 10.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LSY,  0.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LEB,-30.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(NK1,-15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(NK2,-15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == bit_int*84 + music_offset)
            {
                wbPosPelZ->setTaskPos(-z_101, bit_int*RT_TIMER_PERIOD_MS, MODE_RELATIVE);
                wbUBJoint->SetMoveJoint(RSR,-25.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RSY, 25.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(REB,-50.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);

                wbUBJoint->SetMoveJoint(LSR, 25.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LSY,-25.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LEB,-50.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(NK1, 15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(NK2, 15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == bit_int*85 + music_offset)
            {
                wbPosPelZ->setTaskPos(z_101, bit_int*RT_TIMER_PERIOD_MS, MODE_RELATIVE);
                wbUBJoint->SetMoveJoint(RSR,-10.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RSY,  0.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(REB,-30.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);

                wbUBJoint->SetMoveJoint(LSR, 10.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LSY,  0.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LEB,-30.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(NK1,-15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(NK2,-15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == bit_int*86 + music_offset)
            {
                wbPosPelZ->setTaskPos(-z_101, bit_int*RT_TIMER_PERIOD_MS, MODE_RELATIVE);
                wbUBJoint->SetMoveJoint(RSR,-25.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RSY, 25.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(REB,-50.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);

                wbUBJoint->SetMoveJoint(LSR, 25.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LSY,-25.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LEB,-50.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(NK1, 15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(NK2, 15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == bit_int*87 + music_offset)
            {
                wbPosPelZ->setTaskPos(z_101, bit_int*RT_TIMER_PERIOD_MS, MODE_RELATIVE);
                wbUBJoint->SetMoveJoint(RSR,-10.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RSY,  0.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(REB,-30.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);

                wbUBJoint->SetMoveJoint(LSR, 10.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LSY,  0.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LEB,-30.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(NK1,-15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(NK2,-15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
            }
            //Pelvis move + Left move +  Right move=========================================================================================================
            if(sciTimeCnt == bit_int*88 + music_offset)
            {
                z_101 = 0.05;
                wbPosPelZ->setTaskPos(-z_101, bit_int*RT_TIMER_PERIOD_MS, MODE_RELATIVE);
                wbUBJoint->SetMoveJoint(RSR,-25.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RSY, 25.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(REB,-50.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);

                wbUBJoint->SetMoveJoint(LSR, 25.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LSY,-25.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LEB,-50.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(NK1, 15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(NK2, 15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == bit_int*89 + music_offset)
            {
                wbPosPelZ->setTaskPos(z_101, bit_int*RT_TIMER_PERIOD_MS, MODE_RELATIVE);
                wbUBJoint->SetMoveJoint(RSR,-10.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RSY,  0.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(REB,-30.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);

                wbUBJoint->SetMoveJoint(LSR, 10.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LSY,  0.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LEB,-30.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(NK1,-15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(NK2,-15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == bit_int*90 + music_offset)
            {
                wbPosPelZ->setTaskPos(-z_101, bit_int*RT_TIMER_PERIOD_MS, MODE_RELATIVE);
                wbUBJoint->SetMoveJoint(RSR,-25.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RSY, 25.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(REB,-50.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);

                wbUBJoint->SetMoveJoint(LSR, 25.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LSY,-25.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LEB,-50.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(NK1, 15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(NK2, 15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == bit_int*91 + music_offset)
            {
                wbPosPelZ->setTaskPos(z_101, bit_int*RT_TIMER_PERIOD_MS, MODE_RELATIVE);
                wbUBJoint->SetMoveJoint(RSR,-10.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RSY,  0.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(REB,-30.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);

                wbUBJoint->SetMoveJoint(LSR, 10.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LSY,  0.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LEB,-30.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(NK1,-15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(NK2,-15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == bit_int*92 + music_offset)
            {
                wbPosPelZ->setTaskPos(-z_101, bit_int*RT_TIMER_PERIOD_MS, MODE_RELATIVE);
                wbUBJoint->SetMoveJoint(RSR,-25.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RSY, 25.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(REB,-50.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);

                wbUBJoint->SetMoveJoint(LSR, 25.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LSY,-25.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LEB,-50.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(NK1, 15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(NK2, 15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == bit_int*93 + music_offset)
            {
                wbPosPelZ->setTaskPos(z_101, bit_int*RT_TIMER_PERIOD_MS, MODE_RELATIVE);
                wbUBJoint->SetMoveJoint(RSR,-10.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RSY,  0.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(REB,-30.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);

                wbUBJoint->SetMoveJoint(LSR, 10.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LSY,  0.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LEB,-30.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(NK1,-15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(NK2,-15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == bit_int*94 + music_offset)
            {
                wbPosPelZ->setTaskPos(-z_101, bit_int*RT_TIMER_PERIOD_MS, MODE_RELATIVE);
                wbUBJoint->SetMoveJoint(RSR,-25.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RSY, 25.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(REB,-50.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);

                wbUBJoint->SetMoveJoint(LSR, 25.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LSY,-25.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LEB,-50.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(NK1, 15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(NK2, 15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == bit_int*95 + music_offset)
            {
                wbPosPelZ->setTaskPos(z_101, bit_int*RT_TIMER_PERIOD_MS, MODE_RELATIVE);
                wbUBJoint->SetMoveJoint(RSR,-10.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RSY,  0.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(REB,-30.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);

                wbUBJoint->SetMoveJoint(LSR, 10.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LSY,  0.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LEB,-30.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(NK1,-15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(NK2,-15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
            }
            //96~103
            //104~111
            //Left up +  Right up=========================================================================================================
            if(sciTimeCnt == bit_int*112 + music_offset)
            {
            }
            if(sciTimeCnt == bit_int*113 + music_offset)
            {
            }
            if(sciTimeCnt == bit_int*114 + music_offset)
            {
            }
            if(sciTimeCnt == bit_int*115 + music_offset)
            {
            }
            if(sciTimeCnt == bit_int*116 + music_offset)
            {
                wbUBJoint->SetMoveJoint(RSP,-130.f, bit_int*RT_TIMER_PERIOD_MS*4, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RSR, -10.f, bit_int*RT_TIMER_PERIOD_MS*4, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RSY,   0.f, bit_int*RT_TIMER_PERIOD_MS*4, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(REB, -30.f, bit_int*RT_TIMER_PERIOD_MS*4, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RWY,   0.f, bit_int*RT_TIMER_PERIOD_MS*4, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LSP,-130.f, bit_int*RT_TIMER_PERIOD_MS*4, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LSR,  10.f, bit_int*RT_TIMER_PERIOD_MS*4, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LSY,   0.f, bit_int*RT_TIMER_PERIOD_MS*4, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LEB, -30.f, bit_int*RT_TIMER_PERIOD_MS*4, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LWY,   0.f, bit_int*RT_TIMER_PERIOD_MS*4, MODE_ABSOLUTE);
                setMoveFinger(RF1,1,1400);//2 both 0 right 1 left
                setMoveFinger(RF2,1,1400);//2 both 0 right 1 left
                setMoveFinger(RF3,1,1400);//2 both 0 right 1 left
                setMoveFinger(RF4,1,1400);//2 both 0 right 1 left
                setMoveFinger(RF5,1,1400);//2 both 0 right 1 left
                setMoveFinger(LF1,1,1400);//2 both 0 right 1 left
                setMoveFinger(LF2,1,1400);//2 both 0 right 1 left
                setMoveFinger(LF3,1,1400);//2 both 0 right 1 left
                setMoveFinger(LF4,1,1400);//2 both 0 right 1 left
                setMoveFinger(LF5,1,1400);//2 both 0 right 1 left
            }
            if(sciTimeCnt == bit_int*117 + music_offset)
            {

            }
            if(sciTimeCnt == bit_int*118 + music_offset)
            {

            }
            if(sciTimeCnt == bit_int*119 + music_offset)
            {
            }

            //Walk Ready=========================================================================================================
            if(sciTimeCnt == bit_int*120 + music_offset*3)
            {
                wbUBJoint->SetMoveJoint(RSP,  10.f, t1_101*2, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RSR, -10.f, t1_101*2, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RSY,   0.f, t1_101*2, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(REB, -30.f, t1_101*2, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RWY,   0.f, t1_101*2, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LSP,  10.f, t1_101*2, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LSR,  10.f, t1_101*2, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LSY,   0.f, t1_101*2, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LEB, -30.f, t1_101*2, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LWY,   0.f, t1_101*2, MODE_ABSOLUTE);

                wbUBJoint->SetMoveJoint(NKY,   0.f, t1_101, MODE_ABSOLUTE);
                wbPosWST->setTaskPos(0.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);

                setMoveFinger(RF1,-1,700);//2 both 0 right 1 left
                setMoveFinger(RF2,-1,700);//2 both 0 right 1 left
                setMoveFinger(RF3,-1,700);//2 both 0 right 1 left
                setMoveFinger(RF4,-1,700);//2 both 0 right 1 left
                setMoveFinger(RF5,-1,700);//2 both 0 right 1 left
                setMoveFinger(LF1,-1,700);//2 both 0 right 1 left
                setMoveFinger(LF2,-1,700);//2 both 0 right 1 left
                setMoveFinger(LF3,-1,700);//2 both 0 right 1 left
                setMoveFinger(LF4,-1,700);//2 both 0 right 1 left
                setMoveFinger(LF5,-1,700);//2 both 0 right 1 left

            }
            if(sciTimeCnt == bit_int*120 + music_offset*3 + t1_101_int*2+200)
                initHeadHand(1);//0 head . 1 hand . 2 hand+head
            if(sciTimeCnt == bit_int*120 + music_offset*3 + t1_101_int*2+300)
            {
                sciMotionSelector = NO_ACT;
                printf("\n ==========time : %f \n",sciTimeCnt*0.005);
                printf(">>> pick me end\n");
            }
        }
            break;
        case 102:
        {
            t1_101_int = (int)(t1_101/RT_TIMER_PERIOD_MS);
            if(sciTimeCnt == 0)
            {
                z_101 = 0.04;
                setMoveFinger(RF1,-1,700);//2 both 0 right 1 left
                setMoveFinger(RF2,1,700);//2 both 0 right 1 left
                setMoveFinger(RF3,-1,700);//2 both 0 right 1 left
                setMoveFinger(RF4,-1,700);//2 both 0 right 1 left
                setMoveFinger(RF5,-1,700);//2 both 0 right 1 left
                wbUBJoint->SetMoveJoint(RSP, -135.f, t1_101, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RSR, 20.f, t1_101, MODE_ABSOLUTE);
//                wbUBJoint->SetMoveJoint(REB, -10.f, t1_101, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == t1_101_int)
            {
                sciMotionSelector = NO_ACT;
                printf("\n ==========time : %f \n",sciTimeCnt*0.005);
                printf(">>> pick me end\n");
            }
        }
            break;
        case 103:
        {
            initHeadHand(1);
            {
                sciMotionSelector = NO_ACT;
                printf("\n ==========time : %f \n",sciTimeCnt*0.005);
                printf(">>> init\n");
            }
        }
            break;
        case 104:
        {
            t1_101_int = (int)(t1_hapum/RT_TIMER_PERIOD_MS);
            if(sciTimeCnt == 0)
            {
                setMoveFinger(0,1,700);//2 both 0 right 1 left
//                setMoveFinger(RF2,1,700);//2 both 0 right 1 left
//                setMoveFinger(RF3,1,700);//2 both 0 right 1 left
//                setMoveFinger(RF4,1,700);//2 both 0 right 1 left
//                setMoveFinger(RF5,1,700);//2 both 0 right 1 left
                wbUBJoint->SetMoveJoint(RSP, -70.f, t1_hapum, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RSR, 0.f, t1_hapum, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RSY, 60.f, t1_hapum, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(REB, -90.f, t1_hapum, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RWY, -90.f, t1_hapum, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == t1_101_int)
            {
                wbUBJoint->SetMoveJoint(REB, -110.f, 100*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == t1_101_int + 100*1)
            {
                wbUBJoint->SetMoveJoint(REB, -90.f, 100*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == t1_101_int + 100*2)
            {
                wbUBJoint->SetMoveJoint(REB, -110.f, 100*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == t1_101_int + 100*3)
            {
                wbUBJoint->SetMoveJoint(REB, -90.f, 100*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
            }
            //Walk Ready=========================================================================================================
            if(sciTimeCnt == t1_101_int + 100*4)
            {
                Setmove_walk_ready(t1_hapum);
                setMoveFinger(0,-1,700);//2 both 0 right 1 left
            }
            if(sciTimeCnt == t1_101_int + 100*4 + t1_101_int+200)
                initHeadHand(1);//0 head . 1 hand . 2 hand+head
            if(sciTimeCnt == t1_101_int + 100*4 + t1_101_int+300)
            {
                sciMotionSelector = NO_ACT;
                printf("\n ==========time : %f \n",sciTimeCnt*0.005);
                printf(">>> Hapum end\n");
            }
        }
            break;
        case 105: // One
            MotionTime = 1000.f;
            Int_MotionTime = (int)(MotionTime/RT_TIMER_PERIOD_MS);
            if(sciTimeCnt == 0)
            {
                setMoveFinger(RF1,-1,700);  //2 both 0 right 1 left
                setMoveFinger(RF2,1,700);   //2 both 0 right 1 left
                setMoveFinger(RF3,-1,700);  //2 both 0 right 1 left
                setMoveFinger(RF4,-1,700);  //2 both 0 right 1 left
                setMoveFinger(RF5,-1,700);  //2 both 0 right 1 left
                wbUBJoint->SetMoveJoint(RSP,  -70.f, MotionTime, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RSR,   -10.f, MotionTime, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RSY,   00.f, MotionTime, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(REB, -90.f, MotionTime, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RWY,   60.f, MotionTime, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == Int_MotionTime +100)
            {
                Setmove_walk_ready(MotionTime);
                setMoveFinger(RF1,1,700);  //2 both 0 right 1 left
                setMoveFinger(RF2,-1,700);   //2 both 0 right 1 left
                setMoveFinger(RF3,1,700);  //2 both 0 right 1 left
                setMoveFinger(RF4,1,700);  //2 both 0 right 1 left
                setMoveFinger(RF5,1,700);  //2 both 0 right 1 left
            }
            if(sciTimeCnt == Int_MotionTime +100 + Int_MotionTime + 200)
                initHeadHand(1);//0 head . 1 hand . 2 hand+head
            if(sciTimeCnt == Int_MotionTime +100 + Int_MotionTime + 300)
            {
                sciMotionSelector = NO_ACT;
                printf("\n ==========time : %f \n",sciTimeCnt*0.005);
                printf(">>> One end\n");
            }
            break;
        case 106: //Two
            MotionTime = 1000.f;
            Int_MotionTime = (int)(MotionTime/RT_TIMER_PERIOD_MS);
            if(sciTimeCnt == 0)
            {
                setMoveFinger(RF1,-1,700);  //2 both 0 right 1 left
                setMoveFinger(RF2,1,700);   //2 both 0 right 1 left
                setMoveFinger(RF3,1,700);  //2 both 0 right 1 left
                setMoveFinger(RF4,-1,700);  //2 both 0 right 1 left
                setMoveFinger(RF5,-1,700);  //2 both 0 right 1 left
                wbUBJoint->SetMoveJoint(RSP,  -70.f, MotionTime, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RSR,   -10.f, MotionTime, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RSY,   00.f, MotionTime, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(REB, -90.f, MotionTime, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RWY,   60.f, MotionTime, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == Int_MotionTime +100)
            {
                Setmove_walk_ready(MotionTime);
                setMoveFinger(RF1,1,700);  //2 both 0 right 1 left
                setMoveFinger(RF2,-1,700);   //2 both 0 right 1 left
                setMoveFinger(RF3,-1,700);  //2 both 0 right 1 left
                setMoveFinger(RF4,1,700);  //2 both 0 right 1 left
                setMoveFinger(RF5,1,700);  //2 both 0 right 1 left
            }
            if(sciTimeCnt == Int_MotionTime +100 + Int_MotionTime + 200)
                initHeadHand(1);//0 head . 1 hand . 2 hand+head
            if(sciTimeCnt == Int_MotionTime +100 + Int_MotionTime + 300)
            {
                sciMotionSelector = NO_ACT;
                printf("\n ==========time : %f \n",sciTimeCnt*0.005);
                printf(">>> Two end\n");
            }
            break;
        case 107: //Three
            MotionTime = 1000.f;
            Int_MotionTime = (int)(MotionTime/RT_TIMER_PERIOD_MS);
            if(sciTimeCnt == 0)
            {
                setMoveFinger(RF1,-1,700);  //2 both 0 right 1 left
                setMoveFinger(RF2,1,700);   //2 both 0 right 1 left
                setMoveFinger(RF3,1,700);  //2 both 0 right 1 left
                setMoveFinger(RF4,1,700);  //2 both 0 right 1 left
                setMoveFinger(RF5,-1,700);  //2 both 0 right 1 left
                wbUBJoint->SetMoveJoint(RSP,  -70.f, MotionTime, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RSR,   -10.f, MotionTime, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RSY,   00.f, MotionTime, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(REB, -90.f, MotionTime, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RWY,   60.f, MotionTime, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == Int_MotionTime +100)
            {
                Setmove_walk_ready(MotionTime);
                setMoveFinger(RF1,1,700);  //2 both 0 right 1 left
                setMoveFinger(RF2,-1,700);   //2 both 0 right 1 left
                setMoveFinger(RF3,-1,700);  //2 both 0 right 1 left
                setMoveFinger(RF4,-1,700);  //2 both 0 right 1 left
                setMoveFinger(RF5,1,700);  //2 both 0 right 1 left
            }
            if(sciTimeCnt == Int_MotionTime +100 + Int_MotionTime + 200)
                initHeadHand(1);//0 head . 1 hand . 2 hand+head
            if(sciTimeCnt == Int_MotionTime +100 + Int_MotionTime + 300)
            {
                sciMotionSelector = NO_ACT;
                printf("\n ==========time : %f \n",sciTimeCnt*0.005);
                printf(">>> Three end\n");
            }
            break;
        case 108: //Three
            if(fileReadFlag == ENABLE)
            {
                LeadTextData();
            }
            ChaseTextData();
            if(ringIndexBack == ringIndexFront)
            {
                sciMotionSelector = NO_ACT;
                printf("\n ==========time : %f \n",sciTimeCnt*0.005);
                printf(">>> Scene-108 motion end\n");
            }

            break;
        case 815:
            if(sciTimeCnt == 0)
            {
//                wbUBJoint->SetMoveJoint(RSP, -60.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LSP, -60.f, 800.f, MODE_ABSOLUTE);
//                wbUBJoint->SetMoveJoint(RSR, -20.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LSR,  20.f, 800.f, MODE_ABSOLUTE);
//                wbUBJoint->SetMoveJoint(REB, -80.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LEB, -80.f, 800.f, MODE_ABSOLUTE);

//                wbUBJoint->SetMoveJoint(RWY,  60.f, 800.f, MODE_ABSOLUTE);
//                wbUBJoint->SetMoveJoint(LWY, -60.f, 800.f, MODE_ABSOLUTE);

                wbPosWST->setTaskPos(-20, 800, MODE_RELATIVE);
                wbUBJoint->SetMoveJoint(NKY,-30.f, 800.f, MODE_ABSOLUTE);
                setMoveFinger(1,1,1000);//2 both 0 right 1 left RF1
            }
            else if(sciTimeCnt == 360)
            {

            }
            else if(sciTimeCnt == 580)
            {
                wbPosWST->setTaskPos(20, 900, MODE_RELATIVE);
                Setmove_walk_ready(900);
                setMoveFinger(1,-1,700);//2 both 0 right 1 left RF1
            }

            if(sciTimeCnt==800)
                initHeadHand(1);//0 head . 1 hand . 2 hand+head

            if(sciTimeCnt==1100)
            {
                sciMotionSelector = NO_ACT;
                printf("\n ==========time : %f \n",sciTimeCnt*0.005);
                printf(">>> 815 Hello_L end\n");
            }
            break;
        case 816:
            if(sciTimeCnt == 0)
            {
                wbUBJoint->SetMoveJoint(RSP, -60.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LSP, -60.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RSR, -20.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LSR,  20.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(REB, -80.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LEB, -80.f, 800.f, MODE_ABSOLUTE);

                wbUBJoint->SetMoveJoint(RWY,  80.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LWY, -80.f, 800.f, MODE_ABSOLUTE);

                wbPosWST->setTaskPos(50, 900, MODE_RELATIVE);
//                wbUBJoint->SetMoveJoint(NKY, 30.f, 800.f, MODE_ABSOLUTE);
                setMoveFinger(2,1,1000);//2 both 0 right 1 left RF1
            }
            else if(sciTimeCnt == 160)
            {
                wbUBJoint->SetMoveJoint(RSR, -30.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RSY, -20.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RWP,  20.f, 400.f, MODE_ABSOLUTE);

                wbUBJoint->SetMoveJoint(LSR,  30.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LSY,  20.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LWP,  20.f, 400.f, MODE_ABSOLUTE);
            }
            else if(sciTimeCnt == 240)
            {
                wbUBJoint->SetMoveJoint(RSR, -10.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RSY,   0.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RWP,   0.f, 400.f, MODE_ABSOLUTE);

                wbUBJoint->SetMoveJoint(LSR,  10.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LSY,   0.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LWP,   0.f, 400.f, MODE_ABSOLUTE);
            }
//            else if(sciTimeCnt == 320)
//            {
//                wbUBJoint->SetMoveJoint(RSR, -30.f, 400.f, MODE_ABSOLUTE);
//                wbUBJoint->SetMoveJoint(RSY, -20.f, 400.f, MODE_ABSOLUTE);
//                wbUBJoint->SetMoveJoint(RWP,  20.f, 400.f, MODE_ABSOLUTE);

//                wbUBJoint->SetMoveJoint(LSR,  30.f, 400.f, MODE_ABSOLUTE);
//                wbUBJoint->SetMoveJoint(LSY,  20.f, 400.f, MODE_ABSOLUTE);
//                wbUBJoint->SetMoveJoint(LWP,  20.f, 400.f, MODE_ABSOLUTE);
//            }
//            else if(sciTimeCnt == 400)
//            {
//                wbUBJoint->SetMoveJoint(RSR, -10.f, 400.f, MODE_ABSOLUTE);
//                wbUBJoint->SetMoveJoint(RSY,   0.f, 400.f, MODE_ABSOLUTE);
//                wbUBJoint->SetMoveJoint(RWP,   0.f, 400.f, MODE_ABSOLUTE);

//                wbUBJoint->SetMoveJoint(LSR,  10.f, 400.f, MODE_ABSOLUTE);
//                wbUBJoint->SetMoveJoint(LSY,   0.f, 400.f, MODE_ABSOLUTE);
//                wbUBJoint->SetMoveJoint(LWP,   0.f, 400.f, MODE_ABSOLUTE);
//            }
//            else if(sciTimeCnt == 480)
//            {
//                wbUBJoint->SetMoveJoint(RSR, -30.f, 400.f, MODE_ABSOLUTE);
//                wbUBJoint->SetMoveJoint(RSY, -20.f, 400.f, MODE_ABSOLUTE);
//                wbUBJoint->SetMoveJoint(RWP,  20.f, 400.f, MODE_ABSOLUTE);

//                wbUBJoint->SetMoveJoint(LSR,  30.f, 400.f, MODE_ABSOLUTE);
//                wbUBJoint->SetMoveJoint(LSY,  20.f, 400.f, MODE_ABSOLUTE);
//                wbUBJoint->SetMoveJoint(LWP,  20.f, 400.f, MODE_ABSOLUTE);
//            }
//            else if(sciTimeCnt == 560)
//            {
//                wbUBJoint->SetMoveJoint(RSR, -10.f, 400.f, MODE_ABSOLUTE);
//                wbUBJoint->SetMoveJoint(RSY,   0.f, 400.f, MODE_ABSOLUTE);
//                wbUBJoint->SetMoveJoint(RWP,   0.f, 400.f, MODE_ABSOLUTE);

//                wbUBJoint->SetMoveJoint(LSR,  10.f, 400.f, MODE_ABSOLUTE);
//                wbUBJoint->SetMoveJoint(LSY,   0.f, 400.f, MODE_ABSOLUTE);
//                wbUBJoint->SetMoveJoint(LWP,   0.f, 400.f, MODE_ABSOLUTE);
//            }
            else if(sciTimeCnt == 320)
            {
                wbPosWST->setTaskPos(-50, 900, MODE_RELATIVE);
                Setmove_walk_ready(900);
                setMoveFinger(2,-1,700);//2 both 0 right 1 left RF1
            }

            //if(sciTimeCnt==780)


            if(sciTimeCnt==580)
            {
                initHeadHand(1);//0 head . 1 hand . 2 hand+head
                sciMotionSelector = NO_ACT;
                printf("\n ==========time : %f \n",sciTimeCnt*0.005);
                printf(">>> 815 Hello_R end\n");
            }
            break;
        case 817:
            if(sciTimeCnt == 0)
            {
                wbUBJoint->SetMoveJoint(LSP, -50.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LSR, 0.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LEB, -110.f, 800.f, MODE_ABSOLUTE);
            }
            else if(sciTimeCnt == 360)
            {
                wbUBJoint->SetMoveJoint(LSP, -110.f, 600.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LSR, 0.f, 600.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LEB, -20.f, 600.f, MODE_ABSOLUTE);
                setMoveFinger(1,1,1000);//2 both 0 right 1 left RF1
            }
            else if(sciTimeCnt == 580)
            {
                wbUBJoint->SetMoveJoint(LSP, 10.f, 1000.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LSR, 10.f, 1000.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LEB, -30.f, 1000.f, MODE_ABSOLUTE);
                setMoveFinger(1,-1,700);//2 both 0 right 1 left RF1
            }

            if(sciTimeCnt==800)
                initHeadHand(1);//0 head . 1 hand . 2 hand+head

            if(sciTimeCnt==1100)
            {
                sciMotionSelector = NO_ACT;
                printf("\n ==========time : %f \n",sciTimeCnt*0.005);
                printf(">>> 815 Start_L end\n");
            }
            break;
        case 818:
            if(sciTimeCnt == 0)
            {
                wbUBJoint->SetMoveJoint(RSP, -50.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RSR, 0.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(REB, -110.f, 800.f, MODE_ABSOLUTE);
            }
            else if(sciTimeCnt == 360)
            {
                wbUBJoint->SetMoveJoint(RSP, -110.f, 600.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RSR, 0.f, 600.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(REB, -20.f, 600.f, MODE_ABSOLUTE);
                setMoveFinger(0,1,1000);//2 both 0 right 1 left RF1
            }
            else if(sciTimeCnt == 580)
            {
                wbUBJoint->SetMoveJoint(RSP, 10.f, 1000.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RSR, -10.f, 1000.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(REB, -30.f, 1000.f, MODE_ABSOLUTE);
                setMoveFinger(0,-1,700);//2 both 0 right 1 left RF1
            }

            if(sciTimeCnt==800)
                initHeadHand(1);//0 head . 1 hand . 2 hand+head

            if(sciTimeCnt==1100)
            {
                sciMotionSelector = NO_ACT;
                printf("\n ==========time : %f \n",sciTimeCnt*0.005);
                printf(">>> 815 Start_R end\n");
            }
            break;
        case 819:
            if(sciTimeCnt == 0)
            {
                wbUBJoint->SetMoveJoint(RSP, -60.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LSP, -60.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RSR, -20.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LSR,  20.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(REB, -80.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LEB, -80.f, 800.f, MODE_ABSOLUTE);

                wbUBJoint->SetMoveJoint(RWY,  80.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LWY, -80.f, 800.f, MODE_ABSOLUTE);

                wbPosWST->setTaskPos(20, 900, MODE_RELATIVE);
                wbUBJoint->SetMoveJoint(NKY, 30.f, 800.f, MODE_ABSOLUTE);
                setMoveFinger(2,-1,1000);//2 both 0 right 1 left RF1
            }
            else if(sciTimeCnt == 160)
            {
                wbUBJoint->SetMoveJoint(RSR, -30.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RSY,  20.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RWP,  20.f, 400.f, MODE_ABSOLUTE);

                wbUBJoint->SetMoveJoint(LSR,  30.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LSY, -20.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LWP,  20.f, 400.f, MODE_ABSOLUTE);
            }
            else if(sciTimeCnt == 240)
            {
                wbUBJoint->SetMoveJoint(RSR, -10.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RSY,   0.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RWP,   0.f, 400.f, MODE_ABSOLUTE);

                wbUBJoint->SetMoveJoint(LSR,  10.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LSY,   0.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LWP,   0.f, 400.f, MODE_ABSOLUTE);
            }
            else if(sciTimeCnt == 320)
            {
                wbUBJoint->SetMoveJoint(RSR, -30.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RSY,  20.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RWP,  20.f, 400.f, MODE_ABSOLUTE);

                wbUBJoint->SetMoveJoint(LSR,  30.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LSY, -20.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LWP,  20.f, 400.f, MODE_ABSOLUTE);
            }
            else if(sciTimeCnt == 400)
            {
                wbUBJoint->SetMoveJoint(RSR, -10.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RSY,   0.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RWP,   0.f, 400.f, MODE_ABSOLUTE);

                wbUBJoint->SetMoveJoint(LSR,  10.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LSY,   0.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LWP,   0.f, 400.f, MODE_ABSOLUTE);
            }
            else if(sciTimeCnt == 480)
            {
                wbUBJoint->SetMoveJoint(RSR, -30.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RSY,  20.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RWP,  20.f, 400.f, MODE_ABSOLUTE);

                wbUBJoint->SetMoveJoint(LSR,  30.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LSY, -20.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LWP,  20.f, 400.f, MODE_ABSOLUTE);
            }
            else if(sciTimeCnt == 560)
            {
                wbUBJoint->SetMoveJoint(RSR, -10.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RSY,   0.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(RWP,   0.f, 400.f, MODE_ABSOLUTE);

                wbUBJoint->SetMoveJoint(LSR,  10.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LSY,   0.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->SetMoveJoint(LWP,   0.f, 400.f, MODE_ABSOLUTE);
            }
            else if(sciTimeCnt == 640)
            {
                wbPosWST->setTaskPos(-20, 900, MODE_RELATIVE);
                Setmove_walk_ready(900);
                setMoveFinger(2,1,700);//2 both 0 right 1 left RF1
            }

            //if(sciTimeCnt==780)


            if(sciTimeCnt==900)
            {
                initHeadHand(1);//0 head . 1 hand . 2 hand+head
                sciMotionSelector = NO_ACT;
                printf("\n ==========time : %f \n",sciTimeCnt*0.005);
                printf(">>> 815 Hello_R end\n");
            }
            break;




        }



        break;

    default:
        break;
    }
    sciTimeCnt++;
}


void TaskMotion::Setmove_walk_ready(float _MotionTime)
{
    wbUBJoint->SetMoveJoint(RSP,  10.f, _MotionTime, MODE_ABSOLUTE);
    wbUBJoint->SetMoveJoint(RSR, -10.f, _MotionTime, MODE_ABSOLUTE);
    wbUBJoint->SetMoveJoint(RSY,   0.f, _MotionTime, MODE_ABSOLUTE);
    wbUBJoint->SetMoveJoint(REB, -30.f, _MotionTime, MODE_ABSOLUTE);
    wbUBJoint->SetMoveJoint(RWY,   0.f, _MotionTime, MODE_ABSOLUTE);
    wbUBJoint->SetMoveJoint(RWP,   0.f, _MotionTime, MODE_ABSOLUTE);
    wbUBJoint->SetMoveJoint(LSP,  10.f, _MotionTime, MODE_ABSOLUTE);
    wbUBJoint->SetMoveJoint(LSR,  10.f, _MotionTime, MODE_ABSOLUTE);
    wbUBJoint->SetMoveJoint(LSY,   0.f, _MotionTime, MODE_ABSOLUTE);
    wbUBJoint->SetMoveJoint(LEB, -30.f, _MotionTime, MODE_ABSOLUTE);
    wbUBJoint->SetMoveJoint(LWY,   0.f, _MotionTime, MODE_ABSOLUTE);
    wbUBJoint->SetMoveJoint(LWP,   0.f, _MotionTime, MODE_ABSOLUTE);

    wbUBJoint->SetMoveJoint(NKY,   0.f, _MotionTime, MODE_ABSOLUTE);
    wbPosWST->setTaskPos(0.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
}
