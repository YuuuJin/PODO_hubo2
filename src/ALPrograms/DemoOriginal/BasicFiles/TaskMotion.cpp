#include "TaskMotion.h"

#include "ManualCAN.h"

//RBMotorController   _DEV_MC[MAX_MC];
#define INT_TIME 5
extern int Control_OnOff_Flag;
unsigned int RBsetFrictionParameter(unsigned int _canch, unsigned int _bno, int _vel_saturation1, int _amp_compen1, int _vel_saturation2, int _amp_compen2);
void Upperbody_Gain_Lock();
void Upperbody_Gain_Override();
int ZeroGainLeftArm();
int ZeroGainRightArm();


TaskMotion::TaskMotion(pRBCORE_SHM _shm, JointControlClass *_joint){
    Shm = _shm;
    Joint = _joint;

    for(int i=0; i<3; i++){
        wbPosRF[i] = new TaskPos();
        wbPosLF[i] = new TaskPos();
        wbPosRH[i] = new TaskPos();
        wbPosLH[i] = new TaskPos();
    }
    wbPosWST = new TaskPos();
    for(int i=0; i<2; i++){
        wbPosCOM[i] = new TaskPos();
        wbPosELB[i] = new TaskPos();
    }
    wbPosPelZ = new TaskPos();

    wbOriRF = new TaskOri();
    wbOriLF = new TaskOri();
    wbOriRH = new TaskOri();
    wbOriLH = new TaskOri();
    wbOriPel = new TaskOri();

    wbUBJoint = new TaskJoint();

    kine_drc.upper_limit[idRWY2] = 6000*PI;
    kine_drc.lower_limit[idRWY2] = -6000*PI;
    kine_drc.upper_limit[idLWY2] = 6000*PI;
    kine_drc.lower_limit[idLWY2] = -6000*PI;

    kine_drc.upper_limit[idRKN] = 145*D2R;
    kine_drc.upper_limit[idLKN] = 145*D2R;
    kine_drc.lower_limit[idRAP] = -100*D2R;
    kine_drc.lower_limit[idLAP] = -100*D2R;

//    kine_drc.upper_limit[idRSP] = 190*D2R;
//    kine_drc.lower_limit[idRSP] = -100*D2R;
//    kine_drc.upper_limit[idLSP] = 100*D2R;
//    kine_drc.lower_limit[idLSP] = -190*D2R;

    //for Longer wrist version hubo-----------------
//    kine_drc.upper_limit[idRWP] = 115*D2R;
//    kine_drc.lower_limit[idRWP] = -115*D2R;
//    kine_drc.upper_limit[idLWP] = 115*D2R;
//    kine_drc.lower_limit[idLWP] = -115*D2R;
//    kine_drc.L_HAND=0.16;
//    kine_drc.L_HAND = 0.175;
    //----------------------------------------------


    kine_drc.L_ANKLE = 0.0;
    kine_drc.L_ELB_OFFSET = 0.022;
    kine_drc.L_FOOT = 0.095;
    kine_drc.L_HAND = 0.08;
    kine_drc.L_LOWER_ARM = 0.1222+0.04041;
    kine_drc.L_LOWER_LEG = 0.3;
    kine_drc.L_PC2WST = 0.17287;
    kine_drc.L_PEL2PEL = 0.0885*2;
    kine_drc.L_SHOULDER2SHOULDER = 0.2145*2;
    kine_drc.L_UPPER_ARM = 0.17914;
    kine_drc.L_UPPER_LEG = 0.3;
    kine_drc.L_WST2SHOULDER = 0.196;

    kine_drc.m_LeftFoot=2.6626;
    kine_drc.C_LeftFoot[0]=0.0216;
    kine_drc.C_LeftFoot[1]=0.0014;
    kine_drc.C_LeftFoot[2]=-0.0160;

    kine_drc.m_LeftHand=0.3;
    kine_drc.C_LeftHand[0]=0.0033;
    kine_drc.C_LeftHand[1]=0.0015;
    kine_drc.C_LeftHand[2]=-0.0635;

    kine_drc.m_LeftLowerArm=0.5542;
    kine_drc.C_LeftLowerArm[0]=0.0003;
    kine_drc.C_LeftLowerArm[1]=0.0006;
    kine_drc.C_LeftLowerArm[2]=-0.047;

    kine_drc.m_LeftLowerLeg=1.96;
    kine_drc.C_LeftLowerLeg[0]=0.0146;
    kine_drc.C_LeftLowerLeg[1]=0.0146;
    kine_drc.C_LeftLowerLeg[2]=-0.1845;

    kine_drc.m_LeftUpperArm=2.3147;
    kine_drc.C_LeftUpperArm[0]=0.0062;
    kine_drc.C_LeftUpperArm[1]=-0.0178;
    kine_drc.C_LeftUpperArm[2]=-0.0451;

    kine_drc.m_LeftUpperLeg=6.3512;
    kine_drc.C_LeftUpperLeg[0]=0.0175;
    kine_drc.C_LeftUpperLeg[1]=0.0099;
    kine_drc.C_LeftUpperLeg[2]=-0.0995;

    kine_drc.m_RightFoot=2.6626;
    kine_drc.C_RightFoot[0]=0.0216;
    kine_drc.C_RightFoot[1]=-0.0014;
    kine_drc.C_RightFoot[2]=-0.0160;

    kine_drc.m_RightHand=0.3;
    kine_drc.C_RightHand[0]=0.0033;
    kine_drc.C_RightHand[1]=-0.0015;
    kine_drc.C_RightHand[2]=-0.0635;

    kine_drc.m_RightLowerArm=0.5542;
    kine_drc.C_RightLowerArm[0]=0.0003;
    kine_drc.C_RightLowerArm[1]=-0.0006;
    kine_drc.C_RightLowerArm[2]=-0.047;

    kine_drc.m_RightLowerLeg=1.96;
    kine_drc.C_RightLowerLeg[0]=0.0146;
    kine_drc.C_RightLowerLeg[1]=-0.0146;
    kine_drc.C_RightLowerLeg[2]=-0.1845;

    kine_drc.m_RightUpperArm=2.3147;
    kine_drc.C_RightUpperArm[0]=0.0062;
    kine_drc.C_RightUpperArm[1]=0.0178;
    kine_drc.C_RightUpperArm[2]=-0.0451;

    kine_drc.m_RightUpperLeg=6.3512;
    kine_drc.C_RightUpperLeg[0]=0.0175;
    kine_drc.C_RightUpperLeg[1]=-0.0099;
    kine_drc.C_RightUpperLeg[2]=-0.0995;

    kine_drc.m_Pelvis=3.88;
    kine_drc.C_Pelvis[0]=-0.0119;
    kine_drc.C_Pelvis[1]=0;
    kine_drc.C_Pelvis[2]=0.1323;

    kine_drc.m_Torso=7.3 + 0.5;
    kine_drc.C_Torso[0]=-0.0115;
    kine_drc.C_Torso[1]=0.0;
    kine_drc.C_Torso[2]=0.1347+0.01;

    kine_drc.m_LeftWrist = 0.;
    kine_drc.m_RightWrist = 0.;


    kine_drc.iter_limit = 100;
    kine_drc.converge_criterium = 1e-6;
    kine_drc.orientation_weight = 0.01;
}

TaskMotion::~TaskMotion()
{
}

void TaskMotion::ResetGlobalCoord(int RF_OR_LF_OR_PC){
    for(int i=RHY; i<=LAR; i++){
        Qin_34x1[idRHY+i] = Joint->GetJointRefAngle(i)*D2R;
    }
    Qin_34x1[idWST] = Joint->GetJointRefAngle(WST)*D2R;

    Qin_34x1[idRSP] = Joint->GetJointRefAngle(RSP)*D2R;
    Qin_34x1[idRSR] = (Joint->GetJointRefAngle(RSR)+OFFSET_RSR)*D2R;
    Qin_34x1[idRSY] = Joint->GetJointRefAngle(RSY)*D2R;
    Qin_34x1[idREB] = (Joint->GetJointRefAngle(REB)+OFFSET_ELB)*D2R;
    Qin_34x1[idRWY] = Joint->GetJointRefAngle(RWY)*D2R;
    Qin_34x1[idRWP] = Joint->GetJointRefAngle(RWP)*D2R;
    Qin_34x1[idRWY2] = 0.0;//Joint->GetJointRefAngle(RWY2)*D2R;

    Qin_34x1[idLSP] = Joint->GetJointRefAngle(LSP)*D2R;
    Qin_34x1[idLSR] = (Joint->GetJointRefAngle(LSR)+OFFSET_LSR)*D2R;
    Qin_34x1[idLSY] = Joint->GetJointRefAngle(LSY)*D2R;
    Qin_34x1[idLEB] = (Joint->GetJointRefAngle(LEB)+OFFSET_ELB)*D2R;
    Qin_34x1[idLWY] = Joint->GetJointRefAngle(LWY)*D2R;
    Qin_34x1[idLWP] = Joint->GetJointRefAngle(LWP)*D2R;
    Qin_34x1[idLWY2] = 0.0;//Joint->GetJointRefAngle(LWY2)*D2R;


    kine_drc.ResetGlobal(Qin_34x1, RF_OR_LF_OR_PC, Qout_34x1);
    for(int i=0;i<=idWST;i++)
    {
        Q_filt_34x1[i] = Qout_34x1[i];
        Qd_filt_34x1[i] = 0;
        Qdd_filt_34x1[i] = 0;
    }
}



void TaskMotion::RefreshToCurrentReference(void){
    kine_drc.FK_RightFoot_Global(Q_filt_34x1, pRF_3x1, qRF_4x1);
    kine_drc.FK_LeftFoot_Global(Q_filt_34x1, pLF_3x1, qLF_4x1);
    kine_drc.FK_RightHand_Global(Q_filt_34x1, pRH_3x1, qRH_4x1, RElb_ang);
    kine_drc.FK_LeftHand_Global(Q_filt_34x1, pLH_3x1, qLH_4x1, LElb_ang);
    kine_drc.FK_COM_Global(Q_filt_34x1, pCOM_2x1);
    pPelZ = Q_filt_34x1[idZ];

    for(int i=0; i<3; i++){
        wbPosRF[i]->RefCurrent = pRF_3x1[i];
        wbPosLF[i]->RefCurrent = pLF_3x1[i];
        wbPosRH[i]->RefCurrent = pRH_3x1[i];
        wbPosLH[i]->RefCurrent = pLH_3x1[i];
    }
    wbPosPelZ->RefCurrent = Q_filt_34x1[idZ];
    wbPosCOM[0]->RefCurrent = pCOM_2x1[0];
    wbPosCOM[1]->RefCurrent = pCOM_2x1[1];
    wbPosWST->RefCurrent = Q_filt_34x1[idWST]*R2D;
    wbPosELB[0]->RefCurrent = RElb_ang*R2D;
    wbPosELB[1]->RefCurrent = LElb_ang*R2D;

    for(int i=0; i<4; i++){
        wbOriRF->RefCurrent[i] = qRF_4x1[i];
        wbOriLF->RefCurrent[i] = qLF_4x1[i];
        wbOriRH->RefCurrent[i] = qRH_4x1[i];
        wbOriLH->RefCurrent[i] = qLH_4x1[i];
        wbOriPel->RefCurrent[i] = Q_filt_34x1[idQ0+i];
    }

    for(int i=0; i<NO_OF_JOINTS; i++){
        wbUBJoint->motionJoint[i]->RefAngleCurrent = Joint->GetJointRefAngle(i);
    }
}



unsigned char TaskMotion::setMoveCOM(double _xCOM, double _yCOM, double _msTime, unsigned char _mode){
    wbPosCOM[0]->setTaskPos(_xCOM, _msTime, _mode);
    wbPosCOM[1]->setTaskPos(_yCOM, _msTime, _mode);
    return ERR_OK;
}

unsigned char TaskMotion:: setMoveLeg(char _selectLeg, double _xLEG, double _yLEG, double _zLEG, double _ori[4], double _msTime, unsigned char _mode){
    switch(_selectLeg)
    {
    case HUBO_RIGHT:
        wbPosRF[0]->setTaskPos(_xLEG, _msTime, _mode);
        wbPosRF[1]->setTaskPos(_yLEG, _msTime, _mode);
        wbPosRF[2]->setTaskPos(_zLEG, _msTime, _mode);
        if(_ori != NULL)
            wbOriRF->setTaskOri(_ori, _msTime, _mode);
        break;
    case HUBO_LEFT:
        wbPosLF[0]->setTaskPos(_xLEG, _msTime, _mode);
        wbPosLF[1]->setTaskPos(_yLEG, _msTime, _mode);
        wbPosLF[2]->setTaskPos(_zLEG, _msTime, _mode);
        if(_ori != NULL)
            wbOriLF->setTaskOri(_ori, _msTime, _mode);
        break;
    default:
        printf(">>> Wrong leg selection..!!(moveLeg)\n");
        return ERR_WRONG_SELECTION;
    }
    return ERR_OK;
}

void TaskMotion:: setMoveFinger(char _selectFinger,int _direction, float _msTime){
    _direction=_direction*-1;
    if(_selectFinger>=RF1 && _selectFinger<=LF5){
        if(_direction == 1){
            wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[_selectFinger], 10.f, _msTime, MODE_ABSOLUTE);
        }else if(_direction == -1){
            wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[_selectFinger], -10.f, _msTime, MODE_ABSOLUTE);
        }
    }else if(_selectFinger == 0){
        if(_direction == 1){
            for(int i=RF1; i<=RF5; i++) wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[i], 10.f, _msTime, MODE_ABSOLUTE);
        }else if(_direction == -1){
            for(int i=RF1; i<=RF5; i++) wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[i], -10.f, _msTime, MODE_ABSOLUTE);
        }
    }else if(_selectFinger == 1){
        if(_direction == 1){
            for(int i=LF1; i<=LF5; i++) wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[i], 10.f, _msTime, MODE_ABSOLUTE);
        }else if(_direction == -1){
            for(int i=LF1; i<=LF5; i++) wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[i], -10.f, _msTime, MODE_ABSOLUTE);
        }
    }else if(_selectFinger == 2){
        if(_direction == 1){
            for(int i=RF1; i<=LF5; i++) wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[i], 10.f, _msTime, MODE_ABSOLUTE);
        }else if(_direction == -1){
            for(int i=RF1; i<=LF5; i++) wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[i], -10.f, _msTime, MODE_ABSOLUTE);
        }
    }
}

unsigned char TaskMotion::setMoveHand(char _selectHand, double _xHAND, double _yHAND, double _zHAND, double _ori[4], double _msTime, unsigned char _mode){
    switch(_selectHand)
    {
    case HUBO_RIGHT:
        wbPosRH[0]->setTaskPos(_xHAND, _msTime, _mode);
        wbPosRH[1]->setTaskPos(_yHAND, _msTime, _mode);
        wbPosRH[2]->setTaskPos(_zHAND, _msTime, _mode);
        if(_ori != NULL)
            wbOriRH->setTaskOri(_ori, _msTime, _mode);
        break;
    case HUBO_LEFT:
        wbPosLH[0]->setTaskPos(_xHAND, _msTime, _mode);
        wbPosLH[1]->setTaskPos(_yHAND, _msTime, _mode);
        wbPosLH[2]->setTaskPos(_zHAND, _msTime, _mode);
        if(_ori != NULL)
            wbOriLH->setTaskOri(_ori, _msTime, _mode);
        break;
    default:
        //printf(">>> Wrong leg selection..!!(moveHand)\n");
        return ERR_WRONG_SELECTION;
    }
    return ERR_OK;
}




void TaskMotion::moveTaskPosOri(int _taskSpace)
{
    int i;

    switch(_taskSpace)
    {
    case TASK_RF:
        for(i=0; i<3; i++){
            wbPosRF[i]->moveTaskPos();
            des_pRF_3x1[i] = pRF_3x1[i] = wbPosRF[i]->RefCurrent;
        }
        wbOriRF->moveTaskOri();
        for(i=0; i<4; i++){
            des_qRF_4x1[i] = qRF_4x1[i] = wbOriRF->RefCurrent[i];
        }
        break;
    case TASK_LF:
        for(i=0; i<3; i++){
            wbPosLF[i]->moveTaskPos();
            des_pLF_3x1[i] = pLF_3x1[i] = wbPosLF[i]->RefCurrent;
        }
        wbOriLF->moveTaskOri();
        for(i=0; i<4; i++){
            des_qLF_4x1[i] = qLF_4x1[i] = wbOriLF->RefCurrent[i];
        }
        break;
    case TASK_RH:
        for(i=0; i<3; i++){
            wbPosRH[i]->moveTaskPos();
            des_pRH_3x1[i] = pRH_3x1[i] = wbPosRH[i]->RefCurrent;
        }
        wbOriRH->moveTaskOri();
        for(i=0; i<4; i++){
            des_qRH_4x1[i] = qRH_4x1[i] = wbOriRH->RefCurrent[i];
        }
        break;
    case TASK_LH:
        for(i=0; i<3; i++){
            wbPosLH[i]->moveTaskPos();
            des_pLH_3x1[i] = pLH_3x1[i] = wbPosLH[i]->RefCurrent;
        }
        wbOriLH->moveTaskOri();
        for(i=0; i<4; i++){
            des_qLH_4x1[i] = qLH_4x1[i] = wbOriLH->RefCurrent[i];
        }
        break;
    case TASK_PEL:
        wbPosPelZ->moveTaskPos();
        des_pPELz = pPelZ = wbPosPelZ->RefCurrent;
        wbOriPel->moveTaskOri();
        for(i=0; i<4; i++){
            des_qPEL_4x1[i] = qPEL_4x1[i] = wbOriPel->RefCurrent[i];
        }
        break;
    case TASK_COM:
        for(i=0; i<2; i++){
            wbPosCOM[i]->moveTaskPos();
            des_pCOM_2x1[i] = pCOM_2x1[i] = wbPosCOM[i]->RefCurrent;
        }
        break;
    case TASK_WST:
        wbPosWST->moveTaskPos();
        des_rWST = rWST = wbPosWST->RefCurrent*D2R;
        break;
    }
}



void TaskMotion::taskUpdate(){
    for(int i=0; i<NO_OF_MOTION; i++){
        moveTaskPosOri(i);
    }

    if(Arm_Joint_mode_flag == ENABLE){
        for(int i=RSP; i<=LWP; i++){
            wbUBJoint->moveJointAngle(wbUBJoint->motionJoint[i]);
        }

        Q_filt_34x1[idLSP] = wbUBJoint->motionJoint[LSP]->RefAngleCurrent*D2R;
        Q_filt_34x1[idLSR] = (wbUBJoint->motionJoint[LSR]->RefAngleCurrent+OFFSET_LSR)*D2R;
        Q_filt_34x1[idLSY] = wbUBJoint->motionJoint[LSY]->RefAngleCurrent*D2R;
        Q_filt_34x1[idLEB] = (wbUBJoint->motionJoint[LEB]->RefAngleCurrent+OFFSET_ELB)*D2R;
        Q_filt_34x1[idLWY] = wbUBJoint->motionJoint[LWY]->RefAngleCurrent*D2R;
        Q_filt_34x1[idLWP] = wbUBJoint->motionJoint[LWP]->RefAngleCurrent*D2R;
        Q_filt_34x1[idRSP] = wbUBJoint->motionJoint[RSP]->RefAngleCurrent*D2R;
        Q_filt_34x1[idRSR] = (wbUBJoint->motionJoint[RSR]->RefAngleCurrent+OFFSET_RSR)*D2R;
        Q_filt_34x1[idRSY] = wbUBJoint->motionJoint[RSY]->RefAngleCurrent*D2R;
        Q_filt_34x1[idREB] = (wbUBJoint->motionJoint[REB]->RefAngleCurrent+OFFSET_ELB)*D2R;
        Q_filt_34x1[idRWY] = wbUBJoint->motionJoint[RWY]->RefAngleCurrent*D2R;
        Q_filt_34x1[idRWP] = wbUBJoint->motionJoint[RWP]->RefAngleCurrent*D2R;

        Q_filt_34x1[idWST] = des_rWST;
    }
}

double TaskMotion::limit_Qd(double Qd, double Qdd, double Qd_max, double dt)   //joint velocity limit function
{
    double qd_temp = Qd + Qdd*dt;
    double qdd_temp = Qdd;

    if(qd_temp > Qd_max)
        qdd_temp = (Qd_max-Qd)/dt;
    else if(qd_temp < -Qd_max)
        qdd_temp = (-Qd_max-Qd)/dt;

    return qdd_temp;
}

void TaskMotion::WBIK(){

    int RH_ref_frame = GLOBAL;//GLOBAL,LOCAL_PELV, LOCAL_UB
    int LH_ref_frame = GLOBAL;

    if(Arm_Joint_mode_flag == ENABLE){
        kine_drc.IK_LowerBody_Global(Q_filt_34x1, des_pCOM_2x1, des_pPELz, des_qPEL_4x1, des_pRF_3x1, des_qRF_4x1, des_pLF_3x1, des_qLF_4x1, Qout_34x1);
    }else{
        kine_drc.IK_WholeBody_Global(des_pCOM_2x1, des_pPELz, des_qPEL_4x1, des_pRF_3x1, des_qRF_4x1, des_pLF_3x1, des_qLF_4x1, des_pRH_3x1, des_qRH_4x1, des_RElb_ang, RH_ref_frame,des_pLH_3x1, des_qLH_4x1, des_LElb_ang, LH_ref_frame, des_rWST, Qout_34x1);
    }

    kine_drc.set_Q0(Qout_34x1);

    const double DT = 0.005;
    const double KP = 20000;
    const double KD = 200;
    for(int k=idX; k<=idLAR;k++)
       { Q_filt_34x1[k] = Qout_34x1[k];}
    Q_filt_34x1[idWST] = Qout_34x1[idWST];
    for(int k=idRSP;k<=idLWY2;k++)
    {
        Qdd_filt_34x1[k] = limit_Qd(Qd_filt_34x1[k], -KD*Qd_filt_34x1[k] + KP*(Qout_34x1[k]-Q_filt_34x1[k]),2*PI, DT);
        Q_filt_34x1[k] += DT*Qd_filt_34x1[k] + 0.5*DT*DT*Qdd_filt_34x1[k];
        Qd_filt_34x1[k] += DT*Qdd_filt_34x1[k];
    }
    //----------------------------------------

     kine_drc.FK_RightHand_Global(Q_filt_34x1, pRH_3x1, qRH_4x1, RElb_ang);
     kine_drc.FK_LeftHand_Global(Q_filt_34x1, pLH_3x1, qLH_4x1, LElb_ang);
     kine_drc.FK_RightFoot_Global(Q_filt_34x1, pRF_3x1, qRF_4x1);
     kine_drc.FK_LeftFoot_Global(Q_filt_34x1, pLF_3x1, qLF_4x1);
     kine_drc.FK_COM_Global(Q_filt_34x1, pCOM_2x1);

     pPelZ = Q_filt_34x1[idZ];//Qout_34x1[idZ];
     for(int i=0; i<4; i++){
         qPEL_4x1[i] = Q_filt_34x1[idQ0+i];
     }

     kine_drc.set_Q0(Q_filt_34x1);
}



void TaskMotion::initHeadHand(int select){
    if(select == 0){
        printf(">>> Command: FIND_INIT_POS_HEAD\n");
        sharedData->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[0] = 0;  // right
        sharedData->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[1] = 0;  // left
        sharedData->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[2] = 1;  // head
        sharedData->COMMAND[RBCORE_PODO_NO].USER_COMMAND = DAEMON_INIT_FIND_HOME_HAND_HEAD;
        usleep(250*1000);
    }else if(select == 1){
        printf(">>> Command: FIND_INIT_POS_HAND\n");
        sharedData->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[0] = 1;  // right
        sharedData->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[1] = 1;  // left
        sharedData->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[2] = 0;  // head
        sharedData->COMMAND[RBCORE_PODO_NO].USER_COMMAND = DAEMON_INIT_FIND_HOME_HAND_HEAD;
        usleep(250*1000);
    }else if(select == 2){
        printf(">>> Command: FIND_INIT_POS_RHAND_LHAND_NECK\n");
        sharedData->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[0] = 0;  // right
        sharedData->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[1] = 0;  // left
        sharedData->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[2] = 1;  // head
        sharedData->COMMAND[RBCORE_PODO_NO].USER_COMMAND = DAEMON_INIT_FIND_HOME_HAND_HEAD;
        usleep(250*1000);
    }
}
#define HAND_GRASP_VALUE         60
#define HAND_STRETCH_VALUE      -60
#define HAND_STOP_VALUE          0
#define TRIGGER_GRASP_VALUE      14
#define TRIGGER_STRETCH_VALUE   -14

//#define MODIFY_Z_VALUE          0.04



extern int      scidemo_num;
extern float    ringData[RING_SIZE][31];
extern int      ringIndexFront;
extern int      ringIndexBack;
extern float    localJoint[NO_OF_JOINTS];
extern float    localQpel[4];
extern float    localDeltaPCz;
extern float    WR_PelcZ;
extern int      fileReadFlag;
extern FILE*    fpMocap;



int RSPint;
int RSPmode;
float Initial_COM[2];


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
//
float t1_101 = 1500.;
float t1_hapum = 1000.;
int t1_101_int = 0;
float z_101 = 0.04;
int bit_int = 47;
int music_offset = 165;

float MotionTime = 1000.;
int Int_MotionTime = (int)(MotionTime/RT_TIMER_PERIOD_MS);
int Inital_motion_Time = 0;

float dance25_offset_RSY = -2.0f;
float dance25_offset_LSY = -10.0f;
float dance25_offset_LSR = 15.0f;
float dance25_offset_RSR = -1.0f;

float dance101_offset_LSR = 10.0f;
float dance101_offset_RSR = -10.0f;


float dance101_Move_SR = 12.f;


int bit_int_pc;// = BIT_PC;//300;//90; //86
int bit_int_pc_offset = 0;//5;
int music_offset_2018;//120;//770;//1050;
int motion_loop_cnt = 0;

int motion_2_head_offest = 20;

int motion_time_check=0;
int motion_sequence_loop = 0;

void TaskMotion::LeadTextData(void){
    while(fileReadFlag == ENABLE){
        if(((ringIndexFront+1)%RING_SIZE) == ringIndexBack%RING_SIZE){
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
            , &ringData[ringIndexFront][30])){
            fileReadFlag = DISABLE;
            break;
        }else{
            ringIndexFront++;
        }
    }
}

void TaskMotion::ChaseTextData(void){
    float tempData[31];
    if(ringIndexBack%RING_SIZE != ringIndexFront%RING_SIZE){
        ringIndexBack %= RING_SIZE;
        for(int i=0; i<31; i++) tempData[i] = ringData[ringIndexBack][i];

        ringIndexBack++;

        localJoint[RSP] = tempData[6];
        localJoint[REB] = tempData[9];  localJoint[RWY] = tempData[10]; localJoint[RWP] = tempData[11];
        localJoint[LSP] = tempData[12];
        localJoint[LEB] = tempData[15]; localJoint[LWY] = tempData[16];
       localJoint[LWP] = tempData[17]; localJoint[NKY] = tempData[18]; localJoint[NK1] = tempData[19]; localJoint[NK2] = tempData[20];
        if(scidemo_num==25) {
            if(sciTimeCnt < 200) {
                localJoint[RSY] = tempData[8] + dance25_offset_RSY*0.5*(1.0-cos(PIf*sciTimeCnt*0.005f/1.000f));
                localJoint[LSY] = tempData[14] + dance25_offset_LSY*0.5*(1.0-cos(PIf*sciTimeCnt*0.005f/1.000f));
                localJoint[LSR] = tempData[13] + dance25_offset_LSR*0.5*(1.0-cos(PIf*sciTimeCnt*0.005f/1.000f));
                localJoint[RSR] = tempData[7] + dance25_offset_RSR*0.5*(1.0-cos(PIf*sciTimeCnt*0.005f/1.000f));
            }else if(sciTimeCnt > 4400 && sciTimeCnt < 4600){
                localJoint[RSY] = tempData[8]  + dance25_offset_RSY - dance25_offset_RSY*0.5*(1.0-cos(PIf*sciTimeCnt*0.005f/1.000f));
                localJoint[LSY] = tempData[14] + dance25_offset_LSY - dance25_offset_LSY*0.5*(1.0-cos(PIf*sciTimeCnt*0.005f/1.000f));
                localJoint[LSR] = tempData[13] + dance25_offset_LSR - dance25_offset_LSR*0.5*(1.0-cos(PIf*sciTimeCnt*0.005f/1.000f));
                localJoint[RSR] = tempData[7] + dance25_offset_RSR - dance25_offset_RSR*0.5*(1.0-cos(PIf*sciTimeCnt*0.005f/1.000f));
            }else if(sciTimeCnt >= 4600)
            {
                localJoint[RSY] = tempData[8];
                localJoint[LSY] = tempData[14];
                localJoint[LSR] = tempData[13];
                localJoint[RSR] = tempData[7];
            }
            else{
                localJoint[RSY] = tempData[8]  + dance25_offset_RSY;
                localJoint[LSY] = tempData[14] + dance25_offset_LSY;
                localJoint[LSR] = tempData[13] + dance25_offset_LSR;
                localJoint[RSR] = tempData[7] + dance25_offset_RSR;
            }
        }
        else {
        localJoint[RSY] = tempData[8];
        localJoint[LSY] = tempData[14];
        localJoint[LSR] = tempData[13];
        localJoint[RSR] = tempData[7];
    }
        if(scidemo_num == BOW_TWO_W){
            localJoint[WST] = tempData[5]/3.0f;
        }
        else{
            localJoint[WST] = tempData[5];
        }
//        localJoint[RF1] = tempData[21]; localJoint[RF2] = tempData[22]; localJoint[RF3] = tempData[23]; localJoint[RF4] = tempData[24];
//        localJoint[RF5] = tempData[25]; localJoint[LF1] = tempData[26]; localJoint[LF2] = tempData[27]; localJoint[LF3] = tempData[28];
//        localJoint[LF4] = tempData[29]; localJoint[LF5] = tempData[30];
        if(scidemo_num==108){
            localJoint[RF1] = -1.*tempData[21]; localJoint[RF2] = -1.*tempData[22]; localJoint[RF3] = -1.*tempData[23]; localJoint[RF4] = -1.*tempData[24];
            localJoint[RF5] = -1.*tempData[25]; localJoint[LF1] = -1.*tempData[28]; localJoint[LF2] = -1.*tempData[28]; localJoint[LF3] = -1.*tempData[28];
            localJoint[LF4] = -1.*tempData[29]; localJoint[LF5] = -1.*tempData[30];
        }else if(scidemo_num==20){
            localJoint[RF1] = -1.*tempData[21]; localJoint[RF2] = -1.*tempData[22]; localJoint[RF3] = -1.*tempData[23]; localJoint[RF4] = -1.*tempData[24];
            localJoint[RF5] = -1.*tempData[25]; localJoint[LF1] = -1.*tempData[26]; localJoint[LF2] = -1.*tempData[27]; localJoint[LF3] = -1.*tempData[28];
            localJoint[LF4] = -1.*tempData[29]; localJoint[LF5] = -1.*tempData[30];
        }else{
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
        wbUBJoint->motionJoint[LSP]->RefAngleCurrent = localJoint[LSP];
        wbUBJoint->motionJoint[LSR]->RefAngleCurrent = localJoint[LSR];
        wbUBJoint->motionJoint[LSY]->RefAngleCurrent = localJoint[LSY];
        wbUBJoint->motionJoint[LEB]->RefAngleCurrent = localJoint[LEB];
        wbUBJoint->motionJoint[LWY]->RefAngleCurrent = localJoint[LWY];
        wbUBJoint->motionJoint[LWP]->RefAngleCurrent = localJoint[LWP];


        wbUBJoint->motionJoint[RSP]->RefAngleCurrent = localJoint[RSP];
        wbUBJoint->motionJoint[RSR]->RefAngleCurrent = localJoint[RSR];
        wbUBJoint->motionJoint[RSY]->RefAngleCurrent = localJoint[RSY];
        wbUBJoint->motionJoint[REB]->RefAngleCurrent = localJoint[REB];
        wbUBJoint->motionJoint[RWY]->RefAngleCurrent = localJoint[RWY];
        wbUBJoint->motionJoint[RWP]->RefAngleCurrent = localJoint[RWP];
        */

    for(int i=RSP; i < NO_OF_JOINTS; i++){
        if(i==NKY || i==NK1 || i==NK2)  wbUBJoint->motionJoint[i]->RefAngleCurrent = localJoint[i]/100.0f;
        else if(i!=LWY)wbUBJoint->motionJoint[i]->RefAngleCurrent = localJoint[i];
    }
//   if(scidemo_num == 17 || scidemo_num == BOW_TWO_W)
//        localJoint[WST]=45.;
     //printf(" [LEB]: %f\n",localJoint[LEB]);
//    if(scidemo_num != BOW_TWO_W){
    wbPosWST->RefCurrent = localJoint[WST];
//    }
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
    wbOriPel->RefCurrent[0] = tempq0;
    wbOriPel->RefCurrent[1] = tempq1;
    wbOriPel->RefCurrent[2] = tempq2;
    wbOriPel->RefCurrent[3] = tempq3;
//    printf("Quaternion = %f,%f,%f,%f\n",tempq0,tempq1,tempq2,tempq3);


    //for(int i=1; i<=4; i++) wbOriPel->RefCurrent[i] = localQpel[i-1];
}


void TaskMotion::sciDemo(){
    static double temp_ori[4];
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
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK1], -15.f, 2000.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK2], 15.f, 2000.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NKY], 15.f, 2000.f, MODE_ABSOLUTE);
            }

            else if(sciTimeCnt==700)
            {
                wbPosWST->setTaskPos(-45.f, 2000.f, MODE_RELATIVE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK1], 0.f, 2000.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK2], 0.f, 2000.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NKY], 0.f, 2000.f, MODE_ABSOLUTE);
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
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWY], -85.f, 2000.f, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == 1500)
            {
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWY], 0.f, 2000.f, MODE_ABSOLUTE);
            }
            if(ringIndexBack == ringIndexFront)
            {
                sciMotionSelector = NO_ACT;
                //printf(">>> Scene-2 motion end\n");
                printf("\n ========== %f \n",sciTimeCnt*0.005);

                sharedData->COMMAND[PODO_NO].USER_PARA_INT[3] = 201;
                sharedData->COMMAND[PODO_NO].USER_COMMAND = SCI_DEMO;
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
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], 50.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], -30.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY], 40.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -45.f, 800.f, MODE_ABSOLUTE);

                setMoveFinger(0,-1,700);//2 both 0 right 1 left RF1

                //wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK1], 4.f, 800.f, MODE_ABSOLUTE);
                //wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK2], -4.f, 800.f, MODE_ABSOLUTE);

                wbPosWST->setTaskPos(-10.f, 800.f, MODE_ABSOLUTE);
            }
            else if(sciTimeCnt==161)//to body
            {
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], 60.f, 600.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], -15.f, 600.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -70.f, 600.f, MODE_ABSOLUTE);

            }
            else if(sciTimeCnt==281)//to outside
            {
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], 50.f, 600.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], -30.f, 600.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -40.f, 600.f, MODE_ABSOLUTE);
            }
            else if(sciTimeCnt==401)//to body08
            {
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], 60.f, 600.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], -15.f, 600.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -70.f, 600.f, MODE_ABSOLUTE);
            }
            else if(sciTimeCnt==521)//to ready posture
            {
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], 50.f, 600.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], -30.f, 600.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -45.f, 600.f, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == 650)//to walk ready
            {
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], 10.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], -10.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY], 0.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -30.f, 800.f, MODE_ABSOLUTE);

                setMoveFinger(0,1,700);//2 both 0 right 1 left RF1

                //wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK1], 0.f, 800.f, MODE_ABSOLUTE);
                //wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK2], 0.f, 800.f, MODE_ABSOLUTE);

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
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], -70.f, 1500.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -90.f, 1500.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWY], 30.f, 1500.f, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSP], -70.f, 1500.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB], -90.f, 1500.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWY], -30.f, 1500.f, MODE_ABSOLUTE);
                setMoveFinger(2,-1,700);//2 both 0 right 1 left RF1
            }
            else if(sciTimeCnt==401)
            {
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], 30.f, 1500.f, MODE_RELATIVE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSP], 30.f, 1500.f, MODE_RELATIVE);
                wbPosPelZ->setTaskPos(-0.10, 1500.f, MODE_RELATIVE);
            }
            else if(sciTimeCnt==701)
            {
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], -30.f, 1500.f, MODE_RELATIVE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSP], -30.f, 1500.f, MODE_RELATIVE);
                wbPosPelZ->setTaskPos(0.10, 1500.f, MODE_RELATIVE);
            }
            else if(sciTimeCnt==1001)
            {
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], 10.f, 1500.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -30.f, 1500.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWY], 0.f, 1500.f, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSP], 10.f, 1500.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB], -30.f, 1500.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWY], 0.f, 1500.f, MODE_ABSOLUTE);
                setMoveFinger(2,1,700);//2 both 0 right 1 left RF1
            }

            if(sciTimeCnt==401)
            {
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWY], -40.f, 700.f, MODE_RELATIVE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWY], 40.f, 700.f, MODE_RELATIVE);

                //wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK1], 5.f, 700.f, MODE_RELATIVE);
                //wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK2], -5.f, 700.f, MODE_RELATIVE);
                //wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NKY], 2.f, 700.f, MODE_ABSOLUTE);
            }
            else if(sciTimeCnt==541)
            {
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWY], 40.f, 700.f, MODE_RELATIVE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWY], -40.f, 700.f, MODE_RELATIVE);

                //wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK1], -5.f, 700.f, MODE_RELATIVE);
                //wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK2], 5.f, 700.f, MODE_RELATIVE);
                //wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NKY], -2.f, 700.f, MODE_ABSOLUTE);
            }
            else if(sciTimeCnt==681)
            {
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWY], -40.f, 700.f, MODE_RELATIVE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWY], 40.f, 700.f, MODE_RELATIVE);

                //wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK1], 5.f, 700.f, MODE_RELATIVE);
                //wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK2], -5.f, 700.f, MODE_RELATIVE);
                //wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NKY], 2.f, 700.f, MODE_ABSOLUTE);
            }
            else if(sciTimeCnt==821)
            {
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWY], 40.f, 700.f, MODE_RELATIVE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWY], -40.f, 700.f, MODE_RELATIVE);

                //wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK1], -5.f, 700.f, MODE_ABSOLUTE);
                //wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK2], 5.f, 700.f, MODE_ABSOLUTE);
                //wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NKY], 0.f, 700.f, MODE_ABSOLUTE);
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
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], 50.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], -5.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -48.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSP], -30.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], 5.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB], -84.f, 800.f, MODE_ABSOLUTE);

                setMoveFinger(2,-1,700);//2 both 0 right 1 left RF1
            }
            else if(sciTimeCnt==180)// right arm goto front
            {
                wbPosWST->setTaskPos(15.f, 1200.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], -30.f, 1200.f, MODE_ABSOLUTE);
                //wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], -5.f, 1200.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -84.f, 1200.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSP], 50.f, 1200.f, MODE_ABSOLUTE);
                //wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], 5.f, 1200.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB], -48.f, 1200.f, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NKY], -10.f, 1200.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK1], 10.f, 1200.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK2], -10.f, 1200.f, MODE_ABSOLUTE);
            }
            else if(sciTimeCnt==420)// right arm goto back
            {
                wbPosWST->setTaskPos(-15.f, 1200.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], 50.f, 1200.f, MODE_ABSOLUTE);
                //wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], -5.f, 1200.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -48.f, 1200.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSP], -30.f, 1200.f, MODE_ABSOLUTE);
                //wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], 5.f, 1200.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB], -84.f, 1200.f, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NKY], 10.f, 1200.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK1], -10.f, 1200.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK2], 10.f, 1200.f, MODE_ABSOLUTE);
            }
            else if(sciTimeCnt==660)// right arm goto front
            {
                wbPosWST->setTaskPos(15.f, 1200.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], -30.f, 1200.f, MODE_ABSOLUTE);
                //wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], -5.f, 1200.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -84.f, 1200.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSP], 50.f, 1200.f, MODE_ABSOLUTE);
                //wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], 5.f, 1200.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB], -48.f, 1200.f, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NKY], -10.f, 1200.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK1], 10.f, 1200.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK2], -10.f, 1200.f, MODE_ABSOLUTE);
            }
            else if(sciTimeCnt==900)
            {
                wbPosWST->setTaskPos(0.f, 1100.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], 10.f, 1100.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], -10.f, 1100.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -30.f, 1100.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSP], 10.f, 1100.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], 10.f, 1100.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB], -30.f, 1100.f, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NKY], 0.f, 1100.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK1], 0.f, 1100.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK2], 0.f, 1100.f, MODE_ABSOLUTE);

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
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], 5.f, 1800.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], 0.f, 1800.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], -5.f, 1800.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB], 0.f, 1800.f, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK1], -10.f, 1800.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK2], 10.f, 1800.f, MODE_ABSOLUTE);
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
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], -10.f, 1800.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -30.f, 1800.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], 10.f, 1800.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB], -30.f, 1800.f, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK1], 0.f, 1800.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK2], 0.f, 1800.f, MODE_ABSOLUTE);
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
//                sharedData->COMMAND[PODO_NO].USER_PARA_INT[3] = 1001;
//                sharedData->COMMAND[PODO_NO].USER_COMMAND = SCI_DEMO;
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
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK1], 25.f, 1000.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK2], -10.f, 1000.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NKY], -20.f, 1000.f, MODE_ABSOLUTE);
            }
            if(sciTimeCnt==600)//after 2+3sec
            {
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK1], 0.f, 1000.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK2], 0.f, 1000.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NKY], 0.f, 1000.f, MODE_ABSOLUTE);
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
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], 40.f, 1200.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -80.f, 1200.f, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NKY], 20.f, 1200.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK1], 10.f, 1200.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK2], -10.f, 1200.f, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSP], -25.f, 1200.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], -5.f, 1200.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB], -80.f, 1200.f, MODE_ABSOLUTE);

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
        case 1402://
            if(sciTimeCnt == 100)
            {
                wbPosWST->setTaskPos(25.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], -70.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], 10.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -10.f, 800.f, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NKY], -25.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK1], 0.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK2], 0.f, 800.f, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSP], 10.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], 10.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB], -30.f, 800.f, MODE_ABSOLUTE);
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
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], 10.f, 1000.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], -10.f, 1000.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -30.f, 1000.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NKY], 0.f, 1000.f, MODE_ABSOLUTE);
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
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], -40.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], 0.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY], 25.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -90.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWY], -40.f, 800.f, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSP], -40.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], 0.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY], -25.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB], -90.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWY], 40.f, 800.f, MODE_ABSOLUTE);
                setMoveFinger(2,1,700);//2 both 0 right 1 left RF1
            }
            else if(sciTimeCnt==200)//go up
            {
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], -45.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -110.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSP], -45.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB], -110.f, 400.f, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK1], -13.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK2], -13.f, 400.f, MODE_ABSOLUTE);
            }
            else if(sciTimeCnt==280)
            {
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], -40.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -90.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSP], -40.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB], -90.f, 400.f, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK1], 0.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK2], 0.f, 400.f, MODE_ABSOLUTE);
            }
            else if(sciTimeCnt==360)//go up
            {
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], -45.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -110.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSP], -45.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB], -110.f, 400.f, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK1], -13.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK2], -13.f, 400.f, MODE_ABSOLUTE);
            }
            else if(sciTimeCnt==440)
            {
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], -40.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -90.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSP], -40.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB], -90.f, 400.f, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK1], 0.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK2], 0.f, 400.f, MODE_ABSOLUTE);
            }
            else if(sciTimeCnt==520)//go up
            {
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], -45.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -110.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSP], -45.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB], -110.f, 400.f, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK1], -13.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK2], -13.f, 400.f, MODE_ABSOLUTE);
            }
            else if(sciTimeCnt==600)
            {
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], -40.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -90.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSP], -40.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB], -90.f, 400.f, MODE_ABSOLUTE);
            }
            else if(sciTimeCnt==900)
            {
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], 10.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], -10.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY], 0.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -30.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWY], 0.f, 800.f, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSP], 10.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], 10.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY], 0.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB], -30.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWY], 0.f, 800.f, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK1], 0.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK2], 0.f, 800.f, MODE_ABSOLUTE);
                setMoveFinger(2,-1,600);//2 both 0 right 1 left RF1
            }


            if(sciTimeCnt == 520)
            {
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NKY], 20.f, 600.f, MODE_ABSOLUTE);
            }
            else if(sciTimeCnt==640)
            {
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NKY], -20.f, 600.f, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == 760)
            {
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NKY], 20.f, 600.f, MODE_ABSOLUTE);
            }
            else if(sciTimeCnt==880)
            {
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NKY], 0.f, 600.f, MODE_ABSOLUTE);
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
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NKY], 0.f, 2000.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK1], 0.f, 2000.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK2], 0.f, 2000.f, MODE_ABSOLUTE);
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
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWY], -80.f, 600.f, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == 700)
            {
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWY], 0.f, 400.f, MODE_ABSOLUTE);
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
                Initial_COM[0] = pCOM_2x1[0];
                Initial_COM[1] = pCOM_2x1[1];
                printf("pCOM_2x1 = %f, %f\n",pCOM_2x1[0],pCOM_2x1[1]);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], -20.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], 20.f, 400.f, MODE_ABSOLUTE);
            }

            if(sciTimeCnt == 100)
            {
                // LIM TODO
                //setMoveCOM(Hubo2->CurTaskPos.pLF_LP_3x1[1], Hubo2->CurTaskPos.pLF_LP_3x1[2]+0.02, 1500, MODE_ABSOLUTE);
                setMoveCOM(pLF_3x1[0], pLF_3x1[1]+0.02, 1500, MODE_ABSOLUTE);
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
                setMoveCOM(Initial_COM[0], Initial_COM[1], 1800, MODE_ABSOLUTE);
                setMoveLeg(HUBO_RIGHT, 0, 0, -0.003, NULL, 1800, MODE_RELATIVE);
            }
            if(sciTimeCnt == 800)
            {
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], -10.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], 10.f, 400.f, MODE_ABSOLUTE);
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
                sharedData->COMMAND[PODO_NO].USER_PARA_INT[3] = 1901;
                sharedData->COMMAND[PODO_NO].USER_COMMAND = SCI_DEMO;
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
                sharedData->COMMAND[PODO_NO].USER_PARA_INT[3] = 1902;
                sharedData->COMMAND[PODO_NO].USER_COMMAND = SCI_DEMO;
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
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWY], -55.f, 700.f, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == 500)
            {
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWY], 0.f, 700.f, MODE_ABSOLUTE);

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
//                sharedData->COMMAND[PODO_NO].USER_PARA_INT[3] = 2001;
//                sharedData->COMMAND[PODO_NO].USER_COMMAND = SCI_DEMO;
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
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], -50.f, 1200.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], -15.f, 1200.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY], 5.f, 1200.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -115.f, 1200.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWY], 30.f, 1200.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWP], 0.f, 1200.f, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSP], -50.f, 1200.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], 15.f, 1200.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY], -5.f, 1200.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB], -115.f, 1200.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWY], -30.f, 1200.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWP], 0.f, 1200.f, MODE_ABSOLUTE);
                setMoveFinger(LF2,1,1500);//2 both 0 right 1 left RF1
                setMoveFinger(RF2,1,1500);//2 both 0 right 1 left RF1

            }
            // 2000ms = 400step
            if(sciTimeCnt == 380)
            {
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSP], -80.f, 600.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB], -125.f, 600.f, MODE_ABSOLUTE);
                wbPosPelZ->setTaskPos(-0.03, 600.f, MODE_RELATIVE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK1], -8.f, 600.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK2], 8.f, 600.f, MODE_ABSOLUTE);

            }
            // 3000ms = 600step
            if(sciTimeCnt == 500)
            {
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSP], -50.f, 600.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB], -115.f, 600.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWY], -45.f, 600.f, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], -80.f, 600.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -125.f, 600.f, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK1], 8.f, 600.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK2], -8.f, 600.f, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == 580)
            {
                setMoveFinger(LF3,1,1500);//2 both 0 right 1 left RF1
                setMoveFinger(RF3,1,1500);//2 both 0 right 1 left RF1

            }//440+300=740
            // 4000ms = 800step
            if(sciTimeCnt == 620)
            {
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], -50.f, 600.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -115.f, 600.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWY], 45.f, 600.f, MODE_ABSOLUTE);
                wbPosPelZ->setTaskPos(0.03, 600.f, MODE_RELATIVE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK1], 0.f, 600.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK2], 0.f, 600.f, MODE_ABSOLUTE);
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
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWP], -50.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], -70.f, 800.f, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWP], -50.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSP], -70.f, 800.f, MODE_ABSOLUTE);
                wbPosPelZ->setTaskPos(-0.04, 800.f, MODE_RELATIVE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK1], -10.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK2], 10.f, 800.f, MODE_ABSOLUTE);
            }
            // 6000ms = 1200step
            if(sciTimeCnt == 1020)
            {
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWP], 0.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], -50.f, 800.f, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWP], 0.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSP], -50.f, 800.f, MODE_ABSOLUTE);
                wbPosPelZ->setTaskPos(0.04, 800.f, MODE_RELATIVE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK1], 0.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK2], 0.f, 800.f, MODE_ABSOLUTE);
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
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY], -35.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWY], -65.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY], 35.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWY], 65.f, 800.f, MODE_ABSOLUTE);
            }
            // 7800ms = 1560step
            if(sciTimeCnt == 1340)
            {
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY], -5.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWY], -45.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY], 5.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWY], 45.f, 800.f, MODE_ABSOLUTE);
                wbPosPelZ->setTaskPos(-0.04, 800.f, MODE_RELATIVE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK1], 10.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK2], -10.f, 800.f, MODE_ABSOLUTE);
            }
            // 8600ms = 1720step
            if(sciTimeCnt == 1500)
            {
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY], -35.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWY], -65.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY], 35.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWY], 65.f, 800.f, MODE_ABSOLUTE);
                wbPosPelZ->setTaskPos(0.04, 800.f, MODE_RELATIVE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK1], 0.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK2], 0.f, 800.f, MODE_ABSOLUTE);
            }
            // 9400ms = 1880step
            if(sciTimeCnt == 1660)
            {
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY], -5.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWY], -45.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY], 5.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWY], 45.f, 800.f, MODE_ABSOLUTE);
            }
            //10200ms = 2040
            if(sciTimeCnt == 1860)
            {
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], 10.f, 1600.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], -10.f, 1600.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY], 0.f, 1600.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -30.f, 1600.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWY], 0.f, 1600.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWP], 0.f, 1600.f, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSP], 10.f, 1600.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], 10.f, 1600.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY], 0.f, 1600.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB], -30.f, 1600.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWY], 0.f, 1600.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWP], 0.f, 1600.f, MODE_ABSOLUTE);
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
                Initial_COM[0] = pCOM_2x1[0];
                Initial_COM[1] = pCOM_2x1[1];

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], -20.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], 20.f, 400.f, MODE_ABSOLUTE);
                setMoveFinger(2,1,700);//2 both 0 right 1 left RF1
            }
            if(sciTimeCnt == 100)
            {
                // LIM TODO
                //setMoveCOM(Hubo2->CurTaskPos.pLF_LP_3x1[1], Hubo2->CurTaskPos.pLF_LP_3x1[2], 1500, MODE_ABSOLUTE);
                setMoveCOM(pLF_3x1[0], pLF_3x1[1], 1500, MODE_ABSOLUTE);
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
                setMoveCOM(Initial_COM[0], Initial_COM[1], 1800, MODE_ABSOLUTE);
                setMoveLeg(HUBO_RIGHT, 0, 0, -0.003, NULL, 1800, MODE_RELATIVE);
            }
            //up arm
            if(sciTimeCnt == 2400)//490
            {
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], 0.f, 1500.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], -110.f, 1500.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY], -90.f, 1500.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -85.f, 1500.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWY], -90.f, 1500.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWP], 0.f, 1500.f, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSP], 0.f, 1500.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], 110.f, 1500.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY], 90.f, 1500.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB], -85.f, 1500.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWY], 90.f, 1500.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWP], 0.f, 1500.f, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK1], -10.f, 1500.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK2], 10.f, 1500.f, MODE_ABSOLUTE);

                setMoveFinger(2,-1,600);//2 both 0 right 1 left RF1
            }
            if(sciTimeCnt == 2750)
                wbPosWST->setTaskPos(-35.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NKY], 35.f, 800.f, MODE_ABSOLUTE);
            if(sciTimeCnt==2910)
                wbPosWST->setTaskPos(35.f, 1200.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NKY], -35.f, 1200.f, MODE_ABSOLUTE);
            if(sciTimeCnt==3150)
                wbPosWST->setTaskPos(0.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NKY], 0.f, 800.f, MODE_ABSOLUTE);

            if(sciTimeCnt == 3350)
            {
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], 10.f, 1600.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], -10.f, 1600.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY], 0.f, 1600.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -30.f, 1600.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWY], 0.f, 1600.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWP], 0.f, 1600.f, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSP], 10.f, 1600.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], 10.f, 1600.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY], 0.f, 1600.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB], -30.f, 1600.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWY], 0.f, 1600.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWP], 0.f, 1600.f, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK1], 0.f, 1600.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK2], 0.f, 1600.f, MODE_ABSOLUTE);
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
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], 25.f, 700.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], -45.f, 700.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY], 40.f, 700.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -80.f, 700.f, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSP], 25.f, 700.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], 45.f, 700.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY], -40.f, 700.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB], -80.f, 700.f, MODE_ABSOLUTE);
                setMoveFinger(2,-1,700);//2 both 0 right 1 left RF1
            }
            if(sciTimeCnt == 440)
            {
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], 10.f, 700.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], -10.f, 700.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY], 0.f, 700.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -30.f, 700.f, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSP], 10.f, 700.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], 10.f, 700.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY], 0.f, 700.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB], -30.f, 700.f, MODE_ABSOLUTE);
                setMoveFinger(2,1,700);//2 both 0 right 1 left RF1
            }

            if(sciTimeCnt == 140)
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NKY], 20.f, 500.f, MODE_ABSOLUTE);
            if(sciTimeCnt == 240)
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NKY], -20.f, 500.f, MODE_ABSOLUTE);
            if(sciTimeCnt == 340)
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NKY], 20.f, 500.f, MODE_ABSOLUTE);
            if(sciTimeCnt == 440)
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NKY], 0.f, 500.f, MODE_ABSOLUTE);


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
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], -40.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], 0.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY], 25.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -90.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWY], -40.f, 800.f, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSP], -40.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], 0.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY], -25.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB], -90.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWY], 40.f, 800.f, MODE_ABSOLUTE);
                setMoveFinger(2,1,700);//2 both 0 right 1 left RF1
            }
            else if(sciTimeCnt==200)//go up
            {
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], -45.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -110.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSP], -45.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB], -110.f, 400.f, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK1], -13.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK2], -13.f, 400.f, MODE_ABSOLUTE);
            }
            else if(sciTimeCnt==280)
            {
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], -40.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -90.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSP], -40.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB], -90.f, 400.f, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK1], 0.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK2], 0.f, 400.f, MODE_ABSOLUTE);
            }
            else if(sciTimeCnt==360)//go up
            {
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], -45.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -110.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSP], -45.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB], -110.f, 400.f, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK1], -13.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK2], -13.f, 400.f, MODE_ABSOLUTE);
            }
            else if(sciTimeCnt==440)
            {
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], -40.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -90.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSP], -40.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB], -90.f, 400.f, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK1], 0.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK2], 0.f, 400.f, MODE_ABSOLUTE);
            }
            else if(sciTimeCnt==520)//go up
            {
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], -45.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -110.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSP], -45.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB], -110.f, 400.f, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK1], -13.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK2], -13.f, 400.f, MODE_ABSOLUTE);
            }
            else if(sciTimeCnt==600)
            {
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], -40.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -90.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSP], -40.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB], -90.f, 400.f, MODE_ABSOLUTE);
            }
            else if(sciTimeCnt==900)
            {
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], 10.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], -10.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY], 0.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -30.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWY], 0.f, 800.f, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSP], 10.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], 10.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY], 0.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB], -30.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWY], 0.f, 800.f, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK1], 0.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK2], 0.f, 800.f, MODE_ABSOLUTE);
                setMoveFinger(2,-1,600);//2 both 0 right 1 left RF1
            }


            if(sciTimeCnt == 520)
            {
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NKY], 20.f, 600.f, MODE_ABSOLUTE);
            }
            else if(sciTimeCnt==640)
            {
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NKY], -20.f, 600.f, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == 760)
            {
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NKY], 20.f, 600.f, MODE_ABSOLUTE);
            }
            else if(sciTimeCnt==880)
            {
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NKY], 0.f, 600.f, MODE_ABSOLUTE);
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
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], 20.f, 350.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], -30.f, 350.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY], 30.f, 350.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -60.f, 350.f, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSP], 20.f, 350.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], 30.f, 350.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY], -30.f, 350.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB], -60.f, 350.f, MODE_ABSOLUTE);
            }
            else if(sciTimeCnt == 210)
            {
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], 20.f, 350.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], -15.f, 350.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY], 20.f, 350.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -60.f, 350.f, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSP], 20.f, 350.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], 15.f, 350.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY], -20.f, 350.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB], -60.f, 350.f, MODE_ABSOLUTE);
            }
            else if(sciTimeCnt == 280)
            {
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], 20.f, 350.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], -30.f, 350.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY], 30.f, 350.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -60.f, 350.f, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSP], 20.f, 350.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], 30.f, 350.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY], -30.f, 350.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB], -60.f, 350.f, MODE_ABSOLUTE);
            }
            else if(sciTimeCnt == 350)
            {
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], 10.f, 350.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], -10.f, 350.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY], 0.f, 350.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -30.f, 350.f, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSP], 10.f, 350.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], 10.f, 350.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY], 0.f, 350.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB], -30.f, 350.f, MODE_ABSOLUTE);
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
                double **mat_temp1 = matrix(4,4);
                double vec_temp[4],vec_temp1[4],vec_temp2[4];
                RY(20*D2R,mat_temp1);

                DC2QT((const double**)mat_temp1,vec_temp);
                QT2RV(vec_temp,vec_temp1);
                vec_temp2[0] = vec_temp1[0];
                vec_temp2[1] = vec_temp1[1];
                vec_temp2[2] = vec_temp1[2];
                vec_temp2[3] = vec_temp1[3];

                wbOriPel->setTaskOri(vec_temp2, 1200, MODE_ABSOLUTE);
                wbPosWST->setTaskPos(-20, 900, MODE_RELATIVE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSP], -75.f, 900.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR],  10.f, 900.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY], -45.f, 900.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB], -90.f, 900.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NKY], -60.f, 900.f, MODE_ABSOLUTE);
                setMoveFinger(1,1,1000);//2 both 0 right 1 left RF1
                free_matrix(mat_temp1,4,4);
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
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], -55.f, 900.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], 0.f, 900.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY], -30.f, 900.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -15.f, 900.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWY], -70.f, 900.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWP], 0.f, 900.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK1], 10.f, 900.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK2], -10.f, 900.f, MODE_ABSOLUTE);
                setMoveFinger(0,1,1000);//2 both 0 right 1 left RF1
            }
            else if(sciTimeCnt == 380)
            {
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], 10.f, 900.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], -10.f, 900.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY], 0.f, 900.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -30.f, 900.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWY], 0.f, 900.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWP], 0.f, 900.f, MODE_ABSOLUTE);
                setMoveFinger(0,-1,1000);//2 both 0 right 1 left RF1
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK1], 0.f, 900.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK2], 0.f, 900.f, MODE_ABSOLUTE);
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
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], -30.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], -15.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY], 40.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -100.f, 800.f, MODE_ABSOLUTE);//-120
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWY], -40.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWP], 0.f, 800.f, MODE_ABSOLUTE);

                setMoveFinger(0,1,1000);//2 both 0 right 1 left RF1
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK1], -12.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK2], 12.f, 800.f, MODE_ABSOLUTE);
            }
            else if(sciTimeCnt == 360)
            {
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], -30.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], 0.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY], 0.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -50.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWY], -55.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWP], 0.f, 800.f, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSP], 0.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], 20.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY], -50.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB], -70.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWY], 0.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWP], 0.f, 800.f, MODE_ABSOLUTE);
                setMoveFinger(1,-1,1000);//2 both 0 right 1 left RF1
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK1], 12.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK2], -12.f, 800.f, MODE_ABSOLUTE);
            }
            else if(sciTimeCnt == 620)
            {
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], -35.f, 600.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], 10.f, 600.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY], 60.f, 600.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -75.f, 600.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWY], 35.f, 600.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWP], 0.f, 600.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK1], 0.f, 600.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK2], 0.f, 600.f, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSP], -30.f, 600.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], -10.f, 600.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY], -60.f, 600.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB], -50.f, 600.f, MODE_ABSOLUTE);//
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWY], 10.f, 600.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWP], 5.f, 600.f, MODE_ABSOLUTE);
            }
            // 33 ready finished
            else if(sciTimeCnt == 740)//go out
            {
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], -50.f, 550.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], 0.f, 550.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY], 65.f, 550.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -55.f, 550.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWY], 45.f, 550.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWP], 0.f, 550.f, MODE_ABSOLUTE);
            }
            else if(sciTimeCnt == 850)//go ready
            {
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], -35.f, 550.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], 10.f, 550.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY], 60.f, 550.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -75.f, 550.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWY], 35.f, 550.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWP], 0.f, 550.f, MODE_ABSOLUTE);
            }
            else if(sciTimeCnt == 960)//go out
            {
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], -50.f, 550.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], 0.f, 550.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY], 65.f, 550.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -55.f, 550.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWY], 45.f, 550.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWP], 0.f, 550.f, MODE_ABSOLUTE);
            }
            else if(sciTimeCnt == 1070)//go ready
            {
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], -35.f, 550.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], 10.f, 550.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY], 60.f, 550.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -75.f, 550.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWY], 35.f, 550.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWP], 0.f, 550.f, MODE_ABSOLUTE);
            }
            else if(sciTimeCnt == 1180)//go out
            {
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], -50.f, 550.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], 0.f, 550.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY], 65.f, 550.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -55.f, 550.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWY], 45.f, 550.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWP], 0.f, 550.f, MODE_ABSOLUTE);
            }
            else if(sciTimeCnt == 1290)//go ready
            {
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], -35.f, 550.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], 10.f, 550.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY], 60.f, 550.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -75.f, 550.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWY], 35.f, 550.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWP], 0.f, 550.f, MODE_ABSOLUTE);
            }

            // goto walk ready pos
            else if(sciTimeCnt ==1400)
            {
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], 10.f, 1200.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], -10.f, 1200.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY], 0.f, 1200.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -30.f, 1200.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWY], 0.f, 1200.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWP], 0.f, 1200.f, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSP], 10.f, 1200.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], 10.f, 1200.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY], 0.f, 1200.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB], -30.f, 1200.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWY], 0.f, 1200.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWP], 0.f, 1200.f, MODE_ABSOLUTE);
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
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSP], -50.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], 0.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB], -110.f, 800.f, MODE_ABSOLUTE);
            }
            else if(sciTimeCnt == 360)
            {
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSP], -110.f, 600.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], 0.f, 600.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB], -20.f, 600.f, MODE_ABSOLUTE);
                setMoveFinger(1,1,1000);//2 both 0 right 1 left RF1
            }
            else if(sciTimeCnt == 580)
            {
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSP], 10.f, 1000.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], 10.f, 1000.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB], -30.f, 1000.f, MODE_ABSOLUTE);
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
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSP], 10.f, 700.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], 45.f, 700.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY], -70.f, 700.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB], -70.f, 700.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWY], -20.f, 700.f, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], 10.f, 700.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], -45.f, 700.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY], 70.f, 700.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -70.f, 700.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWY], 20.f, 700.f, MODE_ABSOLUTE);
                setMoveFinger(2,-1,1000);//2 both 0 right 1 left RF1
            }
            else if(sciTimeCnt == 300)
            {
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSP], 10.f, 700.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], 10.f, 700.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY], 0.f, 700.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB], -30.f, 700.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWY], 0.f, 700.f, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], 10.f, 700.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], -10.f, 700.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY], 0.f, 700.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -30.f, 700.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWY], 0.f, 700.f, MODE_ABSOLUTE);
                setMoveFinger(2,1,1000);//2 both 0 right 1 left RF1
            }

            if(sciTimeCnt == 150)
            {
                wbPosWST->setTaskPos(30.f, 700.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NKY], 15.f, 700.f, MODE_ABSOLUTE);
            }
            else if(sciTimeCnt == 300)
            {
                wbPosWST->setTaskPos(0.f, 700.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NKY], 0.f, 700.f, MODE_ABSOLUTE);
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
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], 20.f, 350.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], -30.f, 350.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY], 30.f, 350.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -60.f, 350.f, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSP], 20.f, 350.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], 30.f, 350.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY], -30.f, 350.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB], -60.f, 350.f, MODE_ABSOLUTE);

            }
            else if(sciTimeCnt == 210)
            {
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], 20.f, 350.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], -15.f, 350.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY], 20.f, 350.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -60.f, 350.f, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSP], 20.f, 350.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], 15.f, 350.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY], -20.f, 350.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB], -60.f, 350.f, MODE_ABSOLUTE);
            }
            else if(sciTimeCnt == 280)
            {
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], 20.f, 350.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], -30.f, 350.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY], 30.f, 350.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -60.f, 350.f, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSP], 20.f, 350.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], 30.f, 350.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY], -30.f, 350.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB], -60.f, 350.f, MODE_ABSOLUTE);
            }
            else if(sciTimeCnt == 350)
            {
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], 10.f, 350.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], -10.f, 350.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY], 0.f, 350.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -30.f, 350.f, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSP], 10.f, 350.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], 10.f, 350.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY], 0.f, 350.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB], -30.f, 350.f, MODE_ABSOLUTE);
            }

            // draw love

            if(sciTimeCnt == 490)
            {
                setMoveFinger(2,1,1000);//2 both 0 right 1 left RF1
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], 0.f, 1500.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], -110.f, 1500.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY], -90.f, 1500.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -85.f, 1500.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWY], -90.f, 1500.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWP], 0.f, 1500.f, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSP], 0.f, 1500.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], 110.f, 1500.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY], 90.f, 1500.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB], -85.f, 1500.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWY], 90.f, 1500.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWP], 0.f, 1500.f, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK1], -10.f, 1500.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK2], 10.f, 1500.f, MODE_ABSOLUTE);
            }
            else if(sciTimeCnt == 890)
            {
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], 10.f, 1600.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], -10.f, 1600.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY], 0.f, 1600.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -30.f, 1600.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWY], 0.f, 1600.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWP], 0.f, 1600.f, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSP], 10.f, 1600.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], 10.f, 1600.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY], 0.f, 1600.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB], -30.f, 1600.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWY], 0.f, 1600.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWP], 0.f, 1600.f, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK1], 0.f, 1600.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK2], 0.f, 1600.f, MODE_ABSOLUTE);
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
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK1], -15.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK2], -15.f, 400.f, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == 80)
            {
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK1], 15.f, 500.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK2], 15.f, 500.f, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == 180)
            {
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK1], -15.f, 500.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK2], -15.f, 500.f, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == 280)
            {
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK1], 15.f, 500.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK2], 15.f, 500.f, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == 380)
            {
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK1], 0.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK2], 0.f, 400.f, MODE_ABSOLUTE);
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
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NKY], -25.f, 400.f, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == 80)
            {
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NKY], 25.f, 500.f, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == 180)
            {
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NKY], -25.f, 500.f, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == 280)
            {
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NKY], 25.f, 500.f, MODE_ABSOLUTE);
            }
            if(sciTimeCnt==380)
            {
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NKY], 0.f, 400.f, MODE_ABSOLUTE);
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
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK1], 15.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK2], 15.f, 400.f, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == 80)
            {
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK1], -15.f, 500.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK2], -15.f, 500.f, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == 180)
            {
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK1], 15.f, 500.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK2], 15.f, 500.f, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == 280)
            {
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK1], -15.f, 500.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK2], -15.f, 500.f, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == 380)
            {
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK1], 0.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK2], 0.f, 400.f, MODE_ABSOLUTE);
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
//                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], -135.f, t1_101, MODE_ABSOLUTE);
//                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], 20.f, t1_101, MODE_ABSOLUTE);
////                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -10.f, t1_101, MODE_ABSOLUTE);
//            }
            //Pelvis move + Left move =========================================================================================================
            if(sciTimeCnt == 0 + music_offset)
            {
                z_101 = 0.05;
                wbPosPelZ->setTaskPos(-z_101, bit_int*RT_TIMER_PERIOD_MS, MODE_RELATIVE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], 25.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY],-25.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB],-50.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK1], 15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK2], 15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == bit_int + music_offset)
            {
                wbPosPelZ->setTaskPos(z_101, bit_int*RT_TIMER_PERIOD_MS, MODE_RELATIVE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], 10.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY],  0.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB],-30.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK1],-15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK2],-15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == bit_int*2 + music_offset)
            {
                wbPosPelZ->setTaskPos(-z_101, bit_int*RT_TIMER_PERIOD_MS, MODE_RELATIVE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], 25.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY],-25.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB],-50.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK1], 15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK2], 15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == bit_int*3 + music_offset)
            {
                wbPosPelZ->setTaskPos(z_101, bit_int*RT_TIMER_PERIOD_MS, MODE_RELATIVE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], 10.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY],  0.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB],-30.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK1],-15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK2],-15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == bit_int*4 + music_offset)
            {
                wbPosPelZ->setTaskPos(-z_101, bit_int*RT_TIMER_PERIOD_MS, MODE_RELATIVE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], 25.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY],-25.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB],-50.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK1], 15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK2], 15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == bit_int*5 + music_offset)
            {
                wbPosPelZ->setTaskPos(z_101, bit_int*RT_TIMER_PERIOD_MS, MODE_RELATIVE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], 10.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY],  0.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB],-30.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK1],-15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK2],-15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == bit_int*6 + music_offset)
            {
                wbPosPelZ->setTaskPos(-z_101, bit_int*RT_TIMER_PERIOD_MS, MODE_RELATIVE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], 25.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY],-25.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB],-50.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK1], 15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK2], 15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == bit_int*7 + music_offset)
            {
                wbPosPelZ->setTaskPos(z_101, bit_int*RT_TIMER_PERIOD_MS, MODE_RELATIVE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], 10.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY],  0.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB],-30.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK1],-15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK2],-15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
            }
            //Pelvis move + Left move + Shake Right=========================================================================================================
            if(sciTimeCnt == bit_int*8 + music_offset)
            {
                wbPosPelZ->setTaskPos(-z_101, bit_int*RT_TIMER_PERIOD_MS, MODE_RELATIVE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], -120.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
//                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], -15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB],-70.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], 25.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY],-25.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB],-50.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK1], 15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK2], 15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == bit_int*9 + music_offset)
            {
                wbPosPelZ->setTaskPos(z_101, bit_int*RT_TIMER_PERIOD_MS, MODE_RELATIVE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], -130.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
//                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], 0.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB],-10.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], 10.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY],  0.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB],-30.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK1],-15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK2],-15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == bit_int*10 + music_offset)
            {
                wbPosPelZ->setTaskPos(-z_101, bit_int*RT_TIMER_PERIOD_MS, MODE_RELATIVE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], -120.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
//                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], -15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -70.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], 25.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY],-25.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB],-50.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK1], 15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK2], 15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == bit_int*11 + music_offset)
            {
                wbPosPelZ->setTaskPos(z_101, bit_int*RT_TIMER_PERIOD_MS, MODE_RELATIVE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], -130.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
//                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], 0.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -10.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], 10.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY],  0.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB],-30.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK1],-15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK2],-15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == bit_int*12 + music_offset)
            {
                wbPosPelZ->setTaskPos(-z_101, bit_int*RT_TIMER_PERIOD_MS, MODE_RELATIVE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], -120.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
//                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], -15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB],-70.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], 25.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY],-25.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB],-50.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK1], 15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK2], 15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == bit_int*13 + music_offset)
            {
                wbPosPelZ->setTaskPos(z_101, bit_int*RT_TIMER_PERIOD_MS, MODE_RELATIVE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], -130.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
//                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], 0.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -10.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], 10.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY],  0.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB],-30.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK1],-15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK2],-15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == bit_int*14 + music_offset)
            {
                wbPosPelZ->setTaskPos(-z_101, bit_int*RT_TIMER_PERIOD_MS, MODE_RELATIVE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], -120.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
//                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], -15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -70.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR],  25.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY], -25.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB], -50.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK1], 15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK2], 15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == bit_int*15 + music_offset)
            {
                wbPosPelZ->setTaskPos(z_101, bit_int*RT_TIMER_PERIOD_MS, MODE_RELATIVE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], -130.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
//                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], 0.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB],-10.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], 10.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY],  0.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB],-30.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK1],-15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK2],-15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
            }
            //Pelvis move + Left move=========================================================================================================
            if(sciTimeCnt == bit_int*16 + music_offset)
            {
                wbPosPelZ->setTaskPos(-z_101, bit_int*RT_TIMER_PERIOD_MS, MODE_RELATIVE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], 25.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY],-25.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB],-40.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK1], 15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK2], 15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == bit_int*17 + music_offset)
            {
                wbPosPelZ->setTaskPos(z_101, bit_int*RT_TIMER_PERIOD_MS, MODE_RELATIVE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], 10.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY],  0.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB],-30.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK1],-15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK2],-15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == bit_int*18 + music_offset)
            {
                wbPosPelZ->setTaskPos(-z_101, bit_int*RT_TIMER_PERIOD_MS, MODE_RELATIVE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], 25.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY],-25.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB],-50.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK1], 15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK2], 15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == bit_int*19 + music_offset)
            {
                wbPosPelZ->setTaskPos(z_101, bit_int*RT_TIMER_PERIOD_MS, MODE_RELATIVE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], 10.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY],  0.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB],-30.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK1],-15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK2],-15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == bit_int*20 + music_offset)
            {
                wbPosPelZ->setTaskPos(-z_101, bit_int*RT_TIMER_PERIOD_MS, MODE_RELATIVE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], 25.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY],-25.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB],-50.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK1], 15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK2], 15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == bit_int*21 + music_offset)
            {
                wbPosPelZ->setTaskPos(z_101, bit_int*RT_TIMER_PERIOD_MS, MODE_RELATIVE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], 10.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY],  0.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB],-30.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK1],-15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK2],-15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == bit_int*22 + music_offset)
            {
                wbPosPelZ->setTaskPos(-z_101, bit_int*RT_TIMER_PERIOD_MS, MODE_RELATIVE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], 25.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY],-25.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB],-50.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK1], 15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK2], 15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == bit_int*23 + music_offset)
            {
                wbPosPelZ->setTaskPos(z_101, bit_int*RT_TIMER_PERIOD_MS, MODE_RELATIVE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], 10.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY],  0.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB],-30.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK1],-15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK2],-15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
            }
            //Pelvis move + Left move + Shake Right=========================================================================================================
            if(sciTimeCnt == bit_int*24 + music_offset)
            {
                wbPosPelZ->setTaskPos(-z_101, bit_int*RT_TIMER_PERIOD_MS, MODE_RELATIVE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], 25.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY],-25.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB],-50.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], -120.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
//                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], -15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -70.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK1], 15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK2], 15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == bit_int*25 + music_offset)
            {
                wbPosPelZ->setTaskPos(z_101, bit_int*RT_TIMER_PERIOD_MS, MODE_RELATIVE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], 10.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY], 0.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB],-30.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], -130.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
//                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], 0.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -10.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK1],-15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK2],-15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == bit_int*26 + music_offset)
            {
                wbPosPelZ->setTaskPos(-z_101, bit_int*RT_TIMER_PERIOD_MS, MODE_RELATIVE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], 25.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY],-25.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB],-50.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], -120.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
//                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], -15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -70.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK1], 15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK2], 15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == bit_int*27 + music_offset)
            {
                wbPosPelZ->setTaskPos(z_101, bit_int*RT_TIMER_PERIOD_MS, MODE_RELATIVE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], 10.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY], 0.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB],-30.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], -130.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
//                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], 0.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -10.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK1],-15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK2],-15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == bit_int*28 + music_offset)
            {
                wbPosPelZ->setTaskPos(-z_101, bit_int*RT_TIMER_PERIOD_MS, MODE_RELATIVE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], 25.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY],-25.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB],-50.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], -120.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
//                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], -15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -70.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK1], 15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK2], 15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == bit_int*29 + music_offset)
            {
                wbPosPelZ->setTaskPos(z_101, bit_int*RT_TIMER_PERIOD_MS, MODE_RELATIVE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], 10.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY], 0.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB],-30.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], -130.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
//                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], 0.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -10.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK1],-15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK2],-15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                setMoveFinger(RF2,-1,1400);//2 both 0 right 1 left
            }
            if(sciTimeCnt == bit_int*30 + music_offset)
            {
                wbPosPelZ->setTaskPos(-z_101, bit_int*RT_TIMER_PERIOD_MS, MODE_RELATIVE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], 25.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY],-25.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB],-50.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], -120.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
//                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], -15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -70.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK1], 15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK2], 15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == bit_int*31 + music_offset)
            {
                wbPosPelZ->setTaskPos(z_101, bit_int*RT_TIMER_PERIOD_MS, MODE_RELATIVE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], 10.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY], 0.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB],-30.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], -130.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
//                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], 0.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -10.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK1],-15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK2],-15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);

            }
            //Punch Right + Left=========================================================================================================
            if(sciTimeCnt == bit_int*32 + music_offset)
            {
//                setMoveFinger(RF1,-1,800);//2 both 0 right 1 left
//                setMoveFinger(RF2,-1,1400);//2 both 0 right 1 left
//                setMoveFinger(RF3,-1,800);//2 both 0 right 1 left
//                setMoveFinger(RF4,-1,800);//2 both 0 right 1 left
//                setMoveFinger(RF5,-1,800);//2 both 0 right 1 left
                printf("now!?\n");
                wbPosPelZ->setTaskPos(-z_101, bit_int*RT_TIMER_PERIOD_MS, MODE_RELATIVE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], -80.f, bit_int*RT_TIMER_PERIOD_MS*2, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR],  10.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -55.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWY],  90.f, bit_int*RT_TIMER_PERIOD_MS*2, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NKY],  -15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK1], 0.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK2], 0.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbPosWST->setTaskPos(15.f, bit_int*RT_TIMER_PERIOD_MS*2., MODE_ABSOLUTE);
            }
            if(sciTimeCnt == bit_int*33 + music_offset)
            {
                wbPosPelZ->setTaskPos(z_101, bit_int*RT_TIMER_PERIOD_MS, MODE_RELATIVE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -10.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
//                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR],  10.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
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
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSP], -80.f, bit_int*RT_TIMER_PERIOD_MS*2, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR],  20.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB], -55.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWY], -90.f, bit_int*RT_TIMER_PERIOD_MS*2, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NKY],   15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbPosWST->setTaskPos(-15.f, bit_int*RT_TIMER_PERIOD_MS*2., MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP],   10.f, bit_int*RT_TIMER_PERIOD_MS*2, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR],   dance101_Move_SR, bit_int*RT_TIMER_PERIOD_MS*2, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB],  -20.f, bit_int*RT_TIMER_PERIOD_MS*2, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == bit_int*37 + music_offset)
            {
                wbPosPelZ->setTaskPos(z_101, bit_int*RT_TIMER_PERIOD_MS, MODE_RELATIVE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB], -10.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], -10.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
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
                z_101 = 0.03;
                wbPosPelZ->setTaskPos(-z_101, bit_int*RT_TIMER_PERIOD_MS, MODE_RELATIVE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], -80.f, bit_int*RT_TIMER_PERIOD_MS*2, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], -20.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -55.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NKY],  -15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbPosWST->setTaskPos(15.f, bit_int*RT_TIMER_PERIOD_MS*2., MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSP],   10.f, bit_int*RT_TIMER_PERIOD_MS*2, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR],  -dance101_Move_SR, bit_int*RT_TIMER_PERIOD_MS*2, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB],  -20.f, bit_int*RT_TIMER_PERIOD_MS*2, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == bit_int*41 + music_offset)
            {
                wbPosPelZ->setTaskPos(z_101-0.05, bit_int*RT_TIMER_PERIOD_MS, MODE_RELATIVE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -10.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR],  10.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == bit_int*42 + music_offset)
            {
                wbPosPelZ->setTaskPos(-z_101, bit_int*RT_TIMER_PERIOD_MS, MODE_RELATIVE);
            }
            if(sciTimeCnt == bit_int*43 + music_offset)
            {
                wbPosPelZ->setTaskPos(z_101-0.05, bit_int*RT_TIMER_PERIOD_MS, MODE_RELATIVE);
            }
            if(sciTimeCnt == bit_int*44 + music_offset)
            {
                wbPosPelZ->setTaskPos(-(z_101-0.05), bit_int*RT_TIMER_PERIOD_MS, MODE_RELATIVE);
            }
            if(sciTimeCnt == bit_int*45 + music_offset)
            {
                wbPosPelZ->setTaskPos(z_101, bit_int*RT_TIMER_PERIOD_MS, MODE_RELATIVE);
            }
            if(sciTimeCnt == bit_int*46 + music_offset)
            {
                wbPosPelZ->setTaskPos(-(z_101-0.05), bit_int*RT_TIMER_PERIOD_MS, MODE_RELATIVE);
                wbPosWST->setTaskPos(0.f, bit_int*RT_TIMER_PERIOD_MS*2., MODE_ABSOLUTE);
            }
            if(sciTimeCnt == bit_int*47 + music_offset)
            {
                wbPosPelZ->setTaskPos(z_101, bit_int*RT_TIMER_PERIOD_MS, MODE_RELATIVE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NKY],  0.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
            }
            //WST Rotate + Left Right Move=========================================================================================================
            if(sciTimeCnt == bit_int*48 + music_offset)
            {
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], -140.f, bit_int*RT_TIMER_PERIOD_MS*4, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY],   70.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -60.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSP], -130.f, bit_int*RT_TIMER_PERIOD_MS*4, MODE_ABSOLUTE);
//                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY], -70.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB], -30.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbPosWST->setTaskPos(20.f, bit_int*RT_TIMER_PERIOD_MS*2, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == bit_int*49 + music_offset)
            {
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY],   70.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -30.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY], -70.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB], -60.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == bit_int*50 + music_offset)
            {
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY],   70.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -60.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY], -70.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB], -30.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbPosWST->setTaskPos(-20.f, bit_int*RT_TIMER_PERIOD_MS*4, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == bit_int*51 + music_offset)
            {
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY],   70.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -30.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY], -70.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB], -60.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == bit_int*52 + music_offset)
            {
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY],   70.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -60.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY], -70.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB], -30.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == bit_int*53 + music_offset)
            {
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY],   70.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -30.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY], -70.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB], -60.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == bit_int*54 + music_offset)
            {
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY],   70.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -60.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY], -70.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB], -30.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbPosWST->setTaskPos(0.f, bit_int*RT_TIMER_PERIOD_MS*2, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == bit_int*55 + music_offset)
            {
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY],   70.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -30.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY], -70.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB], -60.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
            }
            //Pelvis move + Left move +  Right move=========================================================================================================
            if(sciTimeCnt == bit_int*56 + music_offset)
            {
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP],  13.f, bit_int*20, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], -10.f, bit_int*11, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY],   0.f, bit_int*9, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -30.f, bit_int*9, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWY],   0.f, bit_int*9*2, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSP],  13.f, bit_int*20, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR],  10.f, bit_int*11, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY],   0.f, bit_int*9, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB], -30.f, bit_int*9, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWY],   0.f, bit_int*9*2, MODE_ABSOLUTE);
            }

            //Pelvis move + Left move +  Right move=========================================================================================================
            if(sciTimeCnt == bit_int*64 + music_offset)
            {
                z_101 = 0.05;
                wbPosPelZ->setTaskPos(-z_101, bit_int*RT_TIMER_PERIOD_MS, MODE_RELATIVE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR],-25.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY], 25.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB],-50.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], 25.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY],-25.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB],-50.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK1], 15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK2], 15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == bit_int*65 + music_offset)
            {
                wbPosPelZ->setTaskPos(z_101, bit_int*RT_TIMER_PERIOD_MS, MODE_RELATIVE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR],-10.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY],  0.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB],-30.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], 10.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY],  0.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB],-30.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK1],-15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK2],-15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == bit_int*66 + music_offset)
            {
                wbPosPelZ->setTaskPos(-z_101, bit_int*RT_TIMER_PERIOD_MS, MODE_RELATIVE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR],-25.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY], 25.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB],-50.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], 25.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY],-25.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB],-50.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK1], 15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK2], 15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == bit_int*67 + music_offset)
            {
                wbPosPelZ->setTaskPos(z_101, bit_int*RT_TIMER_PERIOD_MS, MODE_RELATIVE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR],-10.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY],  0.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB],-30.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], 10.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY],  0.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB],-30.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK1],-15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK2],-15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == bit_int*68 + music_offset)
            {
                wbPosPelZ->setTaskPos(-z_101, bit_int*RT_TIMER_PERIOD_MS, MODE_RELATIVE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR],-25.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY], 25.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB],-50.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], 25.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY],-25.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB],-50.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK1], 15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK2], 15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == bit_int*69 + music_offset)
            {
                wbPosPelZ->setTaskPos(z_101, bit_int*RT_TIMER_PERIOD_MS, MODE_RELATIVE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR],-10.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY],  0.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB],-30.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], 10.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY],  0.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB],-30.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK1],-15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK2],-15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == bit_int*70 + music_offset)
            {
                wbPosPelZ->setTaskPos(-z_101, bit_int*RT_TIMER_PERIOD_MS, MODE_RELATIVE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR],-25.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY], 25.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB],-50.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], 25.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY],-25.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB],-50.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK1], 15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK2], 15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == bit_int*71 + music_offset)
            {
                wbPosPelZ->setTaskPos(z_101, bit_int*RT_TIMER_PERIOD_MS, MODE_RELATIVE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR],-10.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY],  0.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB],-30.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], 10.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY],  0.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB],-30.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK1],-15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK2],-15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
            }
            //Pelvis move + Left move +  Right move=========================================================================================================
            if(sciTimeCnt == bit_int*72 + music_offset)
            {
                z_101 = 0.05;
                wbPosPelZ->setTaskPos(-z_101, bit_int*RT_TIMER_PERIOD_MS, MODE_RELATIVE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR],-25.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY], 25.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB],-50.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], 25.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY],-25.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB],-50.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK1], 15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK2], 15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == bit_int*73 + music_offset)
            {
                wbPosPelZ->setTaskPos(z_101, bit_int*RT_TIMER_PERIOD_MS, MODE_RELATIVE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR],-10.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY],  0.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB],-30.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], 10.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY],  0.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB],-30.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK1],-15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK2],-15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == bit_int*74 + music_offset)
            {
                wbPosPelZ->setTaskPos(-z_101, bit_int*RT_TIMER_PERIOD_MS, MODE_RELATIVE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR],-25.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY], 25.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB],-50.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], 25.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY],-25.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB],-50.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK1], 15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK2], 15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == bit_int*75 + music_offset)
            {
                wbPosPelZ->setTaskPos(z_101, bit_int*RT_TIMER_PERIOD_MS, MODE_RELATIVE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR],-10.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY],  0.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB],-30.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], 10.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY],  0.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB],-30.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK1],-15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK2],-15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == bit_int*76 + music_offset)
            {
                wbPosPelZ->setTaskPos(-z_101, bit_int*RT_TIMER_PERIOD_MS, MODE_RELATIVE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR],-25.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY], 25.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB],-50.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], 25.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY],-25.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB],-50.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK1], 15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK2], 15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == bit_int*77 + music_offset)
            {
                wbPosPelZ->setTaskPos(z_101, bit_int*RT_TIMER_PERIOD_MS, MODE_RELATIVE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR],-10.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY],  0.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB],-30.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], 10.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY],  0.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB],-30.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK1],-15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK2],-15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == bit_int*78 + music_offset)
            {
                wbPosPelZ->setTaskPos(-z_101, bit_int*RT_TIMER_PERIOD_MS, MODE_RELATIVE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR],-25.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY], 25.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB],-50.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], 25.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY],-25.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB],-50.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK1], 15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK2], 15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == bit_int*79 + music_offset)
            {
                wbPosPelZ->setTaskPos(z_101, bit_int*RT_TIMER_PERIOD_MS, MODE_RELATIVE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR],-10.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY],  0.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB],-30.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], 10.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY],  0.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB],-30.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK1],-15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK2],-15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
            }
            //Pelvis move + Left move +  Right move=========================================================================================================
            if(sciTimeCnt == bit_int*80 + music_offset)
            {
                z_101 = 0.05;
                wbPosPelZ->setTaskPos(-z_101, bit_int*RT_TIMER_PERIOD_MS, MODE_RELATIVE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR],-25.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY], 25.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB],-50.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], 25.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY],-25.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB],-50.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK1], 15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK2], 15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == bit_int*81 + music_offset)
            {
                wbPosPelZ->setTaskPos(z_101, bit_int*RT_TIMER_PERIOD_MS, MODE_RELATIVE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR],-10.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY],  0.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB],-30.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], 10.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY],  0.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB],-30.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK1],-15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK2],-15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == bit_int*82 + music_offset)
            {
                wbPosPelZ->setTaskPos(-z_101, bit_int*RT_TIMER_PERIOD_MS, MODE_RELATIVE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR],-25.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY], 25.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB],-50.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], 25.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY],-25.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB],-50.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK1], 15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK2], 15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == bit_int*83 + music_offset)
            {
                wbPosPelZ->setTaskPos(z_101, bit_int*RT_TIMER_PERIOD_MS, MODE_RELATIVE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR],-10.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY],  0.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB],-30.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], 10.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY],  0.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB],-30.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK1],-15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK2],-15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == bit_int*84 + music_offset)
            {
                wbPosPelZ->setTaskPos(-z_101, bit_int*RT_TIMER_PERIOD_MS, MODE_RELATIVE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR],-25.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY], 25.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB],-50.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], 25.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY],-25.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB],-50.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK1], 15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK2], 15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == bit_int*85 + music_offset)
            {
                wbPosPelZ->setTaskPos(z_101, bit_int*RT_TIMER_PERIOD_MS, MODE_RELATIVE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR],-10.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY],  0.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB],-30.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], 10.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY],  0.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB],-30.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK1],-15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK2],-15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == bit_int*86 + music_offset)
            {
                wbPosPelZ->setTaskPos(-z_101, bit_int*RT_TIMER_PERIOD_MS, MODE_RELATIVE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR],-25.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY], 25.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB],-50.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], 25.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY],-25.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB],-50.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK1], 15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK2], 15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == bit_int*87 + music_offset)
            {
                wbPosPelZ->setTaskPos(z_101, bit_int*RT_TIMER_PERIOD_MS, MODE_RELATIVE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR],-10.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY],  0.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB],-30.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], 10.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY],  0.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB],-30.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK1],-15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK2],-15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
            }
            //Pelvis move + Left move +  Right move=========================================================================================================
            if(sciTimeCnt == bit_int*88 + music_offset)
            {
                z_101 = 0.05;
                wbPosPelZ->setTaskPos(-z_101, bit_int*RT_TIMER_PERIOD_MS, MODE_RELATIVE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR],-25.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY], 25.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB],-50.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], 25.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY],-25.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB],-50.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK1], 15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK2], 15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == bit_int*89 + music_offset)
            {
                wbPosPelZ->setTaskPos(z_101, bit_int*RT_TIMER_PERIOD_MS, MODE_RELATIVE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR],-10.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY],  0.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB],-30.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], 10.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY],  0.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB],-30.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK1],-15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK2],-15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == bit_int*90 + music_offset)
            {
                wbPosPelZ->setTaskPos(-z_101, bit_int*RT_TIMER_PERIOD_MS, MODE_RELATIVE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR],-25.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY], 25.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB],-50.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], 25.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY],-25.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB],-50.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK1], 15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK2], 15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == bit_int*91 + music_offset)
            {
                wbPosPelZ->setTaskPos(z_101, bit_int*RT_TIMER_PERIOD_MS, MODE_RELATIVE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR],-10.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY],  0.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB],-30.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], 10.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY],  0.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB],-30.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK1],-15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK2],-15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == bit_int*92 + music_offset)
            {
                wbPosPelZ->setTaskPos(-z_101, bit_int*RT_TIMER_PERIOD_MS, MODE_RELATIVE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR],-25.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY], 25.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB],-50.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], 25.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY],-25.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB],-50.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK1], 15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK2], 15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == bit_int*93 + music_offset)
            {
                wbPosPelZ->setTaskPos(z_101, bit_int*RT_TIMER_PERIOD_MS, MODE_RELATIVE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR],-10.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY],  0.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB],-30.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], 10.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY],  0.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB],-30.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK1],-15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK2],-15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == bit_int*94 + music_offset)
            {
                wbPosPelZ->setTaskPos(-z_101, bit_int*RT_TIMER_PERIOD_MS, MODE_RELATIVE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR],-25.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY], 25.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB],-50.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], 25.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY],-25.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB],-50.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK1], 15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK2], 15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == bit_int*95 + music_offset)
            {
                wbPosPelZ->setTaskPos(z_101, bit_int*RT_TIMER_PERIOD_MS, MODE_RELATIVE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR],-10.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY],  0.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB],-30.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], 10.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY],  0.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB],-30.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK1],-15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK2],-15.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
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
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP],-130.f, bit_int*RT_TIMER_PERIOD_MS*4, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], -10.f, bit_int*RT_TIMER_PERIOD_MS*4, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY],   0.f, bit_int*RT_TIMER_PERIOD_MS*4, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -30.f, bit_int*RT_TIMER_PERIOD_MS*4, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWY],   0.f, bit_int*RT_TIMER_PERIOD_MS*4, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSP],-130.f, bit_int*RT_TIMER_PERIOD_MS*4, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR],  10.f, bit_int*RT_TIMER_PERIOD_MS*4, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY],   0.f, bit_int*RT_TIMER_PERIOD_MS*4, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB], -30.f, bit_int*RT_TIMER_PERIOD_MS*4, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWY],   0.f, bit_int*RT_TIMER_PERIOD_MS*4, MODE_ABSOLUTE);
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
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP],  10.f, t1_101*2, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], -10.f, t1_101*2, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY],   0.f, t1_101*2, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -30.f, t1_101*2, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWY],   0.f, t1_101*2, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSP],  10.f, t1_101*2, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR],  10.f, t1_101*2, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY],   0.f, t1_101*2, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB], -30.f, t1_101*2, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWY],   0.f, t1_101*2, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NKY],   0.f, t1_101, MODE_ABSOLUTE);
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
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], -135.f, t1_101, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], 15.f, t1_101, MODE_ABSOLUTE);
//                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -10.f, t1_101, MODE_ABSOLUTE);
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
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], -70.f, t1_hapum, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], 0.f, t1_hapum, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY], 60.f, t1_hapum, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -90.f, t1_hapum, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWY], -90.f, t1_hapum, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == t1_101_int)
            {
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -110.f, 100*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == t1_101_int + 100*1)
            {
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -90.f, 100*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == t1_101_int + 100*2)
            {
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -110.f, 100*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == t1_101_int + 100*3)
            {
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -90.f, 100*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
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
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP],  -70.f, MotionTime, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR],   -10.f, MotionTime, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY],   00.f, MotionTime, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -90.f, MotionTime, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWY],   60.f, MotionTime, MODE_ABSOLUTE);
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
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP],  -70.f, MotionTime, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR],   -10.f, MotionTime, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY],   00.f, MotionTime, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -90.f, MotionTime, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWY],   60.f, MotionTime, MODE_ABSOLUTE);
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
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP],  -70.f, MotionTime, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR],   -10.f, MotionTime, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY],   00.f, MotionTime, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -90.f, MotionTime, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWY],   60.f, MotionTime, MODE_ABSOLUTE);
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
//                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], -60.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSP], -60.f, 800.f, MODE_ABSOLUTE);
//                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], -20.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR],  20.f, 800.f, MODE_ABSOLUTE);
//                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -80.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB], -80.f, 800.f, MODE_ABSOLUTE);

//                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWY],  60.f, 800.f, MODE_ABSOLUTE);
//                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWY], -60.f, 800.f, MODE_ABSOLUTE);

                wbPosWST->setTaskPos(-20, 800, MODE_RELATIVE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NKY],-30.f, 800.f, MODE_ABSOLUTE);
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
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], -60.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSP], -60.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], -20.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR],  20.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -80.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB], -80.f, 800.f, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWY],  80.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWY], -80.f, 800.f, MODE_ABSOLUTE);

                wbPosWST->setTaskPos(50, 900, MODE_RELATIVE);
//                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NKY], 30.f, 800.f, MODE_ABSOLUTE);
                setMoveFinger(2,1,1000);//2 both 0 right 1 left RF1
            }
            else if(sciTimeCnt == 160)
            {
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], -30.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY], -20.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWP],  20.f, 400.f, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR],  30.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY],  20.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWP],  20.f, 400.f, MODE_ABSOLUTE);
            }
            else if(sciTimeCnt == 240)
            {
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], -10.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY],   0.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWP],   0.f, 400.f, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR],  10.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY],   0.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWP],   0.f, 400.f, MODE_ABSOLUTE);
            }
//            else if(sciTimeCnt == 320)
//            {
//                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], -30.f, 400.f, MODE_ABSOLUTE);
//                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY], -20.f, 400.f, MODE_ABSOLUTE);
//                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWP],  20.f, 400.f, MODE_ABSOLUTE);

//                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR],  30.f, 400.f, MODE_ABSOLUTE);
//                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY],  20.f, 400.f, MODE_ABSOLUTE);
//                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWP],  20.f, 400.f, MODE_ABSOLUTE);
//            }
//            else if(sciTimeCnt == 400)
//            {
//                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], -10.f, 400.f, MODE_ABSOLUTE);
//                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY],   0.f, 400.f, MODE_ABSOLUTE);
//                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWP],   0.f, 400.f, MODE_ABSOLUTE);

//                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR],  10.f, 400.f, MODE_ABSOLUTE);
//                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY],   0.f, 400.f, MODE_ABSOLUTE);
//                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWP],   0.f, 400.f, MODE_ABSOLUTE);
//            }
//            else if(sciTimeCnt == 480)
//            {
//                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], -30.f, 400.f, MODE_ABSOLUTE);
//                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY], -20.f, 400.f, MODE_ABSOLUTE);
//                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWP],  20.f, 400.f, MODE_ABSOLUTE);

//                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR],  30.f, 400.f, MODE_ABSOLUTE);
//                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY],  20.f, 400.f, MODE_ABSOLUTE);
//                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWP],  20.f, 400.f, MODE_ABSOLUTE);
//            }
//            else if(sciTimeCnt == 560)
//            {
//                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], -10.f, 400.f, MODE_ABSOLUTE);
//                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY],   0.f, 400.f, MODE_ABSOLUTE);
//                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWP],   0.f, 400.f, MODE_ABSOLUTE);

//                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR],  10.f, 400.f, MODE_ABSOLUTE);
//                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY],   0.f, 400.f, MODE_ABSOLUTE);
//                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWP],   0.f, 400.f, MODE_ABSOLUTE);
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
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSP], -50.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], 0.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB], -110.f, 800.f, MODE_ABSOLUTE);
            }
            else if(sciTimeCnt == 360)
            {
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSP], -110.f, 600.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], 0.f, 600.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB], -20.f, 600.f, MODE_ABSOLUTE);
                setMoveFinger(1,1,1000);//2 both 0 right 1 left RF1
            }
            else if(sciTimeCnt == 580)
            {
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSP], 10.f, 1000.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], 10.f, 1000.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB], -30.f, 1000.f, MODE_ABSOLUTE);
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
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], -50.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], 0.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -110.f, 800.f, MODE_ABSOLUTE);
            }
            else if(sciTimeCnt == 360)
            {
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], -110.f, 600.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], 0.f, 600.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -20.f, 600.f, MODE_ABSOLUTE);
                setMoveFinger(0,1,1000);//2 both 0 right 1 left RF1
            }
            else if(sciTimeCnt == 580)
            {
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], 10.f, 1000.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], -10.f, 1000.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -30.f, 1000.f, MODE_ABSOLUTE);
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
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], -60.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSP], -60.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], -20.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR],  20.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -80.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB], -80.f, 800.f, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWY],  80.f, 800.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWY], -80.f, 800.f, MODE_ABSOLUTE);

                wbPosWST->setTaskPos(20, 900, MODE_RELATIVE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NKY], 30.f, 800.f, MODE_ABSOLUTE);
                setMoveFinger(2,-1,1000);//2 both 0 right 1 left RF1
            }
            else if(sciTimeCnt == 160)
            {
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], -30.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY],  20.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWP],  20.f, 400.f, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR],  30.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY], -20.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWP],  20.f, 400.f, MODE_ABSOLUTE);
            }
            else if(sciTimeCnt == 240)
            {
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], -10.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY],   0.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWP],   0.f, 400.f, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR],  10.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY],   0.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWP],   0.f, 400.f, MODE_ABSOLUTE);
            }
            else if(sciTimeCnt == 320)
            {
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], -30.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY],  20.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWP],  20.f, 400.f, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR],  30.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY], -20.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWP],  20.f, 400.f, MODE_ABSOLUTE);
            }
            else if(sciTimeCnt == 400)
            {
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], -10.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY],   0.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWP],   0.f, 400.f, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR],  10.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY],   0.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWP],   0.f, 400.f, MODE_ABSOLUTE);
            }
            else if(sciTimeCnt == 480)
            {
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], -30.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY],  20.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWP],  20.f, 400.f, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR],  30.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY], -20.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWP],  20.f, 400.f, MODE_ABSOLUTE);
            }
            else if(sciTimeCnt == 560)
            {
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], -10.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY],   0.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWP],   0.f, 400.f, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR],  10.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY],   0.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWP],   0.f, 400.f, MODE_ABSOLUTE);
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




            //--------------------------------------------------------------------------------------
        case HANDSHAKE_READY:
            if(sciTimeCnt == 0){
                FILE_LOG(logSUCCESS) << "Pyeongchang_DEMO: HANDSHAKE_READY";

                                Upperbody_Gain_Override();
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], 20.f, 1000.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], -10.5f, 1000.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY], 0.0f, 1000.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -100.f, 1000.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWY], -10.f, 1000.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWP], 10.f, 1000.f, MODE_ABSOLUTE);
            }if(sciTimeCnt == 200){
                FILE_LOG(logSUCCESS) << "Pyeongchang_DEMO: HANDSHAKE_READY";
//                for(int i=RF1; i<=RF5; i++) wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[i], -10.f, 700, MODE_ABSOLUTE);
//                Upperbody_Gain_Override();

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK1], 10.f, 1000.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK2], 10.f, 1000.f, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], -58.f, 1000.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], 21.5f, 1000.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY], 20.4f, 1000.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -28.f, 1000.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWY], -20.85f, 1000.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWP], 10.f, 1000.f, MODE_ABSOLUTE);
            }if(sciTimeCnt == 100+200){
//                Upperbody_Gain_Override();
                for(int i=RF1; i<=RF5; i++) wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[i], -10.f, 700, MODE_ABSOLUTE);
            }if(sciTimeCnt == 200+200){
                for(int i=RF1; i<=RF5; i++) wbUBJoint->motionJoint[i]->RefAngleCurrent = 90.0f;//setMoveJointAngle(wbUBJoint->motionJoint[i], 10.f, 500, MODE_ABSOLUTE);
            }if(sciTimeCnt == 200+280){
                for(int i=RF1; i<=RF5; i++) wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[i], 0.f, 500, MODE_ABSOLUTE);
            }if(sciTimeCnt == 200+400){
                scidemo_num = NO_ACT;
//                Control_OnOff_Flag = true;
                FILE_LOG(logINFO) << "Pyeongchang_DEMO: HANDSHAKE_READY" << "END!!!";
            }break;
        case HANDSHAKE_LOWGAIN:
            if(sciTimeCnt == 0){
                FILE_LOG(logSUCCESS) << "Pyeongchang_DEMO: HANDSHAKE_LOWGAIN";
                Upperbody_Gain_Override();
            }if(sciTimeCnt == 10){
                scidemo_num = NO_ACT;
            }break;
        case HANDSHAKE_GRIP_ON:
            if(sciTimeCnt == 0){
                FILE_LOG(logSUCCESS) << "Pyeongchang_DEMO: HANDSHAKE_GRIP_ON";
                setMoveFinger(0,-1,700);//2 both 0 right 1 left RF1
            }if(sciTimeCnt == 10){
                scidemo_num = NO_ACT;
            }break;
        case HANDSHAKE_GRIP_OFF:
            if(sciTimeCnt == 0){
                FILE_LOG(logSUCCESS) << "Pyeongchang_DEMO: HANDSHAKE_GRIP_OFF";
                setMoveFinger(0,1,700);//2 both 0 right 1 left RF1
            }if(sciTimeCnt == 10){
                scidemo_num = NO_ACT;
            }break;
        case HANDSHAKE_HIGHGAIN:
            if(sciTimeCnt == 0){
                FILE_LOG(logSUCCESS) << "Pyeongchang_DEMO: HANDSHAKE_HIGHGAIN";
//                Upperbody_Gain_Lock();
            }if(sciTimeCnt == 10){
                scidemo_num = NO_ACT;
            }break;
        case HANDSHAKE_INIT:
            if(sciTimeCnt == 0){
                FILE_LOG(logSUCCESS) << "Pyeongchang_DEMO: HANDSHAKE_INIT";
                Upperbody_Gain_Lock();
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK1], 0.f, 1000.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK2], 0.f, 1000.f, MODE_ABSOLUTE);
                wbUBJoint->motionJoint[RSP]->RefAngleCurrent = Shm->ENCODER[MC_GetID(RSP)][MC_GetCH(RSP)].CurrentPosition;
                wbUBJoint->motionJoint[RSR]->RefAngleCurrent = Shm->ENCODER[MC_GetID(RSR)][MC_GetCH(RSR)].CurrentPosition;
                wbUBJoint->motionJoint[RSY]->RefAngleCurrent = Shm->ENCODER[MC_GetID(RSY)][MC_GetCH(RSY)].CurrentPosition;
                wbUBJoint->motionJoint[REB]->RefAngleCurrent = Shm->ENCODER[MC_GetID(REB)][MC_GetCH(REB)].CurrentPosition;
                wbUBJoint->motionJoint[RWY]->RefAngleCurrent = Shm->ENCODER[MC_GetID(RWY)][MC_GetCH(RWY)].CurrentPosition;
                wbUBJoint->motionJoint[RWP]->RefAngleCurrent = Shm->ENCODER[MC_GetID(RWP)][MC_GetCH(RWP)].CurrentPosition;

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], 10.f, 1500.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], -10.f, 1500.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY], 0.f, 1500.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -30.f, 1500.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWY], 0.f, 1500.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWP], 0.f, 1500.f, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == 300){
                initHeadHand(1);//0 head . 1 hand . 2 hand+head
            }if(sciTimeCnt == 350){
                scidemo_num = NO_ACT;
//                Control_OnOff_Flag = false;
                FILE_LOG(logINFO) << "Pyeongchang_DEMO: HANDSHAKE_INIT" << "END!!!";
            }break;
            //--------------------------------------------------------------------------------------
        case BOUQUET_READY:
            if(sciTimeCnt == 0){
                FILE_LOG(logSUCCESS) << "Pyeongchang_DEMO: BOUQUET_READY";
                setMoveFinger(0,1,700);//2 both 0 right 1 left RF1
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], -15.f, 2000.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], 0.f, 2000.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY], 25.f, 2000.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -90.f, 2000.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWY], -95.f, 2000.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWP], -20.f, 2000.f, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSP], -15.f, 2000.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], 0.f, 2000.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY], -25.f, 2000.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB], -90.f, 2000.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWY], 95.f, 2000.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWP], -20.f, 2000.f, MODE_ABSOLUTE);

            }if(sciTimeCnt == 400){
                scidemo_num = NO_ACT;
                FILE_LOG(logINFO) << "Pyeongchang_DEMO: BOUQUET_READY" << "END!!!";
            }break;
        case BOUQUET_GIVE:
            if(sciTimeCnt == 0){
                FILE_LOG(logSUCCESS) << "Pyeongchang_DEMO: BOUQUET_GIVE";
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], -65.f, 2000.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], 10.f, 2000.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -45.f, 2000.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWY], -110.f, 2000.f, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSP], -65.f, 2000.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], -10.f, 2000.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB], -45.f, 2000.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWY], 110.f, 2000.f, MODE_ABSOLUTE);
            }if(sciTimeCnt == 400){
                scidemo_num = NO_ACT;
                FILE_LOG(logINFO) << "Pyeongchang_DEMO: BOUQUET_GIVE" << "END!!!";
            }break;
        case BOUQUET_INIT:
            if(sciTimeCnt == 0){
                FILE_LOG(logSUCCESS) << "Pyeongchang_DEMO: BOUQUET_INIT";
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], 10.f, 1500.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], -10.f, 1500.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY], 0.f, 1500.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -30.f, 1500.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWY], 0.f, 1500.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWP], 0.f, 1500.f, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSP], 10.f, 1500.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], 10.f, 1500.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY], 0.f, 1500.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB], -30.f, 1500.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWY], 0.f, 1500.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWP], 0.f, 1500.f, MODE_ABSOLUTE);
            }if(sciTimeCnt == 350){
                initHeadHand(1);
            }if(sciTimeCnt == 500){
                scidemo_num = NO_ACT;
                FILE_LOG(logINFO) << "Pyeongchang_DEMO: BOUQUET_INIT" << "END!!!";
            }break;
            //--------------------------------------------------------------------------------------

        case CLAP_READY:
            if(sciTimeCnt == 0){
                FILE_LOG(logSUCCESS) << "Pyeongchang_DEMO: CLAP_READY";
                setMoveFinger(2,1,700);//2 both 0 right 1 left RF1
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], -82.5f, 1500.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], 18.97f, 1500.f, MODE_ABSOLUTE);
//                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY], 30.0f, 1500.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY], 15.0f, 1500.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -72.56f, 1500.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWY], -15.f, 1500.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWP], 38.15f, 1500.f, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSP], -82.5f, 1500.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], -18.95f, 1500.f, MODE_ABSOLUTE);
//                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY], -30.00f, 1500.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY], -15.00f, 1500.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB], -72.56f, 1500.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWY],  15.0f, 1500.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWP], 38.15f, 1500.f, MODE_ABSOLUTE);
            }if(sciTimeCnt == 300){
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY], 42.0f, 250.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY], -42.00f, 250.f, MODE_ABSOLUTE);
            }if(sciTimeCnt == 350){
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY], 30.0f, 150.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY], -30.00f, 150.f, MODE_ABSOLUTE);
                if(clap_cnt == 1){
                    sciTimeCnt = 410;
                }
            }if(sciTimeCnt == 381 && clap_cnt > 1){
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY], 42.0f, 150.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY], -42.00f, 150.f, MODE_ABSOLUTE);
            }if(sciTimeCnt == 411 && clap_cnt > 1){
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY], 30.0f, 150.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY], -30.00f, 150.f, MODE_ABSOLUTE);
                if(--clap_cnt > 1){
                    sciTimeCnt = 350;
                }
            }if(sciTimeCnt == 441){
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], 10.f, 1500.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], -10.f, 1500.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY], 0.f, 1500.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -30.f, 1500.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWY], 0.f, 1500.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWP], 0.f, 1500.f, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSP], 10.f, 1500.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], 10.f, 1500.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY], 0.f, 1500.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB], -30.f, 1500.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWY], 0.f, 1500.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWP], 0.f, 1500.f, MODE_ABSOLUTE);
            }if(sciTimeCnt == 751){
                scidemo_num = NO_ACT;
                FILE_LOG(logINFO) << "Pyeongchang_DEMO: CLAP_READY" << "END!!!";
            }break;
        case CLAP_INIT:
            if(sciTimeCnt == 0){
                FILE_LOG(logINFO) << "Pyeongchang_DEMO: CLAP_INIT";
            }if(sciTimeCnt == 10){
                scidemo_num = NO_ACT;
            }break;
            //--------------------------------------------------------------------------------------

        case HEAD_YES:
            if(sciTimeCnt == 0 + 120){
                FILE_LOG(logSUCCESS) << "Pyeongchang_DEMO: HEAD_YES";
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK1], -15.f, 200.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK2], -15.f, 200.f, MODE_ABSOLUTE);
            }if(sciTimeCnt == 40 + 120){
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK1], 0.f, 200.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK2], 0.f, 200.f, MODE_ABSOLUTE);
            }if(sciTimeCnt == 80 + 120){
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK1], -15.f, 200.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK2], -15.f, 200.f, MODE_ABSOLUTE);
            }if(sciTimeCnt == 120 + 120){
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK1], 0.f, 200.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK2], 0.f, 200.f, MODE_ABSOLUTE);
            }if(sciTimeCnt == 160 + 120){
                scidemo_num = NO_ACT;
                FILE_LOG(logINFO) << "Pyeongchang_DEMO: HEAD_YES" << "END!!!";
            }break;
        case HEAD_NO:
            if(sciTimeCnt == 0){
                FILE_LOG(logSUCCESS) << "Pyeongchang_DEMO: HEAD_NO";
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NKY], -15.f, 200.f, MODE_ABSOLUTE);
            }if(sciTimeCnt == 40){
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NKY], 15.f, 400.f, MODE_ABSOLUTE);
            }if(sciTimeCnt == 120){
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NKY], -15.f, 400.f, MODE_ABSOLUTE);
            }if(sciTimeCnt == 200){
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NKY], 0.f, 200.f, MODE_ABSOLUTE);
            }if(sciTimeCnt == 240){
                scidemo_num = NO_ACT;
                FILE_LOG(logINFO) << "Pyeongchang_DEMO: HEAD_NO" << "END!!!";
            }break;
        case HEAD_STRETCHING:
            if(sciTimeCnt == 0){
                FILE_LOG(logSUCCESS) << "Pyeongchang_DEMO: HEAD_STRETCHING";
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK1], -15.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK2], -15.f, 400.f, MODE_ABSOLUTE);
            }if(sciTimeCnt < 580 && sciTimeCnt > 80){
                wbUBJoint->motionJoint[NK1]->RefAngleCurrent = -1.f*(15.f*cos(2*M_PI*(sciTimeCnt-80)/500.f)) -1.f*(15.f*sin(2*M_PI*(sciTimeCnt-80)/500.f));
                wbUBJoint->motionJoint[NK2]->RefAngleCurrent = -1.f*(15.f*cos(2*M_PI*(sciTimeCnt-80)/500.f))+1.f*(15.f*sin(2*M_PI*(sciTimeCnt-80)/500.f));
            }if(sciTimeCnt == 580){
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK1], 0.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK2], 0.f, 400.f, MODE_ABSOLUTE);
            }if(sciTimeCnt == 660){
                scidemo_num = NO_ACT;
            }break;
        case HEAD_INIT:
            if(sciTimeCnt == 0){
                FILE_LOG(logSUCCESS) << "Pyeongchang_DEMO: HEAD_INIT";
            }if(sciTimeCnt == 10){
                scidemo_num = NO_ACT;
            }break;
            //--------------------------------------------------------------------------------------

        case BOW_ONE:
            if(sciTimeCnt == 0){
                FILE_LOG(logSUCCESS) << "Pyeongchang_DEMO: BOW_ONE";
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], -85.f, 1000.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], 12.f, 1000.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY], 0.f, 1000.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -55.f, 1000.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWY], 85.f, 1000.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWP], 0.f, 1000.f, MODE_ABSOLUTE);
            }if(sciTimeCnt == 200){
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY], 10.f, 500.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWP], -10.f, 500.f, MODE_ABSOLUTE);
            }if(sciTimeCnt == 300){
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY], -10.f, 500.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWP], 10.f, 500.f, MODE_ABSOLUTE);
            }if(sciTimeCnt == 400){
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY], 10.f, 500.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWP], -10.f, 500.f, MODE_ABSOLUTE);
            }if(sciTimeCnt == 500){
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY], -10.f, 500.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWP], 10.f, 500.f, MODE_ABSOLUTE);
            }if(sciTimeCnt == 600){
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY], 10.f, 500.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWP], -10.f, 500.f, MODE_ABSOLUTE);
            }if(sciTimeCnt == 700){
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY], -10.f, 500.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWP], 10.f, 500.f, MODE_ABSOLUTE);
            }if(sciTimeCnt == 800){
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY], 10.f, 500.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWP], -10.f, 500.f, MODE_ABSOLUTE);
            }if(sciTimeCnt == 900){
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY], -10.f, 500.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWP], 10.f, 500.f, MODE_ABSOLUTE);
            }if(sciTimeCnt == 1050){
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], 10.f, 1500.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], -10.f, 1500.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY], 0.f, 1500.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -30.f, 1500.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWY], 0.f, 1500.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWP], 0.f, 1500.f, MODE_ABSOLUTE);
            }if(sciTimeCnt == 1350){
                scidemo_num = NO_ACT;
                FILE_LOG(logINFO) << "Pyeongchang_DEMO: BOW_ONE" << "END";
            }break;
        case BOW_TWO:

            if(sciTimeCnt == 0){
                FILE_LOG(logSUCCESS) << "Pyeongchang_DEMO: BOW_TWO";
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], -60.f, 1000.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], 0.f, 1000.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY], 10.f, 1000.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -80.f, 1000.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWY], 70.f, 1000.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWP], 0.f, 1000.f, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSP], -60.f, 1000.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], 0.f, 1000.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY], -10.f, 1000.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB], -80.f, 1000.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWY], -70.f, 1000.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWP], 0.f, 1000.f, MODE_ABSOLUTE);
            }if(sciTimeCnt == 200){
                wbPosWST->setTaskPos(10, 800, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], -60.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], -20.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY], -20.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -80.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWY], 80.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWP], 20.f, 400.f, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSP], -60.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], 20.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY], 20.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB], -80.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWY], -80.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWP], 20.f, 400.f, MODE_ABSOLUTE);
            }if(sciTimeCnt == 280){
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], -60.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], 0.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY], 10.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -80.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWY], 70.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWP], 0.f, 400.f, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSP], -60.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], 0.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY], -10.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB], -80.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWY], -70.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWP], 0.f, 400.f, MODE_ABSOLUTE);
            }if(sciTimeCnt == 360){
                wbPosWST->setTaskPos(-10, 800, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], -60.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], -20.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY], -20.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -80.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWY], 80.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWP], 20.f, 400.f, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSP], -60.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], 20.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY], 20.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB], -80.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWY], -80.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWP], 20.f, 400.f, MODE_ABSOLUTE);
            }if(sciTimeCnt == 440){
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], -60.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], 0.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY], 10.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -80.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWY], 70.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWP], 0.f, 400.f, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSP], -60.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], 0.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY], -10.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB], -80.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWY], -70.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWP], 0.f, 400.f, MODE_ABSOLUTE);
            }if(sciTimeCnt == 520){
                wbPosWST->setTaskPos(0, 800, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], -60.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], -20.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY], -20.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -80.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWY], 80.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWP], 20.f, 400.f, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSP], -60.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], 20.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY], 20.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB], -80.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWY], -80.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWP], 20.f, 400.f, MODE_ABSOLUTE);
            }if(sciTimeCnt == 600){
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], -60.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], 0.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY], 10.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -80.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWY], 70.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWP], 0.f, 400.f, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSP], -60.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], 0.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY], -10.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB], -80.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWY], -70.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWP], 0.f, 400.f, MODE_ABSOLUTE);
            }
            /*if(sciTimeCnt == 680){
                wbPosWST->setTaskPos(-10, 800, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], -60.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], -20.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY], -20.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -80.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWY], 80.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWP], 20.f, 400.f, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSP], -60.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], 20.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY], 20.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB], -80.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWY], -80.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWP], 20.f, 400.f, MODE_ABSOLUTE);
            }if(sciTimeCnt == 760){
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], -60.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], 0.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY], 10.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -80.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWY], 70.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWP], 0.f, 400.f, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSP], -60.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], 0.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY], -10.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB], -80.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWY], -70.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWP], 0.f, 400.f, MODE_ABSOLUTE);
            }if(sciTimeCnt == 840){
                wbPosWST->setTaskPos(0, 800, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], 10.f, 1500.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], -10.f, 1500.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY], 0.f, 1500.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -30.f, 1500.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWY], 0.f, 1500.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWP], 0.f, 1500.f, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSP], 10.f, 1500.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], 10.f, 1500.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY], 0.f, 1500.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB], -30.f, 1500.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWY], 0.f, 1500.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWP], 0.f, 1500.f, MODE_ABSOLUTE);
            }*/
            if(sciTimeCnt == 680){//1140
                scidemo_num = BOW_LAST_MOTION;
                sciTimeCnt = 0;
                printf("\n ========== %f \n",sciTimeCnt*0.005);
            }
            break;

        case BOW_GREETING:
            if(fileReadFlag == ENABLE){
                LeadTextData();
            }ChaseTextData();
            if(sciTimeCnt == 0){
                FILE_LOG(logSUCCESS) << "Pyeongchang_DEMO: BOW_TWO_W";
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWY], -85.f, 2000.f, MODE_ABSOLUTE);
            }if(sciTimeCnt == 1500){
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWY], 0.f, 2000.f, MODE_ABSOLUTE);
            }if(ringIndexBack == ringIndexFront){
                scidemo_num = BOW_LAST_MOTION;
                sciTimeCnt = 0;
                printf("\n ========== %f \n",sciTimeCnt*0.005);
            }break;
            //--------------------------------------------------------------------------------------


        case BOW_TWO_W://SCENE #2 - Greeting with arm shake
            if(fileReadFlag == ENABLE){
                LeadTextData();
            }ChaseTextData();
            if(sciTimeCnt == 0){
                FILE_LOG(logSUCCESS) << "Pyeongchang_DEMO: BOW_TWO_W";
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWY], -85.f, 2000.f, MODE_ABSOLUTE);
            }if(sciTimeCnt == 1500){
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWY], 0.f, 2000.f, MODE_ABSOLUTE);
            }if(ringIndexBack == ringIndexFront){
                scidemo_num = BOW_LAST_MOTION;
                sciTimeCnt = 0;
                printf("\n ========== %f \n",sciTimeCnt*0.005);
            }break;
        case BOW_LAST_MOTION:
            if(sciTimeCnt == 1)
                initHeadHand(1);
            else if(sciTimeCnt==201)
            {
                scidemo_num = NO_ACT;
                FILE_LOG(logINFO) << "Greeting Motion End!!!!";
            }
            break;

        case WST_TWIST:
            if(sciTimeCnt == 0){
                FILE_LOG(logSUCCESS) << "Pyeongchang_DEMO: WST_TWIST";
                wbPosWST->setTaskPos(45, 2500, MODE_ABSOLUTE);
            }if(sciTimeCnt == 500){
                scidemo_num = NO_ACT;
                FILE_LOG(logINFO) << "Pyeongchang_DEMO: WST_TWIST " << "END";
            }
            break;
        case WST_ZERO:
            if(sciTimeCnt == 0){
                FILE_LOG(logSUCCESS) << "Pyeongchang_DEMO: WST_ZERO";
                wbPosWST->setTaskPos(0, 2500, MODE_ABSOLUTE);
            }if(sciTimeCnt == 500){
                scidemo_num = NO_ACT;
                FILE_LOG(logINFO) << "Pyeongchang_DEMO: WST_ZERO " << "END";
            }
            break;

        case GUID_LEFT:
            if(sciTimeCnt == 0){
                FILE_LOG(logSUCCESS) << "Pyeongchang_DEMO: GUID_LEFT";
                setMoveFinger(1, 1, 700);
                wbPosWST->setTaskPos(10, 800, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], -15.f, 1000.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], -12.f, 1000.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY], 55.f, 1000.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -65.f, 1000.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWY], -90.f, 1000.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWP], 0.f, 1000.f, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSP], 10.f, 1000.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], 50.f, 1000.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY], 70.f, 1000.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB], -20.f, 1000.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWY], 85.f, 1000.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWP], 0.f, 1000.f, MODE_ABSOLUTE);
            }if(sciTimeCnt == 500){
                wbPosWST->setTaskPos(0, 800, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], 10.f, 1500.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], -10.f, 1500.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY], 0.f, 1500.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -30.f, 1500.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWY], 0.f, 1500.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWP], 0.f, 1500.f, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSP], 10.f, 1500.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], 10.f, 1500.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY], 0.f, 1500.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB], -30.f, 1500.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWY], 0.f, 1500.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWP], 0.f, 1500.f, MODE_ABSOLUTE);
            }if(sciTimeCnt == 850){
                initHeadHand(1);
            }if(sciTimeCnt == 900){
                scidemo_num = GUID_INIT;
                FILE_LOG(logINFO) << "Pyeongchang_DEMO: GUID_LEFT " << "END";
            }break;
        case GUID_RIGHT:
            if(sciTimeCnt == 0){
                FILE_LOG(logSUCCESS) << "Pyeongchang_DEMO: GUID_RIGHT";
                setMoveFinger(0, 1, 700);
                wbPosWST->setTaskPos(-10, 800, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], 10.f, 1000.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], -50.f, 1000.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY], -70.f, 1000.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -20.f, 1000.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWY], -85.f, 1000.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWP], 0.f, 1000.f, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSP], -15.f, 1000.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], 12.f, 1000.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY], -55.f, 1000.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB], -65.f, 1000.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWY], 90.f, 1000.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWP], 0.f, 1000.f, MODE_ABSOLUTE);
            }if(sciTimeCnt == 500){
                wbPosWST->setTaskPos(0, 800, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], 10.f, 1500.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], -10.f, 1500.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY], 0.f, 1500.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -30.f, 1500.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWY], 0.f, 1500.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWP], 0.f, 1500.f, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSP], 10.f, 1500.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], 10.f, 1500.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY], 0.f, 1500.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB], -30.f, 1500.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWY], 0.f, 1500.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWP], 0.f, 1500.f, MODE_ABSOLUTE);
            }if(sciTimeCnt == 850){
                initHeadHand(1);
            }if(sciTimeCnt == 900){
                scidemo_num = GUID_INIT;
                FILE_LOG(logINFO) << "Pyeongchang_DEMO: GUID_RIGHT " << "END";
            }break;
        case GUID_INIT:
            if(sciTimeCnt == 0){
                FILE_LOG(logSUCCESS) << "Pyeongchang_DEMO: GUID_INIT";
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], 10.f, 1500.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], -10.f, 1500.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY], 0.f, 1500.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -30.f, 1500.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWY], 0.f, 1500.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWP], 0.f, 1500.f, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSP], 10.f, 1500.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], 10.f, 1500.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY], 0.f, 1500.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB], -30.f, 1500.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWY], 0.f, 1500.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWP], 0.f, 1500.f, MODE_ABSOLUTE);
            }if(sciTimeCnt == 350){
                initHeadHand(1);
            }if(sciTimeCnt == 500){
                scidemo_num = NO_ACT;
            }break;

        case DEMO_SCENE_1: // Hello~~~ /10
            if(sciTimeCnt == 0){
                FILE_LOG(logSUCCESS) << "Pyeongchang_DEMO: DEMO_SCENE_1";
            }if(sciTimeCnt == 10*1000/RT_TIMER_PERIOD_MS+100){
                printf("\n ==========time : %f \n",sciTimeCnt*0.005);
                printf(">>> DEMO_SCENE_1 end\n");
                scidemo_num = NO_ACT;
            }
            break;

        case DEMO_SCENE_2: // promise /15
            if(sciTimeCnt == 0){

                FILE_LOG(logSUCCESS) << "Pyeongchang_DEMO: DEMO_SCENE_2";
                MotionTime = 12000.f;
                Inital_motion_Time = 1000/RT_TIMER_PERIOD_MS;
                Int_MotionTime = (int)(MotionTime/RT_TIMER_PERIOD_MS);

                setMoveFinger(RF1,1,700);
                setMoveFinger(RF5,1,700);
            }
            if(sciTimeCnt == Inital_motion_Time)
            {
                setMoveFinger(RF1,-1,700);
                setMoveFinger(RF2,-1,700);
                setMoveFinger(RF3,-1,700);
                setMoveFinger(RF4,-1,700);
//                setMoveFinger(RF5,-1,700);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP],  -70.f,   1500.0f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR],   -10.f,  1500.0f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY],   10.f,   1500.0f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -90.f,    1500.0f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWY],   -20.f,  1500.0f, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == Inital_motion_Time + Int_MotionTime*0.25 +100){
//                wbPosWST->setTaskPos(-15, 1000, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == Inital_motion_Time + Int_MotionTime*0.5 +100){
//                wbPosWST->setTaskPos(15, 1000, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == Inital_motion_Time + Int_MotionTime*0.75 +100){
//                wbPosWST->setTaskPos(0, 1000, MODE_ABSOLUTE);
            }

            if(sciTimeCnt == Inital_motion_Time + Int_MotionTime +100)
            {
                    Setmove_walk_ready(1500.0f);
                    setMoveFinger(RF1,1,700);
            }
            if(sciTimeCnt == Inital_motion_Time + Int_MotionTime +100 + 400)
                    initHeadHand(1);//0 head . 1 hand . 2 hand+head
            if(sciTimeCnt == Inital_motion_Time + Int_MotionTime +100 + 400)
            {
                printf("\n ==========time : %f \n",sciTimeCnt*0.005);
                printf(">>> DEMO_SCENE_2 end\n");
                sciMotionSelector = NO_ACT;
            }
            break;

        case DEMO_SCENE_3: // can pic? /10
            if(sciTimeCnt == 0){
                FILE_LOG(logSUCCESS) << "Pyeongchang_DEMO: DEMO_SCENE_3";
            }if(sciTimeCnt == 10*1000/RT_TIMER_PERIOD_MS+100){
                printf("\n ==========time : %f \n",sciTimeCnt*0.005);
                printf(">>> DEMO_SCENE_4 end\n");
                scidemo_num = NO_ACT;
            }
            break;

        case DEMO_SCENE_4: // kimchi /5
            if(sciTimeCnt == 0)
            {
                FILE_LOG(logSUCCESS) << "Pyeongchang_DEMO: DEMO_SCENE_4";
                MotionTime = 10000.f;
                Int_MotionTime = (int)(MotionTime/RT_TIMER_PERIOD_MS);


                setMoveFinger(RF1,1,700);  //2 both 0 right 1 left
                setMoveFinger(RF2,1,700);   //2 both 0 right 1 left
                setMoveFinger(RF3,1,700);  //2 both 0 right 1 left
                setMoveFinger(RF4,-1,700);  //2 both 0 right 1 left
                setMoveFinger(RF5,-1,700);  //2 both 0 right 1 left
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP],  -70.f, 1500.0f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR],   -10.f, 1500.0f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY],   00.f, 1500.0f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -90.f, 1500.0f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWY],   60.f, 1500.0f, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == 120){

                setMoveFinger(RF1,-1,700);  //2 both 0 right 1 left
            }
            if(sciTimeCnt == Int_MotionTime +100)
            {
                Setmove_walk_ready(1500.0f);
//                setMoveFinger(RF1,1,700);  //2 both 0 right 1 left
                setMoveFinger(RF1,1,700);  //2 both 0 right 1 left
//                setMoveFinger(RF2,1,700);   //2 both 0 right 1 left
//                setMoveFinger(RF3,1,700);  //2 both 0 right 1 left
//                setMoveFinger(RF4,1,700);  //2 both 0 right 1 left
//                setMoveFinger(RF5,1,700);  //2 both 0 right 1 left
            }
            if(sciTimeCnt == Int_MotionTime +100 + 200 + 200)
                initHeadHand(1);//0 head . 1 hand . 2 hand+head
            if(sciTimeCnt == Int_MotionTime +100 + 200 + 300)
            {
                sciMotionSelector = NO_ACT;
                printf("\n ==========time : %f \n",sciTimeCnt*0.005);
                printf(">>> DEMO_SCENE_4 end\n");
            }
            break;


            //1-1  2-1  1-2  3-1
        case DANCE_GOGOGO:
        {
            z_101 = 0.05;
            //Pelvis move + Left move =========================================================================================================
            if(sciTimeCnt == 10 && motion_sequence_loop == 0)
            {
                printf(">>> 1-1 Start \n");
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSP], -40.f, 200*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB], -60.f, 200*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], -10.f, 200*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], -40.f, 200*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -60.f, 200*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], 10.f,  200*INT_TIME, MODE_ABSOLUTE);

                setMoveFinger(RF2,-1,400);
                setMoveFinger(RF3,-1,400);
                setMoveFinger(RF4,-1,400);
                setMoveFinger(RF5,-1,400);

                setMoveFinger(LF2,-1,400);
                setMoveFinger(LF3,-1,400);
                setMoveFinger(LF4,-1,400);
                setMoveFinger(LF5,-1,400);
            }
            if(sciTimeCnt == music_offset_2018 + 1 && motion_loop_cnt == 0 && motion_sequence_loop >= 2)
            {
                wbPosWST->setTaskPos(15, 1000, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == music_offset_2018 + 1 && motion_loop_cnt == 2 && motion_sequence_loop >= 2)
            {
                wbPosWST->setTaskPos(-15, 1000, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == music_offset_2018 + 1 && motion_loop_cnt == 4 && motion_sequence_loop >= 2)
            {
                wbPosWST->setTaskPos(0, 800, MODE_ABSOLUTE);
            }


            if(sciTimeCnt == 50 && motion_sequence_loop == 0)
            {
                setMoveFinger(RF1,-1,400);
                setMoveFinger(LF1,-1,400);
            }

            if(sciTimeCnt == bit_int_pc*0 + music_offset_2018)
            {
                printf(">>> 1-1 Test \n");
                motion_time_check = 0;
                if(motion_sequence_loop == 1)
                {
                    wbPosWST->setTaskPos(25, 1500, MODE_ABSOLUTE);
                }
                if(motion_sequence_loop == 2)
                {
//                    wbPosWST->setTaskPos(25, 1500, MODE_ABSOLUTE);
                }
                if(motion_sequence_loop == 3)
                {
//                    wbPosWST->setTaskPos(25, 1500, MODE_ABSOLUTE);
                }

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSP], -95.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB], -95.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], -20.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWP], -25.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], -65.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -10.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], 2.5f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWP], 25.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY], -8.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY], 0.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == bit_int_pc/2 + music_offset_2018 && motion_loop_cnt != 0 && motion_sequence_loop != 0)
            {
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK1], 0.f, bit_int_pc/2*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK2], 0.f, bit_int_pc/2*INT_TIME, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == bit_int_pc*1 + music_offset_2018)
            {
                if(motion_sequence_loop != 0)
                {
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK1], -5.f, bit_int_pc/2*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK2], -5.f, bit_int_pc/2*INT_TIME, MODE_ABSOLUTE);
                }

                wbPosPelZ->setTaskPos(-z_101, bit_int_pc*INT_TIME, MODE_RELATIVE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSP], -65.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB], -10.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], -2.5f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWP], 25.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], -95.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -95.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], 20.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWP], -25.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY], 0.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY], 8.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == bit_int_pc + bit_int_pc/2 + music_offset_2018 && motion_sequence_loop != 0)
            {
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK1], 0.f, bit_int_pc/2*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK2], 0.f, bit_int_pc/2*INT_TIME, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == bit_int_pc*2 + music_offset_2018)
            {
                if(motion_sequence_loop != 0)
                {
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK1], -5.f, bit_int_pc/2*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK2], -5.f, bit_int_pc/2*INT_TIME, MODE_ABSOLUTE);
                }
                wbPosPelZ->setTaskPos(z_101, bit_int_pc*INT_TIME, MODE_RELATIVE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSP], -95.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB], -95.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], -20.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWP], -25.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], -65.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -10.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], 2.5f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWP], 25.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY], -8.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY], 0.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);

                if(++motion_loop_cnt < 6) sciTimeCnt = music_offset_2018; //6

            }
            if(sciTimeCnt == bit_int_pc*2 + bit_int_pc/2 + music_offset_2018 && motion_loop_cnt != 0 && motion_sequence_loop != 0)
            {
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK1], 0.f, bit_int_pc/2*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK2], 0.f, bit_int_pc/2*INT_TIME, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == music_offset_2018 + bit_int_pc*3)
            {
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSP], -80.f, (bit_int_pc)*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB], -80.f, (bit_int_pc)*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], -20.f, (bit_int_pc)*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWP], 0.f, (bit_int_pc)*INT_TIME, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], -80.f, (bit_int_pc)*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -80.f, (bit_int_pc)*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], 20.f, (bit_int_pc)*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWP], 0.f, (bit_int_pc)*INT_TIME, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY], -8.f, (bit_int_pc)*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY], 8.f, (bit_int_pc)*INT_TIME, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWY], -90.f, (bit_int_pc)*2.5f*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWY], 90.f, (bit_int_pc)*2.5f*INT_TIME, MODE_ABSOLUTE);

                bit_int_pc = bit_int_pc/2;

                setMoveFinger(2,1, 400);


            }
            if(sciTimeCnt == music_offset_2018 + bit_int_pc*8+30 && motion_sequence_loop == 0)
            {
                scidemo_num = 2019;//20190
                sciTimeCnt = 0;
                music_offset_2018 = 1;
            }
            if(sciTimeCnt == music_offset_2018 + bit_int_pc*8+40 && motion_sequence_loop !=0)
            {
                scidemo_num = 2019;//20190
                sciTimeCnt = 0;
                music_offset_2018 = 1;
            }
            break;
        }
        case 2019:
        {

            if(sciTimeCnt == bit_int_pc*0 + music_offset_2018)
            {
                printf(">>> 2-1 Start \n");
            }
            if(sciTimeCnt == bit_int_pc*0 + motion_2_head_offest + music_offset_2018)
            {
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK1], -15.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK2], -15.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == bit_int_pc*1 + motion_2_head_offest + music_offset_2018)
            {
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK1], 0.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK2], 0.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == bit_int_pc*2 + motion_2_head_offest + music_offset_2018)
            {
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK1], -15.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK2], -15.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == bit_int_pc*3 + motion_2_head_offest + music_offset_2018)
            {
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK1], 0.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK2], 0.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == bit_int_pc*0 + music_offset_2018)
            {
                wbPosPelZ->setTaskPos(-z_101/2, bit_int_pc*INT_TIME, MODE_RELATIVE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSP], -50.f, (bit_int_pc)*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB], -90.f, (bit_int_pc)*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], -20.f, (bit_int_pc)*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWP], 0.f, (bit_int_pc)*INT_TIME, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], -50.f, (bit_int_pc)*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -90.f, (bit_int_pc)*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], 20.f, (bit_int_pc)*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWP], 0.f, (bit_int_pc)*INT_TIME, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY], -8.f, (bit_int_pc)*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY], 8.f, (bit_int_pc)*INT_TIME, MODE_ABSOLUTE);

            }
            if(sciTimeCnt == bit_int_pc*1 + music_offset_2018)
            {
                wbPosPelZ->setTaskPos(z_101/2, bit_int_pc*INT_TIME, MODE_RELATIVE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSP], -80.f, (bit_int_pc)*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB], -80.f, (bit_int_pc)*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], -20.f, (bit_int_pc)*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWP], -0.f, (bit_int_pc)*INT_TIME, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], -80.f, (bit_int_pc)*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -80.f, (bit_int_pc)*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], 20.f, (bit_int_pc )*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWP], -0.f, (bit_int_pc)*INT_TIME, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY], -8.f, (bit_int_pc)*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY], 8.f, (bit_int_pc )*INT_TIME, MODE_ABSOLUTE);

            }
            if(sciTimeCnt == bit_int_pc*2 + music_offset_2018)
            {
                wbPosPelZ->setTaskPos(-z_101/2, bit_int_pc*INT_TIME, MODE_RELATIVE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSP], -50.f, (bit_int_pc)*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB], -90.f, (bit_int_pc)*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], -20.f, (bit_int_pc)*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWP], -0.f, (bit_int_pc)*INT_TIME, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], -50.f, (bit_int_pc)*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -90.f, (bit_int_pc)*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], 20.f, (bit_int_pc)*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWP], -0.f, (bit_int_pc)*INT_TIME, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY], -8.f, (bit_int_pc)*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY], 8.f, (bit_int_pc)*INT_TIME, MODE_ABSOLUTE);

            }
            if(sciTimeCnt == bit_int_pc*3 + music_offset_2018)
            {
//                setMoveFinger(2,-1, 400);

                setMoveFinger(RF2,-1,400);
                setMoveFinger(RF3,-1,400);
                setMoveFinger(RF4,-1,400);
                setMoveFinger(RF5,-1,400);

                setMoveFinger(LF2,-1,400);
                setMoveFinger(LF3,-1,400);
                setMoveFinger(LF4,-1,400);
                setMoveFinger(LF5,-1,400);
            }
            if(sciTimeCnt == bit_int_pc*3 + music_offset_2018)
            {
                wbPosPelZ->setTaskPos(z_101/2, bit_int_pc*INT_TIME, MODE_RELATIVE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSP], -80.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB], -80.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], -20.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWP], 0.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], -80.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -80.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], 20.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWP], 0.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY], -8.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY], 8.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWY], -0.f, (bit_int_pc*5.f)*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWY], 0.f, (bit_int_pc*5.f)*INT_TIME, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == music_offset_2018 + bit_int_pc*4)
            {
                setMoveFinger(RF1,-1,400);
                setMoveFinger(LF1,-1,400);
                scidemo_num = 2020;
                sciTimeCnt = 0;
                music_offset_2018 = 1;
                bit_int_pc = bit_int_pc*2;
                motion_loop_cnt = 0;
            }
            break;
            }
        case 2020:
            {
                if(sciTimeCnt == music_offset_2018 + 1 && motion_loop_cnt == 0 && motion_sequence_loop >=2)
                {
                    wbPosWST->setTaskPos(15, 1000, MODE_ABSOLUTE);
                }
                if(sciTimeCnt == music_offset_2018 + 1 && motion_loop_cnt == 2 && motion_sequence_loop >=2)
                {
                    wbPosWST->setTaskPos(-15, 1200, MODE_ABSOLUTE);
                }
                if(sciTimeCnt == music_offset_2018 + 1 && motion_loop_cnt == 4 && motion_sequence_loop >=2)
                {
                    wbPosWST->setTaskPos(0, 800, MODE_ABSOLUTE);
                }
                if(sciTimeCnt == bit_int_pc*0 + music_offset_2018)
                {
                    printf(">>> 1-2 Start \n");
                    if(motion_sequence_loop == 1)
                    {
                        wbPosWST->setTaskPos(-25, 1500, MODE_ABSOLUTE);
                    }
                    if(motion_sequence_loop == 2)
                    {
//                        wbPosWST->setTaskPos(-25, 1500, MODE_ABSOLUTE);
                    }
                    if(motion_sequence_loop == 3)
                    {
//                        wbPosWST->setTaskPos(-25, 1500, MODE_ABSOLUTE);
                    }

                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSP], -65.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB], -10.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], -2.5f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWP], 25.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);

                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], -95.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -95.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], 20.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWP], -25.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);

                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY], 0.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY], 8.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                }
                if(sciTimeCnt == bit_int_pc/2 + music_offset_2018 && motion_loop_cnt != 0 && motion_sequence_loop != 0)
                {
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK1], 0.f, bit_int_pc/2*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK2], 0.f, bit_int_pc/2*INT_TIME, MODE_ABSOLUTE);
                }
                if(sciTimeCnt == bit_int_pc*1 + music_offset_2018)
                {
                    if(motion_sequence_loop != 0)
                    {
                        wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK1], -5.f, bit_int_pc/2*INT_TIME, MODE_ABSOLUTE);
                        wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK2], -5.f, bit_int_pc/2*INT_TIME, MODE_ABSOLUTE);
                    }

                    wbPosPelZ->setTaskPos(-z_101, bit_int_pc*INT_TIME, MODE_RELATIVE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSP], -95.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB], -95.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], -20.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWP], -25.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);

                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], -65.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -10.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], 2.5f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWP], 25.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);

                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY], -8.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY], 0.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                }
                if(sciTimeCnt == bit_int_pc + bit_int_pc/2 + music_offset_2018 && motion_sequence_loop != 0)
                {
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK1], 0.f, bit_int_pc/2*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK2], 0.f, bit_int_pc/2*INT_TIME, MODE_ABSOLUTE);
                }
                if(sciTimeCnt == bit_int_pc*2 + music_offset_2018 && motion_loop_cnt < 4)
                {
                    if(motion_sequence_loop != 0)
                    {
                        wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK1], -5.f, bit_int_pc/2*INT_TIME, MODE_ABSOLUTE);
                        wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK2], -5.f, bit_int_pc/2*INT_TIME, MODE_ABSOLUTE);
                    }
                    wbPosPelZ->setTaskPos(z_101, bit_int_pc*INT_TIME, MODE_RELATIVE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSP], -65.f,    (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB], -10.f,    (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], -2.5f,    (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWP], 25.f,     (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);

                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], -95.f,    (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -95.f,   (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], 20.f,     (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWP], -25.f,    (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);

                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY], 0.f,      (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY], 8.f,     (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);

                    if(++motion_loop_cnt < 5) sciTimeCnt = music_offset_2018; //6

                }
                if(sciTimeCnt == bit_int_pc*2 + bit_int_pc/2 + music_offset_2018)
                {
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK1], 0.f, bit_int_pc/2*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK2], 0.f, bit_int_pc/2*INT_TIME, MODE_ABSOLUTE);
                }
                if(sciTimeCnt == music_offset_2018 + bit_int_pc*2  && motion_loop_cnt == 4)
                {
                    wbPosPelZ->setTaskPos(z_101, bit_int_pc*INT_TIME, MODE_RELATIVE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSP], -85.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB], -60.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], 20.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWP], 0.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);

                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], -95.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -95.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], 10.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWP], 0.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);

                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY], -60.f, bit_int_pc*3.0f*INT_TIME, MODE_ABSOLUTE);
    //                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY], 8.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);

                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWY], 0.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
    //                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWY], 0.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);

                    bit_int_pc = bit_int_pc/2;
                    bit_int_pc_offset = bit_int_pc_offset/4;
                    //setMoveFinger(0,1,bit_int_pc*INT_TIME);
                    //setMoveFinger(1,1,bit_int_pc*INT_TIME);

                }
                if(sciTimeCnt == music_offset_2018 + bit_int_pc*6+89) //45
                {
                    scidemo_num = 2021;
                    sciTimeCnt = 0;
                    music_offset_2018 = 1;
                }

                break;
            }
        case 2021:
        {
            if(sciTimeCnt == music_offset_2018 + bit_int_pc*0)
            {
                printf(">>> 3-1 Start \n");
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSP], -50.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB], -95.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], 10.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWP], 0.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], -120.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -55.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], 10.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWP], 0.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY], -40.f, (bit_int_pc-bit_int_pc_offset)*2.0f*INT_TIME, MODE_ABSOLUTE);
//                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY], 8.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWY], 0.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
//                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWY], 0.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);


            }
            if(sciTimeCnt == music_offset_2018 + bit_int_pc*1)
            {
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSP], -85.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB], -60.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], 20.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWP], 0.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], -95.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -95.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], 10.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWP], 0.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY], -60.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
//                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY], 8.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWY], 0.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
//                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWY], 0.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);


            }
            if(sciTimeCnt == music_offset_2018 + bit_int_pc*2)
            {
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSP], -50.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB], -95.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], 10.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWP], 0.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], -120.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -55.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], 10.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWP], 0.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY], -40.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
//                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY], 8.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWY], 0.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
//                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWY], 0.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);


            }

            if(sciTimeCnt == music_offset_2018 + bit_int_pc*3)
            {
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSP], -85.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB], -60.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], 20.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWP], 0.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], -95.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -95.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], 10.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWP], 0.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY], -60.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
//                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY], 8.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWY], 0.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
//                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWY], 0.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);


            }
            if(sciTimeCnt == music_offset_2018 + bit_int_pc*4)
            {
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSP], -50.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB], -95.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], 10.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWP], 0.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], -120.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -55.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], 10.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWP], 0.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY], -40.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
//                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY], 8.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWY], 0.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
//                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWY], 0.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);


            }

            if(sciTimeCnt == music_offset_2018 + bit_int_pc*5+89+28) // 30
            {
                printf("Motion Time :  %d \n", motion_time_check);
                if(++motion_sequence_loop < 4){
                    scidemo_num = DANCE_GOGOGO;
                }
                else
                {
                            scidemo_num = 2034;
                }
                sciTimeCnt = 0;
                music_offset_2018 = 1;
                bit_int_pc = bit_int_pc*2;
                bit_int_pc_offset =bit_int_pc_offset*4;
                motion_loop_cnt = 0;
            }

            break;
        }
        case 2034:
        {
            if(sciTimeCnt == music_offset_2018 + bit_int_pc*2+200)
            {
                setMoveFinger(RF1, 1,400);
                setMoveFinger(RF2, 1,400);
                setMoveFinger(RF3, 1,400);
                setMoveFinger(RF4, 1,400);
                setMoveFinger(RF5, 1,400);
                setMoveFinger(LF1, 1,400);
                setMoveFinger(LF2, 1,400);
                setMoveFinger(LF3, 1,400);
                setMoveFinger(LF4, 1,400);
                setMoveFinger(LF5, 1,400);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP],  10.f, t1_101*2, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], -10.f, t1_101*2, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY],   0.f, t1_101*2, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -30.f, t1_101*2, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWY],   0.f, t1_101*2, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSP],  10.f, t1_101*2, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR],  10.f, t1_101*2, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY],   0.f, t1_101*2, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB], -30.f, t1_101*2, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWY],   0.f, t1_101*2, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NKY],   0.f, t1_101, MODE_ABSOLUTE);
                wbPosWST->setTaskPos(0.f, t1_101, MODE_ABSOLUTE);

            }


            if(sciTimeCnt == music_offset_2018 + bit_int_pc*2+1000)
            {
                initHeadHand(1);
            }
            if(sciTimeCnt == music_offset_2018 + bit_int_pc*2+1200)
            {
                printf(">>> Dance Motion end \n");
                scidemo_num = 2035;
                sciTimeCnt = 0;
                bit_int_pc = bit_int_pc*2;
                bit_int_pc_offset =bit_int_pc_offset*4;
                music_offset_2018 = 1;
                motion_loop_cnt = 0;
                sciMotionSelector = NO_ACT;
            }
            break;
        }
//1-3  2-2  1-4  3-2
        case 2022:
            {

                if(sciTimeCnt == bit_int_pc*0 + music_offset_2018)
                {
                    printf(">>> 1-3 Start \n");
                    wbPosWST->setTaskPos(25, 1500, MODE_ABSOLUTE);
                    motion_time_check = 0;
//                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK1], -5.f, bit_int_pc/2*INT_TIME, MODE_ABSOLUTE);
//                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK2], -5.f, bit_int_pc/2*INT_TIME, MODE_ABSOLUTE);

                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSP], -65.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB], -10.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], -2.5f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWP], 25.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);

                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], -95.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -95.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], 20.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWP], -25.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);

                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY], 0.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY], 8.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);


                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWY], 0.f, bit_int_pc*2.5f*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWY], 0.f, bit_int_pc*2.5f*INT_TIME, MODE_ABSOLUTE);

                }
                if(sciTimeCnt == bit_int_pc/2 + music_offset_2018 && motion_loop_cnt != 0)
                {
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK1], 0.f, bit_int_pc/2*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK2], 0.f, bit_int_pc/2*INT_TIME, MODE_ABSOLUTE);
                }
                if(sciTimeCnt == bit_int_pc*1 + music_offset_2018)
                {
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK1], -5.f, bit_int_pc/2*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK2], -5.f, bit_int_pc/2*INT_TIME, MODE_ABSOLUTE);

                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSP], -95.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB], -95.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], -20.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWP], -25.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);


                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], -65.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -10.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], 2.5f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWP], 25.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);

                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY], -8.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY], 0.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);

                }
                if(sciTimeCnt == bit_int_pc + bit_int_pc/2 + music_offset_2018)
                {
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK1], 0.f, bit_int_pc/2*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK2], 0.f, bit_int_pc/2*INT_TIME, MODE_ABSOLUTE);
                }
                if(sciTimeCnt == bit_int_pc*2 + music_offset_2018)
                {
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK1], -5.f, bit_int_pc/2*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK2], -5.f, bit_int_pc/2*INT_TIME, MODE_ABSOLUTE);

                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSP], -65.f,    (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB], -10.f,    (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], -2.5f,    (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWP], 25.f,     (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);

                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], -95.f,    (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -95.f,   (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], 20.f,     (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWP], -25.f,    (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);

                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY], 0.f,      (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY], 8.f,     (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);


                    if(++motion_loop_cnt < 6) sciTimeCnt = music_offset_2018; //6

                }
                if(sciTimeCnt == bit_int_pc*2 + bit_int_pc/2 + music_offset_2018)
                {
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK1], 0.f, bit_int_pc/2*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK2], 0.f, bit_int_pc/2*INT_TIME, MODE_ABSOLUTE);
                }
                if(sciTimeCnt == music_offset_2018 + bit_int_pc*3)
                {
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSP], -80.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB], -80.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], -20.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWP], 0.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);

                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], -80.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -80.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], 20.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWP], 0.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);

                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY], -8.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY], 8.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);

                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWY], -90.f, bit_int_pc*2.5f*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWY], 90.f, bit_int_pc*2.5f*INT_TIME, MODE_ABSOLUTE);

                    bit_int_pc = bit_int_pc/2;

                    setMoveFinger(2,1, 400);


                }
                if(sciTimeCnt == music_offset_2018 + bit_int_pc*6+30) //8
                {
                    scidemo_num = 2023;
                    sciTimeCnt = 0;
                    music_offset_2018 = 1;
                }

                break;
            }
        case 2023:
        {

            if(sciTimeCnt == bit_int_pc*0 + music_offset_2018)
            {
                printf(">>> 2-2 Start \n");

            }
            if(sciTimeCnt == bit_int_pc*0 + motion_2_head_offest + music_offset_2018)
            {
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK1], -15.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK2], -15.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == bit_int_pc*1 + motion_2_head_offest + music_offset_2018)
            {
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK1], 0.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK2], 0.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == bit_int_pc*2 + motion_2_head_offest + music_offset_2018)
            {
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK1], -15.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK2], -15.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == bit_int_pc*3 + motion_2_head_offest + music_offset_2018)
            {
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK1], 0.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK2], 0.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == bit_int_pc*0 + music_offset_2018)
            {
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSP], -50.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB], -90.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], -20.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWP], 0.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], -50.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -90.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], 20.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWP], 0.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY], -8.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY], 8.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);

            }
            if(sciTimeCnt == bit_int_pc*1 + music_offset_2018)
            {
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSP], -80.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB], -80.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], -20.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWP], -0.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], -80.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -80.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], 20.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWP], -0.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY], -8.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY], 8.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);

            }

            if(sciTimeCnt == bit_int_pc*2 + music_offset_2018)
            {
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSP], -50.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB], -90.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], -20.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWP], -0.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], -50.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -90.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], 20.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWP], -0.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY], -8.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY], 8.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);



            }
            if(sciTimeCnt == bit_int_pc*3 + music_offset_2018)
            {
                setMoveFinger(RF2,-1,400);
                setMoveFinger(RF3,-1,400);
                setMoveFinger(RF4,-1,400);
                setMoveFinger(RF5,-1,400);

                setMoveFinger(LF2,-1,400);
                setMoveFinger(LF3,-1,400);
                setMoveFinger(LF4,-1,400);
                setMoveFinger(LF5,-1,400);
            }

            if(sciTimeCnt == bit_int_pc*3 + music_offset_2018)
            {
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSP], -80.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB], -80.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], -20.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWP], 0.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], -80.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -80.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], 20.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWP], 0.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY], -8.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY], 8.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWY], -0.f, (bit_int_pc*5.f)*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWY], 0.f, (bit_int_pc*5.f)*INT_TIME, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == music_offset_2018 + bit_int_pc*4)
            {
                setMoveFinger(RF1,-1,400);
                setMoveFinger(LF1,-1,400);
                scidemo_num = 2024;
                sciTimeCnt = 0;
                music_offset_2018 = 1;
                bit_int_pc = bit_int_pc*2;
                motion_loop_cnt = 0;
            }
            break;
            }
        case 2024:
            {
                if(sciTimeCnt == bit_int_pc*0 + music_offset_2018)
                {
                    printf(">>> 1-4 Start \n");
                    wbPosWST->setTaskPos(-25, 2000, MODE_ABSOLUTE);
//                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK1], -5.f, bit_int_pc/2*INT_TIME, MODE_ABSOLUTE);
//                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK2], -5.f, bit_int_pc/2*INT_TIME, MODE_ABSOLUTE);

                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSP], -65.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB], -10.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], -2.5f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWP], 25.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);

                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], -95.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -95.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], 20.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWP], -25.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);

                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY], 0.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY], 8.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);



                }

                if(sciTimeCnt == bit_int_pc/2 + music_offset_2018)
                {
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK1], 0.f, bit_int_pc/2*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK2], 0.f, bit_int_pc/2*INT_TIME, MODE_ABSOLUTE);
                }
                if(sciTimeCnt == bit_int_pc*1 + music_offset_2018)
                {
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK1], -5.f, bit_int_pc/2*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK2], -5.f, bit_int_pc/2*INT_TIME, MODE_ABSOLUTE);

                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSP], -95.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB], -95.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], -20.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWP], -25.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);


                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], -65.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -10.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], 2.5f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWP], 25.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);

                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY], -8.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY], 0.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);

                }
                if(sciTimeCnt == bit_int_pc + bit_int_pc/2 + music_offset_2018)
                {
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK1], 0.f, bit_int_pc/2*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK2], 0.f, bit_int_pc/2*INT_TIME, MODE_ABSOLUTE);
                }
                if(sciTimeCnt == bit_int_pc*2 + music_offset_2018 && motion_loop_cnt < 4)
                {
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK1], -5.f, bit_int_pc/2*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK2], -5.f, bit_int_pc/2*INT_TIME, MODE_ABSOLUTE);

                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSP], -65.f,    (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB], -10.f,    (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], -2.5f,    (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWP], 25.f,     (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);

                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], -95.f,    (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -95.f,   (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], 20.f,     (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWP], -25.f,    (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);

                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY], 0.f,      (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY], 8.f,     (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);

                    if(++motion_loop_cnt < 5) sciTimeCnt = music_offset_2018; //6

                }
                if(sciTimeCnt == bit_int_pc*2 + bit_int_pc/2 + music_offset_2018)
                {
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK1], 0.f, bit_int_pc/2*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK2], 0.f, bit_int_pc/2*INT_TIME, MODE_ABSOLUTE);
                }
                if(sciTimeCnt == music_offset_2018 + bit_int_pc*2  && motion_loop_cnt == 4)
                {
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSP], -85.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB], -60.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], 20.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWP], 0.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);

                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], -95.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -95.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], 10.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWP], 0.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);

                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY], -60.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
    //                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY], 8.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);

                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWY], 0.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
    //                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWY], 0.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);

                    bit_int_pc = bit_int_pc/2;
                    bit_int_pc_offset = bit_int_pc_offset/4;

                }
                if(sciTimeCnt == music_offset_2018 + bit_int_pc*6+89) //60
                {
                    scidemo_num = 2025;
                    sciTimeCnt = 0;
                    music_offset_2018 = 1;
                }

                break;
            }
        case 2025:
        {
            if(sciTimeCnt == music_offset_2018 + bit_int_pc*0)
            {
                printf(">>> 3-2 Start \n");

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSP], -50.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB], -95.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], 10.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWP], 0.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], -120.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -55.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], 10.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWP], 0.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY], -40.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
//                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY], 8.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWY], 0.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
//                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWY], 0.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);


            }
            if(sciTimeCnt == music_offset_2018 + bit_int_pc*1)
            {
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSP], -85.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB], -60.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], 20.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWP], 0.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], -95.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -95.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], 10.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWP], 0.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY], -60.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
//                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY], 8.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWY], 0.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
//                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWY], 0.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == music_offset_2018 + bit_int_pc*2)
            {
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSP], -50.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB], -95.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], 10.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWP], 0.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], -120.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -55.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], 10.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWP], 0.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY], -40.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
//                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY], 8.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWY], 0.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
//                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWY], 0.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);

            }
            if(sciTimeCnt == music_offset_2018 + bit_int_pc*3)
            {
                wbPosPelZ->setTaskPos(-z_101, 2*bit_int_pc*INT_TIME, MODE_RELATIVE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSP], -85.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB], -60.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], 20.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWP], 0.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], -95.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -95.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], 10.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWP], 0.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY], -60.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
//                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY], 8.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWY], 0.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
//                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWY], 0.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == music_offset_2018 + bit_int_pc*4)
            {
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSP], -50.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB], -95.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], 10.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWP], 0.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], -120.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -55.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], 10.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWP], 0.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY], -40.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
//                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY], 8.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWY], 0.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
//                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWY], 0.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
            }

            if(sciTimeCnt == music_offset_2018 + bit_int_pc*5+89+28)//30
            {
                printf("Motion Time :  %d \n", motion_time_check);
                scidemo_num = 2026;
                sciTimeCnt = 0;
                music_offset_2018 = 1;
                bit_int_pc = bit_int_pc*2;
                bit_int_pc_offset =bit_int_pc_offset*4;
                motion_loop_cnt = 0;
            }

            break;
        }
//1-5  2-3  1-6  3-3
        case 2026:
            {
                if(sciTimeCnt == music_offset_2018 + 1 && motion_loop_cnt == 0)
                {
                    wbPosWST->setTaskPos(25, 2200, MODE_ABSOLUTE);
                }
                if(sciTimeCnt == music_offset_2018 + 1 && motion_loop_cnt == 3)
                {
                    wbPosWST->setTaskPos(-25, 2200, MODE_ABSOLUTE);
                }
                if(sciTimeCnt == music_offset_2018 + 1 && motion_loop_cnt == 6)
                {
                    wbPosWST->setTaskPos(0, 1200, MODE_ABSOLUTE);
                }
                if(sciTimeCnt == bit_int_pc*0 + music_offset_2018)
                {
                    printf(">>> 1-5 Start \n");

//                    wbPosWST->setTaskPos(0, 1500, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK1], -5.f, bit_int_pc/2*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK2], -5.f, bit_int_pc/2*INT_TIME, MODE_ABSOLUTE);

                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSP], -65.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB], -10.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], -2.5f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWP], 25.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);

                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], -95.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -95.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], 20.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWP], -25.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);

                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY], 0.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY], 8.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);


                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWY], 0.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWY], 0.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);

                }

                if(sciTimeCnt == bit_int_pc/2 + music_offset_2018 && motion_loop_cnt != 0)
                {
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK1], 0.f, bit_int_pc/2*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK2], 0.f, bit_int_pc/2*INT_TIME, MODE_ABSOLUTE);
                }
                if(sciTimeCnt == bit_int_pc*1 + music_offset_2018)
                {
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK1], -5.f, bit_int_pc/2*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK2], -5.f, bit_int_pc/2*INT_TIME, MODE_ABSOLUTE);

                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSP], -95.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB], -95.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], -20.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWP], -25.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);


                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], -65.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -10.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], 2.5f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWP], 25.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);

                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY], -8.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY], 0.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);

                }
                if(sciTimeCnt == bit_int_pc + bit_int_pc/2 + music_offset_2018)
                {
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK1], 0.f, bit_int_pc/2*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK2], 0.f, bit_int_pc/2*INT_TIME, MODE_ABSOLUTE);
                }
                if(sciTimeCnt == bit_int_pc*2 + music_offset_2018 && motion_loop_cnt < 6)
                {
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK1], -5.f, bit_int_pc/2*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK2], -5.f, bit_int_pc/2*INT_TIME, MODE_ABSOLUTE);

                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSP], -65.f,    (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB], -10.f,    (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], -2.5f,    (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWP], 25.f,     (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);

                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], -95.f,    (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -95.f,   (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], 20.f,     (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWP], -25.f,    (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);

                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY], 0.f,      (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY], 8.f,     (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);


                    if(++motion_loop_cnt < 7) sciTimeCnt = music_offset_2018; //6

                }
                if(sciTimeCnt == bit_int_pc*2 + bit_int_pc/2 + music_offset_2018)
                {
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK1], 0.f, bit_int_pc/2*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK2], 0.f, bit_int_pc/2*INT_TIME, MODE_ABSOLUTE);
                }
                if(sciTimeCnt == music_offset_2018 + bit_int_pc*2 && motion_loop_cnt == 6)
                {
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSP], -80.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB], -80.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], -20.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWP], 0.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);

                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], -80.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -80.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], 20.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWP], 0.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);

                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY], -8.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY], 8.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);

                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWY], -90.f, bit_int_pc*2.5f*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWY], 90.f, bit_int_pc*2.5f*INT_TIME, MODE_ABSOLUTE);

                    bit_int_pc = bit_int_pc/2;

                    setMoveFinger(2,1, 400);


                }
                if(sciTimeCnt == music_offset_2018 + bit_int_pc*6) //8
                {
                    scidemo_num = 2027;
                    sciTimeCnt = 0;
                    music_offset_2018 = 1;
                }

                break;
            }
        case 2027:
        {

            if(sciTimeCnt == bit_int_pc*0 + music_offset_2018)
            {
                printf(">>> 2-3 Start \n");
            }

            if(sciTimeCnt == bit_int_pc*0 + motion_2_head_offest + music_offset_2018)
            {
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK1], -15.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK2], -15.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == bit_int_pc*1 + motion_2_head_offest + music_offset_2018)
            {
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK1], 0.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK2], 0.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == bit_int_pc*2 + motion_2_head_offest + music_offset_2018)
            {
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK1], -15.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK2], -15.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == bit_int_pc*3 + motion_2_head_offest + music_offset_2018)
            {
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK1], 0.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK2], 0.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == bit_int_pc*0 + music_offset_2018)
            {
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSP], -50.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB], -90.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], -20.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWP], 0.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], -50.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -90.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], 20.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWP], 0.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY], -8.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY], 8.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);


            }

            if(sciTimeCnt == bit_int_pc*1 + music_offset_2018)
            {
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSP], -80.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB], -80.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], -20.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWP], -0.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], -80.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -80.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], 20.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWP], -0.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY], -8.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY], 8.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);


            }


            if(sciTimeCnt == bit_int_pc*2 + music_offset_2018)
            {
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSP], -50.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB], -90.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], -20.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWP], -0.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], -50.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -90.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], 20.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWP], -0.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY], -8.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY], 8.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);



            }
            if(sciTimeCnt == bit_int_pc*3 + music_offset_2018)
            {
                setMoveFinger(RF2,-1,400);
                setMoveFinger(RF3,-1,400);
                setMoveFinger(RF4,-1,400);
                setMoveFinger(RF5,-1,400);

                setMoveFinger(LF2,-1,400);
                setMoveFinger(LF3,-1,400);
                setMoveFinger(LF4,-1,400);
                setMoveFinger(LF5,-1,400);
            }

            if(sciTimeCnt == bit_int_pc*3 + music_offset_2018)
            {
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSP], -80.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB], -80.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], -20.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWP], 0.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], -80.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -80.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], 20.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWP], 0.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY], -8.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY], 8.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWY], -0.f, bit_int_pc*5.f*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWY], 0.f, bit_int_pc*5.f*INT_TIME, MODE_ABSOLUTE);

            }




            if(sciTimeCnt == music_offset_2018 + bit_int_pc*4)
            {
                setMoveFinger(RF1,-1,400);
                setMoveFinger(LF1,-1,400);
                scidemo_num = 2028;
                sciTimeCnt = 0;
                music_offset_2018 = 1;
                bit_int_pc = bit_int_pc*2;
                motion_loop_cnt = 0;
            }

            break;
            }
        case 2028:
            {


                if(sciTimeCnt == music_offset_2018 + 1 && motion_loop_cnt == 0)
                {
                    wbPosWST->setTaskPos(15, 1000, MODE_ABSOLUTE);
                }
                if(sciTimeCnt == music_offset_2018 + 1 && motion_loop_cnt == 2)
                {
                    wbPosWST->setTaskPos(-15, 1000, MODE_ABSOLUTE);
                }
                if(sciTimeCnt == music_offset_2018 + 1 && motion_loop_cnt == 4)
                {
                    wbPosWST->setTaskPos(0, 800, MODE_ABSOLUTE);
                }

                if(sciTimeCnt == bit_int_pc*0 + music_offset_2018)
                {
                    printf(">>> 1-6 Start \n");
//                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK1], -5.f, bit_int_pc/2*INT_TIME, MODE_ABSOLUTE);
//                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK2], -5.f, bit_int_pc/2*INT_TIME, MODE_ABSOLUTE);

                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSP], -65.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB], -10.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], -2.5f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWP], 25.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);

                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], -95.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -95.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], 20.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWP], -25.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);

                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY], 0.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY], 8.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);

                }

                if(sciTimeCnt == bit_int_pc/2 + music_offset_2018)
                {
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK1], 0.f, bit_int_pc/2*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK2], 0.f, bit_int_pc/2*INT_TIME, MODE_ABSOLUTE);
                }
                if(sciTimeCnt == bit_int_pc*1 + music_offset_2018)
                {
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK1], -5.f, bit_int_pc/2*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK2], -5.f, bit_int_pc/2*INT_TIME, MODE_ABSOLUTE);

                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSP], -95.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB], -95.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], -20.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWP], -25.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);

                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], -65.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -10.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], 2.5f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWP], 25.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);

                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY], -8.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY], 0.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);


                }
                if(sciTimeCnt == bit_int_pc + bit_int_pc/2 + music_offset_2018)
                {
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK1], 0.f, bit_int_pc/2*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK2], 0.f, bit_int_pc/2*INT_TIME, MODE_ABSOLUTE);
                }
                if(sciTimeCnt == bit_int_pc*2 + music_offset_2018 && motion_loop_cnt < 4)
                {
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK1], -5.f, bit_int_pc/2*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK2], -5.f, bit_int_pc/2*INT_TIME, MODE_ABSOLUTE);

                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSP], -65.f,    (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB], -10.f,    (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], -2.5f,    (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWP], 25.f,     (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);

                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], -95.f,    (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -95.f,   (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], 20.f,     (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWP], -25.f,    (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);

                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY], 0.f,      (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY], 8.f,     (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);

                    if(++motion_loop_cnt < 5) sciTimeCnt = music_offset_2018; //6

                }
                if(sciTimeCnt == music_offset_2018 + bit_int_pc*2  && motion_loop_cnt == 4)
                {
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSP], -85.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB], -60.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], 20.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWP], 0.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);

                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], -95.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -95.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], 10.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWP], 0.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);

                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY], -60.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
    //                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY], 8.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);

                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWY], 0.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
    //                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWY], 0.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);

                    bit_int_pc = bit_int_pc/2;
                    bit_int_pc_offset = bit_int_pc_offset/4;

                }
                if(sciTimeCnt == bit_int_pc*2 + bit_int_pc/2 + music_offset_2018)
                {
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK1], 0.f, bit_int_pc/2*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK2], 0.f, bit_int_pc/2*INT_TIME, MODE_ABSOLUTE);
                }
                if(sciTimeCnt == music_offset_2018 + bit_int_pc*6+0) //60
                {
                    scidemo_num = 2029;
                    sciTimeCnt = 0;
                    music_offset_2018 = 1;
                }

                break;
            }
        case 2029:
        {
            if(sciTimeCnt == music_offset_2018 + bit_int_pc*0)
            {
                printf(">>> 3-3 Start \n");
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSP], -50.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB], -95.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], 10.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWP], 0.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], -120.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -55.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], 10.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWP], 0.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY], -40.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
//                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY], 8.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWY], 0.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
//                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWY], 0.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == music_offset_2018 + bit_int_pc*1)
            {
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSP], -85.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB], -60.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], 20.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWP], 0.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], -95.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -95.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], 10.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWP], 0.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY], -60.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
//                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY], 8.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWY], 0.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
//                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWY], 0.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == music_offset_2018 + bit_int_pc*2)
            {
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSP], -50.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB], -95.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], 10.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWP], 0.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], -120.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -55.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], 10.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWP], 0.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY], -40.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
//                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY], 8.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWY], 0.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
//                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWY], 0.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
            }


            if(sciTimeCnt == music_offset_2018 + bit_int_pc*3)
            {
                wbPosPelZ->setTaskPos(-z_101, 2*bit_int_pc*INT_TIME, MODE_RELATIVE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSP], -85.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB], -60.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], 20.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWP], 0.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], -95.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -95.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], 10.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWP], 0.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY], -60.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
//                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY], 8.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWY], 0.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
//                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWY], 0.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == music_offset_2018 + bit_int_pc*4)
            {
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSP], -50.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB], -95.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], 10.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWP], 0.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], -120.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -55.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], 10.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWP], 0.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY], -40.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
//                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY], 8.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWY], 0.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
//                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWY], 0.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == music_offset_2018 + bit_int_pc*5+0) //30
            {
                scidemo_num = 2030;
                sciTimeCnt = 0;
                music_offset_2018 = 1;
                bit_int_pc = bit_int_pc*2;
                bit_int_pc_offset =bit_int_pc_offset*4;
                motion_loop_cnt = 0;
            }

            break;
        }
//1-7  2-4  1-8  3-4
        case 2030:
            {

            if(sciTimeCnt == music_offset_2018 + 1 && motion_loop_cnt == 0)
            {
                wbPosWST->setTaskPos(25, 2200, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == music_offset_2018 + 1 && motion_loop_cnt == 3)
            {
                wbPosWST->setTaskPos(-25, 2200, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == music_offset_2018 + 1 && motion_loop_cnt == 6)
            {
                wbPosWST->setTaskPos(0, 1200, MODE_ABSOLUTE);
            }
                if(sciTimeCnt == bit_int_pc*0 + music_offset_2018)
                {
                    printf(">>> 1-7 Start \n");
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK1], -5.f, bit_int_pc/2*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK2], -5.f, bit_int_pc/2*INT_TIME, MODE_ABSOLUTE);

                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSP], -65.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB], -10.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], -2.5f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWP], 25.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);

                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], -95.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -95.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], 20.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWP], -25.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);

                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY], 0.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY], 8.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);


                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWY], 0.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWY], 0.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);

                }
                if(sciTimeCnt == bit_int_pc/2 + music_offset_2018 && motion_loop_cnt != 0)
                {
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK1], 0.f, bit_int_pc/2*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK2], 0.f, bit_int_pc/2*INT_TIME, MODE_ABSOLUTE);
                }
                if(sciTimeCnt == bit_int_pc*1 + music_offset_2018)
                {
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK1], -5.f, bit_int_pc/2*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK2], -5.f, bit_int_pc/2*INT_TIME, MODE_ABSOLUTE);

                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSP], -95.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB], -95.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], -20.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWP], -25.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);


                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], -65.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -10.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], 2.5f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWP], 25.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);

                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY], -8.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY], 0.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);


                }
                if(sciTimeCnt == bit_int_pc + bit_int_pc/2 + music_offset_2018)
                {
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK1], 0.f, bit_int_pc/2*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK2], 0.f, bit_int_pc/2*INT_TIME, MODE_ABSOLUTE);
                }
                if(sciTimeCnt == bit_int_pc*2 + music_offset_2018 && motion_loop_cnt < 6)
                {
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK1], -5.f, bit_int_pc/2*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK2], -5.f, bit_int_pc/2*INT_TIME, MODE_ABSOLUTE);

                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSP], -65.f,    (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB], -10.f,    (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], -2.5f,    (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWP], 25.f,     (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);

                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], -95.f,    (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -95.f,   (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], 20.f,     (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWP], -25.f,    (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);

                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY], 0.f,      (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY], 8.f,     (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);


                    if(++motion_loop_cnt < 7) sciTimeCnt = music_offset_2018; //6

                }
                if(sciTimeCnt == bit_int_pc*2 + bit_int_pc/2 + music_offset_2018)
                {
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK1], 0.f, bit_int_pc/2*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK2], 0.f, bit_int_pc/2*INT_TIME, MODE_ABSOLUTE);
                }
                if(sciTimeCnt == music_offset_2018 + bit_int_pc*2 && motion_loop_cnt == 6)
                {
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSP], -80.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB], -80.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], -20.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWP], 0.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);

                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], -80.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -80.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], 20.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWP], 0.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);

                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY], -8.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY], 8.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);

                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWY], -90.f, bit_int_pc*2.5f*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWY], 90.f, bit_int_pc*2.5f*INT_TIME, MODE_ABSOLUTE);

                    bit_int_pc = bit_int_pc/2;

                    setMoveFinger(2,1, 400);


                }
                if(sciTimeCnt == music_offset_2018 + bit_int_pc*6) //8
                {
                    scidemo_num = 2031;
                    sciTimeCnt = 0;
                    music_offset_2018 = 1;
                }

                break;
            }
        case 2031:
        {

            if(sciTimeCnt == bit_int_pc*0 + music_offset_2018)
            {
                printf(">>> 2-4 Start \n");
                //setMoveFinger(0,-1,bit_int_pc*INT_TIME);
                //setMoveFinger(1,-1,bit_int_pc*INT_TIME);

            }

            if(sciTimeCnt == bit_int_pc*0 + motion_2_head_offest + music_offset_2018)
            {
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK1], -15.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK2], -15.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == bit_int_pc*1 + motion_2_head_offest + music_offset_2018)
            {
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK1], 0.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK2], 0.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == bit_int_pc*2 + motion_2_head_offest + music_offset_2018)
            {
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK1], -15.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK2], -15.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == bit_int_pc*3 + motion_2_head_offest + music_offset_2018)
            {
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK1], 0.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK2], 0.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == bit_int_pc*0 + music_offset_2018)
            {
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSP], -50.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB], -90.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], -20.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWP], 0.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], -50.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -90.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], 20.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWP], 0.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY], -8.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY], 8.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
            }

            if(sciTimeCnt == bit_int_pc*1 + music_offset_2018)
            {

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSP], -80.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB], -80.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], -20.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWP], -0.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], -80.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -80.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], 20.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWP], -0.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY], -8.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY], 8.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);

            }

            if(sciTimeCnt == bit_int_pc*2 + music_offset_2018)
            {
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSP], -50.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB], -90.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], -20.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWP], -0.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], -50.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -90.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], 20.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWP], -0.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY], -8.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY], 8.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == bit_int_pc*3 + music_offset_2018)
            {
                setMoveFinger(RF2,-1,400);
                setMoveFinger(RF3,-1,400);
                setMoveFinger(RF4,-1,400);
                setMoveFinger(RF5,-1,400);

                setMoveFinger(LF2,-1,400);
                setMoveFinger(LF3,-1,400);
                setMoveFinger(LF4,-1,400);
                setMoveFinger(LF5,-1,400);
            }

            if(sciTimeCnt == bit_int_pc*3 + music_offset_2018)
            {
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSP], -80.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB], -80.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], -20.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWP], 0.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], -80.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -80.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], 20.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWP], 0.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY], -8.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY], 8.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWY], -0.f, bit_int_pc*5.f*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWY], 0.f, bit_int_pc*5.f*INT_TIME, MODE_ABSOLUTE);
            }
            if(sciTimeCnt == music_offset_2018 + bit_int_pc*4)
            {
                setMoveFinger(RF1,-1,400);
                setMoveFinger(LF1,-1,400);
                scidemo_num = 2032;
                sciTimeCnt = 0;
                music_offset_2018 = 1;
                bit_int_pc = bit_int_pc*2;
                motion_loop_cnt = 0;
            }
            break;
            }
        case 2032:
            {
                if(sciTimeCnt == music_offset_2018 + 1 && motion_loop_cnt == 0)
                {
                    wbPosWST->setTaskPos(15, 1000, MODE_ABSOLUTE);
                }
                if(sciTimeCnt == music_offset_2018 + 1 && motion_loop_cnt == 2)
                {
                    wbPosWST->setTaskPos(-15, 1000, MODE_ABSOLUTE);
                }
                if(sciTimeCnt == music_offset_2018 + 1 && motion_loop_cnt == 4)
                {
                    wbPosWST->setTaskPos(0, 800, MODE_ABSOLUTE);
                }
                if(sciTimeCnt == bit_int_pc*0 + music_offset_2018)
                {
                    printf(">>> 1-8 Start \n");
//                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK1], -5.f, bit_int_pc/2*INT_TIME, MODE_ABSOLUTE);
//                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK2], -5.f, bit_int_pc/2*INT_TIME, MODE_ABSOLUTE);

                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSP], -65.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB], -10.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], -2.5f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWP], 25.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);

                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], -95.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -95.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], 20.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWP], -25.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);

                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY], 0.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY], 8.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);

                }

                if(sciTimeCnt == bit_int_pc/2 + music_offset_2018)
                {
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK1], 0.f, bit_int_pc/2*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK2], 0.f, bit_int_pc/2*INT_TIME, MODE_ABSOLUTE);
                }
                if(sciTimeCnt == bit_int_pc*1 + music_offset_2018)
                {
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK1], -5.f, bit_int_pc/2*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK2], -5.f, bit_int_pc/2*INT_TIME, MODE_ABSOLUTE);

                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSP], -95.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB], -95.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], -20.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWP], -25.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);


                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], -65.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -10.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], 2.5f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWP], 25.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);

                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY], -8.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY], 0.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);

                }
                if(sciTimeCnt == bit_int_pc + bit_int_pc/2 + music_offset_2018)
                {
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK1], 0.f, bit_int_pc/2*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK2], 0.f, bit_int_pc/2*INT_TIME, MODE_ABSOLUTE);
                }
                if(sciTimeCnt == bit_int_pc*2 + music_offset_2018 && motion_loop_cnt < 4)
                {
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK1], -5.f, bit_int_pc/2*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK2], -5.f, bit_int_pc/2*INT_TIME, MODE_ABSOLUTE);

                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSP], -65.f,    (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB], -10.f,    (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], -2.5f,    (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWP], 25.f,     (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);

                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], -95.f,    (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -95.f,   (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], 20.f,     (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWP], -25.f,    (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);

                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY], 0.f,      (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY], 8.f,     (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);

                    if(++motion_loop_cnt < 5) sciTimeCnt = music_offset_2018; //6

                }
                if(sciTimeCnt == bit_int_pc*2 + bit_int_pc/2 + music_offset_2018)
                {
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK1], 0.f, bit_int_pc/2*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK2], 0.f, bit_int_pc/2*INT_TIME, MODE_ABSOLUTE);
                }
                if(sciTimeCnt == music_offset_2018 + bit_int_pc*2  && motion_loop_cnt == 4)
                {
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSP], -85.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB], -60.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], 20.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWP], 0.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);

                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], -95.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -95.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], 10.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWP], 0.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);

                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY], -60.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
    //                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY], 8.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);

                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWY], 0.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
    //                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWY], 0.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);

                    bit_int_pc = bit_int_pc/2;
                    bit_int_pc_offset = bit_int_pc_offset/4;

                }
                if(sciTimeCnt == music_offset_2018 + bit_int_pc*6+0) //60
                {
                    scidemo_num = 2033;
                    sciTimeCnt = 0;
                    music_offset_2018 = 1;
                }
                break;
            }
        case 2033:
        {
            if(sciTimeCnt == music_offset_2018 + bit_int_pc*0)
            {
                printf(">>> 3-4 Start \n");


                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSP], -50.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB], -95.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], 10.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWP], 0.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], -120.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -55.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], 10.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWP], 0.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY], -40.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
//                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY], 8.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWY], 0.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
//                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWY], 0.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);

            }
            if(sciTimeCnt == music_offset_2018 + bit_int_pc*1)
            {

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSP], -85.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB], -60.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], 20.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWP], 0.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], -95.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -95.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], 10.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWP], 0.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY], -60.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
//                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY], 8.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWY], 0.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
//                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWY], 0.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);


            }
            if(sciTimeCnt == music_offset_2018 + bit_int_pc*2)
            {
                wbPosPelZ->setTaskPos(-z_101, 2*bit_int_pc*INT_TIME, MODE_RELATIVE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSP], -50.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB], -95.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], 10.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWP], 0.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], -120.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -55.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], 10.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWP], 0.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY], -40.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
//                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY], 8.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWY], 0.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
//                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWY], 0.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);



            }

            if(sciTimeCnt == music_offset_2018 + bit_int_pc*3)
            {

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSP], -85.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB], -60.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], 20.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWP], 0.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], -95.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -95.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], 10.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWP], 0.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY], -60.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
//                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY], 8.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWY], 0.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
//                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWY], 0.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);


            }
            if(sciTimeCnt == music_offset_2018 + bit_int_pc*4)
            {

                wbPosPelZ->setTaskPos(z_101, 2*bit_int_pc*INT_TIME, MODE_RELATIVE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSP], -50.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB], -95.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], 10.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWP], 0.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], -120.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -55.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], 10.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWP], 0.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY], -40.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
//                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY], 8.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWY], 0.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
//                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWY], 0.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);


            }
            if(sciTimeCnt == music_offset_2018 + bit_int_pc*4+200)
            {
                setMoveFinger(RF1, 1,400);
                setMoveFinger(RF2, 1,400);
                setMoveFinger(RF3, 1,400);
                setMoveFinger(RF4, 1,400);
                setMoveFinger(RF5, 1,400);
                setMoveFinger(LF1, 1,400);
                setMoveFinger(LF2, 1,400);
                setMoveFinger(LF3, 1,400);
                setMoveFinger(LF4, 1,400);
                setMoveFinger(LF5, 1,400);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP],  10.f, t1_101*2, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], -10.f, t1_101*2, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY],   0.f, t1_101*2, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -30.f, t1_101*2, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWY],   0.f, t1_101*2, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSP],  10.f, t1_101*2, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR],  10.f, t1_101*2, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY],   0.f, t1_101*2, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB], -30.f, t1_101*2, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWY],   0.f, t1_101*2, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NKY],   0.f, t1_101, MODE_ABSOLUTE);
                wbPosWST->setTaskPos(0.f, t1_101, MODE_ABSOLUTE);

            }


            if(sciTimeCnt == music_offset_2018 + bit_int_pc*4+1000)
            {
                initHeadHand(1);
            }
            if(sciTimeCnt == music_offset_2018 + bit_int_pc*4+1200)
            {
                printf(">>> Dance Motion end \n");
                scidemo_num = 2034;
                sciTimeCnt = 0;
                bit_int_pc = bit_int_pc*2;
                bit_int_pc_offset =bit_int_pc_offset*4;
                music_offset_2018 = 1;
                motion_loop_cnt = 0;
                sciMotionSelector = NO_ACT;
            }

            break;
        }
        }

        break;

    default:
        break;
    }
    sciTimeCnt++;
    motion_time_check++;
}


void TaskMotion::Setmove_walk_ready(float _MotionTime){
    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP],  10.f, _MotionTime, MODE_ABSOLUTE);
    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], -10.f, _MotionTime, MODE_ABSOLUTE);
    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY],   0.f, _MotionTime, MODE_ABSOLUTE);
    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -30.f, _MotionTime, MODE_ABSOLUTE);
    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWY],   0.f, _MotionTime, MODE_ABSOLUTE);
    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWP],   0.f, _MotionTime, MODE_ABSOLUTE);
    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSP],  10.f, _MotionTime, MODE_ABSOLUTE);
    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR],  10.f, _MotionTime, MODE_ABSOLUTE);
    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY],   0.f, _MotionTime, MODE_ABSOLUTE);
    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB], -30.f, _MotionTime, MODE_ABSOLUTE);
    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWY],   0.f, _MotionTime, MODE_ABSOLUTE);
    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWP],   0.f, _MotionTime, MODE_ABSOLUTE);

    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NKY],   0.f, _MotionTime, MODE_ABSOLUTE);
    wbPosWST->setTaskPos(0.f, bit_int*RT_TIMER_PERIOD_MS, MODE_ABSOLUTE);
}

void Upperbody_Gain_Lock()
{
    MCenableFrictionCompensation(JOINT_INFO[LSP].canch, JOINT_INFO[LSP].bno, JOINT_INFO[LSP].mch, DISABLE);
    MCBoardSetSwitchingMode(JOINT_INFO[LSP].canch,JOINT_INFO[LSP].bno, SW_MODE_COMPLEMENTARY);
    MCenableFrictionCompensation(JOINT_INFO[LSR].canch, JOINT_INFO[LSR].bno, JOINT_INFO[LSR].mch, DISABLE);
    MCBoardSetSwitchingMode(JOINT_INFO[LSR].canch,JOINT_INFO[LSR].bno, SW_MODE_COMPLEMENTARY);
    MCenableFrictionCompensation(JOINT_INFO[LSY].canch, JOINT_INFO[LSY].bno, JOINT_INFO[LSY].mch, DISABLE);
    MCBoardSetSwitchingMode(JOINT_INFO[LSY].canch,JOINT_INFO[LSY].bno, SW_MODE_COMPLEMENTARY);
    MCenableFrictionCompensation(JOINT_INFO[LEB].canch, JOINT_INFO[LEB].bno, JOINT_INFO[LEB].mch, DISABLE);
    MCBoardSetSwitchingMode(JOINT_INFO[LEB].canch,JOINT_INFO[LEB].bno, SW_MODE_COMPLEMENTARY);

    MCenableFrictionCompensation(JOINT_INFO[RSP].canch, JOINT_INFO[RSP].bno, JOINT_INFO[RSP].mch, DISABLE);
    MCBoardSetSwitchingMode(JOINT_INFO[RSP].canch,JOINT_INFO[RSP].bno, SW_MODE_COMPLEMENTARY);
    MCenableFrictionCompensation(JOINT_INFO[RSR].canch, JOINT_INFO[RSR].bno, JOINT_INFO[RSR].mch, DISABLE);
    MCBoardSetSwitchingMode(JOINT_INFO[RSY].canch,JOINT_INFO[RSY].bno, SW_MODE_COMPLEMENTARY);
    MCenableFrictionCompensation(JOINT_INFO[RSY].canch, JOINT_INFO[RSY].bno, JOINT_INFO[RSY].mch, DISABLE);
    MCBoardSetSwitchingMode(JOINT_INFO[RSR].canch,JOINT_INFO[RSR].bno, SW_MODE_COMPLEMENTARY);
    MCenableFrictionCompensation(JOINT_INFO[REB].canch, JOINT_INFO[REB].bno, JOINT_INFO[REB].mch, DISABLE);
    MCBoardSetSwitchingMode(JOINT_INFO[REB].canch,JOINT_INFO[REB].bno, SW_MODE_COMPLEMENTARY);
    MCenableFrictionCompensation(JOINT_INFO[RWY].canch, JOINT_INFO[RWY].bno, JOINT_INFO[RWY].mch, DISABLE);

            MCJointGainOverride(JOINT_INFO[LSP].canch, JOINT_INFO[LSP].bno, JOINT_INFO[LSP].mch, 0,10000); //--LSP
            MCJointGainOverride(JOINT_INFO[LSR].canch, JOINT_INFO[LSR].bno, JOINT_INFO[LSR].mch, 0,10000); //--LSR
            MCJointGainOverride(JOINT_INFO[LSY].canch, JOINT_INFO[LSY].bno, JOINT_INFO[LSY].mch, 0,10000); //--LSY
            MCJointGainOverride(JOINT_INFO[LEB].canch, JOINT_INFO[LEB].bno, JOINT_INFO[LEB].mch, 0,10000); //--LEB

            MCJointGainOverride(JOINT_INFO[RSP].canch, JOINT_INFO[RSP].bno, JOINT_INFO[RSP].mch, 0,10000); //--RSP
            MCJointGainOverride(JOINT_INFO[RSR].canch, JOINT_INFO[RSR].bno, JOINT_INFO[RSR].mch, 0,10000); //--RSR
            MCJointGainOverride(JOINT_INFO[RSY].canch, JOINT_INFO[RSY].bno, JOINT_INFO[RSY].mch, 0,10000); //--RSY
            MCJointGainOverride(JOINT_INFO[REB].canch, JOINT_INFO[REB].bno, JOINT_INFO[REB].mch, 0,10000); //--REB

            MCJointGainOverride_old(JOINT_INFO[RWY].canch, JOINT_INFO[RWY].bno, 1000, 1000, 10000);
}

void Upperbody_Gain_Override()
{
    //ZeroGainLeftArm();
    ZeroGainRightArm();
}
int ZeroGainRightArm(){
    //-- Rsp
    MCsetFrictionParameter(JOINT_INFO[RSP].canch, JOINT_INFO[RSP].bno, JOINT_INFO[RSP].mch, 1000, 8, 0);
    MCBoardSetSwitchingMode(JOINT_INFO[RSP].canch, JOINT_INFO[RSP].bno, SW_MODE_NON_COMPLEMENTARY);
    MCenableFrictionCompensation(JOINT_INFO[RSP].canch, JOINT_INFO[RSP].bno, JOINT_INFO[RSP].mch, ENABLE);
    MCJointGainOverride(JOINT_INFO[RSP].canch, JOINT_INFO[RSP].bno, JOINT_INFO[RSP].mch, 35, 700);
    usleep(5000);
    MCsetFrictionParameter(JOINT_INFO[RSR].canch, JOINT_INFO[RSR].bno, JOINT_INFO[RSR].mch, 1000, 7, 0);
    MCBoardSetSwitchingMode(JOINT_INFO[RSR].canch, JOINT_INFO[RSR].bno, SW_MODE_NON_COMPLEMENTARY);
    MCenableFrictionCompensation(JOINT_INFO[RSR].canch, JOINT_INFO[RSR].bno, JOINT_INFO[RSR].mch, ENABLE);
    MCJointGainOverride(JOINT_INFO[RSR].canch, JOINT_INFO[RSR].bno, JOINT_INFO[RSR].mch, 50, 700);
    usleep(5000);
    MCsetFrictionParameter(JOINT_INFO[RSY].canch, JOINT_INFO[RSY].bno, JOINT_INFO[RSY].mch, 1000, 9, 0);
    MCBoardSetSwitchingMode(JOINT_INFO[RSY].canch, JOINT_INFO[RSY].bno, SW_MODE_NON_COMPLEMENTARY);
    MCenableFrictionCompensation(JOINT_INFO[RSY].canch, JOINT_INFO[RSY].bno, JOINT_INFO[RSY].mch, ENABLE);
    MCJointGainOverride(JOINT_INFO[RSY].canch, JOINT_INFO[RSY].bno, JOINT_INFO[RSY].mch, 20, 700);
    usleep(5000);
    MCsetFrictionParameter(JOINT_INFO[REB].canch, JOINT_INFO[REB].bno, JOINT_INFO[REB].mch, 1000, 5, 0);
    MCBoardSetSwitchingMode(JOINT_INFO[REB].canch, JOINT_INFO[REB].bno, SW_MODE_NON_COMPLEMENTARY);
    MCenableFrictionCompensation(JOINT_INFO[REB].canch, JOINT_INFO[REB].bno, JOINT_INFO[REB].mch, ENABLE);
    MCJointGainOverride(JOINT_INFO[REB].canch, JOINT_INFO[REB].bno, JOINT_INFO[REB].mch, 38, 50);
    usleep(5000);

    MCJointGainOverride_old(JOINT_INFO[RWY].canch, JOINT_INFO[RWY].bno, 1, 1, 700);
    FILE_LOG(logSUCCESS)<<"Zero Gain Right Arm!";

    return 0;
}

int ZeroGainLeftArm(){
    //-- lsp
    MCsetFrictionParameter(JOINT_INFO[LSP].canch, JOINT_INFO[LSP].bno, JOINT_INFO[LSP].mch, 1000, 10, 0);
    MCBoardSetSwitchingMode(JOINT_INFO[LSP].canch, JOINT_INFO[LSP].bno, SW_MODE_NON_COMPLEMENTARY);
    MCenableFrictionCompensation(JOINT_INFO[LSP].canch, JOINT_INFO[LSP].bno, JOINT_INFO[LSP].mch, ENABLE);
    MCJointGainOverride(JOINT_INFO[LSP].canch, JOINT_INFO[LSP].bno, JOINT_INFO[LSP].mch, 50, 10);
    usleep(5000);

    //-- lsr
    MCsetFrictionParameter(JOINT_INFO[LSR].canch, JOINT_INFO[LSR].bno, JOINT_INFO[LSR].mch, 1000, 7, 0);
    MCBoardSetSwitchingMode(JOINT_INFO[LSR].canch, JOINT_INFO[LSR].bno, SW_MODE_NON_COMPLEMENTARY);
    MCenableFrictionCompensation(JOINT_INFO[LSR].canch, JOINT_INFO[LSR].bno, JOINT_INFO[LSR].mch, ENABLE);
    MCJointGainOverride(JOINT_INFO[LSR].canch, JOINT_INFO[LSR].bno, JOINT_INFO[LSR].mch, 50, 10);
    usleep(5000);

    //--- lsy, leb
    MCsetFrictionParameter(JOINT_INFO[LSY].canch, JOINT_INFO[LSY].bno, JOINT_INFO[LSY].mch, 1000, 9, 0);
    MCBoardSetSwitchingMode(JOINT_INFO[LSY].canch, JOINT_INFO[LSY].bno, SW_MODE_NON_COMPLEMENTARY);
    MCenableFrictionCompensation(JOINT_INFO[LSY].canch, JOINT_INFO[LSY].bno, JOINT_INFO[LSY].mch, ENABLE);
    MCJointGainOverride(JOINT_INFO[LSY].canch, JOINT_INFO[LSY].bno, JOINT_INFO[LSY].mch, 20, 10);
    usleep(5000);
    MCsetFrictionParameter(JOINT_INFO[LEB].canch, JOINT_INFO[LEB].bno, JOINT_INFO[LEB].mch, 1000, 5, 0);
    MCBoardSetSwitchingMode(JOINT_INFO[LEB].canch, JOINT_INFO[LEB].bno, SW_MODE_NON_COMPLEMENTARY);
    MCenableFrictionCompensation(JOINT_INFO[LEB].canch, JOINT_INFO[LEB].bno, JOINT_INFO[LEB].mch, ENABLE);
    MCJointGainOverride(JOINT_INFO[LEB].canch, JOINT_INFO[LEB].bno, JOINT_INFO[LEB].mch, 40, 10);
    usleep(5000);

    MCJointGainOverride_old(JOINT_INFO[LWY].canch, JOINT_INFO[LWY].bno, 1, 1, 10);
    FILE_LOG(logSUCCESS)<<"Zero Gain Left Arm!";
    return 0;
}
