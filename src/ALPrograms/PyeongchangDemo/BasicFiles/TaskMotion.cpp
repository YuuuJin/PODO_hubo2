#include "TaskMotion.h"

#include "ManualCAN.h"
#define INT_TIME 5
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

extern bool Control_OnOff_Flag;

extern int      pyeongchangdemo_num;
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


int bit_int_pc;// = BIT_PC;//300;//90; //86
int bit_int_pc_offset = 20;
int music_offset_2018 = 980;//670;//680;//750;
int motion_loop_cnt = 0;

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

         localJoint[RSP] = tempData[6];  localJoint[RSR] = tempData[7];  localJoint[RSY] = tempData[8];
        localJoint[REB] = tempData[9];  localJoint[RWY] = tempData[10]; localJoint[RWP] = tempData[11];
        if(pyeongchangdemo_num==25)
        localJoint[LSP] = tempData[12];//*0.9;
        else
        localJoint[LSP] = tempData[12];

        if(pyeongchangdemo_num==BOW_TWO_W){
            localJoint[WST] = tempData[5]*0.5f;
        }else{
            localJoint[WST] = tempData[5];
        }


        localJoint[LSR] = tempData[13]; localJoint[LSY] = tempData[14]; localJoint[LEB] = tempData[15]; localJoint[LWY] = 0.0;//tempData[16];
        localJoint[LWP] = tempData[17]; localJoint[NKY] = tempData[18]; localJoint[NK1] = tempData[19]; localJoint[NK2] = tempData[20];
//        localJoint[RF1] = tempData[21]; localJoint[RF2] = tempData[22]; localJoint[RF3] = tempData[23]; localJoint[RF4] = tempData[24];
//        localJoint[RF5] = tempData[25]; localJoint[LF1] = tempData[26]; localJoint[LF2] = tempData[27]; localJoint[LF3] = tempData[28];
//        localJoint[LF4] = tempData[29]; localJoint[LF5] = tempData[30];
        if(pyeongchangdemo_num==108){
            localJoint[RF1] = -1.*tempData[21]; localJoint[RF2] = -1.*tempData[22]; localJoint[RF3] = -1.*tempData[23]; localJoint[RF4] = -1.*tempData[24];
            localJoint[RF5] = -1.*tempData[25]; localJoint[LF1] = -1.*tempData[28]; localJoint[LF2] = -1.*tempData[28]; localJoint[LF3] = -1.*tempData[28];
            localJoint[LF4] = -1.*tempData[29]; localJoint[LF5] = -1.*tempData[30];
        }else if(pyeongchangdemo_num==20){
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
    wbOriPel->RefCurrent[0] = tempq0;
    wbOriPel->RefCurrent[1] = tempq1;
    wbOriPel->RefCurrent[2] = tempq2;
    wbOriPel->RefCurrent[3] = tempq3;
//    printf("Quaternion = %f,%f,%f,%f\n",tempq0,tempq1,tempq2,tempq3);


    //for(int i=1; i<=4; i++) wbOriPel->RefCurrent[i] = localQpel[i-1];
}


void TaskMotion::pyeongchangDemo(){
    switch(sciMotionSelector){
    case PYEONGCHANG_DEMO:
        switch(pyeongchangdemo_num){
            //--------------------------------------------------------------------------------------
        case HANDSHAKE_READY:
            if(tmTimeCnt == 0){
                FILE_LOG(logSUCCESS) << "Pyeongchang_DEMO: HANDSHAKE_READY";
                for(int i=RF1; i<=RF5; i++) wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[i], -10.f, 700, MODE_ABSOLUTE);
                Upperbody_Gain_Override();

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK1], 20.f, 1000.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK2], 20.f, 1000.f, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], -78.f, 1000.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], 21.5f, 1000.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY], 42.4f, 1000.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -28.f, 1000.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWY], -39.85f, 1000.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWP], 10.f, 1000.f, MODE_ABSOLUTE);
            }if(tmTimeCnt == 200){
                for(int i=RF1; i<=RF5; i++) wbUBJoint->motionJoint[i]->RefAngleCurrent = 90.0f;//setMoveJointAngle(wbUBJoint->motionJoint[i], 10.f, 500, MODE_ABSOLUTE);
            }if(tmTimeCnt == 280){
                for(int i=RF1; i<=RF5; i++) wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[i], 0.f, 500, MODE_ABSOLUTE);
            }if(tmTimeCnt == 400){
                pyeongchangdemo_num = NO_ACT;
                Control_OnOff_Flag = true;
            }break;
        case HANDSHAKE_LOWGAIN:
            if(tmTimeCnt == 0){
                FILE_LOG(logSUCCESS) << "Pyeongchang_DEMO: HANDSHAKE_LOWGAIN";
                Upperbody_Gain_Override();
            }if(tmTimeCnt == 10){
                pyeongchangdemo_num = NO_ACT;
            }break;
        case HANDSHAKE_GRIP_ON:
            if(tmTimeCnt == 0){
                FILE_LOG(logSUCCESS) << "Pyeongchang_DEMO: HANDSHAKE_GRIP_ON";
                setMoveFinger(0,-1,700);//2 both 0 right 1 left RF1
            }if(tmTimeCnt == 10){
                pyeongchangdemo_num = NO_ACT;
            }break;
        case HANDSHAKE_GRIP_OFF:
            if(tmTimeCnt == 0){
                FILE_LOG(logSUCCESS) << "Pyeongchang_DEMO: HANDSHAKE_GRIP_OFF";
                setMoveFinger(0,1,700);//2 both 0 right 1 left RF1
            }if(tmTimeCnt == 10){
                pyeongchangdemo_num = NO_ACT;
            }break;
        case HANDSHAKE_HIGHGAIN:
            if(tmTimeCnt == 0){
                FILE_LOG(logSUCCESS) << "Pyeongchang_DEMO: HANDSHAKE_HIGHGAIN";
//                Upperbody_Gain_Lock();
            }if(tmTimeCnt == 10){
                pyeongchangdemo_num = NO_ACT;
            }break;
        case HANDSHAKE_INIT:
            if(tmTimeCnt == 0){
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

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], 10.f, 2000.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], -10.f, 2000.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY], 0.f, 2000.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -30.f, 2000.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWY], 0.f, 2000.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWP], 0.f, 2000.f, MODE_ABSOLUTE);
            }if(tmTimeCnt == 450){
                initHeadHand(1);//0 head . 1 hand . 2 hand+head
            }if(tmTimeCnt == 550){
                pyeongchangdemo_num = NO_ACT;
                Control_OnOff_Flag = false;
            }break;
            //--------------------------------------------------------------------------------------
        case BOUQUET_READY:
            if(tmTimeCnt == 0){
                FILE_LOG(logSUCCESS) << "Pyeongchang_DEMO: BOUQUET_READY";
                setMoveFinger(0,1,700);//2 both 0 right 1 left RF1
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], -15.f, 2000.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], 0.f, 2000.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY], 25.f, 2000.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -90.f, 2000.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWY], -105.f, 2000.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWP], -15.f, 2000.f, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSP], -15.f, 2000.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], 0.f, 2000.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY], -25.f, 2000.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB], -90.f, 2000.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWY], 105.f, 2000.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWP], -15.f, 2000.f, MODE_ABSOLUTE);

            }if(tmTimeCnt == 400){
                pyeongchangdemo_num = NO_ACT;
            }break;
        case BOUQUET_GIVE:
            if(tmTimeCnt == 0){
                FILE_LOG(logSUCCESS) << "Pyeongchang_DEMO: BOUQUET_GIVE";
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], -65.f, 2000.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], 10.f, 2000.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -45.f, 2000.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWY], -110.f, 2000.f, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSP], -65.f, 2000.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], -10.f, 2000.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB], -45.f, 2000.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWY], 110.f, 2000.f, MODE_ABSOLUTE);
            }if(tmTimeCnt == 400){
                pyeongchangdemo_num = NO_ACT;
            }break;
        case BOUQUET_INIT:
            if(tmTimeCnt == 0){
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
            }if(tmTimeCnt == 350){
                initHeadHand(1);
            }if(tmTimeCnt == 500){
                pyeongchangdemo_num = NO_ACT;
            }break;
            //--------------------------------------------------------------------------------------

        case CLAP_READY:
            if(tmTimeCnt == 0){
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
            }if(tmTimeCnt == 300){
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY], 42.0f, 250.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY], -42.00f, 250.f, MODE_ABSOLUTE);
            }if(tmTimeCnt == 350){
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY], 30.0f, 150.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY], -30.00f, 150.f, MODE_ABSOLUTE);
                if(clap_cnt == 1){
                    tmTimeCnt = 410;
                }
            }if(tmTimeCnt == 381 && clap_cnt > 1){
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY], 42.0f, 150.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY], -42.00f, 150.f, MODE_ABSOLUTE);
            }if(tmTimeCnt == 411 && clap_cnt > 1){
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY], 30.0f, 150.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY], -30.00f, 150.f, MODE_ABSOLUTE);
                if(--clap_cnt > 1){
                    tmTimeCnt = 350;
                }
            }if(tmTimeCnt == 441){
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
            }if(tmTimeCnt == 751){
                FILE_LOG(logSUCCESS) << "Pyeongchang_DEMO: CLAP Motion END.";
                pyeongchangdemo_num = NO_ACT;
            }break;
        case CLAP_THREE:
            if(tmTimeCnt == 0){
                FILE_LOG(logSUCCESS) << "Pyeongchang_DEMO: CLAP_THREE";
            }if(tmTimeCnt == 10){
                pyeongchangdemo_num = NO_ACT;
            }break;
        case CLAP_KOREA:
            if(tmTimeCnt == 0){
                FILE_LOG(logSUCCESS) << "Pyeongchang_DEMO: CLAP_KOREA";
            }if(tmTimeCnt == 10){
                pyeongchangdemo_num = NO_ACT;
            }break;
        case CLAP_INIT:
            if(tmTimeCnt == 0){
                FILE_LOG(logSUCCESS) << "Pyeongchang_DEMO: CLAP_INIT";
            }if(tmTimeCnt == 10){
                pyeongchangdemo_num = NO_ACT;
            }break;
            //--------------------------------------------------------------------------------------

        case HEAD_YES:
            if(tmTimeCnt == 0 + 120){
                FILE_LOG(logSUCCESS) << "Pyeongchang_DEMO: HEAD_YES";
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK1], -15.f, 200.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK2], -15.f, 200.f, MODE_ABSOLUTE);
            }if(tmTimeCnt == 40 + 120){
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK1], 0.f, 200.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK2], 0.f, 200.f, MODE_ABSOLUTE);
            }if(tmTimeCnt == 80 + 120){
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK1], -15.f, 200.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK2], -15.f, 200.f, MODE_ABSOLUTE);
            }if(tmTimeCnt == 120 + 120){
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK1], 0.f, 200.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK2], 0.f, 200.f, MODE_ABSOLUTE);
            }if(tmTimeCnt == 160 + 120){
                pyeongchangdemo_num = NO_ACT;
            }break;
        case HEAD_NO:
            if(tmTimeCnt == 0){
                FILE_LOG(logSUCCESS) << "Pyeongchang_DEMO: HEAD_NO";
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NKY], -15.f, 200.f, MODE_ABSOLUTE);
            }if(tmTimeCnt == 40){
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NKY], 15.f, 400.f, MODE_ABSOLUTE);
            }if(tmTimeCnt == 120){
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NKY], -15.f, 400.f, MODE_ABSOLUTE);
            }if(tmTimeCnt == 200){
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NKY], 0.f, 200.f, MODE_ABSOLUTE);
            }if(tmTimeCnt == 240){
                pyeongchangdemo_num = NO_ACT;
            }break;
        case HEAD_STRETCHING:
            if(tmTimeCnt == 0){
                FILE_LOG(logSUCCESS) << "Pyeongchang_DEMO: HEAD_STRETCHING";
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK1], -15.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK2], -15.f, 400.f, MODE_ABSOLUTE);
            }if(tmTimeCnt < 580 && tmTimeCnt > 80){
                wbUBJoint->motionJoint[NK1]->RefAngleCurrent = -1.f*(15.f*cos(2*M_PI*(tmTimeCnt-80)/500.f)) -1.f*(15.f*sin(2*M_PI*(tmTimeCnt-80)/500.f));
                wbUBJoint->motionJoint[NK2]->RefAngleCurrent = -1.f*(15.f*cos(2*M_PI*(tmTimeCnt-80)/500.f))+1.f*(15.f*sin(2*M_PI*(tmTimeCnt-80)/500.f));
            }if(tmTimeCnt == 580){
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK1], 0.f, 400.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[NK2], 0.f, 400.f, MODE_ABSOLUTE);
            }if(tmTimeCnt == 660){
                pyeongchangdemo_num = NO_ACT;
            }break;
        case HEAD_INIT:
            if(tmTimeCnt == 0){
                FILE_LOG(logSUCCESS) << "Pyeongchang_DEMO: HEAD_INIT";
            }if(tmTimeCnt == 10){
                pyeongchangdemo_num = NO_ACT;
            }break;
            //--------------------------------------------------------------------------------------

        case BOW_ONE:
            if(tmTimeCnt == 0){
                FILE_LOG(logSUCCESS) << "Pyeongchang_DEMO: BOW_ONE";
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], -85.f, 1000.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], 12.f, 1000.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY], 0.f, 1000.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -55.f, 1000.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWY], 85.f, 1000.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWP], 0.f, 1000.f, MODE_ABSOLUTE);
            }if(tmTimeCnt == 200){
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY], 10.f, 500.f, MODE_ABSOLUTE);
            }if(tmTimeCnt == 300){
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY], -10.f, 500.f, MODE_ABSOLUTE);
            }if(tmTimeCnt == 400){
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY], 10.f, 500.f, MODE_ABSOLUTE);
            }if(tmTimeCnt == 500){
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY], -10.f, 500.f, MODE_ABSOLUTE);
            }if(tmTimeCnt == 600){
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY], 10.f, 500.f, MODE_ABSOLUTE);
            }if(tmTimeCnt == 700){
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY], -10.f, 500.f, MODE_ABSOLUTE);
            }if(tmTimeCnt == 800){
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY], 10.f, 500.f, MODE_ABSOLUTE);
            }if(tmTimeCnt == 900){
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY], -10.f, 500.f, MODE_ABSOLUTE);
            }if(tmTimeCnt == 1050){
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], 10.f, 1500.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], -10.f, 1500.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY], 0.f, 1500.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -30.f, 1500.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWY], 0.f, 1500.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWP], 0.f, 1500.f, MODE_ABSOLUTE);
            }if(tmTimeCnt == 1350){
                pyeongchangdemo_num = NO_ACT;
            }break;
        case BOW_TWO:
            /*
            if(tmTimeCnt == 0){
                FILE_LOG(logSUCCESS) << "Pyeongchang_DEMO: BOW_TWO";
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], -60.f, 3000.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], 0.f, 3000.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY], 10.f, 3000.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -80.f, 3000.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWY], 70.f, 3000.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWP], 0.f, 3000.f, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSP], -60.f, 3000.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], 0.f, 3000.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY], -10.f, 3000.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB], -80.f, 3000.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWY], -70.f, 3000.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWP], 0.f, 3000.f, MODE_ABSOLUTE);
            }if(tmTimeCnt == 600){
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[WST], 10.f, 300.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], -60.f, 300.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], -20.f, 300.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY], -20.f, 300.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -80.f, 300.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWY], 80.f, 300.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWP], 20.f, 300.f, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSP], -60.f, 300.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], 20.f, 300.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY], 20.f, 300.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB], -80.f, 300.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWY], -80.f, 300.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWP], 20.f, 300.f, MODE_ABSOLUTE);
            }if(tmTimeCnt == 660){
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[WST], -10.f, 300.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], -60.f, 300.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], 0.f, 300.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY], 10.f, 300.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -80.f, 300.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWY], 70.f, 300.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWP], 0.f, 300.f, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSP], -60.f, 300.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], 0.f, 300.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY], -10.f, 300.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB], -80.f, 300.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWY], -70.f, 300.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWP], 0.f, 300.f, MODE_ABSOLUTE);
            }if(tmTimeCnt == 720){
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[WST], 10.f, 300.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], -60.f, 300.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], -20.f, 300.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY], -20.f, 300.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -80.f, 300.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWY], 80.f, 300.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWP], 20.f, 300.f, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSP], -60.f, 300.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], 20.f, 300.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY], 20.f, 300.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB], -80.f, 300.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWY], -80.f, 300.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWP], 20.f, 300.f, MODE_ABSOLUTE);
            }if(tmTimeCnt == 780){
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[WST], -10.f, 300.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], -60.f, 300.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], 0.f, 300.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY], 10.f, 300.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -80.f, 300.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWY], 70.f, 300.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWP], 0.f, 300.f, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSP], -60.f, 300.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], 0.f, 300.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY], -10.f, 300.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB], -80.f, 300.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWY], -70.f, 300.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWP], 0.f, 300.f, MODE_ABSOLUTE);
            }if(tmTimeCnt == 840){
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[WST], 10.f, 300.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], -60.f, 300.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], -20.f, 300.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY], -20.f, 300.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -80.f, 300.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWY], 80.f, 300.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWP], 20.f, 300.f, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSP], -60.f, 300.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], 20.f, 300.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY], 20.f, 300.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB], -80.f, 300.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWY], -80.f, 300.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWP], 20.f, 300.f, MODE_ABSOLUTE);
            }if(tmTimeCnt == 900){
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[WST], -10.f, 300.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], -60.f, 300.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], 0.f, 300.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY], 10.f, 300.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -80.f, 300.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWY], 70.f, 300.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWP], 0.f, 300.f, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSP], -60.f, 300.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], 0.f, 300.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY], -10.f, 300.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB], -80.f, 300.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWY], -70.f, 300.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWP], 0.f, 300.f, MODE_ABSOLUTE);
            }if(tmTimeCnt == 960){
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[WST], 10.f, 300.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], -60.f, 300.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], -20.f, 300.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY], -20.f, 300.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -80.f, 300.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWY], 80.f, 300.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWP], 20.f, 300.f, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSP], -60.f, 300.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], 20.f, 300.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY], 20.f, 300.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB], -80.f, 300.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWY], -80.f, 300.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWP], 20.f, 300.f, MODE_ABSOLUTE);
            }if(tmTimeCnt == 1020){
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[WST], -10.f, 300.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], -60.f, 300.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], 0.f, 300.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY], 10.f, 300.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -80.f, 300.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWY], 70.f, 300.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWP], 0.f, 300.f, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSP], -60.f, 300.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], 0.f, 300.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY], -10.f, 300.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB], -80.f, 300.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWY], -70.f, 300.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWP], 0.f, 300.f, MODE_ABSOLUTE);
            }if(tmTimeCnt == 1200){
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[WST], 0.f, 300.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], 10.f, 3000.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], -10.f, 3000.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY], 0.f, 3000.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -30.f, 3000.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWY], 0.f, 3000.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWP], 0.f, 3000.f, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSP], 10.f, 3000.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], 10.f, 3000.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY], 0.f, 3000.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB], -30.f, 3000.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWY], 0.f, 3000.f, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWP], 0.f, 3000.f, MODE_ABSOLUTE);
            }if(tmTimeCnt == 1800){
                pyeongchangdemo_num = NO_OF_MOTION;
            }*/
            break;

        case BOW_GREETING:
            if(fileReadFlag == ENABLE){
                LeadTextData();
            }ChaseTextData();
            if(tmTimeCnt == 0){
                FILE_LOG(logSUCCESS) << "Pyeongchang_DEMO: BOW_TWO_W";
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWY], -85.f, 2000.f, MODE_ABSOLUTE);
            }if(tmTimeCnt == 1500){
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWY], 0.f, 2000.f, MODE_ABSOLUTE);
            }if(ringIndexBack == ringIndexFront){
                pyeongchangdemo_num = BOW_LAST_MOTION;
                printf("\n ========== %f \n",tmTimeCnt*0.005);
            }break;
            //--------------------------------------------------------------------------------------


        case BOW_TWO_W://SCENE #2 - Greeting with arm shake
            if(fileReadFlag == ENABLE){
                LeadTextData();
            }ChaseTextData();
            if(tmTimeCnt == 0){
                FILE_LOG(logSUCCESS) << "Pyeongchang_DEMO: BOW_TWO_W";
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWY], -85.f, 2000.f, MODE_ABSOLUTE);
            }if(tmTimeCnt == 1500){
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWY], 0.f, 2000.f, MODE_ABSOLUTE);
            }if(ringIndexBack == ringIndexFront){
                pyeongchangdemo_num = BOW_LAST_MOTION;
                printf("\n ========== %f \n",tmTimeCnt*0.005);
            }break;
        case BOW_LAST_MOTION:
            if(tmTimeCnt == 0)
                initHeadHand(1);
            else if(tmTimeCnt==200)
            {
                pyeongchangdemo_num = NO_ACT;
                printf(">>> Greeting motion end\n");
            }
            break;



        case GUID_LEFT:
            if(tmTimeCnt == 0){
                FILE_LOG(logSUCCESS) << "Pyeongchang_DEMO: GUID_LEFT";
                setMoveFinger(1, 1, 700);
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
            }if(tmTimeCnt == 500){
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
            }if(tmTimeCnt == 850){
                initHeadHand(1);
            }if(tmTimeCnt == 900){
                pyeongchangdemo_num = GUID_INIT;
            }break;
        case GUID_RIGHT:
            if(tmTimeCnt == 0){
                FILE_LOG(logSUCCESS) << "Pyeongchang_DEMO: GUID_RIGHT";
                setMoveFinger(0, 1, 700);
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
            }if(tmTimeCnt == 500){
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
            }if(tmTimeCnt == 850){
                initHeadHand(1);
            }if(tmTimeCnt == 900){
                pyeongchangdemo_num = GUID_INIT;
            }break;
        case GUID_INIT:
            if(tmTimeCnt == 0){
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
            }if(tmTimeCnt == 350){
                initHeadHand(1);
            }if(tmTimeCnt == 500){
                pyeongchangdemo_num = NO_ACT;
            }break;

            //1-1  2-1  1-2  3-1
        case 2018:
        {

            //Pelvis move + Left move =========================================================================================================
            if(tmTimeCnt == music_offset_2018 - 450)
            {
                printf(">>> 1-1 Start \n");
                z_101 = 0.04;
                wbPosPelZ->setTaskPos(-z_101, 400.f*INT_TIME, MODE_RELATIVE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSP], -40.f, 400*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LEB], -60.f, 400*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSR], -10.f, 400*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSP], -40.f, 400*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[REB], -60.f, 400*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSR], 10.f, 400*INT_TIME, MODE_ABSOLUTE);
                setMoveFinger(0,-1,400);
                setMoveFinger(1,-1,400);
            }
            if(tmTimeCnt == bit_int_pc*0 + music_offset_2018)
            {
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
            }
            if(tmTimeCnt == bit_int_pc*1 + music_offset_2018)
            {
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
            if(tmTimeCnt == bit_int_pc*2 + music_offset_2018)
            {
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

                if(++motion_loop_cnt < 6) tmTimeCnt = music_offset_2018; //6

            }
            if(tmTimeCnt == music_offset_2018 + bit_int_pc*3)
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

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWY], -60.f, (bit_int_pc)*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWY], 60.f, (bit_int_pc)*INT_TIME, MODE_ABSOLUTE);

                bit_int_pc = bit_int_pc/2;

                setMoveFinger(2,1, 400);


            }
            if(tmTimeCnt == music_offset_2018 + bit_int_pc*8+30)
            {
                pyeongchangdemo_num = 2019;//20190
                tmTimeCnt = 0;
                music_offset_2018 = 1;
            }
            break;
        }
        case 2019:
        {

            if(tmTimeCnt == bit_int_pc*0 + music_offset_2018)
            {
                printf(">>> 2-1 Start \n");
                //setMoveFinger(0,-1,bit_int_pc*INT_TIME);
                //setMoveFinger(1,-1,bit_int_pc*INT_TIME);

            }

            if(tmTimeCnt == bit_int_pc*0 + music_offset_2018)
            {
                wbPosPelZ->setTaskPos(-z_101, bit_int_pc*INT_TIME, MODE_RELATIVE);

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

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWY], -90.f, (bit_int_pc)*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWY], 90.f, (bit_int_pc)*INT_TIME, MODE_ABSOLUTE);


            }

            if(tmTimeCnt == bit_int_pc*1 + music_offset_2018)
            {

                wbPosPelZ->setTaskPos(z_101, bit_int_pc*INT_TIME, MODE_RELATIVE);


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

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWY], -90.f, (bit_int_pc)*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWY], 90.f, (bit_int_pc )*INT_TIME, MODE_ABSOLUTE);

            }


            if(tmTimeCnt == bit_int_pc*2 + music_offset_2018)
            {
                wbPosPelZ->setTaskPos(-z_101, bit_int_pc*INT_TIME, MODE_RELATIVE);

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

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWY], -90.f, (bit_int_pc)*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWY], 90.f, (bit_int_pc)*INT_TIME, MODE_ABSOLUTE);


            }
            if(tmTimeCnt == bit_int_pc*3 + music_offset_2018)
            {
                setMoveFinger(2,-1, 400);
            }

            if(tmTimeCnt == bit_int_pc*3 + music_offset_2018)
            {
                //wbPosPelZ->setTaskPos(z_101, bit_int_pc*INT_TIME, MODE_RELATIVE);


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

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWY], -0.f, (bit_int_pc*4)*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWY], 0.f, (bit_int_pc*4)*INT_TIME, MODE_ABSOLUTE);

            }




            if(tmTimeCnt == music_offset_2018 + bit_int_pc*4)
            {
                pyeongchangdemo_num = 2020;
                tmTimeCnt = 0;
                music_offset_2018 = 1;
                bit_int_pc = bit_int_pc*2;
                motion_loop_cnt = 0;
            }

            break;
            }
        case 2020:
            {
                if(tmTimeCnt == bit_int_pc*0 + music_offset_2018)
                {
                    printf(">>> 1-2 Start \n");

                    wbPosPelZ->setTaskPos(z_101, bit_int_pc*INT_TIME, MODE_RELATIVE);


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

                if(tmTimeCnt == bit_int_pc*1 + music_offset_2018)
                {
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
                if(tmTimeCnt == bit_int_pc*2 + music_offset_2018 && motion_loop_cnt < 4)
                {
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

                    if(++motion_loop_cnt < 5) tmTimeCnt = music_offset_2018; //6

                }
                if(tmTimeCnt == music_offset_2018 + bit_int_pc*2  && motion_loop_cnt == 4)
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

                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY], -60.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
    //                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY], 8.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);

                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWY], 0.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
    //                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWY], 0.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);

                    bit_int_pc = bit_int_pc/2;
                    bit_int_pc_offset = bit_int_pc_offset/4;
                    //setMoveFinger(0,1,bit_int_pc*INT_TIME);
                    //setMoveFinger(1,1,bit_int_pc*INT_TIME);

                }
                if(tmTimeCnt == music_offset_2018 + bit_int_pc*6+45) //8
                {
                    pyeongchangdemo_num = 2021;
                    tmTimeCnt = 0;
                    music_offset_2018 = 1;
                }

                break;
            }
        case 2021:
        {
            if(tmTimeCnt == music_offset_2018 + bit_int_pc*0)
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

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY], -40.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
//                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY], 8.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWY], 0.f, (bit_int_pc-bit_int_pc_offset)*INT_TIME, MODE_ABSOLUTE);
//                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWY], 0.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);


            }
            if(tmTimeCnt == music_offset_2018 + bit_int_pc*1)
            {

                //wbPosPelZ->setTaskPos(-z_101, bit_int_pc*INT_TIME, MODE_RELATIVE);
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
            if(tmTimeCnt == music_offset_2018 + bit_int_pc*2)
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

            if(tmTimeCnt == music_offset_2018 + bit_int_pc*3)
            {

                wbPosPelZ->setTaskPos(-z_101, 2*bit_int_pc*INT_TIME, MODE_RELATIVE);
                //wbPosPelZ->setTaskPos(z_101, bit_int_pc*INT_TIME, MODE_RELATIVE);
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
            if(tmTimeCnt == music_offset_2018 + bit_int_pc*4)
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

            if(tmTimeCnt == music_offset_2018 + bit_int_pc*5+30)
            {
                pyeongchangdemo_num = 2022;
                tmTimeCnt = 0;
                music_offset_2018 = 1;
                bit_int_pc = bit_int_pc*2;
                bit_int_pc_offset =bit_int_pc_offset*4;
                motion_loop_cnt = 0;
            }

            break;
        }
//1-3  2-2  1-4  3-2
        case 2022:
            {
                if(tmTimeCnt == bit_int_pc*0 + music_offset_2018)
                {
                    printf(">>> 1-3 Start \n");

                    wbPosPelZ->setTaskPos(z_101, bit_int_pc*INT_TIME, MODE_RELATIVE);

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

                if(tmTimeCnt == bit_int_pc*1 + music_offset_2018)
                {
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
                if(tmTimeCnt == bit_int_pc*2 + music_offset_2018 && motion_loop_cnt < 6)
                {
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


                    if(++motion_loop_cnt < 7) tmTimeCnt = music_offset_2018; //6

                }
                /*
                if(tmTimeCnt == bit_int_pc*2 + music_offset_2018 && motion_loop_cnt == 6)
                {
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



                }
                */
                if(tmTimeCnt == music_offset_2018 + bit_int_pc*2 && motion_loop_cnt == 6)
                {


                    wbPosPelZ->setTaskPos(z_101, bit_int_pc*INT_TIME, MODE_RELATIVE);
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

                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWY], -90.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWY], 90.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);

                    bit_int_pc = bit_int_pc/2;

                    setMoveFinger(2,1, 400);


                }
                if(tmTimeCnt == music_offset_2018 + bit_int_pc*6+0) //8
                {
                    pyeongchangdemo_num = 2023;
                    tmTimeCnt = 0;
                    music_offset_2018 = 1;
                }

                break;
            }
        case 2023:
        {

            if(tmTimeCnt == bit_int_pc*0 + music_offset_2018)
            {
                printf(">>> 2-2 Start \n");
                //setMoveFinger(0,-1,bit_int_pc*INT_TIME);
                //setMoveFinger(1,-1,bit_int_pc*INT_TIME);

            }

            if(tmTimeCnt == bit_int_pc*0 + music_offset_2018)
            {
                wbPosPelZ->setTaskPos(-z_101, bit_int_pc*INT_TIME, MODE_RELATIVE);

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

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWY], -90.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWY], 90.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);


            }
            if(tmTimeCnt == bit_int_pc*1 + music_offset_2018)
            {
                wbPosPelZ->setTaskPos(z_101, bit_int_pc*INT_TIME, MODE_RELATIVE);


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

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWY], -90.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWY], 90.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);

            }

            if(tmTimeCnt == bit_int_pc*2 + music_offset_2018)
            {
                wbPosPelZ->setTaskPos(-z_101, bit_int_pc*INT_TIME, MODE_RELATIVE);

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

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWY], -90.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWY], 90.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);


            }
            if(tmTimeCnt == bit_int_pc*3 + music_offset_2018)
            {
                setMoveFinger(2,-1, 400);
            }

            if(tmTimeCnt == bit_int_pc*3 + music_offset_2018)
            {

                //wbPosPelZ->setTaskPos(z_101, bit_int_pc*INT_TIME, MODE_RELATIVE);


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

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWY], -0.f, (bit_int_pc*4)*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWY], 0.f, (bit_int_pc*4)*INT_TIME, MODE_ABSOLUTE);


            }




            if(tmTimeCnt == music_offset_2018 + bit_int_pc*4)
            {
                pyeongchangdemo_num = 2024;
                tmTimeCnt = 0;
                music_offset_2018 = 1;
                bit_int_pc = bit_int_pc*2;
                motion_loop_cnt = 0;
            }

            break;
            }
        case 2024:
            {
                if(tmTimeCnt == bit_int_pc*0 + music_offset_2018)
                {
                    printf(">>> 1-4 Start \n");

                    wbPosPelZ->setTaskPos(z_101, bit_int_pc*INT_TIME, MODE_RELATIVE);

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

                if(tmTimeCnt == bit_int_pc*1 + music_offset_2018)
                {
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
                if(tmTimeCnt == bit_int_pc*2 + music_offset_2018 && motion_loop_cnt < 4)
                {
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

                    if(++motion_loop_cnt < 5) tmTimeCnt = music_offset_2018; //6

                }
                if(tmTimeCnt == music_offset_2018 + bit_int_pc*2  && motion_loop_cnt == 4)
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

                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY], -60.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
    //                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY], 8.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);

                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWY], 0.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
    //                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWY], 0.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);

                    bit_int_pc = bit_int_pc/2;
                    bit_int_pc_offset = bit_int_pc_offset/4;

                }
                if(tmTimeCnt == music_offset_2018 + bit_int_pc*6+60) //8
                {
                    pyeongchangdemo_num = 2025;
                    tmTimeCnt = 0;
                    music_offset_2018 = 1;
                }

                break;
            }
        case 2025:
        {
            if(tmTimeCnt == music_offset_2018 + bit_int_pc*0)
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
            if(tmTimeCnt == music_offset_2018 + bit_int_pc*1)
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
            if(tmTimeCnt == music_offset_2018 + bit_int_pc*2)
            {

                //wbPosPelZ->setTaskPos(-z_101, bit_int_pc*INT_TIME, MODE_RELATIVE);

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
            if(tmTimeCnt == music_offset_2018 + bit_int_pc*3)
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
            if(tmTimeCnt == music_offset_2018 + bit_int_pc*4)
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

            if(tmTimeCnt == music_offset_2018 + bit_int_pc*5+30)
            {
                pyeongchangdemo_num = 2026;
                tmTimeCnt = 0;
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
                if(tmTimeCnt == bit_int_pc*0 + music_offset_2018)
                {
                    printf(">>> 1-5 Start \n");

                    wbPosPelZ->setTaskPos(z_101, bit_int_pc*INT_TIME, MODE_RELATIVE);
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

                if(tmTimeCnt == bit_int_pc*1 + music_offset_2018)
                {
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
                if(tmTimeCnt == bit_int_pc*2 + music_offset_2018 && motion_loop_cnt < 6)
                {
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


                    if(++motion_loop_cnt < 7) tmTimeCnt = music_offset_2018; //6

                }
                /*
                if(tmTimeCnt == bit_int_pc*2 + music_offset_2018 && motion_loop_cnt == 6)
                {
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

                }
                */
                if(tmTimeCnt == music_offset_2018 + bit_int_pc*2 && motion_loop_cnt == 6)
                {


                    wbPosPelZ->setTaskPos(z_101, bit_int_pc*INT_TIME, MODE_RELATIVE);
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

                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWY], -90.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWY], 90.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);

                    bit_int_pc = bit_int_pc/2;

                    setMoveFinger(2,1, 400);


                }
                if(tmTimeCnt == music_offset_2018 + bit_int_pc*6) //8
                {
                    pyeongchangdemo_num = 2027;
                    tmTimeCnt = 0;
                    music_offset_2018 = 1;
                }

                break;
            }
        case 2027:
        {

            if(tmTimeCnt == bit_int_pc*0 + music_offset_2018)
            {
                printf(">>> 2-3 Start \n");
                //setMoveFinger(0,-1,bit_int_pc*INT_TIME);
                //setMoveFinger(1,-1,bit_int_pc*INT_TIME);

            }

            if(tmTimeCnt == bit_int_pc*0 + music_offset_2018)
            {
                wbPosPelZ->setTaskPos(-z_101, bit_int_pc*INT_TIME, MODE_RELATIVE);

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

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWY], -90.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWY], 90.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);


            }

            if(tmTimeCnt == bit_int_pc*1 + music_offset_2018)
            {

                wbPosPelZ->setTaskPos(z_101, bit_int_pc*INT_TIME, MODE_RELATIVE);


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

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWY], -90.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWY], 90.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);

            }


            if(tmTimeCnt == bit_int_pc*2 + music_offset_2018)
            {
                wbPosPelZ->setTaskPos(-z_101, bit_int_pc*INT_TIME, MODE_RELATIVE);

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

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWY], -90.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWY], 90.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);


            }
            if(tmTimeCnt == bit_int_pc*3 + music_offset_2018)
            {
                setMoveFinger(2,-1, 400);
            }

            if(tmTimeCnt == bit_int_pc*3 + music_offset_2018)
            {
                //wbPosPelZ->setTaskPos(z_101, bit_int_pc*INT_TIME, MODE_RELATIVE);

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

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWY], -0.f, (bit_int_pc*4)*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWY], 0.f, (bit_int_pc*4)*INT_TIME, MODE_ABSOLUTE);

            }




            if(tmTimeCnt == music_offset_2018 + bit_int_pc*4)
            {
                pyeongchangdemo_num = 2028;
                tmTimeCnt = 0;
                music_offset_2018 = 1;
                bit_int_pc = bit_int_pc*2;
                motion_loop_cnt = 0;
            }

            break;
            }
        case 2028:
            {
                if(tmTimeCnt == bit_int_pc*0 + music_offset_2018)
                {
                    printf(">>> 1-6 Start \n");

                    wbPosPelZ->setTaskPos(z_101, bit_int_pc*INT_TIME, MODE_RELATIVE);


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

                if(tmTimeCnt == bit_int_pc*1 + music_offset_2018)
                {
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
                if(tmTimeCnt == bit_int_pc*2 + music_offset_2018 && motion_loop_cnt < 4)
                {
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

                    if(++motion_loop_cnt < 5) tmTimeCnt = music_offset_2018; //6

                }
                if(tmTimeCnt == music_offset_2018 + bit_int_pc*2  && motion_loop_cnt == 4)
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

                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY], -60.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
    //                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY], 8.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);

                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWY], 0.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
    //                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWY], 0.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);

                    bit_int_pc = bit_int_pc/2;
                    bit_int_pc_offset = bit_int_pc_offset/4;

                }
                if(tmTimeCnt == music_offset_2018 + bit_int_pc*6+60) //8
                {
                    pyeongchangdemo_num = 2029;
                    tmTimeCnt = 0;
                    music_offset_2018 = 1;
                }

                break;
            }
        case 2029:
        {
            if(tmTimeCnt == music_offset_2018 + bit_int_pc*0)
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
            if(tmTimeCnt == music_offset_2018 + bit_int_pc*1)
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
            if(tmTimeCnt == music_offset_2018 + bit_int_pc*2)
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


            if(tmTimeCnt == music_offset_2018 + bit_int_pc*3)
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
            if(tmTimeCnt == music_offset_2018 + bit_int_pc*4)
            {

                //wbPosPelZ->setTaskPos(z_101, bit_int_pc*INT_TIME, MODE_RELATIVE);
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
            if(tmTimeCnt == music_offset_2018 + bit_int_pc*5+30)
            {
                pyeongchangdemo_num = 2030;
                tmTimeCnt = 0;
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
                if(tmTimeCnt == bit_int_pc*0 + music_offset_2018)
                {
                    printf(">>> 1-7 Start \n");

                    wbPosPelZ->setTaskPos(z_101, bit_int_pc*INT_TIME, MODE_RELATIVE);


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

                if(tmTimeCnt == bit_int_pc*1 + music_offset_2018)
                {
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
                if(tmTimeCnt == bit_int_pc*2 + music_offset_2018 && motion_loop_cnt < 6)
                {
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


                    if(++motion_loop_cnt < 7) tmTimeCnt = music_offset_2018; //6

                }
                if(tmTimeCnt == music_offset_2018 + bit_int_pc*2 && motion_loop_cnt == 6)
                {


                    wbPosPelZ->setTaskPos(z_101, bit_int_pc*INT_TIME, MODE_RELATIVE);
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

                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWY], -90.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWY], 90.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);

                    bit_int_pc = bit_int_pc/2;

                    setMoveFinger(2,1, 400);


                }
                if(tmTimeCnt == music_offset_2018 + bit_int_pc*6) //8
                {
                    pyeongchangdemo_num = 2031;
                    tmTimeCnt = 0;
                    music_offset_2018 = 1;
                }

                break;
            }
        case 2031:
        {

            if(tmTimeCnt == bit_int_pc*0 + music_offset_2018)
            {
                printf(">>> 2-4 Start \n");
                //setMoveFinger(0,-1,bit_int_pc*INT_TIME);
                //setMoveFinger(1,-1,bit_int_pc*INT_TIME);

            }

            if(tmTimeCnt == bit_int_pc*0 + music_offset_2018)
            {
                wbPosPelZ->setTaskPos(-z_101, bit_int_pc*INT_TIME, MODE_RELATIVE);

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

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWY], -90.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWY], 90.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);


            }

            if(tmTimeCnt == bit_int_pc*1 + music_offset_2018)
            {

                printf(">>> 2023 Hello_R Motion4 \n");
                wbPosPelZ->setTaskPos(z_101, bit_int_pc*INT_TIME, MODE_RELATIVE);

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

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWY], -90.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWY], 90.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);

            }

            if(tmTimeCnt == bit_int_pc*2 + music_offset_2018)
            {
                wbPosPelZ->setTaskPos(-z_101, bit_int_pc*INT_TIME, MODE_RELATIVE);

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

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWY], -90.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWY], 90.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);


            }
            if(tmTimeCnt == bit_int_pc*3 + music_offset_2018)
            {
                setMoveFinger(2,-1, 400);
            }

            if(tmTimeCnt == bit_int_pc*3 + music_offset_2018)
            {
                //wbPosPelZ->setTaskPos(z_101, bit_int_pc*INT_TIME, MODE_RELATIVE);

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

                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWY], -0.f, (bit_int_pc*4)*INT_TIME, MODE_ABSOLUTE);
                wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWY], 0.f, (bit_int_pc*4)*INT_TIME, MODE_ABSOLUTE);


            }




            if(tmTimeCnt == music_offset_2018 + bit_int_pc*4)
            {
                pyeongchangdemo_num = 2032;
                tmTimeCnt = 0;
                music_offset_2018 = 1;
                bit_int_pc = bit_int_pc*2;
                motion_loop_cnt = 0;
            }

            break;
            }
        case 2032:
            {
                if(tmTimeCnt == bit_int_pc*0 + music_offset_2018)
                {
                    printf(">>> 1-8 Start \n");

                    wbPosPelZ->setTaskPos(z_101, bit_int_pc*INT_TIME, MODE_RELATIVE);


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

                if(tmTimeCnt == bit_int_pc*1 + music_offset_2018)
                {
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
                if(tmTimeCnt == bit_int_pc*2 + music_offset_2018 && motion_loop_cnt < 4)
                {
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

                    if(++motion_loop_cnt < 5) tmTimeCnt = music_offset_2018; //6

                }
                if(tmTimeCnt == music_offset_2018 + bit_int_pc*2  && motion_loop_cnt == 4)
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

                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LSY], -60.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
    //                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RSY], 8.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);

                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[LWY], 0.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);
    //                    wbUBJoint->setMoveJointAngle(wbUBJoint->motionJoint[RWY], 0.f, bit_int_pc*INT_TIME, MODE_ABSOLUTE);

                    bit_int_pc = bit_int_pc/2;
                    bit_int_pc_offset = bit_int_pc_offset/4;

                }
                if(tmTimeCnt == music_offset_2018 + bit_int_pc*6+60) //8
                {
                    pyeongchangdemo_num = 2033;
                    tmTimeCnt = 0;
                    music_offset_2018 = 1;
                }

                break;
            }
        case 2033:
        {
            if(tmTimeCnt == music_offset_2018 + bit_int_pc*0)
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
            if(tmTimeCnt == music_offset_2018 + bit_int_pc*1)
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
            if(tmTimeCnt == music_offset_2018 + bit_int_pc*2)
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

            if(tmTimeCnt == music_offset_2018 + bit_int_pc*3)
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
            if(tmTimeCnt == music_offset_2018 + bit_int_pc*4)
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

            if(tmTimeCnt == music_offset_2018 + bit_int_pc*5+30)
            {
                printf(">>> Dance Motion end \n");
                pyeongchangdemo_num = 2034;
                tmTimeCnt = 0;
                bit_int_pc = bit_int_pc*2;
                bit_int_pc_offset =bit_int_pc_offset*4;
                music_offset_2018 = 1;
                motion_loop_cnt = 0;
                sciMotionSelector = NO_ACT;
            }

            break;
        }

        }break;

    default:
        break;
    }tmTimeCnt++;
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


unsigned int RBsetFrictionParameter(unsigned int _canch, unsigned int _bno,
                                    int _vel_saturation1, int _amp_compen1, int _vel_saturation2, int _amp_compen2) // inhyeok
{
    // MsgID		Byte0	Byte1	Byte2				Byte3				Byte4						Byte5				Byte6				Byte7
    // CMD_TXDF		BNO		0xB0	VEL_Saturation1		VEL_Saturation1		Ampere_Compensation1(mA)	VEL_Saturation2		VEL_Saturation2		Ampere_Compensation2(mA)

    MANUAL_CAN	MCData;

    MCData.channel = _canch;
    MCData.id = COMMAND_CANID;
    MCData.dlc = 8;
    MCData.data[0] = _bno;							// board no.
    MCData.data[1] = 0xB0;							// command
    MCData.data[2] = _vel_saturation1 & 0xFF;
    MCData.data[3] = (_vel_saturation1>>8) & 0xFF;
    MCData.data[4] = _amp_compen1 & 0xFF;
    MCData.data[5] = _vel_saturation2 & 0xFF;
    MCData.data[6] = (_vel_saturation2>>8) & 0xFF;
    MCData.data[7] = _amp_compen2 & 0xFF;

    return PushCANMessage(MCData);
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
    MCJointGainOverride(JOINT_INFO[RSP].canch, JOINT_INFO[RSP].bno, JOINT_INFO[RSP].mch, 50, 10);
    usleep(5000);
    MCsetFrictionParameter(JOINT_INFO[RSR].canch, JOINT_INFO[RSR].bno, JOINT_INFO[RSR].mch, 1000, 7, 0);
    MCBoardSetSwitchingMode(JOINT_INFO[RSR].canch, JOINT_INFO[RSR].bno, SW_MODE_NON_COMPLEMENTARY);
    MCenableFrictionCompensation(JOINT_INFO[RSR].canch, JOINT_INFO[RSR].bno, JOINT_INFO[RSR].mch, ENABLE);
    MCJointGainOverride(JOINT_INFO[RSR].canch, JOINT_INFO[RSR].bno, JOINT_INFO[RSR].mch, 50, 10);
    usleep(5000);
    MCsetFrictionParameter(JOINT_INFO[RSY].canch, JOINT_INFO[RSY].bno, JOINT_INFO[RSY].mch, 1000, 9, 0);
    MCBoardSetSwitchingMode(JOINT_INFO[RSY].canch, JOINT_INFO[RSY].bno, SW_MODE_NON_COMPLEMENTARY);
    MCenableFrictionCompensation(JOINT_INFO[RSY].canch, JOINT_INFO[RSY].bno, JOINT_INFO[RSY].mch, ENABLE);
    MCJointGainOverride(JOINT_INFO[RSY].canch, JOINT_INFO[RSY].bno, JOINT_INFO[RSY].mch, 50, 10);
    usleep(5000);
    MCsetFrictionParameter(JOINT_INFO[REB].canch, JOINT_INFO[REB].bno, JOINT_INFO[REB].mch, 1000, 5, 0);
    MCBoardSetSwitchingMode(JOINT_INFO[REB].canch, JOINT_INFO[REB].bno, SW_MODE_NON_COMPLEMENTARY);
    MCenableFrictionCompensation(JOINT_INFO[REB].canch, JOINT_INFO[REB].bno, JOINT_INFO[REB].mch, ENABLE);
    MCJointGainOverride(JOINT_INFO[REB].canch, JOINT_INFO[REB].bno, JOINT_INFO[REB].mch, 50, 10);
    usleep(5000);

    MCJointGainOverride_old(JOINT_INFO[RWY].canch, JOINT_INFO[RWY].bno, 1, 1, 10);
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
