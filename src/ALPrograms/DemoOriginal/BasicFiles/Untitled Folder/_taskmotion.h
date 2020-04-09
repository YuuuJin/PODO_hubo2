#ifndef TASKMOTION_H
#define TASKMOTION_H

#include "taskpos.h"
#include "taskori.h"
#include "../BasicFiles/BasicJoint.h"
#include "JointInformation.h"


#define __SAVE_WB__

class TaskMotion
{
public:
    explicit TaskMotion();
    ~TaskMotion();

public:
    unsigned char   TaskMotionSelector;

    //--------------------------------------------------------------------------------//
    TaskPos*    wbPosRF[4];
    TaskPos*    wbPosLF[4];
    TaskPos*    wbPosRH[4];
    TaskPos*    wbPosLH[4];
    TaskPos*    wbPosWST;
    TaskPos*    wbPosCOM[3];
    TaskPos*    wbPosPelZ;
    //--------------------------------------------------------------------------------//
    TaskOri*    wbOriRF;
    TaskOri*    wbOriLF;
    TaskOri*    wbOriRH;
    TaskOri*    wbOriLH;
    TaskOri*    wbOriPel;
    //--------------------------------------------------------------------------------//
    JointControlClass*  wbUBJoint;

    //--------------------------------------------------------------------------------//
    unsigned char   sciMotionSelector;
    unsigned long   sciTimeCnt;
    //--------------------------------------------------------------------------------//

    //--------------------------------------------------------------------------------//
    float           prePosREB;
    float           prePosLEB;
    float           prePosRSP;
    float           prePosLSP;
    //--------------------------------------------------------------------------------//
    char filename[100];
    //--------------------------------------------------------------------------------//
    int MoveFinger_Flag;

    float P_gain_yaw_neck;
    float P_gain_pitch_neck;
    int posX_dff;
    int posY_dff;
    float Pos_Neck_Yaw;
    float Pos_Neck_Pitch;

    unsigned char refreshToCurrentReference(void);
    unsigned char changeCoordAndInitialize(unsigned char _pRFflag, unsigned char _qRFflag, unsigned char _pLFflag, unsigned char _qLFflag, unsigned char _pRHflag, unsigned char _qRHflag, unsigned char _pLHflag, unsigned char _qLHflag);

    unsigned char setMoveCOM(float _xCOM, float _yCOM, float _msTime, unsigned char _mode);
    unsigned char setMoveLeg(char _selectLeg, float _xLEG, float _yLsEG, float _zLEG, float _ori[], float _msTime, unsigned char _mode);
    unsigned char setMoveHand(char _selectHand, float _xHAND, float _yHAND, float _zHAND, float _ori[], float _msTime, unsigned char _mode);

    void setMoveFinger(char _selectFinger, int _direction, float _msTime);
    void MoveFinger(char _selectFinger);
    void moveTaskPosOri(int _taskSpace);
    void taskUpdate(int _motionNum = -1);
    void LeadTextData(void);
    void ChaseTextData(void);
    void initHeadHand(int _select);
    void Setmove_walk_ready(float _MotionTime);


    void sciDemo(int _motionNumber, JointControlClass *_wbJoint);

    void Vision2Robot(float _vpos[4], float _rpos[4]);
};

#endif // TASKMOTION_H

