#ifndef TASKMOTION_H
#define TASKMOTION_H


#include "TaskGeneral.h"
#include "BasicTrajectory.h"

#include "BasicMatrix.h"
#include "BasicJoint.h"

#include "../../share/Headers/kine_drc_hubo4.h"
#include "../../share/Headers/kine_drc_hubo2.h"



#define RING_SIZE                400
#define SCI_DEMO                101
#define SCI_DEMO_Control_on                102
#define SCI_DEMO_Control_off               103
#define SCI_DEMO_DATA_SAVE               104


enum PYEONGCHANG_MOTION_ENUM
{
    HANDSHAKE_READY = 10010,
    HANDSHAKE_LOWGAIN,
    HANDSHAKE_GRIP_ON,
    HANDSHAKE_GRIP_OFF,
    HANDSHAKE_HIGHGAIN,
    HANDSHAKE_INIT,
    NO_OF_HANDSHAKE_MOTION,
    BOUQUET_READY = 10020,
    BOUQUET_GIVE,
    BOUQUET_INIT,
    NO_OF_BOUQUET_MOTION,
    CLAP_READY = 10030,
    CLAP_INIT,
    NO_OF_CLAP_MOTION,
    HEAD_YES = 10040,
    HEAD_NO,
    HEAD_STRETCHING,
    HEAD_INIT,
    NO_OF_HEAD_MOTION,
    BOW_ONE = 10050,
    BOW_TWO,
    BOW_TWO_W,
    BOW_GREETING,
    BOW_LAST_MOTION,
    NO_OF_BOW_MOTION,
    GUID_LEFT = 10060,
    GUID_RIGHT,
    GUID_INIT,
    HIGHFIVE_DO,
    HIGHFIVE_INIT,
    DANCE_GOGOGO_READY = 12017,
    DANCE_GOGOGO,
    WST_TWIST,
    WST_ZERO,
    DEMO_SCENE_1,
    DEMO_SCENE_2,
    DEMO_SCENE_3,
    DEMO_SCENE_4,
    NO_OF_PYEONGCHANG_MOTION
};

using namespace rainbow;


class TaskPos;
class TaskOri;
class TaskJoint;
class JointVariable
{
public:
    explicit JointVariable(){
        RefAngleCurrent = 0.f;
        RefAngleDelta = 0.f;
        RefAngleToGo = 0.f;
        RefAngleInitial = 0.f;
        GoalTimeCount = 0;
        CurrentTimeCount = 0;
        MoveFlag = DISABLE;
    }
    ~JointVariable();

public:
    float                       RefAngleCurrent;        // reference to move at this step
    float                       RefAngleDelta;            // reference of the past step
    float                       RefAngleToGo;             // goal position - initial position
    float                       RefAngleInitial;            // initial position
    unsigned long        GoalTimeCount;          // the time at which the goal is reached
    unsigned long        CurrentTimeCount;	  // current time count
    char                        MoveFlag;                   // move flag
};



class TaskMotion
{
    enum TASK_SPACE_ENUM
    {
        TASK_RF = 0,
        TASK_LF,
        TASK_RH,
        TASK_LH,
        TASK_PEL,
        TASK_COM,
        TASK_WST,
        NO_OF_MOTION
    };

public:
    explicit TaskMotion(pRBCORE_SHM _shm, JointControlClass *_joint);
    ~TaskMotion();

private:
    pRBCORE_SHM         Shm;
    JointControlClass   *Joint;


public:
    CKINE_DRC_HUBO2 kine_drc;

    //--------------------------------------------------------------------------------//
    TaskPos*   wbPosRF[3];
    TaskPos*   wbPosLF[3];
    TaskPos*   wbPosRH[3];
    TaskPos*   wbPosLH[3];
    TaskPos*   wbPosWST;
    TaskPos*   wbPosCOM[2];
    TaskPos*   wbPosPelZ;
    TaskPos*   wbPosELB[2];
    //TRHandler   wbPosBPitch;
    //--------------------------------------------------------------------------------//
    TaskOri*   wbOriRF;
    TaskOri*   wbOriLF;
    TaskOri*   wbOriRH;
    TaskOri*   wbOriLH;
    TaskOri*   wbOriPel;

    //--------------------------------------------------------------------------------//
    TaskJoint*  wbUBJoint;

    int         Arm_Joint_mode_flag;


    unsigned char   TaskMotionSelector;
    unsigned char   sciMotionSelector;
    long   sciTimeCnt;

    int MoveFinger_Flag;

    float P_gain_yaw_neck;
    float P_gain_pitch_neck;
    int posX_dff;
    int posY_dff;
    float Pos_Neck_Yaw;
    float Pos_Neck_Pitch;



//    TRHandler   temp
    //--------------------------------------------------------------------------------//

    double      Qin_34x1[34];
    double      Qout_34x1[34];
    double      Qub_34x1[34];
    double      Q_filt_34x1[34], Qd_filt_34x1[34], Qdd_filt_34x1[34];
    double      pCOM_2x1[2];
    double      rWST;
    double      pPelZ;
    double      pRF_3x1[3];
    double      pLF_3x1[3];
    double      pRH_3x1[3];
    double      pLH_3x1[3];

    double      qRF_4x1[4];
    double      qLF_4x1[4];
    double      qRH_4x1[4];
    double      qLH_4x1[4];
    double      qPEL_4x1[4];

    double      RElb_ang;
    double      LElb_ang;


    double      des_pCOM_2x1[2];
    double      des_pPELz;
    double      des_rWST;
    double      des_pRF_3x1[3];
    double      des_pLF_3x1[3];
    double      des_pRH_3x1[3];
    double      des_pLH_3x1[3];

    double      des_qRF_4x1[4];
    double      des_qLF_4x1[4];
    double      des_qRH_4x1[4];
    double      des_qLH_4x1[4];
    double      des_qPEL_4x1[4];

    double      des_RElb_ang;
    double      des_LElb_ang;


    char filename[100];


    unsigned char setMoveCOM(double _xCOM, double _yCOM, double _msTime, unsigned char _mode);
    unsigned char setMoveLeg(char _selectLeg, double _xLEG, double _yLEG, double _zLEG, double _ori[], double _msTime, unsigned char _mode);
    unsigned char setMoveHand(char _selectHand, double _xHAND, double _yHAND, double _zHAND, double _ori[], double _msTime, unsigned char _mode);
    void setMoveFinger(char _selectFinger, int _direction, float _msTime);

    void moveTaskPosOri(int _taskSpace);


    void    ResetGlobalCoord(int RF_OR_LF_OR_PC);
    void    RefreshToCurrentReference();

    void taskUpdate();

    void WBIK();

    double limit_Qd(double Qd, double Qdd, double Qd_max, double dt);


    void Setmove_walk_ready(float _MotionTime);
    void initHeadHand(int select);
    void LeadTextData(void);
    void ChaseTextData(void);


    int     clap_cnt;

    void sciDemo();



};



// ************************************************************************************ //

class TaskPos
{
public:
    explicit TaskPos(){
        RefCurrent = 0.f;
        RefDelta = 0.f;
        RefToGo = 0.f;
        RefInitial = 0.f;
        GoalTimeCount = 0;
        CurrentTimeCount = 0;
        MoveFlag = DISABLE;
    }

    ~TaskPos();

public:
    float                   RefCurrent;                 // reference to move at this step
    float                   RefDelta;                    // reference of the past step
    float                   RefToGo;                    // goal position - initial position
    float                   RefInitial;                   // initial position
    unsigned long           GoalTimeCount;        // the time at which the goal is reached
    unsigned long           CurrentTimeCount;   // current time count
    char                    MoveFlag;                 // move flag

public:
    unsigned char   setTaskPos(float _mag, unsigned long _msTime, unsigned char _mode){
        if(_msTime <= 0) { printf(">>> Goal time must be grater than zero..!!(setTaskPos)\n"); return ERR_GOAL_TIME; }
        if(MoveFlag == ENABLE){
            printf(">>> Moving..!!(setTaskPos)\n");
            return ERR_ALREADY_MOVING;
        }else{
            switch(_mode)
            {
            case MODE_RELATIVE:	// relative mode
                RefToGo = RefCurrent + _mag;
                break;
            case MODE_ABSOLUTE:	// absolute mode
                RefToGo = _mag;
                break;
            default:
                printf(">>> Wrong reference mode..!!(setTaskPos)\n");
                return ERR_WRONG_MODE;
                break;
            }

            RefInitial = RefCurrent;
            CurrentTimeCount = 0;
            GoalTimeCount = ((unsigned long)(_msTime))/RT_TIMER_PERIOD_MS;
            RefDelta = RefToGo - RefCurrent;

            MoveFlag = ENABLE;
        }
        return ERR_OK;
    }

    unsigned char   moveTaskPos(void){
        float tempTime;

        if(MoveFlag == ENABLE){
            CurrentTimeCount++;
            if(GoalTimeCount <= CurrentTimeCount){
                GoalTimeCount = CurrentTimeCount = 0;
                RefCurrent = RefToGo;
                MoveFlag = DISABLE;
                return MOVE_DONE;
            }else{
                tempTime = (float)CurrentTimeCount/(float)GoalTimeCount;
                RefCurrent = RefInitial+RefDelta*0.5f*(1.0f-cosf(PI*tempTime));
            }
        }
        return STILL_MOVING;
    }
};


// ************************************************************************************ //



class TaskOri
{
public:
    explicit TaskOri(){
        for(int i=0; i<4; i++){
            RefCurrent[i] = 0.f;
            RefDelta[i] = 0.f;
            RefDeltaQT[i] = 0.f;
            RefdqQT[i] = 0.f;
            UserInput[i] = 0.f;
            RefInitial[i] = 0.f;
            RefInitialInv[i] = 0.f;
        }
        for(int i=0; i<4; i++){
            RefDeltaRV[i] = 0.f;
            RefdqRV[i] = 0.f;
            TempToGo[i] = 0.f;
        }
        GoalTimeCount = 0;
        CurrentTimeCount = 0;
        MoveFlag = DISABLE;
    }

    ~TaskOri();

public:
    double                   RefCurrent[4];              // reference pattern to move at this step
    double                   RefDelta[4];                  // reference pattern of the past step
    double                   RefDeltaQT[4];
    double                   UserInput[4];
    double                   RefInitial[4];                  // initial position
    double                   RefInitialInv[4];
    double                   RefdqQT[4];

    double                   RefDeltaRV[4];
    double                   RefdqRV[4];
    double                   TempToGo[4];

    unsigned long	  GoalTimeCount;            // the time at which the goal is reached
    unsigned long	  CurrentTimeCount;       // current time count
    unsigned char	  MoveFlag;                     // move flag

public:
    unsigned char   setTaskOri(double _mag[4], double _msTime, unsigned char _mode){
        if(_msTime <= 0) { printf(">>> Goal time must be grater than zero..!!(setTaskOri)\n"); return ERR_GOAL_TIME; }

        if(MoveFlag == ENABLE){
            printf(">>> Moving..!!(setTaskOri)\n");
            return ERR_ALREADY_MOVING;
        }else{
            MoveFlag = DISABLE;

            TempToGo[0] = _mag[0];
            TempToGo[1] = _mag[1];
            TempToGo[2] = _mag[2];
            TempToGo[3] = _mag[3];

            RefInitial[0] = RefCurrent[0];
            RefInitial[1] = RefCurrent[1];
            RefInitial[2] = RefCurrent[2];
            RefInitial[3] = RefCurrent[3];

            switch(_mode)
            {
            case MODE_RELATIVE: //relative mode
                RV2QT(TempToGo, UserInput);
                RefDeltaQT[0] = UserInput[0];
                RefDeltaQT[1] = UserInput[1];
                RefDeltaQT[2] = UserInput[2];
                RefDeltaQT[3] = UserInput[3];
                break;
            case MODE_ABSOLUTE: //absolute mode
                RV2QT(TempToGo, UserInput);
                iQTinv(RefInitial, RefInitialInv);
                QTcross(RefInitialInv, UserInput, RefDeltaQT);
                break;
            default:
                printf(">>> Wrong reference mode..!!(setTaskOri)\n");
                return ERR_WRONG_MODE;
                break;
            }

            CurrentTimeCount = 0;
            GoalTimeCount = ((unsigned long)(_msTime))/RT_TIMER_PERIOD_MS;
            MoveFlag = ENABLE;
        }
        return ERR_OK;
    }
    unsigned char   moveTaskOri(void){
        float tempTime;

        if(MoveFlag == ENABLE){
            CurrentTimeCount++;
            if(GoalTimeCount <= CurrentTimeCount){
                GoalTimeCount = CurrentTimeCount = 0;
                QTcross(RefInitial, RefDeltaQT, RefCurrent);
                MoveFlag = DISABLE;
                return MOVE_DONE;
            }else{
                QT2RV(RefDeltaQT, RefDeltaRV);

                tempTime = (float)CurrentTimeCount/(float)GoalTimeCount;
                RefdqRV[0] = RefDeltaRV[0]*0.5f*(1.0f-cosf(PI*tempTime));
                RefdqRV[1] = RefDeltaRV[1];
                RefdqRV[2] = RefDeltaRV[2];
                RefdqRV[3] = RefDeltaRV[3];

                RV2QT(RefdqRV, RefdqQT);
                QTcross(RefInitial, RefdqQT, RefCurrent);
            }
        }
        return STILL_MOVING;
    }


    int iQTinv(const double *qt_4x1, double *result_4x1){
        result_4x1[0] = qt_4x1[0]/((qt_4x1[0])*(qt_4x1[0])+(qt_4x1[1])*(qt_4x1[1])+(qt_4x1[2])*(qt_4x1[2])+(qt_4x1[3])*(qt_4x1[3]));
        result_4x1[1] = -qt_4x1[1]/((qt_4x1[0])*(qt_4x1[0])+(qt_4x1[1])*(qt_4x1[1])+(qt_4x1[2])*(qt_4x1[2])+(qt_4x1[3])*(qt_4x1[3]));
        result_4x1[2] = -qt_4x1[2]/((qt_4x1[0])*(qt_4x1[0])+(qt_4x1[1])*(qt_4x1[1])+(qt_4x1[2])*(qt_4x1[2])+(qt_4x1[3])*(qt_4x1[3]));
        result_4x1[3] = -qt_4x1[3]/((qt_4x1[0])*(qt_4x1[0])+(qt_4x1[1])*(qt_4x1[1])+(qt_4x1[2])*(qt_4x1[2])+(qt_4x1[3])*(qt_4x1[3]));

        return 0;
    }
//    int iQT2RV(const double *qt_4x1, double *rv){
//        double EPS (2.e-6);

//        double temp;
//        rv[0] = acosf(qt_4x1[0])*2.f;

//        if(fabs(sinf(rv[0]/2.f)) < EPS){
//            rv[1] = qt_4x1[1];
//            rv[2] = qt_4x1[2];
//            rv[3] = qt_4x1[3];
//        }else{
//            rv[1] = qt_4x1[1]/sinf(rv[0]/2.f);
//            rv[2] = qt_4x1[2]/sinf(rv[0]/2.f);
//            rv[3] = qt_4x1[3]/sinf(rv[0]/2.f);


//            temp = sqrt(rv[1]*rv[1]+rv[2]*rv[2]+rv[3]*rv[3]);
//            rv[1] /= temp;
//            rv[2] /= temp;
//            rv[3] /= temp;
//        }

//        return 0;
//    }
//    int iRV2QT(const double *rv, double *qt_4x1){
//        double temp = sqrtf(rv[1]*rv[1]+rv[2]*rv[2]+rv[3]*rv[3]);

//        if(temp > 0.5f){
//            qt_4x1[0] = cosf(rv[0]/2.f);
//            qt_4x1[1] = rv[1]/temp*sinf(rv[0]/2.f);
//            qt_4x1[2] = rv[2]/temp*sinf(rv[0]/2.f);
//            qt_4x1[3] = rv[3]/temp*sinf(rv[0]/2.f);
//        }else{
//            qt_4x1[0] = 1.f;
//            qt_4x1[1] = 0.f;
//            qt_4x1[2] = 0.f;
//            qt_4x1[3] = 0.f;
//        }

//        return 0;
//    }

};

// ************************************************************************************ //


class TaskJoint
{
public:
    explicit TaskJoint(){
        for(int i=0; i<NO_OF_JOINTS; i++)
            motionJoint[i] = new JointVariable();
    }

    ~TaskJoint();

public:
    JointVariable*  motionJoint[NO_OF_JOINTS];

    void refreshToCurrentReference(void){
        for(int i=0; i<NO_OF_JOINTS; i++){
            motionJoint[i]->RefAngleCurrent = sharedData->ENCODER[MC_GetID(i)][MC_GetCH(i)].CurrentReference;
            motionJoint[i]->MoveFlag = DISABLE;
        }
    }

    unsigned char setMoveJointAngle(JointVariable *_joint, float _angle, float _msTime, unsigned int _mode){
        if(_msTime <= 0) { printf(">>> Goal time must be grater than zero..!!(setMoveJointAngle\n"); return ERR_GOAL_TIME; }

        _joint->MoveFlag = DISABLE;
        switch(_mode)
        {
        case 0x00:	// relative mode
            _joint->RefAngleToGo = _joint->RefAngleCurrent + _angle;
            break;
        case 0x01:	// absolute mode
            _joint->RefAngleToGo = _angle;
            break;
        default:
            printf(">>> Wrong reference mode(RBsetMoveJointAngle)..!!\n");
            return ERR_WRONG_MODE;
            break;
        }
        _joint->RefAngleInitial = _joint->RefAngleCurrent;
        _joint->RefAngleDelta = _joint->RefAngleToGo - _joint->RefAngleCurrent;
        _joint->CurrentTimeCount = 0;

        _joint->GoalTimeCount = (unsigned long)(_msTime/RT_TIMER_PERIOD_MS);
        _joint->MoveFlag = ENABLE;
        return ERR_OK;
    }

    unsigned char moveJointAngle(JointVariable *_joint){
        // for reference generator
        if(_joint->MoveFlag == ENABLE){
            _joint->CurrentTimeCount++;
            if(_joint->GoalTimeCount <= _joint->CurrentTimeCount){
                _joint->GoalTimeCount = _joint->CurrentTimeCount = 0;
                _joint->RefAngleCurrent = _joint->RefAngleToGo;
                _joint->MoveFlag = DISABLE;
                return MOVE_DONE;
            }else{
                _joint->RefAngleCurrent = _joint->RefAngleInitial+_joint->RefAngleDelta*0.5f*(1.0f-cos(PI/(float)_joint->GoalTimeCount*(float)_joint->CurrentTimeCount));
            }
        }
        return STILL_MOVING;
    }

    unsigned char moveFinger(JointVariable *_joint){
        // for reference generator
        if(_joint->MoveFlag == ENABLE){
            _joint->CurrentTimeCount++;
            if(_joint->GoalTimeCount <= _joint->CurrentTimeCount)
            {
                _joint->GoalTimeCount = _joint->CurrentTimeCount = 0;
                _joint->RefAngleCurrent = 0;
                _joint->MoveFlag = DISABLE;
                return MOVE_DONE;
            }
            /*
            else if(_joint->GoalTimeCount/2 <= _joint->CurrentTimeCount)
            {
                if(_joint->RefAngleToGo > 0)
                    _joint->RefAngleCurrent = -127;
                else if(_joint->RefAngleToGo < 0)
                    _joint->RefAngleCurrent = 127;
            }
            else
            {
                if(_joint->RefAngleToGo > 0)
                    _joint->RefAngleCurrent = 127;
                else if(_joint->RefAngleToGo < 0)
                    _joint->RefAngleCurrent = -127;
            }
            */
            else{
                if(_joint->RefAngleToGo > 0.01)
                    _joint->RefAngleCurrent = 127;
                else if(_joint->RefAngleToGo < -0.01)
                    _joint->RefAngleCurrent = -127;
            }
        }
        return STILL_MOVING;
    }

    void jointUpdate(int _motionNum = -1){
//        for(int i=0; i<NO_OF_JOINTS; i++){
//            if(sharedData->MotionOwner[MC_GetID(i)][MC_GetCH(i)] == _motionNum){
//                moveFinger(motionJoint[i]);
//                moveJointAngle(motionJoint[i]);
//                sharedData->MotionJointRef[i] = motionJoint[i]->RefAngleCurrent;
//            }
//        }
    }
};




#endif // TASKMOTION_H
