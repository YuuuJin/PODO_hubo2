#include "taskjoint.h"


extern pHUBO2 Hubo2;


TaskJoint::TaskJoint()
{
    for(int i=0; i<NO_OF_JOINTS; i++)
        motionJoint[i] = new JointVariable();
}



void TaskJoint::refreshToCurrentReference(void)
{
    for(int i=0; i<NO_OF_JOINTS; i++)
    {
        motionJoint[i]->RefAngleCurrent = Hubo2->MotionJointRef[i];
        motionJoint[i]->MoveFlag = DISABLE;
    }
}


unsigned char TaskJoint::setMoveJointAngle(JointVariable *_joint, float _angle, float _msTime, unsigned int _mode)
{
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

    _joint->GoalTimeCount = (unsigned long)(_msTime/INT_TIME);
    _joint->MoveFlag = ENABLE;
    return ERR_OK;
}


unsigned char TaskJoint::moveJointAngle(JointVariable *_joint)
{
    // for reference generator
    if(_joint->MoveFlag == ENABLE)
    {
        _joint->CurrentTimeCount++;
        if(_joint->GoalTimeCount <= _joint->CurrentTimeCount)
        {
            _joint->GoalTimeCount = _joint->CurrentTimeCount = 0;
            _joint->RefAngleCurrent = _joint->RefAngleToGo;
            _joint->MoveFlag = DISABLE;
            return MOVE_DONE;
        }
        else
        {
            _joint->RefAngleCurrent = _joint->RefAngleInitial+_joint->RefAngleDelta*0.5f*(1.0f-cos(PI/(float)_joint->GoalTimeCount*(float)_joint->CurrentTimeCount));
        }
    }
    return STILL_MOVING;
}

unsigned char TaskJoint::moveFinger(JointVariable *_joint)
{
    // for reference generator
    if(_joint->MoveFlag == ENABLE)
    {
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
        else
        {
            if(_joint->RefAngleToGo > 0)
                _joint->RefAngleCurrent = 127;
            else if(_joint->RefAngleToGo < 0)
                _joint->RefAngleCurrent = -127;
        }
    }
    return STILL_MOVING;
}

void TaskJoint::jointUpdate(int _motionNum)
{
    for(int i=0; i<NO_OF_JOINTS; i++)
    {
        if(Hubo2->MotionOwned[i] == _motionNum)
        {
            moveFinger(motionJoint[i]);
            moveJointAngle(motionJoint[i]);
            Hubo2->MotionJointRef[i] = motionJoint[i]->RefAngleCurrent;
        }
    }
}
