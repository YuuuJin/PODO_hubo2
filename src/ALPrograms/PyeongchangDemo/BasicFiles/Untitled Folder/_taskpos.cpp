#include "taskpos.h"

TaskPos::TaskPos()
{
    RefCurrent = 0.f;
    RefDelta = 0.f;
    RefToGo = 0.f;
    RefInitial = 0.f;
    GoalTimeCount = 0;
    CurrentTimeCount = 0;
    MoveFlag = DISABLE;
}


unsigned char TaskPos::setTaskPos(float _mag, unsigned long _msTime, unsigned char _mode)
{
    if(_msTime <= 0) { printf(">>> Goal time must be grater than zero..!!(setTaskPos)\n"); return ERR_GOAL_TIME; }
    if(MoveFlag == ENABLE)
    {
        printf(">>> Moving..!!(setTaskPos)\n");
        return ERR_ALREADY_MOVING;
    }
    else
    {
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


unsigned char TaskPos::moveTaskPos(void)
{
    float tempTime;

    if(MoveFlag == ENABLE)
    {
        CurrentTimeCount++;
        if(GoalTimeCount <= CurrentTimeCount)
        {
            GoalTimeCount = CurrentTimeCount = 0;
            RefCurrent = RefToGo;
            MoveFlag = DISABLE;
            return MOVE_DONE;
        }
        else
        {
            tempTime = (float)CurrentTimeCount/(float)GoalTimeCount;
            RefCurrent = RefInitial+RefDelta*0.5f*(1.0f-cosf(RBCORE_PI*tempTime));
        }
    }
    return STILL_MOVING;
}
