#ifndef TASKJOINT_H
#define TASKJOINT_H


#include "taskGeneral.h"
#include "jointvariable.h"

class TaskJoint
{
public:
    explicit TaskJoint();
    ~TaskJoint();

public:
    JointVariable*  motionJoint[NO_OF_JOINTS];

    void refreshToCurrentReference(void);

    unsigned char setMoveJointAngle(JointVariable *_joint, float _angle, float _msTime, unsigned int _mode);
    unsigned char moveJointAngle(JointVariable *_joint);
    unsigned char moveFinger(JointVariable *_joint);
    void jointUpdate(int _motionNum = -1);
};

#endif // TASKJOINT_H
