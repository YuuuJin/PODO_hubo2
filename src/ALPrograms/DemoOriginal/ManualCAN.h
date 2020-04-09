#ifndef MANUALCAN_H
#define MANUALCAN_H
#include "../../../share/Headers/RBSharedMemory.h"
#include "../../../share/Headers/JointInformation.h"

// Manual CAN for GainOverride
#define SW_MODE_COMPLEMENTARY           0x00
#define SW_MODE_NON_COMPLEMENTARY       0x01

const struct {
    int canch;          // CAN Channel
    int bno;            // Board Number
    int mch;            // Motor Channel
} JOINT_INFO[NO_OF_JOINTS] = {
    {0,0,1}, {0,0,2}, {0,1,1}, {0,2,1}, {0,3,1}, {0,3,2},       // RHY, RHR, RHP, RKN, RAP, RAR,
    {0,4,1}, {0,4,2}, {0,5,1}, {0,6,1}, {0,7,1}, {0,7,2},       // LHY, LHR, LHP, LKN, LAP, LAR,
    {1,9,1}, {1,9,2}, {1,10,1},{1,10,2},{1,32,1},{1,32,2},      // RSP, RSR, RSY, REB, RWY, RWP,
    {1,11,1},{1,11,2},{1,12,1},{1,12,2},{1,33,1},{1,33,2},      // LSP, LSR, LSY, LEB, LWY, LWP,
    {0,8,1},                                                    // WST,
    {1,34,1},{1,34,2},{1,34,3},                                 // NKY, NK1, NK2,
    {1,36,1},{1,36,2},{1,36,3},{1,36,4},{1,36,5},               // RF1~RF5
    {1,37,1},{1,37,2},{1,37,3},{1,37,4},{1,37,5}                // RF1~RF5
};


int	PushCANMessage(MANUAL_CAN MCData);
int MCJointGainOverride_old(unsigned int _canch, unsigned int _bno, int _gain1, int _gain2, int _msec);

int MCJointGainOverride(unsigned int _canch, unsigned int _bno, int _mch, int logscale, short _msec);
int MCBoardSetSwitchingMode(unsigned int _canch, unsigned int _bno, int _mode);
int MCenableFrictionCompensation(unsigned int _canch, unsigned int _bno, unsigned int _mch, unsigned int _enable);
int MCsetFrictionParameter(unsigned int _canch, unsigned int _bno,
                                    unsigned int _mch, short _vel_saturation, int _amp_compen, int _vel_dead);
int MCJointEnableFeedbackControl(unsigned int _canch, unsigned int _bno, unsigned int _mch, int _enable);
int MCJointSetControlMode(unsigned int _canch, unsigned int _bno, unsigned int _mch, int _mode);
int MCJointPWMCommand2chHR(unsigned int _canch, unsigned int _bno, int mode1, short duty1, int mode2, short duty2);
int MCWristFTsensorNull(unsigned int _canch, unsigned int _bno);
int MCJointRequestEncoderPosition(unsigned int _canch, unsigned _bno, int _mode);


//////////////OKKEE ADDED



int MCJointFindHome(unsigned int _canch, unsigned int _bno, int _mch);




//////////////FINDHOME
#endif // MANUALCAN_H
