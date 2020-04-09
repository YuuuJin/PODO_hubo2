#ifndef EXTERNHEADER_H
#define EXTERNHEADER_H



// Devices --------
#include "RBMotorController.h"
#include "RBFTSensor.h"
#include "RBIMUSensor.h"
#include "RBSmartPower.h"


extern RBMotorController   _DEV_MC[MAX_MC];
extern RBFTSensor          _DEV_FT[MAX_FT];
extern RBIMUSensor         _DEV_IMU[MAX_IMU];
extern RBSmartPower        _DEV_SP[MAX_SP];

extern int     _VERSION;
extern int     _NO_OF_AL;
extern int     _NO_OF_COMM_CH;
extern int     _NO_OF_MC;
extern int     _NO_OF_FT;
extern int     _NO_OF_IMU;
extern int     _NO_OF_SP;

#endif // EXTERNHEADER_H
