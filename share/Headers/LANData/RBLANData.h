#ifndef RBLANDATA_H
#define RBLANDATA_H

#include "../RBSharedMemory.h"
#include "../UserSharedMemory.h"

typedef struct __LAN_STRUCT_GUI2PODO__
{
    USER_COMMAND            UserCMD;
    GUI2MOTION              UserG2M;
} LAN_GUI2PODO,*pLAN_GUI2PODO;

typedef struct __LAN_STRUCT_PODO2GUI__
{
    RBCORE_SHM              CoreSHM;
    MOTION2GUI              UserM2G;
} LAN_PODO2GUI, *pLAN_PODO2GUI;

typedef struct __LAN_STRUCT_RM2HUBO__
{

    IMU_SENSOR      IMU[MAX_IMU];
    DATA_ARM_T      data_arm[7];    // to be deleted
    DATA_ARM_T      data_armR[7];   // new
    DATA_ARM_T      data_armL[7];   // new


} LAN_RM2HUBO, *pLAN_RM2HUBO;

#endif // RBLANDATA_H
