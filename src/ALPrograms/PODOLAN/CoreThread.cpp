#include "CoreThread.h"


extern pRBCORE_SHM             sharedData;
extern pUSER_SHM               sharedUSER;

CoreThread::CoreThread()
{
}

void CoreThread::run(){
    QTimer timerPODO2GUI, timerGUI2PODO, timerRMRead, timerRMWrite;
    CoreWorker worker;

    connect(&timerPODO2GUI, SIGNAL(timeout()), &worker, SLOT(onPODO2GUI()));
    connect(&timerGUI2PODO, SIGNAL(timeout()), &worker, SLOT(onGUI2PODO()));
    timerPODO2GUI.start(50);
    timerGUI2PODO.start(50);

//    connect(&timerRMRead, SIGNAL(timeout()), &worker, SLOT(onRMRead()));
//    connect(&timerRMWrite, SIGNAL(timeout()), &worker, SLOT(onRMWrite()));
//    timerRMRead.start(10);
//    timerRMWrite.start(10);

    exec();
}



CoreWorker::CoreWorker(){
    serverPODOGUI = new PODO_GUI_Server();
    serverPODOGUI->RBServerOpen(QHostAddress::AnyIPv4, 4000);

    clientRM = new RM_Link_Client();
}


void CoreWorker::SendtoGUI(){
    memcpy(&(DATA_PODO.CoreSHM), sharedData, sizeof(RBCORE_SHM));
    memcpy(&(DATA_PODO.UserM2G), &(sharedUSER->M2G), sizeof(MOTION2GUI));

    QByteArray SendData = QByteArray::fromRawData((char*)&DATA_PODO, sizeof(LAN_PODO2GUI));
    serverPODOGUI->RBSendData(SendData);
}

void CoreWorker::ReadfromGUI(){    
    QByteArray tempData = serverPODOGUI->dataReceived[0];
    serverPODOGUI->dataReceived.pop_front();

//    USER_COMMAND cmd;
//    memcpy(&cmd, tempData, sizeof(USER_COMMAND));

//    int target = cmd.COMMAND_TARGET;
//    for(int i=0; i<MAX_COMMAND_DATA; i++){
//        sharedData->COMMAND[target].USER_PARA_CHAR[i]    = cmd.COMMAND_DATA.USER_PARA_CHAR[i];
//        sharedData->COMMAND[target].USER_PARA_INT[i]     = cmd.COMMAND_DATA.USER_PARA_INT[i];
//        sharedData->COMMAND[target].USER_PARA_FLOAT[i]   = cmd.COMMAND_DATA.USER_PARA_FLOAT[i];
//        sharedData->COMMAND[target].USER_PARA_DOUBLE[i]  = cmd.COMMAND_DATA.USER_PARA_DOUBLE[i];
//    }
//    sharedData->COMMAND[target].USER_COMMAND = cmd.COMMAND_DATA.USER_COMMAND;

    LAN_GUI2PODO tempDATA;
    memcpy(&tempDATA, tempData, sizeof(LAN_GUI2PODO));
    memcpy(&(sharedUSER->G2M), &(tempDATA.UserG2M), sizeof(GUI2MOTION));

    int target = tempDATA.UserCMD.COMMAND_TARGET;
    for(int i=0; i<MAX_COMMAND_DATA; i++){
        sharedData->COMMAND[target].USER_PARA_CHAR[i]    = tempDATA.UserCMD.COMMAND_DATA.USER_PARA_CHAR[i];
        sharedData->COMMAND[target].USER_PARA_INT[i]     = tempDATA.UserCMD.COMMAND_DATA.USER_PARA_INT[i];
        sharedData->COMMAND[target].USER_PARA_FLOAT[i]   = tempDATA.UserCMD.COMMAND_DATA.USER_PARA_FLOAT[i];
        sharedData->COMMAND[target].USER_PARA_DOUBLE[i]  = tempDATA.UserCMD.COMMAND_DATA.USER_PARA_DOUBLE[i];
    }
    sharedData->COMMAND[target].USER_COMMAND = tempDATA.UserCMD.COMMAND_DATA.USER_COMMAND;
}

void CoreWorker::onPODO2GUI(){
    if(serverPODOGUI->RBConnectionState == RBLAN_CS_CONNECTED){
        SendtoGUI();
    }
}

void CoreWorker::onGUI2PODO(){
    if(serverPODOGUI->dataReceived.size() > 0){
        ReadfromGUI();
    }
}


void CoreWorker::onRMRead(){
    while(clientRM->dataReceived.size() > 1)
    {
        clientRM->dataReceived.pop_front();
    }
    if(clientRM->dataReceived.size() > 0){
        QByteArray tempData = clientRM->dataReceived[0];        
        clientRM->dataReceived.pop_front();

        LAN_RM2HUBO tempDATA;
        memcpy(&tempDATA, tempData.data(), sizeof(LAN_RM2HUBO));



        memcpy(&(sharedData->data_arm), &(tempDATA.data_arm), sizeof(DATA_ARM_T)*7);
        memcpy(&(sharedData->data_armL), &(tempDATA.data_armL), sizeof(DATA_ARM_T)*7);
        memcpy(&(sharedData->data_armR), &(tempDATA.data_armR), sizeof(DATA_ARM_T)*7);
        memcpy(&(sharedData->IMU), &(tempDATA.IMU), sizeof(IMU_SENSOR)*MAX_IMU);



//        double temp_ms = tempDATA.motion_ms_time;
//        double temp_mm = tempDATA.motion_mm_mag;




//        sharedData->UB_MOTION_TIME_MS = temp_ms;
//        sharedData->UB_MOTION_MAG_MM = temp_mm;
//        sharedData->IS_UB_MOVE = true;
    }
}

int connectCnt = 0;
void CoreWorker::onRMWrite(){
    if(clientRM->RBConnectionState == RBLAN_CS_CONNECTED){
        connectCnt = 0;

//        if(sharedUSER->RM_cmd != 0){
//            int tempSend = sharedUSER->RM_cmd;
//            sharedUSER->RM_cmd = 0;

//            QByteArray SendData = QByteArray::fromRawData((char*)&tempSend, sizeof(int));
//            clientRM->RBSendData(SendData);
//            FILE_LOG(logINFO) << "RM_cmd: " << tempSend;
//        }
    }else{
        connectCnt++;
        if(connectCnt%20 == 0){
            clientRM->RBConnect("192.168.0.30", 4001);
       //     FILE_LOG(logINFO) << "RM Link Connection Try..";
        }
    }
}
