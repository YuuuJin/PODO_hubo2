#include "PODOServer.h"

PODO_GUI_Server::PODO_GUI_Server()
{
    //dataSize = sizeof(USER_COMMAND);
    dataSize = sizeof(LAN_GUI2PODO);
    dataReceived.clear();
    RBData.resize(dataSize);
}

void PODO_GUI_Server::RBReadData(){
    QDataStream in(RBTcpClient);
    in.setVersion(QDataStream::Qt_5_2);

    if(RBTcpClient->bytesAvailable() < dataSize)
        return;

    while(RBTcpClient->bytesAvailable() >= dataSize){
        in.readRawData(RBData.data(), dataSize);
        dataReceived.push_back(RBData);
    }
}


// =========================================

PODO_ROS_Server::PODO_ROS_Server()
{
    dataSize = sizeof(LAN_ROS2PODO);
    dataReceived.clear();
    RBData.resize(dataSize);
}

void PODO_ROS_Server::RBReadData(){
    QDataStream in(RBTcpClient);
    in.setVersion(QDataStream::Qt_5_2);

    if(RBTcpClient->bytesAvailable() < dataSize)
        return;

    while(RBTcpClient->bytesAvailable() >= dataSize){
        in.readRawData(RBData.data(), dataSize);
        dataReceived.push_back(RBData);
    }
}

RM_Link_Client::RM_Link_Client(){
    dataSize = sizeof(LAN_RM2HUBO);
    dataReceived.clear();
    RBData.resize(dataSize);
}


void RM_Link_Client::RBReadData(){
    QDataStream in(RBClientSocket);
    in.setVersion(QDataStream::Qt_5_2);

    if(RBClientSocket->bytesAvailable() < dataSize)
        return;

    while(RBClientSocket->bytesAvailable() >= dataSize){
        in.readRawData(RBData.data(), dataSize);
        dataReceived.push_back(RBData);
    }
}
