#ifndef CORETHREAD_H
#define CORETHREAD_H

#include <sys/mman.h>
#include <fcntl.h>

#include <QThread>
#include <QByteArray>
#include <QTimer>

#include "PODOServer.h"


class CoreThread : public QThread
{
    Q_OBJECT
public:
    CoreThread();

protected:
    void run();
};


class CoreWorker : public QObject
{
    Q_OBJECT
public:
    CoreWorker();

private slots:
    void onPODO2GUI();
    void onGUI2PODO();
    void onRMRead();
    void onRMWrite();
private:
    PODO_GUI_Server         *serverPODOGUI;

    RM_Link_Client          *clientRM;
    LAN_PODO2GUI            DATA_PODO;
    LAN_RM2HUBO             DATA_RM2HUBO;
    void    SendtoGUI();
    void    ReadfromGUI();

};



#endif // CORETHREAD_H
