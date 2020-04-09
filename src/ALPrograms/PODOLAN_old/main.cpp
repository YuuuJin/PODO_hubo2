#include <QCoreApplication>
#include <iostream>
#include <signal.h>
#include "CoreThread.h"

int     PODO_NO = -1;


pRBCORE_SHM             sharedData;
pUSER_SHM               sharedUSER;

void CheckArguments(int argc, char *argv[]){
    int opt = 0;
    int podoNum = -1;
    while((opt = getopt(argc, argv, "p:")) != -1){
        switch(opt){
        case 'p':
            podoNum = atoi(optarg);
            if(podoNum == 0){
                FILE_LOG(logERROR) << optarg;
                FILE_LOG(logERROR) << "Invalid option for AL";
                FILE_LOG(logERROR) << "Valid options are \"true\", \"TRUE\", \"false\", \"FALSE\"";
                FILE_LOG(logERROR) << "Use default value";
            }else{
                PODO_NO = podoNum;
            }
            break;
        case '?':
            if(optopt == 'p'){
                FILE_LOG(logERROR) << "Option for AL";
                FILE_LOG(logERROR) << "Valid options are \"Integer Values\"";
            }
        }
    }


    std::cout << std::endl;
    FILE_LOG(logERROR) << "===========AL Setting============";
    FILE_LOG(logWARNING) << argv[0];
    FILE_LOG(logWARNING) << "AL Number: " << PODO_NO;
    FILE_LOG(logERROR) << "=================================";
    std::cout << std::endl;
}

int CreateSharedMemory(){
    mlockall(MCL_CURRENT|MCL_FUTURE);

    int ret = true;

    int shmFD;
    // Core Shared Memory Creation ============================================
    shmFD = shm_open(RBCORE_SHM_NAME, O_RDWR, 0666);
    if(shmFD == -1){
        FILE_LOG(logERROR) << "Fail to open core shared memory";
        ret = false;
    }else{
        if(ftruncate(shmFD, sizeof(RBCORE_SHM)) == -1){
            FILE_LOG(logERROR) << "Fail to truncate core shared memory";
            ret = false;
        }else{
            sharedData = (pRBCORE_SHM)mmap(0, sizeof(RBCORE_SHM), PROT_READ|PROT_WRITE, MAP_SHARED, shmFD, 0);
            if(sharedData == (void*)-1){
                FILE_LOG(logERROR) << "Fail to mapping core shared memory";
                ret = false;
            }
        }
    }
    FILE_LOG(logSUCCESS) << "Core shared memory creation = OK";
    // =========================================================================

    // User Shared Memory Creation ============================================
    shmFD = shm_open(USER_SHM_NAME, O_CREAT|O_RDWR, 0666);
    if(shmFD == -1){
        FILE_LOG(logERROR) << "Fail to open user shared memory";
        ret = false;
    }else{
        if(ftruncate(shmFD, sizeof(USER_SHM)) == -1){
            FILE_LOG(logERROR) << "Fail to truncate user shared memory";
            ret = false;
        }else{
            sharedUSER = (pUSER_SHM)mmap(0, sizeof(USER_SHM), PROT_READ|PROT_WRITE, MAP_SHARED, shmFD, 0);
            if(sharedUSER == (void*)-1){
                FILE_LOG(logERROR) << "Fail to mapping user shared memory";
                ret = false;
            }
        }
        memset(sharedUSER, 0, sizeof(USER_SHM));
    }
    FILE_LOG(logSUCCESS) << "User shared memory creation = OK";
    // =========================================================================
    return ret;
}

int main(int argc, char *argv[])
{
    // Block memory swapping ------------------------------
    mlockall(MCL_CURRENT|MCL_FUTURE);


    CheckArguments(argc, argv);
    int ret = CreateSharedMemory();

    QCoreApplication a(argc, argv);

    if(ret == false){
        FILE_LOG(logERROR) << "Initialize Failed..";
        FILE_LOG(logERROR) << "Terminate AL..";
    }
    CoreThread  core;
    core.start();

    return a.exec();
}
