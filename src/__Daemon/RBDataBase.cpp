#include "RBDataBase.h"

RBDataBase::RBDataBase()
{
    dbCore = QSqlDatabase::addDatabase("QSQLITE");
}

void RBDataBase::SetFilename(QString name){
    filename = name;
}

bool RBDataBase::OpenDB(){
    dbCore.setDatabaseName(filename);
    if(!dbCore.open()){
        FILE_LOG(logERROR) << "Can't open database file [" << filename.toStdString().data() << "]";
        return false;
    }

    QString strQuery;
    QSqlQuery query(dbCore);
    query.setForwardOnly(true);

    // General ----
    query.exec("SELECT * from General");
    if(query.next()){
        _DB_GENERAL.VERSION         = query.value(0).toInt();
        _DB_GENERAL.NO_OF_AL        = query.value(1).toInt();
        _DB_GENERAL.NO_OF_COMM_CH   = query.value(2).toInt();
        _DB_GENERAL.NO_OF_MC        = query.value(3).toInt();
        _DB_GENERAL.NO_OF_FT        = query.value(4).toInt();
        _DB_GENERAL.NO_OF_IMU       = query.value(5).toInt();
        _DB_GENERAL.NO_OF_SP        = query.value(6).toInt();
    }

    // PODO AL ----
    strQuery.sprintf("SELECT * from AL");
    query.exec(strQuery);
    for(int i=0; i<_DB_GENERAL.NO_OF_AL; i++){
        if(query.next()){
            _DB_AL[i].ALName          = query.value(0).toString();
            _DB_AL[i].FileName        = query.value(1).toString();
            _DB_AL[i].PathName        = query.value(2).toString();
        }
    }

    // Motor Controller ----
    strQuery.sprintf("SELECT * from MotionController");
    query.exec(strQuery);
    for(int i=0; i<_DB_GENERAL.NO_OF_MC; i++){
        if(query.next()){
            _DB_MC[i].BOARD_ID          = query.value(0).toInt();
            _DB_MC[i].BOARD_NAME        = query.value(1).toString();
            _DB_MC[i].BOARD_TYPE        = query.value(2).toInt();
            _DB_MC[i].MOTOR_CHANNEL     = _DB_MC[i].BOARD_TYPE;
            _DB_MC[i].CAN_CHANNEL       = query.value(3).toInt();
            _DB_MC[i].ID_SEND_REF       = query.value(4).toInt();
            _DB_MC[i].ID_RCV_ENC        = query.value(5).toInt();
            _DB_MC[i].ID_RCV_STAT       = query.value(6).toInt();
            _DB_MC[i].ID_RCV_INFO       = query.value(7).toInt();
            _DB_MC[i].ID_RCV_PARA       = query.value(8).toInt();
            _DB_MC[i].ID_SEND_GENERAL   = query.value(9).toInt();

            if(_DB_MC[i].MOTOR_CHANNEL == 3){
                _DB_MC[i].JOINTS[0].HARMONIC = 100.0;
                _DB_MC[i].JOINTS[0].PULLY_DRIVE = 1.0;
                _DB_MC[i].JOINTS[0].PULLY_DRIVEN = 1.0;
                _DB_MC[i].JOINTS[0].ENCODER_RESOLUTION = 128.0;
                _DB_MC[i].JOINTS[0].PPR = _DB_MC[i].JOINTS[0].HARMONIC * _DB_MC[i].JOINTS[0].PULLY_DRIVEN / _DB_MC[i].JOINTS[0].PULLY_DRIVE * _DB_MC[i].JOINTS[0].ENCODER_RESOLUTION / 360.0;
                _DB_MC[i].JOINTS[0].FRIC_PARAM1 = 0.0;
                _DB_MC[i].JOINTS[0].FRIC_PARAM2 = 0.0;

                _DB_MC[i].JOINTS[1].HARMONIC = 100.0;
                _DB_MC[i].JOINTS[1].PULLY_DRIVE = 1.0;
                _DB_MC[i].JOINTS[1].PULLY_DRIVEN = 1.0;
                _DB_MC[i].JOINTS[1].ENCODER_RESOLUTION = 128.0;
                _DB_MC[i].JOINTS[1].PPR = _DB_MC[i].JOINTS[1].HARMONIC * _DB_MC[i].JOINTS[1].PULLY_DRIVEN / _DB_MC[i].JOINTS[1].PULLY_DRIVE * _DB_MC[i].JOINTS[1].ENCODER_RESOLUTION / 360.0;
                _DB_MC[i].JOINTS[1].FRIC_PARAM1 = 0.0;
                _DB_MC[i].JOINTS[1].FRIC_PARAM2 = 0.0;

                _DB_MC[i].JOINTS[2].HARMONIC = 100.0;
                _DB_MC[i].JOINTS[2].PULLY_DRIVE = 1.0;
                _DB_MC[i].JOINTS[2].PULLY_DRIVEN = 1.0;
                _DB_MC[i].JOINTS[2].ENCODER_RESOLUTION = 128.0;
                _DB_MC[i].JOINTS[2].PPR = _DB_MC[i].JOINTS[2].HARMONIC * _DB_MC[i].JOINTS[2].PULLY_DRIVEN / _DB_MC[i].JOINTS[2].PULLY_DRIVE * _DB_MC[i].JOINTS[2].ENCODER_RESOLUTION / 360.0;
                _DB_MC[i].JOINTS[2].FRIC_PARAM1 = 0.0;
                _DB_MC[i].JOINTS[2].FRIC_PARAM2 = 0.0;
            }else if(_DB_MC[i].MOTOR_CHANNEL == 5){
                _DB_MC[i].JOINTS[0].HARMONIC = 256.0;
                _DB_MC[i].JOINTS[0].PULLY_DRIVE = 1.0;
                _DB_MC[i].JOINTS[0].PULLY_DRIVEN = 1.0;
                _DB_MC[i].JOINTS[0].ENCODER_RESOLUTION = 128.0;
                _DB_MC[i].JOINTS[0].PPR = _DB_MC[i].JOINTS[0].HARMONIC * _DB_MC[i].JOINTS[0].PULLY_DRIVEN / _DB_MC[i].JOINTS[0].PULLY_DRIVE * _DB_MC[i].JOINTS[0].ENCODER_RESOLUTION / 360.0;
                _DB_MC[i].JOINTS[0].FRIC_PARAM1 = 0.0;
                _DB_MC[i].JOINTS[0].FRIC_PARAM2 = 0.0;

                _DB_MC[i].JOINTS[1].HARMONIC = 256.0;
                _DB_MC[i].JOINTS[1].PULLY_DRIVE = 1.0;
                _DB_MC[i].JOINTS[1].PULLY_DRIVEN = 1.0;
                _DB_MC[i].JOINTS[1].ENCODER_RESOLUTION = 128.0;
                _DB_MC[i].JOINTS[1].PPR = _DB_MC[i].JOINTS[1].HARMONIC * _DB_MC[i].JOINTS[1].PULLY_DRIVEN / _DB_MC[i].JOINTS[1].PULLY_DRIVE * _DB_MC[i].JOINTS[1].ENCODER_RESOLUTION / 360.0;
                _DB_MC[i].JOINTS[1].FRIC_PARAM1 = 0.0;
                _DB_MC[i].JOINTS[1].FRIC_PARAM2 = 0.0;

                _DB_MC[i].JOINTS[2].HARMONIC = 256.0;
                _DB_MC[i].JOINTS[2].PULLY_DRIVE = 1.0;
                _DB_MC[i].JOINTS[2].PULLY_DRIVEN = 1.0;
                _DB_MC[i].JOINTS[2].ENCODER_RESOLUTION = 128.0;
                _DB_MC[i].JOINTS[2].PPR = _DB_MC[i].JOINTS[2].HARMONIC * _DB_MC[i].JOINTS[2].PULLY_DRIVEN / _DB_MC[i].JOINTS[2].PULLY_DRIVE * _DB_MC[i].JOINTS[2].ENCODER_RESOLUTION / 360.0;
                _DB_MC[i].JOINTS[2].FRIC_PARAM1 = 0.0;
                _DB_MC[i].JOINTS[2].FRIC_PARAM2 = 0.0;

                _DB_MC[i].JOINTS[3].HARMONIC = 256.0;
                _DB_MC[i].JOINTS[3].PULLY_DRIVE = 1.0;
                _DB_MC[i].JOINTS[3].PULLY_DRIVEN = 1.0;
                _DB_MC[i].JOINTS[3].ENCODER_RESOLUTION = 128.0;
                _DB_MC[i].JOINTS[3].PPR = _DB_MC[i].JOINTS[3].HARMONIC * _DB_MC[i].JOINTS[3].PULLY_DRIVEN / _DB_MC[i].JOINTS[3].PULLY_DRIVE * _DB_MC[i].JOINTS[3].ENCODER_RESOLUTION / 360.0;
                _DB_MC[i].JOINTS[3].FRIC_PARAM1 = 0.0;
                _DB_MC[i].JOINTS[3].FRIC_PARAM2 = 0.0;

                _DB_MC[i].JOINTS[4].HARMONIC = 256.0;
                _DB_MC[i].JOINTS[4].PULLY_DRIVE = 1.0;
                _DB_MC[i].JOINTS[4].PULLY_DRIVEN = 1.0;
                _DB_MC[i].JOINTS[4].ENCODER_RESOLUTION = 128.0;
                _DB_MC[i].JOINTS[4].PPR = _DB_MC[i].JOINTS[4].HARMONIC * _DB_MC[i].JOINTS[4].PULLY_DRIVEN / _DB_MC[i].JOINTS[4].PULLY_DRIVE * _DB_MC[i].JOINTS[4].ENCODER_RESOLUTION / 360.0;
                _DB_MC[i].JOINTS[4].FRIC_PARAM1 = 0.0;
                _DB_MC[i].JOINTS[4].FRIC_PARAM2 = 0.0;
            }else{
                for(int j=0; j<MOTOR_2CH; j++){
                    _DB_MC[i].JOINTS[j].HARMONIC = query.value(10+j*6).toDouble();
                    _DB_MC[i].JOINTS[j].PULLY_DRIVE = query.value(11+j*6).toDouble();
                    _DB_MC[i].JOINTS[j].PULLY_DRIVEN = query.value(12+j*6).toDouble();
                    _DB_MC[i].JOINTS[j].ENCODER_RESOLUTION = query.value(13+j*6).toDouble();
                    _DB_MC[i].JOINTS[j].PPR = _DB_MC[i].JOINTS[j].HARMONIC * _DB_MC[i].JOINTS[j].PULLY_DRIVEN / _DB_MC[i].JOINTS[j].PULLY_DRIVE * _DB_MC[i].JOINTS[j].ENCODER_RESOLUTION / 360.0;
                    _DB_MC[i].JOINTS[j].FRIC_PARAM1 = query.value(14+j*6).toDouble();
                    _DB_MC[i].JOINTS[j].FRIC_PARAM2 = query.value(15+j*6).toDouble();
                }
            }
        }
    }

    // FT Sensor ----
    strQuery.sprintf("SELECT * from FTSensor");
    query.exec(strQuery);
    for(int i=0; i<_DB_GENERAL.NO_OF_FT; i++){
        if(query.next()){
            _DB_FT[i].BOARD_ID      = query.value(0).toInt();
            _DB_FT[i].BOARD_NAME    = query.value(1).toString();
            _DB_FT[i].SENSOR_ID     = query.value(2).toInt();
            _DB_FT[i].CAN_CHANNEL   = query.value(3).toInt();
            _DB_FT[i].SENSOR_TYPE   = query.value(4).toInt();
            _DB_FT[i].ID_RCV_DATA1  = query.value(5).toInt();
            _DB_FT[i].ID_RCV_DATA2  = query.value(6).toInt();
            _DB_FT[i].ID_RCV_ACC    = query.value(7).toInt();
            _DB_FT[i].ID_RCV_STAT   = query.value(8).toInt();
            _DB_FT[i].ID_RCV_INFO   = query.value(9).toInt();
            _DB_FT[i].ID_RCV_PARA   = query.value(10).toInt();
        }
    }

    // IMU Sensor ----
    strQuery.sprintf("SELECT * from IMUSensor");
    query.exec(strQuery);
    for(int i=0; i<_DB_GENERAL.NO_OF_IMU; i++){
        if(query.next()){
            _DB_IMU[i].BOARD_ID     = query.value(0).toInt();
            _DB_IMU[i].BOARD_NAME   = query.value(1).toString();
            _DB_IMU[i].SENSOR_ID    = query.value(2).toInt();
            _DB_IMU[i].CAN_CHANNEL  = query.value(3).toInt();
            _DB_IMU[i].SENSOR_TYPE  = query.value(4).toInt();
            _DB_IMU[i].ID_RCV_DATA1 = query.value(5).toInt();
            _DB_IMU[i].ID_RCV_DATA2 = query.value(6).toInt();
            _DB_IMU[i].ID_RCV_STAT  = query.value(7).toInt();
            _DB_IMU[i].ID_RCV_INFO  = query.value(8).toInt();
            _DB_IMU[i].ID_RCV_PARA  = query.value(9).toInt();
        }
    }

    // Smart Power Controller ----
    strQuery.sprintf("SELECT * from SmartPower");
    query.exec(strQuery);
    for(int i=0; i<_DB_GENERAL.NO_OF_SP; i++){
        if(query.next()){
            _DB_SP[i].BOARD_ID      = query.value(0).toInt();
            _DB_SP[i].BOARD_NAME    = query.value(1).toString();
            _DB_SP[i].CAN_CHANNEL   = query.value(2).toInt();
            _DB_SP[i].ID_RCV_DATA   = query.value(3).toInt();
            _DB_SP[i].ID_RCV_INFO   = query.value(5).toInt();
            _DB_SP[i].ID_SEND_GENERAL = query.value(6).toInt();
        }
    }

    dbCore.close();
    return true;
}
