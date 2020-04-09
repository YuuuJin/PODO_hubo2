#include "TutorialDialog.h"
#include "ui_TutorialDialog.h"

#include "BasicFiles/PODOALDialog.h"


enum DebrisALCOMMAND
{
    DemoFireAL_NO_ACT = 100,
    DemoFireAL_READYPOS,
    DemoFireAL_MOTIONSTART,
    DemoFireAL_TEST,
    DemoFireAL_SET,
    DemoFireAL_SW_READY,
    DemoFireAL_SW_SET,
    DemoFireAL_SW_GRASP,
    DemoFireAL_SW_HOLD,
    DemoFireAL_SW_BACK
};


enum TUTORIAL_COMMAND
{
    TUTORIAL_NO_ACT = 100,
    TUTORIAL_FINGER_CONTROL
};

enum JS_QP_COMMAND
{
    JS_NO_ACT = 500,
    JS_QP_START,
    JS_QP_STOP,
    JS_TEST,
    JS_INIT_POS,
    JS_TCP_CONNECT

};


TutorialDialog::TutorialDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::TutorialDialog)
{
    ui->setupUi(this);
    ALNum_Tuto = PODOALDialog::GetALNumFromFileName("ALTutorial");
    ALNum_Demo = PODOALDialog::GetALNumFromFileName("DemoFireAL");

    connect(pLAN, SIGNAL(NewPODOData()), this, SLOT(UpdateSomethings()));
}

TutorialDialog::~TutorialDialog()
{
    delete ui;
}


void TutorialDialog::UpdateSomethings(){

    QString str;

    ui->LE_IMU_ROLL->setText(str.sprintf("%.2f", PODO_DATA.CoreSHM.IMU[0].Roll));
    ui->LE_IMU_PITCH->setText(str.sprintf("%.2f", PODO_DATA.CoreSHM.IMU[0].Pitch));
    ui->LE_IMU_YAW->setText(str.sprintf("%.2f", PODO_DATA.CoreSHM.IMU[0].Yaw));

    if(modiCnt < 20){
        ui->LE_MODI_R0->setReadOnly(true);
        ui->LE_MODI_R1->setReadOnly(true);
        ui->LE_MODI_R2->setReadOnly(true);
        ui->LE_MODI_R3->setReadOnly(true);
        ui->LE_MODI_R4->setReadOnly(true);

        ui->LE_MODI_L0->setReadOnly(true);
        ui->LE_MODI_L1->setReadOnly(true);
        ui->LE_MODI_L2->setReadOnly(true);
        ui->LE_MODI_L3->setReadOnly(true);
        ui->LE_MODI_L4->setReadOnly(true);

        ui->LE_MODI_R0->setText(QString().sprintf("%.2f", PODO_DATA.CoreSHM.EXF_R_Modifier[0]));
        ui->LE_MODI_R1->setText(QString().sprintf("%.2f", PODO_DATA.CoreSHM.EXF_R_Modifier[1]));
        ui->LE_MODI_R2->setText(QString().sprintf("%.2f", PODO_DATA.CoreSHM.EXF_R_Modifier[2]));
        ui->LE_MODI_R3->setText(QString().sprintf("%.2f", PODO_DATA.CoreSHM.EXF_R_Modifier[3]));
        ui->LE_MODI_R4->setText(QString().sprintf("%.2f", PODO_DATA.CoreSHM.EXF_R_Modifier[4]));

        ui->LE_MODI_L0->setText(QString().sprintf("%.2f", PODO_DATA.CoreSHM.EXF_L_Modifier[0]));
        ui->LE_MODI_L1->setText(QString().sprintf("%.2f", PODO_DATA.CoreSHM.EXF_L_Modifier[1]));
        ui->LE_MODI_L2->setText(QString().sprintf("%.2f", PODO_DATA.CoreSHM.EXF_L_Modifier[2]));
        ui->LE_MODI_L3->setText(QString().sprintf("%.2f", PODO_DATA.CoreSHM.EXF_L_Modifier[3]));
        ui->LE_MODI_L4->setText(QString().sprintf("%.2f", PODO_DATA.CoreSHM.EXF_L_Modifier[4]));

        modiCnt++;
    }else if(modiCnt <25){
        ui->LE_MODI_R0->setReadOnly(false);
        ui->LE_MODI_R1->setReadOnly(false);
        ui->LE_MODI_R2->setReadOnly(false);
        ui->LE_MODI_R3->setReadOnly(false);
        ui->LE_MODI_R4->setReadOnly(false);

        ui->LE_MODI_L0->setReadOnly(false);
        ui->LE_MODI_L1->setReadOnly(false);
        ui->LE_MODI_L2->setReadOnly(false);
        ui->LE_MODI_L3->setReadOnly(false);
        ui->LE_MODI_L4->setReadOnly(false);

        modiCnt++;
    }
}



void TutorialDialog::on_BTN_R_GRASP0_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = TUTORIAL_FINGER_CONTROL;
    cmd.COMMAND_TARGET = ALNum_Tuto;

    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = 0;     // right
    cmd.COMMAND_DATA.USER_PARA_CHAR[1] = 0;     // 0th finger
    cmd.COMMAND_DATA.USER_PARA_CHAR[2] = ui->LE_CURRENT->text().toInt();    // current
    pLAN->SendCommand(cmd);
}

void TutorialDialog::on_BTN_R_GRASP1_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = TUTORIAL_FINGER_CONTROL;
    cmd.COMMAND_TARGET = ALNum_Tuto;

    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = 0;     // right
    cmd.COMMAND_DATA.USER_PARA_CHAR[1] = 1;     // 1th finger
    cmd.COMMAND_DATA.USER_PARA_CHAR[2] = ui->LE_CURRENT->text().toInt();    // current
    pLAN->SendCommand(cmd);
}

void TutorialDialog::on_BTN_R_GRASP2_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = TUTORIAL_FINGER_CONTROL;
    cmd.COMMAND_TARGET = ALNum_Tuto;

    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = 0;     // right
    cmd.COMMAND_DATA.USER_PARA_CHAR[1] = 2;     // 2th finger
    cmd.COMMAND_DATA.USER_PARA_CHAR[2] = ui->LE_CURRENT->text().toInt();    // current
    pLAN->SendCommand(cmd);
}

void TutorialDialog::on_BTN_R_GRASP3_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = TUTORIAL_FINGER_CONTROL;
    cmd.COMMAND_TARGET = ALNum_Tuto;

    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = 0;     // right
    cmd.COMMAND_DATA.USER_PARA_CHAR[1] = 3;     // 3th finger
    cmd.COMMAND_DATA.USER_PARA_CHAR[2] = ui->LE_CURRENT->text().toInt();    // current
    pLAN->SendCommand(cmd);
}

void TutorialDialog::on_BTN_R_GRASP4_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = TUTORIAL_FINGER_CONTROL;
    cmd.COMMAND_TARGET = ALNum_Tuto;

    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = 0;     // right
    cmd.COMMAND_DATA.USER_PARA_CHAR[1] = 4;     // 4th finger
    cmd.COMMAND_DATA.USER_PARA_CHAR[2] = ui->LE_CURRENT->text().toInt();    // current
    pLAN->SendCommand(cmd);
}

void TutorialDialog::on_BTN_R_GRASP_ALL_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = TUTORIAL_FINGER_CONTROL;
    cmd.COMMAND_TARGET = ALNum_Tuto;

    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = 0;     // right
    cmd.COMMAND_DATA.USER_PARA_CHAR[1] = -1;    // all finger
    cmd.COMMAND_DATA.USER_PARA_CHAR[2] = ui->LE_CURRENT->text().toInt();    // current
    pLAN->SendCommand(cmd);
}

void TutorialDialog::on_BTN_R_RELEASE0_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = TUTORIAL_FINGER_CONTROL;
    cmd.COMMAND_TARGET = ALNum_Tuto;

    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = 0;     // right
    cmd.COMMAND_DATA.USER_PARA_CHAR[1] = 0;     // 0th finger
    cmd.COMMAND_DATA.USER_PARA_CHAR[2] = -ui->LE_CURRENT->text().toInt();    // current
    pLAN->SendCommand(cmd);
}

void TutorialDialog::on_BTN_R_RELEASE1_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = TUTORIAL_FINGER_CONTROL;
    cmd.COMMAND_TARGET = ALNum_Tuto;

    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = 0;     // right
    cmd.COMMAND_DATA.USER_PARA_CHAR[1] = 1;     // 1th finger
    cmd.COMMAND_DATA.USER_PARA_CHAR[2] = -ui->LE_CURRENT->text().toInt();    // current
    pLAN->SendCommand(cmd);
}

void TutorialDialog::on_BTN_R_RELEASE2_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = TUTORIAL_FINGER_CONTROL;
    cmd.COMMAND_TARGET = ALNum_Tuto;

    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = 0;     // right
    cmd.COMMAND_DATA.USER_PARA_CHAR[1] = 2;     // 2th finger
    cmd.COMMAND_DATA.USER_PARA_CHAR[2] = -ui->LE_CURRENT->text().toInt();    // current
    pLAN->SendCommand(cmd);
}

void TutorialDialog::on_BTN_R_RELEASE3_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = TUTORIAL_FINGER_CONTROL;
    cmd.COMMAND_TARGET = ALNum_Tuto;

    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = 0;     // right
    cmd.COMMAND_DATA.USER_PARA_CHAR[1] = 3;     // 3th finger
    cmd.COMMAND_DATA.USER_PARA_CHAR[2] = -ui->LE_CURRENT->text().toInt();    // current
    pLAN->SendCommand(cmd);
}

void TutorialDialog::on_BTN_R_RELEASE4_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = TUTORIAL_FINGER_CONTROL;
    cmd.COMMAND_TARGET = ALNum_Tuto;

    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = 0;     // right
    cmd.COMMAND_DATA.USER_PARA_CHAR[1] = 4;     // 4th finger
    cmd.COMMAND_DATA.USER_PARA_CHAR[2] = -ui->LE_CURRENT->text().toInt();    // current
    pLAN->SendCommand(cmd);
}

void TutorialDialog::on_BTN_R_RELEASE_ALL_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = TUTORIAL_FINGER_CONTROL;
    cmd.COMMAND_TARGET = ALNum_Tuto;

    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = 0;     // right
    cmd.COMMAND_DATA.USER_PARA_CHAR[1] = -1;    // all finger
    cmd.COMMAND_DATA.USER_PARA_CHAR[2] = -ui->LE_CURRENT->text().toInt();    // current
    pLAN->SendCommand(cmd);
}





void TutorialDialog::on_BTN_L_GRASP0_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = TUTORIAL_FINGER_CONTROL;
    cmd.COMMAND_TARGET = ALNum_Tuto;

    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = 1;     // left
    cmd.COMMAND_DATA.USER_PARA_CHAR[1] = 0;     // 0th finger
    cmd.COMMAND_DATA.USER_PARA_CHAR[2] = ui->LE_CURRENT->text().toInt();    // current
    pLAN->SendCommand(cmd);
}

void TutorialDialog::on_BTN_L_GRASP1_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = TUTORIAL_FINGER_CONTROL;
    cmd.COMMAND_TARGET = ALNum_Tuto;

    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = 1;     // left
    cmd.COMMAND_DATA.USER_PARA_CHAR[1] = 1;     // 1th finger
    cmd.COMMAND_DATA.USER_PARA_CHAR[2] = ui->LE_CURRENT->text().toInt();    // current
    pLAN->SendCommand(cmd);
}

void TutorialDialog::on_BTN_L_GRASP2_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = TUTORIAL_FINGER_CONTROL;
    cmd.COMMAND_TARGET = ALNum_Tuto;

    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = 1;     // left
    cmd.COMMAND_DATA.USER_PARA_CHAR[1] = 2;     // 2th finger
    cmd.COMMAND_DATA.USER_PARA_CHAR[2] = ui->LE_CURRENT->text().toInt();    // current
    pLAN->SendCommand(cmd);
}

void TutorialDialog::on_BTN_L_GRASP3_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = TUTORIAL_FINGER_CONTROL;
    cmd.COMMAND_TARGET = ALNum_Tuto;

    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = 1;     // left
    cmd.COMMAND_DATA.USER_PARA_CHAR[1] = 3;     // 3th finger
    cmd.COMMAND_DATA.USER_PARA_CHAR[2] = ui->LE_CURRENT->text().toInt();    // current
    pLAN->SendCommand(cmd);
}

void TutorialDialog::on_BTN_L_GRASP4_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = TUTORIAL_FINGER_CONTROL;
    cmd.COMMAND_TARGET = ALNum_Tuto;

    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = 1;     // left
    cmd.COMMAND_DATA.USER_PARA_CHAR[1] = 4;     // 4th finger
    cmd.COMMAND_DATA.USER_PARA_CHAR[2] = ui->LE_CURRENT->text().toInt();    // current
    pLAN->SendCommand(cmd);
}

void TutorialDialog::on_BTN_L_GRASP_ALL_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = TUTORIAL_FINGER_CONTROL;
    cmd.COMMAND_TARGET = ALNum_Tuto;

    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = 1;     // left
    cmd.COMMAND_DATA.USER_PARA_CHAR[1] = -1;    // all finger
    cmd.COMMAND_DATA.USER_PARA_CHAR[2] = ui->LE_CURRENT->text().toInt();    // current
    pLAN->SendCommand(cmd);
}

void TutorialDialog::on_BTN_L_RELEASE0_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = TUTORIAL_FINGER_CONTROL;
    cmd.COMMAND_TARGET = ALNum_Tuto;

    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = 1;     // left
    cmd.COMMAND_DATA.USER_PARA_CHAR[1] = 0;     // 0th finger
    cmd.COMMAND_DATA.USER_PARA_CHAR[2] = -ui->LE_CURRENT->text().toInt();    // current
    pLAN->SendCommand(cmd);
}

void TutorialDialog::on_BTN_L_RELEASE1_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = TUTORIAL_FINGER_CONTROL;
    cmd.COMMAND_TARGET = ALNum_Tuto;

    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = 1;     // left
    cmd.COMMAND_DATA.USER_PARA_CHAR[1] = 1;     // 1th finger
    cmd.COMMAND_DATA.USER_PARA_CHAR[2] = -ui->LE_CURRENT->text().toInt();    // current
    pLAN->SendCommand(cmd);
}

void TutorialDialog::on_BTN_L_RELEASE2_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = TUTORIAL_FINGER_CONTROL;
    cmd.COMMAND_TARGET = ALNum_Tuto;

    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = 1;     // left
    cmd.COMMAND_DATA.USER_PARA_CHAR[1] = 2;     // 2th finger
    cmd.COMMAND_DATA.USER_PARA_CHAR[2] = -ui->LE_CURRENT->text().toInt();    // current
    pLAN->SendCommand(cmd);
}

void TutorialDialog::on_BTN_L_RELEASE3_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = TUTORIAL_FINGER_CONTROL;
    cmd.COMMAND_TARGET = ALNum_Tuto;

    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = 1;     // left
    cmd.COMMAND_DATA.USER_PARA_CHAR[1] = 3;     // 3th finger
    cmd.COMMAND_DATA.USER_PARA_CHAR[2] = -ui->LE_CURRENT->text().toInt();    // current
    pLAN->SendCommand(cmd);
}

void TutorialDialog::on_BTN_L_RELEASE4_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = TUTORIAL_FINGER_CONTROL;
    cmd.COMMAND_TARGET = ALNum_Tuto;

    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = 1;     // left
    cmd.COMMAND_DATA.USER_PARA_CHAR[1] = 4;     // 4th finger
    cmd.COMMAND_DATA.USER_PARA_CHAR[2] = -ui->LE_CURRENT->text().toInt();    // current
    pLAN->SendCommand(cmd);
}

void TutorialDialog::on_BTN_L_RELEASE_ALL_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = TUTORIAL_FINGER_CONTROL;
    cmd.COMMAND_TARGET = ALNum_Tuto;

    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = 1;     // left
    cmd.COMMAND_DATA.USER_PARA_CHAR[1] = -1;    // all finger
    cmd.COMMAND_DATA.USER_PARA_CHAR[2] = -ui->LE_CURRENT->text().toInt();    // current
    pLAN->SendCommand(cmd);
}



void TutorialDialog::on_BTN_INIT_RHAND_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = DAEMON_INIT_FIND_HOME;
    cmd.COMMAND_TARGET = RBCORE_PODO_NO;

    cmd.COMMAND_DATA.USER_PARA_INT[0] = 999;    // special init for hand
    cmd.COMMAND_DATA.USER_PARA_INT[1] = 1;      // right hand
    pLAN->SendCommand(cmd);
}

void TutorialDialog::on_BTN_INIT_LHAND_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = DAEMON_INIT_FIND_HOME;
    cmd.COMMAND_TARGET = RBCORE_PODO_NO;

    cmd.COMMAND_DATA.USER_PARA_INT[0] = 999;    // special init for hand
    cmd.COMMAND_DATA.USER_PARA_INT[1] = 2;      // left hand
    pLAN->SendCommand(cmd);
}

void TutorialDialog::on_BTN_INIT_ALLHAND_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = DAEMON_INIT_FIND_HOME;
    cmd.COMMAND_TARGET = RBCORE_PODO_NO;

    cmd.COMMAND_DATA.USER_PARA_INT[0] = 999;    // special init for hand
    cmd.COMMAND_DATA.USER_PARA_INT[1] = 3;      // both hand
    pLAN->SendCommand(cmd);
}



void TutorialDialog::on_BTN_READ_MODI_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = DAEMON_INIT_SET_FINGER_MODIFIER;
    cmd.COMMAND_TARGET = RBCORE_PODO_NO;

    cmd.COMMAND_DATA.USER_PARA_INT[0] = 1;
    pLAN->SendCommand(cmd);

    modiCnt = 0;
}

void TutorialDialog::on_BTN_SET_MODI_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = DAEMON_INIT_SET_FINGER_MODIFIER;
    cmd.COMMAND_TARGET = RBCORE_PODO_NO;

    cmd.COMMAND_DATA.USER_PARA_INT[0] = 0;

    cmd.COMMAND_DATA.USER_PARA_FLOAT[0] = ui->LE_MODI_R0->text().toFloat();
    cmd.COMMAND_DATA.USER_PARA_FLOAT[1] = ui->LE_MODI_R1->text().toFloat();
    cmd.COMMAND_DATA.USER_PARA_FLOAT[2] = ui->LE_MODI_R2->text().toFloat();
    cmd.COMMAND_DATA.USER_PARA_FLOAT[3] = ui->LE_MODI_R3->text().toFloat();
    cmd.COMMAND_DATA.USER_PARA_FLOAT[4] = ui->LE_MODI_R4->text().toFloat();

    cmd.COMMAND_DATA.USER_PARA_FLOAT[5] = ui->LE_MODI_L0->text().toFloat();
    cmd.COMMAND_DATA.USER_PARA_FLOAT[6] = ui->LE_MODI_L1->text().toFloat();
    cmd.COMMAND_DATA.USER_PARA_FLOAT[7] = ui->LE_MODI_L2->text().toFloat();
    cmd.COMMAND_DATA.USER_PARA_FLOAT[8] = ui->LE_MODI_L3->text().toFloat();
    cmd.COMMAND_DATA.USER_PARA_FLOAT[9] = ui->LE_MODI_L4->text().toFloat();

    pLAN->SendCommand(cmd);
}

void TutorialDialog::on_BTN_QP_START_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = JS_QP_START;
    cmd.COMMAND_TARGET = ALNum_Tuto;

    pLAN->SendCommand(cmd);

}

void TutorialDialog::on_BTN_QP_STOP_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = JS_QP_STOP;
    cmd.COMMAND_TARGET = ALNum_Tuto;

    pLAN->SendCommand(cmd);

}

void TutorialDialog::on_BTN_TEST_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = JS_TEST;
    cmd.COMMAND_TARGET = ALNum_Tuto;

    pLAN->SendCommand(cmd);

}

void TutorialDialog::on_BTN_IMU_NULL_clicked()
{
    // IMU enable
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = DAEMON_SENSOR_IMU_NULL;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = 0;
    cmd.COMMAND_TARGET = RBCORE_PODO_NO;

    pLAN->SendCommand(cmd);
}

void TutorialDialog::on_BTN_INIT_POS_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = JS_INIT_POS;
    cmd.COMMAND_TARGET = ALNum_Tuto;

    pLAN->SendCommand(cmd);

}

void TutorialDialog::on_BTN_TCP_CONNECT_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = JS_TCP_CONNECT;
    cmd.COMMAND_TARGET = ALNum_Tuto;

    pLAN->SendCommand(cmd);

}
