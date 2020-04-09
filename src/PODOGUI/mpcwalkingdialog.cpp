#include "mpcwalkingdialog.h"
#include "ui_mpcwalkingdialog.h"
#include "BasicFiles/PODOALDialog.h"


enum TUTORIAL_COMMAND
{
    TUTORIAL_NO_ACT = 100,
    FORWARD_WALKING,
    RIGHT_WALKING,
    LEFT_WALKING,
    CCW_WALKING,
    CW_WALKING,
    JOY_STICK_WALKING_START,
    JOY_STICK_WALKING_PARA_CHANGE,
    JOY_STICK_WALKING_STOP,
    LIM_WALKING,
    DATA_SAVE,
    JOY_STICK_WALK_READY,
    JOY_STICK_WALK_START
};
enum WALKREADYCOMMAND
{
    WALKREADY_NO_ACT = 100,
    WALKREADY_GO_WALKREADYPOS,
    WALKREADY_GO_HOMEPOS,
    WALKREADY_WHEELCMD,
    WALKREADY_INFINITY
};


MPCWalkingDialog::MPCWalkingDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::MPCWalkingDialog)
{
    ui->setupUi(this);

    alNumMPCWalking = PODOALDialog::GetALNumFromFileName("MPCWalking");
    alNumWalkReady = PODOALDialog::GetALNumFromFileName("WalkReady");

    displayTimer = new QTimer(this);
    connect(displayTimer, SIGNAL(timeout()), this, SLOT(DisplayUpdate()));
    displayTimer->start(50);//50
}

MPCWalkingDialog::~MPCWalkingDialog()
{
    delete ui;
}

void MPCWalkingDialog::on_BTN_Walk_Ready_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_TARGET = alNumWalkReady;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = 0;
    cmd.COMMAND_DATA.USER_COMMAND = WALKREADY_GO_WALKREADYPOS;

    pLAN->SendCommand(cmd);
}

void MPCWalkingDialog::on_BTN_FORWARD_clicked()
{
    USER_COMMAND cmd;

    pLAN->G2MData->StepNum = ui->LE_STEP_NUM->text().toInt();
    pLAN->G2MData->StepLength = ui->LE_STEP_LENGTH->text().toDouble();
    pLAN->G2MData->StepAngle = ui->LE_STEP_ANGLE->text().toDouble();
    pLAN->G2MData->StepOffset = ui->LE_STEP_OFFSET->text().toDouble();


    cmd.COMMAND_TARGET = alNumMPCWalking;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = 0;
    cmd.COMMAND_DATA.USER_COMMAND = FORWARD_WALKING;

    pLAN->SendCommand(cmd);
}

void MPCWalkingDialog::on_BTN_LEFT_clicked()
{
    USER_COMMAND cmd;
    pLAN->G2MData->StepNum = ui->LE_STEP_NUM->text().toInt();
    pLAN->G2MData->StepLength = ui->LE_STEP_LENGTH->text().toDouble();
    pLAN->G2MData->StepAngle = ui->LE_STEP_ANGLE->text().toDouble();
    pLAN->G2MData->StepOffset = ui->LE_STEP_OFFSET->text().toDouble();

    cmd.COMMAND_TARGET = alNumMPCWalking;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = 0;
    cmd.COMMAND_DATA.USER_COMMAND = LEFT_WALKING;

    pLAN->SendCommand(cmd);
}

void MPCWalkingDialog::on_BTN_RIGHT_clicked()
{
    USER_COMMAND cmd;
    pLAN->G2MData->StepNum = ui->LE_STEP_NUM->text().toInt();
    pLAN->G2MData->StepLength = ui->LE_STEP_LENGTH->text().toDouble();
    pLAN->G2MData->StepAngle = ui->LE_STEP_ANGLE->text().toDouble();
    pLAN->G2MData->StepOffset = ui->LE_STEP_OFFSET->text().toDouble();

    cmd.COMMAND_TARGET = alNumMPCWalking;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = 0;
    cmd.COMMAND_DATA.USER_COMMAND = RIGHT_WALKING;

    pLAN->SendCommand(cmd);
}

void MPCWalkingDialog::on_BTN_CCW_clicked()
{
    USER_COMMAND cmd;
    pLAN->G2MData->StepNum = ui->LE_STEP_NUM->text().toInt();
    pLAN->G2MData->StepLength = ui->LE_STEP_LENGTH->text().toDouble();
    pLAN->G2MData->StepAngle = ui->LE_STEP_ANGLE->text().toDouble();
    pLAN->G2MData->StepOffset = ui->LE_STEP_OFFSET->text().toDouble();

    cmd.COMMAND_TARGET = alNumMPCWalking;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = 0;
    cmd.COMMAND_DATA.USER_COMMAND = CCW_WALKING;

    pLAN->SendCommand(cmd);
}

void MPCWalkingDialog::on_BTN_CW_clicked()
{
    USER_COMMAND cmd;
    pLAN->G2MData->StepNum = ui->LE_STEP_NUM->text().toInt();
    pLAN->G2MData->StepLength = ui->LE_STEP_LENGTH->text().toDouble();
    pLAN->G2MData->StepAngle = ui->LE_STEP_ANGLE->text().toDouble();
    pLAN->G2MData->StepOffset = ui->LE_STEP_OFFSET->text().toDouble();

    cmd.COMMAND_TARGET = alNumMPCWalking;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = 0;
    cmd.COMMAND_DATA.USER_COMMAND = CW_WALKING;

    pLAN->SendCommand(cmd);

}

void MPCWalkingDialog::on_BTN_SAVE_START_clicked()
{
    USER_COMMAND cmd;

     cmd.COMMAND_TARGET = alNumMPCWalking;
     cmd.COMMAND_DATA.USER_PARA_INT[0] = 1;
     cmd.COMMAND_DATA.USER_COMMAND = DATA_SAVE;

     pLAN->SendCommand(cmd);
}

void MPCWalkingDialog::on_BTN_DATA_SAVE_clicked()
{
    USER_COMMAND cmd;

    cmd.COMMAND_TARGET = alNumMPCWalking;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = 0;
    cmd.COMMAND_DATA.USER_COMMAND = DATA_SAVE;

    pLAN->SendCommand(cmd);
}

void MPCWalkingDialog::on_pushButton_8_clicked()
{

}
