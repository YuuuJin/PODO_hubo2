#include "SensorDialog.h"
#include "ui_SensorDialog.h"

SensorDialog::SensorDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::SensorDialog)
{
    ui->setupUi(this);

    connect(pLAN, SIGNAL(NewPODOData()), this, SLOT(UpdateSensors()));
}

SensorDialog::~SensorDialog()
{
    delete ui;
}


void SensorDialog::UpdateSensors(){
    QString str;

    // FT ======
    ui->LE_SENSOR_RAFT_MX->setText(str.sprintf("%.2f", PODO_DATA.CoreSHM.FT[0].Mx));
    ui->LE_SENSOR_RAFT_MY->setText(str.sprintf("%.2f", PODO_DATA.CoreSHM.FT[0].My));
    ui->LE_SENSOR_RAFT_MZ->setText(str.sprintf("%.2f", PODO_DATA.CoreSHM.FT[0].Mz));
    ui->LE_SENSOR_RAFT_FX->setText(str.sprintf("%.2f", PODO_DATA.CoreSHM.FT[0].Fx));
    ui->LE_SENSOR_RAFT_FY->setText(str.sprintf("%.2f", PODO_DATA.CoreSHM.FT[0].Fy));
    ui->LE_SENSOR_RAFT_FZ->setText(str.sprintf("%.2f", PODO_DATA.CoreSHM.FT[0].Fz));

    ui->LE_SENSOR_LAFT_MX->setText(str.sprintf("%.2f", PODO_DATA.CoreSHM.FT[1].Mx));
    ui->LE_SENSOR_LAFT_MY->setText(str.sprintf("%.2f", PODO_DATA.CoreSHM.FT[1].My));
    ui->LE_SENSOR_LAFT_MZ->setText(str.sprintf("%.2f", PODO_DATA.CoreSHM.FT[1].Mz));
    ui->LE_SENSOR_LAFT_FX->setText(str.sprintf("%.2f", PODO_DATA.CoreSHM.FT[1].Fx));
    ui->LE_SENSOR_LAFT_FY->setText(str.sprintf("%.2f", PODO_DATA.CoreSHM.FT[1].Fy));
    ui->LE_SENSOR_LAFT_FZ->setText(str.sprintf("%.2f", PODO_DATA.CoreSHM.FT[1].Fz));

    ui->LE_SENSOR_RWFT_MX->setText(str.sprintf("%.2f", PODO_DATA.CoreSHM.FT[2].Mx));
    ui->LE_SENSOR_RWFT_MY->setText(str.sprintf("%.2f", PODO_DATA.CoreSHM.FT[2].My));
    ui->LE_SENSOR_RWFT_MZ->setText(str.sprintf("%.2f", PODO_DATA.CoreSHM.FT[2].Mz));
    ui->LE_SENSOR_RWFT_FX->setText(str.sprintf("%.2f", PODO_DATA.CoreSHM.FT[2].Fx));
    ui->LE_SENSOR_RWFT_FY->setText(str.sprintf("%.2f", PODO_DATA.CoreSHM.FT[2].Fy));
    ui->LE_SENSOR_RWFT_FZ->setText(str.sprintf("%.2f", PODO_DATA.CoreSHM.FT[2].Fz));

    ui->LE_SENSOR_LWFT_MX->setText(str.sprintf("%.2f", PODO_DATA.CoreSHM.FT[3].Mx));
    ui->LE_SENSOR_LWFT_MY->setText(str.sprintf("%.2f", PODO_DATA.CoreSHM.FT[3].My));
    ui->LE_SENSOR_LWFT_MZ->setText(str.sprintf("%.2f", PODO_DATA.CoreSHM.FT[3].Mz));
    ui->LE_SENSOR_LWFT_FX->setText(str.sprintf("%.2f", PODO_DATA.CoreSHM.FT[3].Fx));
    ui->LE_SENSOR_LWFT_FY->setText(str.sprintf("%.2f", PODO_DATA.CoreSHM.FT[3].Fy));
    ui->LE_SENSOR_LWFT_FZ->setText(str.sprintf("%.2f", PODO_DATA.CoreSHM.FT[3].Fz));

    // IMU ======
    ui->LE_SENSOR_CIMU_ROLL->setText(str.sprintf("%.2f", PODO_DATA.CoreSHM.IMU[0].Roll));
    ui->LE_SENSOR_CIMU_PITCH->setText(str.sprintf("%.2f", PODO_DATA.CoreSHM.IMU[0].Pitch));
    ui->LE_SENSOR_CIMU_ROLL_VEL->setText(str.sprintf("%.2f", PODO_DATA.CoreSHM.IMU[0].RollVel));
    ui->LE_SENSOR_CIMU_PITCH_VEL->setText(str.sprintf("%.2f", PODO_DATA.CoreSHM.IMU[0].PitchVel));
    ui->LE_SENSOR_CIMU_ROLL_ACC->setText(str.sprintf("%.2f", PODO_DATA.CoreSHM.IMU[0].AccX));
    ui->LE_SENSOR_CIMU_PITCH_ACC->setText(str.sprintf("%.2f", PODO_DATA.CoreSHM.IMU[0].AccY));
}

void SensorDialog::on_BTN_SENSOR_ENABLE_clicked(){
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = 1;     // on
    cmd.COMMAND_DATA.USER_COMMAND = DAEMON_SENSOR_SENSOR_ONOFF;
    cmd.COMMAND_TARGET = RBCORE_PODO_NO;
    pLAN->SendCommand(cmd);
}

void SensorDialog::on_BTN_SENSOR_DISABLE_clicked(){
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = 0;     // off
    cmd.COMMAND_DATA.USER_COMMAND = DAEMON_SENSOR_SENSOR_ONOFF;
    cmd.COMMAND_TARGET = RBCORE_PODO_NO;
    pLAN->SendCommand(cmd);
}

void SensorDialog::on_BTN_SENSOR_FT_NULL_clicked(){
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = -1;     // all
    cmd.COMMAND_DATA.USER_COMMAND = DAEMON_SENSOR_FT_NULL;
    cmd.COMMAND_TARGET = RBCORE_PODO_NO;
    pLAN->SendCommand(cmd);
}

void SensorDialog::on_BTN_SENSOR_IMU_NULL_clicked(){
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = -1;	// all
    cmd.COMMAND_DATA.USER_COMMAND = DAEMON_SENSOR_IMU_NULL;
    cmd.COMMAND_TARGET = RBCORE_PODO_NO;
    pLAN->SendCommand(cmd);
}

void SensorDialog::on_BTN_CIMU_GET_OFFSET_clicked(){

}

void SensorDialog::on_BTN_CIMU_SET_OFFSET_clicked(){
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = 0;	//CIMU
    cmd.COMMAND_DATA.USER_PARA_FLOAT[0] = ui->LE_SENSOR_CIMU_ROLL_OFFSET->text().toFloat();
    cmd.COMMAND_DATA.USER_PARA_FLOAT[1] = ui->LE_SENSOR_CIMU_PITCH_OFFSET->text().toFloat();
    cmd.COMMAND_DATA.USER_COMMAND = DAEMON_SENSOR_IMU_OFFSET_SET;
    cmd.COMMAND_TARGET = RBCORE_PODO_NO;
    pLAN->SendCommand(cmd);
}

void SensorDialog::on_BTN_NEW_IMU_ENABLE_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = DAEMON_SENSOR_IMU_NULL;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = 1;
    cmd.COMMAND_TARGET = RBCORE_PODO_NO;
    pLAN->SendCommand(cmd);
}

void SensorDialog::on_BTN_NEW_IMU_NULL_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = DAEMON_SENSOR_IMU_NULL;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = 2;
    cmd.COMMAND_TARGET = RBCORE_PODO_NO;
    pLAN->SendCommand(cmd);
}

void SensorDialog::on_BTN_NEW_IMU_RESET_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = DAEMON_SENSOR_IMU_NULL;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = 3;
    cmd.COMMAND_TARGET = RBCORE_PODO_NO;
    pLAN->SendCommand(cmd);
}
