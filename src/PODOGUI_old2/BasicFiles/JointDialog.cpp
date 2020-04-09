#include "JointDialog.h"
#include "ui_JointDialog.h"


inline void DisplayJointReference(int jnum, QLineEdit *edit){
    edit->setText(QString().sprintf("%.3f", PODO_DATA.CoreSHM.ENCODER[MC_GetID(jnum)][MC_GetCH(jnum)].CurrentReference));
}
inline void DisplayJointEncoder(int jnum, QLineEdit *edit){
    edit->setText(QString().sprintf("%.3f", PODO_DATA.CoreSHM.ENCODER[MC_GetID(jnum)][MC_GetCH(jnum)].CurrentPosition));
}

JointDialog::JointDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::JointDialog)
{
    ui->setupUi(this);

    connect(pLAN, SIGNAL(NewPODOData()), this, SLOT(UpdateJoints()));
}

JointDialog::~JointDialog()
{
    delete ui;
}

void JointDialog::UpdateJoints(){
    QString str;

    if(ui->RB_JOINT_REFERENCE->isChecked()){
        DisplayJointReference(RHY  , ui->LE_JOINT_RHY);
        DisplayJointReference(RHR  , ui->LE_JOINT_RHR);
        DisplayJointReference(RHP  , ui->LE_JOINT_RHP);
        DisplayJointReference(RKN  , ui->LE_JOINT_RKN);
        DisplayJointReference(RAP  , ui->LE_JOINT_RAP);
        DisplayJointReference(RAR  , ui->LE_JOINT_RAR);

        DisplayJointReference(RSP  , ui->LE_JOINT_RSP);
        DisplayJointReference(RSR  , ui->LE_JOINT_RSR);
        DisplayJointReference(RSY  , ui->LE_JOINT_RSY);
        DisplayJointReference(REB  , ui->LE_JOINT_REB);
        DisplayJointReference(RWY  , ui->LE_JOINT_RWY);
        DisplayJointReference(RWP  , ui->LE_JOINT_RWP);

        DisplayJointReference(LHY  , ui->LE_JOINT_LHY);
        DisplayJointReference(LHR  , ui->LE_JOINT_LHR);
        DisplayJointReference(LHP  , ui->LE_JOINT_LHP);
        DisplayJointReference(LKN  , ui->LE_JOINT_LKN);
        DisplayJointReference(LAP  , ui->LE_JOINT_LAP);
        DisplayJointReference(LAR  , ui->LE_JOINT_LAR);

        DisplayJointReference(LSP  , ui->LE_JOINT_LSP);
        DisplayJointReference(LSR  , ui->LE_JOINT_LSR);
        DisplayJointReference(LSY  , ui->LE_JOINT_LSY);
        DisplayJointReference(LEB  , ui->LE_JOINT_LEB);
        DisplayJointReference(LWY  , ui->LE_JOINT_LWY);
        DisplayJointReference(LWP  , ui->LE_JOINT_LWP);

        DisplayJointReference(WST  , ui->LE_JOINT_WST);

        DisplayJointReference(NKY  , ui->LE_JOINT_NKY);
        DisplayJointReference(NK1  , ui->LE_JOINT_NK1);
        DisplayJointReference(NK2  , ui->LE_JOINT_NK2);

        DisplayJointReference(RF1  , ui->LE_JOINT_RF1);
        DisplayJointReference(RF2  , ui->LE_JOINT_RF2);
        DisplayJointReference(RF3  , ui->LE_JOINT_RF3);
        DisplayJointReference(RF4  , ui->LE_JOINT_RF4);
        DisplayJointReference(RF5  , ui->LE_JOINT_RF5);

        DisplayJointReference(LF1  , ui->LE_JOINT_LF1);
        DisplayJointReference(LF2  , ui->LE_JOINT_LF2);
        DisplayJointReference(LF3  , ui->LE_JOINT_LF3);
        DisplayJointReference(LF4  , ui->LE_JOINT_LF4);
        DisplayJointReference(LF5  , ui->LE_JOINT_LF5);
    }else{
        DisplayJointEncoder(RHY  , ui->LE_JOINT_RHY);
        DisplayJointEncoder(RHR  , ui->LE_JOINT_RHR);
        DisplayJointEncoder(RHP  , ui->LE_JOINT_RHP);
        DisplayJointEncoder(RKN  , ui->LE_JOINT_RKN);
        DisplayJointEncoder(RAP  , ui->LE_JOINT_RAP);
        DisplayJointEncoder(RAR  , ui->LE_JOINT_RAR);

        DisplayJointEncoder(RSP  , ui->LE_JOINT_RSP);
        DisplayJointEncoder(RSR  , ui->LE_JOINT_RSR);
        DisplayJointEncoder(RSY  , ui->LE_JOINT_RSY);
        DisplayJointEncoder(REB  , ui->LE_JOINT_REB);
        DisplayJointEncoder(RWY  , ui->LE_JOINT_RWY);
        DisplayJointEncoder(RWP  , ui->LE_JOINT_RWP);

        DisplayJointEncoder(LHY  , ui->LE_JOINT_LHY);
        DisplayJointEncoder(LHR  , ui->LE_JOINT_LHR);
        DisplayJointEncoder(LHP  , ui->LE_JOINT_LHP);
        DisplayJointEncoder(LKN  , ui->LE_JOINT_LKN);
        DisplayJointEncoder(LAP  , ui->LE_JOINT_LAP);
        DisplayJointEncoder(LAR  , ui->LE_JOINT_LAR);

        DisplayJointEncoder(LSP  , ui->LE_JOINT_LSP);
        DisplayJointEncoder(LSR  , ui->LE_JOINT_LSR);
        DisplayJointEncoder(LSY  , ui->LE_JOINT_LSY);
        DisplayJointEncoder(LEB  , ui->LE_JOINT_LEB);
        DisplayJointEncoder(LWY  , ui->LE_JOINT_LWY);
        DisplayJointEncoder(LWP  , ui->LE_JOINT_LWP);

        DisplayJointEncoder(WST  , ui->LE_JOINT_WST);

        DisplayJointEncoder(NKY  , ui->LE_JOINT_NKY);
        DisplayJointEncoder(NK1  , ui->LE_JOINT_NK1);
        DisplayJointEncoder(NK2  , ui->LE_JOINT_NK2);

        DisplayJointEncoder(RF1  , ui->LE_JOINT_RF1);
        DisplayJointEncoder(RF2  , ui->LE_JOINT_RF2);
        DisplayJointEncoder(RF3  , ui->LE_JOINT_RF3);
        DisplayJointEncoder(RF4  , ui->LE_JOINT_RF4);
        DisplayJointEncoder(RF5  , ui->LE_JOINT_RF5);

        DisplayJointEncoder(LF1  , ui->LE_JOINT_LF1);
        DisplayJointEncoder(LF2  , ui->LE_JOINT_LF2);
        DisplayJointEncoder(LF3  , ui->LE_JOINT_LF3);
        DisplayJointEncoder(LF4  , ui->LE_JOINT_LF4);
        DisplayJointEncoder(LF5  , ui->LE_JOINT_LF5);
    }
}

void JointDialog::on_BTN_ENC_ENABLE_clicked(){
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = 1;     // on
    cmd.COMMAND_DATA.USER_COMMAND = DAEMON_SENSOR_ENCODER_ONOFF;
    cmd.COMMAND_TARGET = RBCORE_PODO_NO;
    pLAN->SendCommand(cmd);
}

void JointDialog::on_BTN_ENC_DISABLE_clicked(){
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = 0;     // off
    cmd.COMMAND_DATA.USER_COMMAND = DAEMON_SENSOR_ENCODER_ONOFF;
    cmd.COMMAND_TARGET = RBCORE_PODO_NO;
    pLAN->SendCommand(cmd);
}
