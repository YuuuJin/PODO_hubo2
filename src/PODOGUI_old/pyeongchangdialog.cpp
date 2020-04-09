#include "pyeongchangdialog.h"
#include "ui_pyeongchangdialog.h"

#include "BasicFiles/PODOALDialog.h"

#define PYEONGCHANG_DEMO        88
#define SOUND_PATH          "../share/Wave/"

PyeongchangDialog::PyeongchangDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::PyeongchangDialog)
{
    ui->setupUi(this);
    ALNum_Pyeongchang      = PODOALDialog::GetALNumFromFileName("PyeongchangDemo");
    alNumWalkReady         = PODOALDialog::GetALNumFromFileName("WalkReady");
    openALnum = 0;


    ui->btn_hand_lg->setVisible(false);
    ui->btn_hand_hg->setVisible(false);
    ui->LE_CLAP_CNT->setText("5");

    ui->btn_clap_korea->setVisible(false);
    ui->btn_clap_one->setVisible(false);
    ui->btn_clap_three->setVisible(false);
    ui->btn_clap_init->setVisible(false);
    ui->btn_head_init->setVisible(false);

    demoSound[1] = new QSound("../share/Wave/handshake_eng.wav");
    demoSound[11] = new QSound("../share/Wave/handshake_kor.wav");

    demoSound[2] = new QSound("../share/Wave/bouquet_eng.wav");
    demoSound[12] = new QSound("../share/Wave/bouquet_kor.wav");

    demoSound[4] = new QSound("../share/Wave/greeting_eng.wav");
    demoSound[14] = new QSound("../share/Wave/greeting_kor.wav");

    demoSound[3] = new QSound("../share/Wave/bow_eng.wav");
    demoSound[13] = new QSound("../share/Wave/bow_kor.wav");

    demoSound[5] = new QSound("../share/Wave/head_ok_eng.wav");
    demoSound[15] = new QSound("../share/Wave/head_ok_kor.wav");

    demoSound[6] = new QSound("../share/Wave/head_no_eng.wav");
    demoSound[16] = new QSound("../share/Wave/head_no_kor.wav");

    demoSound[7] = new QSound("../share/Wave/neck_eng.wav");
    demoSound[17] = new QSound("../share/Wave/neck_kor.wav");

    _sound = ENGLISH;

}

PyeongchangDialog::~PyeongchangDialog()
{
    delete ui;
}

void PyeongchangDialog::GeneralCommand(int motion_num, int sound_num){
    if(motion_num >= 0){
        USER_COMMAND cmd;
        cmd.COMMAND_TARGET = ALNum_Pyeongchang;
        cmd.COMMAND_DATA.USER_COMMAND = PYEONGCHANG_DEMO;
        cmd.COMMAND_DATA.USER_PARA_INT[3] = motion_num;
        pLAN->SendCommand(cmd);
    }
    if(sound_num > 0){
        if(_sound == KOREAN){
            demoSound[sound_num + 10]->play();
        }
        else if(_sound == ENGLISH){
            demoSound[sound_num]->play();
        }
    }
}
void PyeongchangDialog::WalkreadyCommand(int motion_num, int sound_num){
    if(motion_num >= 0){
        USER_COMMAND cmd;
        cmd.COMMAND_TARGET = alNumWalkReady;
        cmd.COMMAND_DATA.USER_COMMAND = motion_num;
        cmd.COMMAND_DATA.USER_PARA_INT[0] = 0;
        pLAN->SendCommand(cmd);
    }
}
void PyeongchangDialog::OpenAlCommand(int Al_num){
    if(Al_num >= 0){
        USER_COMMAND cmd;
        cmd.COMMAND_DATA.USER_PARA_INT[0] = Al_num;
        cmd.COMMAND_DATA.USER_COMMAND = DAEMON_PROCESS_CREATE;
        cmd.COMMAND_TARGET = RBCORE_PODO_NO;
        pLAN->SendCommand(cmd);
    }
}
void PyeongchangDialog::on_btn_motion1_clicked(){
    GeneralCommand(TASK_HANDSHAKE,0);
    ui->btn_motion1->setDisabled(true);
}void PyeongchangDialog::on_btn_motion2_clicked(){
    GeneralCommand(TASK_BOUQUET,0);
    ui->btn_motion2->setDisabled(true);
}void PyeongchangDialog::on_btn_motion3_clicked(){
    GeneralCommand(TASK_MOTION3,0);
    ui->btn_motion3->setDisabled(true);
}void PyeongchangDialog::on_btn_motion4_clicked(){
    GeneralCommand(TASK_MOTION4,0);
    ui->btn_motion4->setDisabled(true);
}void PyeongchangDialog::on_btn_motion5_clicked(){
    GeneralCommand(TASK_MOTION5,0);
    ui->btn_motion5->setDisabled(true);
}void PyeongchangDialog::on_btn_motion6_clicked(){
    GeneralCommand(TASK_MOTION6,0);
    ui->btn_motion6->setDisabled(true);
}void PyeongchangDialog::on_btn_motion7_clicked(){
    GeneralCommand(TASK_MOTION7,0);
    ui->btn_motion7->setDisabled(true);
}void PyeongchangDialog::on_btn_motion8_clicked(){
    GeneralCommand(TASK_MOTION8,0);
    ui->btn_motion8->setDisabled(true);
}void PyeongchangDialog::on_btn_motion9_clicked(){
    GeneralCommand(TASK_MOTION9,0);
    ui->btn_motion9->setDisabled(true);
}void PyeongchangDialog::on_btn_motion10_clicked(){
    GeneralCommand(TASK_MOTION10,0);
    ui->btn_motion10->setDisabled(true);
}void PyeongchangDialog::on_btn_enable_clicked(){
    ui->btn_motion1->setEnabled(true);
    ui->btn_motion2->setEnabled(true);
    ui->btn_motion3->setEnabled(true);
    ui->btn_motion4->setEnabled(true);
    ui->btn_motion5->setEnabled(true);
    ui->btn_motion6->setEnabled(true);
    ui->btn_motion7->setEnabled(true);
    ui->btn_motion8->setEnabled(true);
    ui->btn_motion9->setEnabled(true);
    ui->btn_motion10->setEnabled(true);
}void PyeongchangDialog::on_btn_disable_clicked(){
    ui->btn_motion1->setDisabled(true);
    ui->btn_motion2->setDisabled(true);
    ui->btn_motion3->setDisabled(true);
    ui->btn_motion4->setDisabled(true);
    ui->btn_motion5->setDisabled(true);
    ui->btn_motion6->setDisabled(true);
    ui->btn_motion7->setDisabled(true);
    ui->btn_motion8->setDisabled(true);
    ui->btn_motion9->setDisabled(true);
    ui->btn_motion10->setDisabled(true);
}void PyeongchangDialog::on_btn_Test1_clicked(){
    GeneralCommand(TEST_1,0);
}void PyeongchangDialog::on_btn_Test2_clicked(){
    GeneralCommand(TEST_2,0);
}void PyeongchangDialog::on_btn_hand_ready_clicked(){
    GeneralCommand(HANDSHAKE_READY,0);
}void PyeongchangDialog::on_btn_hand_lg_clicked(){
    GeneralCommand(HANDSHAKE_LOWGAIN,0);
}void PyeongchangDialog::on_btn_hand_hg_clicked(){
    GeneralCommand(HANDSHAKE_HIGHGAIN,0);
}void PyeongchangDialog::on_btn_hand_init_clicked(){
    GeneralCommand(HANDSHAKE_INIT,0);
}void PyeongchangDialog::on_btn_bouquet_ready_clicked(){
    GeneralCommand(BOUQUET_READY,0);
}void PyeongchangDialog::on_btn_bouquet_give_clicked(){
    GeneralCommand(BOUQUET_GIVE,2);
}void PyeongchangDialog::on_btn_bouquet_init_clicked(){
    GeneralCommand(BOUQUET_INIT,0);
}void PyeongchangDialog::on_btn_hand_gon_clicked(){
    GeneralCommand(HANDSHAKE_GRIP_ON,1);
}void PyeongchangDialog::on_btn_hand_goff_clicked(){
    GeneralCommand(HANDSHAKE_GRIP_OFF,0);
}void PyeongchangDialog::on_btn_clap_ready_clicked(){
    USER_COMMAND cmd;
    cmd.COMMAND_TARGET = ALNum_Pyeongchang;
    cmd.COMMAND_DATA.USER_COMMAND = PYEONGCHANG_DEMO;
    cmd.COMMAND_DATA.USER_PARA_INT[3] = CLAP_READY;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = ui->LE_CLAP_CNT->text().toInt();
    pLAN->SendCommand(cmd);
}void PyeongchangDialog::on_btn_clap_one_clicked(){
    GeneralCommand(CLAP_ONE,0);
}void PyeongchangDialog::on_btn_clap_three_clicked(){
    GeneralCommand(CLAP_THREE,0);
}void PyeongchangDialog::on_btn_clap_korea_clicked(){
    GeneralCommand(CLAP_KOREA,0);
}void PyeongchangDialog::on_btn_clap_init_clicked(){
    GeneralCommand(CLAP_INIT,0);
}void PyeongchangDialog::on_btn_head_yes_clicked(){
    GeneralCommand(HEAD_YES,5);
}void PyeongchangDialog::on_btn_head_no_clicked(){
    GeneralCommand(HEAD_NO,6);
}void PyeongchangDialog::on_btn_head_stretching_clicked(){
    GeneralCommand(HEAD_STRETCHING,7);
}void PyeongchangDialog::on_btn_head_init_clicked(){
    GeneralCommand(HEAD_INIT,0);
}void PyeongchangDialog::on_btn_bow_one_clicked(){
    GeneralCommand(BOW_ONE,3);
}void PyeongchangDialog::on_btn_bow_two_clicked(){
    GeneralCommand(BOW_TWO,3);
}void PyeongchangDialog::on_btn_bow_two_w_clicked(){
    GeneralCommand(BOW_TWO_W,3);
}void PyeongchangDialog::on_btn_bow_greeting_clicked(){
    GeneralCommand(BOW_GREETING,4);
}void PyeongchangDialog::on_btn_openal_clicked(){
    if(openALnum == 0){
        OpenAlCommand(ALNum_Pyeongchang);
        openALnum = 1;
        ui->btn_openal->setText("Open Pyeongchang");
    }
    else if(openALnum == 1){
        OpenAlCommand(alNumWalkReady);
        ui->btn_openal->setText("Open Walkready");
        openALnum = 0;
    }
}void PyeongchangDialog::on_btn_walkready_clicked(){
    WalkreadyCommand(WALKREADY_GO_WALKREADYPOS,0);

}void PyeongchangDialog::on_btn_homepos_clicked(){
    WalkreadyCommand(WALKREADY_GO_HOMEPOS,0);
}

void PyeongchangDialog::on_btn_leg_mode_clicked()
{
    if(ui->btn_leg_mode->text() == "English"){
        ui->btn_leg_mode->setText("Korean");
        _sound = KOREAN;
    }
    else if(ui->btn_leg_mode->text() == "Korean"){
        ui->btn_leg_mode->setText("English");
        _sound = ENGLISH;
    }
}
