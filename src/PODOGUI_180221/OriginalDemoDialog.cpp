#include "OriginalDemoDialog.h"
#include "ui_OriginalDemoDialog.h"

#include "BasicFiles/PODOALDialog.h"
#include <iostream>

#define SCI_DEMO                101
#define SCI_DEMO_Control_on                102
#define SCI_DEMO_Control_off               103
#define SCI_DEMO_DATA_SAVE               104


#define PYEONGCHANG_DEMO        88
#define SOUND_PATH          "../share/Wave/"
#define Normal_walking_speed 0.9
#define Pyeongchang_walking_speed 0.45

enum WALKREADYCOMMAND
{
    WALKREADY_NO_ACT = 100,
    WALKREADY_GO_WALKREADYPOS,
    WALKREADY_GO_HOMEPOS,
    WALKREADY_WHEELCMD,
    WALKREADY_INFINITY
};
enum FREEWALKCOMMAND
{
    FREEWALK_NO_ACT = 100,
    FREEWALK_CUR_DSP,
    FREEWALK_WALK,
    FREEWALK_SAVE,
    FREEWALK_CONTROL_TEST,
    FREEWALK_PRE_WALK,
    FREEWALK_MOTION_CHECK,
    FREEWALK_INITIALIZE,
    FREEWALK_TERRAIN,
    FREEWALK_ADDMASS,
    FREEWALK_WIDE,
    FREEWALK_DSP_HOLD_WALK,
    FREEWALK_STOP,
    FREEWALK_PYEONGCHANG
};
enum WalkingStopModeCommand{
    COMPLETE_STOP_WALKING = 0,
    SPREAD_STOP_WALKING
};
enum Inside_OutsideMode
{
    INSIDE_WALKING = 0,

    OUTSIDE_WALKING
};
enum{
    NORMAL_WALKING =0,
    TERRAIN_WALKING,
    TERRAIN_WALKING_ONE_STEP,
    LADDER_WALKING,
    GOAL_WALKING
};
enum PYEONGCHANG_TEST_ENUM
{
    TEST_1 = 200,
    TEST_2,
    TEST_3,
    TEST_4,
    ENGLISH,
    KOREAN,
    NO_OF_TEST
};
enum PYEONGCHANG_MOTION_ENUM
{
    HANDSHAKE_READY = 10010,
    HANDSHAKE_LOWGAIN,
    HANDSHAKE_GRIP_ON,
    HANDSHAKE_GRIP_OFF,
    HANDSHAKE_HIGHGAIN,
    HANDSHAKE_INIT,
    NO_OF_HANDSHAKE_MOTION,
    BOUQUET_READY = 10020,
    BOUQUET_GIVE,
    BOUQUET_INIT,
    NO_OF_BOUQUET_MOTION,
    CLAP_READY = 10030,
    CLAP_ONE,
    CLAP_THREE,
    CLAP_KOREA,
    CLAP_INIT,
    NO_OF_CLAP_MOTION,
    HEAD_YES = 10040,
    HEAD_NO,
    HEAD_STRETCHING,
    HEAD_INIT,
    NO_OF_HEAD_MOTION,
    BOW_ONE = 10050,
    BOW_TWO,
    BOW_TWO_W,
    BOW_GREETING,
    BOW_LAST_MOTION,
    NO_OF_BOW_MOTION,
    GUID_LEFT = 10060,
    GUID_RIGHT,
    GUID_INIT,
    HIGHFIVE_DO,
    HIGHFIVE_INIT,
    DANCE_GOGOGO_READY = 12017,
    DANCE_GOGOGO,
    NO_OF_PYEONGCHANG_MOTION
};

using namespace std;
OriginalDemoDialog::OriginalDemoDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::OriginalDemoDialog)
{
    ui->setupUi(this);
    ALNum_Demo = PODOALDialog::GetALNumFromFileName("DemoOriginal");
    ALNumWalkReady      = PODOALDialog::GetALNumFromFileName("WalkReady");
    ALNum_Pyeongchang      = PODOALDialog::GetALNumFromFileName("PyeongchangDemo");
    alNumFreeWalking    = PODOALDialog::GetALNumFromFileName("FreeWalking");

    //   demoSound[0] = new QSound("../share/Wave/s2_1.wav");
    ////    demoSound[1] = new QSound("../share/Wave/s3_1.wav");
    //    demoSound[1] = new QSound("../share/Wave/bow_kor.wav");
    //    demoSound[2] = new QSound("../share/Wave/s11_1_kor.wav");

    //    demoSound[3] = new QSound("../share/Wave/s4_1.wav");
    //    demoSound[4] = new QSound("../share/Wave/s5_1.wav");
    //    demoSound[5] = new QSound("../share/Wave/s5_2.wav");
    //    demoSound[6] = new QSound("../share/Wave/s6_1.wav");
    //    demoSound[7] = new QSound("../share/Wave/s8_1.wav");
    //    demoSound[8] = new QSound("../share/Wave/s8_2.wav");
    //    demoSound[9] = new QSound("../share/Wave/s9_1.wav");
    //    demoSound[10] = new QSound("../share/Wave/s10_1.wav");
    //    demoSound[11] = new QSound("../share/Wave/s12_1.wav");
    ////    demoSound[12] = new QSound("../share/Wave/s17_1.wav");
    //    demoSound[12] = new QSound("../share/Wave/29_fighting_kor.wav");
    //    demoSound[13] = new QSound("../share/Wave/s18_1.wav");
    //    demoSound[14] = new QSound("../share/Wave/s19_1.wav");
    //    demoSound[15] = new QSound("../share/Wave/s20_1.wav");
    //    demoSound[16] = new QSound("../share/Wave/s21_1.wav");
    ////    demoSound[17] = new QSound("../share/Wave/s23_1.wav");
    //    demoSound[17] = new QSound("../share/Wave/Nouee.wav");
    //    demoSound[18] = new QSound("../share/Wave/s26_1.wav");
    //    demoSound[19] = new QSound("../share/Wave/s28_1.wav");
    //    demoSound[20] = new QSound("../share/Wave/s29_1.wav");
    //    demoSound[21] = new QSound("../share/Wave/s29_2.wav");
    //    demoSound[22] = new QSound("../share/Wave/s29_3.wav");
    //    demoSound[23] = new QSound("../share/Wave/s30_1.wav");
    //    demoSound[24] = new QSound("../share/Wave/s31_1.wav");
    //    demoSound[25] = new QSound("../share/Wave/s32_1.wav");
    //    demoSound[26] = new QSound("../share/Wave/s33_1.wav");

    ////    demoSound[27] = new QSound("../share/Wave/GangNamStyle.wav");
    //    demoSound[27] = new QSound("../share/Wave/GangNamStyle.wav");
    //    demoSound[28] = new QSound("../share/Wave/pickme_30sec.wav");

        demoSound[140] = new QSound("../share/Wave/GangNamStyle.wav");
        demoSound[141] = new QSound("../share/Wave/pickme_30sec.wav");

        demoSound[142] = new QSound("../share/Wave/s19_1.wav");
            demoSound[143] = new QSound("../share/Wave/s32_1.wav");


                demoSound[144] = new QSound("../share/Wave/Nouee.wav");

    //    demoSound[29] = new QSound("../share/Wave/s2_1_kor.wav");
    //    demoSound[29+50] = new QSound("../share/Wave/s2_1.wav");
    //    demoSound[30] = new QSound("../share/Wave/hihubo.wav");

    //    demoSound[50]      = new QSound("../share/hubo_pyeonchang/21_shake_gently_kor.wav");
    //    demoSound[50+50]   = new QSound("../share/hubo_pyeonchang/21_shake_gently_eng.wav");

    //    demoSound[51]   = new QSound("../share/hubo_pyeonchang/01_greeting_kor.wav");
    //    demoSound[51+50]   = new QSound("../share/hubo_pyeonchang/01_greeting_eng.wav");

    //    demoSound[52]   = new QSound("../share/hubo_pyeonchang/39_move_left_kor.wav");
    //    demoSound[53]   = new QSound("../share/hubo_pyeonchang/40_move_right_kor.wav");

    //    demoSound[52+50]   = new QSound("../share/hubo_pyeonchang/39_move_left_eng.wav");
    //    demoSound[53+50]   = new QSound("../share/hubo_pyeonchang/40_move_right_eng.wav");

    //    demoSound[54]   = new QSound("../share/Wave/bouquet_kor.wav");
    //    demoSound[54+50]   = new QSound("../share/Wave/bouquet_eng.wav");


    demoSound[1]   = new QSound("../share/hubo_pyeonchang/01_greeting_kor.wav");
    demoSound[2]   = new QSound("../share/hubo_pyeonchang/02_hello_kor.wav");
    demoSound[3]   = new QSound("../share/hubo_pyeonchang/03_name_kor.wav");
    demoSound[4]   = new QSound("../share/hubo_pyeonchang/04_hahaha_kor.wav");
    demoSound[5]   = new QSound("../share/hubo_pyeonchang/05_sad_kor.wav");
    demoSound[6]   = new QSound("../share/hubo_pyeonchang/06_ofcourse_kor.wav");
    demoSound[7]   = new QSound("../share/hubo_pyeonchang/07_1millon_kor.wav");
    demoSound[8]   = new QSound("../share/hubo_pyeonchang/08_finger_kor.wav");
    demoSound[9]   = new QSound("../share/hubo_pyeonchang/09_cute_kor.wav");
    demoSound[10]   = new QSound("../share/hubo_pyeonchang/10_love_kor.wav");
    demoSound[11]   = new QSound("../share/hubo_pyeonchang/11_hoot_kor.wav");
    demoSound[12]   = new QSound("../share/hubo_pyeonchang/12_bye_kor.wav");
    demoSound[13]   = new QSound("../share/hubo_pyeonchang/13_1stRobot_kor.wav");
    demoSound[14]   = new QSound("../share/hubo_pyeonchang/14_closer_kor.wav");
    demoSound[15]   = new QSound("../share/hubo_pyeonchang/15_curious_kor.wav");
    demoSound[16]   = new QSound("../share/hubo_pyeonchang/16_dance_kor.wav");
    demoSound[17]   = new QSound("../share/hubo_pyeonchang/17_enjoy_kor.wav");
    demoSound[18]   = new QSound("../share/hubo_pyeonchang/18_from_kor.wav");
    demoSound[19]   = new QSound("../share/hubo_pyeonchang/19_okay_kor.wav");
    demoSound[20]   = new QSound("../share/hubo_pyeonchang/20_shake_kor.wav");
    demoSound[21]   = new QSound("../share/hubo_pyeonchang/21_shake_gently_kor.wav");
    demoSound[22]   = new QSound("../share/hubo_pyeonchang/22_sorry_can't_do_kor.wav");
    demoSound[23]   = new QSound("../share/hubo_pyeonchang/23_walk_kor.wav");
    demoSound[24]   = new QSound("../share/hubo_pyeonchang/24_welcome_kor.wav");
    demoSound[25]   = new QSound("../share/hubo_pyeonchang/25_don't_know_kor.wav");
    demoSound[26]   = new QSound("../share/hubo_pyeonchang/26_eat_kor.wav");
    demoSound[27]   = new QSound("../share/hubo_pyeonchang/27_feeling_kor.wav");
    demoSound[28]   = new QSound("../share/hubo_pyeonchang/28_cheer_kor.wav");
    demoSound[29]   = new QSound("../share/hubo_pyeonchang/29_fighting_kor.wav");
    demoSound[30]   = new QSound("../share/hubo_pyeonchang/30_cold_kor.wav");
    demoSound[31]   = new QSound("../share/hubo_pyeonchang/31_picture_kor.wav");
    demoSound[32]   = new QSound("../share/hubo_pyeonchang/32_hero_wakeup_kor.wav");
    demoSound[33]   = new QSound("../share/hubo_pyeonchang/33_nerf_this_kor.wav");
    demoSound[34]   = new QSound("../share/hubo_pyeonchang/34_my_skill_time_kor.wav");
    demoSound[35]   = new QSound("../share/hubo_pyeonchang/35_online_kor.wav");
    demoSound[36]   = new QSound("../share/hubo_pyeonchang/36_yes_kor.wav");
    demoSound[37]   = new QSound("../share/hubo_pyeonchang/37_no_kor.wav");
    demoSound[38]   = new QSound("../share/hubo_pyeonchang/38_move_kor.wav");
    demoSound[39]   = new QSound("../share/hubo_pyeonchang/39_move_left_kor.wav");
    demoSound[40]   = new QSound("../share/hubo_pyeonchang/40_move_right_kor.wav");

    demoSound[1+50]   = new QSound("../share/hubo_pyeonchang/01_greeting_eng.wav");
    demoSound[2+50]   = new QSound("../share/hubo_pyeonchang/02_hello_eng.wav");
    demoSound[3+50]   = new QSound("../share/hubo_pyeonchang/03_name_eng.wav");
    demoSound[4+50]   = new QSound("../share/hubo_pyeonchang/04_hahaha_eng.wav");
    demoSound[5+50]   = new QSound("../share/hubo_pyeonchang/05_sad_eng.wav");
    demoSound[6+50]   = new QSound("../share/hubo_pyeonchang/06_ofcourse_eng.wav");
    demoSound[7+50]   = new QSound("../share/hubo_pyeonchang/07_1millon_eng.wav");
    demoSound[8+50]   = new QSound("../share/hubo_pyeonchang/08_finger_eng.wav");
    demoSound[9+50]   = new QSound("../share/hubo_pyeonchang/09_cute_eng.wav");
    demoSound[10+50]   = new QSound("../share/hubo_pyeonchang/10_love_eng.wav");
    demoSound[11+50]   = new QSound("../share/hubo_pyeonchang/11_hoot_eng.wav");
    demoSound[12+50]   = new QSound("../share/hubo_pyeonchang/12_bye_eng.wav");
    demoSound[13+50]   = new QSound("../share/hubo_pyeonchang/13_1stRobot_eng.wav");
    demoSound[14+50]   = new QSound("../share/hubo_pyeonchang/14_closer_eng.wav");
    demoSound[15+50]   = new QSound("../share/hubo_pyeonchang/15_curious_eng.wav");
    demoSound[16+50]   = new QSound("../share/hubo_pyeonchang/16_dance_eng.wav");
    demoSound[17+50]   = new QSound("../share/hubo_pyeonchang/17_enjoy_eng.wav");
    demoSound[18+50]   = new QSound("../share/hubo_pyeonchang/18_from_eng.wav");
    demoSound[19+50]   = new QSound("../share/hubo_pyeonchang/19_okay_eng.wav");
    demoSound[20+50]   = new QSound("../share/hubo_pyeonchang/20_shake_eng.wav");
    demoSound[21+50]   = new QSound("../share/hubo_pyeonchang/21_shake_gently_eng.wav");
    demoSound[22+50]   = new QSound("../share/hubo_pyeonchang/22_sorry_can't_do_eng.wav");
    demoSound[23+50]   = new QSound("../share/hubo_pyeonchang/23_walk_eng.wav");
    demoSound[24+50]   = new QSound("../share/hubo_pyeonchang/24_welcome_eng.wav");
    demoSound[25+50]   = new QSound("../share/hubo_pyeonchang/25_don't_know_eng.wav");
    demoSound[26+50]   = new QSound("../share/hubo_pyeonchang/26_eat_eng.wav");
    demoSound[27+50]   = new QSound("../share/hubo_pyeonchang/27_feeling_eng.wav");
    demoSound[28+50]   = new QSound("../share/hubo_pyeonchang/28_cheer_eng.wav");
    demoSound[29+50]   = new QSound("../share/hubo_pyeonchang/29_fighting_eng.wav");
    demoSound[30+50]   = new QSound("../share/hubo_pyeonchang/30_cold_eng.wav");
    demoSound[31+50]   = new QSound("../share/hubo_pyeonchang/31_picture_eng.wav");
    demoSound[32+50]   = new QSound("../share/hubo_pyeonchang/32_hero_wakeup_eng.wav");
    demoSound[33+50]   = new QSound("../share/hubo_pyeonchang/33_nerf_this_eng.wav");
    demoSound[34+50]   = new QSound("../share/hubo_pyeonchang/34_my_skill_time_eng.wav");
    demoSound[35+50]   = new QSound("../share/hubo_pyeonchang/35_online_eng.wav");
    demoSound[36+50]   = new QSound("../share/hubo_pyeonchang/36_yes_eng.wav");
    demoSound[37+50]   = new QSound("../share/hubo_pyeonchang/37_no_eng.wav");
    demoSound[38+50]   = new QSound("../share/hubo_pyeonchang/38_move_eng.wav");
    demoSound[39+50]   = new QSound("../share/hubo_pyeonchang/39_move_left_eng.wav");
    demoSound[40+50]   = new QSound("../share/hubo_pyeonchang/40_move_right_eng.wav");

    demoSound[100] = new QSound("../share/hubo_pyeonchang/vip/vip_meet_you_kor.wav");
    demoSound[101] = new QSound("../share/hubo_pyeonchang/vip/vip_name_kor.wav");
    demoSound[102] = new QSound("../share/hubo_pyeonchang/vip/vip_photo_kor.wav");
    demoSound[103] = new QSound("../share/hubo_pyeonchang/vip/vip_meet_you_eng.wav");
    demoSound[104] = new QSound("../share/hubo_pyeonchang/vip/vip_name_eng.wav");
    demoSound[105] = new QSound("../share/hubo_pyeonchang/vip/vip_photo_eng.wav");
    demoSound[106] = new QSound("../share/hubo_pyeonchang/vip/vip_meet_you_fra.wav");
    demoSound[107] = new QSound("../share/hubo_pyeonchang/vip/vip_name_fra.wav");
    demoSound[108] = new QSound("../share/hubo_pyeonchang/vip/vip_photo_fra.wav");
    demoSound[109] = new QSound("../share/hubo_pyeonchang/vip/vip_meet_you_spa.wav");
    demoSound[110] = new QSound("../share/hubo_pyeonchang/vip/vip_name_spa.wav");
    demoSound[111] = new QSound("../share/hubo_pyeonchang/vip/vip_photo_spa.wav");
    demoSound[112] = new QSound("../share/hubo_pyeonchang/vip/vip_meet_you_chi.wav");
    demoSound[113] = new QSound("../share/hubo_pyeonchang/vip/vip_name_chi.wav");
    demoSound[114] = new QSound("../share/hubo_pyeonchang/vip/vip_photo_chi.wav");
    demoSound[115] = new QSound("../share/hubo_pyeonchang/vip/vip_meet_you_jap.wav");
    demoSound[116] = new QSound("../share/hubo_pyeonchang/vip/vip_name_jap.wav");
    demoSound[117] = new QSound("../share/hubo_pyeonchang/vip/vip_photo_jap.wav");

    demoSound[120] = new QSound("../share/hubo_pyeonchang/wa.wav");
    demoSound[121] = new QSound("../share/hubo_pyeonchang/oh.wav");
    demoSound[145] = new QSound("../share/hubo_pyeonchang/passioncrew.wav");


    serial = new QSerialPort(this);
    serial->setPortName("/dev/ttyUSB0");
    serial->setBaudRate(9600);
    serial->setDataBits(QSerialPort::Data8);
    serial->setParity(QSerialPort::NoParity);
    serial->setStopBits(QSerialPort::OneStop);
    int ret = serial->open(QIODevice::ReadWrite);
    cout << "Serial Open: " << ret << endl;

    // Dance
//    ui->BTN_DANCE_25READY->setDisabled(false);
        ui->BTN_DANCE_SCENE25->setDisabled(true);
        ui->BTN_DANCE_FINISH->setDisabled(true);
//    ui->BTN_DANCE_PICKMEREADY->setDisabled(false);
        ui->BTN_DANCE_PICKME->setDisabled(true);

        ui->BTN_FACE_THINK->setVisible(false);
        ui->BTN_FACE_WINK->setVisible(false);
        ui->BTN_FACE_SIMPLE->setVisible(false);

        ui->BTN_DANCE_GOGOGO->setDisabled(true);
        ui->btn_leg_mode->setText("English");
        _sound = ENGLISH;

}

OriginalDemoDialog::~OriginalDemoDialog()
{
    delete ui;
}

void OriginalDemoDialog::on_BTN_WALKREADY_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_TARGET = ALNumWalkReady;
    cmd.COMMAND_DATA.USER_COMMAND = WALKREADY_GO_WALKREADYPOS;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = 0;
    pLAN->SendCommand(cmd);
}

void OriginalDemoDialog::on_BTN_GOTOHOME_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_TARGET = ALNumWalkReady;
    cmd.COMMAND_DATA.USER_COMMAND = WALKREADY_GO_HOMEPOS;
    pLAN->SendCommand(cmd);
}

void OriginalDemoDialog::on_BTN_INIT_HAND_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = DAEMON_INIT_FIND_HOME_HAND_HEAD;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = 1; // right
    cmd.COMMAND_DATA.USER_PARA_CHAR[1] = 1; // left
    cmd.COMMAND_DATA.USER_PARA_CHAR[2] = 0; // head
    cmd.COMMAND_TARGET = RBCORE_PODO_NO;
    pLAN->SendCommand(cmd);
}


/**********************************************************************************/

int OriginalDemoDialog::RBSerialWrite(const char *_uart_frame, uchar _bytes, uchar _mode){
    char temp[20];
    if(_mode == 0x00) while((RBReadPort(temp, 20, 0x00) != 0));

    return write(serialHandler, _uart_frame, _bytes);
}

int OriginalDemoDialog::RBReadPort(char *_uart_frame, uchar _bytes, uchar _mode){
    uchar receivedByte = 0;
    uchar index = 0;
    uint loopTime = 0;
    char receivedData[20];
    uchar i;

    if(_mode == 0x00){
        usleep(10000);
        return read(serialHandler, _uart_frame, _bytes);
    }else{
        while(receivedByte < _bytes){
            usleep(10000);

            index = read(serialHandler, &receivedData[index], 20);
            if(index > 0){
                for(i=0 ; i<index ; i++) _uart_frame[receivedByte+i] = receivedData[i];
                receivedByte += index;
            }

            if(loopTime > 50) return -1;
            else loopTime++;
        }
        return receivedByte;
    }
}



/**********************************************************************************/
// Face Change
void OriginalDemoDialog::ChangeFace(int emotion){
//    char sendData[1];
//    sendData[0] = emotion;
//    RBSerialWrite((const char*)sendData, 1, 0x01);
    QByteArray data;
    data.push_back((char)emotion);
    serial->write(data);
}

void OriginalDemoDialog::on_BTN_FACE_SMILE_clicked()        {ChangeFace(1);}
void OriginalDemoDialog::on_BTN_FACE_IMPASSIVE_clicked()    {ChangeFace(2);}
void OriginalDemoDialog::on_BTN_FACE_SAD_clicked()          {ChangeFace(3);}
void OriginalDemoDialog::on_BTN_FACE_ANGRY_clicked()        {ChangeFace(4);}
void OriginalDemoDialog::on_BTN_FACE_WONDER_clicked()       {ChangeFace(5);}
void OriginalDemoDialog::on_BTN_FACE_THINK_clicked()        {ChangeFace(6);}
void OriginalDemoDialog::on_BTN_FACE_WINK_clicked()         {ChangeFace(7);}
void OriginalDemoDialog::on_BTN_FACE_SIMPLE_clicked()       {ChangeFace(8);}
void OriginalDemoDialog::on_BTN_FACE_HAPPY_clicked()        {ChangeFace(9);}

void OriginalDemoDialog::on_BTN_FACE_1_clicked()            {ChangeFace(17);}
void OriginalDemoDialog::on_BTN_FACE_2_clicked()            {ChangeFace(18);}
void OriginalDemoDialog::on_BTN_FACE_3_clicked()            {ChangeFace(19);}

void OriginalDemoDialog::on_BTN_FACE_YES_clicked()          {ChangeFace(33);}
void OriginalDemoDialog::on_BTN_FACE_NO_clicked()           {ChangeFace(34);}
void OriginalDemoDialog::on_BTN_FACE_HELLO_clicked()        {ChangeFace(35);}
void OriginalDemoDialog::on_BTN_FACE_HUBO_clicked()         {ChangeFace(36);}
void OriginalDemoDialog::on_BTN_FACE_LEFT_clicked()         {ChangeFace(37);}
void OriginalDemoDialog::on_BTN_FACE_RIGHT_clicked()        {ChangeFace(38);}
void OriginalDemoDialog::on_BTN_FACE_QUESTION_clicked()     {ChangeFace(39);}
void OriginalDemoDialog::on_BTN_FACE_EXCLMATION_clicked()   {ChangeFace(40);}
void OriginalDemoDialog::on_BTN_FACE_QUESTION2_clicked()    {ChangeFace(41);}
void OriginalDemoDialog::on_BTN_FACE_EXCLMATION2_clicked()  {ChangeFace(42);} //0x2A

/**********************************************************************************/




void OriginalDemoDialog::GeneralCommand(int _TARGET_AL,int motion_num, int sound_num){
    if(motion_num >= 0){
        USER_COMMAND cmd;
        cmd.COMMAND_TARGET = ALNum_Demo;//_TARGET_AL;
        if(cmd.COMMAND_TARGET == ALNum_Pyeongchang)
        cmd.COMMAND_DATA.USER_COMMAND = SCI_DEMO;//PYEONGCHANG_DEMO;//SCI_DEMO;
        else if(cmd.COMMAND_TARGET == ALNum_Demo)
            cmd.COMMAND_DATA.USER_COMMAND = SCI_DEMO;
        cmd.COMMAND_DATA.USER_PARA_INT[3] = motion_num;
        pLAN->SendCommand(cmd);
    }

    if(sound_num >= 0 && sound_num < 140) {
        if(_sound == ENGLISH){
            sound_num += 50;
        }
    }
    if(sound_num == 140 || sound_num == 141 || sound_num == 145){
    demoSound[sound_num]->play();
    }
}

/**********************************************************************************/
// Head
void OriginalDemoDialog::on_BTN_HEAD_YES_clicked()      {GeneralCommand(ALNum_Demo,8801, 36);ui->BTN_HEAD_YES->setDisabled(true);}
void OriginalDemoDialog::on_BTN_HEAD_NO_clicked()       {GeneralCommand(ALNum_Demo,8802, 37);ui->BTN_HEAD_NO->setDisabled(true);}
void OriginalDemoDialog::on_BTN_HEAD_QUESTION_clicked() {GeneralCommand(ALNum_Demo,13, 15);ui->BTN_HEAD_QUESTION->setDisabled(true);}
void OriginalDemoDialog::on_btn_head_stretching_clicked(){GeneralCommand(ALNum_Pyeongchang,HEAD_STRETCHING, 35); ui->btn_head_stretching->setDisabled(true);   }
void OriginalDemoDialog::on_btn_head_init_clicked(){ui->BTN_HEAD_YES->setEnabled(true); ui->BTN_HEAD_NO->setEnabled(true); ui->btn_head_stretching->setEnabled(true); ui->BTN_HEAD_QUESTION->setEnabled(true);}

/**********************************************************************************/


/**********************************************************************************/
// Rock Papers Scissors
void OriginalDemoDialog::on_BTN_RSP_READY_clicked()     {GeneralCommand(ALNum_Demo,1401, -1);ui->BTN_RSP_READY->setDisabled(true);ui->BTN_RSP_DO->setDisabled(false);}
void OriginalDemoDialog::on_BTN_RSP_DO_clicked()        {GeneralCommand(ALNum_Demo,1402, -1);ui->BTN_RSP_DO->setDisabled(true);ui->BTN_RSP_READY->setDisabled(false);}
/**********************************************************************************/



/**********************************************************************************/
// Hubo2 Motion
void OriginalDemoDialog::on_BTN_MOTION_GREETING_clicked()   {GeneralCommand(ALNum_Demo,BOW_TWO_W, 17);ui->BTN_MOTION_GREETING->setDisabled(true);}
void OriginalDemoDialog::on_BTN_MOTION_BOW_clicked()        {GeneralCommand(ALNum_Demo,3, 1);ui->BTN_MOTION_BOW->setDisabled(true);}
void OriginalDemoDialog::on_BTN_MOTION_HAPUM_clicked()      {GeneralCommand(ALNum_Demo,104, -1);ui->BTN_MOTION_HAPUM->setDisabled(true);}
void OriginalDemoDialog::on_BTN_MOTION_ONE_clicked()        {GeneralCommand(ALNum_Demo,105, -1);ui->BTN_MOTION_ONE->setDisabled(true);}
void OriginalDemoDialog::on_BTN_MOTION_TWO_clicked()        {GeneralCommand(ALNum_Demo,106, -1);ui->BTN_MOTION_TWO->setDisabled(true);}
void OriginalDemoDialog::on_BTN_MOTION_THREE_clicked()      {GeneralCommand(ALNum_Demo,107, -1);ui->BTN_MOTION_THREE->setDisabled(true);}
void OriginalDemoDialog::on_BTN_MOTION_STORY1_clicked()     {GeneralCommand(ALNum_Demo,4, 3);ui->BTN_MOTION_STORY1->setDisabled(true);}
void OriginalDemoDialog::on_BTN_MOTION_STORY2_clicked()     {GeneralCommand(ALNum_Demo,5, 4);ui->BTN_MOTION_STORY2->setDisabled(true);}
void OriginalDemoDialog::on_BTN_MOTION_STORY3_clicked()     {GeneralCommand(ALNum_Demo,108, 2);ui->BTN_MOTION_STORY3->setDisabled(true);}
void OriginalDemoDialog::on_BTN_MOTION_STORY4_clicked()     {GeneralCommand(ALNum_Demo,30, 23);ui->BTN_MOTION_STORY4->setDisabled(true);}
void OriginalDemoDialog::on_BTN_MOTION_STORY5_clicked()     {GeneralCommand(ALNum_Demo,38, 19);ui->BTN_MOTION_STORY5->setDisabled(true);}
void OriginalDemoDialog::on_BTN_MOTION_STORY6_clicked()     {GeneralCommand(ALNum_Demo,2801, -1);ui->BTN_MOTION_STORY6->setDisabled(true);}
void OriginalDemoDialog::on_BTN_MOTION_OKAY_clicked()       {GeneralCommand(ALNum_Demo,11, 19);ui->BTN_MOTION_OKAY->setDisabled(true);}
void OriginalDemoDialog::on_BTN_MOTION_QUESTION_clicked()   {GeneralCommand(ALNum_Demo,12, 15);ui->BTN_MOTION_QUESTION->setDisabled(true);}
void OriginalDemoDialog::on_BTN_MOTION_CRY_clicked()        {GeneralCommand(ALNum_Demo,15, 5);ui->BTN_MOTION_CRY->setDisabled(true);}
void OriginalDemoDialog::on_BTN_MOTION_CUTECUTE_clicked()   {GeneralCommand(ALNum_Demo,20, -1);ui->BTN_MOTION_CUTECUTE->setDisabled(true);}
void OriginalDemoDialog::on_BTN_MOTION_COMEON_clicked()     {GeneralCommand(ALNum_Demo,16, 14);ui->BTN_MOTION_COMEON->setDisabled(true);}
void OriginalDemoDialog::on_BTN_MOTION_FIGHTING_clicked()   {GeneralCommand(ALNum_Demo,17, 29);ui->BTN_MOTION_FIGHTING->setDisabled(true);}
void OriginalDemoDialog::on_BTN_MOTION_LOOK_clicked()       {GeneralCommand(ALNum_Demo,1, -1);ui->BTN_MOTION_LOOK->setDisabled(true);}
void OriginalDemoDialog::on_BTN_MOTION_DANCECUTE_clicked()  {GeneralCommand(ALNum_Demo,21, -1);ui->BTN_MOTION_DANCECUTE->setDisabled(true);}
void OriginalDemoDialog::on_BTN_MOTION_CUTE_clicked()       {GeneralCommand(ALNum_Demo,26, -1);ui->BTN_MOTION_CUTE->setDisabled(true);}
void OriginalDemoDialog::on_BTN_MOTION_CHASE_clicked()      {GeneralCommand(ALNum_Demo,6, -1);ui->BTN_MOTION_CHASE->setDisabled(true);}
void OriginalDemoDialog::on_BTN_MOTION_SUWHA_clicked()      {GeneralCommand(ALNum_Demo,29, -1);ui->BTN_MOTION_SUWHA->setDisabled(true);}
void OriginalDemoDialog::on_BTN_MOTION_HEUNG_clicked()      {GeneralCommand(ALNum_Demo,31, -1);ui->BTN_MOTION_HEUNG->setDisabled(true);}
/**********************************************************************************/



/**********************************************************************************/
// 815
void OriginalDemoDialog::on_BTN_815_HELLOL_clicked()        {GeneralCommand(ALNum_Demo,815, -1);ui->BTN_815_HELLOL->setDisabled(true);}
void OriginalDemoDialog::on_BTN_815_HELLOR_clicked()        {GeneralCommand(ALNum_Demo,816, -1);ui->BTN_815_HELLOR->setDisabled(true);}
void OriginalDemoDialog::on_BTN_815_STARTL_clicked()        {GeneralCommand(ALNum_Demo,817, -1);ui->BTN_815_STARTL->setDisabled(true);}
void OriginalDemoDialog::on_BTN_815_STARTR_clicked()        {GeneralCommand(ALNum_Demo,818, -1);ui->BTN_815_STARTR->setDisabled(true);}
void OriginalDemoDialog::on_BTN_815_FIGHTING_clicked()      {GeneralCommand(ALNum_Demo,17, -1);ui->BTN_815_FIGHTING->setDisabled(true);}
void OriginalDemoDialog::on_BTN_815_PROUDKOREA_clicked()    {GeneralCommand(ALNum_Demo,5, -1);ui->BTN_815_PROUDKOREA->setDisabled(true);}
void OriginalDemoDialog::on_BTN_815_STORY1_clicked()        {GeneralCommand(ALNum_Demo,4, -1);ui->BTN_815_STORY1->setDisabled(true);}
/**********************************************************************************/



/**********************************************************************************/
// Dance
void OriginalDemoDialog::on_BTN_DANCE_25READY_clicked()     {
    USER_COMMAND cmd;
    cmd.COMMAND_TARGET = ALNum_Demo;
    cmd.COMMAND_DATA.USER_COMMAND = SCI_DEMO_Control_on;

    pLAN->SendCommand(cmd);
    ui->BTN_DANCE_25READY->setDisabled(true);
    ui->BTN_DANCE_SCENE25->setDisabled(false);
    ui->BTN_DANCE_FINISH->setDisabled(false);
}  // TODO

void OriginalDemoDialog::on_BTN_DANCE_FINISH_clicked()      {
    USER_COMMAND cmd;
    cmd.COMMAND_TARGET = ALNum_Demo;
    cmd.COMMAND_DATA.USER_COMMAND = SCI_DEMO_Control_off;

    pLAN->SendCommand(cmd);
    ui->BTN_DANCE_FINISH->setDisabled(true);
    ui->BTN_DANCE_25READY->setDisabled(false);
}  // TODO

void OriginalDemoDialog::on_BTN_DANCE_SCENE25_clicked()     {GeneralCommand(ALNum_Demo,25, 140);ui->BTN_DANCE_SCENE25->setDisabled(true);}

void OriginalDemoDialog::on_BTN_DANCE_PICKMEREADY_clicked() {GeneralCommand(ALNum_Demo,102, -1);ui->BTN_DANCE_PICKMEREADY->setDisabled(true);ui->BTN_DANCE_PICKME->setDisabled(false);}
void OriginalDemoDialog::on_BTN_DANCE_PICKME_clicked()      {GeneralCommand(ALNum_Demo,101, 141);ui->BTN_DANCE_PICKME->setDisabled(true);ui->BTN_DANCE_PICKMEREADY->setDisabled(false);}
void OriginalDemoDialog::on_BTN_DANCE_SCENE22_clicked()     {GeneralCommand(ALNum_Demo,22, -1);ui->BTN_DANCE_SCENE22->setDisabled(true);}
void OriginalDemoDialog::on_BTN_DANCE_ILOVEYOU_clicked()    {GeneralCommand(ALNum_Demo,32, 143);ui->BTN_DANCE_ILOVEYOU->setDisabled(true);}
void OriginalDemoDialog::on_BTN_DANCE_FINGERSHOW_clicked()  {GeneralCommand(ALNum_Demo,19, 142);ui->BTN_DANCE_FINGERSHOW->setDisabled(true);}
void OriginalDemoDialog::on_BTN_DANCE_HEUNG_clicked()       {GeneralCommand(ALNum_Demo,23, 144);ui->BTN_DANCE_HEUNG->setDisabled(true);}
/**********************************************************************************/



/**********************************************************************************/
// One Foot
void OriginalDemoDialog::on_BTN_ONEFOOT_ON_clicked()        {}  // TODO
void OriginalDemoDialog::on_BTN_ONEFOOT_POS_clicked()       {GeneralCommand(ALNum_Demo,1801, -1);}
void OriginalDemoDialog::on_BTN_ONEFOOT_DO_clicked()        {GeneralCommand(ALNum_Demo,1802, -1);}
void OriginalDemoDialog::on_BTN_ONEFOOT_V_clicked()         {GeneralCommand(ALNum_Demo,-1, -1);}
void OriginalDemoDialog::on_BTN_ONEFOOT_HOMEPOS_clicked()   {GeneralCommand(ALNum_Demo,1804, -1);}
/**********************************************************************************/



/**********************************************************************************/
// Control OnOff
void OriginalDemoDialog::on_BTN_CTRL_OFFPOS_clicked()       {GeneralCommand(ALNum_Demo,801, -1);}
void OriginalDemoDialog::on_BTN_CTRL_ONPOS_clicked()        {GeneralCommand(ALNum_Demo,802, -1);}
void OriginalDemoDialog::on_BTN_CTRL_START_clicked()        {}  // TODO
void OriginalDemoDialog::on_BTN_CTRL_STOP_clicked()         {}  // TODO
/**********************************************************************************/

void OriginalDemoDialog::on_BTN_ENABLE_DISABLE_clicked()
{

//    QString txt1,txt2;
//    txt1="';
//    txt2='All Btn Enable';


    if(ui->BTN_ENABLE_DISABLE->text()=="All Btn Disable")
    {
        ui->BTN_ENABLE_DISABLE->setText("All Btn Enable");
        /**********************************************************************************/
        // Head
        ui->BTN_HEAD_YES->setDisabled(true);
        ui->BTN_HEAD_NO->setDisabled(true);
        ui->BTN_HEAD_QUESTION->setDisabled(true);
        ui->btn_head_stretching->setDisabled(true);
        ui->btn_head_init->setDisabled(true);
        /*****************************/

        /**********************************************************************************/
        //clap
        ui->btn_clap_ready->setDisabled(true);
        /*****************************/

        /**********************************************************************************/
        //guidance
        ui->btn_guid_init->setDisabled(true);
        ui->btn_guid_left->setDisabled(true);
        ui->btn_guid_right->setDisabled(true);
        /*****************************/

        /**********************************************************************************/
        //bow
        ui->btn_bow_one->setDisabled(true);
        ui->btn_bow_two->setDisabled(true);
        /*****************************/

        /**********************************************************************************/
        // Rock Papers Scissors
        ui->BTN_RSP_READY->setDisabled(true);
        ui->BTN_RSP_DO->setDisabled(true);
        /**********************************************************************************/


        /**********************************************************************************/
        // Hubo2 Motion
        ui->BTN_MOTION_GREETING->setDisabled(true);
        ui->BTN_MOTION_BOW->setDisabled(true);
        ui->BTN_MOTION_HAPUM->setDisabled(true);
        ui->BTN_MOTION_ONE->setDisabled(true);
        ui->BTN_MOTION_TWO->setDisabled(true);
        ui->BTN_MOTION_THREE->setDisabled(true);
        ui->BTN_MOTION_STORY1->setDisabled(true);
        ui->BTN_MOTION_STORY2->setDisabled(true);
        ui->BTN_MOTION_STORY3->setDisabled(true);
        ui->BTN_MOTION_STORY4->setDisabled(true);
        ui->BTN_MOTION_STORY5->setDisabled(true);
        ui->BTN_MOTION_STORY6->setDisabled(true);
        ui->BTN_MOTION_OKAY->setDisabled(true);
        ui->BTN_MOTION_QUESTION->setDisabled(true);
        ui->BTN_MOTION_CRY->setDisabled(true);
        ui->BTN_MOTION_CUTECUTE->setDisabled(true);
        ui->BTN_MOTION_COMEON->setDisabled(true);
        ui->BTN_MOTION_FIGHTING->setDisabled(true);
        ui->BTN_MOTION_LOOK->setDisabled(true);
        ui->BTN_MOTION_DANCECUTE->setDisabled(true);
        ui->BTN_MOTION_CUTE->setDisabled(true);
        ui->BTN_MOTION_CHASE->setDisabled(true);
        ui->BTN_MOTION_SUWHA->setDisabled(true);
        ui->BTN_MOTION_HEUNG->setDisabled(true);
        /**********************************************************************************/

        /**********************************************************************************/
        // 815
        ui->BTN_815_HELLOL->setDisabled(true);
        ui->BTN_815_HELLOR->setDisabled(true);
        ui->BTN_815_STARTL->setDisabled(true);
        ui->BTN_815_STARTR->setDisabled(true);
        ui->BTN_815_FIGHTING->setDisabled(true);
        ui->BTN_815_PROUDKOREA->setDisabled(true);
        ui->BTN_815_STORY1->setDisabled(true);
        /**********************************************************************************/


        /**********************************************************************************/
        // Dance
//        ui->BTN_DANCE_25READY->setDisabled(true);
//        ui->BTN_DANCE_SCENE25->setDisabled(true);
//        ui->BTN_DANCE_FINISH->setDisabled(true);
//        ui->BTN_DANCE_PICKMEREADY->setDisabled(true);
//        ui->BTN_DANCE_PICKME->setDisabled(true);
        ui->BTN_DANCE_SCENE22->setDisabled(true);
        ui->BTN_DANCE_ILOVEYOU->setDisabled(true);
        ui->BTN_DANCE_FINGERSHOW->setDisabled(true);
        ui->BTN_DANCE_HEUNG->setDisabled(true);
        ui->BTN_DANCE_GOGOGO->setDisabled(true);
    }
    else
    {
        ui->BTN_ENABLE_DISABLE->setText("All Btn Disable");
        /**********************************************************************************/
        // Head
        ui->BTN_HEAD_YES->setDisabled(false);
        ui->BTN_HEAD_NO->setDisabled(false);
        ui->BTN_HEAD_QUESTION->setDisabled(false);
        ui->btn_head_stretching->setDisabled(false);
        ui->btn_head_init->setDisabled(false);
        /*****************************/

        /**********************************************************************************/
        //clap
        ui->btn_clap_ready->setDisabled(false);
        /*****************************/

        /**********************************************************************************/
        //guidance
        ui->btn_guid_init->setDisabled(false);
        ui->btn_guid_left->setDisabled(false);
        ui->btn_guid_right->setDisabled(false);
        /*****************************/

        /**********************************************************************************/
        //bow
        ui->btn_bow_one->setDisabled(false);
        ui->btn_bow_two->setDisabled(false);
        /*****************************/

        // Rock Papers Scissors
        ui->BTN_RSP_READY->setDisabled(false);
        ui->BTN_RSP_DO->setDisabled(false);
        /*****************************/

        /**********************************************************************************/
        // Hubo2 Motion
        ui->BTN_MOTION_GREETING->setDisabled(false);
        ui->BTN_MOTION_BOW->setDisabled(false);
        ui->BTN_MOTION_HAPUM->setDisabled(false);
        ui->BTN_MOTION_ONE->setDisabled(false);
        ui->BTN_MOTION_TWO->setDisabled(false);
        ui->BTN_MOTION_THREE->setDisabled(false);
        ui->BTN_MOTION_STORY1->setDisabled(false);
        ui->BTN_MOTION_STORY2->setDisabled(false);
        ui->BTN_MOTION_STORY3->setDisabled(false);
        ui->BTN_MOTION_STORY4->setDisabled(false);
        ui->BTN_MOTION_STORY5->setDisabled(false);
        ui->BTN_MOTION_STORY6->setDisabled(false);
        ui->BTN_MOTION_OKAY->setDisabled(false);
        ui->BTN_MOTION_QUESTION->setDisabled(false);
        ui->BTN_MOTION_CRY->setDisabled(false);
        ui->BTN_MOTION_CUTECUTE->setDisabled(false);
        ui->BTN_MOTION_COMEON->setDisabled(false);
        ui->BTN_MOTION_FIGHTING->setDisabled(false);
        ui->BTN_MOTION_LOOK->setDisabled(false);
        ui->BTN_MOTION_DANCECUTE->setDisabled(false);
        ui->BTN_MOTION_CUTE->setDisabled(false);
        ui->BTN_MOTION_CHASE->setDisabled(false);
        ui->BTN_MOTION_SUWHA->setDisabled(false);
        ui->BTN_MOTION_HEUNG->setDisabled(false);
        /**********************************************************************************/

        /**********************************************************************************/
        // 815
        ui->BTN_815_HELLOL->setDisabled(false);
        ui->BTN_815_HELLOR->setDisabled(false);
        ui->BTN_815_STARTL->setDisabled(false);
        ui->BTN_815_STARTR->setDisabled(false);
        ui->BTN_815_FIGHTING->setDisabled(false);
        ui->BTN_815_PROUDKOREA->setDisabled(false);
        ui->BTN_815_STORY1->setDisabled(false);
        /**********************************************************************************/


        /**********************************************************************************/
        // Dance
//        ui->BTN_DANCE_25READY->setDisabled(false);
//        ui->BTN_DANCE_SCENE25->setDisabled(false);
//        ui->BTN_DANCE_FINISH->setDisabled(false);
//        ui->BTN_DANCE_PICKMEREADY->setDisabled(false);
//        ui->BTN_DANCE_PICKME->setDisabled(false);
        ui->BTN_DANCE_SCENE22->setDisabled(false);
        ui->BTN_DANCE_ILOVEYOU->setDisabled(false);
        ui->BTN_DANCE_FINGERSHOW->setDisabled(false);
        ui->BTN_DANCE_HEUNG->setDisabled(false);
        ui->BTN_DANCE_GOGOGO->setDisabled(false);
    }

}

//void OriginalDemoDialog::on_BTN_test_001_clicked()
//{

//}

void OriginalDemoDialog::on_btn_hand_ready_clicked(){
    GeneralCommand(ALNum_Demo,HANDSHAKE_READY,21);
}void OriginalDemoDialog::on_btn_hand_gon_clicked(){
    GeneralCommand(ALNum_Pyeongchang,HANDSHAKE_GRIP_ON,-1);
}void OriginalDemoDialog::on_btn_hand_goff_clicked(){
    GeneralCommand(ALNum_Pyeongchang,HANDSHAKE_GRIP_OFF,-1);
}void OriginalDemoDialog::on_btn_hand_init_clicked(){
    GeneralCommand(ALNum_Pyeongchang,HANDSHAKE_INIT,-1);//12);
}void OriginalDemoDialog::on_btn_clap_ready_clicked(){
    USER_COMMAND cmd;
    cmd.COMMAND_TARGET = ALNum_Demo;
    cmd.COMMAND_DATA.USER_COMMAND = SCI_DEMO;
    cmd.COMMAND_DATA.USER_PARA_INT[3] = CLAP_READY;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = ui->LE_CLAP_CNT->text().toInt();
    pLAN->SendCommand(cmd);
    ui->btn_clap_ready->setDisabled(true);
}void OriginalDemoDialog::on_btn_bow_one_clicked(){
    GeneralCommand(ALNum_Pyeongchang,BOW_ONE,-1);
     ui->btn_bow_one->setDisabled(true);
}void OriginalDemoDialog::on_btn_bow_two_clicked(){
    GeneralCommand(ALNum_Pyeongchang,BOW_TWO_W,-1);
    ui->btn_bow_two->setDisabled(true);
}void OriginalDemoDialog::on_btn_leg_mode_clicked(){
    if(ui->btn_leg_mode->text() == "English"){
        ui->btn_leg_mode->setText("Korean");
        _sound = KOREAN;
    }
    else if(ui->btn_leg_mode->text() == "Korean"){
        ui->btn_leg_mode->setText("English");
        _sound = ENGLISH;
    }
}void OriginalDemoDialog::on_btn_bouquet_ready_clicked(){
    GeneralCommand(ALNum_Pyeongchang,BOUQUET_READY,-1);
}void OriginalDemoDialog::on_btn_bouquet_give_clicked(){
    GeneralCommand(ALNum_Pyeongchang,BOUQUET_GIVE,-1);
}void OriginalDemoDialog::on_btn_bouquet_init_clicked(){
    GeneralCommand(ALNum_Pyeongchang,BOUQUET_INIT,-1);
}void OriginalDemoDialog::on_btn_guid_left_clicked(){
    GeneralCommand(ALNum_Pyeongchang,GUID_LEFT,40);
    ui->btn_guid_left->setDisabled(true);
}void OriginalDemoDialog::on_btn_guid_right_clicked(){
    GeneralCommand(ALNum_Pyeongchang,GUID_RIGHT,39);
    ui->btn_guid_right->setDisabled(true);
}void OriginalDemoDialog::on_btn_guid_init_clicked(){
    GeneralCommand(ALNum_Pyeongchang,GUID_INIT,-1);
    ui->btn_guid_init->setDisabled(true);
}void OriginalDemoDialog::on_btn_hand_sound_clicked(){
    int sound_num = 20;
    if(_sound == ENGLISH){
        sound_num += 50;
    }
//    demoSound[sound_num]->play();
}void OriginalDemoDialog::on_btn_high_five_go_clicked()
{
    GeneralCommand(ALNum_Pyeongchang,HIGHFIVE_DO,-1);
//    ui->btn_high_five_init->setDisabled(true);
}void OriginalDemoDialog::on_btn_high_five_init_clicked()
{
    GeneralCommand(ALNum_Pyeongchang,HIGHFIVE_DO,-1);
//    ui->btn_high_five_init->setDisabled(true);
}void OriginalDemoDialog::on_BTN_DANCE_GOGOGO_clicked(){
//    on_BTN_INT_PYEONGCHANG_WALK_clicked();

//    usleep(1000000);

    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_INT[10] = 0;
    cmd.COMMAND_DATA.USER_PARA_INT[11] = 0;

    GeneralCommand(ALNum_Pyeongchang,DANCE_GOGOGO,145);
    ui->BTN_DANCE_GOGOGO->setDisabled(true);
}void OriginalDemoDialog::on_BTN_DANCE_GOGOGOREADY_clicked(){
    GeneralCommand(ALNum_Pyeongchang,DANCE_GOGOGO_READY,-1);
    ui->BTN_DANCE_GOGOGO->setDisabled(false);
    ui->BTN_DANCE_GOGOGO_2->setDisabled(false);
    demoSound[145]->stop();
}
void OriginalDemoDialog::on_BTN_INT_INITIALIZE_clicked(){
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = FREEWALK_INITIALIZE;
    cmd.COMMAND_TARGET = alNumFreeWalking;
    pLAN->SendCommand(cmd);
}
enum WalkingModeCommand{
    FORWARD_WALKING =0,
    BACKWARD_WALKING,
    RIGHTSIDE_WALKING,
    LEFTSIDE_WALKING,
    CWROT_WALKING,
    CCWROT_WALKING,
    GOTOWR_WALKING
};
void OriginalDemoDialog::on_BTN_INT_FORWARD_clicked()
{
    USER_COMMAND cmd;
    pLAN->G2MData->StepTime = Normal_walking_speed;//1.f;
    pLAN->G2MData->WalkingStopModeCommand = COMPLETE_STOP_WALKING;
    pLAN->G2MData->WalkingModeCommand = FORWARD_WALKING;

    pLAN->G2MData->StepNum = ui->LE_STEP_NUM->text().toInt();
    pLAN->G2MData->StepLength = ui->LE_STEP_LENGTH->text().toDouble();
    pLAN->G2MData->StepAngle = ui->LE_ROT_ANGLE->text().toDouble();

    if(pLAN->G2MData->StepLength>0.35) return;
    if(pLAN->G2MData->StepLength<0.001) return;
    if(pLAN->G2MData->StepAngle>45) return;
//    if(pLAN->G2MData->StepNum>30) return;

        cmd.COMMAND_DATA.USER_PARA_INT[4] = INSIDE_WALKING;

    cmd.COMMAND_DATA.USER_PARA_INT[9] = NORMAL_WALKING;
    cmd.COMMAND_DATA.USER_COMMAND = FREEWALK_WALK;
    cmd.COMMAND_TARGET = alNumFreeWalking;

    pLAN->SendCommand(cmd);
}

void OriginalDemoDialog::on_BTN_INT_BACKWARD_clicked()
{
    USER_COMMAND cmd;
    pLAN->G2MData->StepTime = Normal_walking_speed;//1.f;
    pLAN->G2MData->WalkingStopModeCommand = COMPLETE_STOP_WALKING;
    pLAN->G2MData->WalkingModeCommand = BACKWARD_WALKING;
    pLAN->G2MData->StepNum = ui->LE_STEP_NUM->text().toInt();
    pLAN->G2MData->StepLength = ui->LE_STEP_LENGTH->text().toDouble();
    pLAN->G2MData->StepAngle = ui->LE_ROT_ANGLE->text().toDouble();
    if(pLAN->G2MData->StepLength>0.35) return;
    if(pLAN->G2MData->StepLength<0.001) return;
    if(pLAN->G2MData->StepAngle>45) return;
//    if(pLAN->G2MData->StepNum>30) return;

        cmd.COMMAND_DATA.USER_PARA_INT[4] = INSIDE_WALKING;

    cmd.COMMAND_DATA.USER_PARA_INT[9] = NORMAL_WALKING;
    cmd.COMMAND_DATA.USER_COMMAND = FREEWALK_WALK;
    cmd.COMMAND_TARGET = alNumFreeWalking;

    pLAN->SendCommand(cmd);
}

void OriginalDemoDialog::on_BTN_INT_RIGHT_clicked()
{
    USER_COMMAND cmd;
    pLAN->G2MData->StepTime = Normal_walking_speed;//1.f;
    pLAN->G2MData->WalkingStopModeCommand = COMPLETE_STOP_WALKING;
    pLAN->G2MData->WalkingModeCommand = RIGHTSIDE_WALKING;
    pLAN->G2MData->StepNum = ui->LE_STEP_NUM->text().toInt();
    pLAN->G2MData->StepLength = ui->LE_STEP_LENGTH->text().toDouble();
    pLAN->G2MData->StepAngle = ui->LE_ROT_ANGLE->text().toDouble();

    if(pLAN->G2MData->StepLength>0.35) return;
    if(pLAN->G2MData->StepLength<0.001) return;
    if(pLAN->G2MData->StepAngle>45) return;
//    if(pLAN->G2MData->StepNum>30) return;

        cmd.COMMAND_DATA.USER_PARA_INT[4] = INSIDE_WALKING;
    cmd.COMMAND_DATA.USER_PARA_INT[9] = NORMAL_WALKING;
    cmd.COMMAND_DATA.USER_COMMAND = FREEWALK_WALK;
    cmd.COMMAND_TARGET = alNumFreeWalking;
    pLAN->SendCommand(cmd);
}

void OriginalDemoDialog::on_BTN_INT_LEFT_clicked()
{
    USER_COMMAND cmd;
    pLAN->G2MData->StepTime = Normal_walking_speed;//0.9f;
    pLAN->G2MData->WalkingStopModeCommand = COMPLETE_STOP_WALKING;
    pLAN->G2MData->WalkingModeCommand = LEFTSIDE_WALKING;
    pLAN->G2MData->StepNum = ui->LE_STEP_NUM->text().toInt();
    pLAN->G2MData->StepLength = ui->LE_STEP_LENGTH->text().toDouble();
    pLAN->G2MData->StepAngle = ui->LE_ROT_ANGLE->text().toDouble();
    if(pLAN->G2MData->StepLength>0.35) return;
    if(pLAN->G2MData->StepLength<0.001) return;
    if(pLAN->G2MData->StepAngle>45) return;
//    if(pLAN->G2MData->StepNum>30) return;

        cmd.COMMAND_DATA.USER_PARA_INT[4] = INSIDE_WALKING;
    cmd.COMMAND_DATA.USER_PARA_INT[9] = NORMAL_WALKING;
    cmd.COMMAND_DATA.USER_COMMAND = FREEWALK_WALK;
    cmd.COMMAND_TARGET = alNumFreeWalking;
    pLAN->SendCommand(cmd);
}

void OriginalDemoDialog::on_BTN_INT_CW_clicked()
{
    USER_COMMAND cmd;
    pLAN->G2MData->StepTime = Normal_walking_speed;//0.9f;
    pLAN->G2MData->WalkingStopModeCommand = COMPLETE_STOP_WALKING;
    pLAN->G2MData->WalkingModeCommand = CWROT_WALKING;
    pLAN->G2MData->StepNum = ui->LE_STEP_NUM->text().toInt();
    pLAN->G2MData->StepLength = ui->LE_STEP_LENGTH->text().toDouble();
    pLAN->G2MData->StepAngle = ui->LE_ROT_ANGLE->text().toDouble();
    if(pLAN->G2MData->StepLength>0.35) return;
    if(pLAN->G2MData->StepLength<0.001) return;
    if(pLAN->G2MData->StepAngle>45) return;
//    if(pLAN->G2MData->StepNum>30) return;

        cmd.COMMAND_DATA.USER_PARA_INT[4] = INSIDE_WALKING;

    cmd.COMMAND_DATA.USER_PARA_INT[9] = NORMAL_WALKING;
    cmd.COMMAND_DATA.USER_COMMAND = FREEWALK_WALK;
    cmd.COMMAND_TARGET = alNumFreeWalking;
    pLAN->SendCommand(cmd);
}

void OriginalDemoDialog::on_BTN_INT_CCW_clicked()
{
    USER_COMMAND cmd;
    pLAN->G2MData->StepTime = Normal_walking_speed;
    pLAN->G2MData->WalkingStopModeCommand = COMPLETE_STOP_WALKING;
    pLAN->G2MData->WalkingModeCommand = CCWROT_WALKING;
    pLAN->G2MData->StepNum = ui->LE_STEP_NUM->text().toInt();
    pLAN->G2MData->StepLength = ui->LE_STEP_LENGTH->text().toDouble();
    pLAN->G2MData->StepAngle = ui->LE_ROT_ANGLE->text().toDouble();
    if(pLAN->G2MData->StepLength>0.35) return;
    if(pLAN->G2MData->StepLength<0.001) return;
    if(pLAN->G2MData->StepAngle>45) return;
//    if(pLAN->G2MData->StepNum>30) return;

        cmd.COMMAND_DATA.USER_PARA_INT[4] = INSIDE_WALKING;

    cmd.COMMAND_DATA.USER_PARA_INT[9] = NORMAL_WALKING;
    cmd.COMMAND_DATA.USER_COMMAND = FREEWALK_WALK;
    cmd.COMMAND_TARGET = alNumFreeWalking;
    pLAN->SendCommand(cmd);
}

void OriginalDemoDialog::on_BTN_DATA_SAVE_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = 0;
    cmd.COMMAND_DATA.USER_COMMAND = FREEWALK_SAVE;
    cmd.COMMAND_TARGET = ALNum_Demo;
    pLAN->SendCommand(cmd);
}

void OriginalDemoDialog::on_BTN_DATA_SAVE_2_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = 888;
    cmd.COMMAND_TARGET = 0;
    pLAN->SendCommand(cmd);
}

void OriginalDemoDialog::on_BTN_DATA_SAVE_START_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = 1;
    cmd.COMMAND_DATA.USER_COMMAND = FREEWALK_SAVE;
    cmd.COMMAND_TARGET = ALNum_Demo;
    pLAN->SendCommand(cmd);
}

void OriginalDemoDialog::on_BTN_INT_PYEONGCHANG_WALK_clicked()
{
    USER_COMMAND cmd;
    pLAN->G2MData->StepTime = Pyeongchang_walking_speed;

    cmd.COMMAND_DATA.USER_PARA_INT[4] = INSIDE_WALKING;
    cmd.COMMAND_DATA.USER_PARA_INT[9] = NORMAL_WALKING;
    cmd.COMMAND_DATA.USER_COMMAND = FREEWALK_PYEONGCHANG;
    cmd.COMMAND_TARGET = alNumFreeWalking;
    pLAN->SendCommand(cmd);
}

void OriginalDemoDialog::on_BTN_INT_PYEONGCHANG_WALK_2_clicked()
{
//    USER_COMMAND cmd;
//    pLAN->G2MData->StepTime = Pyeongchang_walking_speed;//0.45f;
//    pLAN->G2MData->WalkingStopModeCommand = COMPLETE_STOP_WALKING;
//    pLAN->G2MData->WalkingModeCommand = CWROT_WALKING;
//    pLAN->G2MData->StepNum = 65;//ui->LE_STEP_NUM->text().toInt();
//    pLAN->G2MData->StepLength = 0.001;//ui->LE_STEP_LENGTH->text().toDouble();
//    pLAN->G2MData->StepAngle = 2.0;//ui->LE_ROT_ANGLE->text().toDouble();
//    if(pLAN->G2MData->StepLength>0.35) return;
//    if(pLAN->G2MData->StepLength<0.001) return;
//    if(pLAN->G2MData->StepAngle>45) return;
////    if(pLAN->G2MData->StepNum>30) return;

//        cmd.COMMAND_DATA.USER_PARA_INT[4] = INSIDE_WALKING;

//    cmd.COMMAND_DATA.USER_PARA_INT[9] = NORMAL_WALKING;
//    cmd.COMMAND_DATA.USER_COMMAND = FREEWALK_WALK;
//    cmd.COMMAND_TARGET = alNumFreeWalking;
//    pLAN->SendCommand(cmd);
}


void OriginalDemoDialog::on_BTN_DANCE_GOGOGO_2_clicked()
{

    demoSound[145]->play();
    on_BTN_INT_PYEONGCHANG_WALK_clicked();

    usleep(1000000);

    USER_COMMAND cmd;
    cmd.COMMAND_TARGET = ALNum_Demo;//_TARGET_AL;
    cmd.COMMAND_DATA.USER_COMMAND = SCI_DEMO;
    cmd.COMMAND_DATA.USER_PARA_INT[3] = DANCE_GOGOGO;
    cmd.COMMAND_DATA.USER_PARA_INT[11] = 1;
    cmd.COMMAND_DATA.USER_PARA_INT[10] = 1;
    pLAN->SendCommand(cmd);

    ui->BTN_DANCE_GOGOGO_2->setDisabled(true);
}
