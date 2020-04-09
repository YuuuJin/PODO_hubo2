#include "OriginalDemoDialog.h"
#include "ui_OriginalDemoDialog.h"

#include "BasicFiles/PODOALDialog.h"
#include <iostream>

#define SCI_DEMO                101
#define PYEONGCHANG_DEMO        88
#define SOUND_PATH          "../share/Wave/"

enum WALKREADYCOMMAND
{
    WALKREADY_NO_ACT = 100,
    WALKREADY_GO_WALKREADYPOS,
    WALKREADY_GO_HOMEPOS,
    WALKREADY_WHEELCMD,
    WALKREADY_INFINITY
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
    HANDSHAKE_READY = 10,
    HANDSHAKE_LOWGAIN,
    HANDSHAKE_GRIP_ON,
    HANDSHAKE_GRIP_OFF,
    HANDSHAKE_HIGHGAIN,
    HANDSHAKE_INIT,
    NO_OF_HANDSHAKE_MOTION,
    BOUQUET_READY = 20,
    BOUQUET_GIVE,
    BOUQUET_INIT,
    NO_OF_BOUQUET_MOTION,
    CLAP_READY = 30,
    CLAP_ONE,
    CLAP_THREE,
    CLAP_KOREA,
    CLAP_INIT,
    NO_OF_CLAP_MOTION,
    HEAD_YES = 40,
    HEAD_NO,
    HEAD_STRETCHING,
    HEAD_INIT,
    NO_OF_HEAD_MOTION,
    BOW_ONE = 50,
    BOW_TWO,
    BOW_TWO_W,
    BOW_GREETING,
    BOW_LAST_MOTION,
    NO_OF_BOW_MOTION,
    GUID_LEFT = 60,
    GUID_RIGHT,
    GUID_INIT,
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

    demoSound[0] = new QSound("../share/Wave/s2_1.wav");
//    demoSound[1] = new QSound("../share/Wave/s3_1.wav");
    demoSound[1] = new QSound("../share/Wave/hubo-pyeongchang.wav");
    demoSound[2] = new QSound("../share/Wave/s11_1.wav");

    demoSound[3] = new QSound("../share/Wave/s4_1.wav");
    demoSound[4] = new QSound("../share/Wave/s5_1.wav");
    demoSound[5] = new QSound("../share/Wave/s5_2.wav");
    demoSound[6] = new QSound("../share/Wave/s6_1.wav");
    demoSound[7] = new QSound("../share/Wave/s8_1.wav");
    demoSound[8] = new QSound("../share/Wave/s8_2.wav");
    demoSound[9] = new QSound("../share/Wave/s9_1.wav");
    demoSound[10] = new QSound("../share/Wave/s10_1.wav");
    demoSound[11] = new QSound("../share/Wave/s12_1.wav");
//    demoSound[12] = new QSound("../share/Wave/s17_1.wav");
    demoSound[12] = new QSound("../share/Wave/iwilldomybest.wav");
    demoSound[13] = new QSound("../share/Wave/s18_1.wav");
    demoSound[14] = new QSound("../share/Wave/s19_1.wav");
    demoSound[15] = new QSound("../share/Wave/s20_1.wav");
    demoSound[16] = new QSound("../share/Wave/s21_1.wav");
//    demoSound[17] = new QSound("../share/Wave/s23_1.wav");
    demoSound[17] = new QSound("../share/Wave/Nouee.wav");
    demoSound[18] = new QSound("../share/Wave/s26_1.wav");
    demoSound[19] = new QSound("../share/Wave/s28_1.wav");
    demoSound[20] = new QSound("../share/Wave/s29_1.wav");
    demoSound[21] = new QSound("../share/Wave/s29_2.wav");
    demoSound[22] = new QSound("../share/Wave/s29_3.wav");
    demoSound[23] = new QSound("../share/Wave/s30_1.wav");
    demoSound[24] = new QSound("../share/Wave/s31_1.wav");
    demoSound[25] = new QSound("../share/Wave/s32_1.wav");
    demoSound[26] = new QSound("../share/Wave/s33_1.wav");

//    demoSound[27] = new QSound("../share/Wave/GangNamStyle.wav");
    demoSound[27] = new QSound("../share/Wave/GangNamStyle.wav");
    demoSound[28] = new QSound("../share/Wave/pickme_30sec.wav");

    demoSound[29] = new QSound("../share/Wave/hihubonicetomeetu.wav");
    demoSound[30] = new QSound("../share/Wave/hihubo.wav");


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
void OriginalDemoDialog::on_BTN_FACE_SMILE_clicked()    {ChangeFace(1);}
void OriginalDemoDialog::on_BTN_FACE_IMPASSIVE_clicked(){ChangeFace(2);}
void OriginalDemoDialog::on_BTN_FACE_SAD_clicked()      {ChangeFace(3);}
void OriginalDemoDialog::on_BTN_FACE_ANGRY_clicked()    {ChangeFace(4);}
void OriginalDemoDialog::on_BTN_FACE_WONDER_clicked()   {ChangeFace(5);}
void OriginalDemoDialog::on_BTN_FACE_THINK_clicked()    {ChangeFace(6);}
void OriginalDemoDialog::on_BTN_FACE_WINK_clicked()     {ChangeFace(7);}
void OriginalDemoDialog::on_BTN_FACE_1_clicked()        {ChangeFace(17);}
void OriginalDemoDialog::on_BTN_FACE_2_clicked()        {ChangeFace(18);}
void OriginalDemoDialog::on_BTN_FACE_3_clicked()        {ChangeFace(19);}
void OriginalDemoDialog::on_BTN_FACE_YES_clicked()      {ChangeFace(33);}
void OriginalDemoDialog::on_BTN_FACE_NO_clicked()       {ChangeFace(34);}
void OriginalDemoDialog::on_BTN_FACE_HELLO_clicked()    {ChangeFace(35);}
void OriginalDemoDialog::on_BTN_FACE_HUBO_clicked()     {ChangeFace(36);}

/**********************************************************************************/




void OriginalDemoDialog::GeneralCommand(int _TARGET_AL,int motion_num, int sound_num){
    if(motion_num >= 0){
        USER_COMMAND cmd;
        cmd.COMMAND_TARGET = _TARGET_AL;
        if(cmd.COMMAND_TARGET == ALNum_Pyeongchang)
        cmd.COMMAND_DATA.USER_COMMAND = PYEONGCHANG_DEMO;//SCI_DEMO;
        else if(cmd.COMMAND_TARGET == ALNum_Demo)
            cmd.COMMAND_DATA.USER_COMMAND = SCI_DEMO;
        cmd.COMMAND_DATA.USER_PARA_INT[3] = motion_num;
        pLAN->SendCommand(cmd);
    }

    if(sound_num >= 0)
        demoSound[sound_num]->play();
}

/**********************************************************************************/
// Head
void OriginalDemoDialog::on_BTN_HEAD_YES_clicked()      {GeneralCommand(ALNum_Demo,8801, -1);ui->BTN_HEAD_YES->setDisabled(true);}
void OriginalDemoDialog::on_BTN_HEAD_NO_clicked()       {GeneralCommand(ALNum_Demo,8802, -1);ui->BTN_HEAD_NO->setDisabled(true);}
void OriginalDemoDialog::on_BTN_HEAD_QUESTION_clicked() {GeneralCommand(ALNum_Demo,13, -1);ui->BTN_HEAD_QUESTION->setDisabled(true);}
/**********************************************************************************/


/**********************************************************************************/
// Rock Papers Scissors
void OriginalDemoDialog::on_BTN_RSP_READY_clicked()     {GeneralCommand(ALNum_Demo,1401, -1);ui->BTN_RSP_READY->setDisabled(true);ui->BTN_RSP_DO->setDisabled(false);}
void OriginalDemoDialog::on_BTN_RSP_DO_clicked()        {GeneralCommand(ALNum_Demo,1402, -1);ui->BTN_RSP_DO->setDisabled(true);ui->BTN_RSP_READY->setDisabled(false);}
/**********************************************************************************/



/**********************************************************************************/
// Hubo2 Motion
void OriginalDemoDialog::on_BTN_MOTION_GREETING_clicked()   {GeneralCommand(ALNum_Demo,2, 29);ui->BTN_MOTION_GREETING->setDisabled(true);}
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
void OriginalDemoDialog::on_BTN_MOTION_OKAY_clicked()       {GeneralCommand(ALNum_Demo,11, 2);ui->BTN_MOTION_OKAY->setDisabled(true);}
void OriginalDemoDialog::on_BTN_MOTION_QUESTION_clicked()   {GeneralCommand(ALNum_Demo,12, 11);ui->BTN_MOTION_QUESTION->setDisabled(true);}
void OriginalDemoDialog::on_BTN_MOTION_CRY_clicked()        {GeneralCommand(ALNum_Demo,15, -1);ui->BTN_MOTION_CRY->setDisabled(true);}
void OriginalDemoDialog::on_BTN_MOTION_CUTECUTE_clicked()   {GeneralCommand(ALNum_Demo,20, 15);ui->BTN_MOTION_CUTECUTE->setDisabled(true);}
void OriginalDemoDialog::on_BTN_MOTION_COMEON_clicked()     {GeneralCommand(ALNum_Demo,16, -1);ui->BTN_MOTION_COMEON->setDisabled(true);}
void OriginalDemoDialog::on_BTN_MOTION_FIGHTING_clicked()   {GeneralCommand(ALNum_Demo,17, 12);ui->BTN_MOTION_FIGHTING->setDisabled(true);}
void OriginalDemoDialog::on_BTN_MOTION_LOOK_clicked()       {GeneralCommand(ALNum_Demo,1, -1);ui->BTN_MOTION_LOOK->setDisabled(true);}
void OriginalDemoDialog::on_BTN_MOTION_DANCECUTE_clicked()  {GeneralCommand(ALNum_Demo,21, 16);ui->BTN_MOTION_DANCECUTE->setDisabled(true);}
void OriginalDemoDialog::on_BTN_MOTION_CUTE_clicked()       {GeneralCommand(ALNum_Demo,26, 18);ui->BTN_MOTION_CUTE->setDisabled(true);}
void OriginalDemoDialog::on_BTN_MOTION_CHASE_clicked()      {GeneralCommand(ALNum_Demo,6, 6);ui->BTN_MOTION_CHASE->setDisabled(true);}
void OriginalDemoDialog::on_BTN_MOTION_SUWHA_clicked()      {GeneralCommand(ALNum_Demo,29, 20);ui->BTN_MOTION_SUWHA->setDisabled(true);}
void OriginalDemoDialog::on_BTN_MOTION_HEUNG_clicked()      {GeneralCommand(ALNum_Demo,31, 24);ui->BTN_MOTION_HEUNG->setDisabled(true);}
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
void OriginalDemoDialog::on_BTN_DANCE_25READY_clicked()     {                       ui->BTN_DANCE_25READY->setDisabled(true);ui->BTN_DANCE_SCENE25->setDisabled(false);ui->BTN_DANCE_FINISH->setDisabled(false);}  // TODO
void OriginalDemoDialog::on_BTN_DANCE_SCENE25_clicked()     {GeneralCommand(ALNum_Demo,25, 27);ui->BTN_DANCE_SCENE25->setDisabled(true);}
void OriginalDemoDialog::on_BTN_DANCE_FINISH_clicked()      {                       ui->BTN_DANCE_FINISH->setDisabled(true);ui->BTN_DANCE_25READY->setDisabled(false);}  // TODO
void OriginalDemoDialog::on_BTN_DANCE_PICKMEREADY_clicked() {GeneralCommand(ALNum_Demo,102, -1);ui->BTN_DANCE_PICKMEREADY->setDisabled(true);ui->BTN_DANCE_PICKME->setDisabled(false);}
void OriginalDemoDialog::on_BTN_DANCE_PICKME_clicked()      {GeneralCommand(ALNum_Demo,101, 28);ui->BTN_DANCE_PICKME->setDisabled(true);ui->BTN_DANCE_PICKMEREADY->setDisabled(false);}
void OriginalDemoDialog::on_BTN_DANCE_SCENE22_clicked()     {GeneralCommand(ALNum_Demo,22, -1);ui->BTN_DANCE_SCENE22->setDisabled(true);}
void OriginalDemoDialog::on_BTN_DANCE_ILOVEYOU_clicked()    {GeneralCommand(ALNum_Demo,32, 25);ui->BTN_DANCE_ILOVEYOU->setDisabled(true);}
void OriginalDemoDialog::on_BTN_DANCE_FINGERSHOW_clicked()  {GeneralCommand(ALNum_Demo,19, 14);ui->BTN_DANCE_FINGERSHOW->setDisabled(true);}
void OriginalDemoDialog::on_BTN_DANCE_HEUNG_clicked()       {GeneralCommand(ALNum_Demo,23, 17);ui->BTN_DANCE_HEUNG->setDisabled(true);}
/**********************************************************************************/



/**********************************************************************************/
// One Foot
void OriginalDemoDialog::on_BTN_ONEFOOT_ON_clicked()        {}  // TODO
void OriginalDemoDialog::on_BTN_ONEFOOT_POS_clicked()       {GeneralCommand(ALNum_Demo,1801, -1);}
void OriginalDemoDialog::on_BTN_ONEFOOT_DO_clicked()        {GeneralCommand(ALNum_Demo,1802, -1);}
void OriginalDemoDialog::on_BTN_ONEFOOT_V_clicked()         {GeneralCommand(ALNum_Demo,-1, 13);}
void OriginalDemoDialog::on_BTN_ONEFOOT_HOMEPOS_clicked()   {GeneralCommand(ALNum_Demo,1804, -1);}
/**********************************************************************************/



/**********************************************************************************/
// Control OnOff
void OriginalDemoDialog::on_BTN_CTRL_OFFPOS_clicked()       {GeneralCommand(ALNum_Demo,801, 7);}
void OriginalDemoDialog::on_BTN_CTRL_ONPOS_clicked()        {GeneralCommand(ALNum_Demo,802, 8);}
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
        /*****************************/


        /*****************************/
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
    }
    else
    {
        ui->BTN_ENABLE_DISABLE->setText("All Btn Disable");
        /**********************************************************************************/
        // Head
        ui->BTN_HEAD_YES->setDisabled(false);
        ui->BTN_HEAD_NO->setDisabled(false);
        ui->BTN_HEAD_QUESTION->setDisabled(false);
        /*****************************/


        /*****************************/
        // Rock Papers Scissors
        ui->BTN_RSP_READY->setDisabled(false);
        ui->BTN_RSP_DO->setDisabled(false);

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
    }

}

//void OriginalDemoDialog::on_BTN_test_001_clicked()
//{

//}

void OriginalDemoDialog::on_btn_hand_ready_clicked()
{
    GeneralCommand(ALNum_Pyeongchang,HANDSHAKE_READY,0);
}

void OriginalDemoDialog::on_btn_hand_gon_clicked()
{
    GeneralCommand(ALNum_Pyeongchang,HANDSHAKE_GRIP_ON,1);
}

void OriginalDemoDialog::on_btn_hand_goff_clicked()
{
    GeneralCommand(ALNum_Pyeongchang,HANDSHAKE_GRIP_OFF,0);
}

void OriginalDemoDialog::on_btn_hand_init_clicked()
{
    GeneralCommand(ALNum_Pyeongchang,HANDSHAKE_INIT,0);
}

void OriginalDemoDialog::on_btn_clap_ready_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_TARGET = ALNum_Pyeongchang;
    cmd.COMMAND_DATA.USER_COMMAND = PYEONGCHANG_DEMO;
    cmd.COMMAND_DATA.USER_PARA_INT[3] = CLAP_READY;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = ui->LE_CLAP_CNT->text().toInt();
    pLAN->SendCommand(cmd);
}

void OriginalDemoDialog::on_btn_bow_one_clicked()
{
    GeneralCommand(ALNum_Pyeongchang,BOW_ONE,3);
}

void OriginalDemoDialog::on_btn_bow_two_clicked()
{
    GeneralCommand(ALNum_Pyeongchang,BOW_TWO,3);
}

void OriginalDemoDialog::on_btn_leg_mode_clicked()
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

void OriginalDemoDialog::on_btn_bouquet_ready_clicked()
{
    GeneralCommand(ALNum_Pyeongchang,BOUQUET_READY,0);
}

void OriginalDemoDialog::on_btn_bouquet_give_clicked()
{
    GeneralCommand(ALNum_Pyeongchang,BOUQUET_GIVE,2);
}

void OriginalDemoDialog::on_btn_bouquet_init_clicked()
{
    GeneralCommand(ALNum_Pyeongchang,BOUQUET_INIT,0);
}

void OriginalDemoDialog::on_btn_guid_left_clicked()
{
    GeneralCommand(ALNum_Pyeongchang,GUID_LEFT,0);
//    ui->btn_guid_left->setEnabled(false);
}

void OriginalDemoDialog::on_btn_guid_right_clicked()
{
    GeneralCommand(ALNum_Pyeongchang,GUID_RIGHT,0);
//    ui->btn_guid_right->setEnabled(false);
}

void OriginalDemoDialog::on_btn_guid_init_clicked()
{
    GeneralCommand(ALNum_Pyeongchang,GUID_INIT,0);
//    ui->btn_guid_init->setEnabled(false);
}
