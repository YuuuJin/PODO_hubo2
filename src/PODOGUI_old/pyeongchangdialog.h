#ifndef PYEONGCHANGDIALOG_H
#define PYEONGCHANGDIALOG_H

#include <QDialog>
#include <QSound>
#include <QString>

#include "CommonHeader.h"

enum WALKREADYCOMMAND
{
    WALKREADY_NO_ACT = 100,
    WALKREADY_GO_WALKREADYPOS,
    WALKREADY_GO_HOMEPOS,
    WALKREADY_WHEELCMD,
    WALKREADY_INFINITY
};
enum PYEONGCHANG_TASK_ENUM
{
    TASK_HANDSHAKE = 100,
    TASK_BOUQUET,
    TASK_MOTION3,
    TASK_MOTION4,
    TASK_MOTION5,
    TASK_MOTION6,
    TASK_MOTION7,
    TASK_MOTION8,
    TASK_MOTION9,
    TASK_MOTION10,
    NO_OF_MOTION
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
    NO_OF_BOW_MOTION,
    NO_OF_PYEONGCHANG_MOTION
};


namespace Ui {
class PyeongchangDialog;
}

class PyeongchangDialog : public QDialog
{
    Q_OBJECT

public:
    explicit PyeongchangDialog(QWidget *parent = 0);
    ~PyeongchangDialog();

    void GeneralCommand(int motion_num, int sound_num);
    void WalkreadyCommand(int motion_num, int sound_num);
    void OpenAlCommand(int Al_num);

private slots:
    void on_btn_motion1_clicked();

    void on_btn_motion2_clicked();

    void on_btn_motion3_clicked();

    void on_btn_motion4_clicked();

    void on_btn_motion5_clicked();

    void on_btn_motion6_clicked();

    void on_btn_motion7_clicked();

    void on_btn_motion8_clicked();

    void on_btn_motion9_clicked();

    void on_btn_motion10_clicked();

    void on_btn_enable_clicked();

    void on_btn_disable_clicked();

    void on_btn_Test1_clicked();

    void on_btn_Test2_clicked();

    void on_btn_hand_ready_clicked();

    void on_btn_hand_lg_clicked();

    void on_btn_hand_hg_clicked();

    void on_btn_hand_init_clicked();

    void on_btn_bouquet_ready_clicked();

    void on_btn_bouquet_give_clicked();

    void on_btn_bouquet_init_clicked();

    void on_btn_hand_gon_clicked();

    void on_btn_hand_goff_clicked();

    void on_btn_clap_ready_clicked();

    void on_btn_clap_one_clicked();

    void on_btn_clap_three_clicked();

    void on_btn_clap_korea_clicked();

    void on_btn_clap_init_clicked();

    void on_btn_head_yes_clicked();

    void on_btn_head_no_clicked();

    void on_btn_head_stretching_clicked();

    void on_btn_head_init_clicked();

    void on_btn_bow_one_clicked();

    void on_btn_bow_two_clicked();

    void on_btn_bow_two_w_clicked();

    void on_btn_bow_greeting_clicked();

    void on_btn_openal_clicked();

    void on_btn_walkready_clicked();

    void on_btn_homepos_clicked();

    void on_btn_leg_mode_clicked();

private:
    Ui::PyeongchangDialog *ui;

    int             openALnum;
    int             ALNum_Pyeongchang;
    int					alNumWalkReady;
    QSound          *demoSound[35];
    int             _sound;
};

#endif // PYEONGCHANGDIALOG_H
