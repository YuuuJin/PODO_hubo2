#ifndef ORIGINALDEMODIALOG_H
#define ORIGINALDEMODIALOG_H

#include <QDialog>
#include <QString>
#include <QSound>
#include <QtSerialPort/QtSerialPort>

#include "CommonHeader.h"

namespace Ui {
class OriginalDemoDialog;
}

class OriginalDemoDialog : public QDialog
{
    Q_OBJECT

public:
    explicit OriginalDemoDialog(QWidget *parent = 0);
    ~OriginalDemoDialog();


    int RBSerialWrite(const char *_uart_frame, uchar _bytes, uchar _mode);
    int RBReadPort(char *_uart_frame, uchar _bytes, uchar _mode);

    void ChangeFace(int emotion);
    void GeneralCommand(int _TARGET_AL,int motion_num, int sound_num);


private slots:
    void on_BTN_WALKREADY_clicked();

    void on_BTN_GOTOHOME_clicked();

    void on_BTN_INIT_HAND_clicked();

    void on_BTN_FACE_SMILE_clicked();

    void on_BTN_FACE_IMPASSIVE_clicked();

    void on_BTN_FACE_SAD_clicked();

    void on_BTN_FACE_ANGRY_clicked();

    void on_BTN_FACE_WONDER_clicked();

    void on_BTN_FACE_THINK_clicked();

    void on_BTN_FACE_WINK_clicked();

    void on_BTN_FACE_1_clicked();

    void on_BTN_FACE_2_clicked();

    void on_BTN_FACE_3_clicked();

    void on_BTN_FACE_YES_clicked();

    void on_BTN_FACE_NO_clicked();

    void on_BTN_FACE_HELLO_clicked();

    void on_BTN_FACE_HUBO_clicked();

    void on_BTN_HEAD_YES_clicked();

    void on_BTN_HEAD_NO_clicked();

    void on_BTN_HEAD_QUESTION_clicked();

    void on_BTN_RSP_READY_clicked();

    void on_BTN_RSP_DO_clicked();

    void on_BTN_MOTION_GREETING_clicked();

    void on_BTN_MOTION_BOW_clicked();

    void on_BTN_MOTION_HAPUM_clicked();

    void on_BTN_MOTION_ONE_clicked();

    void on_BTN_MOTION_TWO_clicked();

    void on_BTN_MOTION_THREE_clicked();

    void on_BTN_MOTION_STORY1_clicked();

    void on_BTN_MOTION_STORY2_clicked();

    void on_BTN_MOTION_STORY3_clicked();

    void on_BTN_MOTION_STORY4_clicked();

    void on_BTN_MOTION_STORY5_clicked();

    void on_BTN_MOTION_STORY6_clicked();

    void on_BTN_MOTION_OKAY_clicked();

    void on_BTN_MOTION_QUESTION_clicked();

    void on_BTN_MOTION_CRY_clicked();

    void on_BTN_MOTION_CUTECUTE_clicked();

    void on_BTN_MOTION_COMEON_clicked();

    void on_BTN_MOTION_FIGHTING_clicked();

    void on_BTN_MOTION_LOOK_clicked();

    void on_BTN_MOTION_DANCECUTE_clicked();

    void on_BTN_MOTION_CUTE_clicked();

    void on_BTN_MOTION_CHASE_clicked();

    void on_BTN_MOTION_SUWHA_clicked();

    void on_BTN_MOTION_HEUNG_clicked();

    void on_BTN_815_HELLOL_clicked();

    void on_BTN_815_HELLOR_clicked();

    void on_BTN_815_STARTL_clicked();

    void on_BTN_815_STARTR_clicked();

    void on_BTN_815_FIGHTING_clicked();

    void on_BTN_815_PROUDKOREA_clicked();

    void on_BTN_815_STORY1_clicked();

    void on_BTN_DANCE_25READY_clicked();

    void on_BTN_DANCE_SCENE25_clicked();

    void on_BTN_DANCE_FINISH_clicked();

    void on_BTN_DANCE_PICKMEREADY_clicked();

    void on_BTN_DANCE_PICKME_clicked();

    void on_BTN_DANCE_SCENE22_clicked();

    void on_BTN_DANCE_ILOVEYOU_clicked();

    void on_BTN_DANCE_FINGERSHOW_clicked();

    void on_BTN_DANCE_HEUNG_clicked();

    void on_BTN_ONEFOOT_ON_clicked();

    void on_BTN_ONEFOOT_POS_clicked();

    void on_BTN_ONEFOOT_DO_clicked();

    void on_BTN_ONEFOOT_V_clicked();

    void on_BTN_ONEFOOT_HOMEPOS_clicked();

    void on_BTN_CTRL_OFFPOS_clicked();

    void on_BTN_CTRL_ONPOS_clicked();

    void on_BTN_CTRL_START_clicked();

    void on_BTN_CTRL_STOP_clicked();

    void on_BTN_ENABLE_DISABLE_clicked();

    void on_btn_hand_ready_clicked();

    void on_btn_hand_gon_clicked();

    void on_btn_hand_goff_clicked();

    void on_btn_hand_init_clicked();

    void on_btn_clap_ready_clicked();

    void on_btn_bow_one_clicked();

    void on_btn_bow_two_clicked();

    void on_btn_leg_mode_clicked();

    void on_btn_bouquet_ready_clicked();

    void on_btn_bouquet_give_clicked();

    void on_btn_bouquet_init_clicked();

    void on_btn_guid_left_clicked();

    void on_btn_guid_right_clicked();

    void on_btn_guid_init_clicked();

private:
    Ui::OriginalDemoDialog *ui;

    QSerialPort     *serial;
    int             serialHandler;
    int             ALNum_Demo;
    int				ALNumWalkReady;
    int				ALNum_Pyeongchang;


    QSound          *demoSound[35];
    int             _sound;

};

#endif // ORIGINALDEMODIALOG_H
