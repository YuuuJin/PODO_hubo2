#ifndef TUTORIALDIALOG_H
#define TUTORIALDIALOG_H

#include <QDialog>
#include "CommonHeader.h"

namespace Ui {
class TutorialDialog;
}

class TutorialDialog : public QDialog
{
    Q_OBJECT

public:
    explicit TutorialDialog(QWidget *parent = 0);
    ~TutorialDialog();


private slots:
    void UpdateSomethings();

    void on_BTN_R_GRASP0_clicked();
    void on_BTN_R_GRASP1_clicked();
    void on_BTN_R_GRASP2_clicked();
    void on_BTN_R_GRASP3_clicked();
    void on_BTN_R_GRASP4_clicked();
    void on_BTN_R_GRASP_ALL_clicked();

    void on_BTN_R_RELEASE0_clicked();
    void on_BTN_R_RELEASE1_clicked();
    void on_BTN_R_RELEASE2_clicked();
    void on_BTN_R_RELEASE3_clicked();
    void on_BTN_R_RELEASE4_clicked();
    void on_BTN_R_RELEASE_ALL_clicked();

    void on_BTN_L_GRASP0_clicked();
    void on_BTN_L_GRASP1_clicked();
    void on_BTN_L_GRASP2_clicked();
    void on_BTN_L_GRASP3_clicked();
    void on_BTN_L_GRASP4_clicked();
    void on_BTN_L_GRASP_ALL_clicked();

    void on_BTN_L_RELEASE0_clicked();
    void on_BTN_L_RELEASE1_clicked();
    void on_BTN_L_RELEASE2_clicked();
    void on_BTN_L_RELEASE3_clicked();
    void on_BTN_L_RELEASE4_clicked();
    void on_BTN_L_RELEASE_ALL_clicked();

    void on_BTN_INIT_RHAND_clicked();
    void on_BTN_INIT_LHAND_clicked();
    void on_BTN_INIT_ALLHAND_clicked();

    void on_BTN_READ_MODI_clicked();
    void on_BTN_SET_MODI_clicked();

    void on_BTN_QP_START_clicked();

    void on_BTN_QP_STOP_clicked();

    void on_BTN_TEST_clicked();

    void on_BTN_IMU_NULL_clicked();

    void on_BTN_INIT_POS_clicked();

    void on_BTN_TCP_CONNECT_clicked();

private:
    Ui::TutorialDialog *ui;

    int ALNum_Tuto;
    int ALNum_Demo;

    int modiCnt;
};

#endif // TUTORIALDIALOG_H
