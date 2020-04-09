#ifndef MPCWALKINGDIALOG_H
#define MPCWALKINGDIALOG_H

#include <QDialog>

namespace Ui {
class MPCWalkingDialog;
}

class MPCWalkingDialog : public QDialog
{
    Q_OBJECT

public:
    explicit MPCWalkingDialog(QWidget *parent = 0);
    ~MPCWalkingDialog();

private slots:
    void on_BTN_Walk_Ready_clicked();

    void on_BTN_FORWARD_clicked();

    void on_BTN_LEFT_clicked();

    void on_BTN_RIGHT_clicked();

    void on_BTN_CCW_clicked();

    void on_BTN_CW_clicked();

    void on_BTN_SAVE_START_clicked();

    void on_BTN_DATA_SAVE_clicked();

    void on_pushButton_8_clicked();

private:
    Ui::MPCWalkingDialog *ui;
    QTimer				*displayTimer;

    int					alNumMPCWalking;
    int					alNumWalkReady;
};

#endif // MPCWALKINGDIALOG_H
