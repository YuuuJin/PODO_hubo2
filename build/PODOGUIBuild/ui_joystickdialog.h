/********************************************************************************
** Form generated from reading UI file 'joystickdialog.ui'
**
** Created by: Qt User Interface Compiler version 5.8.0
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_JOYSTICKDIALOG_H
#define UI_JOYSTICKDIALOG_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QDialog>
#include <QtWidgets/QFrame>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QTableWidget>

QT_BEGIN_NAMESPACE

class Ui_JoyStickDialog
{
public:
    QPushButton *JOY_BTN_START;
    QPushButton *JOY_BTN_STOP;
    QTableWidget *JOY_TABLE_INFO_LEFT;
    QTableWidget *JOY_TABLE_INFO_RIGHT;
    QPushButton *JOY_TAB_WHEELSTART;
    QPushButton *JOY_TAB_WHEELSTOP;
    QPushButton *MANUAL_BTN_START;
    QPushButton *MANUAL_BTN_STOP;
    QPushButton *MANUAL_BTN_GOTOTASKPOSE_0;
    QPushButton *MANUAL_BTN_GOTOTASKPOSE_180;
    QPushButton *MANUAL_BTN_GOTOMOVEPOSE_0;
    QPushButton *MANUAL_BTN_GOTOMOVEPOSE_180;
    QFrame *line;
    QFrame *line_2;
    QFrame *line_3;
    QPushButton *MANUAL_HP_PLUS;
    QPushButton *MANUAL_HP_MINUS;
    QFrame *line_4;
    QLineEdit *MANUAL_HP_ANGLE;
    QPushButton *MANUAL_RH_GRIB;
    QPushButton *MANUAL_RH_STOP;
    QPushButton *MANUAL_RH_OPEN;
    QPushButton *MANUAL_LH_STOP;
    QPushButton *MANUAL_LH_GRIB;
    QPushButton *MANUAL_LH_OPEN;
    QFrame *line_5;
    QLabel *label;
    QLabel *label_2;
    QLabel *label_3;
    QLabel *label_4;
    QLabel *label_5;
    QPushButton *MANUAL_RARM_GAIN_OVER_START;
    QPushButton *MANUAL_RARM_GAIN_OVER_RETURN;
    QPushButton *MANUAL_LARM_GAIN_OVER_START;
    QPushButton *MANUAL_LARM_GAIN_OVER_RETURN;
    QFrame *line_6;
    QPushButton *MANUAL_BTN_TWO_HAND_START;
    QPushButton *MANUAL_BTN_FOOT_START;
    QPushButton *MANUAL_BTN_TWO_FOOT_START;
    QPushButton *MANUAL_BTN_STANDING_ONE_HAND;
    QPushButton *MANUAL_BTN_STANDING_TWO_HAND;
    QPushButton *MANUAL_BTN_DRIVE_ON;
    QPushButton *MANUAL_BTN_DRIVE_OFF;
    QLabel *label_6;
    QLabel *label_7;
    QPushButton *BT_WALK_READY;
    QPushButton *BT_TEST_STOP;
    QLabel *label_8;
    QLineEdit *LE_STEP_TIME;
    QPushButton *BT_READY_TO_WALK;
    QLineEdit *LE_NO_OF_STEP;
    QPushButton *BT_LOGGING;
    QPushButton *BT_ONE_LEG_READY_2;
    QPushButton *BT_WALK_TEST;
    QLineEdit *LE_STEP_STRIDE;
    QPushButton *BT_DATA_SAVE;
    QPushButton *BT_ONE_LEG_READY;
    QLabel *label_9;
    QPushButton *MANUAL_BTN_JOY_WALKING_ON;
    QLineEdit *LE_JOYSTICK_WALKING_STATUS;
    QPushButton *MANUAL_BTN_JOY_WALKING_OFF;
    QLineEdit *LE_DEBUG;
    QPushButton *BT_HOME_POS;
    QPushButton *BT_WALK_FORWARD;
    QPushButton *BT_WALK_BACKWARD;

    void setupUi(QDialog *JoyStickDialog)
    {
        if (JoyStickDialog->objectName().isEmpty())
            JoyStickDialog->setObjectName(QStringLiteral("JoyStickDialog"));
        JoyStickDialog->resize(773, 718);
        JOY_BTN_START = new QPushButton(JoyStickDialog);
        JOY_BTN_START->setObjectName(QStringLiteral("JOY_BTN_START"));
        JOY_BTN_START->setGeometry(QRect(10, 10, 111, 31));
        JOY_BTN_STOP = new QPushButton(JoyStickDialog);
        JOY_BTN_STOP->setObjectName(QStringLiteral("JOY_BTN_STOP"));
        JOY_BTN_STOP->setEnabled(false);
        JOY_BTN_STOP->setGeometry(QRect(130, 10, 111, 31));
        JOY_TABLE_INFO_LEFT = new QTableWidget(JoyStickDialog);
        if (JOY_TABLE_INFO_LEFT->columnCount() < 1)
            JOY_TABLE_INFO_LEFT->setColumnCount(1);
        QTableWidgetItem *__qtablewidgetitem = new QTableWidgetItem();
        JOY_TABLE_INFO_LEFT->setHorizontalHeaderItem(0, __qtablewidgetitem);
        if (JOY_TABLE_INFO_LEFT->rowCount() < 9)
            JOY_TABLE_INFO_LEFT->setRowCount(9);
        QTableWidgetItem *__qtablewidgetitem1 = new QTableWidgetItem();
        JOY_TABLE_INFO_LEFT->setVerticalHeaderItem(0, __qtablewidgetitem1);
        QTableWidgetItem *__qtablewidgetitem2 = new QTableWidgetItem();
        JOY_TABLE_INFO_LEFT->setVerticalHeaderItem(1, __qtablewidgetitem2);
        QTableWidgetItem *__qtablewidgetitem3 = new QTableWidgetItem();
        JOY_TABLE_INFO_LEFT->setVerticalHeaderItem(2, __qtablewidgetitem3);
        QTableWidgetItem *__qtablewidgetitem4 = new QTableWidgetItem();
        JOY_TABLE_INFO_LEFT->setVerticalHeaderItem(3, __qtablewidgetitem4);
        QTableWidgetItem *__qtablewidgetitem5 = new QTableWidgetItem();
        JOY_TABLE_INFO_LEFT->setVerticalHeaderItem(4, __qtablewidgetitem5);
        QTableWidgetItem *__qtablewidgetitem6 = new QTableWidgetItem();
        JOY_TABLE_INFO_LEFT->setVerticalHeaderItem(5, __qtablewidgetitem6);
        QTableWidgetItem *__qtablewidgetitem7 = new QTableWidgetItem();
        JOY_TABLE_INFO_LEFT->setVerticalHeaderItem(6, __qtablewidgetitem7);
        QTableWidgetItem *__qtablewidgetitem8 = new QTableWidgetItem();
        JOY_TABLE_INFO_LEFT->setVerticalHeaderItem(7, __qtablewidgetitem8);
        QTableWidgetItem *__qtablewidgetitem9 = new QTableWidgetItem();
        JOY_TABLE_INFO_LEFT->setVerticalHeaderItem(8, __qtablewidgetitem9);
        JOY_TABLE_INFO_LEFT->setObjectName(QStringLiteral("JOY_TABLE_INFO_LEFT"));
        JOY_TABLE_INFO_LEFT->setGeometry(QRect(10, 50, 121, 271));
        QFont font;
        font.setPointSize(7);
        JOY_TABLE_INFO_LEFT->setFont(font);
        JOY_TABLE_INFO_LEFT->horizontalHeader()->setMinimumSectionSize(8);
        JOY_TABLE_INFO_LEFT->verticalHeader()->setDefaultSectionSize(27);
        JOY_TABLE_INFO_LEFT->verticalHeader()->setMinimumSectionSize(10);
        JOY_TABLE_INFO_RIGHT = new QTableWidget(JoyStickDialog);
        if (JOY_TABLE_INFO_RIGHT->columnCount() < 1)
            JOY_TABLE_INFO_RIGHT->setColumnCount(1);
        QTableWidgetItem *__qtablewidgetitem10 = new QTableWidgetItem();
        JOY_TABLE_INFO_RIGHT->setHorizontalHeaderItem(0, __qtablewidgetitem10);
        if (JOY_TABLE_INFO_RIGHT->rowCount() < 9)
            JOY_TABLE_INFO_RIGHT->setRowCount(9);
        QTableWidgetItem *__qtablewidgetitem11 = new QTableWidgetItem();
        JOY_TABLE_INFO_RIGHT->setVerticalHeaderItem(0, __qtablewidgetitem11);
        QTableWidgetItem *__qtablewidgetitem12 = new QTableWidgetItem();
        JOY_TABLE_INFO_RIGHT->setVerticalHeaderItem(1, __qtablewidgetitem12);
        QTableWidgetItem *__qtablewidgetitem13 = new QTableWidgetItem();
        JOY_TABLE_INFO_RIGHT->setVerticalHeaderItem(2, __qtablewidgetitem13);
        QTableWidgetItem *__qtablewidgetitem14 = new QTableWidgetItem();
        JOY_TABLE_INFO_RIGHT->setVerticalHeaderItem(3, __qtablewidgetitem14);
        QTableWidgetItem *__qtablewidgetitem15 = new QTableWidgetItem();
        JOY_TABLE_INFO_RIGHT->setVerticalHeaderItem(4, __qtablewidgetitem15);
        QTableWidgetItem *__qtablewidgetitem16 = new QTableWidgetItem();
        JOY_TABLE_INFO_RIGHT->setVerticalHeaderItem(5, __qtablewidgetitem16);
        QTableWidgetItem *__qtablewidgetitem17 = new QTableWidgetItem();
        JOY_TABLE_INFO_RIGHT->setVerticalHeaderItem(6, __qtablewidgetitem17);
        QTableWidgetItem *__qtablewidgetitem18 = new QTableWidgetItem();
        JOY_TABLE_INFO_RIGHT->setVerticalHeaderItem(7, __qtablewidgetitem18);
        QTableWidgetItem *__qtablewidgetitem19 = new QTableWidgetItem();
        JOY_TABLE_INFO_RIGHT->setVerticalHeaderItem(8, __qtablewidgetitem19);
        JOY_TABLE_INFO_RIGHT->setObjectName(QStringLiteral("JOY_TABLE_INFO_RIGHT"));
        JOY_TABLE_INFO_RIGHT->setGeometry(QRect(130, 50, 121, 271));
        JOY_TABLE_INFO_RIGHT->setFont(font);
        JOY_TABLE_INFO_RIGHT->horizontalHeader()->setMinimumSectionSize(8);
        JOY_TABLE_INFO_RIGHT->verticalHeader()->setDefaultSectionSize(27);
        JOY_TABLE_INFO_RIGHT->verticalHeader()->setMinimumSectionSize(10);
        JOY_TAB_WHEELSTART = new QPushButton(JoyStickDialog);
        JOY_TAB_WHEELSTART->setObjectName(QStringLiteral("JOY_TAB_WHEELSTART"));
        JOY_TAB_WHEELSTART->setEnabled(false);
        JOY_TAB_WHEELSTART->setGeometry(QRect(270, 10, 181, 31));
        JOY_TAB_WHEELSTOP = new QPushButton(JoyStickDialog);
        JOY_TAB_WHEELSTOP->setObjectName(QStringLiteral("JOY_TAB_WHEELSTOP"));
        JOY_TAB_WHEELSTOP->setEnabled(false);
        JOY_TAB_WHEELSTOP->setGeometry(QRect(460, 10, 181, 31));
        MANUAL_BTN_START = new QPushButton(JoyStickDialog);
        MANUAL_BTN_START->setObjectName(QStringLiteral("MANUAL_BTN_START"));
        MANUAL_BTN_START->setEnabled(true);
        MANUAL_BTN_START->setGeometry(QRect(290, 50, 151, 31));
        MANUAL_BTN_STOP = new QPushButton(JoyStickDialog);
        MANUAL_BTN_STOP->setObjectName(QStringLiteral("MANUAL_BTN_STOP"));
        MANUAL_BTN_STOP->setEnabled(true);
        MANUAL_BTN_STOP->setGeometry(QRect(600, 50, 111, 111));
        MANUAL_BTN_GOTOTASKPOSE_0 = new QPushButton(JoyStickDialog);
        MANUAL_BTN_GOTOTASKPOSE_0->setObjectName(QStringLiteral("MANUAL_BTN_GOTOTASKPOSE_0"));
        MANUAL_BTN_GOTOTASKPOSE_0->setEnabled(true);
        MANUAL_BTN_GOTOTASKPOSE_0->setGeometry(QRect(460, 180, 181, 31));
        MANUAL_BTN_GOTOTASKPOSE_180 = new QPushButton(JoyStickDialog);
        MANUAL_BTN_GOTOTASKPOSE_180->setObjectName(QStringLiteral("MANUAL_BTN_GOTOTASKPOSE_180"));
        MANUAL_BTN_GOTOTASKPOSE_180->setEnabled(true);
        MANUAL_BTN_GOTOTASKPOSE_180->setGeometry(QRect(270, 180, 181, 31));
        MANUAL_BTN_GOTOMOVEPOSE_0 = new QPushButton(JoyStickDialog);
        MANUAL_BTN_GOTOMOVEPOSE_0->setObjectName(QStringLiteral("MANUAL_BTN_GOTOMOVEPOSE_0"));
        MANUAL_BTN_GOTOMOVEPOSE_0->setEnabled(true);
        MANUAL_BTN_GOTOMOVEPOSE_0->setGeometry(QRect(270, 220, 181, 31));
        MANUAL_BTN_GOTOMOVEPOSE_180 = new QPushButton(JoyStickDialog);
        MANUAL_BTN_GOTOMOVEPOSE_180->setObjectName(QStringLiteral("MANUAL_BTN_GOTOMOVEPOSE_180"));
        MANUAL_BTN_GOTOMOVEPOSE_180->setEnabled(true);
        MANUAL_BTN_GOTOMOVEPOSE_180->setGeometry(QRect(460, 220, 181, 31));
        line = new QFrame(JoyStickDialog);
        line->setObjectName(QStringLiteral("line"));
        line->setGeometry(QRect(250, 10, 20, 501));
        line->setFrameShape(QFrame::VLine);
        line->setFrameShadow(QFrame::Sunken);
        line_2 = new QFrame(JoyStickDialog);
        line_2->setObjectName(QStringLiteral("line_2"));
        line_2->setGeometry(QRect(270, 160, 491, 16));
        line_2->setFrameShape(QFrame::HLine);
        line_2->setFrameShadow(QFrame::Sunken);
        line_3 = new QFrame(JoyStickDialog);
        line_3->setObjectName(QStringLiteral("line_3"));
        line_3->setGeometry(QRect(270, 250, 491, 16));
        line_3->setFrameShape(QFrame::HLine);
        line_3->setFrameShadow(QFrame::Sunken);
        MANUAL_HP_PLUS = new QPushButton(JoyStickDialog);
        MANUAL_HP_PLUS->setObjectName(QStringLiteral("MANUAL_HP_PLUS"));
        MANUAL_HP_PLUS->setGeometry(QRect(530, 270, 111, 31));
        MANUAL_HP_MINUS = new QPushButton(JoyStickDialog);
        MANUAL_HP_MINUS->setObjectName(QStringLiteral("MANUAL_HP_MINUS"));
        MANUAL_HP_MINUS->setGeometry(QRect(270, 270, 111, 31));
        line_4 = new QFrame(JoyStickDialog);
        line_4->setObjectName(QStringLiteral("line_4"));
        line_4->setGeometry(QRect(270, 310, 491, 16));
        line_4->setFrameShape(QFrame::HLine);
        line_4->setFrameShadow(QFrame::Sunken);
        MANUAL_HP_ANGLE = new QLineEdit(JoyStickDialog);
        MANUAL_HP_ANGLE->setObjectName(QStringLiteral("MANUAL_HP_ANGLE"));
        MANUAL_HP_ANGLE->setGeometry(QRect(400, 270, 113, 31));
        MANUAL_RH_GRIB = new QPushButton(JoyStickDialog);
        MANUAL_RH_GRIB->setObjectName(QStringLiteral("MANUAL_RH_GRIB"));
        MANUAL_RH_GRIB->setGeometry(QRect(270, 330, 111, 31));
        MANUAL_RH_STOP = new QPushButton(JoyStickDialog);
        MANUAL_RH_STOP->setObjectName(QStringLiteral("MANUAL_RH_STOP"));
        MANUAL_RH_STOP->setGeometry(QRect(400, 330, 111, 31));
        MANUAL_RH_OPEN = new QPushButton(JoyStickDialog);
        MANUAL_RH_OPEN->setObjectName(QStringLiteral("MANUAL_RH_OPEN"));
        MANUAL_RH_OPEN->setGeometry(QRect(530, 330, 111, 31));
        MANUAL_LH_STOP = new QPushButton(JoyStickDialog);
        MANUAL_LH_STOP->setObjectName(QStringLiteral("MANUAL_LH_STOP"));
        MANUAL_LH_STOP->setGeometry(QRect(400, 370, 111, 31));
        MANUAL_LH_GRIB = new QPushButton(JoyStickDialog);
        MANUAL_LH_GRIB->setObjectName(QStringLiteral("MANUAL_LH_GRIB"));
        MANUAL_LH_GRIB->setGeometry(QRect(270, 370, 111, 31));
        MANUAL_LH_OPEN = new QPushButton(JoyStickDialog);
        MANUAL_LH_OPEN->setObjectName(QStringLiteral("MANUAL_LH_OPEN"));
        MANUAL_LH_OPEN->setGeometry(QRect(530, 370, 111, 31));
        line_5 = new QFrame(JoyStickDialog);
        line_5->setObjectName(QStringLiteral("line_5"));
        line_5->setGeometry(QRect(270, 410, 491, 16));
        line_5->setFrameShape(QFrame::HLine);
        line_5->setFrameShadow(QFrame::Sunken);
        label = new QLabel(JoyStickDialog);
        label->setObjectName(QStringLiteral("label"));
        label->setGeometry(QRect(680, 10, 81, 21));
        label_2 = new QLabel(JoyStickDialog);
        label_2->setObjectName(QStringLiteral("label_2"));
        label_2->setGeometry(QRect(660, 200, 81, 21));
        label_3 = new QLabel(JoyStickDialog);
        label_3->setObjectName(QStringLiteral("label_3"));
        label_3->setGeometry(QRect(660, 280, 81, 21));
        label_4 = new QLabel(JoyStickDialog);
        label_4->setObjectName(QStringLiteral("label_4"));
        label_4->setGeometry(QRect(660, 350, 81, 21));
        label_5 = new QLabel(JoyStickDialog);
        label_5->setObjectName(QStringLiteral("label_5"));
        label_5->setGeometry(QRect(660, 450, 81, 21));
        MANUAL_RARM_GAIN_OVER_START = new QPushButton(JoyStickDialog);
        MANUAL_RARM_GAIN_OVER_START->setObjectName(QStringLiteral("MANUAL_RARM_GAIN_OVER_START"));
        MANUAL_RARM_GAIN_OVER_START->setGeometry(QRect(270, 430, 111, 31));
        MANUAL_RARM_GAIN_OVER_RETURN = new QPushButton(JoyStickDialog);
        MANUAL_RARM_GAIN_OVER_RETURN->setObjectName(QStringLiteral("MANUAL_RARM_GAIN_OVER_RETURN"));
        MANUAL_RARM_GAIN_OVER_RETURN->setGeometry(QRect(400, 430, 111, 31));
        MANUAL_LARM_GAIN_OVER_START = new QPushButton(JoyStickDialog);
        MANUAL_LARM_GAIN_OVER_START->setObjectName(QStringLiteral("MANUAL_LARM_GAIN_OVER_START"));
        MANUAL_LARM_GAIN_OVER_START->setGeometry(QRect(270, 470, 111, 31));
        MANUAL_LARM_GAIN_OVER_RETURN = new QPushButton(JoyStickDialog);
        MANUAL_LARM_GAIN_OVER_RETURN->setObjectName(QStringLiteral("MANUAL_LARM_GAIN_OVER_RETURN"));
        MANUAL_LARM_GAIN_OVER_RETURN->setGeometry(QRect(400, 470, 111, 31));
        line_6 = new QFrame(JoyStickDialog);
        line_6->setObjectName(QStringLiteral("line_6"));
        line_6->setGeometry(QRect(20, 500, 741, 20));
        line_6->setFrameShape(QFrame::HLine);
        line_6->setFrameShadow(QFrame::Sunken);
        MANUAL_BTN_TWO_HAND_START = new QPushButton(JoyStickDialog);
        MANUAL_BTN_TWO_HAND_START->setObjectName(QStringLiteral("MANUAL_BTN_TWO_HAND_START"));
        MANUAL_BTN_TWO_HAND_START->setEnabled(true);
        MANUAL_BTN_TWO_HAND_START->setGeometry(QRect(460, 50, 141, 31));
        MANUAL_BTN_FOOT_START = new QPushButton(JoyStickDialog);
        MANUAL_BTN_FOOT_START->setObjectName(QStringLiteral("MANUAL_BTN_FOOT_START"));
        MANUAL_BTN_FOOT_START->setEnabled(true);
        MANUAL_BTN_FOOT_START->setGeometry(QRect(290, 90, 151, 31));
        MANUAL_BTN_TWO_FOOT_START = new QPushButton(JoyStickDialog);
        MANUAL_BTN_TWO_FOOT_START->setObjectName(QStringLiteral("MANUAL_BTN_TWO_FOOT_START"));
        MANUAL_BTN_TWO_FOOT_START->setEnabled(true);
        MANUAL_BTN_TWO_FOOT_START->setGeometry(QRect(460, 90, 141, 31));
        MANUAL_BTN_STANDING_ONE_HAND = new QPushButton(JoyStickDialog);
        MANUAL_BTN_STANDING_ONE_HAND->setObjectName(QStringLiteral("MANUAL_BTN_STANDING_ONE_HAND"));
        MANUAL_BTN_STANDING_ONE_HAND->setEnabled(true);
        MANUAL_BTN_STANDING_ONE_HAND->setGeometry(QRect(290, 130, 151, 31));
        MANUAL_BTN_STANDING_TWO_HAND = new QPushButton(JoyStickDialog);
        MANUAL_BTN_STANDING_TWO_HAND->setObjectName(QStringLiteral("MANUAL_BTN_STANDING_TWO_HAND"));
        MANUAL_BTN_STANDING_TWO_HAND->setEnabled(true);
        MANUAL_BTN_STANDING_TWO_HAND->setGeometry(QRect(460, 130, 141, 31));
        MANUAL_BTN_DRIVE_ON = new QPushButton(JoyStickDialog);
        MANUAL_BTN_DRIVE_ON->setObjectName(QStringLiteral("MANUAL_BTN_DRIVE_ON"));
        MANUAL_BTN_DRIVE_ON->setEnabled(true);
        MANUAL_BTN_DRIVE_ON->setGeometry(QRect(190, 330, 16, 21));
        MANUAL_BTN_DRIVE_OFF = new QPushButton(JoyStickDialog);
        MANUAL_BTN_DRIVE_OFF->setObjectName(QStringLiteral("MANUAL_BTN_DRIVE_OFF"));
        MANUAL_BTN_DRIVE_OFF->setEnabled(true);
        MANUAL_BTN_DRIVE_OFF->setGeometry(QRect(210, 330, 16, 21));
        label_6 = new QLabel(JoyStickDialog);
        label_6->setObjectName(QStringLiteral("label_6"));
        label_6->setGeometry(QRect(400, 590, 101, 17));
        label_7 = new QLabel(JoyStickDialog);
        label_7->setObjectName(QStringLiteral("label_7"));
        label_7->setGeometry(QRect(200, 590, 81, 17));
        BT_WALK_READY = new QPushButton(JoyStickDialog);
        BT_WALK_READY->setObjectName(QStringLiteral("BT_WALK_READY"));
        BT_WALK_READY->setGeometry(QRect(200, 520, 141, 41));
        BT_TEST_STOP = new QPushButton(JoyStickDialog);
        BT_TEST_STOP->setObjectName(QStringLiteral("BT_TEST_STOP"));
        BT_TEST_STOP->setGeometry(QRect(20, 630, 141, 61));
        label_8 = new QLabel(JoyStickDialog);
        label_8->setObjectName(QStringLiteral("label_8"));
        label_8->setGeometry(QRect(300, 590, 91, 17));
        LE_STEP_TIME = new QLineEdit(JoyStickDialog);
        LE_STEP_TIME->setObjectName(QStringLiteral("LE_STEP_TIME"));
        LE_STEP_TIME->setGeometry(QRect(300, 610, 91, 27));
        BT_READY_TO_WALK = new QPushButton(JoyStickDialog);
        BT_READY_TO_WALK->setObjectName(QStringLiteral("BT_READY_TO_WALK"));
        BT_READY_TO_WALK->setGeometry(QRect(380, 650, 111, 51));
        LE_NO_OF_STEP = new QLineEdit(JoyStickDialog);
        LE_NO_OF_STEP->setObjectName(QStringLiteral("LE_NO_OF_STEP"));
        LE_NO_OF_STEP->setGeometry(QRect(200, 610, 91, 27));
        BT_LOGGING = new QPushButton(JoyStickDialog);
        BT_LOGGING->setObjectName(QStringLiteral("BT_LOGGING"));
        BT_LOGGING->setGeometry(QRect(200, 650, 161, 51));
        BT_ONE_LEG_READY_2 = new QPushButton(JoyStickDialog);
        BT_ONE_LEG_READY_2->setObjectName(QStringLiteral("BT_ONE_LEG_READY_2"));
        BT_ONE_LEG_READY_2->setGeometry(QRect(580, 510, 151, 31));
        BT_WALK_TEST = new QPushButton(JoyStickDialog);
        BT_WALK_TEST->setObjectName(QStringLiteral("BT_WALK_TEST"));
        BT_WALK_TEST->setGeometry(QRect(520, 590, 171, 51));
        LE_STEP_STRIDE = new QLineEdit(JoyStickDialog);
        LE_STEP_STRIDE->setObjectName(QStringLiteral("LE_STEP_STRIDE"));
        LE_STEP_STRIDE->setGeometry(QRect(400, 610, 91, 27));
        BT_DATA_SAVE = new QPushButton(JoyStickDialog);
        BT_DATA_SAVE->setObjectName(QStringLiteral("BT_DATA_SAVE"));
        BT_DATA_SAVE->setGeometry(QRect(20, 560, 141, 61));
        BT_ONE_LEG_READY = new QPushButton(JoyStickDialog);
        BT_ONE_LEG_READY->setObjectName(QStringLiteral("BT_ONE_LEG_READY"));
        BT_ONE_LEG_READY->setGeometry(QRect(580, 540, 151, 31));
        label_9 = new QLabel(JoyStickDialog);
        label_9->setObjectName(QStringLiteral("label_9"));
        label_9->setGeometry(QRect(30, 520, 81, 21));
        MANUAL_BTN_JOY_WALKING_ON = new QPushButton(JoyStickDialog);
        MANUAL_BTN_JOY_WALKING_ON->setObjectName(QStringLiteral("MANUAL_BTN_JOY_WALKING_ON"));
        MANUAL_BTN_JOY_WALKING_ON->setEnabled(true);
        MANUAL_BTN_JOY_WALKING_ON->setGeometry(QRect(20, 380, 161, 31));
        LE_JOYSTICK_WALKING_STATUS = new QLineEdit(JoyStickDialog);
        LE_JOYSTICK_WALKING_STATUS->setObjectName(QStringLiteral("LE_JOYSTICK_WALKING_STATUS"));
        LE_JOYSTICK_WALKING_STATUS->setEnabled(false);
        LE_JOYSTICK_WALKING_STATUS->setGeometry(QRect(200, 380, 41, 71));
        LE_JOYSTICK_WALKING_STATUS->setReadOnly(true);
        MANUAL_BTN_JOY_WALKING_OFF = new QPushButton(JoyStickDialog);
        MANUAL_BTN_JOY_WALKING_OFF->setObjectName(QStringLiteral("MANUAL_BTN_JOY_WALKING_OFF"));
        MANUAL_BTN_JOY_WALKING_OFF->setEnabled(true);
        MANUAL_BTN_JOY_WALKING_OFF->setGeometry(QRect(20, 420, 161, 31));
        LE_DEBUG = new QLineEdit(JoyStickDialog);
        LE_DEBUG->setObjectName(QStringLiteral("LE_DEBUG"));
        LE_DEBUG->setEnabled(false);
        LE_DEBUG->setGeometry(QRect(190, 470, 51, 31));
        BT_HOME_POS = new QPushButton(JoyStickDialog);
        BT_HOME_POS->setObjectName(QStringLiteral("BT_HOME_POS"));
        BT_HOME_POS->setGeometry(QRect(350, 520, 141, 41));
        BT_WALK_FORWARD = new QPushButton(JoyStickDialog);
        BT_WALK_FORWARD->setObjectName(QStringLiteral("BT_WALK_FORWARD"));
        BT_WALK_FORWARD->setGeometry(QRect(510, 660, 91, 51));
        BT_WALK_BACKWARD = new QPushButton(JoyStickDialog);
        BT_WALK_BACKWARD->setObjectName(QStringLiteral("BT_WALK_BACKWARD"));
        BT_WALK_BACKWARD->setGeometry(QRect(620, 660, 91, 51));

        retranslateUi(JoyStickDialog);

        QMetaObject::connectSlotsByName(JoyStickDialog);
    } // setupUi

    void retranslateUi(QDialog *JoyStickDialog)
    {
        JoyStickDialog->setWindowTitle(QApplication::translate("JoyStickDialog", "Dialog", Q_NULLPTR));
        JOY_BTN_START->setText(QApplication::translate("JoyStickDialog", "Joy Start", Q_NULLPTR));
        JOY_BTN_STOP->setText(QApplication::translate("JoyStickDialog", "Joy Stop", Q_NULLPTR));
        QTableWidgetItem *___qtablewidgetitem = JOY_TABLE_INFO_LEFT->horizontalHeaderItem(0);
        ___qtablewidgetitem->setText(QApplication::translate("JoyStickDialog", "Value", Q_NULLPTR));
        QTableWidgetItem *___qtablewidgetitem1 = JOY_TABLE_INFO_LEFT->verticalHeaderItem(0);
        ___qtablewidgetitem1->setText(QApplication::translate("JoyStickDialog", "L-LT", Q_NULLPTR));
        QTableWidgetItem *___qtablewidgetitem2 = JOY_TABLE_INFO_LEFT->verticalHeaderItem(1);
        ___qtablewidgetitem2->setText(QApplication::translate("JoyStickDialog", "L-LB", Q_NULLPTR));
        QTableWidgetItem *___qtablewidgetitem3 = JOY_TABLE_INFO_LEFT->verticalHeaderItem(2);
        ___qtablewidgetitem3->setText(QApplication::translate("JoyStickDialog", "L-JOG-RL", Q_NULLPTR));
        QTableWidgetItem *___qtablewidgetitem4 = JOY_TABLE_INFO_LEFT->verticalHeaderItem(3);
        ___qtablewidgetitem4->setText(QApplication::translate("JoyStickDialog", "L-JOG-UD", Q_NULLPTR));
        QTableWidgetItem *___qtablewidgetitem5 = JOY_TABLE_INFO_LEFT->verticalHeaderItem(4);
        ___qtablewidgetitem5->setText(QApplication::translate("JoyStickDialog", "L-ARW_RL", Q_NULLPTR));
        QTableWidgetItem *___qtablewidgetitem6 = JOY_TABLE_INFO_LEFT->verticalHeaderItem(5);
        ___qtablewidgetitem6->setText(QApplication::translate("JoyStickDialog", "L-ARW_UD", Q_NULLPTR));
        QTableWidgetItem *___qtablewidgetitem7 = JOY_TABLE_INFO_LEFT->verticalHeaderItem(6);
        ___qtablewidgetitem7->setText(QApplication::translate("JoyStickDialog", "R-JOG-BTN", Q_NULLPTR));
        QTableWidgetItem *___qtablewidgetitem8 = JOY_TABLE_INFO_LEFT->verticalHeaderItem(7);
        ___qtablewidgetitem8->setText(QApplication::translate("JoyStickDialog", "L-JOG-BTN", Q_NULLPTR));
        QTableWidgetItem *___qtablewidgetitem9 = JOY_TABLE_INFO_LEFT->verticalHeaderItem(8);
        ___qtablewidgetitem9->setText(QApplication::translate("JoyStickDialog", "BTN-BACK", Q_NULLPTR));
        QTableWidgetItem *___qtablewidgetitem10 = JOY_TABLE_INFO_RIGHT->horizontalHeaderItem(0);
        ___qtablewidgetitem10->setText(QApplication::translate("JoyStickDialog", "Value", Q_NULLPTR));
        QTableWidgetItem *___qtablewidgetitem11 = JOY_TABLE_INFO_RIGHT->verticalHeaderItem(0);
        ___qtablewidgetitem11->setText(QApplication::translate("JoyStickDialog", "R-RT", Q_NULLPTR));
        QTableWidgetItem *___qtablewidgetitem12 = JOY_TABLE_INFO_RIGHT->verticalHeaderItem(1);
        ___qtablewidgetitem12->setText(QApplication::translate("JoyStickDialog", "R-RB", Q_NULLPTR));
        QTableWidgetItem *___qtablewidgetitem13 = JOY_TABLE_INFO_RIGHT->verticalHeaderItem(2);
        ___qtablewidgetitem13->setText(QApplication::translate("JoyStickDialog", "R-JOG-RL", Q_NULLPTR));
        QTableWidgetItem *___qtablewidgetitem14 = JOY_TABLE_INFO_RIGHT->verticalHeaderItem(3);
        ___qtablewidgetitem14->setText(QApplication::translate("JoyStickDialog", "R-JOG-UD", Q_NULLPTR));
        QTableWidgetItem *___qtablewidgetitem15 = JOY_TABLE_INFO_RIGHT->verticalHeaderItem(4);
        ___qtablewidgetitem15->setText(QApplication::translate("JoyStickDialog", "BTN-Y", Q_NULLPTR));
        QTableWidgetItem *___qtablewidgetitem16 = JOY_TABLE_INFO_RIGHT->verticalHeaderItem(5);
        ___qtablewidgetitem16->setText(QApplication::translate("JoyStickDialog", "BTN-X", Q_NULLPTR));
        QTableWidgetItem *___qtablewidgetitem17 = JOY_TABLE_INFO_RIGHT->verticalHeaderItem(6);
        ___qtablewidgetitem17->setText(QApplication::translate("JoyStickDialog", "BTN-B", Q_NULLPTR));
        QTableWidgetItem *___qtablewidgetitem18 = JOY_TABLE_INFO_RIGHT->verticalHeaderItem(7);
        ___qtablewidgetitem18->setText(QApplication::translate("JoyStickDialog", "BTN-A", Q_NULLPTR));
        QTableWidgetItem *___qtablewidgetitem19 = JOY_TABLE_INFO_RIGHT->verticalHeaderItem(8);
        ___qtablewidgetitem19->setText(QApplication::translate("JoyStickDialog", "BTN-START", Q_NULLPTR));
        JOY_TAB_WHEELSTART->setText(QApplication::translate("JoyStickDialog", "Wheel Manual Move Start", Q_NULLPTR));
        JOY_TAB_WHEELSTOP->setText(QApplication::translate("JoyStickDialog", "Wheel Manual Move Stop", Q_NULLPTR));
        MANUAL_BTN_START->setText(QApplication::translate("JoyStickDialog", "One Hand", Q_NULLPTR));
        MANUAL_BTN_STOP->setText(QApplication::translate("JoyStickDialog", "Manual Stop", Q_NULLPTR));
        MANUAL_BTN_GOTOTASKPOSE_0->setText(QApplication::translate("JoyStickDialog", "Task Pos(0)", Q_NULLPTR));
        MANUAL_BTN_GOTOTASKPOSE_180->setText(QApplication::translate("JoyStickDialog", "Task Pos(180)", Q_NULLPTR));
        MANUAL_BTN_GOTOMOVEPOSE_0->setText(QApplication::translate("JoyStickDialog", "Move Pos(0)", Q_NULLPTR));
        MANUAL_BTN_GOTOMOVEPOSE_180->setText(QApplication::translate("JoyStickDialog", "Move Pos(180)", Q_NULLPTR));
        MANUAL_HP_PLUS->setText(QApplication::translate("JoyStickDialog", "Pitch +", Q_NULLPTR));
        MANUAL_HP_MINUS->setText(QApplication::translate("JoyStickDialog", "Pitch -", Q_NULLPTR));
        MANUAL_HP_ANGLE->setText(QApplication::translate("JoyStickDialog", "10", Q_NULLPTR));
        MANUAL_RH_GRIB->setText(QApplication::translate("JoyStickDialog", "RH grib", Q_NULLPTR));
        MANUAL_RH_STOP->setText(QApplication::translate("JoyStickDialog", "RH stop", Q_NULLPTR));
        MANUAL_RH_OPEN->setText(QApplication::translate("JoyStickDialog", "RH open", Q_NULLPTR));
        MANUAL_LH_STOP->setText(QApplication::translate("JoyStickDialog", "LH stop", Q_NULLPTR));
        MANUAL_LH_GRIB->setText(QApplication::translate("JoyStickDialog", "LH grib", Q_NULLPTR));
        MANUAL_LH_OPEN->setText(QApplication::translate("JoyStickDialog", "LH open", Q_NULLPTR));
        label->setText(QApplication::translate("JoyStickDialog", "Mode", Q_NULLPTR));
        label_2->setText(QApplication::translate("JoyStickDialog", "Pos", Q_NULLPTR));
        label_3->setText(QApplication::translate("JoyStickDialog", "Torso ang", Q_NULLPTR));
        label_4->setText(QApplication::translate("JoyStickDialog", "Hand", Q_NULLPTR));
        label_5->setText(QApplication::translate("JoyStickDialog", "Gain Over", Q_NULLPTR));
        MANUAL_RARM_GAIN_OVER_START->setText(QApplication::translate("JoyStickDialog", "R low gain", Q_NULLPTR));
        MANUAL_RARM_GAIN_OVER_RETURN->setText(QApplication::translate("JoyStickDialog", "R high gain", Q_NULLPTR));
        MANUAL_LARM_GAIN_OVER_START->setText(QApplication::translate("JoyStickDialog", "L low gain", Q_NULLPTR));
        MANUAL_LARM_GAIN_OVER_RETURN->setText(QApplication::translate("JoyStickDialog", "L high gain", Q_NULLPTR));
        MANUAL_BTN_TWO_HAND_START->setText(QApplication::translate("JoyStickDialog", "Two Hand", Q_NULLPTR));
        MANUAL_BTN_FOOT_START->setText(QApplication::translate("JoyStickDialog", "One Foot", Q_NULLPTR));
        MANUAL_BTN_TWO_FOOT_START->setText(QApplication::translate("JoyStickDialog", "Two Foot", Q_NULLPTR));
        MANUAL_BTN_STANDING_ONE_HAND->setText(QApplication::translate("JoyStickDialog", "One Hand Standing", Q_NULLPTR));
        MANUAL_BTN_STANDING_TWO_HAND->setText(QApplication::translate("JoyStickDialog", "Two Hand Standing", Q_NULLPTR));
        MANUAL_BTN_DRIVE_ON->setText(QApplication::translate("JoyStickDialog", "Manual Drive On", Q_NULLPTR));
        MANUAL_BTN_DRIVE_OFF->setText(QApplication::translate("JoyStickDialog", "Manual Drive Off", Q_NULLPTR));
        label_6->setText(QApplication::translate("JoyStickDialog", "Step Stride(m)", Q_NULLPTR));
        label_7->setText(QApplication::translate("JoyStickDialog", "No Of Step", Q_NULLPTR));
        BT_WALK_READY->setText(QApplication::translate("JoyStickDialog", "Walk Ready", Q_NULLPTR));
        BT_TEST_STOP->setText(QApplication::translate("JoyStickDialog", "Stop", Q_NULLPTR));
        label_8->setText(QApplication::translate("JoyStickDialog", "Step Time(s)", Q_NULLPTR));
        LE_STEP_TIME->setText(QApplication::translate("JoyStickDialog", "0.7", Q_NULLPTR));
        BT_READY_TO_WALK->setText(QApplication::translate("JoyStickDialog", "Ready to Walk", Q_NULLPTR));
        LE_NO_OF_STEP->setText(QApplication::translate("JoyStickDialog", "5", Q_NULLPTR));
        BT_LOGGING->setText(QApplication::translate("JoyStickDialog", "Test", Q_NULLPTR));
        BT_ONE_LEG_READY_2->setText(QApplication::translate("JoyStickDialog", "Left Stance pos", Q_NULLPTR));
        BT_WALK_TEST->setText(QApplication::translate("JoyStickDialog", "Position Walking", Q_NULLPTR));
        LE_STEP_STRIDE->setText(QApplication::translate("JoyStickDialog", "0", Q_NULLPTR));
        BT_DATA_SAVE->setText(QApplication::translate("JoyStickDialog", "data save", Q_NULLPTR));
        BT_ONE_LEG_READY->setText(QApplication::translate("JoyStickDialog", "Right Stance pos", Q_NULLPTR));
        label_9->setText(QApplication::translate("JoyStickDialog", "HBWalking", Q_NULLPTR));
        MANUAL_BTN_JOY_WALKING_ON->setText(QApplication::translate("JoyStickDialog", "Joy Stick Walking On", Q_NULLPTR));
        MANUAL_BTN_JOY_WALKING_OFF->setText(QApplication::translate("JoyStickDialog", "Joy Stick Walking Off", Q_NULLPTR));
        LE_DEBUG->setText(QApplication::translate("JoyStickDialog", "10", Q_NULLPTR));
        BT_HOME_POS->setText(QApplication::translate("JoyStickDialog", "Home Pos", Q_NULLPTR));
        BT_WALK_FORWARD->setText(QApplication::translate("JoyStickDialog", "Forward\n"
"3 step", Q_NULLPTR));
        BT_WALK_BACKWARD->setText(QApplication::translate("JoyStickDialog", "Backward\n"
"3 step", Q_NULLPTR));
    } // retranslateUi

};

namespace Ui {
    class JoyStickDialog: public Ui_JoyStickDialog {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_JOYSTICKDIALOG_H
