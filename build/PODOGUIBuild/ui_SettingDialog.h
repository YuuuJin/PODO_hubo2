/********************************************************************************
** Form generated from reading UI file 'SettingDialog.ui'
**
** Created by: Qt User Interface Compiler version 5.8.0
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_SETTINGDIALOG_H
#define UI_SETTINGDIALOG_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QDialog>
#include <QtWidgets/QFrame>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QRadioButton>
#include <QtWidgets/QTableWidget>

QT_BEGIN_NAMESPACE

class Ui_SettingDialog
{
public:
    QTableWidget *TW_0;
    QTableWidget *TW_1;
    QTableWidget *TW_2;
    QTableWidget *TW_3;
    QLabel *LB_SELECTED;
    QPushButton *BTN_CAN_CHECK;
    QPushButton *BTN_FIND_HOME;
    QGroupBox *groupBox_2;
    QPushButton *BTN_MOVE_JOINT;
    QLabel *label_2;
    QLineEdit *LE_MOVE_DEGREE;
    QLineEdit *LE_MOVE_TIME;
    QLabel *label;
    QFrame *line;
    QGroupBox *groupBox;
    QPushButton *BTN_EXECUTE_COMMAND;
    QRadioButton *RB_MODE_POS;
    QRadioButton *RB_MODE_PWM;
    QRadioButton *RB_INIT_POS;
    QRadioButton *RB_ENC_ZERO;
    QRadioButton *RB_FET_OFF;
    QRadioButton *RB_FET_ON;
    QRadioButton *RB_CTRL_OFF;
    QRadioButton *RB_CTRL_ON;
    QFrame *line_3;
    QFrame *line_4;
    QRadioButton *RB_ERROR_CLEAR;
    QRadioButton *RB_JOINT_RECOVER;
    QTableWidget *TW_4;
    QTableWidget *TW_5;
    QTableWidget *TW_6;
    QPushButton *BTN_INIT_HAND;
    QPushButton *BTN_INIT_HEAD;

    void setupUi(QDialog *SettingDialog)
    {
        if (SettingDialog->objectName().isEmpty())
            SettingDialog->setObjectName(QStringLiteral("SettingDialog"));
        SettingDialog->resize(760, 760);
        TW_0 = new QTableWidget(SettingDialog);
        TW_0->setObjectName(QStringLiteral("TW_0"));
        TW_0->setGeometry(QRect(10, 10, 241, 211));
        TW_1 = new QTableWidget(SettingDialog);
        TW_1->setObjectName(QStringLiteral("TW_1"));
        TW_1->setGeometry(QRect(260, 10, 241, 211));
        TW_2 = new QTableWidget(SettingDialog);
        TW_2->setObjectName(QStringLiteral("TW_2"));
        TW_2->setGeometry(QRect(10, 230, 241, 211));
        TW_2->setSelectionMode(QAbstractItemView::SingleSelection);
        TW_2->setSelectionBehavior(QAbstractItemView::SelectRows);
        TW_3 = new QTableWidget(SettingDialog);
        TW_3->setObjectName(QStringLiteral("TW_3"));
        TW_3->setGeometry(QRect(260, 230, 241, 211));
        LB_SELECTED = new QLabel(SettingDialog);
        LB_SELECTED->setObjectName(QStringLiteral("LB_SELECTED"));
        LB_SELECTED->setGeometry(QRect(530, 110, 121, 21));
        BTN_CAN_CHECK = new QPushButton(SettingDialog);
        BTN_CAN_CHECK->setObjectName(QStringLiteral("BTN_CAN_CHECK"));
        BTN_CAN_CHECK->setGeometry(QRect(520, 10, 101, 31));
        QFont font;
        font.setPointSize(10);
        BTN_CAN_CHECK->setFont(font);
        BTN_FIND_HOME = new QPushButton(SettingDialog);
        BTN_FIND_HOME->setObjectName(QStringLiteral("BTN_FIND_HOME"));
        BTN_FIND_HOME->setGeometry(QRect(650, 10, 101, 31));
        BTN_FIND_HOME->setFont(font);
        groupBox_2 = new QGroupBox(SettingDialog);
        groupBox_2->setObjectName(QStringLiteral("groupBox_2"));
        groupBox_2->setGeometry(QRect(520, 140, 231, 81));
        groupBox_2->setStyleSheet(QLatin1String("QGroupBox {\n"
"    border: 1px solid gray;\n"
"    border-radius: 5px;\n"
"    margin-top: 7px;\n"
"}\n"
"\n"
"QGroupBox::title {\n"
"    subcontrol-origin: margin;\n"
"    left: 8px;\n"
"    padding: 0 3px 0 3px;\n"
"}\n"
"\n"
""));
        BTN_MOVE_JOINT = new QPushButton(groupBox_2);
        BTN_MOVE_JOINT->setObjectName(QStringLiteral("BTN_MOVE_JOINT"));
        BTN_MOVE_JOINT->setGeometry(QRect(150, 20, 71, 51));
        BTN_MOVE_JOINT->setFont(font);
        label_2 = new QLabel(groupBox_2);
        label_2->setObjectName(QStringLiteral("label_2"));
        label_2->setGeometry(QRect(10, 50, 51, 21));
        label_2->setFont(font);
        label_2->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        LE_MOVE_DEGREE = new QLineEdit(groupBox_2);
        LE_MOVE_DEGREE->setObjectName(QStringLiteral("LE_MOVE_DEGREE"));
        LE_MOVE_DEGREE->setGeometry(QRect(70, 20, 61, 23));
        LE_MOVE_DEGREE->setFont(font);
        LE_MOVE_DEGREE->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        LE_MOVE_TIME = new QLineEdit(groupBox_2);
        LE_MOVE_TIME->setObjectName(QStringLiteral("LE_MOVE_TIME"));
        LE_MOVE_TIME->setGeometry(QRect(70, 50, 61, 23));
        LE_MOVE_TIME->setFont(font);
        LE_MOVE_TIME->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        label = new QLabel(groupBox_2);
        label->setObjectName(QStringLiteral("label"));
        label->setGeometry(QRect(10, 20, 51, 21));
        label->setFont(font);
        label->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        line = new QFrame(SettingDialog);
        line->setObjectName(QStringLiteral("line"));
        line->setGeometry(QRect(520, 43, 111, 16));
        line->setFrameShape(QFrame::HLine);
        line->setFrameShadow(QFrame::Sunken);
        groupBox = new QGroupBox(SettingDialog);
        groupBox->setObjectName(QStringLiteral("groupBox"));
        groupBox->setGeometry(QRect(520, 230, 231, 241));
        groupBox->setStyleSheet(QLatin1String("QGroupBox {\n"
"    border: 1px solid gray;\n"
"    border-radius: 5px;\n"
"    margin-top: 7px;\n"
"}\n"
"\n"
"QGroupBox::title {\n"
"    subcontrol-origin: margin;\n"
"    left: 8px;\n"
"    padding: 0 3px 0 3px;\n"
"}\n"
"\n"
""));
        BTN_EXECUTE_COMMAND = new QPushButton(groupBox);
        BTN_EXECUTE_COMMAND->setObjectName(QStringLiteral("BTN_EXECUTE_COMMAND"));
        BTN_EXECUTE_COMMAND->setGeometry(QRect(10, 190, 211, 41));
        BTN_EXECUTE_COMMAND->setFont(font);
        RB_MODE_POS = new QRadioButton(groupBox);
        RB_MODE_POS->setObjectName(QStringLiteral("RB_MODE_POS"));
        RB_MODE_POS->setGeometry(QRect(10, 120, 91, 22));
        RB_MODE_POS->setFont(font);
        RB_MODE_PWM = new QRadioButton(groupBox);
        RB_MODE_PWM->setObjectName(QStringLiteral("RB_MODE_PWM"));
        RB_MODE_PWM->setGeometry(QRect(120, 120, 91, 22));
        RB_MODE_PWM->setFont(font);
        RB_INIT_POS = new QRadioButton(groupBox);
        RB_INIT_POS->setObjectName(QStringLiteral("RB_INIT_POS"));
        RB_INIT_POS->setGeometry(QRect(10, 20, 91, 22));
        RB_INIT_POS->setFont(font);
        RB_INIT_POS->setChecked(true);
        RB_ENC_ZERO = new QRadioButton(groupBox);
        RB_ENC_ZERO->setObjectName(QStringLiteral("RB_ENC_ZERO"));
        RB_ENC_ZERO->setGeometry(QRect(120, 20, 91, 22));
        RB_ENC_ZERO->setFont(font);
        RB_FET_OFF = new QRadioButton(groupBox);
        RB_FET_OFF->setObjectName(QStringLiteral("RB_FET_OFF"));
        RB_FET_OFF->setGeometry(QRect(120, 60, 91, 22));
        RB_FET_OFF->setFont(font);
        RB_FET_ON = new QRadioButton(groupBox);
        RB_FET_ON->setObjectName(QStringLiteral("RB_FET_ON"));
        RB_FET_ON->setGeometry(QRect(10, 60, 91, 22));
        RB_FET_ON->setFont(font);
        RB_CTRL_OFF = new QRadioButton(groupBox);
        RB_CTRL_OFF->setObjectName(QStringLiteral("RB_CTRL_OFF"));
        RB_CTRL_OFF->setGeometry(QRect(120, 90, 91, 22));
        RB_CTRL_OFF->setFont(font);
        RB_CTRL_ON = new QRadioButton(groupBox);
        RB_CTRL_ON->setObjectName(QStringLiteral("RB_CTRL_ON"));
        RB_CTRL_ON->setGeometry(QRect(10, 90, 91, 22));
        RB_CTRL_ON->setFont(font);
        line_3 = new QFrame(groupBox);
        line_3->setObjectName(QStringLiteral("line_3"));
        line_3->setGeometry(QRect(10, 50, 111, 16));
        line_3->setFrameShape(QFrame::HLine);
        line_3->setFrameShadow(QFrame::Sunken);
        line_4 = new QFrame(groupBox);
        line_4->setObjectName(QStringLiteral("line_4"));
        line_4->setGeometry(QRect(10, 110, 111, 16));
        line_4->setFrameShape(QFrame::HLine);
        line_4->setFrameShadow(QFrame::Sunken);
        RB_ERROR_CLEAR = new QRadioButton(groupBox);
        RB_ERROR_CLEAR->setObjectName(QStringLiteral("RB_ERROR_CLEAR"));
        RB_ERROR_CLEAR->setGeometry(QRect(10, 150, 91, 22));
        RB_ERROR_CLEAR->setFont(font);
        RB_JOINT_RECOVER = new QRadioButton(groupBox);
        RB_JOINT_RECOVER->setObjectName(QStringLiteral("RB_JOINT_RECOVER"));
        RB_JOINT_RECOVER->setGeometry(QRect(120, 150, 91, 22));
        RB_JOINT_RECOVER->setFont(font);
        TW_4 = new QTableWidget(SettingDialog);
        TW_4->setObjectName(QStringLiteral("TW_4"));
        TW_4->setGeometry(QRect(510, 480, 241, 161));
        TW_5 = new QTableWidget(SettingDialog);
        TW_5->setObjectName(QStringLiteral("TW_5"));
        TW_5->setGeometry(QRect(10, 450, 241, 191));
        TW_6 = new QTableWidget(SettingDialog);
        TW_6->setObjectName(QStringLiteral("TW_6"));
        TW_6->setGeometry(QRect(260, 450, 241, 191));
        BTN_INIT_HAND = new QPushButton(SettingDialog);
        BTN_INIT_HAND->setObjectName(QStringLiteral("BTN_INIT_HAND"));
        BTN_INIT_HAND->setGeometry(QRect(520, 60, 101, 31));
        BTN_INIT_HAND->setFont(font);
        BTN_INIT_HEAD = new QPushButton(SettingDialog);
        BTN_INIT_HEAD->setObjectName(QStringLiteral("BTN_INIT_HEAD"));
        BTN_INIT_HEAD->setGeometry(QRect(650, 60, 101, 31));
        BTN_INIT_HEAD->setFont(font);

        retranslateUi(SettingDialog);

        QMetaObject::connectSlotsByName(SettingDialog);
    } // setupUi

    void retranslateUi(QDialog *SettingDialog)
    {
        SettingDialog->setWindowTitle(QApplication::translate("SettingDialog", "Dialog", Q_NULLPTR));
        LB_SELECTED->setText(QApplication::translate("SettingDialog", "Selected: ", Q_NULLPTR));
        BTN_CAN_CHECK->setText(QApplication::translate("SettingDialog", "CAN Check", Q_NULLPTR));
        BTN_FIND_HOME->setText(QApplication::translate("SettingDialog", "Find Home", Q_NULLPTR));
        groupBox_2->setTitle(QApplication::translate("SettingDialog", "Move", Q_NULLPTR));
        BTN_MOVE_JOINT->setText(QApplication::translate("SettingDialog", "Move\n"
"Joint", Q_NULLPTR));
        label_2->setText(QApplication::translate("SettingDialog", "Time :", Q_NULLPTR));
        LE_MOVE_DEGREE->setText(QApplication::translate("SettingDialog", "10", Q_NULLPTR));
        LE_MOVE_TIME->setText(QApplication::translate("SettingDialog", "2000", Q_NULLPTR));
        label->setText(QApplication::translate("SettingDialog", "Angle :", Q_NULLPTR));
        groupBox->setTitle(QApplication::translate("SettingDialog", "Control", Q_NULLPTR));
        BTN_EXECUTE_COMMAND->setText(QApplication::translate("SettingDialog", "Execute", Q_NULLPTR));
        RB_MODE_POS->setText(QApplication::translate("SettingDialog", "Position", Q_NULLPTR));
        RB_MODE_PWM->setText(QApplication::translate("SettingDialog", "PWM", Q_NULLPTR));
        RB_INIT_POS->setText(QApplication::translate("SettingDialog", "Init Pos", Q_NULLPTR));
        RB_ENC_ZERO->setText(QApplication::translate("SettingDialog", "Enc Zero", Q_NULLPTR));
        RB_FET_OFF->setText(QApplication::translate("SettingDialog", "FET Off", Q_NULLPTR));
        RB_FET_ON->setText(QApplication::translate("SettingDialog", "FET On", Q_NULLPTR));
        RB_CTRL_OFF->setText(QApplication::translate("SettingDialog", "Ctrl Off", Q_NULLPTR));
        RB_CTRL_ON->setText(QApplication::translate("SettingDialog", "Ctrl On", Q_NULLPTR));
        RB_ERROR_CLEAR->setText(QApplication::translate("SettingDialog", "Err Clear", Q_NULLPTR));
        RB_JOINT_RECOVER->setText(QApplication::translate("SettingDialog", "Recover", Q_NULLPTR));
        BTN_INIT_HAND->setText(QApplication::translate("SettingDialog", "Init Hand", Q_NULLPTR));
        BTN_INIT_HEAD->setText(QApplication::translate("SettingDialog", "Init Head", Q_NULLPTR));
    } // retranslateUi

};

namespace Ui {
    class SettingDialog: public Ui_SettingDialog {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_SETTINGDIALOG_H
