/********************************************************************************
** Form generated from reading UI file 'TutorialDialog.ui'
**
** Created by: Qt User Interface Compiler version 5.8.0
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_TUTORIALDIALOG_H
#define UI_TUTORIALDIALOG_H

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

QT_BEGIN_NAMESPACE

class Ui_TutorialDialog
{
public:
    QLineEdit *LE_CURRENT;
    QPushButton *BTN_QP_START;
    QPushButton *BTN_QP_STOP;
    QGroupBox *groupBox;
    QPushButton *BTN_SET_MODI;
    QLineEdit *LE_MODI_R4;
    QLineEdit *LE_MODI_R3;
    QLineEdit *LE_MODI_R0;
    QLineEdit *LE_MODI_L0;
    QPushButton *BTN_READ_MODI;
    QLineEdit *LE_MODI_L3;
    QLineEdit *LE_MODI_L1;
    QLineEdit *LE_MODI_R2;
    QLineEdit *LE_MODI_L4;
    QLabel *label_22;
    QLineEdit *LE_MODI_R1;
    QLineEdit *LE_MODI_L2;
    QLabel *label_21;
    QPushButton *BTN_INIT_ALLHAND;
    QLabel *label_11;
    QPushButton *BTN_R_GRASP0;
    QPushButton *BTN_R_RELEASE3;
    QLabel *label_20;
    QLabel *LB_LF_MODE;
    QPushButton *BTN_R_GRASP1;
    QFrame *line;
    QLabel *label_2;
    QLabel *label_19;
    QPushButton *BTN_L_RELEASE1;
    QPushButton *BTN_L_RELEASE0;
    QPushButton *BTN_R_RELEASE2;
    QLabel *label_17;
    QLabel *label_8;
    QPushButton *BTN_L_GRASP0;
    QPushButton *BTN_R_RELEASE_ALL;
    QPushButton *BTN_R_GRASP2;
    QLabel *label_15;
    QPushButton *BTN_L_RELEASE4;
    QPushButton *BTN_R_GRASP4;
    QPushButton *BTN_L_GRASP2;
    QLabel *label_18;
    QPushButton *BTN_R_RELEASE1;
    QPushButton *BTN_L_RELEASE2;
    QLabel *label_14;
    QPushButton *BTN_R_RELEASE0;
    QLabel *label_13;
    QLabel *LB_RF_MODE;
    QPushButton *BTN_L_RELEASE_ALL;
    QPushButton *BTN_L_GRASP_ALL;
    QLabel *label_7;
    QPushButton *BTN_L_GRASP3;
    QPushButton *BTN_R_GRASP_ALL;
    QPushButton *BTN_R_RELEASE4;
    QLabel *label_10;
    QPushButton *BTN_R_GRASP3;
    QLabel *label_9;
    QPushButton *BTN_L_GRASP1;
    QPushButton *BTN_L_GRASP4;
    QPushButton *BTN_L_RELEASE3;
    QLabel *label_12;
    QPushButton *BTN_TEST;
    QPushButton *BTN_IMU_NULL;
    QPushButton *BTN_INIT_POS;
    QLineEdit *LE_IMU_ROLL;
    QLineEdit *LE_IMU_PITCH;
    QLineEdit *LE_IMU_YAW;
    QLabel *label_16;
    QLabel *label_23;
    QLabel *label_24;
    QPushButton *BTN_TCP_CONNECT;

    void setupUi(QDialog *TutorialDialog)
    {
        if (TutorialDialog->objectName().isEmpty())
            TutorialDialog->setObjectName(QStringLiteral("TutorialDialog"));
        TutorialDialog->resize(703, 652);
        LE_CURRENT = new QLineEdit(TutorialDialog);
        LE_CURRENT->setObjectName(QStringLiteral("LE_CURRENT"));
        LE_CURRENT->setGeometry(QRect(100, 150, 71, 22));
        LE_CURRENT->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        BTN_QP_START = new QPushButton(TutorialDialog);
        BTN_QP_START->setObjectName(QStringLiteral("BTN_QP_START"));
        BTN_QP_START->setGeometry(QRect(230, 140, 91, 41));
        QFont font;
        font.setBold(true);
        font.setWeight(75);
        BTN_QP_START->setFont(font);
        BTN_QP_STOP = new QPushButton(TutorialDialog);
        BTN_QP_STOP->setObjectName(QStringLiteral("BTN_QP_STOP"));
        BTN_QP_STOP->setGeometry(QRect(230, 190, 91, 41));
        BTN_QP_STOP->setFont(font);
        groupBox = new QGroupBox(TutorialDialog);
        groupBox->setObjectName(QStringLiteral("groupBox"));
        groupBox->setGeometry(QRect(539, 389, 121, 71));
        BTN_SET_MODI = new QPushButton(groupBox);
        BTN_SET_MODI->setObjectName(QStringLiteral("BTN_SET_MODI"));
        BTN_SET_MODI->setGeometry(QRect(10, 40, 171, 22));
        LE_MODI_R4 = new QLineEdit(groupBox);
        LE_MODI_R4->setObjectName(QStringLiteral("LE_MODI_R4"));
        LE_MODI_R4->setGeometry(QRect(10, -20, 71, 22));
        LE_MODI_R4->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        LE_MODI_R3 = new QLineEdit(groupBox);
        LE_MODI_R3->setObjectName(QStringLiteral("LE_MODI_R3"));
        LE_MODI_R3->setGeometry(QRect(10, -50, 71, 22));
        LE_MODI_R3->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        LE_MODI_R0 = new QLineEdit(groupBox);
        LE_MODI_R0->setObjectName(QStringLiteral("LE_MODI_R0"));
        LE_MODI_R0->setGeometry(QRect(10, -140, 71, 22));
        LE_MODI_R0->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        LE_MODI_L0 = new QLineEdit(groupBox);
        LE_MODI_L0->setObjectName(QStringLiteral("LE_MODI_L0"));
        LE_MODI_L0->setGeometry(QRect(110, -140, 71, 22));
        LE_MODI_L0->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        BTN_READ_MODI = new QPushButton(groupBox);
        BTN_READ_MODI->setObjectName(QStringLiteral("BTN_READ_MODI"));
        BTN_READ_MODI->setGeometry(QRect(10, 10, 171, 22));
        LE_MODI_L3 = new QLineEdit(groupBox);
        LE_MODI_L3->setObjectName(QStringLiteral("LE_MODI_L3"));
        LE_MODI_L3->setGeometry(QRect(110, -50, 71, 22));
        LE_MODI_L3->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        LE_MODI_L1 = new QLineEdit(groupBox);
        LE_MODI_L1->setObjectName(QStringLiteral("LE_MODI_L1"));
        LE_MODI_L1->setGeometry(QRect(110, -110, 71, 22));
        LE_MODI_L1->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        LE_MODI_R2 = new QLineEdit(groupBox);
        LE_MODI_R2->setObjectName(QStringLiteral("LE_MODI_R2"));
        LE_MODI_R2->setGeometry(QRect(10, -80, 71, 22));
        LE_MODI_R2->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        LE_MODI_L4 = new QLineEdit(groupBox);
        LE_MODI_L4->setObjectName(QStringLiteral("LE_MODI_L4"));
        LE_MODI_L4->setGeometry(QRect(110, -20, 71, 22));
        LE_MODI_L4->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        label_22 = new QLabel(groupBox);
        label_22->setObjectName(QStringLiteral("label_22"));
        label_22->setGeometry(QRect(110, -170, 71, 21));
        QFont font1;
        font1.setPointSize(10);
        font1.setBold(true);
        font1.setWeight(75);
        label_22->setFont(font1);
        label_22->setAlignment(Qt::AlignCenter);
        LE_MODI_R1 = new QLineEdit(groupBox);
        LE_MODI_R1->setObjectName(QStringLiteral("LE_MODI_R1"));
        LE_MODI_R1->setGeometry(QRect(10, -110, 71, 22));
        LE_MODI_R1->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        LE_MODI_L2 = new QLineEdit(groupBox);
        LE_MODI_L2->setObjectName(QStringLiteral("LE_MODI_L2"));
        LE_MODI_L2->setGeometry(QRect(110, -80, 71, 22));
        LE_MODI_L2->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        label_21 = new QLabel(groupBox);
        label_21->setObjectName(QStringLiteral("label_21"));
        label_21->setGeometry(QRect(10, -170, 71, 21));
        label_21->setFont(font1);
        label_21->setAlignment(Qt::AlignCenter);
        BTN_INIT_ALLHAND = new QPushButton(groupBox);
        BTN_INIT_ALLHAND->setObjectName(QStringLiteral("BTN_INIT_ALLHAND"));
        BTN_INIT_ALLHAND->setGeometry(QRect(280, 210, 91, 41));
        BTN_INIT_ALLHAND->setFont(font);
        label_11 = new QLabel(groupBox);
        label_11->setObjectName(QStringLiteral("label_11"));
        label_11->setGeometry(QRect(-230, 150, 51, 21));
        BTN_R_GRASP0 = new QPushButton(groupBox);
        BTN_R_GRASP0->setObjectName(QStringLiteral("BTN_R_GRASP0"));
        BTN_R_GRASP0->setGeometry(QRect(-170, 90, 71, 22));
        BTN_R_RELEASE3 = new QPushButton(groupBox);
        BTN_R_RELEASE3->setObjectName(QStringLiteral("BTN_R_RELEASE3"));
        BTN_R_RELEASE3->setGeometry(QRect(-90, 180, 71, 22));
        label_20 = new QLabel(groupBox);
        label_20->setObjectName(QStringLiteral("label_20"));
        label_20->setGeometry(QRect(40, 150, 51, 21));
        LB_LF_MODE = new QLabel(groupBox);
        LB_LF_MODE->setObjectName(QStringLiteral("LB_LF_MODE"));
        LB_LF_MODE->setGeometry(QRect(40, 40, 191, 31));
        LB_LF_MODE->setFont(font1);
        BTN_R_GRASP1 = new QPushButton(groupBox);
        BTN_R_GRASP1->setObjectName(QStringLiteral("BTN_R_GRASP1"));
        BTN_R_GRASP1->setGeometry(QRect(-170, 120, 71, 22));
        line = new QFrame(groupBox);
        line->setObjectName(QStringLiteral("line"));
        line->setGeometry(QRect(-220, 30, 118, 3));
        QFont font2;
        font2.setPointSize(9);
        line->setFont(font2);
        line->setLineWidth(2);
        line->setFrameShape(QFrame::HLine);
        line->setFrameShadow(QFrame::Sunken);
        label_2 = new QLabel(groupBox);
        label_2->setObjectName(QStringLiteral("label_2"));
        label_2->setGeometry(QRect(-220, 0, 171, 31));
        QFont font3;
        font3.setPointSize(12);
        font3.setBold(true);
        font3.setWeight(75);
        label_2->setFont(font3);
        label_19 = new QLabel(groupBox);
        label_19->setObjectName(QStringLiteral("label_19"));
        label_19->setGeometry(QRect(40, 120, 51, 21));
        BTN_L_RELEASE1 = new QPushButton(groupBox);
        BTN_L_RELEASE1->setObjectName(QStringLiteral("BTN_L_RELEASE1"));
        BTN_L_RELEASE1->setGeometry(QRect(180, 120, 71, 22));
        BTN_L_RELEASE0 = new QPushButton(groupBox);
        BTN_L_RELEASE0->setObjectName(QStringLiteral("BTN_L_RELEASE0"));
        BTN_L_RELEASE0->setGeometry(QRect(180, 90, 71, 22));
        BTN_R_RELEASE2 = new QPushButton(groupBox);
        BTN_R_RELEASE2->setObjectName(QStringLiteral("BTN_R_RELEASE2"));
        BTN_R_RELEASE2->setGeometry(QRect(-90, 150, 71, 22));
        label_17 = new QLabel(groupBox);
        label_17->setObjectName(QStringLiteral("label_17"));
        label_17->setGeometry(QRect(40, 210, 51, 21));
        label_8 = new QLabel(groupBox);
        label_8->setObjectName(QStringLiteral("label_8"));
        label_8->setGeometry(QRect(200, 10, 191, 21));
        BTN_L_GRASP0 = new QPushButton(groupBox);
        BTN_L_GRASP0->setObjectName(QStringLiteral("BTN_L_GRASP0"));
        BTN_L_GRASP0->setGeometry(QRect(100, 90, 71, 22));
        BTN_R_RELEASE_ALL = new QPushButton(groupBox);
        BTN_R_RELEASE_ALL->setObjectName(QStringLiteral("BTN_R_RELEASE_ALL"));
        BTN_R_RELEASE_ALL->setGeometry(QRect(-90, 260, 71, 22));
        BTN_R_GRASP2 = new QPushButton(groupBox);
        BTN_R_GRASP2->setObjectName(QStringLiteral("BTN_R_GRASP2"));
        BTN_R_GRASP2->setGeometry(QRect(-170, 150, 71, 22));
        label_15 = new QLabel(groupBox);
        label_15->setObjectName(QStringLiteral("label_15"));
        label_15->setGeometry(QRect(60, 260, 31, 21));
        BTN_L_RELEASE4 = new QPushButton(groupBox);
        BTN_L_RELEASE4->setObjectName(QStringLiteral("BTN_L_RELEASE4"));
        BTN_L_RELEASE4->setGeometry(QRect(180, 210, 71, 22));
        BTN_R_GRASP4 = new QPushButton(groupBox);
        BTN_R_GRASP4->setObjectName(QStringLiteral("BTN_R_GRASP4"));
        BTN_R_GRASP4->setGeometry(QRect(-170, 210, 71, 22));
        BTN_L_GRASP2 = new QPushButton(groupBox);
        BTN_L_GRASP2->setObjectName(QStringLiteral("BTN_L_GRASP2"));
        BTN_L_GRASP2->setGeometry(QRect(100, 150, 71, 22));
        label_18 = new QLabel(groupBox);
        label_18->setObjectName(QStringLiteral("label_18"));
        label_18->setGeometry(QRect(40, 180, 51, 21));
        BTN_R_RELEASE1 = new QPushButton(groupBox);
        BTN_R_RELEASE1->setObjectName(QStringLiteral("BTN_R_RELEASE1"));
        BTN_R_RELEASE1->setGeometry(QRect(-90, 120, 71, 22));
        BTN_L_RELEASE2 = new QPushButton(groupBox);
        BTN_L_RELEASE2->setObjectName(QStringLiteral("BTN_L_RELEASE2"));
        BTN_L_RELEASE2->setGeometry(QRect(180, 150, 71, 22));
        label_14 = new QLabel(groupBox);
        label_14->setObjectName(QStringLiteral("label_14"));
        label_14->setGeometry(QRect(-210, 260, 31, 21));
        BTN_R_RELEASE0 = new QPushButton(groupBox);
        BTN_R_RELEASE0->setObjectName(QStringLiteral("BTN_R_RELEASE0"));
        BTN_R_RELEASE0->setGeometry(QRect(-90, 90, 71, 22));
        label_13 = new QLabel(groupBox);
        label_13->setObjectName(QStringLiteral("label_13"));
        label_13->setGeometry(QRect(-230, 210, 51, 21));
        LB_RF_MODE = new QLabel(groupBox);
        LB_RF_MODE->setObjectName(QStringLiteral("LB_RF_MODE"));
        LB_RF_MODE->setGeometry(QRect(-230, 40, 201, 31));
        LB_RF_MODE->setFont(font1);
        BTN_L_RELEASE_ALL = new QPushButton(groupBox);
        BTN_L_RELEASE_ALL->setObjectName(QStringLiteral("BTN_L_RELEASE_ALL"));
        BTN_L_RELEASE_ALL->setGeometry(QRect(180, 260, 71, 22));
        BTN_L_GRASP_ALL = new QPushButton(groupBox);
        BTN_L_GRASP_ALL->setObjectName(QStringLiteral("BTN_L_GRASP_ALL"));
        BTN_L_GRASP_ALL->setGeometry(QRect(100, 260, 71, 22));
        label_7 = new QLabel(groupBox);
        label_7->setObjectName(QStringLiteral("label_7"));
        label_7->setGeometry(QRect(-50, 10, 161, 21));
        BTN_L_GRASP3 = new QPushButton(groupBox);
        BTN_L_GRASP3->setObjectName(QStringLiteral("BTN_L_GRASP3"));
        BTN_L_GRASP3->setGeometry(QRect(100, 180, 71, 22));
        BTN_R_GRASP_ALL = new QPushButton(groupBox);
        BTN_R_GRASP_ALL->setObjectName(QStringLiteral("BTN_R_GRASP_ALL"));
        BTN_R_GRASP_ALL->setGeometry(QRect(-170, 260, 71, 22));
        BTN_R_RELEASE4 = new QPushButton(groupBox);
        BTN_R_RELEASE4->setObjectName(QStringLiteral("BTN_R_RELEASE4"));
        BTN_R_RELEASE4->setGeometry(QRect(-90, 210, 71, 22));
        label_10 = new QLabel(groupBox);
        label_10->setObjectName(QStringLiteral("label_10"));
        label_10->setGeometry(QRect(-230, 120, 51, 21));
        BTN_R_GRASP3 = new QPushButton(groupBox);
        BTN_R_GRASP3->setObjectName(QStringLiteral("BTN_R_GRASP3"));
        BTN_R_GRASP3->setGeometry(QRect(-170, 180, 71, 22));
        label_9 = new QLabel(groupBox);
        label_9->setObjectName(QStringLiteral("label_9"));
        label_9->setGeometry(QRect(-230, 80, 51, 31));
        BTN_L_GRASP1 = new QPushButton(groupBox);
        BTN_L_GRASP1->setObjectName(QStringLiteral("BTN_L_GRASP1"));
        BTN_L_GRASP1->setGeometry(QRect(100, 120, 71, 22));
        BTN_L_GRASP4 = new QPushButton(groupBox);
        BTN_L_GRASP4->setObjectName(QStringLiteral("BTN_L_GRASP4"));
        BTN_L_GRASP4->setGeometry(QRect(100, 210, 71, 22));
        BTN_L_RELEASE3 = new QPushButton(groupBox);
        BTN_L_RELEASE3->setObjectName(QStringLiteral("BTN_L_RELEASE3"));
        BTN_L_RELEASE3->setGeometry(QRect(180, 180, 71, 22));
        label_12 = new QLabel(groupBox);
        label_12->setObjectName(QStringLiteral("label_12"));
        label_12->setGeometry(QRect(-230, 180, 51, 21));
        BTN_TEST = new QPushButton(TutorialDialog);
        BTN_TEST->setObjectName(QStringLiteral("BTN_TEST"));
        BTN_TEST->setGeometry(QRect(210, 370, 91, 41));
        BTN_TEST->setFont(font);
        BTN_IMU_NULL = new QPushButton(TutorialDialog);
        BTN_IMU_NULL->setObjectName(QStringLiteral("BTN_IMU_NULL"));
        BTN_IMU_NULL->setGeometry(QRect(30, 440, 181, 41));
        BTN_INIT_POS = new QPushButton(TutorialDialog);
        BTN_INIT_POS->setObjectName(QStringLiteral("BTN_INIT_POS"));
        BTN_INIT_POS->setGeometry(QRect(230, 80, 91, 41));
        BTN_INIT_POS->setFont(font);
        LE_IMU_ROLL = new QLineEdit(TutorialDialog);
        LE_IMU_ROLL->setObjectName(QStringLiteral("LE_IMU_ROLL"));
        LE_IMU_ROLL->setGeometry(QRect(100, 500, 71, 22));
        LE_IMU_ROLL->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        LE_IMU_PITCH = new QLineEdit(TutorialDialog);
        LE_IMU_PITCH->setObjectName(QStringLiteral("LE_IMU_PITCH"));
        LE_IMU_PITCH->setGeometry(QRect(100, 530, 71, 22));
        LE_IMU_PITCH->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        LE_IMU_YAW = new QLineEdit(TutorialDialog);
        LE_IMU_YAW->setObjectName(QStringLiteral("LE_IMU_YAW"));
        LE_IMU_YAW->setGeometry(QRect(100, 560, 71, 22));
        LE_IMU_YAW->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        label_16 = new QLabel(TutorialDialog);
        label_16->setObjectName(QStringLiteral("label_16"));
        label_16->setGeometry(QRect(60, 500, 51, 31));
        label_23 = new QLabel(TutorialDialog);
        label_23->setObjectName(QStringLiteral("label_23"));
        label_23->setGeometry(QRect(60, 560, 51, 31));
        label_24 = new QLabel(TutorialDialog);
        label_24->setObjectName(QStringLiteral("label_24"));
        label_24->setGeometry(QRect(60, 530, 51, 31));
        BTN_TCP_CONNECT = new QPushButton(TutorialDialog);
        BTN_TCP_CONNECT->setObjectName(QStringLiteral("BTN_TCP_CONNECT"));
        BTN_TCP_CONNECT->setGeometry(QRect(410, 130, 91, 41));
        BTN_TCP_CONNECT->setFont(font);

        retranslateUi(TutorialDialog);

        QMetaObject::connectSlotsByName(TutorialDialog);
    } // setupUi

    void retranslateUi(QDialog *TutorialDialog)
    {
        TutorialDialog->setWindowTitle(QApplication::translate("TutorialDialog", "Dialog", Q_NULLPTR));
        LE_CURRENT->setText(QApplication::translate("TutorialDialog", "30", Q_NULLPTR));
        BTN_QP_START->setText(QApplication::translate("TutorialDialog", "QP\n"
"START", Q_NULLPTR));
        BTN_QP_STOP->setText(QApplication::translate("TutorialDialog", "QP\n"
"STOP", Q_NULLPTR));
        groupBox->setTitle(QApplication::translate("TutorialDialog", "trash", Q_NULLPTR));
        BTN_SET_MODI->setText(QApplication::translate("TutorialDialog", "Set", Q_NULLPTR));
        LE_MODI_R4->setText(QApplication::translate("TutorialDialog", "30", Q_NULLPTR));
        LE_MODI_R3->setText(QApplication::translate("TutorialDialog", "30", Q_NULLPTR));
        LE_MODI_R0->setText(QApplication::translate("TutorialDialog", "30", Q_NULLPTR));
        LE_MODI_L0->setText(QApplication::translate("TutorialDialog", "30", Q_NULLPTR));
        BTN_READ_MODI->setText(QApplication::translate("TutorialDialog", "Read", Q_NULLPTR));
        LE_MODI_L3->setText(QApplication::translate("TutorialDialog", "30", Q_NULLPTR));
        LE_MODI_L1->setText(QApplication::translate("TutorialDialog", "30", Q_NULLPTR));
        LE_MODI_R2->setText(QApplication::translate("TutorialDialog", "30", Q_NULLPTR));
        LE_MODI_L4->setText(QApplication::translate("TutorialDialog", "30", Q_NULLPTR));
        label_22->setText(QApplication::translate("TutorialDialog", "Left", Q_NULLPTR));
        LE_MODI_R1->setText(QApplication::translate("TutorialDialog", "30", Q_NULLPTR));
        LE_MODI_L2->setText(QApplication::translate("TutorialDialog", "30", Q_NULLPTR));
        label_21->setText(QApplication::translate("TutorialDialog", "Right", Q_NULLPTR));
        BTN_INIT_ALLHAND->setText(QApplication::translate("TutorialDialog", "All\n"
"Init", Q_NULLPTR));
        label_11->setText(QApplication::translate("TutorialDialog", "Finger2", Q_NULLPTR));
        BTN_R_GRASP0->setText(QApplication::translate("TutorialDialog", "Grasp", Q_NULLPTR));
        BTN_R_RELEASE3->setText(QApplication::translate("TutorialDialog", "Release", Q_NULLPTR));
        label_20->setText(QApplication::translate("TutorialDialog", "Finger2", Q_NULLPTR));
        LB_LF_MODE->setText(QApplication::translate("TutorialDialog", "Left : ", Q_NULLPTR));
        BTN_R_GRASP1->setText(QApplication::translate("TutorialDialog", "Grasp", Q_NULLPTR));
        label_2->setText(QApplication::translate("TutorialDialog", "Finger Control", Q_NULLPTR));
        label_19->setText(QApplication::translate("TutorialDialog", "Finger1", Q_NULLPTR));
        BTN_L_RELEASE1->setText(QApplication::translate("TutorialDialog", "Release", Q_NULLPTR));
        BTN_L_RELEASE0->setText(QApplication::translate("TutorialDialog", "Release", Q_NULLPTR));
        BTN_R_RELEASE2->setText(QApplication::translate("TutorialDialog", "Release", Q_NULLPTR));
        label_17->setText(QApplication::translate("TutorialDialog", "Finger4", Q_NULLPTR));
        label_8->setText(QApplication::translate("TutorialDialog", "each digit means 0.083mA", Q_NULLPTR));
        BTN_L_GRASP0->setText(QApplication::translate("TutorialDialog", "Grasp", Q_NULLPTR));
        BTN_R_RELEASE_ALL->setText(QApplication::translate("TutorialDialog", "Release", Q_NULLPTR));
        BTN_R_GRASP2->setText(QApplication::translate("TutorialDialog", "Grasp", Q_NULLPTR));
        label_15->setText(QApplication::translate("TutorialDialog", "All ", Q_NULLPTR));
        BTN_L_RELEASE4->setText(QApplication::translate("TutorialDialog", "Release", Q_NULLPTR));
        BTN_R_GRASP4->setText(QApplication::translate("TutorialDialog", "Grasp", Q_NULLPTR));
        BTN_L_GRASP2->setText(QApplication::translate("TutorialDialog", "Grasp", Q_NULLPTR));
        label_18->setText(QApplication::translate("TutorialDialog", "Finger3", Q_NULLPTR));
        BTN_R_RELEASE1->setText(QApplication::translate("TutorialDialog", "Release", Q_NULLPTR));
        BTN_L_RELEASE2->setText(QApplication::translate("TutorialDialog", "Release", Q_NULLPTR));
        label_14->setText(QApplication::translate("TutorialDialog", "All ", Q_NULLPTR));
        BTN_R_RELEASE0->setText(QApplication::translate("TutorialDialog", "Release", Q_NULLPTR));
        label_13->setText(QApplication::translate("TutorialDialog", "Finger4", Q_NULLPTR));
        LB_RF_MODE->setText(QApplication::translate("TutorialDialog", "Right : ", Q_NULLPTR));
        BTN_L_RELEASE_ALL->setText(QApplication::translate("TutorialDialog", "Release", Q_NULLPTR));
        BTN_L_GRASP_ALL->setText(QApplication::translate("TutorialDialog", "Grasp", Q_NULLPTR));
        label_7->setText(QApplication::translate("TutorialDialog", "Desired Current (0~100)", Q_NULLPTR));
        BTN_L_GRASP3->setText(QApplication::translate("TutorialDialog", "Grasp", Q_NULLPTR));
        BTN_R_GRASP_ALL->setText(QApplication::translate("TutorialDialog", "Grasp", Q_NULLPTR));
        BTN_R_RELEASE4->setText(QApplication::translate("TutorialDialog", "Release", Q_NULLPTR));
        label_10->setText(QApplication::translate("TutorialDialog", "Finger1", Q_NULLPTR));
        BTN_R_GRASP3->setText(QApplication::translate("TutorialDialog", "Grasp", Q_NULLPTR));
        label_9->setText(QApplication::translate("TutorialDialog", "Thumb\n"
"Gripper", Q_NULLPTR));
        BTN_L_GRASP1->setText(QApplication::translate("TutorialDialog", "Grasp", Q_NULLPTR));
        BTN_L_GRASP4->setText(QApplication::translate("TutorialDialog", "Grasp", Q_NULLPTR));
        BTN_L_RELEASE3->setText(QApplication::translate("TutorialDialog", "Release", Q_NULLPTR));
        label_12->setText(QApplication::translate("TutorialDialog", "Finger3", Q_NULLPTR));
        BTN_TEST->setText(QApplication::translate("TutorialDialog", "TEST", Q_NULLPTR));
        BTN_IMU_NULL->setText(QApplication::translate("TutorialDialog", "Pelvis IMU Nulling / Enable", Q_NULLPTR));
        BTN_INIT_POS->setText(QApplication::translate("TutorialDialog", "INIT\n"
"POS", Q_NULLPTR));
        LE_IMU_ROLL->setText(QApplication::translate("TutorialDialog", "30", Q_NULLPTR));
        LE_IMU_PITCH->setText(QApplication::translate("TutorialDialog", "30", Q_NULLPTR));
        LE_IMU_YAW->setText(QApplication::translate("TutorialDialog", "30", Q_NULLPTR));
        label_16->setText(QApplication::translate("TutorialDialog", "Roll", Q_NULLPTR));
        label_23->setText(QApplication::translate("TutorialDialog", "Yaw", Q_NULLPTR));
        label_24->setText(QApplication::translate("TutorialDialog", "Pitch", Q_NULLPTR));
        BTN_TCP_CONNECT->setText(QApplication::translate("TutorialDialog", "TCP\n"
"CONNECT", Q_NULLPTR));
    } // retranslateUi

};

namespace Ui {
    class TutorialDialog: public Ui_TutorialDialog {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_TUTORIALDIALOG_H
