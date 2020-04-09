/********************************************************************************
** Form generated from reading UI file 'ModelDialog.ui'
**
** Created by: Qt User Interface Compiler version 5.8.0
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MODELDIALOG_H
#define UI_MODELDIALOG_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QDialog>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QRadioButton>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_ModelDialog
{
public:
    QGroupBox *groupBox_2;
    QCheckBox *CB_USE_ENCODER;
    QCheckBox *CB_SHOW_FLOOR;
    QCheckBox *CB_SHOW_FT;
    QWidget *gridLayoutWidget_3;
    QGridLayout *LAYOUT_MODEL;
    QGroupBox *groupBox;
    QPushButton *BT_CamLeft;
    QPushButton *BT_CamFront;
    QPushButton *BT_CamRight;
    QWidget *gridLayoutWidget;
    QGridLayout *LAYOUT_SLIDER;
    QGroupBox *groupBox_3;
    QPushButton *BTN_SOUND_1;
    QPushButton *BTN_SOUND_2;
    QPushButton *BTN_SOUND_3;
    QPushButton *BTN_SOUND_4;
    QPushButton *BTN_SOUND_5;
    QPushButton *BTN_SOUND_6;
    QPushButton *BTN_SOUND_15;
    QPushButton *BTN_SOUND_16;
    QPushButton *BTN_SOUND_17;
    QPushButton *BTN_SOUND_18;
    QPushButton *BTN_SOUND_19;
    QPushButton *BTN_SOUND_20;
    QPushButton *BTN_SOUND_21;
    QPushButton *BTN_SOUND_22;
    QPushButton *BTN_SOUND_23;
    QPushButton *BTN_SOUND_24;
    QPushButton *BTN_SOUND_25;
    QPushButton *BTN_SOUND_26;
    QPushButton *BTN_SOUND_27;
    QPushButton *BTN_SOUND_28;
    QPushButton *BTN_SOUND_29;
    QPushButton *BTN_SOUND_30;
    QPushButton *BTN_SOUND_31;
    QPushButton *BTN_SOUND_32;
    QPushButton *BTN_SOUND_33;
    QPushButton *BTN_SOUND_34;
    QPushButton *BTN_SOUND_35;
    QPushButton *BTN_SOUND_36;
    QPushButton *BTN_SOUND_37;
    QPushButton *BTN_SOUND_38;
    QPushButton *BTN_SOUND_39;
    QPushButton *BTN_SOUND_40;
    QPushButton *BTN_SOUND_41;
    QPushButton *BTN_SOUND_42;
    QPushButton *BTN_SOUND_43;
    QPushButton *BTN_SOUND_44;
    QPushButton *BTN_SOUND_45;
    QPushButton *BTN_SOUND_46;
    QPushButton *BTN_SOUND_47;
    QPushButton *BTN_SOUND_88;
    QPushButton *BTN_SOUND_89;
    QPushButton *BTN_SOUND_90;
    QPushButton *BTN_SOUND_92;
    QPushButton *BTN_SOUND_93;
    QPushButton *BTN_SOUND_95;
    QPushButton *BTN_SOUND_96;
    QPushButton *BTN_SOUND_97;
    QPushButton *BTN_SOUND_99;
    QPushButton *BTN_SOUND_101;
    QPushButton *BTN_SOUND_102;
    QLabel *label;
    QLabel *label_2;
    QLabel *label_3;
    QLabel *label_4;
    QLabel *label_5;
    QRadioButton *rad_kor;
    QRadioButton *rad_eng;
    QPushButton *BTN_SOUND_104;
    QPushButton *BTN_SOUND_105;
    QPushButton *BTN_SOUND_91;
    QPushButton *BTN_SOUND_94;
    QPushButton *BTN_SOUND_98;
    QPushButton *BTN_SOUND_100;
    QPushButton *BTN_SOUND_103;
    QPushButton *BTN_SOUND_7;
    QLabel *label_6;
    QPushButton *BTN_ENABLE;
    QPushButton *BTN_DISABLE;
    QPushButton *BTN_wa;
    QPushButton *BTN_oh;
    QPushButton *BTN_QUICK_STOP;
    QGroupBox *groupBox_4;
    QWidget *gridLayoutWidget_12;
    QGridLayout *gridLayout_12;
    QLineEdit *LE_SENSOR_LAFT_MY;
    QLineEdit *LE_SENSOR_LWFT_MY;
    QLabel *label_43;
    QLabel *label_32;
    QLineEdit *LE_SENSOR_LWFT_MZ;
    QLabel *label_44;
    QLabel *label_45;
    QLabel *label_39;
    QLineEdit *LE_SENSOR_RAFT_FZ;
    QLabel *label_47;
    QLineEdit *LE_SENSOR_LAFT_FZ;
    QLineEdit *LE_SENSOR_LAFT_MX;
    QLabel *label_48;
    QLineEdit *LE_SENSOR_RWFT_FZ;
    QLineEdit *LE_SENSOR_LWFT_MX;
    QLineEdit *LE_SENSOR_RWFT_MY;
    QLineEdit *LE_SENSOR_RAFT_MX;
    QLabel *label_42;
    QLabel *label_31;
    QLineEdit *LE_SENSOR_RWFT_MX;
    QLineEdit *LE_SENSOR_RAFT_MY;
    QLineEdit *LE_SENSOR_RWFT_MZ;
    QLineEdit *LE_SENSOR_RAFT_MZ;
    QLineEdit *LE_SENSOR_LAFT_MZ;
    QLabel *label_61;
    QLineEdit *LE_SENSOR_RWFT_FY;
    QLineEdit *LE_SENSOR_RWFT_FX;
    QLineEdit *LE_SENSOR_LWFT_FY;
    QLineEdit *LE_SENSOR_LWFT_FX;
    QLineEdit *LE_SENSOR_RAFT_FY;
    QLineEdit *LE_SENSOR_RAFT_FX;
    QLineEdit *LE_SENSOR_LAFT_FY;
    QLineEdit *LE_SENSOR_LAFT_FX;
    QLineEdit *LE_SENSOR_LWFT_FZ;
    QLabel *label_62;
    QPushButton *BTN_SENSOR_FT_NULL;

    void setupUi(QDialog *ModelDialog)
    {
        if (ModelDialog->objectName().isEmpty())
            ModelDialog->setObjectName(QStringLiteral("ModelDialog"));
        ModelDialog->resize(476, 988);
        groupBox_2 = new QGroupBox(ModelDialog);
        groupBox_2->setObjectName(QStringLiteral("groupBox_2"));
        groupBox_2->setGeometry(QRect(130, 420, 181, 81));
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
        CB_USE_ENCODER = new QCheckBox(groupBox_2);
        CB_USE_ENCODER->setObjectName(QStringLiteral("CB_USE_ENCODER"));
        CB_USE_ENCODER->setGeometry(QRect(10, 50, 71, 21));
        CB_USE_ENCODER->setChecked(false);
        CB_SHOW_FLOOR = new QCheckBox(groupBox_2);
        CB_SHOW_FLOOR->setObjectName(QStringLiteral("CB_SHOW_FLOOR"));
        CB_SHOW_FLOOR->setGeometry(QRect(10, 20, 71, 21));
        CB_SHOW_FLOOR->setChecked(false);
        CB_SHOW_FT = new QCheckBox(groupBox_2);
        CB_SHOW_FT->setObjectName(QStringLiteral("CB_SHOW_FT"));
        CB_SHOW_FT->setGeometry(QRect(90, 20, 81, 21));
        CB_SHOW_FT->setChecked(false);
        gridLayoutWidget_3 = new QWidget(ModelDialog);
        gridLayoutWidget_3->setObjectName(QStringLiteral("gridLayoutWidget_3"));
        gridLayoutWidget_3->setGeometry(QRect(10, 10, 441, 401));
        LAYOUT_MODEL = new QGridLayout(gridLayoutWidget_3);
        LAYOUT_MODEL->setObjectName(QStringLiteral("LAYOUT_MODEL"));
        LAYOUT_MODEL->setContentsMargins(0, 0, 0, 0);
        groupBox = new QGroupBox(ModelDialog);
        groupBox->setObjectName(QStringLiteral("groupBox"));
        groupBox->setGeometry(QRect(10, 420, 111, 81));
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
        BT_CamLeft = new QPushButton(groupBox);
        BT_CamLeft->setObjectName(QStringLiteral("BT_CamLeft"));
        BT_CamLeft->setGeometry(QRect(10, 20, 41, 23));
        QSizePolicy sizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(BT_CamLeft->sizePolicy().hasHeightForWidth());
        BT_CamLeft->setSizePolicy(sizePolicy);
        BT_CamLeft->setMaximumSize(QSize(500, 16777215));
        BT_CamFront = new QPushButton(groupBox);
        BT_CamFront->setObjectName(QStringLiteral("BT_CamFront"));
        BT_CamFront->setGeometry(QRect(60, 20, 41, 51));
        sizePolicy.setHeightForWidth(BT_CamFront->sizePolicy().hasHeightForWidth());
        BT_CamFront->setSizePolicy(sizePolicy);
        BT_CamFront->setMaximumSize(QSize(500, 16777215));
        BT_CamRight = new QPushButton(groupBox);
        BT_CamRight->setObjectName(QStringLiteral("BT_CamRight"));
        BT_CamRight->setGeometry(QRect(10, 50, 41, 23));
        sizePolicy.setHeightForWidth(BT_CamRight->sizePolicy().hasHeightForWidth());
        BT_CamRight->setSizePolicy(sizePolicy);
        BT_CamRight->setMaximumSize(QSize(500, 16777215));
        gridLayoutWidget = new QWidget(ModelDialog);
        gridLayoutWidget->setObjectName(QStringLiteral("gridLayoutWidget"));
        gridLayoutWidget->setGeometry(QRect(320, 430, 131, 71));
        LAYOUT_SLIDER = new QGridLayout(gridLayoutWidget);
        LAYOUT_SLIDER->setObjectName(QStringLiteral("LAYOUT_SLIDER"));
        LAYOUT_SLIDER->setContentsMargins(0, 0, 0, 0);
        groupBox_3 = new QGroupBox(ModelDialog);
        groupBox_3->setObjectName(QStringLiteral("groupBox_3"));
        groupBox_3->setGeometry(QRect(440, 960, 20, 20));
        groupBox_3->setStyleSheet(QLatin1String("QGroupBox {\n"
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
        BTN_SOUND_1 = new QPushButton(groupBox_3);
        BTN_SOUND_1->setObjectName(QStringLiteral("BTN_SOUND_1"));
        BTN_SOUND_1->setGeometry(QRect(20, 50, 71, 21));
        sizePolicy.setHeightForWidth(BTN_SOUND_1->sizePolicy().hasHeightForWidth());
        BTN_SOUND_1->setSizePolicy(sizePolicy);
        BTN_SOUND_1->setMaximumSize(QSize(500, 16777215));
        QFont font;
        font.setPointSize(8);
        BTN_SOUND_1->setFont(font);
        BTN_SOUND_2 = new QPushButton(groupBox_3);
        BTN_SOUND_2->setObjectName(QStringLiteral("BTN_SOUND_2"));
        BTN_SOUND_2->setGeometry(QRect(100, 50, 71, 21));
        sizePolicy.setHeightForWidth(BTN_SOUND_2->sizePolicy().hasHeightForWidth());
        BTN_SOUND_2->setSizePolicy(sizePolicy);
        BTN_SOUND_2->setMaximumSize(QSize(500, 16777215));
        BTN_SOUND_2->setFont(font);
        BTN_SOUND_3 = new QPushButton(groupBox_3);
        BTN_SOUND_3->setObjectName(QStringLiteral("BTN_SOUND_3"));
        BTN_SOUND_3->setGeometry(QRect(180, 50, 71, 21));
        sizePolicy.setHeightForWidth(BTN_SOUND_3->sizePolicy().hasHeightForWidth());
        BTN_SOUND_3->setSizePolicy(sizePolicy);
        BTN_SOUND_3->setMaximumSize(QSize(500, 16777215));
        BTN_SOUND_3->setFont(font);
        BTN_SOUND_4 = new QPushButton(groupBox_3);
        BTN_SOUND_4->setObjectName(QStringLiteral("BTN_SOUND_4"));
        BTN_SOUND_4->setGeometry(QRect(260, 50, 71, 21));
        sizePolicy.setHeightForWidth(BTN_SOUND_4->sizePolicy().hasHeightForWidth());
        BTN_SOUND_4->setSizePolicy(sizePolicy);
        BTN_SOUND_4->setMaximumSize(QSize(500, 16777215));
        BTN_SOUND_4->setFont(font);
        BTN_SOUND_5 = new QPushButton(groupBox_3);
        BTN_SOUND_5->setObjectName(QStringLiteral("BTN_SOUND_5"));
        BTN_SOUND_5->setGeometry(QRect(340, 50, 71, 21));
        sizePolicy.setHeightForWidth(BTN_SOUND_5->sizePolicy().hasHeightForWidth());
        BTN_SOUND_5->setSizePolicy(sizePolicy);
        BTN_SOUND_5->setMaximumSize(QSize(500, 16777215));
        BTN_SOUND_5->setFont(font);
        BTN_SOUND_6 = new QPushButton(groupBox_3);
        BTN_SOUND_6->setObjectName(QStringLiteral("BTN_SOUND_6"));
        BTN_SOUND_6->setGeometry(QRect(20, 80, 71, 21));
        sizePolicy.setHeightForWidth(BTN_SOUND_6->sizePolicy().hasHeightForWidth());
        BTN_SOUND_6->setSizePolicy(sizePolicy);
        BTN_SOUND_6->setMaximumSize(QSize(500, 16777215));
        BTN_SOUND_6->setFont(font);
        BTN_SOUND_15 = new QPushButton(groupBox_3);
        BTN_SOUND_15->setObjectName(QStringLiteral("BTN_SOUND_15"));
        BTN_SOUND_15->setGeometry(QRect(180, 80, 71, 21));
        sizePolicy.setHeightForWidth(BTN_SOUND_15->sizePolicy().hasHeightForWidth());
        BTN_SOUND_15->setSizePolicy(sizePolicy);
        BTN_SOUND_15->setMaximumSize(QSize(500, 16777215));
        BTN_SOUND_15->setFont(font);
        BTN_SOUND_16 = new QPushButton(groupBox_3);
        BTN_SOUND_16->setObjectName(QStringLiteral("BTN_SOUND_16"));
        BTN_SOUND_16->setGeometry(QRect(260, 80, 71, 21));
        sizePolicy.setHeightForWidth(BTN_SOUND_16->sizePolicy().hasHeightForWidth());
        BTN_SOUND_16->setSizePolicy(sizePolicy);
        BTN_SOUND_16->setMaximumSize(QSize(500, 16777215));
        BTN_SOUND_16->setFont(font);
        BTN_SOUND_17 = new QPushButton(groupBox_3);
        BTN_SOUND_17->setObjectName(QStringLiteral("BTN_SOUND_17"));
        BTN_SOUND_17->setGeometry(QRect(340, 80, 71, 21));
        sizePolicy.setHeightForWidth(BTN_SOUND_17->sizePolicy().hasHeightForWidth());
        BTN_SOUND_17->setSizePolicy(sizePolicy);
        BTN_SOUND_17->setMaximumSize(QSize(500, 16777215));
        BTN_SOUND_17->setFont(font);
        BTN_SOUND_18 = new QPushButton(groupBox_3);
        BTN_SOUND_18->setObjectName(QStringLiteral("BTN_SOUND_18"));
        BTN_SOUND_18->setGeometry(QRect(20, 110, 71, 21));
        sizePolicy.setHeightForWidth(BTN_SOUND_18->sizePolicy().hasHeightForWidth());
        BTN_SOUND_18->setSizePolicy(sizePolicy);
        BTN_SOUND_18->setMaximumSize(QSize(500, 16777215));
        BTN_SOUND_18->setFont(font);
        BTN_SOUND_19 = new QPushButton(groupBox_3);
        BTN_SOUND_19->setObjectName(QStringLiteral("BTN_SOUND_19"));
        BTN_SOUND_19->setGeometry(QRect(100, 110, 71, 21));
        sizePolicy.setHeightForWidth(BTN_SOUND_19->sizePolicy().hasHeightForWidth());
        BTN_SOUND_19->setSizePolicy(sizePolicy);
        BTN_SOUND_19->setMaximumSize(QSize(500, 16777215));
        BTN_SOUND_19->setFont(font);
        BTN_SOUND_20 = new QPushButton(groupBox_3);
        BTN_SOUND_20->setObjectName(QStringLiteral("BTN_SOUND_20"));
        BTN_SOUND_20->setGeometry(QRect(180, 110, 71, 21));
        sizePolicy.setHeightForWidth(BTN_SOUND_20->sizePolicy().hasHeightForWidth());
        BTN_SOUND_20->setSizePolicy(sizePolicy);
        BTN_SOUND_20->setMaximumSize(QSize(500, 16777215));
        BTN_SOUND_20->setFont(font);
        BTN_SOUND_21 = new QPushButton(groupBox_3);
        BTN_SOUND_21->setObjectName(QStringLiteral("BTN_SOUND_21"));
        BTN_SOUND_21->setGeometry(QRect(260, 110, 71, 21));
        sizePolicy.setHeightForWidth(BTN_SOUND_21->sizePolicy().hasHeightForWidth());
        BTN_SOUND_21->setSizePolicy(sizePolicy);
        BTN_SOUND_21->setMaximumSize(QSize(500, 16777215));
        BTN_SOUND_21->setFont(font);
        BTN_SOUND_22 = new QPushButton(groupBox_3);
        BTN_SOUND_22->setObjectName(QStringLiteral("BTN_SOUND_22"));
        BTN_SOUND_22->setGeometry(QRect(340, 110, 71, 21));
        sizePolicy.setHeightForWidth(BTN_SOUND_22->sizePolicy().hasHeightForWidth());
        BTN_SOUND_22->setSizePolicy(sizePolicy);
        BTN_SOUND_22->setMaximumSize(QSize(500, 16777215));
        BTN_SOUND_22->setFont(font);
        BTN_SOUND_23 = new QPushButton(groupBox_3);
        BTN_SOUND_23->setObjectName(QStringLiteral("BTN_SOUND_23"));
        BTN_SOUND_23->setGeometry(QRect(20, 140, 71, 21));
        sizePolicy.setHeightForWidth(BTN_SOUND_23->sizePolicy().hasHeightForWidth());
        BTN_SOUND_23->setSizePolicy(sizePolicy);
        BTN_SOUND_23->setMaximumSize(QSize(500, 16777215));
        BTN_SOUND_23->setFont(font);
        BTN_SOUND_24 = new QPushButton(groupBox_3);
        BTN_SOUND_24->setObjectName(QStringLiteral("BTN_SOUND_24"));
        BTN_SOUND_24->setGeometry(QRect(100, 140, 71, 21));
        sizePolicy.setHeightForWidth(BTN_SOUND_24->sizePolicy().hasHeightForWidth());
        BTN_SOUND_24->setSizePolicy(sizePolicy);
        BTN_SOUND_24->setMaximumSize(QSize(500, 16777215));
        BTN_SOUND_24->setFont(font);
        BTN_SOUND_25 = new QPushButton(groupBox_3);
        BTN_SOUND_25->setObjectName(QStringLiteral("BTN_SOUND_25"));
        BTN_SOUND_25->setGeometry(QRect(180, 140, 71, 21));
        sizePolicy.setHeightForWidth(BTN_SOUND_25->sizePolicy().hasHeightForWidth());
        BTN_SOUND_25->setSizePolicy(sizePolicy);
        BTN_SOUND_25->setMaximumSize(QSize(500, 16777215));
        BTN_SOUND_25->setFont(font);
        BTN_SOUND_26 = new QPushButton(groupBox_3);
        BTN_SOUND_26->setObjectName(QStringLiteral("BTN_SOUND_26"));
        BTN_SOUND_26->setGeometry(QRect(260, 140, 71, 21));
        sizePolicy.setHeightForWidth(BTN_SOUND_26->sizePolicy().hasHeightForWidth());
        BTN_SOUND_26->setSizePolicy(sizePolicy);
        BTN_SOUND_26->setMaximumSize(QSize(500, 16777215));
        BTN_SOUND_26->setFont(font);
        BTN_SOUND_27 = new QPushButton(groupBox_3);
        BTN_SOUND_27->setObjectName(QStringLiteral("BTN_SOUND_27"));
        BTN_SOUND_27->setGeometry(QRect(340, 140, 71, 21));
        sizePolicy.setHeightForWidth(BTN_SOUND_27->sizePolicy().hasHeightForWidth());
        BTN_SOUND_27->setSizePolicy(sizePolicy);
        BTN_SOUND_27->setMaximumSize(QSize(500, 16777215));
        BTN_SOUND_27->setFont(font);
        BTN_SOUND_28 = new QPushButton(groupBox_3);
        BTN_SOUND_28->setObjectName(QStringLiteral("BTN_SOUND_28"));
        BTN_SOUND_28->setGeometry(QRect(20, 170, 71, 21));
        sizePolicy.setHeightForWidth(BTN_SOUND_28->sizePolicy().hasHeightForWidth());
        BTN_SOUND_28->setSizePolicy(sizePolicy);
        BTN_SOUND_28->setMaximumSize(QSize(500, 16777215));
        BTN_SOUND_28->setFont(font);
        BTN_SOUND_29 = new QPushButton(groupBox_3);
        BTN_SOUND_29->setObjectName(QStringLiteral("BTN_SOUND_29"));
        BTN_SOUND_29->setGeometry(QRect(100, 170, 71, 21));
        sizePolicy.setHeightForWidth(BTN_SOUND_29->sizePolicy().hasHeightForWidth());
        BTN_SOUND_29->setSizePolicy(sizePolicy);
        BTN_SOUND_29->setMaximumSize(QSize(500, 16777215));
        BTN_SOUND_29->setFont(font);
        BTN_SOUND_30 = new QPushButton(groupBox_3);
        BTN_SOUND_30->setObjectName(QStringLiteral("BTN_SOUND_30"));
        BTN_SOUND_30->setGeometry(QRect(180, 170, 71, 21));
        sizePolicy.setHeightForWidth(BTN_SOUND_30->sizePolicy().hasHeightForWidth());
        BTN_SOUND_30->setSizePolicy(sizePolicy);
        BTN_SOUND_30->setMaximumSize(QSize(500, 16777215));
        BTN_SOUND_30->setFont(font);
        BTN_SOUND_31 = new QPushButton(groupBox_3);
        BTN_SOUND_31->setObjectName(QStringLiteral("BTN_SOUND_31"));
        BTN_SOUND_31->setGeometry(QRect(260, 170, 71, 21));
        sizePolicy.setHeightForWidth(BTN_SOUND_31->sizePolicy().hasHeightForWidth());
        BTN_SOUND_31->setSizePolicy(sizePolicy);
        BTN_SOUND_31->setMaximumSize(QSize(500, 16777215));
        BTN_SOUND_31->setFont(font);
        BTN_SOUND_32 = new QPushButton(groupBox_3);
        BTN_SOUND_32->setObjectName(QStringLiteral("BTN_SOUND_32"));
        BTN_SOUND_32->setGeometry(QRect(340, 170, 71, 21));
        sizePolicy.setHeightForWidth(BTN_SOUND_32->sizePolicy().hasHeightForWidth());
        BTN_SOUND_32->setSizePolicy(sizePolicy);
        BTN_SOUND_32->setMaximumSize(QSize(500, 16777215));
        BTN_SOUND_32->setFont(font);
        BTN_SOUND_33 = new QPushButton(groupBox_3);
        BTN_SOUND_33->setObjectName(QStringLiteral("BTN_SOUND_33"));
        BTN_SOUND_33->setGeometry(QRect(20, 200, 71, 21));
        sizePolicy.setHeightForWidth(BTN_SOUND_33->sizePolicy().hasHeightForWidth());
        BTN_SOUND_33->setSizePolicy(sizePolicy);
        BTN_SOUND_33->setMaximumSize(QSize(500, 16777215));
        BTN_SOUND_33->setFont(font);
        BTN_SOUND_34 = new QPushButton(groupBox_3);
        BTN_SOUND_34->setObjectName(QStringLiteral("BTN_SOUND_34"));
        BTN_SOUND_34->setGeometry(QRect(100, 200, 71, 21));
        sizePolicy.setHeightForWidth(BTN_SOUND_34->sizePolicy().hasHeightForWidth());
        BTN_SOUND_34->setSizePolicy(sizePolicy);
        BTN_SOUND_34->setMaximumSize(QSize(500, 16777215));
        BTN_SOUND_34->setFont(font);
        BTN_SOUND_35 = new QPushButton(groupBox_3);
        BTN_SOUND_35->setObjectName(QStringLiteral("BTN_SOUND_35"));
        BTN_SOUND_35->setGeometry(QRect(180, 200, 71, 21));
        sizePolicy.setHeightForWidth(BTN_SOUND_35->sizePolicy().hasHeightForWidth());
        BTN_SOUND_35->setSizePolicy(sizePolicy);
        BTN_SOUND_35->setMaximumSize(QSize(500, 16777215));
        BTN_SOUND_35->setFont(font);
        BTN_SOUND_36 = new QPushButton(groupBox_3);
        BTN_SOUND_36->setObjectName(QStringLiteral("BTN_SOUND_36"));
        BTN_SOUND_36->setGeometry(QRect(260, 200, 71, 21));
        sizePolicy.setHeightForWidth(BTN_SOUND_36->sizePolicy().hasHeightForWidth());
        BTN_SOUND_36->setSizePolicy(sizePolicy);
        BTN_SOUND_36->setMaximumSize(QSize(500, 16777215));
        BTN_SOUND_36->setFont(font);
        BTN_SOUND_37 = new QPushButton(groupBox_3);
        BTN_SOUND_37->setObjectName(QStringLiteral("BTN_SOUND_37"));
        BTN_SOUND_37->setGeometry(QRect(340, 200, 71, 21));
        sizePolicy.setHeightForWidth(BTN_SOUND_37->sizePolicy().hasHeightForWidth());
        BTN_SOUND_37->setSizePolicy(sizePolicy);
        BTN_SOUND_37->setMaximumSize(QSize(500, 16777215));
        BTN_SOUND_37->setFont(font);
        BTN_SOUND_38 = new QPushButton(groupBox_3);
        BTN_SOUND_38->setObjectName(QStringLiteral("BTN_SOUND_38"));
        BTN_SOUND_38->setGeometry(QRect(20, 230, 71, 21));
        sizePolicy.setHeightForWidth(BTN_SOUND_38->sizePolicy().hasHeightForWidth());
        BTN_SOUND_38->setSizePolicy(sizePolicy);
        BTN_SOUND_38->setMaximumSize(QSize(500, 16777215));
        BTN_SOUND_38->setFont(font);
        BTN_SOUND_39 = new QPushButton(groupBox_3);
        BTN_SOUND_39->setObjectName(QStringLiteral("BTN_SOUND_39"));
        BTN_SOUND_39->setGeometry(QRect(100, 230, 71, 21));
        sizePolicy.setHeightForWidth(BTN_SOUND_39->sizePolicy().hasHeightForWidth());
        BTN_SOUND_39->setSizePolicy(sizePolicy);
        BTN_SOUND_39->setMaximumSize(QSize(500, 16777215));
        BTN_SOUND_39->setFont(font);
        BTN_SOUND_40 = new QPushButton(groupBox_3);
        BTN_SOUND_40->setObjectName(QStringLiteral("BTN_SOUND_40"));
        BTN_SOUND_40->setGeometry(QRect(180, 230, 71, 21));
        sizePolicy.setHeightForWidth(BTN_SOUND_40->sizePolicy().hasHeightForWidth());
        BTN_SOUND_40->setSizePolicy(sizePolicy);
        BTN_SOUND_40->setMaximumSize(QSize(500, 16777215));
        BTN_SOUND_40->setFont(font);
        BTN_SOUND_41 = new QPushButton(groupBox_3);
        BTN_SOUND_41->setObjectName(QStringLiteral("BTN_SOUND_41"));
        BTN_SOUND_41->setGeometry(QRect(260, 230, 71, 21));
        sizePolicy.setHeightForWidth(BTN_SOUND_41->sizePolicy().hasHeightForWidth());
        BTN_SOUND_41->setSizePolicy(sizePolicy);
        BTN_SOUND_41->setMaximumSize(QSize(500, 16777215));
        BTN_SOUND_41->setFont(font);
        BTN_SOUND_42 = new QPushButton(groupBox_3);
        BTN_SOUND_42->setObjectName(QStringLiteral("BTN_SOUND_42"));
        BTN_SOUND_42->setGeometry(QRect(340, 230, 71, 21));
        sizePolicy.setHeightForWidth(BTN_SOUND_42->sizePolicy().hasHeightForWidth());
        BTN_SOUND_42->setSizePolicy(sizePolicy);
        BTN_SOUND_42->setMaximumSize(QSize(500, 16777215));
        BTN_SOUND_42->setFont(font);
        BTN_SOUND_43 = new QPushButton(groupBox_3);
        BTN_SOUND_43->setObjectName(QStringLiteral("BTN_SOUND_43"));
        BTN_SOUND_43->setGeometry(QRect(20, 260, 71, 21));
        sizePolicy.setHeightForWidth(BTN_SOUND_43->sizePolicy().hasHeightForWidth());
        BTN_SOUND_43->setSizePolicy(sizePolicy);
        BTN_SOUND_43->setMaximumSize(QSize(500, 16777215));
        BTN_SOUND_43->setFont(font);
        BTN_SOUND_44 = new QPushButton(groupBox_3);
        BTN_SOUND_44->setObjectName(QStringLiteral("BTN_SOUND_44"));
        BTN_SOUND_44->setGeometry(QRect(100, 260, 71, 21));
        sizePolicy.setHeightForWidth(BTN_SOUND_44->sizePolicy().hasHeightForWidth());
        BTN_SOUND_44->setSizePolicy(sizePolicy);
        BTN_SOUND_44->setMaximumSize(QSize(500, 16777215));
        BTN_SOUND_44->setFont(font);
        BTN_SOUND_45 = new QPushButton(groupBox_3);
        BTN_SOUND_45->setObjectName(QStringLiteral("BTN_SOUND_45"));
        BTN_SOUND_45->setGeometry(QRect(180, 260, 71, 21));
        sizePolicy.setHeightForWidth(BTN_SOUND_45->sizePolicy().hasHeightForWidth());
        BTN_SOUND_45->setSizePolicy(sizePolicy);
        BTN_SOUND_45->setMaximumSize(QSize(500, 16777215));
        BTN_SOUND_45->setFont(font);
        BTN_SOUND_46 = new QPushButton(groupBox_3);
        BTN_SOUND_46->setObjectName(QStringLiteral("BTN_SOUND_46"));
        BTN_SOUND_46->setGeometry(QRect(260, 260, 71, 21));
        sizePolicy.setHeightForWidth(BTN_SOUND_46->sizePolicy().hasHeightForWidth());
        BTN_SOUND_46->setSizePolicy(sizePolicy);
        BTN_SOUND_46->setMaximumSize(QSize(500, 16777215));
        BTN_SOUND_46->setFont(font);
        BTN_SOUND_47 = new QPushButton(groupBox_3);
        BTN_SOUND_47->setObjectName(QStringLiteral("BTN_SOUND_47"));
        BTN_SOUND_47->setGeometry(QRect(340, 260, 71, 21));
        sizePolicy.setHeightForWidth(BTN_SOUND_47->sizePolicy().hasHeightForWidth());
        BTN_SOUND_47->setSizePolicy(sizePolicy);
        BTN_SOUND_47->setMaximumSize(QSize(500, 16777215));
        BTN_SOUND_47->setFont(font);
        BTN_SOUND_88 = new QPushButton(groupBox_3);
        BTN_SOUND_88->setObjectName(QStringLiteral("BTN_SOUND_88"));
        BTN_SOUND_88->setGeometry(QRect(80, 310, 51, 31));
        sizePolicy.setHeightForWidth(BTN_SOUND_88->sizePolicy().hasHeightForWidth());
        BTN_SOUND_88->setSizePolicy(sizePolicy);
        BTN_SOUND_88->setMaximumSize(QSize(500, 16777215));
        BTN_SOUND_88->setFont(font);
        BTN_SOUND_89 = new QPushButton(groupBox_3);
        BTN_SOUND_89->setObjectName(QStringLiteral("BTN_SOUND_89"));
        BTN_SOUND_89->setGeometry(QRect(80, 350, 51, 31));
        sizePolicy.setHeightForWidth(BTN_SOUND_89->sizePolicy().hasHeightForWidth());
        BTN_SOUND_89->setSizePolicy(sizePolicy);
        BTN_SOUND_89->setMaximumSize(QSize(500, 16777215));
        BTN_SOUND_89->setFont(font);
        BTN_SOUND_90 = new QPushButton(groupBox_3);
        BTN_SOUND_90->setObjectName(QStringLiteral("BTN_SOUND_90"));
        BTN_SOUND_90->setGeometry(QRect(80, 390, 51, 31));
        sizePolicy.setHeightForWidth(BTN_SOUND_90->sizePolicy().hasHeightForWidth());
        BTN_SOUND_90->setSizePolicy(sizePolicy);
        BTN_SOUND_90->setMaximumSize(QSize(500, 16777215));
        BTN_SOUND_90->setFont(font);
        BTN_SOUND_92 = new QPushButton(groupBox_3);
        BTN_SOUND_92->setObjectName(QStringLiteral("BTN_SOUND_92"));
        BTN_SOUND_92->setGeometry(QRect(150, 350, 51, 31));
        sizePolicy.setHeightForWidth(BTN_SOUND_92->sizePolicy().hasHeightForWidth());
        BTN_SOUND_92->setSizePolicy(sizePolicy);
        BTN_SOUND_92->setMaximumSize(QSize(500, 16777215));
        BTN_SOUND_92->setFont(font);
        BTN_SOUND_93 = new QPushButton(groupBox_3);
        BTN_SOUND_93->setObjectName(QStringLiteral("BTN_SOUND_93"));
        BTN_SOUND_93->setGeometry(QRect(150, 390, 51, 31));
        sizePolicy.setHeightForWidth(BTN_SOUND_93->sizePolicy().hasHeightForWidth());
        BTN_SOUND_93->setSizePolicy(sizePolicy);
        BTN_SOUND_93->setMaximumSize(QSize(500, 16777215));
        BTN_SOUND_93->setFont(font);
        BTN_SOUND_95 = new QPushButton(groupBox_3);
        BTN_SOUND_95->setObjectName(QStringLiteral("BTN_SOUND_95"));
        BTN_SOUND_95->setGeometry(QRect(220, 350, 51, 31));
        sizePolicy.setHeightForWidth(BTN_SOUND_95->sizePolicy().hasHeightForWidth());
        BTN_SOUND_95->setSizePolicy(sizePolicy);
        BTN_SOUND_95->setMaximumSize(QSize(500, 16777215));
        BTN_SOUND_95->setFont(font);
        BTN_SOUND_96 = new QPushButton(groupBox_3);
        BTN_SOUND_96->setObjectName(QStringLiteral("BTN_SOUND_96"));
        BTN_SOUND_96->setGeometry(QRect(220, 390, 51, 31));
        sizePolicy.setHeightForWidth(BTN_SOUND_96->sizePolicy().hasHeightForWidth());
        BTN_SOUND_96->setSizePolicy(sizePolicy);
        BTN_SOUND_96->setMaximumSize(QSize(500, 16777215));
        BTN_SOUND_96->setFont(font);
        BTN_SOUND_97 = new QPushButton(groupBox_3);
        BTN_SOUND_97->setObjectName(QStringLiteral("BTN_SOUND_97"));
        BTN_SOUND_97->setGeometry(QRect(290, 310, 51, 31));
        sizePolicy.setHeightForWidth(BTN_SOUND_97->sizePolicy().hasHeightForWidth());
        BTN_SOUND_97->setSizePolicy(sizePolicy);
        BTN_SOUND_97->setMaximumSize(QSize(500, 16777215));
        BTN_SOUND_97->setFont(font);
        BTN_SOUND_99 = new QPushButton(groupBox_3);
        BTN_SOUND_99->setObjectName(QStringLiteral("BTN_SOUND_99"));
        BTN_SOUND_99->setGeometry(QRect(290, 390, 51, 31));
        sizePolicy.setHeightForWidth(BTN_SOUND_99->sizePolicy().hasHeightForWidth());
        BTN_SOUND_99->setSizePolicy(sizePolicy);
        BTN_SOUND_99->setMaximumSize(QSize(500, 16777215));
        BTN_SOUND_99->setFont(font);
        BTN_SOUND_101 = new QPushButton(groupBox_3);
        BTN_SOUND_101->setObjectName(QStringLiteral("BTN_SOUND_101"));
        BTN_SOUND_101->setGeometry(QRect(360, 350, 51, 31));
        sizePolicy.setHeightForWidth(BTN_SOUND_101->sizePolicy().hasHeightForWidth());
        BTN_SOUND_101->setSizePolicy(sizePolicy);
        BTN_SOUND_101->setMaximumSize(QSize(500, 16777215));
        BTN_SOUND_101->setFont(font);
        BTN_SOUND_102 = new QPushButton(groupBox_3);
        BTN_SOUND_102->setObjectName(QStringLiteral("BTN_SOUND_102"));
        BTN_SOUND_102->setGeometry(QRect(360, 390, 51, 31));
        sizePolicy.setHeightForWidth(BTN_SOUND_102->sizePolicy().hasHeightForWidth());
        BTN_SOUND_102->setSizePolicy(sizePolicy);
        BTN_SOUND_102->setMaximumSize(QSize(500, 16777215));
        BTN_SOUND_102->setFont(font);
        label = new QLabel(groupBox_3);
        label->setObjectName(QStringLiteral("label"));
        label->setGeometry(QRect(10, 290, 51, 17));
        QFont font1;
        font1.setPointSize(10);
        font1.setBold(true);
        font1.setWeight(75);
        font1.setStyleStrategy(QFont::PreferDefault);
        label->setFont(font1);
        label_2 = new QLabel(groupBox_3);
        label_2->setObjectName(QStringLiteral("label_2"));
        label_2->setGeometry(QRect(80, 290, 51, 17));
        label_2->setFont(font1);
        label_3 = new QLabel(groupBox_3);
        label_3->setObjectName(QStringLiteral("label_3"));
        label_3->setGeometry(QRect(150, 290, 51, 17));
        label_3->setFont(font1);
        label_4 = new QLabel(groupBox_3);
        label_4->setObjectName(QStringLiteral("label_4"));
        label_4->setGeometry(QRect(220, 290, 51, 17));
        label_4->setFont(font1);
        label_5 = new QLabel(groupBox_3);
        label_5->setObjectName(QStringLiteral("label_5"));
        label_5->setGeometry(QRect(290, 290, 51, 17));
        label_5->setFont(font1);
        rad_kor = new QRadioButton(groupBox_3);
        rad_kor->setObjectName(QStringLiteral("rad_kor"));
        rad_kor->setGeometry(QRect(20, 20, 71, 23));
        rad_eng = new QRadioButton(groupBox_3);
        rad_eng->setObjectName(QStringLiteral("rad_eng"));
        rad_eng->setGeometry(QRect(100, 20, 71, 23));
        BTN_SOUND_104 = new QPushButton(groupBox_3);
        BTN_SOUND_104->setObjectName(QStringLiteral("BTN_SOUND_104"));
        BTN_SOUND_104->setGeometry(QRect(10, 350, 51, 31));
        sizePolicy.setHeightForWidth(BTN_SOUND_104->sizePolicy().hasHeightForWidth());
        BTN_SOUND_104->setSizePolicy(sizePolicy);
        BTN_SOUND_104->setMaximumSize(QSize(500, 16777215));
        BTN_SOUND_104->setFont(font);
        BTN_SOUND_105 = new QPushButton(groupBox_3);
        BTN_SOUND_105->setObjectName(QStringLiteral("BTN_SOUND_105"));
        BTN_SOUND_105->setGeometry(QRect(10, 390, 51, 31));
        sizePolicy.setHeightForWidth(BTN_SOUND_105->sizePolicy().hasHeightForWidth());
        BTN_SOUND_105->setSizePolicy(sizePolicy);
        BTN_SOUND_105->setMaximumSize(QSize(500, 16777215));
        BTN_SOUND_105->setFont(font);
        BTN_SOUND_91 = new QPushButton(groupBox_3);
        BTN_SOUND_91->setObjectName(QStringLiteral("BTN_SOUND_91"));
        BTN_SOUND_91->setGeometry(QRect(150, 310, 51, 31));
        sizePolicy.setHeightForWidth(BTN_SOUND_91->sizePolicy().hasHeightForWidth());
        BTN_SOUND_91->setSizePolicy(sizePolicy);
        BTN_SOUND_91->setMaximumSize(QSize(500, 16777215));
        BTN_SOUND_91->setFont(font);
        BTN_SOUND_94 = new QPushButton(groupBox_3);
        BTN_SOUND_94->setObjectName(QStringLiteral("BTN_SOUND_94"));
        BTN_SOUND_94->setGeometry(QRect(220, 310, 51, 31));
        sizePolicy.setHeightForWidth(BTN_SOUND_94->sizePolicy().hasHeightForWidth());
        BTN_SOUND_94->setSizePolicy(sizePolicy);
        BTN_SOUND_94->setMaximumSize(QSize(500, 16777215));
        BTN_SOUND_94->setFont(font);
        BTN_SOUND_98 = new QPushButton(groupBox_3);
        BTN_SOUND_98->setObjectName(QStringLiteral("BTN_SOUND_98"));
        BTN_SOUND_98->setGeometry(QRect(290, 350, 51, 31));
        sizePolicy.setHeightForWidth(BTN_SOUND_98->sizePolicy().hasHeightForWidth());
        BTN_SOUND_98->setSizePolicy(sizePolicy);
        BTN_SOUND_98->setMaximumSize(QSize(500, 16777215));
        BTN_SOUND_98->setFont(font);
        BTN_SOUND_100 = new QPushButton(groupBox_3);
        BTN_SOUND_100->setObjectName(QStringLiteral("BTN_SOUND_100"));
        BTN_SOUND_100->setGeometry(QRect(360, 310, 51, 31));
        sizePolicy.setHeightForWidth(BTN_SOUND_100->sizePolicy().hasHeightForWidth());
        BTN_SOUND_100->setSizePolicy(sizePolicy);
        BTN_SOUND_100->setMaximumSize(QSize(500, 16777215));
        BTN_SOUND_100->setFont(font);
        BTN_SOUND_103 = new QPushButton(groupBox_3);
        BTN_SOUND_103->setObjectName(QStringLiteral("BTN_SOUND_103"));
        BTN_SOUND_103->setGeometry(QRect(10, 310, 51, 31));
        sizePolicy.setHeightForWidth(BTN_SOUND_103->sizePolicy().hasHeightForWidth());
        BTN_SOUND_103->setSizePolicy(sizePolicy);
        BTN_SOUND_103->setMaximumSize(QSize(500, 16777215));
        BTN_SOUND_103->setFont(font);
        BTN_SOUND_7 = new QPushButton(groupBox_3);
        BTN_SOUND_7->setObjectName(QStringLiteral("BTN_SOUND_7"));
        BTN_SOUND_7->setGeometry(QRect(100, 80, 71, 21));
        sizePolicy.setHeightForWidth(BTN_SOUND_7->sizePolicy().hasHeightForWidth());
        BTN_SOUND_7->setSizePolicy(sizePolicy);
        BTN_SOUND_7->setMaximumSize(QSize(500, 16777215));
        BTN_SOUND_7->setFont(font);
        label_6 = new QLabel(groupBox_3);
        label_6->setObjectName(QStringLiteral("label_6"));
        label_6->setGeometry(QRect(360, 290, 51, 17));
        label_6->setFont(font1);
        BTN_ENABLE = new QPushButton(groupBox_3);
        BTN_ENABLE->setObjectName(QStringLiteral("BTN_ENABLE"));
        BTN_ENABLE->setGeometry(QRect(180, 20, 51, 21));
        sizePolicy.setHeightForWidth(BTN_ENABLE->sizePolicy().hasHeightForWidth());
        BTN_ENABLE->setSizePolicy(sizePolicy);
        BTN_ENABLE->setMaximumSize(QSize(500, 16777215));
        BTN_ENABLE->setFont(font);
        BTN_DISABLE = new QPushButton(groupBox_3);
        BTN_DISABLE->setObjectName(QStringLiteral("BTN_DISABLE"));
        BTN_DISABLE->setGeometry(QRect(240, 20, 51, 21));
        sizePolicy.setHeightForWidth(BTN_DISABLE->sizePolicy().hasHeightForWidth());
        BTN_DISABLE->setSizePolicy(sizePolicy);
        BTN_DISABLE->setMaximumSize(QSize(500, 16777215));
        BTN_DISABLE->setFont(font);
        BTN_wa = new QPushButton(groupBox_3);
        BTN_wa->setObjectName(QStringLiteral("BTN_wa"));
        BTN_wa->setGeometry(QRect(300, 20, 51, 21));
        sizePolicy.setHeightForWidth(BTN_wa->sizePolicy().hasHeightForWidth());
        BTN_wa->setSizePolicy(sizePolicy);
        BTN_wa->setMaximumSize(QSize(500, 16777215));
        BTN_wa->setFont(font);
        BTN_oh = new QPushButton(groupBox_3);
        BTN_oh->setObjectName(QStringLiteral("BTN_oh"));
        BTN_oh->setGeometry(QRect(360, 20, 51, 21));
        sizePolicy.setHeightForWidth(BTN_oh->sizePolicy().hasHeightForWidth());
        BTN_oh->setSizePolicy(sizePolicy);
        BTN_oh->setMaximumSize(QSize(500, 16777215));
        BTN_oh->setFont(font);
        BTN_QUICK_STOP = new QPushButton(groupBox_3);
        BTN_QUICK_STOP->setObjectName(QStringLiteral("BTN_QUICK_STOP"));
        BTN_QUICK_STOP->setGeometry(QRect(190, 430, 71, 21));
        sizePolicy.setHeightForWidth(BTN_QUICK_STOP->sizePolicy().hasHeightForWidth());
        BTN_QUICK_STOP->setSizePolicy(sizePolicy);
        BTN_QUICK_STOP->setMaximumSize(QSize(500, 16777215));
        BTN_QUICK_STOP->setFont(font);
        groupBox_4 = new QGroupBox(ModelDialog);
        groupBox_4->setObjectName(QStringLiteral("groupBox_4"));
        groupBox_4->setEnabled(true);
        groupBox_4->setGeometry(QRect(0, 510, 471, 201));
        QFont font2;
        font2.setPointSize(11);
        font2.setBold(true);
        font2.setWeight(75);
        groupBox_4->setFont(font2);
        groupBox_4->setStyleSheet(QLatin1String("QGroupBox {\n"
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
        gridLayoutWidget_12 = new QWidget(groupBox_4);
        gridLayoutWidget_12->setObjectName(QStringLiteral("gridLayoutWidget_12"));
        gridLayoutWidget_12->setEnabled(true);
        gridLayoutWidget_12->setGeometry(QRect(10, 60, 451, 131));
        gridLayoutWidget_12->setFont(font);
        gridLayout_12 = new QGridLayout(gridLayoutWidget_12);
        gridLayout_12->setObjectName(QStringLiteral("gridLayout_12"));
        gridLayout_12->setContentsMargins(0, 0, 0, 0);
        LE_SENSOR_LAFT_MY = new QLineEdit(gridLayoutWidget_12);
        LE_SENSOR_LAFT_MY->setObjectName(QStringLiteral("LE_SENSOR_LAFT_MY"));
        LE_SENSOR_LAFT_MY->setEnabled(true);
        QSizePolicy sizePolicy1(QSizePolicy::Fixed, QSizePolicy::Fixed);
        sizePolicy1.setHorizontalStretch(0);
        sizePolicy1.setVerticalStretch(0);
        sizePolicy1.setHeightForWidth(LE_SENSOR_LAFT_MY->sizePolicy().hasHeightForWidth());
        LE_SENSOR_LAFT_MY->setSizePolicy(sizePolicy1);
        LE_SENSOR_LAFT_MY->setMinimumSize(QSize(60, 20));
        LE_SENSOR_LAFT_MY->setMaximumSize(QSize(60, 20));
        QFont font3;
        font3.setPointSize(9);
        LE_SENSOR_LAFT_MY->setFont(font3);
        LE_SENSOR_LAFT_MY->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        LE_SENSOR_LAFT_MY->setReadOnly(true);

        gridLayout_12->addWidget(LE_SENSOR_LAFT_MY, 4, 2, 1, 1);

        LE_SENSOR_LWFT_MY = new QLineEdit(gridLayoutWidget_12);
        LE_SENSOR_LWFT_MY->setObjectName(QStringLiteral("LE_SENSOR_LWFT_MY"));
        LE_SENSOR_LWFT_MY->setEnabled(true);
        sizePolicy1.setHeightForWidth(LE_SENSOR_LWFT_MY->sizePolicy().hasHeightForWidth());
        LE_SENSOR_LWFT_MY->setSizePolicy(sizePolicy1);
        LE_SENSOR_LWFT_MY->setMinimumSize(QSize(60, 20));
        LE_SENSOR_LWFT_MY->setMaximumSize(QSize(60, 20));
        LE_SENSOR_LWFT_MY->setFont(font3);
        LE_SENSOR_LWFT_MY->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        LE_SENSOR_LWFT_MY->setReadOnly(true);

        gridLayout_12->addWidget(LE_SENSOR_LWFT_MY, 2, 2, 1, 1);

        label_43 = new QLabel(gridLayoutWidget_12);
        label_43->setObjectName(QStringLiteral("label_43"));
        label_43->setEnabled(true);
        sizePolicy1.setHeightForWidth(label_43->sizePolicy().hasHeightForWidth());
        label_43->setSizePolicy(sizePolicy1);
        label_43->setMinimumSize(QSize(60, 20));
        label_43->setMaximumSize(QSize(60, 20));
        label_43->setFont(font3);
        label_43->setAlignment(Qt::AlignCenter);

        gridLayout_12->addWidget(label_43, 0, 5, 1, 1);

        label_32 = new QLabel(gridLayoutWidget_12);
        label_32->setObjectName(QStringLiteral("label_32"));
        label_32->setEnabled(true);
        sizePolicy1.setHeightForWidth(label_32->sizePolicy().hasHeightForWidth());
        label_32->setSizePolicy(sizePolicy1);
        label_32->setMinimumSize(QSize(60, 20));
        label_32->setMaximumSize(QSize(60, 20));
        label_32->setFont(font3);
        label_32->setAlignment(Qt::AlignCenter);

        gridLayout_12->addWidget(label_32, 0, 3, 1, 1);

        LE_SENSOR_LWFT_MZ = new QLineEdit(gridLayoutWidget_12);
        LE_SENSOR_LWFT_MZ->setObjectName(QStringLiteral("LE_SENSOR_LWFT_MZ"));
        LE_SENSOR_LWFT_MZ->setEnabled(true);
        sizePolicy1.setHeightForWidth(LE_SENSOR_LWFT_MZ->sizePolicy().hasHeightForWidth());
        LE_SENSOR_LWFT_MZ->setSizePolicy(sizePolicy1);
        LE_SENSOR_LWFT_MZ->setMinimumSize(QSize(60, 20));
        LE_SENSOR_LWFT_MZ->setMaximumSize(QSize(60, 20));
        LE_SENSOR_LWFT_MZ->setFont(font3);
        LE_SENSOR_LWFT_MZ->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        LE_SENSOR_LWFT_MZ->setReadOnly(true);

        gridLayout_12->addWidget(LE_SENSOR_LWFT_MZ, 2, 3, 1, 1);

        label_44 = new QLabel(gridLayoutWidget_12);
        label_44->setObjectName(QStringLiteral("label_44"));
        label_44->setEnabled(true);
        sizePolicy1.setHeightForWidth(label_44->sizePolicy().hasHeightForWidth());
        label_44->setSizePolicy(sizePolicy1);
        label_44->setMinimumSize(QSize(60, 20));
        label_44->setMaximumSize(QSize(60, 20));
        label_44->setFont(font3);
        label_44->setAlignment(Qt::AlignCenter);

        gridLayout_12->addWidget(label_44, 0, 1, 1, 1);

        label_45 = new QLabel(gridLayoutWidget_12);
        label_45->setObjectName(QStringLiteral("label_45"));
        label_45->setEnabled(true);
        label_45->setMinimumSize(QSize(50, 20));
        label_45->setMaximumSize(QSize(50, 20));
        label_45->setFont(font3);
        label_45->setAlignment(Qt::AlignCenter);

        gridLayout_12->addWidget(label_45, 1, 0, 1, 1);

        label_39 = new QLabel(gridLayoutWidget_12);
        label_39->setObjectName(QStringLiteral("label_39"));
        label_39->setEnabled(true);
        label_39->setMinimumSize(QSize(50, 20));
        label_39->setMaximumSize(QSize(50, 20));
        label_39->setFont(font3);
        label_39->setAlignment(Qt::AlignCenter);

        gridLayout_12->addWidget(label_39, 2, 0, 1, 1);

        LE_SENSOR_RAFT_FZ = new QLineEdit(gridLayoutWidget_12);
        LE_SENSOR_RAFT_FZ->setObjectName(QStringLiteral("LE_SENSOR_RAFT_FZ"));
        LE_SENSOR_RAFT_FZ->setEnabled(true);
        sizePolicy1.setHeightForWidth(LE_SENSOR_RAFT_FZ->sizePolicy().hasHeightForWidth());
        LE_SENSOR_RAFT_FZ->setSizePolicy(sizePolicy1);
        LE_SENSOR_RAFT_FZ->setMinimumSize(QSize(60, 20));
        LE_SENSOR_RAFT_FZ->setMaximumSize(QSize(60, 20));
        LE_SENSOR_RAFT_FZ->setFont(font3);
        LE_SENSOR_RAFT_FZ->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        LE_SENSOR_RAFT_FZ->setReadOnly(true);

        gridLayout_12->addWidget(LE_SENSOR_RAFT_FZ, 3, 6, 1, 1);

        label_47 = new QLabel(gridLayoutWidget_12);
        label_47->setObjectName(QStringLiteral("label_47"));
        label_47->setEnabled(true);
        label_47->setMinimumSize(QSize(50, 20));
        label_47->setMaximumSize(QSize(50, 20));
        label_47->setFont(font3);
        label_47->setAlignment(Qt::AlignCenter);

        gridLayout_12->addWidget(label_47, 3, 0, 1, 1);

        LE_SENSOR_LAFT_FZ = new QLineEdit(gridLayoutWidget_12);
        LE_SENSOR_LAFT_FZ->setObjectName(QStringLiteral("LE_SENSOR_LAFT_FZ"));
        LE_SENSOR_LAFT_FZ->setEnabled(true);
        sizePolicy1.setHeightForWidth(LE_SENSOR_LAFT_FZ->sizePolicy().hasHeightForWidth());
        LE_SENSOR_LAFT_FZ->setSizePolicy(sizePolicy1);
        LE_SENSOR_LAFT_FZ->setMinimumSize(QSize(60, 20));
        LE_SENSOR_LAFT_FZ->setMaximumSize(QSize(60, 20));
        LE_SENSOR_LAFT_FZ->setFont(font3);
        LE_SENSOR_LAFT_FZ->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        LE_SENSOR_LAFT_FZ->setReadOnly(true);

        gridLayout_12->addWidget(LE_SENSOR_LAFT_FZ, 4, 6, 1, 1);

        LE_SENSOR_LAFT_MX = new QLineEdit(gridLayoutWidget_12);
        LE_SENSOR_LAFT_MX->setObjectName(QStringLiteral("LE_SENSOR_LAFT_MX"));
        LE_SENSOR_LAFT_MX->setEnabled(true);
        sizePolicy1.setHeightForWidth(LE_SENSOR_LAFT_MX->sizePolicy().hasHeightForWidth());
        LE_SENSOR_LAFT_MX->setSizePolicy(sizePolicy1);
        LE_SENSOR_LAFT_MX->setMinimumSize(QSize(60, 20));
        LE_SENSOR_LAFT_MX->setMaximumSize(QSize(60, 20));
        LE_SENSOR_LAFT_MX->setFont(font3);
        LE_SENSOR_LAFT_MX->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        LE_SENSOR_LAFT_MX->setReadOnly(true);

        gridLayout_12->addWidget(LE_SENSOR_LAFT_MX, 4, 1, 1, 1);

        label_48 = new QLabel(gridLayoutWidget_12);
        label_48->setObjectName(QStringLiteral("label_48"));
        label_48->setEnabled(true);
        label_48->setMinimumSize(QSize(50, 20));
        label_48->setMaximumSize(QSize(50, 20));
        label_48->setFont(font3);
        label_48->setAlignment(Qt::AlignCenter);

        gridLayout_12->addWidget(label_48, 4, 0, 1, 1);

        LE_SENSOR_RWFT_FZ = new QLineEdit(gridLayoutWidget_12);
        LE_SENSOR_RWFT_FZ->setObjectName(QStringLiteral("LE_SENSOR_RWFT_FZ"));
        LE_SENSOR_RWFT_FZ->setEnabled(true);
        sizePolicy1.setHeightForWidth(LE_SENSOR_RWFT_FZ->sizePolicy().hasHeightForWidth());
        LE_SENSOR_RWFT_FZ->setSizePolicy(sizePolicy1);
        LE_SENSOR_RWFT_FZ->setMinimumSize(QSize(60, 20));
        LE_SENSOR_RWFT_FZ->setMaximumSize(QSize(60, 20));
        LE_SENSOR_RWFT_FZ->setFont(font3);
        LE_SENSOR_RWFT_FZ->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        LE_SENSOR_RWFT_FZ->setReadOnly(true);

        gridLayout_12->addWidget(LE_SENSOR_RWFT_FZ, 1, 6, 1, 1);

        LE_SENSOR_LWFT_MX = new QLineEdit(gridLayoutWidget_12);
        LE_SENSOR_LWFT_MX->setObjectName(QStringLiteral("LE_SENSOR_LWFT_MX"));
        LE_SENSOR_LWFT_MX->setEnabled(true);
        sizePolicy1.setHeightForWidth(LE_SENSOR_LWFT_MX->sizePolicy().hasHeightForWidth());
        LE_SENSOR_LWFT_MX->setSizePolicy(sizePolicy1);
        LE_SENSOR_LWFT_MX->setMinimumSize(QSize(60, 20));
        LE_SENSOR_LWFT_MX->setMaximumSize(QSize(60, 20));
        LE_SENSOR_LWFT_MX->setFont(font3);
        LE_SENSOR_LWFT_MX->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        LE_SENSOR_LWFT_MX->setReadOnly(true);

        gridLayout_12->addWidget(LE_SENSOR_LWFT_MX, 2, 1, 1, 1);

        LE_SENSOR_RWFT_MY = new QLineEdit(gridLayoutWidget_12);
        LE_SENSOR_RWFT_MY->setObjectName(QStringLiteral("LE_SENSOR_RWFT_MY"));
        LE_SENSOR_RWFT_MY->setEnabled(true);
        sizePolicy1.setHeightForWidth(LE_SENSOR_RWFT_MY->sizePolicy().hasHeightForWidth());
        LE_SENSOR_RWFT_MY->setSizePolicy(sizePolicy1);
        LE_SENSOR_RWFT_MY->setMinimumSize(QSize(60, 20));
        LE_SENSOR_RWFT_MY->setMaximumSize(QSize(60, 20));
        LE_SENSOR_RWFT_MY->setFont(font3);
        LE_SENSOR_RWFT_MY->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        LE_SENSOR_RWFT_MY->setReadOnly(true);

        gridLayout_12->addWidget(LE_SENSOR_RWFT_MY, 1, 2, 1, 1);

        LE_SENSOR_RAFT_MX = new QLineEdit(gridLayoutWidget_12);
        LE_SENSOR_RAFT_MX->setObjectName(QStringLiteral("LE_SENSOR_RAFT_MX"));
        LE_SENSOR_RAFT_MX->setEnabled(true);
        sizePolicy1.setHeightForWidth(LE_SENSOR_RAFT_MX->sizePolicy().hasHeightForWidth());
        LE_SENSOR_RAFT_MX->setSizePolicy(sizePolicy1);
        LE_SENSOR_RAFT_MX->setMinimumSize(QSize(60, 20));
        LE_SENSOR_RAFT_MX->setMaximumSize(QSize(60, 20));
        LE_SENSOR_RAFT_MX->setFont(font3);
        LE_SENSOR_RAFT_MX->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        LE_SENSOR_RAFT_MX->setReadOnly(true);

        gridLayout_12->addWidget(LE_SENSOR_RAFT_MX, 3, 1, 1, 1);

        label_42 = new QLabel(gridLayoutWidget_12);
        label_42->setObjectName(QStringLiteral("label_42"));
        label_42->setEnabled(true);
        sizePolicy1.setHeightForWidth(label_42->sizePolicy().hasHeightForWidth());
        label_42->setSizePolicy(sizePolicy1);
        label_42->setMinimumSize(QSize(60, 20));
        label_42->setMaximumSize(QSize(60, 20));
        label_42->setFont(font3);
        label_42->setAlignment(Qt::AlignCenter);

        gridLayout_12->addWidget(label_42, 0, 6, 1, 1);

        label_31 = new QLabel(gridLayoutWidget_12);
        label_31->setObjectName(QStringLiteral("label_31"));
        label_31->setEnabled(true);
        sizePolicy1.setHeightForWidth(label_31->sizePolicy().hasHeightForWidth());
        label_31->setSizePolicy(sizePolicy1);
        label_31->setMinimumSize(QSize(60, 20));
        label_31->setMaximumSize(QSize(60, 20));
        label_31->setFont(font3);
        label_31->setAlignment(Qt::AlignCenter);

        gridLayout_12->addWidget(label_31, 0, 2, 1, 1);

        LE_SENSOR_RWFT_MX = new QLineEdit(gridLayoutWidget_12);
        LE_SENSOR_RWFT_MX->setObjectName(QStringLiteral("LE_SENSOR_RWFT_MX"));
        LE_SENSOR_RWFT_MX->setEnabled(true);
        sizePolicy1.setHeightForWidth(LE_SENSOR_RWFT_MX->sizePolicy().hasHeightForWidth());
        LE_SENSOR_RWFT_MX->setSizePolicy(sizePolicy1);
        LE_SENSOR_RWFT_MX->setMinimumSize(QSize(60, 20));
        LE_SENSOR_RWFT_MX->setMaximumSize(QSize(60, 20));
        LE_SENSOR_RWFT_MX->setFont(font3);
        LE_SENSOR_RWFT_MX->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        LE_SENSOR_RWFT_MX->setReadOnly(true);

        gridLayout_12->addWidget(LE_SENSOR_RWFT_MX, 1, 1, 1, 1);

        LE_SENSOR_RAFT_MY = new QLineEdit(gridLayoutWidget_12);
        LE_SENSOR_RAFT_MY->setObjectName(QStringLiteral("LE_SENSOR_RAFT_MY"));
        LE_SENSOR_RAFT_MY->setEnabled(true);
        sizePolicy1.setHeightForWidth(LE_SENSOR_RAFT_MY->sizePolicy().hasHeightForWidth());
        LE_SENSOR_RAFT_MY->setSizePolicy(sizePolicy1);
        LE_SENSOR_RAFT_MY->setMinimumSize(QSize(60, 20));
        LE_SENSOR_RAFT_MY->setMaximumSize(QSize(60, 20));
        LE_SENSOR_RAFT_MY->setFont(font3);
        LE_SENSOR_RAFT_MY->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        LE_SENSOR_RAFT_MY->setReadOnly(true);

        gridLayout_12->addWidget(LE_SENSOR_RAFT_MY, 3, 2, 1, 1);

        LE_SENSOR_RWFT_MZ = new QLineEdit(gridLayoutWidget_12);
        LE_SENSOR_RWFT_MZ->setObjectName(QStringLiteral("LE_SENSOR_RWFT_MZ"));
        LE_SENSOR_RWFT_MZ->setEnabled(true);
        sizePolicy1.setHeightForWidth(LE_SENSOR_RWFT_MZ->sizePolicy().hasHeightForWidth());
        LE_SENSOR_RWFT_MZ->setSizePolicy(sizePolicy1);
        LE_SENSOR_RWFT_MZ->setMinimumSize(QSize(60, 20));
        LE_SENSOR_RWFT_MZ->setMaximumSize(QSize(60, 20));
        LE_SENSOR_RWFT_MZ->setFont(font3);
        LE_SENSOR_RWFT_MZ->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        LE_SENSOR_RWFT_MZ->setReadOnly(true);

        gridLayout_12->addWidget(LE_SENSOR_RWFT_MZ, 1, 3, 1, 1);

        LE_SENSOR_RAFT_MZ = new QLineEdit(gridLayoutWidget_12);
        LE_SENSOR_RAFT_MZ->setObjectName(QStringLiteral("LE_SENSOR_RAFT_MZ"));
        LE_SENSOR_RAFT_MZ->setEnabled(true);
        sizePolicy1.setHeightForWidth(LE_SENSOR_RAFT_MZ->sizePolicy().hasHeightForWidth());
        LE_SENSOR_RAFT_MZ->setSizePolicy(sizePolicy1);
        LE_SENSOR_RAFT_MZ->setMinimumSize(QSize(60, 20));
        LE_SENSOR_RAFT_MZ->setMaximumSize(QSize(60, 20));
        LE_SENSOR_RAFT_MZ->setFont(font3);
        LE_SENSOR_RAFT_MZ->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        LE_SENSOR_RAFT_MZ->setReadOnly(true);

        gridLayout_12->addWidget(LE_SENSOR_RAFT_MZ, 3, 3, 1, 1);

        LE_SENSOR_LAFT_MZ = new QLineEdit(gridLayoutWidget_12);
        LE_SENSOR_LAFT_MZ->setObjectName(QStringLiteral("LE_SENSOR_LAFT_MZ"));
        LE_SENSOR_LAFT_MZ->setEnabled(true);
        sizePolicy1.setHeightForWidth(LE_SENSOR_LAFT_MZ->sizePolicy().hasHeightForWidth());
        LE_SENSOR_LAFT_MZ->setSizePolicy(sizePolicy1);
        LE_SENSOR_LAFT_MZ->setMinimumSize(QSize(60, 20));
        LE_SENSOR_LAFT_MZ->setMaximumSize(QSize(60, 20));
        LE_SENSOR_LAFT_MZ->setFont(font3);
        LE_SENSOR_LAFT_MZ->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        LE_SENSOR_LAFT_MZ->setReadOnly(true);

        gridLayout_12->addWidget(LE_SENSOR_LAFT_MZ, 4, 3, 1, 1);

        label_61 = new QLabel(gridLayoutWidget_12);
        label_61->setObjectName(QStringLiteral("label_61"));
        label_61->setEnabled(true);
        sizePolicy1.setHeightForWidth(label_61->sizePolicy().hasHeightForWidth());
        label_61->setSizePolicy(sizePolicy1);
        label_61->setMinimumSize(QSize(60, 20));
        label_61->setMaximumSize(QSize(60, 20));
        label_61->setFont(font3);
        label_61->setAlignment(Qt::AlignCenter);

        gridLayout_12->addWidget(label_61, 0, 4, 1, 1);

        LE_SENSOR_RWFT_FY = new QLineEdit(gridLayoutWidget_12);
        LE_SENSOR_RWFT_FY->setObjectName(QStringLiteral("LE_SENSOR_RWFT_FY"));
        LE_SENSOR_RWFT_FY->setEnabled(true);
        sizePolicy1.setHeightForWidth(LE_SENSOR_RWFT_FY->sizePolicy().hasHeightForWidth());
        LE_SENSOR_RWFT_FY->setSizePolicy(sizePolicy1);
        LE_SENSOR_RWFT_FY->setMinimumSize(QSize(60, 20));
        LE_SENSOR_RWFT_FY->setMaximumSize(QSize(60, 20));
        LE_SENSOR_RWFT_FY->setFont(font3);
        LE_SENSOR_RWFT_FY->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        LE_SENSOR_RWFT_FY->setReadOnly(true);

        gridLayout_12->addWidget(LE_SENSOR_RWFT_FY, 1, 5, 1, 1);

        LE_SENSOR_RWFT_FX = new QLineEdit(gridLayoutWidget_12);
        LE_SENSOR_RWFT_FX->setObjectName(QStringLiteral("LE_SENSOR_RWFT_FX"));
        LE_SENSOR_RWFT_FX->setEnabled(true);
        sizePolicy1.setHeightForWidth(LE_SENSOR_RWFT_FX->sizePolicy().hasHeightForWidth());
        LE_SENSOR_RWFT_FX->setSizePolicy(sizePolicy1);
        LE_SENSOR_RWFT_FX->setMinimumSize(QSize(60, 20));
        LE_SENSOR_RWFT_FX->setMaximumSize(QSize(60, 20));
        LE_SENSOR_RWFT_FX->setFont(font3);
        LE_SENSOR_RWFT_FX->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        LE_SENSOR_RWFT_FX->setReadOnly(true);

        gridLayout_12->addWidget(LE_SENSOR_RWFT_FX, 1, 4, 1, 1);

        LE_SENSOR_LWFT_FY = new QLineEdit(gridLayoutWidget_12);
        LE_SENSOR_LWFT_FY->setObjectName(QStringLiteral("LE_SENSOR_LWFT_FY"));
        LE_SENSOR_LWFT_FY->setEnabled(true);
        sizePolicy1.setHeightForWidth(LE_SENSOR_LWFT_FY->sizePolicy().hasHeightForWidth());
        LE_SENSOR_LWFT_FY->setSizePolicy(sizePolicy1);
        LE_SENSOR_LWFT_FY->setMinimumSize(QSize(60, 20));
        LE_SENSOR_LWFT_FY->setMaximumSize(QSize(60, 20));
        LE_SENSOR_LWFT_FY->setFont(font3);
        LE_SENSOR_LWFT_FY->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        LE_SENSOR_LWFT_FY->setReadOnly(true);

        gridLayout_12->addWidget(LE_SENSOR_LWFT_FY, 2, 5, 1, 1);

        LE_SENSOR_LWFT_FX = new QLineEdit(gridLayoutWidget_12);
        LE_SENSOR_LWFT_FX->setObjectName(QStringLiteral("LE_SENSOR_LWFT_FX"));
        LE_SENSOR_LWFT_FX->setEnabled(true);
        sizePolicy1.setHeightForWidth(LE_SENSOR_LWFT_FX->sizePolicy().hasHeightForWidth());
        LE_SENSOR_LWFT_FX->setSizePolicy(sizePolicy1);
        LE_SENSOR_LWFT_FX->setMinimumSize(QSize(60, 20));
        LE_SENSOR_LWFT_FX->setMaximumSize(QSize(60, 20));
        LE_SENSOR_LWFT_FX->setFont(font3);
        LE_SENSOR_LWFT_FX->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        LE_SENSOR_LWFT_FX->setReadOnly(true);

        gridLayout_12->addWidget(LE_SENSOR_LWFT_FX, 2, 4, 1, 1);

        LE_SENSOR_RAFT_FY = new QLineEdit(gridLayoutWidget_12);
        LE_SENSOR_RAFT_FY->setObjectName(QStringLiteral("LE_SENSOR_RAFT_FY"));
        LE_SENSOR_RAFT_FY->setEnabled(true);
        sizePolicy1.setHeightForWidth(LE_SENSOR_RAFT_FY->sizePolicy().hasHeightForWidth());
        LE_SENSOR_RAFT_FY->setSizePolicy(sizePolicy1);
        LE_SENSOR_RAFT_FY->setMinimumSize(QSize(60, 20));
        LE_SENSOR_RAFT_FY->setMaximumSize(QSize(60, 20));
        LE_SENSOR_RAFT_FY->setFont(font3);
        LE_SENSOR_RAFT_FY->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        LE_SENSOR_RAFT_FY->setReadOnly(true);

        gridLayout_12->addWidget(LE_SENSOR_RAFT_FY, 3, 5, 1, 1);

        LE_SENSOR_RAFT_FX = new QLineEdit(gridLayoutWidget_12);
        LE_SENSOR_RAFT_FX->setObjectName(QStringLiteral("LE_SENSOR_RAFT_FX"));
        LE_SENSOR_RAFT_FX->setEnabled(true);
        sizePolicy1.setHeightForWidth(LE_SENSOR_RAFT_FX->sizePolicy().hasHeightForWidth());
        LE_SENSOR_RAFT_FX->setSizePolicy(sizePolicy1);
        LE_SENSOR_RAFT_FX->setMinimumSize(QSize(60, 20));
        LE_SENSOR_RAFT_FX->setMaximumSize(QSize(60, 20));
        LE_SENSOR_RAFT_FX->setFont(font3);
        LE_SENSOR_RAFT_FX->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        LE_SENSOR_RAFT_FX->setReadOnly(true);

        gridLayout_12->addWidget(LE_SENSOR_RAFT_FX, 3, 4, 1, 1);

        LE_SENSOR_LAFT_FY = new QLineEdit(gridLayoutWidget_12);
        LE_SENSOR_LAFT_FY->setObjectName(QStringLiteral("LE_SENSOR_LAFT_FY"));
        LE_SENSOR_LAFT_FY->setEnabled(true);
        sizePolicy1.setHeightForWidth(LE_SENSOR_LAFT_FY->sizePolicy().hasHeightForWidth());
        LE_SENSOR_LAFT_FY->setSizePolicy(sizePolicy1);
        LE_SENSOR_LAFT_FY->setMinimumSize(QSize(60, 20));
        LE_SENSOR_LAFT_FY->setMaximumSize(QSize(60, 20));
        LE_SENSOR_LAFT_FY->setFont(font3);
        LE_SENSOR_LAFT_FY->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        LE_SENSOR_LAFT_FY->setReadOnly(true);

        gridLayout_12->addWidget(LE_SENSOR_LAFT_FY, 4, 5, 1, 1);

        LE_SENSOR_LAFT_FX = new QLineEdit(gridLayoutWidget_12);
        LE_SENSOR_LAFT_FX->setObjectName(QStringLiteral("LE_SENSOR_LAFT_FX"));
        LE_SENSOR_LAFT_FX->setEnabled(true);
        sizePolicy1.setHeightForWidth(LE_SENSOR_LAFT_FX->sizePolicy().hasHeightForWidth());
        LE_SENSOR_LAFT_FX->setSizePolicy(sizePolicy1);
        LE_SENSOR_LAFT_FX->setMinimumSize(QSize(60, 20));
        LE_SENSOR_LAFT_FX->setMaximumSize(QSize(60, 20));
        LE_SENSOR_LAFT_FX->setFont(font3);
        LE_SENSOR_LAFT_FX->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        LE_SENSOR_LAFT_FX->setReadOnly(true);

        gridLayout_12->addWidget(LE_SENSOR_LAFT_FX, 4, 4, 1, 1);

        LE_SENSOR_LWFT_FZ = new QLineEdit(gridLayoutWidget_12);
        LE_SENSOR_LWFT_FZ->setObjectName(QStringLiteral("LE_SENSOR_LWFT_FZ"));
        LE_SENSOR_LWFT_FZ->setEnabled(true);
        sizePolicy1.setHeightForWidth(LE_SENSOR_LWFT_FZ->sizePolicy().hasHeightForWidth());
        LE_SENSOR_LWFT_FZ->setSizePolicy(sizePolicy1);
        LE_SENSOR_LWFT_FZ->setMinimumSize(QSize(60, 20));
        LE_SENSOR_LWFT_FZ->setMaximumSize(QSize(60, 20));
        LE_SENSOR_LWFT_FZ->setFont(font3);
        LE_SENSOR_LWFT_FZ->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        LE_SENSOR_LWFT_FZ->setReadOnly(true);

        gridLayout_12->addWidget(LE_SENSOR_LWFT_FZ, 2, 6, 1, 1);

        label_62 = new QLabel(groupBox_4);
        label_62->setObjectName(QStringLiteral("label_62"));
        label_62->setEnabled(true);
        label_62->setGeometry(QRect(10, 40, 101, 16));
        QFont font4;
        font4.setPointSize(9);
        font4.setBold(true);
        font4.setWeight(75);
        label_62->setFont(font4);
        label_62->setAlignment(Qt::AlignLeading|Qt::AlignLeft|Qt::AlignVCenter);
        BTN_SENSOR_FT_NULL = new QPushButton(groupBox_4);
        BTN_SENSOR_FT_NULL->setObjectName(QStringLiteral("BTN_SENSOR_FT_NULL"));
        BTN_SENSOR_FT_NULL->setEnabled(true);
        BTN_SENSOR_FT_NULL->setGeometry(QRect(310, 20, 71, 31));
        BTN_SENSOR_FT_NULL->setFont(font3);

        retranslateUi(ModelDialog);

        QMetaObject::connectSlotsByName(ModelDialog);
    } // setupUi

    void retranslateUi(QDialog *ModelDialog)
    {
        ModelDialog->setWindowTitle(QApplication::translate("ModelDialog", "Dialog", Q_NULLPTR));
        groupBox_2->setTitle(QApplication::translate("ModelDialog", "Control", Q_NULLPTR));
        CB_USE_ENCODER->setText(QApplication::translate("ModelDialog", "Encoder", Q_NULLPTR));
        CB_SHOW_FLOOR->setText(QApplication::translate("ModelDialog", "Floor", Q_NULLPTR));
        CB_SHOW_FT->setText(QApplication::translate("ModelDialog", "FT Sensor", Q_NULLPTR));
        groupBox->setTitle(QApplication::translate("ModelDialog", "Camera", Q_NULLPTR));
        BT_CamLeft->setText(QApplication::translate("ModelDialog", "Left", Q_NULLPTR));
        BT_CamFront->setText(QApplication::translate("ModelDialog", "Front", Q_NULLPTR));
        BT_CamRight->setText(QApplication::translate("ModelDialog", "Right", Q_NULLPTR));
        groupBox_3->setTitle(QApplication::translate("ModelDialog", "voice player", Q_NULLPTR));
        BTN_SOUND_1->setText(QApplication::translate("ModelDialog", "Greeting", Q_NULLPTR));
        BTN_SOUND_2->setText(QApplication::translate("ModelDialog", "hello", Q_NULLPTR));
        BTN_SOUND_3->setText(QApplication::translate("ModelDialog", "name", Q_NULLPTR));
        BTN_SOUND_4->setText(QApplication::translate("ModelDialog", "hahaha", Q_NULLPTR));
        BTN_SOUND_5->setText(QApplication::translate("ModelDialog", "sad", Q_NULLPTR));
        BTN_SOUND_6->setText(QApplication::translate("ModelDialog", "Of course", Q_NULLPTR));
        BTN_SOUND_15->setText(QApplication::translate("ModelDialog", "finger", Q_NULLPTR));
        BTN_SOUND_16->setText(QApplication::translate("ModelDialog", "cute", Q_NULLPTR));
        BTN_SOUND_17->setText(QApplication::translate("ModelDialog", "love", Q_NULLPTR));
        BTN_SOUND_18->setText(QApplication::translate("ModelDialog", "hoot", Q_NULLPTR));
        BTN_SOUND_19->setText(QApplication::translate("ModelDialog", "bye", Q_NULLPTR));
        BTN_SOUND_20->setText(QApplication::translate("ModelDialog", "1stRobot", Q_NULLPTR));
        BTN_SOUND_21->setText(QApplication::translate("ModelDialog", "closer", Q_NULLPTR));
        BTN_SOUND_22->setText(QApplication::translate("ModelDialog", "curious", Q_NULLPTR));
        BTN_SOUND_23->setText(QApplication::translate("ModelDialog", "dance", Q_NULLPTR));
        BTN_SOUND_24->setText(QApplication::translate("ModelDialog", "enjoy", Q_NULLPTR));
        BTN_SOUND_25->setText(QApplication::translate("ModelDialog", "from", Q_NULLPTR));
        BTN_SOUND_26->setText(QApplication::translate("ModelDialog", "okay", Q_NULLPTR));
        BTN_SOUND_27->setText(QApplication::translate("ModelDialog", "shake", Q_NULLPTR));
        BTN_SOUND_28->setText(QApplication::translate("ModelDialog", "shake_gently", Q_NULLPTR));
        BTN_SOUND_29->setText(QApplication::translate("ModelDialog", "can't do", Q_NULLPTR));
        BTN_SOUND_30->setText(QApplication::translate("ModelDialog", "walk", Q_NULLPTR));
        BTN_SOUND_31->setText(QApplication::translate("ModelDialog", "welcome", Q_NULLPTR));
        BTN_SOUND_32->setText(QApplication::translate("ModelDialog", "don't know", Q_NULLPTR));
        BTN_SOUND_33->setText(QApplication::translate("ModelDialog", "eat", Q_NULLPTR));
        BTN_SOUND_34->setText(QApplication::translate("ModelDialog", "feeling", Q_NULLPTR));
        BTN_SOUND_35->setText(QApplication::translate("ModelDialog", "cheer", Q_NULLPTR));
        BTN_SOUND_36->setText(QApplication::translate("ModelDialog", "fighting", Q_NULLPTR));
        BTN_SOUND_37->setText(QApplication::translate("ModelDialog", "cold", Q_NULLPTR));
        BTN_SOUND_38->setText(QApplication::translate("ModelDialog", "picture", Q_NULLPTR));
        BTN_SOUND_39->setText(QApplication::translate("ModelDialog", "hero", Q_NULLPTR));
        BTN_SOUND_40->setText(QApplication::translate("ModelDialog", "nerf this", Q_NULLPTR));
        BTN_SOUND_41->setText(QApplication::translate("ModelDialog", "my skill time", Q_NULLPTR));
        BTN_SOUND_42->setText(QApplication::translate("ModelDialog", "online", Q_NULLPTR));
        BTN_SOUND_43->setText(QApplication::translate("ModelDialog", "yes", Q_NULLPTR));
        BTN_SOUND_44->setText(QApplication::translate("ModelDialog", "no", Q_NULLPTR));
        BTN_SOUND_45->setText(QApplication::translate("ModelDialog", "move", Q_NULLPTR));
        BTN_SOUND_46->setText(QApplication::translate("ModelDialog", "move left", Q_NULLPTR));
        BTN_SOUND_47->setText(QApplication::translate("ModelDialog", "move right", Q_NULLPTR));
        BTN_SOUND_88->setText(QApplication::translate("ModelDialog", "Meet you", Q_NULLPTR));
        BTN_SOUND_89->setText(QApplication::translate("ModelDialog", "name", Q_NULLPTR));
        BTN_SOUND_90->setText(QApplication::translate("ModelDialog", "Photo", Q_NULLPTR));
        BTN_SOUND_92->setText(QApplication::translate("ModelDialog", "name", Q_NULLPTR));
        BTN_SOUND_93->setText(QApplication::translate("ModelDialog", "Photo", Q_NULLPTR));
        BTN_SOUND_95->setText(QApplication::translate("ModelDialog", "name", Q_NULLPTR));
        BTN_SOUND_96->setText(QApplication::translate("ModelDialog", "Photo", Q_NULLPTR));
        BTN_SOUND_97->setText(QApplication::translate("ModelDialog", "Meet you", Q_NULLPTR));
        BTN_SOUND_99->setText(QApplication::translate("ModelDialog", "Photo", Q_NULLPTR));
        BTN_SOUND_101->setText(QApplication::translate("ModelDialog", "name", Q_NULLPTR));
        BTN_SOUND_102->setText(QApplication::translate("ModelDialog", "Photo", Q_NULLPTR));
        label->setText(QApplication::translate("ModelDialog", "English", Q_NULLPTR));
        label_2->setText(QApplication::translate("ModelDialog", "French", Q_NULLPTR));
        label_3->setText(QApplication::translate("ModelDialog", "Spanish", Q_NULLPTR));
        label_4->setText(QApplication::translate("ModelDialog", "Chinese", Q_NULLPTR));
        label_5->setText(QApplication::translate("ModelDialog", "Japanese", Q_NULLPTR));
        rad_kor->setText(QApplication::translate("ModelDialog", "Korean", Q_NULLPTR));
        rad_eng->setText(QApplication::translate("ModelDialog", "English", Q_NULLPTR));
        BTN_SOUND_104->setText(QApplication::translate("ModelDialog", "name", Q_NULLPTR));
        BTN_SOUND_105->setText(QApplication::translate("ModelDialog", "Photo", Q_NULLPTR));
        BTN_SOUND_91->setText(QApplication::translate("ModelDialog", "Meet you", Q_NULLPTR));
        BTN_SOUND_94->setText(QApplication::translate("ModelDialog", "Meet you", Q_NULLPTR));
        BTN_SOUND_98->setText(QApplication::translate("ModelDialog", "name", Q_NULLPTR));
        BTN_SOUND_100->setText(QApplication::translate("ModelDialog", "Meet you", Q_NULLPTR));
        BTN_SOUND_103->setText(QApplication::translate("ModelDialog", "Meet you", Q_NULLPTR));
        BTN_SOUND_7->setText(QApplication::translate("ModelDialog", "1millon", Q_NULLPTR));
        label_6->setText(QApplication::translate("ModelDialog", "Korean", Q_NULLPTR));
        BTN_ENABLE->setText(QApplication::translate("ModelDialog", "Enable", Q_NULLPTR));
        BTN_DISABLE->setText(QApplication::translate("ModelDialog", "Disable", Q_NULLPTR));
        BTN_wa->setText(QApplication::translate("ModelDialog", "waaa!", Q_NULLPTR));
        BTN_oh->setText(QApplication::translate("ModelDialog", "oh....", Q_NULLPTR));
        BTN_QUICK_STOP->setText(QApplication::translate("ModelDialog", "QUICK STOP", Q_NULLPTR));
        groupBox_4->setTitle(QApplication::translate("ModelDialog", "Sensor Data", Q_NULLPTR));
        label_43->setText(QApplication::translate("ModelDialog", "Fy", Q_NULLPTR));
        label_32->setText(QApplication::translate("ModelDialog", "Mz", Q_NULLPTR));
        label_44->setText(QApplication::translate("ModelDialog", "Mx", Q_NULLPTR));
        label_45->setText(QApplication::translate("ModelDialog", "RWFT", Q_NULLPTR));
        label_39->setText(QApplication::translate("ModelDialog", "LWFT", Q_NULLPTR));
        label_47->setText(QApplication::translate("ModelDialog", "RAFT", Q_NULLPTR));
        label_48->setText(QApplication::translate("ModelDialog", "LAFT", Q_NULLPTR));
        label_42->setText(QApplication::translate("ModelDialog", "Fz", Q_NULLPTR));
        label_31->setText(QApplication::translate("ModelDialog", "My", Q_NULLPTR));
        label_61->setText(QApplication::translate("ModelDialog", "Fx", Q_NULLPTR));
        label_62->setText(QApplication::translate("ModelDialog", "FT(Mx,My,Fz)", Q_NULLPTR));
        BTN_SENSOR_FT_NULL->setText(QApplication::translate("ModelDialog", "FT NULL", Q_NULLPTR));
    } // retranslateUi

};

namespace Ui {
    class ModelDialog: public Ui_ModelDialog {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MODELDIALOG_H
