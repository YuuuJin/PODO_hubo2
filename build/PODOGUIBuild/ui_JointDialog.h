/********************************************************************************
** Form generated from reading UI file 'JointDialog.ui'
**
** Created by: Qt User Interface Compiler version 5.8.0
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_JOINTDIALOG_H
#define UI_JOINTDIALOG_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QDialog>
#include <QtWidgets/QFrame>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QRadioButton>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_JointDialog
{
public:
    QLabel *label;
    QGroupBox *groupBox;
    QWidget *gridLayoutWidget;
    QGridLayout *gridLayout;
    QLineEdit *LE_JOINT_RWP;
    QLabel *label_5;
    QLabel *label_6;
    QLabel *label_3;
    QLineEdit *LE_JOINT_RSP;
    QLabel *label_2;
    QLabel *label_4;
    QLineEdit *LE_JOINT_RSY;
    QLineEdit *LE_JOINT_RSR;
    QLineEdit *LE_JOINT_REB;
    QLabel *label_7;
    QLineEdit *LE_JOINT_RWY;
    QWidget *horizontalLayoutWidget_2;
    QHBoxLayout *horizontalLayout_2;
    QRadioButton *RB_JOINT_REFERENCE;
    QRadioButton *RB_JOINT_ENCODER;
    QWidget *gridLayoutWidget_2;
    QGridLayout *gridLayout_2;
    QLabel *label_8;
    QLabel *label_9;
    QLineEdit *LE_JOINT_LSP;
    QLabel *label_10;
    QLabel *label_11;
    QLineEdit *LE_JOINT_LSR;
    QLabel *label_12;
    QLabel *label_13;
    QLineEdit *LE_JOINT_LSY;
    QLineEdit *LE_JOINT_LEB;
    QLineEdit *LE_JOINT_LWY;
    QLineEdit *LE_JOINT_LWP;
    QWidget *gridLayoutWidget_4;
    QGridLayout *gridLayout_4;
    QLineEdit *LE_JOINT_RAR;
    QLabel *label_14;
    QLabel *label_19;
    QLabel *label_20;
    QLineEdit *LE_JOINT_RHY;
    QLabel *label_21;
    QLabel *label_22;
    QLineEdit *LE_JOINT_RHP;
    QLineEdit *LE_JOINT_RHR;
    QLineEdit *LE_JOINT_RKN;
    QLabel *label_23;
    QLineEdit *LE_JOINT_RAP;
    QWidget *gridLayoutWidget_5;
    QGridLayout *gridLayout_6;
    QLineEdit *LE_JOINT_LAR;
    QLabel *label_30;
    QLabel *label_31;
    QLabel *label_32;
    QLineEdit *LE_JOINT_LHY;
    QLabel *label_33;
    QLabel *label_34;
    QLineEdit *LE_JOINT_LHP;
    QLineEdit *LE_JOINT_LHR;
    QLineEdit *LE_JOINT_LKN;
    QLabel *label_35;
    QLineEdit *LE_JOINT_LAP;
    QWidget *gridLayoutWidget_7;
    QGridLayout *gridLayout_8;
    QLabel *label_29;
    QLabel *label_36;
    QLineEdit *LE_JOINT_NK2;
    QLineEdit *LE_JOINT_NK1;
    QWidget *gridLayoutWidget_8;
    QGridLayout *gridLayout_9;
    QLabel *label_37;
    QLabel *label_38;
    QLineEdit *LE_JOINT_WST;
    QLineEdit *LE_JOINT_NKY;
    QWidget *gridLayoutWidget_9;
    QGridLayout *gridLayout_7;
    QLineEdit *LE_JOINT_RF3;
    QLineEdit *LE_JOINT_RF2;
    QLabel *label_28;
    QLabel *label_26;
    QLineEdit *LE_JOINT_RF4;
    QLabel *label_39;
    QLabel *label_40;
    QLineEdit *LE_JOINT_RF5;
    QLabel *label_79;
    QLineEdit *LE_JOINT_RF1;
    QWidget *gridLayoutWidget_10;
    QGridLayout *gridLayout_10;
    QLineEdit *LE_JOINT_LF3;
    QLineEdit *LE_JOINT_LF2;
    QLabel *label_41;
    QLabel *label_42;
    QLineEdit *LE_JOINT_LF4;
    QLabel *label_43;
    QLabel *label_44;
    QLineEdit *LE_JOINT_LF5;
    QLabel *label_80;
    QLineEdit *LE_JOINT_LF1;
    QPushButton *BTN_ENC_ENABLE;
    QPushButton *BTN_ENC_DISABLE;
    QFrame *line;

    void setupUi(QDialog *JointDialog)
    {
        if (JointDialog->objectName().isEmpty())
            JointDialog->setObjectName(QStringLiteral("JointDialog"));
        JointDialog->resize(320, 725);
        label = new QLabel(JointDialog);
        label->setObjectName(QStringLiteral("label"));
        label->setEnabled(true);
        label->setGeometry(QRect(10, 0, 111, 31));
        QFont font;
        font.setPointSize(12);
        font.setBold(true);
        font.setWeight(75);
        label->setFont(font);
        groupBox = new QGroupBox(JointDialog);
        groupBox->setObjectName(QStringLiteral("groupBox"));
        groupBox->setEnabled(true);
        groupBox->setGeometry(QRect(10, 50, 301, 661));
        QFont font1;
        font1.setBold(true);
        font1.setWeight(75);
        groupBox->setFont(font1);
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
        gridLayoutWidget = new QWidget(groupBox);
        gridLayoutWidget->setObjectName(QStringLiteral("gridLayoutWidget"));
        gridLayoutWidget->setEnabled(true);
        gridLayoutWidget->setGeometry(QRect(10, 160, 131, 161));
        gridLayout = new QGridLayout(gridLayoutWidget);
        gridLayout->setObjectName(QStringLiteral("gridLayout"));
        gridLayout->setContentsMargins(0, 0, 0, 0);
        LE_JOINT_RWP = new QLineEdit(gridLayoutWidget);
        LE_JOINT_RWP->setObjectName(QStringLiteral("LE_JOINT_RWP"));
        LE_JOINT_RWP->setEnabled(true);
        QSizePolicy sizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(LE_JOINT_RWP->sizePolicy().hasHeightForWidth());
        LE_JOINT_RWP->setSizePolicy(sizePolicy);
        LE_JOINT_RWP->setMinimumSize(QSize(70, 20));
        LE_JOINT_RWP->setMaximumSize(QSize(70, 20));
        QFont font2;
        font2.setFamily(QStringLiteral("Serif"));
        font2.setPointSize(9);
        LE_JOINT_RWP->setFont(font2);
        LE_JOINT_RWP->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        LE_JOINT_RWP->setReadOnly(true);

        gridLayout->addWidget(LE_JOINT_RWP, 5, 1, 1, 1);

        label_5 = new QLabel(gridLayoutWidget);
        label_5->setObjectName(QStringLiteral("label_5"));
        label_5->setEnabled(true);
        sizePolicy.setHeightForWidth(label_5->sizePolicy().hasHeightForWidth());
        label_5->setSizePolicy(sizePolicy);
        label_5->setMinimumSize(QSize(50, 20));
        label_5->setMaximumSize(QSize(50, 20));
        label_5->setFont(font2);
        label_5->setAlignment(Qt::AlignCenter);

        gridLayout->addWidget(label_5, 4, 0, 1, 1);

        label_6 = new QLabel(gridLayoutWidget);
        label_6->setObjectName(QStringLiteral("label_6"));
        label_6->setEnabled(true);
        sizePolicy.setHeightForWidth(label_6->sizePolicy().hasHeightForWidth());
        label_6->setSizePolicy(sizePolicy);
        label_6->setMinimumSize(QSize(50, 20));
        label_6->setMaximumSize(QSize(50, 20));
        label_6->setFont(font2);
        label_6->setAlignment(Qt::AlignCenter);

        gridLayout->addWidget(label_6, 5, 0, 1, 1);

        label_3 = new QLabel(gridLayoutWidget);
        label_3->setObjectName(QStringLiteral("label_3"));
        label_3->setEnabled(true);
        sizePolicy.setHeightForWidth(label_3->sizePolicy().hasHeightForWidth());
        label_3->setSizePolicy(sizePolicy);
        label_3->setMinimumSize(QSize(50, 20));
        label_3->setMaximumSize(QSize(50, 20));
        label_3->setFont(font2);
        label_3->setAlignment(Qt::AlignCenter);

        gridLayout->addWidget(label_3, 2, 0, 1, 1);

        LE_JOINT_RSP = new QLineEdit(gridLayoutWidget);
        LE_JOINT_RSP->setObjectName(QStringLiteral("LE_JOINT_RSP"));
        LE_JOINT_RSP->setEnabled(true);
        sizePolicy.setHeightForWidth(LE_JOINT_RSP->sizePolicy().hasHeightForWidth());
        LE_JOINT_RSP->setSizePolicy(sizePolicy);
        LE_JOINT_RSP->setMinimumSize(QSize(70, 20));
        LE_JOINT_RSP->setMaximumSize(QSize(70, 20));
        LE_JOINT_RSP->setFont(font2);
        LE_JOINT_RSP->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        LE_JOINT_RSP->setReadOnly(true);

        gridLayout->addWidget(LE_JOINT_RSP, 0, 1, 1, 1);

        label_2 = new QLabel(gridLayoutWidget);
        label_2->setObjectName(QStringLiteral("label_2"));
        label_2->setEnabled(true);
        sizePolicy.setHeightForWidth(label_2->sizePolicy().hasHeightForWidth());
        label_2->setSizePolicy(sizePolicy);
        label_2->setMinimumSize(QSize(50, 20));
        label_2->setMaximumSize(QSize(50, 20));
        label_2->setFont(font2);
        label_2->setAlignment(Qt::AlignCenter);

        gridLayout->addWidget(label_2, 0, 0, 1, 1);

        label_4 = new QLabel(gridLayoutWidget);
        label_4->setObjectName(QStringLiteral("label_4"));
        label_4->setEnabled(true);
        sizePolicy.setHeightForWidth(label_4->sizePolicy().hasHeightForWidth());
        label_4->setSizePolicy(sizePolicy);
        label_4->setMinimumSize(QSize(50, 20));
        label_4->setMaximumSize(QSize(50, 20));
        label_4->setFont(font2);
        label_4->setAlignment(Qt::AlignCenter);

        gridLayout->addWidget(label_4, 3, 0, 1, 1);

        LE_JOINT_RSY = new QLineEdit(gridLayoutWidget);
        LE_JOINT_RSY->setObjectName(QStringLiteral("LE_JOINT_RSY"));
        LE_JOINT_RSY->setEnabled(true);
        sizePolicy.setHeightForWidth(LE_JOINT_RSY->sizePolicy().hasHeightForWidth());
        LE_JOINT_RSY->setSizePolicy(sizePolicy);
        LE_JOINT_RSY->setMinimumSize(QSize(70, 20));
        LE_JOINT_RSY->setMaximumSize(QSize(70, 20));
        QFont font3;
        font3.setFamily(QStringLiteral("Serif"));
        font3.setPointSize(8);
        LE_JOINT_RSY->setFont(font3);
        LE_JOINT_RSY->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        LE_JOINT_RSY->setReadOnly(true);

        gridLayout->addWidget(LE_JOINT_RSY, 2, 1, 1, 1);

        LE_JOINT_RSR = new QLineEdit(gridLayoutWidget);
        LE_JOINT_RSR->setObjectName(QStringLiteral("LE_JOINT_RSR"));
        LE_JOINT_RSR->setEnabled(true);
        sizePolicy.setHeightForWidth(LE_JOINT_RSR->sizePolicy().hasHeightForWidth());
        LE_JOINT_RSR->setSizePolicy(sizePolicy);
        LE_JOINT_RSR->setMinimumSize(QSize(70, 20));
        LE_JOINT_RSR->setMaximumSize(QSize(70, 20));
        LE_JOINT_RSR->setFont(font2);
        LE_JOINT_RSR->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        LE_JOINT_RSR->setReadOnly(true);

        gridLayout->addWidget(LE_JOINT_RSR, 1, 1, 1, 1);

        LE_JOINT_REB = new QLineEdit(gridLayoutWidget);
        LE_JOINT_REB->setObjectName(QStringLiteral("LE_JOINT_REB"));
        LE_JOINT_REB->setEnabled(true);
        sizePolicy.setHeightForWidth(LE_JOINT_REB->sizePolicy().hasHeightForWidth());
        LE_JOINT_REB->setSizePolicy(sizePolicy);
        LE_JOINT_REB->setMinimumSize(QSize(70, 20));
        LE_JOINT_REB->setMaximumSize(QSize(70, 20));
        LE_JOINT_REB->setFont(font2);
        LE_JOINT_REB->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        LE_JOINT_REB->setReadOnly(true);

        gridLayout->addWidget(LE_JOINT_REB, 3, 1, 1, 1);

        label_7 = new QLabel(gridLayoutWidget);
        label_7->setObjectName(QStringLiteral("label_7"));
        label_7->setEnabled(true);
        sizePolicy.setHeightForWidth(label_7->sizePolicy().hasHeightForWidth());
        label_7->setSizePolicy(sizePolicy);
        label_7->setMinimumSize(QSize(50, 20));
        label_7->setMaximumSize(QSize(50, 20));
        label_7->setFont(font2);
        label_7->setAlignment(Qt::AlignCenter);

        gridLayout->addWidget(label_7, 1, 0, 1, 1);

        LE_JOINT_RWY = new QLineEdit(gridLayoutWidget);
        LE_JOINT_RWY->setObjectName(QStringLiteral("LE_JOINT_RWY"));
        LE_JOINT_RWY->setEnabled(true);
        sizePolicy.setHeightForWidth(LE_JOINT_RWY->sizePolicy().hasHeightForWidth());
        LE_JOINT_RWY->setSizePolicy(sizePolicy);
        LE_JOINT_RWY->setMinimumSize(QSize(70, 20));
        LE_JOINT_RWY->setMaximumSize(QSize(70, 20));
        LE_JOINT_RWY->setFont(font2);
        LE_JOINT_RWY->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        LE_JOINT_RWY->setReadOnly(true);

        gridLayout->addWidget(LE_JOINT_RWY, 4, 1, 1, 1);

        horizontalLayoutWidget_2 = new QWidget(groupBox);
        horizontalLayoutWidget_2->setObjectName(QStringLiteral("horizontalLayoutWidget_2"));
        horizontalLayoutWidget_2->setEnabled(true);
        horizontalLayoutWidget_2->setGeometry(QRect(10, 30, 281, 31));
        horizontalLayout_2 = new QHBoxLayout(horizontalLayoutWidget_2);
        horizontalLayout_2->setObjectName(QStringLiteral("horizontalLayout_2"));
        horizontalLayout_2->setContentsMargins(0, 0, 0, 0);
        RB_JOINT_REFERENCE = new QRadioButton(horizontalLayoutWidget_2);
        RB_JOINT_REFERENCE->setObjectName(QStringLiteral("RB_JOINT_REFERENCE"));
        RB_JOINT_REFERENCE->setEnabled(true);
        QFont font4;
        font4.setPointSize(9);
        RB_JOINT_REFERENCE->setFont(font4);
        RB_JOINT_REFERENCE->setChecked(true);

        horizontalLayout_2->addWidget(RB_JOINT_REFERENCE);

        RB_JOINT_ENCODER = new QRadioButton(horizontalLayoutWidget_2);
        RB_JOINT_ENCODER->setObjectName(QStringLiteral("RB_JOINT_ENCODER"));
        RB_JOINT_ENCODER->setEnabled(true);
        RB_JOINT_ENCODER->setFont(font4);

        horizontalLayout_2->addWidget(RB_JOINT_ENCODER);

        gridLayoutWidget_2 = new QWidget(groupBox);
        gridLayoutWidget_2->setObjectName(QStringLiteral("gridLayoutWidget_2"));
        gridLayoutWidget_2->setEnabled(true);
        gridLayoutWidget_2->setGeometry(QRect(160, 160, 128, 161));
        gridLayout_2 = new QGridLayout(gridLayoutWidget_2);
        gridLayout_2->setObjectName(QStringLiteral("gridLayout_2"));
        gridLayout_2->setContentsMargins(0, 0, 0, 0);
        label_8 = new QLabel(gridLayoutWidget_2);
        label_8->setObjectName(QStringLiteral("label_8"));
        label_8->setEnabled(true);
        sizePolicy.setHeightForWidth(label_8->sizePolicy().hasHeightForWidth());
        label_8->setSizePolicy(sizePolicy);
        label_8->setMinimumSize(QSize(50, 20));
        label_8->setMaximumSize(QSize(50, 20));
        label_8->setFont(font2);
        label_8->setAlignment(Qt::AlignCenter);

        gridLayout_2->addWidget(label_8, 0, 0, 1, 1);

        label_9 = new QLabel(gridLayoutWidget_2);
        label_9->setObjectName(QStringLiteral("label_9"));
        label_9->setEnabled(true);
        sizePolicy.setHeightForWidth(label_9->sizePolicy().hasHeightForWidth());
        label_9->setSizePolicy(sizePolicy);
        label_9->setMinimumSize(QSize(50, 20));
        label_9->setMaximumSize(QSize(50, 20));
        label_9->setFont(font2);
        label_9->setAlignment(Qt::AlignCenter);

        gridLayout_2->addWidget(label_9, 3, 0, 1, 1);

        LE_JOINT_LSP = new QLineEdit(gridLayoutWidget_2);
        LE_JOINT_LSP->setObjectName(QStringLiteral("LE_JOINT_LSP"));
        LE_JOINT_LSP->setEnabled(true);
        sizePolicy.setHeightForWidth(LE_JOINT_LSP->sizePolicy().hasHeightForWidth());
        LE_JOINT_LSP->setSizePolicy(sizePolicy);
        LE_JOINT_LSP->setMinimumSize(QSize(70, 20));
        LE_JOINT_LSP->setMaximumSize(QSize(70, 20));
        LE_JOINT_LSP->setFont(font2);
        LE_JOINT_LSP->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        LE_JOINT_LSP->setReadOnly(true);

        gridLayout_2->addWidget(LE_JOINT_LSP, 0, 1, 1, 1);

        label_10 = new QLabel(gridLayoutWidget_2);
        label_10->setObjectName(QStringLiteral("label_10"));
        label_10->setEnabled(true);
        sizePolicy.setHeightForWidth(label_10->sizePolicy().hasHeightForWidth());
        label_10->setSizePolicy(sizePolicy);
        label_10->setMinimumSize(QSize(50, 20));
        label_10->setMaximumSize(QSize(50, 20));
        label_10->setFont(font2);
        label_10->setAlignment(Qt::AlignCenter);

        gridLayout_2->addWidget(label_10, 5, 0, 1, 1);

        label_11 = new QLabel(gridLayoutWidget_2);
        label_11->setObjectName(QStringLiteral("label_11"));
        label_11->setEnabled(true);
        sizePolicy.setHeightForWidth(label_11->sizePolicy().hasHeightForWidth());
        label_11->setSizePolicy(sizePolicy);
        label_11->setMinimumSize(QSize(50, 20));
        label_11->setMaximumSize(QSize(50, 20));
        label_11->setFont(font2);
        label_11->setAlignment(Qt::AlignCenter);

        gridLayout_2->addWidget(label_11, 4, 0, 1, 1);

        LE_JOINT_LSR = new QLineEdit(gridLayoutWidget_2);
        LE_JOINT_LSR->setObjectName(QStringLiteral("LE_JOINT_LSR"));
        LE_JOINT_LSR->setEnabled(true);
        sizePolicy.setHeightForWidth(LE_JOINT_LSR->sizePolicy().hasHeightForWidth());
        LE_JOINT_LSR->setSizePolicy(sizePolicy);
        LE_JOINT_LSR->setMinimumSize(QSize(70, 20));
        LE_JOINT_LSR->setMaximumSize(QSize(70, 20));
        LE_JOINT_LSR->setFont(font2);
        LE_JOINT_LSR->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        LE_JOINT_LSR->setReadOnly(true);

        gridLayout_2->addWidget(LE_JOINT_LSR, 1, 1, 1, 1);

        label_12 = new QLabel(gridLayoutWidget_2);
        label_12->setObjectName(QStringLiteral("label_12"));
        label_12->setEnabled(true);
        sizePolicy.setHeightForWidth(label_12->sizePolicy().hasHeightForWidth());
        label_12->setSizePolicy(sizePolicy);
        label_12->setMinimumSize(QSize(50, 20));
        label_12->setMaximumSize(QSize(50, 20));
        label_12->setFont(font2);
        label_12->setAlignment(Qt::AlignCenter);

        gridLayout_2->addWidget(label_12, 2, 0, 1, 1);

        label_13 = new QLabel(gridLayoutWidget_2);
        label_13->setObjectName(QStringLiteral("label_13"));
        label_13->setEnabled(true);
        sizePolicy.setHeightForWidth(label_13->sizePolicy().hasHeightForWidth());
        label_13->setSizePolicy(sizePolicy);
        label_13->setMinimumSize(QSize(50, 20));
        label_13->setMaximumSize(QSize(50, 20));
        label_13->setFont(font2);
        label_13->setAlignment(Qt::AlignCenter);

        gridLayout_2->addWidget(label_13, 1, 0, 1, 1);

        LE_JOINT_LSY = new QLineEdit(gridLayoutWidget_2);
        LE_JOINT_LSY->setObjectName(QStringLiteral("LE_JOINT_LSY"));
        LE_JOINT_LSY->setEnabled(true);
        sizePolicy.setHeightForWidth(LE_JOINT_LSY->sizePolicy().hasHeightForWidth());
        LE_JOINT_LSY->setSizePolicy(sizePolicy);
        LE_JOINT_LSY->setMinimumSize(QSize(70, 20));
        LE_JOINT_LSY->setMaximumSize(QSize(70, 20));
        LE_JOINT_LSY->setFont(font2);
        LE_JOINT_LSY->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        LE_JOINT_LSY->setReadOnly(true);

        gridLayout_2->addWidget(LE_JOINT_LSY, 2, 1, 1, 1);

        LE_JOINT_LEB = new QLineEdit(gridLayoutWidget_2);
        LE_JOINT_LEB->setObjectName(QStringLiteral("LE_JOINT_LEB"));
        LE_JOINT_LEB->setEnabled(true);
        sizePolicy.setHeightForWidth(LE_JOINT_LEB->sizePolicy().hasHeightForWidth());
        LE_JOINT_LEB->setSizePolicy(sizePolicy);
        LE_JOINT_LEB->setMinimumSize(QSize(70, 20));
        LE_JOINT_LEB->setMaximumSize(QSize(70, 20));
        LE_JOINT_LEB->setFont(font2);
        LE_JOINT_LEB->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        LE_JOINT_LEB->setReadOnly(true);

        gridLayout_2->addWidget(LE_JOINT_LEB, 3, 1, 1, 1);

        LE_JOINT_LWY = new QLineEdit(gridLayoutWidget_2);
        LE_JOINT_LWY->setObjectName(QStringLiteral("LE_JOINT_LWY"));
        LE_JOINT_LWY->setEnabled(true);
        sizePolicy.setHeightForWidth(LE_JOINT_LWY->sizePolicy().hasHeightForWidth());
        LE_JOINT_LWY->setSizePolicy(sizePolicy);
        LE_JOINT_LWY->setMinimumSize(QSize(70, 20));
        LE_JOINT_LWY->setMaximumSize(QSize(70, 20));
        LE_JOINT_LWY->setFont(font2);
        LE_JOINT_LWY->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        LE_JOINT_LWY->setReadOnly(true);

        gridLayout_2->addWidget(LE_JOINT_LWY, 4, 1, 1, 1);

        LE_JOINT_LWP = new QLineEdit(gridLayoutWidget_2);
        LE_JOINT_LWP->setObjectName(QStringLiteral("LE_JOINT_LWP"));
        LE_JOINT_LWP->setEnabled(true);
        sizePolicy.setHeightForWidth(LE_JOINT_LWP->sizePolicy().hasHeightForWidth());
        LE_JOINT_LWP->setSizePolicy(sizePolicy);
        LE_JOINT_LWP->setMinimumSize(QSize(70, 20));
        LE_JOINT_LWP->setMaximumSize(QSize(70, 20));
        LE_JOINT_LWP->setFont(font2);
        LE_JOINT_LWP->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        LE_JOINT_LWP->setReadOnly(true);

        gridLayout_2->addWidget(LE_JOINT_LWP, 5, 1, 1, 1);

        gridLayoutWidget_4 = new QWidget(groupBox);
        gridLayoutWidget_4->setObjectName(QStringLiteral("gridLayoutWidget_4"));
        gridLayoutWidget_4->setEnabled(true);
        gridLayoutWidget_4->setGeometry(QRect(10, 480, 131, 161));
        gridLayout_4 = new QGridLayout(gridLayoutWidget_4);
        gridLayout_4->setObjectName(QStringLiteral("gridLayout_4"));
        gridLayout_4->setContentsMargins(0, 0, 0, 0);
        LE_JOINT_RAR = new QLineEdit(gridLayoutWidget_4);
        LE_JOINT_RAR->setObjectName(QStringLiteral("LE_JOINT_RAR"));
        LE_JOINT_RAR->setEnabled(true);
        sizePolicy.setHeightForWidth(LE_JOINT_RAR->sizePolicy().hasHeightForWidth());
        LE_JOINT_RAR->setSizePolicy(sizePolicy);
        LE_JOINT_RAR->setMinimumSize(QSize(70, 20));
        LE_JOINT_RAR->setMaximumSize(QSize(70, 20));
        LE_JOINT_RAR->setFont(font2);
        LE_JOINT_RAR->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        LE_JOINT_RAR->setReadOnly(true);

        gridLayout_4->addWidget(LE_JOINT_RAR, 5, 1, 1, 1);

        label_14 = new QLabel(gridLayoutWidget_4);
        label_14->setObjectName(QStringLiteral("label_14"));
        label_14->setEnabled(true);
        sizePolicy.setHeightForWidth(label_14->sizePolicy().hasHeightForWidth());
        label_14->setSizePolicy(sizePolicy);
        label_14->setMinimumSize(QSize(50, 20));
        label_14->setMaximumSize(QSize(50, 20));
        label_14->setFont(font2);
        label_14->setAlignment(Qt::AlignCenter);

        gridLayout_4->addWidget(label_14, 4, 0, 1, 1);

        label_19 = new QLabel(gridLayoutWidget_4);
        label_19->setObjectName(QStringLiteral("label_19"));
        label_19->setEnabled(true);
        sizePolicy.setHeightForWidth(label_19->sizePolicy().hasHeightForWidth());
        label_19->setSizePolicy(sizePolicy);
        label_19->setMinimumSize(QSize(50, 20));
        label_19->setMaximumSize(QSize(50, 20));
        label_19->setFont(font2);
        label_19->setAlignment(Qt::AlignCenter);

        gridLayout_4->addWidget(label_19, 5, 0, 1, 1);

        label_20 = new QLabel(gridLayoutWidget_4);
        label_20->setObjectName(QStringLiteral("label_20"));
        label_20->setEnabled(true);
        sizePolicy.setHeightForWidth(label_20->sizePolicy().hasHeightForWidth());
        label_20->setSizePolicy(sizePolicy);
        label_20->setMinimumSize(QSize(50, 20));
        label_20->setMaximumSize(QSize(50, 20));
        label_20->setFont(font2);
        label_20->setAlignment(Qt::AlignCenter);

        gridLayout_4->addWidget(label_20, 2, 0, 1, 1);

        LE_JOINT_RHY = new QLineEdit(gridLayoutWidget_4);
        LE_JOINT_RHY->setObjectName(QStringLiteral("LE_JOINT_RHY"));
        LE_JOINT_RHY->setEnabled(true);
        sizePolicy.setHeightForWidth(LE_JOINT_RHY->sizePolicy().hasHeightForWidth());
        LE_JOINT_RHY->setSizePolicy(sizePolicy);
        LE_JOINT_RHY->setMinimumSize(QSize(70, 20));
        LE_JOINT_RHY->setMaximumSize(QSize(70, 20));
        LE_JOINT_RHY->setFont(font2);
        LE_JOINT_RHY->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        LE_JOINT_RHY->setReadOnly(true);

        gridLayout_4->addWidget(LE_JOINT_RHY, 0, 1, 1, 1);

        label_21 = new QLabel(gridLayoutWidget_4);
        label_21->setObjectName(QStringLiteral("label_21"));
        label_21->setEnabled(true);
        sizePolicy.setHeightForWidth(label_21->sizePolicy().hasHeightForWidth());
        label_21->setSizePolicy(sizePolicy);
        label_21->setMinimumSize(QSize(50, 20));
        label_21->setMaximumSize(QSize(50, 20));
        label_21->setFont(font2);
        label_21->setAlignment(Qt::AlignCenter);

        gridLayout_4->addWidget(label_21, 0, 0, 1, 1);

        label_22 = new QLabel(gridLayoutWidget_4);
        label_22->setObjectName(QStringLiteral("label_22"));
        label_22->setEnabled(true);
        sizePolicy.setHeightForWidth(label_22->sizePolicy().hasHeightForWidth());
        label_22->setSizePolicy(sizePolicy);
        label_22->setMinimumSize(QSize(50, 20));
        label_22->setMaximumSize(QSize(50, 20));
        label_22->setFont(font2);
        label_22->setAlignment(Qt::AlignCenter);

        gridLayout_4->addWidget(label_22, 3, 0, 1, 1);

        LE_JOINT_RHP = new QLineEdit(gridLayoutWidget_4);
        LE_JOINT_RHP->setObjectName(QStringLiteral("LE_JOINT_RHP"));
        LE_JOINT_RHP->setEnabled(true);
        sizePolicy.setHeightForWidth(LE_JOINT_RHP->sizePolicy().hasHeightForWidth());
        LE_JOINT_RHP->setSizePolicy(sizePolicy);
        LE_JOINT_RHP->setMinimumSize(QSize(70, 20));
        LE_JOINT_RHP->setMaximumSize(QSize(70, 20));
        LE_JOINT_RHP->setFont(font2);
        LE_JOINT_RHP->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        LE_JOINT_RHP->setReadOnly(true);

        gridLayout_4->addWidget(LE_JOINT_RHP, 2, 1, 1, 1);

        LE_JOINT_RHR = new QLineEdit(gridLayoutWidget_4);
        LE_JOINT_RHR->setObjectName(QStringLiteral("LE_JOINT_RHR"));
        LE_JOINT_RHR->setEnabled(true);
        sizePolicy.setHeightForWidth(LE_JOINT_RHR->sizePolicy().hasHeightForWidth());
        LE_JOINT_RHR->setSizePolicy(sizePolicy);
        LE_JOINT_RHR->setMinimumSize(QSize(70, 20));
        LE_JOINT_RHR->setMaximumSize(QSize(70, 20));
        LE_JOINT_RHR->setFont(font2);
        LE_JOINT_RHR->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        LE_JOINT_RHR->setReadOnly(true);

        gridLayout_4->addWidget(LE_JOINT_RHR, 1, 1, 1, 1);

        LE_JOINT_RKN = new QLineEdit(gridLayoutWidget_4);
        LE_JOINT_RKN->setObjectName(QStringLiteral("LE_JOINT_RKN"));
        LE_JOINT_RKN->setEnabled(true);
        sizePolicy.setHeightForWidth(LE_JOINT_RKN->sizePolicy().hasHeightForWidth());
        LE_JOINT_RKN->setSizePolicy(sizePolicy);
        LE_JOINT_RKN->setMinimumSize(QSize(70, 20));
        LE_JOINT_RKN->setMaximumSize(QSize(70, 20));
        LE_JOINT_RKN->setFont(font2);
        LE_JOINT_RKN->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        LE_JOINT_RKN->setReadOnly(true);

        gridLayout_4->addWidget(LE_JOINT_RKN, 3, 1, 1, 1);

        label_23 = new QLabel(gridLayoutWidget_4);
        label_23->setObjectName(QStringLiteral("label_23"));
        label_23->setEnabled(true);
        sizePolicy.setHeightForWidth(label_23->sizePolicy().hasHeightForWidth());
        label_23->setSizePolicy(sizePolicy);
        label_23->setMinimumSize(QSize(50, 20));
        label_23->setMaximumSize(QSize(50, 20));
        label_23->setFont(font2);
        label_23->setAlignment(Qt::AlignCenter);

        gridLayout_4->addWidget(label_23, 1, 0, 1, 1);

        LE_JOINT_RAP = new QLineEdit(gridLayoutWidget_4);
        LE_JOINT_RAP->setObjectName(QStringLiteral("LE_JOINT_RAP"));
        LE_JOINT_RAP->setEnabled(true);
        sizePolicy.setHeightForWidth(LE_JOINT_RAP->sizePolicy().hasHeightForWidth());
        LE_JOINT_RAP->setSizePolicy(sizePolicy);
        LE_JOINT_RAP->setMinimumSize(QSize(70, 20));
        LE_JOINT_RAP->setMaximumSize(QSize(70, 20));
        LE_JOINT_RAP->setFont(font2);
        LE_JOINT_RAP->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        LE_JOINT_RAP->setReadOnly(true);

        gridLayout_4->addWidget(LE_JOINT_RAP, 4, 1, 1, 1);

        gridLayoutWidget_5 = new QWidget(groupBox);
        gridLayoutWidget_5->setObjectName(QStringLiteral("gridLayoutWidget_5"));
        gridLayoutWidget_5->setEnabled(true);
        gridLayoutWidget_5->setGeometry(QRect(160, 480, 128, 161));
        gridLayout_6 = new QGridLayout(gridLayoutWidget_5);
        gridLayout_6->setObjectName(QStringLiteral("gridLayout_6"));
        gridLayout_6->setContentsMargins(0, 0, 0, 0);
        LE_JOINT_LAR = new QLineEdit(gridLayoutWidget_5);
        LE_JOINT_LAR->setObjectName(QStringLiteral("LE_JOINT_LAR"));
        LE_JOINT_LAR->setEnabled(true);
        sizePolicy.setHeightForWidth(LE_JOINT_LAR->sizePolicy().hasHeightForWidth());
        LE_JOINT_LAR->setSizePolicy(sizePolicy);
        LE_JOINT_LAR->setMinimumSize(QSize(70, 20));
        LE_JOINT_LAR->setMaximumSize(QSize(70, 20));
        LE_JOINT_LAR->setFont(font2);
        LE_JOINT_LAR->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        LE_JOINT_LAR->setReadOnly(true);

        gridLayout_6->addWidget(LE_JOINT_LAR, 5, 1, 1, 1);

        label_30 = new QLabel(gridLayoutWidget_5);
        label_30->setObjectName(QStringLiteral("label_30"));
        label_30->setEnabled(true);
        sizePolicy.setHeightForWidth(label_30->sizePolicy().hasHeightForWidth());
        label_30->setSizePolicy(sizePolicy);
        label_30->setMinimumSize(QSize(50, 20));
        label_30->setMaximumSize(QSize(50, 20));
        label_30->setFont(font2);
        label_30->setAlignment(Qt::AlignCenter);

        gridLayout_6->addWidget(label_30, 4, 0, 1, 1);

        label_31 = new QLabel(gridLayoutWidget_5);
        label_31->setObjectName(QStringLiteral("label_31"));
        label_31->setEnabled(true);
        sizePolicy.setHeightForWidth(label_31->sizePolicy().hasHeightForWidth());
        label_31->setSizePolicy(sizePolicy);
        label_31->setMinimumSize(QSize(50, 20));
        label_31->setMaximumSize(QSize(50, 20));
        label_31->setFont(font2);
        label_31->setAlignment(Qt::AlignCenter);

        gridLayout_6->addWidget(label_31, 5, 0, 1, 1);

        label_32 = new QLabel(gridLayoutWidget_5);
        label_32->setObjectName(QStringLiteral("label_32"));
        label_32->setEnabled(true);
        sizePolicy.setHeightForWidth(label_32->sizePolicy().hasHeightForWidth());
        label_32->setSizePolicy(sizePolicy);
        label_32->setMinimumSize(QSize(50, 20));
        label_32->setMaximumSize(QSize(50, 20));
        label_32->setFont(font2);
        label_32->setAlignment(Qt::AlignCenter);

        gridLayout_6->addWidget(label_32, 2, 0, 1, 1);

        LE_JOINT_LHY = new QLineEdit(gridLayoutWidget_5);
        LE_JOINT_LHY->setObjectName(QStringLiteral("LE_JOINT_LHY"));
        LE_JOINT_LHY->setEnabled(true);
        sizePolicy.setHeightForWidth(LE_JOINT_LHY->sizePolicy().hasHeightForWidth());
        LE_JOINT_LHY->setSizePolicy(sizePolicy);
        LE_JOINT_LHY->setMinimumSize(QSize(70, 20));
        LE_JOINT_LHY->setMaximumSize(QSize(70, 20));
        LE_JOINT_LHY->setFont(font2);
        LE_JOINT_LHY->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        LE_JOINT_LHY->setReadOnly(true);

        gridLayout_6->addWidget(LE_JOINT_LHY, 0, 1, 1, 1);

        label_33 = new QLabel(gridLayoutWidget_5);
        label_33->setObjectName(QStringLiteral("label_33"));
        label_33->setEnabled(true);
        sizePolicy.setHeightForWidth(label_33->sizePolicy().hasHeightForWidth());
        label_33->setSizePolicy(sizePolicy);
        label_33->setMinimumSize(QSize(50, 20));
        label_33->setMaximumSize(QSize(50, 20));
        label_33->setFont(font2);
        label_33->setAlignment(Qt::AlignCenter);

        gridLayout_6->addWidget(label_33, 0, 0, 1, 1);

        label_34 = new QLabel(gridLayoutWidget_5);
        label_34->setObjectName(QStringLiteral("label_34"));
        label_34->setEnabled(true);
        sizePolicy.setHeightForWidth(label_34->sizePolicy().hasHeightForWidth());
        label_34->setSizePolicy(sizePolicy);
        label_34->setMinimumSize(QSize(50, 20));
        label_34->setMaximumSize(QSize(50, 20));
        label_34->setFont(font2);
        label_34->setAlignment(Qt::AlignCenter);

        gridLayout_6->addWidget(label_34, 3, 0, 1, 1);

        LE_JOINT_LHP = new QLineEdit(gridLayoutWidget_5);
        LE_JOINT_LHP->setObjectName(QStringLiteral("LE_JOINT_LHP"));
        LE_JOINT_LHP->setEnabled(true);
        sizePolicy.setHeightForWidth(LE_JOINT_LHP->sizePolicy().hasHeightForWidth());
        LE_JOINT_LHP->setSizePolicy(sizePolicy);
        LE_JOINT_LHP->setMinimumSize(QSize(70, 20));
        LE_JOINT_LHP->setMaximumSize(QSize(70, 20));
        LE_JOINT_LHP->setFont(font2);
        LE_JOINT_LHP->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        LE_JOINT_LHP->setReadOnly(true);

        gridLayout_6->addWidget(LE_JOINT_LHP, 2, 1, 1, 1);

        LE_JOINT_LHR = new QLineEdit(gridLayoutWidget_5);
        LE_JOINT_LHR->setObjectName(QStringLiteral("LE_JOINT_LHR"));
        LE_JOINT_LHR->setEnabled(true);
        sizePolicy.setHeightForWidth(LE_JOINT_LHR->sizePolicy().hasHeightForWidth());
        LE_JOINT_LHR->setSizePolicy(sizePolicy);
        LE_JOINT_LHR->setMinimumSize(QSize(70, 20));
        LE_JOINT_LHR->setMaximumSize(QSize(70, 20));
        LE_JOINT_LHR->setFont(font2);
        LE_JOINT_LHR->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        LE_JOINT_LHR->setReadOnly(true);

        gridLayout_6->addWidget(LE_JOINT_LHR, 1, 1, 1, 1);

        LE_JOINT_LKN = new QLineEdit(gridLayoutWidget_5);
        LE_JOINT_LKN->setObjectName(QStringLiteral("LE_JOINT_LKN"));
        LE_JOINT_LKN->setEnabled(true);
        sizePolicy.setHeightForWidth(LE_JOINT_LKN->sizePolicy().hasHeightForWidth());
        LE_JOINT_LKN->setSizePolicy(sizePolicy);
        LE_JOINT_LKN->setMinimumSize(QSize(70, 20));
        LE_JOINT_LKN->setMaximumSize(QSize(70, 20));
        LE_JOINT_LKN->setFont(font3);
        LE_JOINT_LKN->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        LE_JOINT_LKN->setReadOnly(true);

        gridLayout_6->addWidget(LE_JOINT_LKN, 3, 1, 1, 1);

        label_35 = new QLabel(gridLayoutWidget_5);
        label_35->setObjectName(QStringLiteral("label_35"));
        label_35->setEnabled(true);
        sizePolicy.setHeightForWidth(label_35->sizePolicy().hasHeightForWidth());
        label_35->setSizePolicy(sizePolicy);
        label_35->setMinimumSize(QSize(50, 20));
        label_35->setMaximumSize(QSize(50, 20));
        label_35->setFont(font2);
        label_35->setAlignment(Qt::AlignCenter);

        gridLayout_6->addWidget(label_35, 1, 0, 1, 1);

        LE_JOINT_LAP = new QLineEdit(gridLayoutWidget_5);
        LE_JOINT_LAP->setObjectName(QStringLiteral("LE_JOINT_LAP"));
        LE_JOINT_LAP->setEnabled(true);
        sizePolicy.setHeightForWidth(LE_JOINT_LAP->sizePolicy().hasHeightForWidth());
        LE_JOINT_LAP->setSizePolicy(sizePolicy);
        LE_JOINT_LAP->setMinimumSize(QSize(70, 20));
        LE_JOINT_LAP->setMaximumSize(QSize(70, 20));
        LE_JOINT_LAP->setFont(font2);
        LE_JOINT_LAP->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        LE_JOINT_LAP->setReadOnly(true);

        gridLayout_6->addWidget(LE_JOINT_LAP, 4, 1, 1, 1);

        gridLayoutWidget_7 = new QWidget(groupBox);
        gridLayoutWidget_7->setObjectName(QStringLiteral("gridLayoutWidget_7"));
        gridLayoutWidget_7->setEnabled(true);
        gridLayoutWidget_7->setGeometry(QRect(160, 70, 128, 61));
        gridLayout_8 = new QGridLayout(gridLayoutWidget_7);
        gridLayout_8->setObjectName(QStringLiteral("gridLayout_8"));
        gridLayout_8->setContentsMargins(0, 0, 0, 0);
        label_29 = new QLabel(gridLayoutWidget_7);
        label_29->setObjectName(QStringLiteral("label_29"));
        label_29->setEnabled(true);
        sizePolicy.setHeightForWidth(label_29->sizePolicy().hasHeightForWidth());
        label_29->setSizePolicy(sizePolicy);
        label_29->setMinimumSize(QSize(50, 20));
        label_29->setMaximumSize(QSize(50, 20));
        label_29->setFont(font2);
        label_29->setAlignment(Qt::AlignCenter);

        gridLayout_8->addWidget(label_29, 1, 0, 1, 1);

        label_36 = new QLabel(gridLayoutWidget_7);
        label_36->setObjectName(QStringLiteral("label_36"));
        label_36->setEnabled(true);
        sizePolicy.setHeightForWidth(label_36->sizePolicy().hasHeightForWidth());
        label_36->setSizePolicy(sizePolicy);
        label_36->setMinimumSize(QSize(50, 20));
        label_36->setMaximumSize(QSize(50, 20));
        label_36->setFont(font2);
        label_36->setAlignment(Qt::AlignCenter);

        gridLayout_8->addWidget(label_36, 0, 0, 1, 1);

        LE_JOINT_NK2 = new QLineEdit(gridLayoutWidget_7);
        LE_JOINT_NK2->setObjectName(QStringLiteral("LE_JOINT_NK2"));
        LE_JOINT_NK2->setEnabled(true);
        sizePolicy.setHeightForWidth(LE_JOINT_NK2->sizePolicy().hasHeightForWidth());
        LE_JOINT_NK2->setSizePolicy(sizePolicy);
        LE_JOINT_NK2->setMinimumSize(QSize(70, 20));
        LE_JOINT_NK2->setMaximumSize(QSize(70, 20));
        LE_JOINT_NK2->setFont(font2);
        LE_JOINT_NK2->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        LE_JOINT_NK2->setReadOnly(true);

        gridLayout_8->addWidget(LE_JOINT_NK2, 1, 1, 1, 1);

        LE_JOINT_NK1 = new QLineEdit(gridLayoutWidget_7);
        LE_JOINT_NK1->setObjectName(QStringLiteral("LE_JOINT_NK1"));
        LE_JOINT_NK1->setEnabled(true);
        sizePolicy.setHeightForWidth(LE_JOINT_NK1->sizePolicy().hasHeightForWidth());
        LE_JOINT_NK1->setSizePolicy(sizePolicy);
        LE_JOINT_NK1->setMinimumSize(QSize(70, 20));
        LE_JOINT_NK1->setMaximumSize(QSize(70, 20));
        LE_JOINT_NK1->setFont(font2);
        LE_JOINT_NK1->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        LE_JOINT_NK1->setReadOnly(true);

        gridLayout_8->addWidget(LE_JOINT_NK1, 0, 1, 1, 1);

        gridLayoutWidget_8 = new QWidget(groupBox);
        gridLayoutWidget_8->setObjectName(QStringLiteral("gridLayoutWidget_8"));
        gridLayoutWidget_8->setEnabled(true);
        gridLayoutWidget_8->setGeometry(QRect(10, 70, 128, 61));
        gridLayout_9 = new QGridLayout(gridLayoutWidget_8);
        gridLayout_9->setObjectName(QStringLiteral("gridLayout_9"));
        gridLayout_9->setContentsMargins(0, 0, 0, 0);
        label_37 = new QLabel(gridLayoutWidget_8);
        label_37->setObjectName(QStringLiteral("label_37"));
        label_37->setEnabled(true);
        sizePolicy.setHeightForWidth(label_37->sizePolicy().hasHeightForWidth());
        label_37->setSizePolicy(sizePolicy);
        label_37->setMinimumSize(QSize(50, 20));
        label_37->setMaximumSize(QSize(50, 20));
        label_37->setFont(font2);
        label_37->setAlignment(Qt::AlignCenter);

        gridLayout_9->addWidget(label_37, 1, 0, 1, 1);

        label_38 = new QLabel(gridLayoutWidget_8);
        label_38->setObjectName(QStringLiteral("label_38"));
        label_38->setEnabled(true);
        sizePolicy.setHeightForWidth(label_38->sizePolicy().hasHeightForWidth());
        label_38->setSizePolicy(sizePolicy);
        label_38->setMinimumSize(QSize(50, 20));
        label_38->setMaximumSize(QSize(50, 20));
        label_38->setFont(font2);
        label_38->setAlignment(Qt::AlignCenter);

        gridLayout_9->addWidget(label_38, 0, 0, 1, 1);

        LE_JOINT_WST = new QLineEdit(gridLayoutWidget_8);
        LE_JOINT_WST->setObjectName(QStringLiteral("LE_JOINT_WST"));
        LE_JOINT_WST->setEnabled(true);
        sizePolicy.setHeightForWidth(LE_JOINT_WST->sizePolicy().hasHeightForWidth());
        LE_JOINT_WST->setSizePolicy(sizePolicy);
        LE_JOINT_WST->setMinimumSize(QSize(70, 20));
        LE_JOINT_WST->setMaximumSize(QSize(70, 20));
        LE_JOINT_WST->setFont(font2);
        LE_JOINT_WST->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        LE_JOINT_WST->setReadOnly(true);

        gridLayout_9->addWidget(LE_JOINT_WST, 1, 1, 1, 1);

        LE_JOINT_NKY = new QLineEdit(gridLayoutWidget_8);
        LE_JOINT_NKY->setObjectName(QStringLiteral("LE_JOINT_NKY"));
        LE_JOINT_NKY->setEnabled(true);
        sizePolicy.setHeightForWidth(LE_JOINT_NKY->sizePolicy().hasHeightForWidth());
        LE_JOINT_NKY->setSizePolicy(sizePolicy);
        LE_JOINT_NKY->setMinimumSize(QSize(70, 20));
        LE_JOINT_NKY->setMaximumSize(QSize(70, 20));
        LE_JOINT_NKY->setFont(font2);
        LE_JOINT_NKY->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        LE_JOINT_NKY->setReadOnly(true);

        gridLayout_9->addWidget(LE_JOINT_NKY, 0, 1, 1, 1);

        gridLayoutWidget_9 = new QWidget(groupBox);
        gridLayoutWidget_9->setObjectName(QStringLiteral("gridLayoutWidget_9"));
        gridLayoutWidget_9->setEnabled(true);
        gridLayoutWidget_9->setGeometry(QRect(10, 340, 128, 126));
        gridLayout_7 = new QGridLayout(gridLayoutWidget_9);
        gridLayout_7->setObjectName(QStringLiteral("gridLayout_7"));
        gridLayout_7->setContentsMargins(0, 0, 0, 0);
        LE_JOINT_RF3 = new QLineEdit(gridLayoutWidget_9);
        LE_JOINT_RF3->setObjectName(QStringLiteral("LE_JOINT_RF3"));
        LE_JOINT_RF3->setEnabled(true);
        sizePolicy.setHeightForWidth(LE_JOINT_RF3->sizePolicy().hasHeightForWidth());
        LE_JOINT_RF3->setSizePolicy(sizePolicy);
        LE_JOINT_RF3->setMinimumSize(QSize(70, 20));
        LE_JOINT_RF3->setMaximumSize(QSize(70, 20));
        LE_JOINT_RF3->setFont(font2);
        LE_JOINT_RF3->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        LE_JOINT_RF3->setReadOnly(true);

        gridLayout_7->addWidget(LE_JOINT_RF3, 2, 1, 1, 1);

        LE_JOINT_RF2 = new QLineEdit(gridLayoutWidget_9);
        LE_JOINT_RF2->setObjectName(QStringLiteral("LE_JOINT_RF2"));
        LE_JOINT_RF2->setEnabled(true);
        sizePolicy.setHeightForWidth(LE_JOINT_RF2->sizePolicy().hasHeightForWidth());
        LE_JOINT_RF2->setSizePolicy(sizePolicy);
        LE_JOINT_RF2->setMinimumSize(QSize(70, 20));
        LE_JOINT_RF2->setMaximumSize(QSize(70, 20));
        LE_JOINT_RF2->setFont(font2);
        LE_JOINT_RF2->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        LE_JOINT_RF2->setReadOnly(true);

        gridLayout_7->addWidget(LE_JOINT_RF2, 1, 1, 1, 1);

        label_28 = new QLabel(gridLayoutWidget_9);
        label_28->setObjectName(QStringLiteral("label_28"));
        label_28->setEnabled(true);
        sizePolicy.setHeightForWidth(label_28->sizePolicy().hasHeightForWidth());
        label_28->setSizePolicy(sizePolicy);
        label_28->setMinimumSize(QSize(50, 20));
        label_28->setMaximumSize(QSize(50, 20));
        label_28->setFont(font2);
        label_28->setAlignment(Qt::AlignCenter);

        gridLayout_7->addWidget(label_28, 2, 0, 1, 1);

        label_26 = new QLabel(gridLayoutWidget_9);
        label_26->setObjectName(QStringLiteral("label_26"));
        label_26->setEnabled(true);
        sizePolicy.setHeightForWidth(label_26->sizePolicy().hasHeightForWidth());
        label_26->setSizePolicy(sizePolicy);
        label_26->setMinimumSize(QSize(50, 20));
        label_26->setMaximumSize(QSize(50, 20));
        label_26->setFont(font2);
        label_26->setAlignment(Qt::AlignCenter);

        gridLayout_7->addWidget(label_26, 1, 0, 1, 1);

        LE_JOINT_RF4 = new QLineEdit(gridLayoutWidget_9);
        LE_JOINT_RF4->setObjectName(QStringLiteral("LE_JOINT_RF4"));
        LE_JOINT_RF4->setEnabled(true);
        sizePolicy.setHeightForWidth(LE_JOINT_RF4->sizePolicy().hasHeightForWidth());
        LE_JOINT_RF4->setSizePolicy(sizePolicy);
        LE_JOINT_RF4->setMinimumSize(QSize(70, 20));
        LE_JOINT_RF4->setMaximumSize(QSize(70, 20));
        LE_JOINT_RF4->setFont(font2);
        LE_JOINT_RF4->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        LE_JOINT_RF4->setReadOnly(true);

        gridLayout_7->addWidget(LE_JOINT_RF4, 3, 1, 1, 1);

        label_39 = new QLabel(gridLayoutWidget_9);
        label_39->setObjectName(QStringLiteral("label_39"));
        label_39->setEnabled(true);
        sizePolicy.setHeightForWidth(label_39->sizePolicy().hasHeightForWidth());
        label_39->setSizePolicy(sizePolicy);
        label_39->setMinimumSize(QSize(50, 20));
        label_39->setMaximumSize(QSize(50, 20));
        label_39->setFont(font2);
        label_39->setAlignment(Qt::AlignCenter);

        gridLayout_7->addWidget(label_39, 3, 0, 1, 1);

        label_40 = new QLabel(gridLayoutWidget_9);
        label_40->setObjectName(QStringLiteral("label_40"));
        label_40->setEnabled(true);
        sizePolicy.setHeightForWidth(label_40->sizePolicy().hasHeightForWidth());
        label_40->setSizePolicy(sizePolicy);
        label_40->setMinimumSize(QSize(50, 20));
        label_40->setMaximumSize(QSize(50, 20));
        label_40->setFont(font2);
        label_40->setAlignment(Qt::AlignCenter);

        gridLayout_7->addWidget(label_40, 4, 0, 1, 1);

        LE_JOINT_RF5 = new QLineEdit(gridLayoutWidget_9);
        LE_JOINT_RF5->setObjectName(QStringLiteral("LE_JOINT_RF5"));
        LE_JOINT_RF5->setEnabled(true);
        sizePolicy.setHeightForWidth(LE_JOINT_RF5->sizePolicy().hasHeightForWidth());
        LE_JOINT_RF5->setSizePolicy(sizePolicy);
        LE_JOINT_RF5->setMinimumSize(QSize(70, 20));
        LE_JOINT_RF5->setMaximumSize(QSize(70, 20));
        LE_JOINT_RF5->setFont(font2);
        LE_JOINT_RF5->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        LE_JOINT_RF5->setReadOnly(true);

        gridLayout_7->addWidget(LE_JOINT_RF5, 4, 1, 1, 1);

        label_79 = new QLabel(gridLayoutWidget_9);
        label_79->setObjectName(QStringLiteral("label_79"));
        label_79->setEnabled(true);
        sizePolicy.setHeightForWidth(label_79->sizePolicy().hasHeightForWidth());
        label_79->setSizePolicy(sizePolicy);
        label_79->setMinimumSize(QSize(50, 20));
        label_79->setMaximumSize(QSize(50, 20));
        label_79->setFont(font2);
        label_79->setAlignment(Qt::AlignCenter);

        gridLayout_7->addWidget(label_79, 0, 0, 1, 1);

        LE_JOINT_RF1 = new QLineEdit(gridLayoutWidget_9);
        LE_JOINT_RF1->setObjectName(QStringLiteral("LE_JOINT_RF1"));
        LE_JOINT_RF1->setEnabled(true);
        sizePolicy.setHeightForWidth(LE_JOINT_RF1->sizePolicy().hasHeightForWidth());
        LE_JOINT_RF1->setSizePolicy(sizePolicy);
        LE_JOINT_RF1->setMinimumSize(QSize(70, 20));
        LE_JOINT_RF1->setMaximumSize(QSize(70, 20));
        LE_JOINT_RF1->setFont(font2);
        LE_JOINT_RF1->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        LE_JOINT_RF1->setReadOnly(true);

        gridLayout_7->addWidget(LE_JOINT_RF1, 0, 1, 1, 1);

        gridLayoutWidget_10 = new QWidget(groupBox);
        gridLayoutWidget_10->setObjectName(QStringLiteral("gridLayoutWidget_10"));
        gridLayoutWidget_10->setEnabled(true);
        gridLayoutWidget_10->setGeometry(QRect(160, 340, 128, 126));
        gridLayout_10 = new QGridLayout(gridLayoutWidget_10);
        gridLayout_10->setObjectName(QStringLiteral("gridLayout_10"));
        gridLayout_10->setContentsMargins(0, 0, 0, 0);
        LE_JOINT_LF3 = new QLineEdit(gridLayoutWidget_10);
        LE_JOINT_LF3->setObjectName(QStringLiteral("LE_JOINT_LF3"));
        LE_JOINT_LF3->setEnabled(true);
        sizePolicy.setHeightForWidth(LE_JOINT_LF3->sizePolicy().hasHeightForWidth());
        LE_JOINT_LF3->setSizePolicy(sizePolicy);
        LE_JOINT_LF3->setMinimumSize(QSize(70, 20));
        LE_JOINT_LF3->setMaximumSize(QSize(70, 20));
        LE_JOINT_LF3->setFont(font2);
        LE_JOINT_LF3->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        LE_JOINT_LF3->setReadOnly(true);

        gridLayout_10->addWidget(LE_JOINT_LF3, 2, 1, 1, 1);

        LE_JOINT_LF2 = new QLineEdit(gridLayoutWidget_10);
        LE_JOINT_LF2->setObjectName(QStringLiteral("LE_JOINT_LF2"));
        LE_JOINT_LF2->setEnabled(true);
        sizePolicy.setHeightForWidth(LE_JOINT_LF2->sizePolicy().hasHeightForWidth());
        LE_JOINT_LF2->setSizePolicy(sizePolicy);
        LE_JOINT_LF2->setMinimumSize(QSize(70, 20));
        LE_JOINT_LF2->setMaximumSize(QSize(70, 20));
        LE_JOINT_LF2->setFont(font2);
        LE_JOINT_LF2->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        LE_JOINT_LF2->setReadOnly(true);

        gridLayout_10->addWidget(LE_JOINT_LF2, 1, 1, 1, 1);

        label_41 = new QLabel(gridLayoutWidget_10);
        label_41->setObjectName(QStringLiteral("label_41"));
        label_41->setEnabled(true);
        sizePolicy.setHeightForWidth(label_41->sizePolicy().hasHeightForWidth());
        label_41->setSizePolicy(sizePolicy);
        label_41->setMinimumSize(QSize(50, 20));
        label_41->setMaximumSize(QSize(50, 20));
        label_41->setFont(font2);
        label_41->setAlignment(Qt::AlignCenter);

        gridLayout_10->addWidget(label_41, 2, 0, 1, 1);

        label_42 = new QLabel(gridLayoutWidget_10);
        label_42->setObjectName(QStringLiteral("label_42"));
        label_42->setEnabled(true);
        sizePolicy.setHeightForWidth(label_42->sizePolicy().hasHeightForWidth());
        label_42->setSizePolicy(sizePolicy);
        label_42->setMinimumSize(QSize(50, 20));
        label_42->setMaximumSize(QSize(50, 20));
        label_42->setFont(font2);
        label_42->setAlignment(Qt::AlignCenter);

        gridLayout_10->addWidget(label_42, 1, 0, 1, 1);

        LE_JOINT_LF4 = new QLineEdit(gridLayoutWidget_10);
        LE_JOINT_LF4->setObjectName(QStringLiteral("LE_JOINT_LF4"));
        LE_JOINT_LF4->setEnabled(true);
        sizePolicy.setHeightForWidth(LE_JOINT_LF4->sizePolicy().hasHeightForWidth());
        LE_JOINT_LF4->setSizePolicy(sizePolicy);
        LE_JOINT_LF4->setMinimumSize(QSize(70, 20));
        LE_JOINT_LF4->setMaximumSize(QSize(70, 20));
        LE_JOINT_LF4->setFont(font2);
        LE_JOINT_LF4->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        LE_JOINT_LF4->setReadOnly(true);

        gridLayout_10->addWidget(LE_JOINT_LF4, 3, 1, 1, 1);

        label_43 = new QLabel(gridLayoutWidget_10);
        label_43->setObjectName(QStringLiteral("label_43"));
        label_43->setEnabled(true);
        sizePolicy.setHeightForWidth(label_43->sizePolicy().hasHeightForWidth());
        label_43->setSizePolicy(sizePolicy);
        label_43->setMinimumSize(QSize(50, 20));
        label_43->setMaximumSize(QSize(50, 20));
        label_43->setFont(font2);
        label_43->setAlignment(Qt::AlignCenter);

        gridLayout_10->addWidget(label_43, 3, 0, 1, 1);

        label_44 = new QLabel(gridLayoutWidget_10);
        label_44->setObjectName(QStringLiteral("label_44"));
        label_44->setEnabled(true);
        sizePolicy.setHeightForWidth(label_44->sizePolicy().hasHeightForWidth());
        label_44->setSizePolicy(sizePolicy);
        label_44->setMinimumSize(QSize(50, 20));
        label_44->setMaximumSize(QSize(50, 20));
        label_44->setFont(font2);
        label_44->setAlignment(Qt::AlignCenter);

        gridLayout_10->addWidget(label_44, 4, 0, 1, 1);

        LE_JOINT_LF5 = new QLineEdit(gridLayoutWidget_10);
        LE_JOINT_LF5->setObjectName(QStringLiteral("LE_JOINT_LF5"));
        LE_JOINT_LF5->setEnabled(true);
        sizePolicy.setHeightForWidth(LE_JOINT_LF5->sizePolicy().hasHeightForWidth());
        LE_JOINT_LF5->setSizePolicy(sizePolicy);
        LE_JOINT_LF5->setMinimumSize(QSize(70, 20));
        LE_JOINT_LF5->setMaximumSize(QSize(70, 20));
        LE_JOINT_LF5->setFont(font2);
        LE_JOINT_LF5->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        LE_JOINT_LF5->setReadOnly(true);

        gridLayout_10->addWidget(LE_JOINT_LF5, 4, 1, 1, 1);

        label_80 = new QLabel(gridLayoutWidget_10);
        label_80->setObjectName(QStringLiteral("label_80"));
        label_80->setEnabled(true);
        sizePolicy.setHeightForWidth(label_80->sizePolicy().hasHeightForWidth());
        label_80->setSizePolicy(sizePolicy);
        label_80->setMinimumSize(QSize(50, 20));
        label_80->setMaximumSize(QSize(50, 20));
        label_80->setFont(font2);
        label_80->setAlignment(Qt::AlignCenter);

        gridLayout_10->addWidget(label_80, 0, 0, 1, 1);

        LE_JOINT_LF1 = new QLineEdit(gridLayoutWidget_10);
        LE_JOINT_LF1->setObjectName(QStringLiteral("LE_JOINT_LF1"));
        LE_JOINT_LF1->setEnabled(true);
        sizePolicy.setHeightForWidth(LE_JOINT_LF1->sizePolicy().hasHeightForWidth());
        LE_JOINT_LF1->setSizePolicy(sizePolicy);
        LE_JOINT_LF1->setMinimumSize(QSize(70, 20));
        LE_JOINT_LF1->setMaximumSize(QSize(70, 20));
        LE_JOINT_LF1->setFont(font2);
        LE_JOINT_LF1->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        LE_JOINT_LF1->setReadOnly(true);

        gridLayout_10->addWidget(LE_JOINT_LF1, 0, 1, 1, 1);

        BTN_ENC_ENABLE = new QPushButton(JointDialog);
        BTN_ENC_ENABLE->setObjectName(QStringLiteral("BTN_ENC_ENABLE"));
        BTN_ENC_ENABLE->setEnabled(true);
        BTN_ENC_ENABLE->setGeometry(QRect(140, 10, 81, 31));
        BTN_ENC_ENABLE->setFont(font4);
        BTN_ENC_DISABLE = new QPushButton(JointDialog);
        BTN_ENC_DISABLE->setObjectName(QStringLiteral("BTN_ENC_DISABLE"));
        BTN_ENC_DISABLE->setEnabled(true);
        BTN_ENC_DISABLE->setGeometry(QRect(230, 10, 81, 31));
        BTN_ENC_DISABLE->setFont(font4);
        line = new QFrame(JointDialog);
        line->setObjectName(QStringLiteral("line"));
        line->setEnabled(true);
        line->setGeometry(QRect(10, 30, 118, 3));
        QFont font5;
        font5.setPointSize(8);
        line->setFont(font5);
        line->setLineWidth(2);
        line->setFrameShape(QFrame::HLine);
        line->setFrameShadow(QFrame::Sunken);

        retranslateUi(JointDialog);

        QMetaObject::connectSlotsByName(JointDialog);
    } // setupUi

    void retranslateUi(QDialog *JointDialog)
    {
        JointDialog->setWindowTitle(QApplication::translate("JointDialog", "Dialog", Q_NULLPTR));
        label->setText(QApplication::translate("JointDialog", "Joint", Q_NULLPTR));
        groupBox->setTitle(QApplication::translate("JointDialog", "Joint Reference && Encoder", Q_NULLPTR));
        label_5->setText(QApplication::translate("JointDialog", "RWY", Q_NULLPTR));
        label_6->setText(QApplication::translate("JointDialog", "RWP", Q_NULLPTR));
        label_3->setText(QApplication::translate("JointDialog", "RSY", Q_NULLPTR));
        label_2->setText(QApplication::translate("JointDialog", "RSP", Q_NULLPTR));
        label_4->setText(QApplication::translate("JointDialog", "REB", Q_NULLPTR));
        label_7->setText(QApplication::translate("JointDialog", "RSR", Q_NULLPTR));
        RB_JOINT_REFERENCE->setText(QApplication::translate("JointDialog", "Reference", Q_NULLPTR));
        RB_JOINT_ENCODER->setText(QApplication::translate("JointDialog", "Encoder", Q_NULLPTR));
        label_8->setText(QApplication::translate("JointDialog", "LSP", Q_NULLPTR));
        label_9->setText(QApplication::translate("JointDialog", "LEB", Q_NULLPTR));
        label_10->setText(QApplication::translate("JointDialog", "LWP", Q_NULLPTR));
        label_11->setText(QApplication::translate("JointDialog", "LWY", Q_NULLPTR));
        label_12->setText(QApplication::translate("JointDialog", "LSY", Q_NULLPTR));
        label_13->setText(QApplication::translate("JointDialog", "LSR", Q_NULLPTR));
        label_14->setText(QApplication::translate("JointDialog", "RAP", Q_NULLPTR));
        label_19->setText(QApplication::translate("JointDialog", "RAR", Q_NULLPTR));
        label_20->setText(QApplication::translate("JointDialog", "RHP", Q_NULLPTR));
        label_21->setText(QApplication::translate("JointDialog", "RHY", Q_NULLPTR));
        label_22->setText(QApplication::translate("JointDialog", "RKN", Q_NULLPTR));
        label_23->setText(QApplication::translate("JointDialog", "RHR", Q_NULLPTR));
        label_30->setText(QApplication::translate("JointDialog", "LAP", Q_NULLPTR));
        label_31->setText(QApplication::translate("JointDialog", "LAR", Q_NULLPTR));
        label_32->setText(QApplication::translate("JointDialog", "LHP", Q_NULLPTR));
        label_33->setText(QApplication::translate("JointDialog", "LHY", Q_NULLPTR));
        label_34->setText(QApplication::translate("JointDialog", "LKN", Q_NULLPTR));
        label_35->setText(QApplication::translate("JointDialog", "LHR", Q_NULLPTR));
        label_29->setText(QApplication::translate("JointDialog", "NK2", Q_NULLPTR));
        label_36->setText(QApplication::translate("JointDialog", "NK1", Q_NULLPTR));
        label_37->setText(QApplication::translate("JointDialog", "WST", Q_NULLPTR));
        label_38->setText(QApplication::translate("JointDialog", "NKY", Q_NULLPTR));
        label_28->setText(QApplication::translate("JointDialog", "RF3", Q_NULLPTR));
        label_26->setText(QApplication::translate("JointDialog", "RF2", Q_NULLPTR));
        label_39->setText(QApplication::translate("JointDialog", "RF4", Q_NULLPTR));
        label_40->setText(QApplication::translate("JointDialog", "RF5", Q_NULLPTR));
        label_79->setText(QApplication::translate("JointDialog", "RF1", Q_NULLPTR));
        label_41->setText(QApplication::translate("JointDialog", "LF3", Q_NULLPTR));
        label_42->setText(QApplication::translate("JointDialog", "LF2", Q_NULLPTR));
        label_43->setText(QApplication::translate("JointDialog", "LF4", Q_NULLPTR));
        label_44->setText(QApplication::translate("JointDialog", "LF5", Q_NULLPTR));
        label_80->setText(QApplication::translate("JointDialog", "LF1", Q_NULLPTR));
        BTN_ENC_ENABLE->setText(QApplication::translate("JointDialog", "Enc.Enable", Q_NULLPTR));
        BTN_ENC_DISABLE->setText(QApplication::translate("JointDialog", "Enc. Disable", Q_NULLPTR));
    } // retranslateUi

};

namespace Ui {
    class JointDialog: public Ui_JointDialog {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_JOINTDIALOG_H
