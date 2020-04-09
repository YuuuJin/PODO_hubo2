/********************************************************************************
** Form generated from reading UI file 'SensorDialog.ui'
**
** Created by: Qt User Interface Compiler version 5.8.0
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_SENSORDIALOG_H
#define UI_SENSORDIALOG_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QDialog>
#include <QtWidgets/QFrame>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_SensorDialog
{
public:
    QGroupBox *groupBox;
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
    QWidget *gridLayoutWidget_15;
    QGridLayout *gridLayout_15;
    QLineEdit *LE_SENSOR_CIMU_PITCH;
    QLineEdit *LE_SENSOR_CIMU_PITCH_VEL;
    QLabel *label_53;
    QLineEdit *LE_SENSOR_CIMU_PITCH_ACC;
    QLabel *label_60;
    QLabel *label_49;
    QLabel *label_52;
    QLabel *label_54;
    QLineEdit *LE_SENSOR_CIMU_ROLL;
    QLineEdit *LE_SENSOR_CIMU_ROLL_VEL;
    QLineEdit *LE_SENSOR_CIMU_ROLL_ACC;
    QLabel *label_63;
    QWidget *gridLayoutWidget_3;
    QGridLayout *gridLayout_3;
    QLineEdit *LE_SENSOR_CIMU_PITCH_OFFSET;
    QLabel *label_55;
    QLineEdit *LE_SENSOR_CIMU_ROLL_OFFSET;
    QLabel *label_46;
    QPushButton *BTN_CIMU_GET_OFFSET;
    QPushButton *BTN_CIMU_SET_OFFSET;
    QLabel *label_65;
    QWidget *gridLayoutWidget_14;
    QGridLayout *gridLayout_14;
    QLineEdit *LE_ZMP_LY;
    QLineEdit *LE_ZMP_LX;
    QLabel *label_51;
    QLineEdit *LE_ZMP_RY;
    QLabel *label_50;
    QLabel *label_56;
    QLineEdit *LE_ZMP_RX;
    QLabel *label_57;
    QLineEdit *LE_ZMP_WX;
    QLineEdit *LE_ZMP_WY;
    QLabel *label_58;
    QFrame *line_2;
    QFrame *line_3;
    QPushButton *BTN_NEW_IMU_ENABLE;
    QPushButton *BTN_NEW_IMU_NULL;
    QPushButton *BTN_NEW_IMU_RESET;
    QLabel *label;
    QFrame *line;
    QPushButton *BTN_SENSOR_FT_NULL;
    QPushButton *BTN_SENSOR_IMU_NULL;
    QPushButton *BTN_SENSOR_ENABLE;
    QPushButton *BTN_SENSOR_DISABLE;

    void setupUi(QDialog *SensorDialog)
    {
        if (SensorDialog->objectName().isEmpty())
            SensorDialog->setObjectName(QStringLiteral("SensorDialog"));
        SensorDialog->resize(490, 541);
        groupBox = new QGroupBox(SensorDialog);
        groupBox->setObjectName(QStringLiteral("groupBox"));
        groupBox->setEnabled(true);
        groupBox->setGeometry(QRect(10, 50, 471, 471));
        QFont font;
        font.setPointSize(11);
        font.setBold(true);
        font.setWeight(75);
        groupBox->setFont(font);
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
        gridLayoutWidget_12 = new QWidget(groupBox);
        gridLayoutWidget_12->setObjectName(QStringLiteral("gridLayoutWidget_12"));
        gridLayoutWidget_12->setEnabled(true);
        gridLayoutWidget_12->setGeometry(QRect(10, 60, 451, 131));
        QFont font1;
        font1.setPointSize(8);
        gridLayoutWidget_12->setFont(font1);
        gridLayout_12 = new QGridLayout(gridLayoutWidget_12);
        gridLayout_12->setObjectName(QStringLiteral("gridLayout_12"));
        gridLayout_12->setContentsMargins(0, 0, 0, 0);
        LE_SENSOR_LAFT_MY = new QLineEdit(gridLayoutWidget_12);
        LE_SENSOR_LAFT_MY->setObjectName(QStringLiteral("LE_SENSOR_LAFT_MY"));
        LE_SENSOR_LAFT_MY->setEnabled(true);
        QSizePolicy sizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(LE_SENSOR_LAFT_MY->sizePolicy().hasHeightForWidth());
        LE_SENSOR_LAFT_MY->setSizePolicy(sizePolicy);
        LE_SENSOR_LAFT_MY->setMinimumSize(QSize(60, 20));
        LE_SENSOR_LAFT_MY->setMaximumSize(QSize(60, 20));
        QFont font2;
        font2.setPointSize(9);
        LE_SENSOR_LAFT_MY->setFont(font2);
        LE_SENSOR_LAFT_MY->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        LE_SENSOR_LAFT_MY->setReadOnly(true);

        gridLayout_12->addWidget(LE_SENSOR_LAFT_MY, 4, 2, 1, 1);

        LE_SENSOR_LWFT_MY = new QLineEdit(gridLayoutWidget_12);
        LE_SENSOR_LWFT_MY->setObjectName(QStringLiteral("LE_SENSOR_LWFT_MY"));
        LE_SENSOR_LWFT_MY->setEnabled(true);
        sizePolicy.setHeightForWidth(LE_SENSOR_LWFT_MY->sizePolicy().hasHeightForWidth());
        LE_SENSOR_LWFT_MY->setSizePolicy(sizePolicy);
        LE_SENSOR_LWFT_MY->setMinimumSize(QSize(60, 20));
        LE_SENSOR_LWFT_MY->setMaximumSize(QSize(60, 20));
        LE_SENSOR_LWFT_MY->setFont(font2);
        LE_SENSOR_LWFT_MY->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        LE_SENSOR_LWFT_MY->setReadOnly(true);

        gridLayout_12->addWidget(LE_SENSOR_LWFT_MY, 2, 2, 1, 1);

        label_43 = new QLabel(gridLayoutWidget_12);
        label_43->setObjectName(QStringLiteral("label_43"));
        label_43->setEnabled(true);
        sizePolicy.setHeightForWidth(label_43->sizePolicy().hasHeightForWidth());
        label_43->setSizePolicy(sizePolicy);
        label_43->setMinimumSize(QSize(60, 20));
        label_43->setMaximumSize(QSize(60, 20));
        label_43->setFont(font2);
        label_43->setAlignment(Qt::AlignCenter);

        gridLayout_12->addWidget(label_43, 0, 5, 1, 1);

        label_32 = new QLabel(gridLayoutWidget_12);
        label_32->setObjectName(QStringLiteral("label_32"));
        label_32->setEnabled(true);
        sizePolicy.setHeightForWidth(label_32->sizePolicy().hasHeightForWidth());
        label_32->setSizePolicy(sizePolicy);
        label_32->setMinimumSize(QSize(60, 20));
        label_32->setMaximumSize(QSize(60, 20));
        label_32->setFont(font2);
        label_32->setAlignment(Qt::AlignCenter);

        gridLayout_12->addWidget(label_32, 0, 3, 1, 1);

        LE_SENSOR_LWFT_MZ = new QLineEdit(gridLayoutWidget_12);
        LE_SENSOR_LWFT_MZ->setObjectName(QStringLiteral("LE_SENSOR_LWFT_MZ"));
        LE_SENSOR_LWFT_MZ->setEnabled(true);
        sizePolicy.setHeightForWidth(LE_SENSOR_LWFT_MZ->sizePolicy().hasHeightForWidth());
        LE_SENSOR_LWFT_MZ->setSizePolicy(sizePolicy);
        LE_SENSOR_LWFT_MZ->setMinimumSize(QSize(60, 20));
        LE_SENSOR_LWFT_MZ->setMaximumSize(QSize(60, 20));
        LE_SENSOR_LWFT_MZ->setFont(font2);
        LE_SENSOR_LWFT_MZ->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        LE_SENSOR_LWFT_MZ->setReadOnly(true);

        gridLayout_12->addWidget(LE_SENSOR_LWFT_MZ, 2, 3, 1, 1);

        label_44 = new QLabel(gridLayoutWidget_12);
        label_44->setObjectName(QStringLiteral("label_44"));
        label_44->setEnabled(true);
        sizePolicy.setHeightForWidth(label_44->sizePolicy().hasHeightForWidth());
        label_44->setSizePolicy(sizePolicy);
        label_44->setMinimumSize(QSize(60, 20));
        label_44->setMaximumSize(QSize(60, 20));
        label_44->setFont(font2);
        label_44->setAlignment(Qt::AlignCenter);

        gridLayout_12->addWidget(label_44, 0, 1, 1, 1);

        label_45 = new QLabel(gridLayoutWidget_12);
        label_45->setObjectName(QStringLiteral("label_45"));
        label_45->setEnabled(true);
        label_45->setMinimumSize(QSize(50, 20));
        label_45->setMaximumSize(QSize(50, 20));
        label_45->setFont(font2);
        label_45->setAlignment(Qt::AlignCenter);

        gridLayout_12->addWidget(label_45, 1, 0, 1, 1);

        label_39 = new QLabel(gridLayoutWidget_12);
        label_39->setObjectName(QStringLiteral("label_39"));
        label_39->setEnabled(true);
        label_39->setMinimumSize(QSize(50, 20));
        label_39->setMaximumSize(QSize(50, 20));
        label_39->setFont(font2);
        label_39->setAlignment(Qt::AlignCenter);

        gridLayout_12->addWidget(label_39, 2, 0, 1, 1);

        LE_SENSOR_RAFT_FZ = new QLineEdit(gridLayoutWidget_12);
        LE_SENSOR_RAFT_FZ->setObjectName(QStringLiteral("LE_SENSOR_RAFT_FZ"));
        LE_SENSOR_RAFT_FZ->setEnabled(true);
        sizePolicy.setHeightForWidth(LE_SENSOR_RAFT_FZ->sizePolicy().hasHeightForWidth());
        LE_SENSOR_RAFT_FZ->setSizePolicy(sizePolicy);
        LE_SENSOR_RAFT_FZ->setMinimumSize(QSize(60, 20));
        LE_SENSOR_RAFT_FZ->setMaximumSize(QSize(60, 20));
        LE_SENSOR_RAFT_FZ->setFont(font2);
        LE_SENSOR_RAFT_FZ->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        LE_SENSOR_RAFT_FZ->setReadOnly(true);

        gridLayout_12->addWidget(LE_SENSOR_RAFT_FZ, 3, 6, 1, 1);

        label_47 = new QLabel(gridLayoutWidget_12);
        label_47->setObjectName(QStringLiteral("label_47"));
        label_47->setEnabled(true);
        label_47->setMinimumSize(QSize(50, 20));
        label_47->setMaximumSize(QSize(50, 20));
        label_47->setFont(font2);
        label_47->setAlignment(Qt::AlignCenter);

        gridLayout_12->addWidget(label_47, 3, 0, 1, 1);

        LE_SENSOR_LAFT_FZ = new QLineEdit(gridLayoutWidget_12);
        LE_SENSOR_LAFT_FZ->setObjectName(QStringLiteral("LE_SENSOR_LAFT_FZ"));
        LE_SENSOR_LAFT_FZ->setEnabled(true);
        sizePolicy.setHeightForWidth(LE_SENSOR_LAFT_FZ->sizePolicy().hasHeightForWidth());
        LE_SENSOR_LAFT_FZ->setSizePolicy(sizePolicy);
        LE_SENSOR_LAFT_FZ->setMinimumSize(QSize(60, 20));
        LE_SENSOR_LAFT_FZ->setMaximumSize(QSize(60, 20));
        LE_SENSOR_LAFT_FZ->setFont(font2);
        LE_SENSOR_LAFT_FZ->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        LE_SENSOR_LAFT_FZ->setReadOnly(true);

        gridLayout_12->addWidget(LE_SENSOR_LAFT_FZ, 4, 6, 1, 1);

        LE_SENSOR_LAFT_MX = new QLineEdit(gridLayoutWidget_12);
        LE_SENSOR_LAFT_MX->setObjectName(QStringLiteral("LE_SENSOR_LAFT_MX"));
        LE_SENSOR_LAFT_MX->setEnabled(true);
        sizePolicy.setHeightForWidth(LE_SENSOR_LAFT_MX->sizePolicy().hasHeightForWidth());
        LE_SENSOR_LAFT_MX->setSizePolicy(sizePolicy);
        LE_SENSOR_LAFT_MX->setMinimumSize(QSize(60, 20));
        LE_SENSOR_LAFT_MX->setMaximumSize(QSize(60, 20));
        LE_SENSOR_LAFT_MX->setFont(font2);
        LE_SENSOR_LAFT_MX->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        LE_SENSOR_LAFT_MX->setReadOnly(true);

        gridLayout_12->addWidget(LE_SENSOR_LAFT_MX, 4, 1, 1, 1);

        label_48 = new QLabel(gridLayoutWidget_12);
        label_48->setObjectName(QStringLiteral("label_48"));
        label_48->setEnabled(true);
        label_48->setMinimumSize(QSize(50, 20));
        label_48->setMaximumSize(QSize(50, 20));
        label_48->setFont(font2);
        label_48->setAlignment(Qt::AlignCenter);

        gridLayout_12->addWidget(label_48, 4, 0, 1, 1);

        LE_SENSOR_RWFT_FZ = new QLineEdit(gridLayoutWidget_12);
        LE_SENSOR_RWFT_FZ->setObjectName(QStringLiteral("LE_SENSOR_RWFT_FZ"));
        LE_SENSOR_RWFT_FZ->setEnabled(true);
        sizePolicy.setHeightForWidth(LE_SENSOR_RWFT_FZ->sizePolicy().hasHeightForWidth());
        LE_SENSOR_RWFT_FZ->setSizePolicy(sizePolicy);
        LE_SENSOR_RWFT_FZ->setMinimumSize(QSize(60, 20));
        LE_SENSOR_RWFT_FZ->setMaximumSize(QSize(60, 20));
        LE_SENSOR_RWFT_FZ->setFont(font2);
        LE_SENSOR_RWFT_FZ->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        LE_SENSOR_RWFT_FZ->setReadOnly(true);

        gridLayout_12->addWidget(LE_SENSOR_RWFT_FZ, 1, 6, 1, 1);

        LE_SENSOR_LWFT_MX = new QLineEdit(gridLayoutWidget_12);
        LE_SENSOR_LWFT_MX->setObjectName(QStringLiteral("LE_SENSOR_LWFT_MX"));
        LE_SENSOR_LWFT_MX->setEnabled(true);
        sizePolicy.setHeightForWidth(LE_SENSOR_LWFT_MX->sizePolicy().hasHeightForWidth());
        LE_SENSOR_LWFT_MX->setSizePolicy(sizePolicy);
        LE_SENSOR_LWFT_MX->setMinimumSize(QSize(60, 20));
        LE_SENSOR_LWFT_MX->setMaximumSize(QSize(60, 20));
        LE_SENSOR_LWFT_MX->setFont(font2);
        LE_SENSOR_LWFT_MX->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        LE_SENSOR_LWFT_MX->setReadOnly(true);

        gridLayout_12->addWidget(LE_SENSOR_LWFT_MX, 2, 1, 1, 1);

        LE_SENSOR_RWFT_MY = new QLineEdit(gridLayoutWidget_12);
        LE_SENSOR_RWFT_MY->setObjectName(QStringLiteral("LE_SENSOR_RWFT_MY"));
        LE_SENSOR_RWFT_MY->setEnabled(true);
        sizePolicy.setHeightForWidth(LE_SENSOR_RWFT_MY->sizePolicy().hasHeightForWidth());
        LE_SENSOR_RWFT_MY->setSizePolicy(sizePolicy);
        LE_SENSOR_RWFT_MY->setMinimumSize(QSize(60, 20));
        LE_SENSOR_RWFT_MY->setMaximumSize(QSize(60, 20));
        LE_SENSOR_RWFT_MY->setFont(font2);
        LE_SENSOR_RWFT_MY->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        LE_SENSOR_RWFT_MY->setReadOnly(true);

        gridLayout_12->addWidget(LE_SENSOR_RWFT_MY, 1, 2, 1, 1);

        LE_SENSOR_RAFT_MX = new QLineEdit(gridLayoutWidget_12);
        LE_SENSOR_RAFT_MX->setObjectName(QStringLiteral("LE_SENSOR_RAFT_MX"));
        LE_SENSOR_RAFT_MX->setEnabled(true);
        sizePolicy.setHeightForWidth(LE_SENSOR_RAFT_MX->sizePolicy().hasHeightForWidth());
        LE_SENSOR_RAFT_MX->setSizePolicy(sizePolicy);
        LE_SENSOR_RAFT_MX->setMinimumSize(QSize(60, 20));
        LE_SENSOR_RAFT_MX->setMaximumSize(QSize(60, 20));
        LE_SENSOR_RAFT_MX->setFont(font2);
        LE_SENSOR_RAFT_MX->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        LE_SENSOR_RAFT_MX->setReadOnly(true);

        gridLayout_12->addWidget(LE_SENSOR_RAFT_MX, 3, 1, 1, 1);

        label_42 = new QLabel(gridLayoutWidget_12);
        label_42->setObjectName(QStringLiteral("label_42"));
        label_42->setEnabled(true);
        sizePolicy.setHeightForWidth(label_42->sizePolicy().hasHeightForWidth());
        label_42->setSizePolicy(sizePolicy);
        label_42->setMinimumSize(QSize(60, 20));
        label_42->setMaximumSize(QSize(60, 20));
        label_42->setFont(font2);
        label_42->setAlignment(Qt::AlignCenter);

        gridLayout_12->addWidget(label_42, 0, 6, 1, 1);

        label_31 = new QLabel(gridLayoutWidget_12);
        label_31->setObjectName(QStringLiteral("label_31"));
        label_31->setEnabled(true);
        sizePolicy.setHeightForWidth(label_31->sizePolicy().hasHeightForWidth());
        label_31->setSizePolicy(sizePolicy);
        label_31->setMinimumSize(QSize(60, 20));
        label_31->setMaximumSize(QSize(60, 20));
        label_31->setFont(font2);
        label_31->setAlignment(Qt::AlignCenter);

        gridLayout_12->addWidget(label_31, 0, 2, 1, 1);

        LE_SENSOR_RWFT_MX = new QLineEdit(gridLayoutWidget_12);
        LE_SENSOR_RWFT_MX->setObjectName(QStringLiteral("LE_SENSOR_RWFT_MX"));
        LE_SENSOR_RWFT_MX->setEnabled(true);
        sizePolicy.setHeightForWidth(LE_SENSOR_RWFT_MX->sizePolicy().hasHeightForWidth());
        LE_SENSOR_RWFT_MX->setSizePolicy(sizePolicy);
        LE_SENSOR_RWFT_MX->setMinimumSize(QSize(60, 20));
        LE_SENSOR_RWFT_MX->setMaximumSize(QSize(60, 20));
        LE_SENSOR_RWFT_MX->setFont(font2);
        LE_SENSOR_RWFT_MX->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        LE_SENSOR_RWFT_MX->setReadOnly(true);

        gridLayout_12->addWidget(LE_SENSOR_RWFT_MX, 1, 1, 1, 1);

        LE_SENSOR_RAFT_MY = new QLineEdit(gridLayoutWidget_12);
        LE_SENSOR_RAFT_MY->setObjectName(QStringLiteral("LE_SENSOR_RAFT_MY"));
        LE_SENSOR_RAFT_MY->setEnabled(true);
        sizePolicy.setHeightForWidth(LE_SENSOR_RAFT_MY->sizePolicy().hasHeightForWidth());
        LE_SENSOR_RAFT_MY->setSizePolicy(sizePolicy);
        LE_SENSOR_RAFT_MY->setMinimumSize(QSize(60, 20));
        LE_SENSOR_RAFT_MY->setMaximumSize(QSize(60, 20));
        LE_SENSOR_RAFT_MY->setFont(font2);
        LE_SENSOR_RAFT_MY->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        LE_SENSOR_RAFT_MY->setReadOnly(true);

        gridLayout_12->addWidget(LE_SENSOR_RAFT_MY, 3, 2, 1, 1);

        LE_SENSOR_RWFT_MZ = new QLineEdit(gridLayoutWidget_12);
        LE_SENSOR_RWFT_MZ->setObjectName(QStringLiteral("LE_SENSOR_RWFT_MZ"));
        LE_SENSOR_RWFT_MZ->setEnabled(true);
        sizePolicy.setHeightForWidth(LE_SENSOR_RWFT_MZ->sizePolicy().hasHeightForWidth());
        LE_SENSOR_RWFT_MZ->setSizePolicy(sizePolicy);
        LE_SENSOR_RWFT_MZ->setMinimumSize(QSize(60, 20));
        LE_SENSOR_RWFT_MZ->setMaximumSize(QSize(60, 20));
        LE_SENSOR_RWFT_MZ->setFont(font2);
        LE_SENSOR_RWFT_MZ->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        LE_SENSOR_RWFT_MZ->setReadOnly(true);

        gridLayout_12->addWidget(LE_SENSOR_RWFT_MZ, 1, 3, 1, 1);

        LE_SENSOR_RAFT_MZ = new QLineEdit(gridLayoutWidget_12);
        LE_SENSOR_RAFT_MZ->setObjectName(QStringLiteral("LE_SENSOR_RAFT_MZ"));
        LE_SENSOR_RAFT_MZ->setEnabled(true);
        sizePolicy.setHeightForWidth(LE_SENSOR_RAFT_MZ->sizePolicy().hasHeightForWidth());
        LE_SENSOR_RAFT_MZ->setSizePolicy(sizePolicy);
        LE_SENSOR_RAFT_MZ->setMinimumSize(QSize(60, 20));
        LE_SENSOR_RAFT_MZ->setMaximumSize(QSize(60, 20));
        LE_SENSOR_RAFT_MZ->setFont(font2);
        LE_SENSOR_RAFT_MZ->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        LE_SENSOR_RAFT_MZ->setReadOnly(true);

        gridLayout_12->addWidget(LE_SENSOR_RAFT_MZ, 3, 3, 1, 1);

        LE_SENSOR_LAFT_MZ = new QLineEdit(gridLayoutWidget_12);
        LE_SENSOR_LAFT_MZ->setObjectName(QStringLiteral("LE_SENSOR_LAFT_MZ"));
        LE_SENSOR_LAFT_MZ->setEnabled(true);
        sizePolicy.setHeightForWidth(LE_SENSOR_LAFT_MZ->sizePolicy().hasHeightForWidth());
        LE_SENSOR_LAFT_MZ->setSizePolicy(sizePolicy);
        LE_SENSOR_LAFT_MZ->setMinimumSize(QSize(60, 20));
        LE_SENSOR_LAFT_MZ->setMaximumSize(QSize(60, 20));
        LE_SENSOR_LAFT_MZ->setFont(font2);
        LE_SENSOR_LAFT_MZ->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        LE_SENSOR_LAFT_MZ->setReadOnly(true);

        gridLayout_12->addWidget(LE_SENSOR_LAFT_MZ, 4, 3, 1, 1);

        label_61 = new QLabel(gridLayoutWidget_12);
        label_61->setObjectName(QStringLiteral("label_61"));
        label_61->setEnabled(true);
        sizePolicy.setHeightForWidth(label_61->sizePolicy().hasHeightForWidth());
        label_61->setSizePolicy(sizePolicy);
        label_61->setMinimumSize(QSize(60, 20));
        label_61->setMaximumSize(QSize(60, 20));
        label_61->setFont(font2);
        label_61->setAlignment(Qt::AlignCenter);

        gridLayout_12->addWidget(label_61, 0, 4, 1, 1);

        LE_SENSOR_RWFT_FY = new QLineEdit(gridLayoutWidget_12);
        LE_SENSOR_RWFT_FY->setObjectName(QStringLiteral("LE_SENSOR_RWFT_FY"));
        LE_SENSOR_RWFT_FY->setEnabled(true);
        sizePolicy.setHeightForWidth(LE_SENSOR_RWFT_FY->sizePolicy().hasHeightForWidth());
        LE_SENSOR_RWFT_FY->setSizePolicy(sizePolicy);
        LE_SENSOR_RWFT_FY->setMinimumSize(QSize(60, 20));
        LE_SENSOR_RWFT_FY->setMaximumSize(QSize(60, 20));
        LE_SENSOR_RWFT_FY->setFont(font2);
        LE_SENSOR_RWFT_FY->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        LE_SENSOR_RWFT_FY->setReadOnly(true);

        gridLayout_12->addWidget(LE_SENSOR_RWFT_FY, 1, 5, 1, 1);

        LE_SENSOR_RWFT_FX = new QLineEdit(gridLayoutWidget_12);
        LE_SENSOR_RWFT_FX->setObjectName(QStringLiteral("LE_SENSOR_RWFT_FX"));
        LE_SENSOR_RWFT_FX->setEnabled(true);
        sizePolicy.setHeightForWidth(LE_SENSOR_RWFT_FX->sizePolicy().hasHeightForWidth());
        LE_SENSOR_RWFT_FX->setSizePolicy(sizePolicy);
        LE_SENSOR_RWFT_FX->setMinimumSize(QSize(60, 20));
        LE_SENSOR_RWFT_FX->setMaximumSize(QSize(60, 20));
        LE_SENSOR_RWFT_FX->setFont(font2);
        LE_SENSOR_RWFT_FX->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        LE_SENSOR_RWFT_FX->setReadOnly(true);

        gridLayout_12->addWidget(LE_SENSOR_RWFT_FX, 1, 4, 1, 1);

        LE_SENSOR_LWFT_FY = new QLineEdit(gridLayoutWidget_12);
        LE_SENSOR_LWFT_FY->setObjectName(QStringLiteral("LE_SENSOR_LWFT_FY"));
        LE_SENSOR_LWFT_FY->setEnabled(true);
        sizePolicy.setHeightForWidth(LE_SENSOR_LWFT_FY->sizePolicy().hasHeightForWidth());
        LE_SENSOR_LWFT_FY->setSizePolicy(sizePolicy);
        LE_SENSOR_LWFT_FY->setMinimumSize(QSize(60, 20));
        LE_SENSOR_LWFT_FY->setMaximumSize(QSize(60, 20));
        LE_SENSOR_LWFT_FY->setFont(font2);
        LE_SENSOR_LWFT_FY->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        LE_SENSOR_LWFT_FY->setReadOnly(true);

        gridLayout_12->addWidget(LE_SENSOR_LWFT_FY, 2, 5, 1, 1);

        LE_SENSOR_LWFT_FX = new QLineEdit(gridLayoutWidget_12);
        LE_SENSOR_LWFT_FX->setObjectName(QStringLiteral("LE_SENSOR_LWFT_FX"));
        LE_SENSOR_LWFT_FX->setEnabled(true);
        sizePolicy.setHeightForWidth(LE_SENSOR_LWFT_FX->sizePolicy().hasHeightForWidth());
        LE_SENSOR_LWFT_FX->setSizePolicy(sizePolicy);
        LE_SENSOR_LWFT_FX->setMinimumSize(QSize(60, 20));
        LE_SENSOR_LWFT_FX->setMaximumSize(QSize(60, 20));
        LE_SENSOR_LWFT_FX->setFont(font2);
        LE_SENSOR_LWFT_FX->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        LE_SENSOR_LWFT_FX->setReadOnly(true);

        gridLayout_12->addWidget(LE_SENSOR_LWFT_FX, 2, 4, 1, 1);

        LE_SENSOR_RAFT_FY = new QLineEdit(gridLayoutWidget_12);
        LE_SENSOR_RAFT_FY->setObjectName(QStringLiteral("LE_SENSOR_RAFT_FY"));
        LE_SENSOR_RAFT_FY->setEnabled(true);
        sizePolicy.setHeightForWidth(LE_SENSOR_RAFT_FY->sizePolicy().hasHeightForWidth());
        LE_SENSOR_RAFT_FY->setSizePolicy(sizePolicy);
        LE_SENSOR_RAFT_FY->setMinimumSize(QSize(60, 20));
        LE_SENSOR_RAFT_FY->setMaximumSize(QSize(60, 20));
        LE_SENSOR_RAFT_FY->setFont(font2);
        LE_SENSOR_RAFT_FY->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        LE_SENSOR_RAFT_FY->setReadOnly(true);

        gridLayout_12->addWidget(LE_SENSOR_RAFT_FY, 3, 5, 1, 1);

        LE_SENSOR_RAFT_FX = new QLineEdit(gridLayoutWidget_12);
        LE_SENSOR_RAFT_FX->setObjectName(QStringLiteral("LE_SENSOR_RAFT_FX"));
        LE_SENSOR_RAFT_FX->setEnabled(true);
        sizePolicy.setHeightForWidth(LE_SENSOR_RAFT_FX->sizePolicy().hasHeightForWidth());
        LE_SENSOR_RAFT_FX->setSizePolicy(sizePolicy);
        LE_SENSOR_RAFT_FX->setMinimumSize(QSize(60, 20));
        LE_SENSOR_RAFT_FX->setMaximumSize(QSize(60, 20));
        LE_SENSOR_RAFT_FX->setFont(font2);
        LE_SENSOR_RAFT_FX->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        LE_SENSOR_RAFT_FX->setReadOnly(true);

        gridLayout_12->addWidget(LE_SENSOR_RAFT_FX, 3, 4, 1, 1);

        LE_SENSOR_LAFT_FY = new QLineEdit(gridLayoutWidget_12);
        LE_SENSOR_LAFT_FY->setObjectName(QStringLiteral("LE_SENSOR_LAFT_FY"));
        LE_SENSOR_LAFT_FY->setEnabled(true);
        sizePolicy.setHeightForWidth(LE_SENSOR_LAFT_FY->sizePolicy().hasHeightForWidth());
        LE_SENSOR_LAFT_FY->setSizePolicy(sizePolicy);
        LE_SENSOR_LAFT_FY->setMinimumSize(QSize(60, 20));
        LE_SENSOR_LAFT_FY->setMaximumSize(QSize(60, 20));
        LE_SENSOR_LAFT_FY->setFont(font2);
        LE_SENSOR_LAFT_FY->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        LE_SENSOR_LAFT_FY->setReadOnly(true);

        gridLayout_12->addWidget(LE_SENSOR_LAFT_FY, 4, 5, 1, 1);

        LE_SENSOR_LAFT_FX = new QLineEdit(gridLayoutWidget_12);
        LE_SENSOR_LAFT_FX->setObjectName(QStringLiteral("LE_SENSOR_LAFT_FX"));
        LE_SENSOR_LAFT_FX->setEnabled(true);
        sizePolicy.setHeightForWidth(LE_SENSOR_LAFT_FX->sizePolicy().hasHeightForWidth());
        LE_SENSOR_LAFT_FX->setSizePolicy(sizePolicy);
        LE_SENSOR_LAFT_FX->setMinimumSize(QSize(60, 20));
        LE_SENSOR_LAFT_FX->setMaximumSize(QSize(60, 20));
        LE_SENSOR_LAFT_FX->setFont(font2);
        LE_SENSOR_LAFT_FX->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        LE_SENSOR_LAFT_FX->setReadOnly(true);

        gridLayout_12->addWidget(LE_SENSOR_LAFT_FX, 4, 4, 1, 1);

        LE_SENSOR_LWFT_FZ = new QLineEdit(gridLayoutWidget_12);
        LE_SENSOR_LWFT_FZ->setObjectName(QStringLiteral("LE_SENSOR_LWFT_FZ"));
        LE_SENSOR_LWFT_FZ->setEnabled(true);
        sizePolicy.setHeightForWidth(LE_SENSOR_LWFT_FZ->sizePolicy().hasHeightForWidth());
        LE_SENSOR_LWFT_FZ->setSizePolicy(sizePolicy);
        LE_SENSOR_LWFT_FZ->setMinimumSize(QSize(60, 20));
        LE_SENSOR_LWFT_FZ->setMaximumSize(QSize(60, 20));
        LE_SENSOR_LWFT_FZ->setFont(font2);
        LE_SENSOR_LWFT_FZ->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        LE_SENSOR_LWFT_FZ->setReadOnly(true);

        gridLayout_12->addWidget(LE_SENSOR_LWFT_FZ, 2, 6, 1, 1);

        label_62 = new QLabel(groupBox);
        label_62->setObjectName(QStringLiteral("label_62"));
        label_62->setEnabled(true);
        label_62->setGeometry(QRect(10, 40, 101, 16));
        QFont font3;
        font3.setPointSize(9);
        font3.setBold(true);
        font3.setWeight(75);
        label_62->setFont(font3);
        label_62->setAlignment(Qt::AlignLeading|Qt::AlignLeft|Qt::AlignVCenter);
        gridLayoutWidget_15 = new QWidget(groupBox);
        gridLayoutWidget_15->setObjectName(QStringLiteral("gridLayoutWidget_15"));
        gridLayoutWidget_15->setEnabled(true);
        gridLayoutWidget_15->setGeometry(QRect(10, 230, 251, 74));
        gridLayoutWidget_15->setFont(font2);
        gridLayout_15 = new QGridLayout(gridLayoutWidget_15);
        gridLayout_15->setObjectName(QStringLiteral("gridLayout_15"));
        gridLayout_15->setContentsMargins(0, 0, 0, 0);
        LE_SENSOR_CIMU_PITCH = new QLineEdit(gridLayoutWidget_15);
        LE_SENSOR_CIMU_PITCH->setObjectName(QStringLiteral("LE_SENSOR_CIMU_PITCH"));
        LE_SENSOR_CIMU_PITCH->setEnabled(true);
        sizePolicy.setHeightForWidth(LE_SENSOR_CIMU_PITCH->sizePolicy().hasHeightForWidth());
        LE_SENSOR_CIMU_PITCH->setSizePolicy(sizePolicy);
        LE_SENSOR_CIMU_PITCH->setMinimumSize(QSize(60, 20));
        LE_SENSOR_CIMU_PITCH->setMaximumSize(QSize(60, 20));
        LE_SENSOR_CIMU_PITCH->setFont(font2);
        LE_SENSOR_CIMU_PITCH->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        LE_SENSOR_CIMU_PITCH->setReadOnly(true);

        gridLayout_15->addWidget(LE_SENSOR_CIMU_PITCH, 3, 1, 1, 1);

        LE_SENSOR_CIMU_PITCH_VEL = new QLineEdit(gridLayoutWidget_15);
        LE_SENSOR_CIMU_PITCH_VEL->setObjectName(QStringLiteral("LE_SENSOR_CIMU_PITCH_VEL"));
        LE_SENSOR_CIMU_PITCH_VEL->setEnabled(true);
        sizePolicy.setHeightForWidth(LE_SENSOR_CIMU_PITCH_VEL->sizePolicy().hasHeightForWidth());
        LE_SENSOR_CIMU_PITCH_VEL->setSizePolicy(sizePolicy);
        LE_SENSOR_CIMU_PITCH_VEL->setMinimumSize(QSize(60, 20));
        LE_SENSOR_CIMU_PITCH_VEL->setMaximumSize(QSize(60, 20));
        LE_SENSOR_CIMU_PITCH_VEL->setFont(font2);
        LE_SENSOR_CIMU_PITCH_VEL->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        LE_SENSOR_CIMU_PITCH_VEL->setReadOnly(true);

        gridLayout_15->addWidget(LE_SENSOR_CIMU_PITCH_VEL, 3, 2, 1, 1);

        label_53 = new QLabel(gridLayoutWidget_15);
        label_53->setObjectName(QStringLiteral("label_53"));
        label_53->setEnabled(true);
        sizePolicy.setHeightForWidth(label_53->sizePolicy().hasHeightForWidth());
        label_53->setSizePolicy(sizePolicy);
        label_53->setMinimumSize(QSize(50, 20));
        label_53->setMaximumSize(QSize(50, 20));
        label_53->setFont(font2);
        label_53->setAlignment(Qt::AlignCenter);

        gridLayout_15->addWidget(label_53, 3, 0, 1, 1);

        LE_SENSOR_CIMU_PITCH_ACC = new QLineEdit(gridLayoutWidget_15);
        LE_SENSOR_CIMU_PITCH_ACC->setObjectName(QStringLiteral("LE_SENSOR_CIMU_PITCH_ACC"));
        LE_SENSOR_CIMU_PITCH_ACC->setEnabled(true);
        sizePolicy.setHeightForWidth(LE_SENSOR_CIMU_PITCH_ACC->sizePolicy().hasHeightForWidth());
        LE_SENSOR_CIMU_PITCH_ACC->setSizePolicy(sizePolicy);
        LE_SENSOR_CIMU_PITCH_ACC->setMinimumSize(QSize(60, 20));
        LE_SENSOR_CIMU_PITCH_ACC->setMaximumSize(QSize(60, 20));
        LE_SENSOR_CIMU_PITCH_ACC->setFont(font2);
        LE_SENSOR_CIMU_PITCH_ACC->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        LE_SENSOR_CIMU_PITCH_ACC->setReadOnly(true);

        gridLayout_15->addWidget(LE_SENSOR_CIMU_PITCH_ACC, 3, 3, 1, 1);

        label_60 = new QLabel(gridLayoutWidget_15);
        label_60->setObjectName(QStringLiteral("label_60"));
        label_60->setEnabled(true);
        sizePolicy.setHeightForWidth(label_60->sizePolicy().hasHeightForWidth());
        label_60->setSizePolicy(sizePolicy);
        label_60->setMinimumSize(QSize(60, 20));
        label_60->setMaximumSize(QSize(60, 20));
        label_60->setFont(font2);
        label_60->setAlignment(Qt::AlignCenter);

        gridLayout_15->addWidget(label_60, 0, 3, 1, 1);

        label_49 = new QLabel(gridLayoutWidget_15);
        label_49->setObjectName(QStringLiteral("label_49"));
        label_49->setEnabled(true);
        sizePolicy.setHeightForWidth(label_49->sizePolicy().hasHeightForWidth());
        label_49->setSizePolicy(sizePolicy);
        label_49->setMinimumSize(QSize(60, 20));
        label_49->setMaximumSize(QSize(60, 20));
        label_49->setFont(font2);
        label_49->setAlignment(Qt::AlignCenter);

        gridLayout_15->addWidget(label_49, 0, 2, 1, 1);

        label_52 = new QLabel(gridLayoutWidget_15);
        label_52->setObjectName(QStringLiteral("label_52"));
        label_52->setEnabled(true);
        sizePolicy.setHeightForWidth(label_52->sizePolicy().hasHeightForWidth());
        label_52->setSizePolicy(sizePolicy);
        label_52->setMinimumSize(QSize(60, 20));
        label_52->setMaximumSize(QSize(60, 20));
        label_52->setFont(font2);
        label_52->setAlignment(Qt::AlignCenter);

        gridLayout_15->addWidget(label_52, 0, 1, 1, 1);

        label_54 = new QLabel(gridLayoutWidget_15);
        label_54->setObjectName(QStringLiteral("label_54"));
        label_54->setEnabled(true);
        sizePolicy.setHeightForWidth(label_54->sizePolicy().hasHeightForWidth());
        label_54->setSizePolicy(sizePolicy);
        label_54->setMinimumSize(QSize(50, 20));
        label_54->setMaximumSize(QSize(50, 20));
        label_54->setFont(font2);
        label_54->setAlignment(Qt::AlignCenter);

        gridLayout_15->addWidget(label_54, 1, 0, 2, 1);

        LE_SENSOR_CIMU_ROLL = new QLineEdit(gridLayoutWidget_15);
        LE_SENSOR_CIMU_ROLL->setObjectName(QStringLiteral("LE_SENSOR_CIMU_ROLL"));
        LE_SENSOR_CIMU_ROLL->setEnabled(true);
        sizePolicy.setHeightForWidth(LE_SENSOR_CIMU_ROLL->sizePolicy().hasHeightForWidth());
        LE_SENSOR_CIMU_ROLL->setSizePolicy(sizePolicy);
        LE_SENSOR_CIMU_ROLL->setMinimumSize(QSize(60, 20));
        LE_SENSOR_CIMU_ROLL->setMaximumSize(QSize(60, 20));
        LE_SENSOR_CIMU_ROLL->setFont(font2);
        LE_SENSOR_CIMU_ROLL->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        LE_SENSOR_CIMU_ROLL->setReadOnly(true);

        gridLayout_15->addWidget(LE_SENSOR_CIMU_ROLL, 1, 1, 2, 1);

        LE_SENSOR_CIMU_ROLL_VEL = new QLineEdit(gridLayoutWidget_15);
        LE_SENSOR_CIMU_ROLL_VEL->setObjectName(QStringLiteral("LE_SENSOR_CIMU_ROLL_VEL"));
        LE_SENSOR_CIMU_ROLL_VEL->setEnabled(true);
        sizePolicy.setHeightForWidth(LE_SENSOR_CIMU_ROLL_VEL->sizePolicy().hasHeightForWidth());
        LE_SENSOR_CIMU_ROLL_VEL->setSizePolicy(sizePolicy);
        LE_SENSOR_CIMU_ROLL_VEL->setMinimumSize(QSize(60, 20));
        LE_SENSOR_CIMU_ROLL_VEL->setMaximumSize(QSize(60, 20));
        LE_SENSOR_CIMU_ROLL_VEL->setFont(font2);
        LE_SENSOR_CIMU_ROLL_VEL->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        LE_SENSOR_CIMU_ROLL_VEL->setReadOnly(true);

        gridLayout_15->addWidget(LE_SENSOR_CIMU_ROLL_VEL, 1, 2, 2, 1);

        LE_SENSOR_CIMU_ROLL_ACC = new QLineEdit(gridLayoutWidget_15);
        LE_SENSOR_CIMU_ROLL_ACC->setObjectName(QStringLiteral("LE_SENSOR_CIMU_ROLL_ACC"));
        LE_SENSOR_CIMU_ROLL_ACC->setEnabled(true);
        sizePolicy.setHeightForWidth(LE_SENSOR_CIMU_ROLL_ACC->sizePolicy().hasHeightForWidth());
        LE_SENSOR_CIMU_ROLL_ACC->setSizePolicy(sizePolicy);
        LE_SENSOR_CIMU_ROLL_ACC->setMinimumSize(QSize(60, 20));
        LE_SENSOR_CIMU_ROLL_ACC->setMaximumSize(QSize(60, 20));
        LE_SENSOR_CIMU_ROLL_ACC->setFont(font2);
        LE_SENSOR_CIMU_ROLL_ACC->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        LE_SENSOR_CIMU_ROLL_ACC->setReadOnly(true);

        gridLayout_15->addWidget(LE_SENSOR_CIMU_ROLL_ACC, 1, 3, 2, 1);

        label_63 = new QLabel(groupBox);
        label_63->setObjectName(QStringLiteral("label_63"));
        label_63->setEnabled(true);
        label_63->setGeometry(QRect(10, 210, 121, 16));
        label_63->setFont(font3);
        label_63->setAlignment(Qt::AlignLeading|Qt::AlignLeft|Qt::AlignVCenter);
        gridLayoutWidget_3 = new QWidget(groupBox);
        gridLayoutWidget_3->setObjectName(QStringLiteral("gridLayoutWidget_3"));
        gridLayoutWidget_3->setEnabled(true);
        gridLayoutWidget_3->setGeometry(QRect(270, 380, 194, 71));
        gridLayout_3 = new QGridLayout(gridLayoutWidget_3);
        gridLayout_3->setObjectName(QStringLiteral("gridLayout_3"));
        gridLayout_3->setContentsMargins(0, 0, 0, 0);
        LE_SENSOR_CIMU_PITCH_OFFSET = new QLineEdit(gridLayoutWidget_3);
        LE_SENSOR_CIMU_PITCH_OFFSET->setObjectName(QStringLiteral("LE_SENSOR_CIMU_PITCH_OFFSET"));
        LE_SENSOR_CIMU_PITCH_OFFSET->setEnabled(true);
        LE_SENSOR_CIMU_PITCH_OFFSET->setMinimumSize(QSize(55, 20));
        LE_SENSOR_CIMU_PITCH_OFFSET->setMaximumSize(QSize(55, 20));
        LE_SENSOR_CIMU_PITCH_OFFSET->setFont(font2);
        LE_SENSOR_CIMU_PITCH_OFFSET->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        LE_SENSOR_CIMU_PITCH_OFFSET->setReadOnly(false);

        gridLayout_3->addWidget(LE_SENSOR_CIMU_PITCH_OFFSET, 1, 1, 1, 1);

        label_55 = new QLabel(gridLayoutWidget_3);
        label_55->setObjectName(QStringLiteral("label_55"));
        label_55->setEnabled(true);
        sizePolicy.setHeightForWidth(label_55->sizePolicy().hasHeightForWidth());
        label_55->setSizePolicy(sizePolicy);
        label_55->setMinimumSize(QSize(55, 20));
        label_55->setMaximumSize(QSize(55, 20));
        label_55->setFont(font2);
        label_55->setAlignment(Qt::AlignCenter);

        gridLayout_3->addWidget(label_55, 1, 0, 1, 1);

        LE_SENSOR_CIMU_ROLL_OFFSET = new QLineEdit(gridLayoutWidget_3);
        LE_SENSOR_CIMU_ROLL_OFFSET->setObjectName(QStringLiteral("LE_SENSOR_CIMU_ROLL_OFFSET"));
        LE_SENSOR_CIMU_ROLL_OFFSET->setEnabled(true);
        LE_SENSOR_CIMU_ROLL_OFFSET->setMinimumSize(QSize(55, 20));
        LE_SENSOR_CIMU_ROLL_OFFSET->setMaximumSize(QSize(55, 20));
        LE_SENSOR_CIMU_ROLL_OFFSET->setFont(font2);
        LE_SENSOR_CIMU_ROLL_OFFSET->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        LE_SENSOR_CIMU_ROLL_OFFSET->setReadOnly(false);

        gridLayout_3->addWidget(LE_SENSOR_CIMU_ROLL_OFFSET, 0, 1, 1, 1);

        label_46 = new QLabel(gridLayoutWidget_3);
        label_46->setObjectName(QStringLiteral("label_46"));
        label_46->setEnabled(true);
        sizePolicy.setHeightForWidth(label_46->sizePolicy().hasHeightForWidth());
        label_46->setSizePolicy(sizePolicy);
        label_46->setMinimumSize(QSize(55, 20));
        label_46->setMaximumSize(QSize(55, 20));
        label_46->setFont(font2);
        label_46->setAlignment(Qt::AlignCenter);

        gridLayout_3->addWidget(label_46, 0, 0, 1, 1);

        BTN_CIMU_GET_OFFSET = new QPushButton(gridLayoutWidget_3);
        BTN_CIMU_GET_OFFSET->setObjectName(QStringLiteral("BTN_CIMU_GET_OFFSET"));
        BTN_CIMU_GET_OFFSET->setEnabled(true);
        BTN_CIMU_GET_OFFSET->setMinimumSize(QSize(70, 0));
        BTN_CIMU_GET_OFFSET->setMaximumSize(QSize(70, 16777215));
        BTN_CIMU_GET_OFFSET->setFont(font2);

        gridLayout_3->addWidget(BTN_CIMU_GET_OFFSET, 0, 2, 1, 1);

        BTN_CIMU_SET_OFFSET = new QPushButton(gridLayoutWidget_3);
        BTN_CIMU_SET_OFFSET->setObjectName(QStringLiteral("BTN_CIMU_SET_OFFSET"));
        BTN_CIMU_SET_OFFSET->setEnabled(true);
        BTN_CIMU_SET_OFFSET->setMinimumSize(QSize(70, 0));
        BTN_CIMU_SET_OFFSET->setMaximumSize(QSize(70, 16777215));
        BTN_CIMU_SET_OFFSET->setFont(font2);

        gridLayout_3->addWidget(BTN_CIMU_SET_OFFSET, 1, 2, 1, 1);

        label_65 = new QLabel(groupBox);
        label_65->setObjectName(QStringLiteral("label_65"));
        label_65->setEnabled(true);
        label_65->setGeometry(QRect(10, 330, 121, 16));
        label_65->setFont(font3);
        label_65->setAlignment(Qt::AlignLeading|Qt::AlignLeft|Qt::AlignVCenter);
        gridLayoutWidget_14 = new QWidget(groupBox);
        gridLayoutWidget_14->setObjectName(QStringLiteral("gridLayoutWidget_14"));
        gridLayoutWidget_14->setEnabled(true);
        gridLayoutWidget_14->setGeometry(QRect(10, 350, 204, 101));
        gridLayoutWidget_14->setFont(font2);
        gridLayout_14 = new QGridLayout(gridLayoutWidget_14);
        gridLayout_14->setObjectName(QStringLiteral("gridLayout_14"));
        gridLayout_14->setContentsMargins(0, 0, 0, 0);
        LE_ZMP_LY = new QLineEdit(gridLayoutWidget_14);
        LE_ZMP_LY->setObjectName(QStringLiteral("LE_ZMP_LY"));
        LE_ZMP_LY->setEnabled(true);
        LE_ZMP_LY->setMinimumSize(QSize(70, 20));
        LE_ZMP_LY->setMaximumSize(QSize(70, 20));
        LE_ZMP_LY->setFont(font2);
        LE_ZMP_LY->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        LE_ZMP_LY->setReadOnly(true);

        gridLayout_14->addWidget(LE_ZMP_LY, 2, 2, 1, 1);

        LE_ZMP_LX = new QLineEdit(gridLayoutWidget_14);
        LE_ZMP_LX->setObjectName(QStringLiteral("LE_ZMP_LX"));
        LE_ZMP_LX->setEnabled(true);
        LE_ZMP_LX->setMinimumSize(QSize(70, 20));
        LE_ZMP_LX->setMaximumSize(QSize(70, 20));
        LE_ZMP_LX->setFont(font2);
        LE_ZMP_LX->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        LE_ZMP_LX->setReadOnly(true);

        gridLayout_14->addWidget(LE_ZMP_LX, 2, 1, 1, 1);

        label_51 = new QLabel(gridLayoutWidget_14);
        label_51->setObjectName(QStringLiteral("label_51"));
        label_51->setEnabled(true);
        sizePolicy.setHeightForWidth(label_51->sizePolicy().hasHeightForWidth());
        label_51->setSizePolicy(sizePolicy);
        label_51->setMinimumSize(QSize(50, 20));
        label_51->setMaximumSize(QSize(50, 20));
        label_51->setFont(font2);
        label_51->setAlignment(Qt::AlignCenter);

        gridLayout_14->addWidget(label_51, 1, 0, 1, 1);

        LE_ZMP_RY = new QLineEdit(gridLayoutWidget_14);
        LE_ZMP_RY->setObjectName(QStringLiteral("LE_ZMP_RY"));
        LE_ZMP_RY->setEnabled(true);
        LE_ZMP_RY->setMinimumSize(QSize(70, 20));
        LE_ZMP_RY->setMaximumSize(QSize(70, 20));
        LE_ZMP_RY->setFont(font2);
        LE_ZMP_RY->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        LE_ZMP_RY->setReadOnly(true);

        gridLayout_14->addWidget(LE_ZMP_RY, 1, 2, 1, 1);

        label_50 = new QLabel(gridLayoutWidget_14);
        label_50->setObjectName(QStringLiteral("label_50"));
        label_50->setEnabled(true);
        label_50->setMinimumSize(QSize(70, 20));
        label_50->setMaximumSize(QSize(70, 20));
        label_50->setFont(font2);
        label_50->setAlignment(Qt::AlignCenter);

        gridLayout_14->addWidget(label_50, 0, 1, 1, 1);

        label_56 = new QLabel(gridLayoutWidget_14);
        label_56->setObjectName(QStringLiteral("label_56"));
        label_56->setEnabled(true);
        label_56->setMinimumSize(QSize(70, 20));
        label_56->setMaximumSize(QSize(70, 20));
        label_56->setFont(font2);
        label_56->setAlignment(Qt::AlignCenter);

        gridLayout_14->addWidget(label_56, 0, 2, 1, 1);

        LE_ZMP_RX = new QLineEdit(gridLayoutWidget_14);
        LE_ZMP_RX->setObjectName(QStringLiteral("LE_ZMP_RX"));
        LE_ZMP_RX->setEnabled(true);
        LE_ZMP_RX->setMinimumSize(QSize(70, 20));
        LE_ZMP_RX->setMaximumSize(QSize(70, 20));
        LE_ZMP_RX->setFont(font2);
        LE_ZMP_RX->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        LE_ZMP_RX->setReadOnly(true);

        gridLayout_14->addWidget(LE_ZMP_RX, 1, 1, 1, 1);

        label_57 = new QLabel(gridLayoutWidget_14);
        label_57->setObjectName(QStringLiteral("label_57"));
        label_57->setEnabled(true);
        sizePolicy.setHeightForWidth(label_57->sizePolicy().hasHeightForWidth());
        label_57->setSizePolicy(sizePolicy);
        label_57->setMinimumSize(QSize(50, 20));
        label_57->setMaximumSize(QSize(50, 20));
        label_57->setFont(font2);
        label_57->setAlignment(Qt::AlignCenter);

        gridLayout_14->addWidget(label_57, 2, 0, 1, 1);

        LE_ZMP_WX = new QLineEdit(gridLayoutWidget_14);
        LE_ZMP_WX->setObjectName(QStringLiteral("LE_ZMP_WX"));
        LE_ZMP_WX->setEnabled(true);
        LE_ZMP_WX->setMinimumSize(QSize(70, 20));
        LE_ZMP_WX->setMaximumSize(QSize(70, 20));
        LE_ZMP_WX->setFont(font2);
        LE_ZMP_WX->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        LE_ZMP_WX->setReadOnly(true);

        gridLayout_14->addWidget(LE_ZMP_WX, 3, 1, 1, 1);

        LE_ZMP_WY = new QLineEdit(gridLayoutWidget_14);
        LE_ZMP_WY->setObjectName(QStringLiteral("LE_ZMP_WY"));
        LE_ZMP_WY->setEnabled(true);
        LE_ZMP_WY->setMinimumSize(QSize(70, 20));
        LE_ZMP_WY->setMaximumSize(QSize(70, 20));
        LE_ZMP_WY->setFont(font2);
        LE_ZMP_WY->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        LE_ZMP_WY->setReadOnly(true);

        gridLayout_14->addWidget(LE_ZMP_WY, 3, 2, 1, 1);

        label_58 = new QLabel(gridLayoutWidget_14);
        label_58->setObjectName(QStringLiteral("label_58"));
        label_58->setEnabled(true);
        sizePolicy.setHeightForWidth(label_58->sizePolicy().hasHeightForWidth());
        label_58->setSizePolicy(sizePolicy);
        label_58->setMinimumSize(QSize(50, 20));
        label_58->setMaximumSize(QSize(50, 20));
        label_58->setFont(font2);
        label_58->setAlignment(Qt::AlignCenter);

        gridLayout_14->addWidget(label_58, 3, 0, 1, 1);

        line_2 = new QFrame(groupBox);
        line_2->setObjectName(QStringLiteral("line_2"));
        line_2->setEnabled(true);
        line_2->setGeometry(QRect(10, 200, 118, 3));
        line_2->setFrameShape(QFrame::HLine);
        line_2->setFrameShadow(QFrame::Sunken);
        line_3 = new QFrame(groupBox);
        line_3->setObjectName(QStringLiteral("line_3"));
        line_3->setEnabled(true);
        line_3->setGeometry(QRect(10, 320, 118, 3));
        line_3->setFrameShape(QFrame::HLine);
        line_3->setFrameShadow(QFrame::Sunken);
        BTN_NEW_IMU_ENABLE = new QPushButton(groupBox);
        BTN_NEW_IMU_ENABLE->setObjectName(QStringLiteral("BTN_NEW_IMU_ENABLE"));
        BTN_NEW_IMU_ENABLE->setGeometry(QRect(270, 230, 80, 22));
        BTN_NEW_IMU_NULL = new QPushButton(groupBox);
        BTN_NEW_IMU_NULL->setObjectName(QStringLiteral("BTN_NEW_IMU_NULL"));
        BTN_NEW_IMU_NULL->setGeometry(QRect(270, 260, 80, 22));
        BTN_NEW_IMU_RESET = new QPushButton(groupBox);
        BTN_NEW_IMU_RESET->setObjectName(QStringLiteral("BTN_NEW_IMU_RESET"));
        BTN_NEW_IMU_RESET->setGeometry(QRect(270, 290, 80, 22));
        label = new QLabel(SensorDialog);
        label->setObjectName(QStringLiteral("label"));
        label->setEnabled(true);
        label->setGeometry(QRect(10, 2, 131, 31));
        QFont font4;
        font4.setPointSize(12);
        font4.setBold(true);
        font4.setWeight(75);
        label->setFont(font4);
        line = new QFrame(SensorDialog);
        line->setObjectName(QStringLiteral("line"));
        line->setEnabled(true);
        line->setGeometry(QRect(10, 32, 118, 3));
        line->setFont(font2);
        line->setLineWidth(2);
        line->setFrameShape(QFrame::HLine);
        line->setFrameShadow(QFrame::Sunken);
        BTN_SENSOR_FT_NULL = new QPushButton(SensorDialog);
        BTN_SENSOR_FT_NULL->setObjectName(QStringLiteral("BTN_SENSOR_FT_NULL"));
        BTN_SENSOR_FT_NULL->setEnabled(true);
        BTN_SENSOR_FT_NULL->setGeometry(QRect(330, 10, 71, 31));
        BTN_SENSOR_FT_NULL->setFont(font2);
        BTN_SENSOR_IMU_NULL = new QPushButton(SensorDialog);
        BTN_SENSOR_IMU_NULL->setObjectName(QStringLiteral("BTN_SENSOR_IMU_NULL"));
        BTN_SENSOR_IMU_NULL->setEnabled(true);
        BTN_SENSOR_IMU_NULL->setGeometry(QRect(410, 10, 71, 31));
        BTN_SENSOR_IMU_NULL->setFont(font2);
        BTN_SENSOR_ENABLE = new QPushButton(SensorDialog);
        BTN_SENSOR_ENABLE->setObjectName(QStringLiteral("BTN_SENSOR_ENABLE"));
        BTN_SENSOR_ENABLE->setEnabled(true);
        BTN_SENSOR_ENABLE->setGeometry(QRect(130, 10, 71, 31));
        BTN_SENSOR_ENABLE->setFont(font2);
        BTN_SENSOR_DISABLE = new QPushButton(SensorDialog);
        BTN_SENSOR_DISABLE->setObjectName(QStringLiteral("BTN_SENSOR_DISABLE"));
        BTN_SENSOR_DISABLE->setEnabled(true);
        BTN_SENSOR_DISABLE->setGeometry(QRect(210, 10, 71, 31));
        BTN_SENSOR_DISABLE->setFont(font2);

        retranslateUi(SensorDialog);

        QMetaObject::connectSlotsByName(SensorDialog);
    } // setupUi

    void retranslateUi(QDialog *SensorDialog)
    {
        SensorDialog->setWindowTitle(QApplication::translate("SensorDialog", "Dialog", Q_NULLPTR));
        groupBox->setTitle(QApplication::translate("SensorDialog", "Sensor Data", Q_NULLPTR));
        label_43->setText(QApplication::translate("SensorDialog", "Fy", Q_NULLPTR));
        label_32->setText(QApplication::translate("SensorDialog", "Mz", Q_NULLPTR));
        label_44->setText(QApplication::translate("SensorDialog", "Mx", Q_NULLPTR));
        label_45->setText(QApplication::translate("SensorDialog", "RWFT", Q_NULLPTR));
        label_39->setText(QApplication::translate("SensorDialog", "LWFT", Q_NULLPTR));
        label_47->setText(QApplication::translate("SensorDialog", "RAFT", Q_NULLPTR));
        label_48->setText(QApplication::translate("SensorDialog", "LAFT", Q_NULLPTR));
        label_42->setText(QApplication::translate("SensorDialog", "Fz", Q_NULLPTR));
        label_31->setText(QApplication::translate("SensorDialog", "My", Q_NULLPTR));
        label_61->setText(QApplication::translate("SensorDialog", "Fx", Q_NULLPTR));
        label_62->setText(QApplication::translate("SensorDialog", "FT(Mx,My,Fz)", Q_NULLPTR));
        label_53->setText(QApplication::translate("SensorDialog", "Pitch", Q_NULLPTR));
        label_60->setText(QApplication::translate("SensorDialog", "ACC ", Q_NULLPTR));
        label_49->setText(QApplication::translate("SensorDialog", "Gyro", Q_NULLPTR));
        label_52->setText(QApplication::translate("SensorDialog", "Angle", Q_NULLPTR));
        label_54->setText(QApplication::translate("SensorDialog", "Roll", Q_NULLPTR));
        label_63->setText(QApplication::translate("SensorDialog", "IMU(Roll,Pitch,Vel)", Q_NULLPTR));
        label_55->setText(QApplication::translate("SensorDialog", "Pitch Off.", Q_NULLPTR));
        label_46->setText(QApplication::translate("SensorDialog", "Roll Off.", Q_NULLPTR));
        BTN_CIMU_GET_OFFSET->setText(QApplication::translate("SensorDialog", "Get Off.", Q_NULLPTR));
        BTN_CIMU_SET_OFFSET->setText(QApplication::translate("SensorDialog", "Set Off.", Q_NULLPTR));
        label_65->setText(QApplication::translate("SensorDialog", "ZMP", Q_NULLPTR));
        label_51->setText(QApplication::translate("SensorDialog", "Right", Q_NULLPTR));
        label_50->setText(QApplication::translate("SensorDialog", "X", Q_NULLPTR));
        label_56->setText(QApplication::translate("SensorDialog", "Y", Q_NULLPTR));
        label_57->setText(QApplication::translate("SensorDialog", "Left", Q_NULLPTR));
        label_58->setText(QApplication::translate("SensorDialog", "Whole", Q_NULLPTR));
        BTN_NEW_IMU_ENABLE->setText(QApplication::translate("SensorDialog", "Enable", Q_NULLPTR));
        BTN_NEW_IMU_NULL->setText(QApplication::translate("SensorDialog", "Nulling", Q_NULLPTR));
        BTN_NEW_IMU_RESET->setText(QApplication::translate("SensorDialog", "Reset", Q_NULLPTR));
        label->setText(QApplication::translate("SensorDialog", "Sensor", Q_NULLPTR));
        BTN_SENSOR_FT_NULL->setText(QApplication::translate("SensorDialog", "FT NULL", Q_NULLPTR));
        BTN_SENSOR_IMU_NULL->setText(QApplication::translate("SensorDialog", "IMU NULL", Q_NULLPTR));
        BTN_SENSOR_ENABLE->setText(QApplication::translate("SensorDialog", "Enable", Q_NULLPTR));
        BTN_SENSOR_DISABLE->setText(QApplication::translate("SensorDialog", "Disable", Q_NULLPTR));
    } // retranslateUi

};

namespace Ui {
    class SensorDialog: public Ui_SensorDialog {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_SENSORDIALOG_H
