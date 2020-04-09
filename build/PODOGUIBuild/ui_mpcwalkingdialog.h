/********************************************************************************
** Form generated from reading UI file 'mpcwalkingdialog.ui'
**
** Created by: Qt User Interface Compiler version 5.8.0
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MPCWALKINGDIALOG_H
#define UI_MPCWALKINGDIALOG_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QDialog>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QPushButton>

QT_BEGIN_NAMESPACE

class Ui_MPCWalkingDialog
{
public:
    QPushButton *BTN_Walk_Ready;
    QPushButton *BTN_FORWARD;
    QPushButton *BTN_SAVE_START;
    QPushButton *BTN_DATA_SAVE;
    QPushButton *BTN_CCW;
    QPushButton *BTN_CW;
    QPushButton *BTN_LEFT;
    QPushButton *BTN_RIGHT;
    QLineEdit *LE_STEP_NUM;
    QLineEdit *LE_STEP_LENGTH;
    QLineEdit *LE_STEP_ANGLE;
    QLineEdit *LE_STEP_OFFSET;
    QPushButton *pushButton_8;
    QPushButton *pushButton_9;
    QLineEdit *LE_WEIGHT;
    QLabel *Weight;

    void setupUi(QDialog *MPCWalkingDialog)
    {
        if (MPCWalkingDialog->objectName().isEmpty())
            MPCWalkingDialog->setObjectName(QStringLiteral("MPCWalkingDialog"));
        MPCWalkingDialog->resize(610, 622);
        BTN_Walk_Ready = new QPushButton(MPCWalkingDialog);
        BTN_Walk_Ready->setObjectName(QStringLiteral("BTN_Walk_Ready"));
        BTN_Walk_Ready->setGeometry(QRect(30, 20, 101, 41));
        BTN_FORWARD = new QPushButton(MPCWalkingDialog);
        BTN_FORWARD->setObjectName(QStringLiteral("BTN_FORWARD"));
        BTN_FORWARD->setGeometry(QRect(30, 80, 101, 41));
        BTN_SAVE_START = new QPushButton(MPCWalkingDialog);
        BTN_SAVE_START->setObjectName(QStringLiteral("BTN_SAVE_START"));
        BTN_SAVE_START->setGeometry(QRect(310, 190, 81, 41));
        BTN_DATA_SAVE = new QPushButton(MPCWalkingDialog);
        BTN_DATA_SAVE->setObjectName(QStringLiteral("BTN_DATA_SAVE"));
        BTN_DATA_SAVE->setGeometry(QRect(310, 249, 81, 41));
        BTN_CCW = new QPushButton(MPCWalkingDialog);
        BTN_CCW->setObjectName(QStringLiteral("BTN_CCW"));
        BTN_CCW->setGeometry(QRect(140, 180, 131, 41));
        BTN_CW = new QPushButton(MPCWalkingDialog);
        BTN_CW->setObjectName(QStringLiteral("BTN_CW"));
        BTN_CW->setGeometry(QRect(140, 250, 131, 41));
        BTN_LEFT = new QPushButton(MPCWalkingDialog);
        BTN_LEFT->setObjectName(QStringLiteral("BTN_LEFT"));
        BTN_LEFT->setGeometry(QRect(10, 180, 101, 41));
        BTN_RIGHT = new QPushButton(MPCWalkingDialog);
        BTN_RIGHT->setObjectName(QStringLiteral("BTN_RIGHT"));
        BTN_RIGHT->setGeometry(QRect(10, 250, 101, 41));
        LE_STEP_NUM = new QLineEdit(MPCWalkingDialog);
        LE_STEP_NUM->setObjectName(QStringLiteral("LE_STEP_NUM"));
        LE_STEP_NUM->setGeometry(QRect(190, 10, 71, 22));
        LE_STEP_LENGTH = new QLineEdit(MPCWalkingDialog);
        LE_STEP_LENGTH->setObjectName(QStringLiteral("LE_STEP_LENGTH"));
        LE_STEP_LENGTH->setGeometry(QRect(190, 50, 71, 22));
        LE_STEP_ANGLE = new QLineEdit(MPCWalkingDialog);
        LE_STEP_ANGLE->setObjectName(QStringLiteral("LE_STEP_ANGLE"));
        LE_STEP_ANGLE->setGeometry(QRect(190, 130, 71, 22));
        LE_STEP_OFFSET = new QLineEdit(MPCWalkingDialog);
        LE_STEP_OFFSET->setObjectName(QStringLiteral("LE_STEP_OFFSET"));
        LE_STEP_OFFSET->setGeometry(QRect(190, 90, 71, 22));
        pushButton_8 = new QPushButton(MPCWalkingDialog);
        pushButton_8->setObjectName(QStringLiteral("pushButton_8"));
        pushButton_8->setGeometry(QRect(300, 20, 91, 41));
        pushButton_9 = new QPushButton(MPCWalkingDialog);
        pushButton_9->setObjectName(QStringLiteral("pushButton_9"));
        pushButton_9->setGeometry(QRect(300, 80, 91, 41));
        LE_WEIGHT = new QLineEdit(MPCWalkingDialog);
        LE_WEIGHT->setObjectName(QStringLiteral("LE_WEIGHT"));
        LE_WEIGHT->setGeometry(QRect(100, 320, 71, 22));
        Weight = new QLabel(MPCWalkingDialog);
        Weight->setObjectName(QStringLiteral("Weight"));
        Weight->setGeometry(QRect(20, 320, 61, 21));

        retranslateUi(MPCWalkingDialog);

        QMetaObject::connectSlotsByName(MPCWalkingDialog);
    } // setupUi

    void retranslateUi(QDialog *MPCWalkingDialog)
    {
        MPCWalkingDialog->setWindowTitle(QApplication::translate("MPCWalkingDialog", "Dialog", Q_NULLPTR));
        BTN_Walk_Ready->setText(QApplication::translate("MPCWalkingDialog", "Walk Ready", Q_NULLPTR));
        BTN_FORWARD->setText(QApplication::translate("MPCWalkingDialog", "Forward", Q_NULLPTR));
        BTN_SAVE_START->setText(QApplication::translate("MPCWalkingDialog", "Save Start", Q_NULLPTR));
        BTN_DATA_SAVE->setText(QApplication::translate("MPCWalkingDialog", "Data Save", Q_NULLPTR));
        BTN_CCW->setText(QApplication::translate("MPCWalkingDialog", "++CCW LEFT Rotation", Q_NULLPTR));
        BTN_CW->setText(QApplication::translate("MPCWalkingDialog", "--CW Right Rotation", Q_NULLPTR));
        BTN_LEFT->setText(QApplication::translate("MPCWalkingDialog", "LEFT", Q_NULLPTR));
        BTN_RIGHT->setText(QApplication::translate("MPCWalkingDialog", "RIGHT", Q_NULLPTR));
        LE_STEP_NUM->setText(QApplication::translate("MPCWalkingDialog", "10", Q_NULLPTR));
        LE_STEP_LENGTH->setText(QApplication::translate("MPCWalkingDialog", "0.001", Q_NULLPTR));
        LE_STEP_ANGLE->setText(QApplication::translate("MPCWalkingDialog", "15", Q_NULLPTR));
        LE_STEP_OFFSET->setText(QApplication::translate("MPCWalkingDialog", "0.25", Q_NULLPTR));
        pushButton_8->setText(QApplication::translate("MPCWalkingDialog", "Joy stick on", Q_NULLPTR));
        pushButton_9->setText(QApplication::translate("MPCWalkingDialog", "Para Change", Q_NULLPTR));
        LE_WEIGHT->setText(QApplication::translate("MPCWalkingDialog", "230", Q_NULLPTR));
        Weight->setText(QApplication::translate("MPCWalkingDialog", "Weight", Q_NULLPTR));
    } // retranslateUi

};

namespace Ui {
    class MPCWalkingDialog: public Ui_MPCWalkingDialog {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MPCWALKINGDIALOG_H
