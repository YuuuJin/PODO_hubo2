/****************************************************************************
** Meta object code from reading C++ file 'mpcwalkingdialog.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.8.0)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../src/PODOGUI/mpcwalkingdialog.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'mpcwalkingdialog.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.8.0. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_MPCWalkingDialog_t {
    QByteArrayData data[11];
    char stringdata0[220];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_MPCWalkingDialog_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_MPCWalkingDialog_t qt_meta_stringdata_MPCWalkingDialog = {
    {
QT_MOC_LITERAL(0, 0, 16), // "MPCWalkingDialog"
QT_MOC_LITERAL(1, 17, 25), // "on_BTN_Walk_Ready_clicked"
QT_MOC_LITERAL(2, 43, 0), // ""
QT_MOC_LITERAL(3, 44, 22), // "on_BTN_FORWARD_clicked"
QT_MOC_LITERAL(4, 67, 19), // "on_BTN_LEFT_clicked"
QT_MOC_LITERAL(5, 87, 20), // "on_BTN_RIGHT_clicked"
QT_MOC_LITERAL(6, 108, 18), // "on_BTN_CCW_clicked"
QT_MOC_LITERAL(7, 127, 17), // "on_BTN_CW_clicked"
QT_MOC_LITERAL(8, 145, 25), // "on_BTN_SAVE_START_clicked"
QT_MOC_LITERAL(9, 171, 24), // "on_BTN_DATA_SAVE_clicked"
QT_MOC_LITERAL(10, 196, 23) // "on_pushButton_8_clicked"

    },
    "MPCWalkingDialog\0on_BTN_Walk_Ready_clicked\0"
    "\0on_BTN_FORWARD_clicked\0on_BTN_LEFT_clicked\0"
    "on_BTN_RIGHT_clicked\0on_BTN_CCW_clicked\0"
    "on_BTN_CW_clicked\0on_BTN_SAVE_START_clicked\0"
    "on_BTN_DATA_SAVE_clicked\0"
    "on_pushButton_8_clicked"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_MPCWalkingDialog[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
       9,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    0,   59,    2, 0x08 /* Private */,
       3,    0,   60,    2, 0x08 /* Private */,
       4,    0,   61,    2, 0x08 /* Private */,
       5,    0,   62,    2, 0x08 /* Private */,
       6,    0,   63,    2, 0x08 /* Private */,
       7,    0,   64,    2, 0x08 /* Private */,
       8,    0,   65,    2, 0x08 /* Private */,
       9,    0,   66,    2, 0x08 /* Private */,
      10,    0,   67,    2, 0x08 /* Private */,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,

       0        // eod
};

void MPCWalkingDialog::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        MPCWalkingDialog *_t = static_cast<MPCWalkingDialog *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->on_BTN_Walk_Ready_clicked(); break;
        case 1: _t->on_BTN_FORWARD_clicked(); break;
        case 2: _t->on_BTN_LEFT_clicked(); break;
        case 3: _t->on_BTN_RIGHT_clicked(); break;
        case 4: _t->on_BTN_CCW_clicked(); break;
        case 5: _t->on_BTN_CW_clicked(); break;
        case 6: _t->on_BTN_SAVE_START_clicked(); break;
        case 7: _t->on_BTN_DATA_SAVE_clicked(); break;
        case 8: _t->on_pushButton_8_clicked(); break;
        default: ;
        }
    }
    Q_UNUSED(_a);
}

const QMetaObject MPCWalkingDialog::staticMetaObject = {
    { &QDialog::staticMetaObject, qt_meta_stringdata_MPCWalkingDialog.data,
      qt_meta_data_MPCWalkingDialog,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *MPCWalkingDialog::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *MPCWalkingDialog::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_MPCWalkingDialog.stringdata0))
        return static_cast<void*>(const_cast< MPCWalkingDialog*>(this));
    return QDialog::qt_metacast(_clname);
}

int MPCWalkingDialog::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QDialog::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 9)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 9;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 9)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 9;
    }
    return _id;
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
