#ifndef MODELDIALOG_H
#define MODELDIALOG_H

#include <QDialog>
#include <QGLWidget>
#include <QtOpenGL>
#include <QSound>
#include <QString>


#include "isnl/opengl/glskeleton.h"
#include "CommonHeader.h"

static isnl::vec3 basecolor = isnl::vec3(1.0f, 1.0f, 1.0f);
GLSTL*  newStl(float x, float y, float z, std::string filename);

class QLabel;
class QMenu;
class QScrollArea;
class QSlider;
class GLWidget;


namespace Ui {
class ModelDialog;
}

class ModelDialog : public QDialog
{
    Q_OBJECT

public:
    explicit ModelDialog(QWidget *parent = 0);
    ~ModelDialog();

private:
    Ui::ModelDialog *ui;
    QTimer		*displayTimer;

    GLWidget	*glWidget;
    QLabel		*pixmapLabel;
    QSlider		*xSlider;
    QSlider		*ySlider;
    QSlider		*zSlider;

    QAction		*exitAct;
    QAction		*aboutAct;
    QAction		*aboutQtAct;
    QSound          *playdemoSound[150];

    QSlider *createSlider(const char *changedSignal, const char *setterSlot);

private slots:
    void DisplayUpdate();
    void on_BT_CamLeft_clicked();
    void on_BT_CamRight_clicked();
    void on_BT_CamFront_clicked();

    void on_BTN_SOUND_1_clicked();
    void on_BTN_SOUND_2_clicked();
    void on_BTN_SOUND_3_clicked();
    void on_BTN_SOUND_4_clicked();
    void on_BTN_SOUND_5_clicked();
    void on_BTN_SOUND_6_clicked();
    void on_BTN_SOUND_7_clicked();
    void on_BTN_SOUND_15_clicked();
    void on_BTN_SOUND_16_clicked();
    void on_BTN_SOUND_17_clicked();
    void on_BTN_SOUND_18_clicked();
    void on_BTN_SOUND_19_clicked();
    void on_BTN_SOUND_20_clicked();
    void on_BTN_SOUND_21_clicked();
    void on_BTN_SOUND_22_clicked();
    void on_BTN_SOUND_23_clicked();
    void on_BTN_SOUND_24_clicked();
    void on_BTN_SOUND_25_clicked();
    void on_BTN_SOUND_26_clicked();
    void on_BTN_SOUND_27_clicked();
    void on_BTN_SOUND_28_clicked();
    void on_BTN_SOUND_29_clicked();
    void on_BTN_SOUND_30_clicked();
    void on_BTN_SOUND_31_clicked();
    void on_BTN_SOUND_32_clicked();
    void on_BTN_SOUND_33_clicked();
    void on_BTN_SOUND_34_clicked();
    void on_BTN_SOUND_35_clicked();
    void on_BTN_SOUND_36_clicked();
    void on_BTN_SOUND_37_clicked();
    void on_BTN_SOUND_38_clicked();
    void on_BTN_SOUND_39_clicked();
    void on_BTN_SOUND_40_clicked();
    void on_BTN_SOUND_41_clicked();
    void on_BTN_SOUND_42_clicked();
    void on_BTN_SOUND_43_clicked();
    void on_BTN_SOUND_44_clicked();
    void on_BTN_SOUND_45_clicked();
    void on_BTN_SOUND_46_clicked();
    void on_BTN_SOUND_47_clicked();
    void on_BTN_SOUND_88_clicked();
    void on_BTN_SOUND_89_clicked();
    void on_BTN_SOUND_90_clicked();
    void on_BTN_SOUND_91_clicked();
    void on_BTN_SOUND_92_clicked();
    void on_BTN_SOUND_93_clicked();
    void on_BTN_SOUND_94_clicked();
    void on_BTN_SOUND_95_clicked();
    void on_BTN_SOUND_96_clicked();
    void on_BTN_SOUND_97_clicked();
    void on_BTN_SOUND_98_clicked();
    void on_BTN_SOUND_99_clicked();
    void on_BTN_SOUND_100_clicked();
    void on_BTN_SOUND_101_clicked();
    void on_BTN_SOUND_102_clicked();
    void on_BTN_SOUND_103_clicked();
    void on_BTN_SOUND_104_clicked();
    void on_BTN_SOUND_105_clicked();

    void on_BTN_ENABLE_clicked();
    void on_BTN_DISABLE_clicked();
};



//==========================================
class HUBOModel : public GLSkeleton
{
protected:
    std::vector<GLComplex*> objects;
public:
    HUBOModel(Bones& bones, Joints& joints, std::vector<GLComplex*> objects) : GLSkeleton(bones, joints){
        this->objects = objects;
    }

    void setColor(int id, const isnl::vec3& color){
        GLComplex& temp = *objects[id];
        for(int i = 0; i < temp.size(); ++i){
            temp[i]->setBaseColor(color);
        }
    }
};
inline GLSTL*     newStl(float x, float y, float z, std::string filename){
    GLSTL *ret = new GLSTL(filename);
    ret->setPosition(isnl::pos(x,y,z));
    ret->setBaseColor(basecolor);
    return ret;
}
inline GLBox*      newBox(float x, float y, float z, float sx, float sy, float sz){
    return new GLBox(isnl::pos(x,y,z), sx, sy, sz, basecolor);
}
//===========================================





class GLWidget : public QGLWidget
{
    Q_OBJECT
    friend class ModelDialog;

public:
    GLWidget(QWidget *parent = 0);
    ~GLWidget();

    int xRotation() const { return xRot; }
    int yRotation() const { return yRot; }
    int zRotation() const { return zRot; }

public slots:
    void setXRotation(int angle);
    void setYRotation(int angle);
    void setZRotation(int angle);

signals:
    void xRotationChanged(int angle);
    void yRotationChanged(int angle);
    void zRotationChanged(int angle);

protected:
    void initializeGL();
    void paintGL();
    void resizeGL(int width, int height);
    void mousePressEvent(QMouseEvent *event);
    void mouseMoveEvent(QMouseEvent *event);
    void wheelEvent(QWheelEvent *event);

private:
    void normalizeAngle(int *angle);

    GLComplex		globjs;
    HUBOModel       *model;
    GLGridPlane 	*floor;
    GLArrow         *rwforce, *lwforce, *rfforce, *lfforce;

    int xRot;
    int yRot;
    int zRot;
    int gear1Rot;

    QPoint currPos, lastPos;

    HUBOModel*     newModel();
};


#endif // MODELDIALOG_H
