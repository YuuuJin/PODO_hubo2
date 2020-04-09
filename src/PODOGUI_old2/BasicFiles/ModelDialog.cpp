#include "ModelDialog.h"
#include "ui_ModelDialog.h"


using namespace isnl;


enum JointSequentialNumber_Finger{
    RF_a1 = NO_OF_JOINTS-10, RF_a2, RF_a3,
    RF_b1, RF_b2, RF_b3,
    RF_c1, RF_c2, RF_c3,
    LF_a1, LF_a2, LF_a3,
    LF_b1, LF_b2, LF_b3,
    LF_c1, LF_c2, LF_c3, NO_OF_MODEL_JOINTS
};


enum BONE_ID{
    B_WST,B_TORSO,B_HEAD,
    B_LSP,B_LSR,B_LSY,B_LEB,B_LWY,B_LWP,B_LWFT,
    B_LF1,B_LF2,B_LF3,B_LF4,B_LF5,
    B_RSP,B_RSR,B_RSY,B_REB,B_RWY,B_RWP,B_RWFT,
    B_RF1,B_RF2,B_RF3,B_RF4,B_RF5,
    B_LHY,B_LHR,B_LHP,B_LKN,B_LAP,B_LAR,
    B_RHY,B_RHR,B_RHP,B_RKN,B_RAP,B_RAR,
    NBONE
};
enum COMP_ID{
    JRHY, JRHR, JRHP, JRKN, JRAP, JRAR,
    JLHY, JLHR, JLHP, JLKN, JLAP, JLAR,
    JRSP, JRSR, JRSY, JREB, JRWY, JRWP,
    JLSP, JLSR, JLSY, JLEB, JLWY, JLWP,
    JWST,
    JNKY, JNK1, JNK2,
    JRF1,JRF2,JRF3,JRF4,JRF5,
    JLF1,JLF2,JLF3,JLF4,JLF5,
    BWST,BTORSO,BCAMERA,
    BLSP,BLSR,BLSY,BLEB,BLWY,BLWP,BLWFT,
    BLF1,BLF2,BLF3,BLF4,BLF5,
    BRSP,BRSR,BRSY,BREB,BRWY,BRWP,BRWFT,
    BRF1,BRF2,BRF3,BRF4,BRF5,
    BLHY,BLHR,BLHP,BLKN,BLAP,BLAR,
    BRHY,BRHR,BRHP,BRKN,BRAP,BRAR,
    NCOMP
};
static const char COMP_NAME[NCOMP+1][10] = {
    "RHY", "RHR", "RHP", "RKN", "RAP", "RAR",
    "LHY", "LHR", "LHP", "LKN", "LAP", "LAR",
    "RSP", "RSR", "RSY", "REB", "RWY", "RWP",
    "LSP", "LSR", "LSY", "LEB", "LWY", "LWP",
    "WST",
    "NKY", "NK1", "NK2",
    "JRF1","JRF2","JRF3","JRF4","JRF5",
    "JLF1","JLF2","JLF3","JLF4","JLF5",

    "BWST","BTORSO","BHEAD",
    "BLSP","BLSR","BLSY","BLEB","BLWY","BLWP","BLWFT",
    "BLF1","BLF2","BLF3","BLF4","BLF5",
    "BRSP","BRSR","BRSY","BREB","BRWY","BRWP","BRWFT",
    "BRF1","BRF2","BRF3","BRF4","BRF5",
    "BLHY","BLHR","BLHP","BLKN","BLAP","BLAR",
    "BRHY","BRHR","BRHP","BRKN","BRAP","BRAR",
    "Null"
};


std::string STLPath = "../share/GUI/stl/";
std::string MeshPath = "../share/GUI/meshes/";

ModelDialog::ModelDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::ModelDialog)
{
    ui->setupUi(this);

    glWidget = new GLWidget(this);

    QScrollArea *glWidgetArea = new QScrollArea;
    glWidgetArea->setWidget(glWidget);
    glWidgetArea->setWidgetResizable(true);
    glWidgetArea->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    glWidgetArea->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    glWidgetArea->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
    glWidgetArea->setMinimumSize(50, 50);

    xSlider = createSlider(SIGNAL(xRotationChanged(int)), SLOT(setXRotation(int)));
    ySlider = createSlider(SIGNAL(yRotationChanged(int)), SLOT(setYRotation(int)));
    zSlider = createSlider(SIGNAL(zRotationChanged(int)), SLOT(setZRotation(int)));

    ui->LAYOUT_MODEL->addWidget(glWidgetArea, 0, 0);
    ui->LAYOUT_SLIDER->addWidget(xSlider,0,0);
    ui->LAYOUT_SLIDER->addWidget(ySlider,1,0);
    ui->LAYOUT_SLIDER->addWidget(zSlider,2,0);

    xSlider->setValue(180 * 16);
    ySlider->setValue(180 * 16);
    zSlider->setValue(180 * 16);

    displayTimer = new QTimer(this);
    connect(displayTimer, SIGNAL(timeout()), this, SLOT(DisplayUpdate()));
    displayTimer->start(100);
    //connect(pLAN, SIGNAL(NewPODOData()), this, SLOT(DisplayUpdate()));

    ui->rad_kor->setChecked(true);


    playdemoSound[1]   = new QSound("../share/hubo_pyeonchang/01_greeting_kor.wav");
    playdemoSound[2]   = new QSound("../share/hubo_pyeonchang/02_hello_kor.wav");
    playdemoSound[3]   = new QSound("../share/hubo_pyeonchang/03_name_kor.wav");
    playdemoSound[4]   = new QSound("../share/hubo_pyeonchang/04_hahaha_kor.wav");
    playdemoSound[5]   = new QSound("../share/hubo_pyeonchang/05_sad_kor.wav");
    playdemoSound[6]   = new QSound("../share/hubo_pyeonchang/06_ofcourse_kor.wav");
    playdemoSound[7]   = new QSound("../share/hubo_pyeonchang/07_1millon_kor.wav");
    playdemoSound[8]   = new QSound("../share/hubo_pyeonchang/08_finger_kor.wav");
    playdemoSound[9]   = new QSound("../share/hubo_pyeonchang/09_cute_kor.wav");
    playdemoSound[10]   = new QSound("../share/hubo_pyeonchang/10_love_kor.wav");
    playdemoSound[11]   = new QSound("../share/hubo_pyeonchang/11_hoot_kor.wav");
    playdemoSound[12]   = new QSound("../share/hubo_pyeonchang/12_bye_kor.wav");
    playdemoSound[13]   = new QSound("../share/hubo_pyeonchang/13_1stRobot_kor.wav");
    playdemoSound[14]   = new QSound("../share/hubo_pyeonchang/14_closer_kor.wav");
    playdemoSound[15]   = new QSound("../share/hubo_pyeonchang/15_curious_kor.wav");
    playdemoSound[16]   = new QSound("../share/hubo_pyeonchang/16_dance_kor.wav");
    playdemoSound[17]   = new QSound("../share/hubo_pyeonchang/17_enjoy_kor.wav");
    playdemoSound[18]   = new QSound("../share/hubo_pyeonchang/18_from_kor.wav");
    playdemoSound[19]   = new QSound("../share/hubo_pyeonchang/19_okay_kor.wav");
    playdemoSound[20]   = new QSound("../share/hubo_pyeonchang/20_shake_kor.wav");
    playdemoSound[21]   = new QSound("../share/hubo_pyeonchang/21_shake_gently_kor.wav");
    playdemoSound[22]   = new QSound("../share/hubo_pyeonchang/22_sorry_can't_do_kor.wav");
    playdemoSound[23]   = new QSound("../share/hubo_pyeonchang/23_walk_kor.wav");
    playdemoSound[24]   = new QSound("../share/hubo_pyeonchang/24_welcome_kor.wav");
    playdemoSound[25]   = new QSound("../share/hubo_pyeonchang/25_don't_know_kor.wav");
    playdemoSound[26]   = new QSound("../share/hubo_pyeonchang/26_eat_kor.wav");
    playdemoSound[27]   = new QSound("../share/hubo_pyeonchang/27_feeling_kor.wav");
    playdemoSound[28]   = new QSound("../share/hubo_pyeonchang/28_cheer_kor.wav");
    playdemoSound[29]   = new QSound("../share/hubo_pyeonchang/29_fighting_kor.wav");
    playdemoSound[30]   = new QSound("../share/hubo_pyeonchang/30_cold_kor.wav");
    playdemoSound[31]   = new QSound("../share/hubo_pyeonchang/31_picture_kor.wav");
    playdemoSound[32]   = new QSound("../share/hubo_pyeonchang/32_hero_wakeup_kor.wav");
    playdemoSound[33]   = new QSound("../share/hubo_pyeonchang/33_nerf_this_kor.wav");
    playdemoSound[34]   = new QSound("../share/hubo_pyeonchang/34_my_skill_time_kor.wav");
    playdemoSound[35]   = new QSound("../share/hubo_pyeonchang/35_online_kor.wav");
    playdemoSound[36]   = new QSound("../share/hubo_pyeonchang/36_yes_kor.wav");
    playdemoSound[37]   = new QSound("../share/hubo_pyeonchang/37_no_kor.wav");
    playdemoSound[38]   = new QSound("../share/hubo_pyeonchang/38_move_kor.wav");
    playdemoSound[39]   = new QSound("../share/hubo_pyeonchang/39_move_left_kor.wav");
    playdemoSound[40]   = new QSound("../share/hubo_pyeonchang/40_move_right_kor.wav");

    playdemoSound[1+50]   = new QSound("../share/hubo_pyeonchang/01_greeting_eng.wav");
    playdemoSound[2+50]   = new QSound("../share/hubo_pyeonchang/02_hello_eng.wav");
    playdemoSound[3+50]   = new QSound("../share/hubo_pyeonchang/03_name_eng.wav");
    playdemoSound[4+50]   = new QSound("../share/hubo_pyeonchang/04_hahaha_eng.wav");
    playdemoSound[5+50]   = new QSound("../share/hubo_pyeonchang/05_sad_eng.wav");
    playdemoSound[6+50]   = new QSound("../share/hubo_pyeonchang/06_ofcourse_eng.wav");
    playdemoSound[7+50]   = new QSound("../share/hubo_pyeonchang/07_1millon_eng.wav");
    playdemoSound[8+50]   = new QSound("../share/hubo_pyeonchang/08_finger_eng.wav");
    playdemoSound[9+50]   = new QSound("../share/hubo_pyeonchang/09_cute_eng.wav");
    playdemoSound[10+50]   = new QSound("../share/hubo_pyeonchang/10_love_eng.wav");
    playdemoSound[11+50]   = new QSound("../share/hubo_pyeonchang/11_hoot_eng.wav");
    playdemoSound[12+50]   = new QSound("../share/hubo_pyeonchang/12_bye_eng.wav");
    playdemoSound[13+50]   = new QSound("../share/hubo_pyeonchang/13_1stRobot_eng.wav");
    playdemoSound[14+50]   = new QSound("../share/hubo_pyeonchang/14_closer_eng.wav");
    playdemoSound[15+50]   = new QSound("../share/hubo_pyeonchang/15_curious_eng.wav");
    playdemoSound[16+50]   = new QSound("../share/hubo_pyeonchang/16_dance_eng.wav");
    playdemoSound[17+50]   = new QSound("../share/hubo_pyeonchang/17_enjoy_eng.wav");
    playdemoSound[18+50]   = new QSound("../share/hubo_pyeonchang/18_from_eng.wav");
    playdemoSound[19+50]   = new QSound("../share/hubo_pyeonchang/19_okay_eng.wav");
    playdemoSound[20+50]   = new QSound("../share/hubo_pyeonchang/20_shake_eng.wav");
    playdemoSound[21+50]   = new QSound("../share/hubo_pyeonchang/21_shake_gently_eng.wav");
    playdemoSound[22+50]   = new QSound("../share/hubo_pyeonchang/22_sorry_can't_do_eng.wav");
    playdemoSound[23+50]   = new QSound("../share/hubo_pyeonchang/23_walk_eng.wav");
    playdemoSound[24+50]   = new QSound("../share/hubo_pyeonchang/24_welcome_eng.wav");
    playdemoSound[25+50]   = new QSound("../share/hubo_pyeonchang/25_don't_know_eng.wav");
    playdemoSound[26+50]   = new QSound("../share/hubo_pyeonchang/26_eat_eng.wav");
    playdemoSound[27+50]   = new QSound("../share/hubo_pyeonchang/27_feeling_eng.wav");
    playdemoSound[28+50]   = new QSound("../share/hubo_pyeonchang/28_cheer_eng.wav");
    playdemoSound[29+50]   = new QSound("../share/hubo_pyeonchang/29_fighting_eng.wav");
    playdemoSound[30+50]   = new QSound("../share/hubo_pyeonchang/30_cold_eng.wav");
    playdemoSound[31+50]   = new QSound("../share/hubo_pyeonchang/31_picture_eng.wav");
    playdemoSound[32+50]   = new QSound("../share/hubo_pyeonchang/32_hero_wakeup_eng.wav");
    playdemoSound[33+50]   = new QSound("../share/hubo_pyeonchang/33_nerf_this_eng.wav");
    playdemoSound[34+50]   = new QSound("../share/hubo_pyeonchang/34_my_skill_time_eng.wav");
    playdemoSound[35+50]   = new QSound("../share/hubo_pyeonchang/35_online_eng.wav");
    playdemoSound[36+50]   = new QSound("../share/hubo_pyeonchang/36_yes_eng.wav");
    playdemoSound[37+50]   = new QSound("../share/hubo_pyeonchang/37_no_eng.wav");
    playdemoSound[38+50]   = new QSound("../share/hubo_pyeonchang/38_move_eng.wav");
    playdemoSound[39+50]   = new QSound("../share/hubo_pyeonchang/39_move_left_eng.wav");
    playdemoSound[40+50]   = new QSound("../share/hubo_pyeonchang/40_move_right_eng.wav");

    playdemoSound[100] = new QSound("../share/hubo_pyeonchang/vip/vip_meet_you_kor.wav");
    playdemoSound[101] = new QSound("../share/hubo_pyeonchang/vip/vip_name_kor.wav");
    playdemoSound[102] = new QSound("../share/hubo_pyeonchang/vip/vip_photo_kor.wav");
    playdemoSound[103] = new QSound("../share/hubo_pyeonchang/vip/vip_meet_you_eng.wav");
    playdemoSound[104] = new QSound("../share/hubo_pyeonchang/vip/vip_name_eng.wav");
    playdemoSound[105] = new QSound("../share/hubo_pyeonchang/vip/vip_photo_eng.wav");
    playdemoSound[106] = new QSound("../share/hubo_pyeonchang/vip/vip_meet_you_fra.wav");
    playdemoSound[107] = new QSound("../share/hubo_pyeonchang/vip/vip_name_fra.wav");
    playdemoSound[108] = new QSound("../share/hubo_pyeonchang/vip/vip_photo_fra.wav");
    playdemoSound[109] = new QSound("../share/hubo_pyeonchang/vip/vip_meet_you_spa.wav");
    playdemoSound[110] = new QSound("../share/hubo_pyeonchang/vip/vip_name_spa.wav");
    playdemoSound[111] = new QSound("../share/hubo_pyeonchang/vip/vip_photo_spa.wav");
    playdemoSound[112] = new QSound("../share/hubo_pyeonchang/vip/vip_meet_you_chi.wav");
    playdemoSound[113] = new QSound("../share/hubo_pyeonchang/vip/vip_name_chi.wav");
    playdemoSound[114] = new QSound("../share/hubo_pyeonchang/vip/vip_photo_chi.wav");
    playdemoSound[115] = new QSound("../share/hubo_pyeonchang/vip/vip_meet_you_jap.wav");
    playdemoSound[116] = new QSound("../share/hubo_pyeonchang/vip/vip_name_jap.wav");
    playdemoSound[117] = new QSound("../share/hubo_pyeonchang/vip/vip_photo_jap.wav");

    playdemoSound[120] = new QSound("../share/hubo_pyeonchang/wa.wav");
    playdemoSound[121] = new QSound("../share/hubo_pyeonchang/oh.wav");


















}

ModelDialog::~ModelDialog()
{
    delete ui;
}


void ModelDialog::DisplayUpdate(){
    if(ui->CB_USE_ENCODER->isChecked()){
        for(int i = 0; i < NO_OF_JOINTS; ++i)
            glWidget->model->ref[i+7] = JointEncoder(i) * D2Rf;

        glWidget->model->ref[3] = 1.0;
        glWidget->model->ref[4] = 0.0;
        glWidget->model->ref[5] = 0.0;
        glWidget->model->ref[6] = 0.0;
    }else{
        for(int i = 0; i < NO_OF_JOINTS; ++i)
            glWidget->model->ref[i+7]= JointReference(i) * D2Rf;
        glWidget->model->ref[3] = 1.0f;
        glWidget->model->ref[4] = 0.0f;
        glWidget->model->ref[5] = 0.0f;
        glWidget->model->ref[6] = 0.0f;
    }

    // Fingers always use encoder
    //glWidget->model->ref[RF_a1+7] = glWidget->model->ref[RF_a2+7] = glWidget->model->ref[RF_a3+7] = ( 100/36.2)*(36.2-JointEncoder(RF1))*D2Rf;
    //glWidget->model->ref[RF_b1+7] = glWidget->model->ref[RF_b2+7] = glWidget->model->ref[RF_b3+7] = ( 100/36.2)*(36.2-JointEncoder(RF1))*D2Rf;
    //glWidget->model->ref[RF_c1+7] = glWidget->model->ref[RF_c2+7] = glWidget->model->ref[RF_c3+7] = (-100/36.2)*(36.2-JointEncoder(RF1))*D2Rf;

    //glWidget->model->ref[LF_a1+7] = glWidget->model->ref[LF_a2+7] = glWidget->model->ref[LF_a3+7] = ( 100/36.2)*(36.2-JointEncoder(LF1))*D2Rf;
    //glWidget->model->ref[LF_b1+7] = glWidget->model->ref[LF_b2+7] = glWidget->model->ref[LF_b3+7] = ( 100/36.2)*(36.2-JointEncoder(LF1))*D2Rf;
    //glWidget->model->ref[LF_c1+7] = glWidget->model->ref[LF_c2+7] = glWidget->model->ref[LF_c3+7] = (-100/36.2)*(36.2-JointEncoder(LF1))*D2Rf;



    glWidget->floor->setVisible(ui->CB_SHOW_FLOOR->isChecked());
    glWidget->rfforce->setVisible(ui->CB_SHOW_FT->isChecked());
    glWidget->rwforce->setVisible(ui->CB_SHOW_FT->isChecked());
    glWidget->lfforce->setVisible(ui->CB_SHOW_FT->isChecked());
    glWidget->lwforce->setVisible(ui->CB_SHOW_FT->isChecked());


    glWidget->updateGL();
}


QSlider* ModelDialog::createSlider(const char *changedSignal, const char *setterSlot){
    QSlider *slider = new QSlider(Qt::Horizontal);
    slider->setRange(0, 360 * 16);
    slider->setSingleStep(16);
    slider->setPageStep(15 * 16);
    slider->setTickInterval(15 * 16);
    slider->setTickPosition(QSlider::TicksRight);
    connect(slider, SIGNAL(valueChanged(int)), glWidget, setterSlot);
    connect(glWidget, changedSignal, slider, SLOT(setValue(int)));
    return slider;
}

void ModelDialog::on_BT_CamFront_clicked(){
    xSlider->setValue(180 * 16);
    ySlider->setValue(180 * 16);
    zSlider->setValue(180 * 16);
}
void ModelDialog::on_BT_CamRight_clicked(){
    xSlider->setValue(180 * 16);
    ySlider->setValue(180 * 16);
    zSlider->setValue(270 * 16);
}
void ModelDialog::on_BT_CamLeft_clicked(){
    xSlider->setValue(180 * 16);
    ySlider->setValue(180 * 16);
    zSlider->setValue( 90 * 16);
}


GLWidget::GLWidget(QWidget *parent)
    : QGLWidget(parent)
{
    xRot = 0;
    yRot = 0;
    zRot = 0;


    floor = new GLGridPlane(0.5f, 0.5f, 20, 20);
    floor->setPosition(isnl::pos(0.f, 0.f, -1.f));

    model = newModel();
    model->ref.resize(NO_OF_JOINTS+7);//NO_OF_JOINTS+7);
    model->ref[3] = 1.f;

    //globjs << model;
    globjs << floor;

    rwforce = new GLArrow(0.01f ,0.5f); rwforce->setBaseColor(1, 0, 0);
    lwforce = new GLArrow(0.01f ,0.5f); lwforce->setBaseColor(1, 0, 0);
    rfforce = new GLArrow(0.01f ,0.5f); rfforce->setBaseColor(1, 0, 0);
    lfforce = new GLArrow(0.01f ,0.5f); lfforce->setBaseColor(1, 0, 0);
    globjs << rwforce << lwforce << rfforce << lfforce;

}
GLWidget::~GLWidget()
{
    makeCurrent();
}

void GLWidget::setXRotation(int angle){
    normalizeAngle(&angle);
    if (angle != xRot) {
        xRot = angle;
        emit xRotationChanged(angle);
    }
}
void GLWidget::setYRotation(int angle){
    normalizeAngle(&angle);
    if (angle != yRot) {
        yRot = angle;
        emit yRotationChanged(angle);
    }
}
void GLWidget::setZRotation(int angle){
    normalizeAngle(&angle);
    if (angle != zRot) {
        zRot = angle;
        emit zRotationChanged(angle);
    }
}
void GLWidget::initializeGL()
{
    GLfloat lightColor[] = {1.0f, 1.0f, 1.0f, 1.0f};

    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glEnable(GL_LIGHT1);
    glEnable(GL_LIGHT2);
    glEnable(GL_LIGHT3);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, lightColor);
    glLightfv(GL_LIGHT1, GL_DIFFUSE, lightColor);
    glLightfv(GL_LIGHT2, GL_DIFFUSE, lightColor);
    glLightfv(GL_LIGHT3, GL_DIFFUSE, lightColor);
    glEnable(GL_DEPTH_TEST);

    globjs.initialize();

    glEnable(GL_NORMALIZE);
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
}
void GLWidget::paintGL()
{
    GLfloat lightPos0[4] = { 10.0f,-10.0f, 20.0f, 0.7f };
    GLfloat lightPos1[4] = {-10.0f, 10.0f, 20.0f, 0.7f };
    GLfloat lightPos2[4] = {-10.0f,-10.0f, 20.0f, 0.7f };
    GLfloat lightPos3[4] = { 10.0f, 10.0f, 20.0f, 0.7f };

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);


    glPushMatrix();

    glRotated(xRot / 16.0 -180, 1.0, 0.0, 0.0);
    glRotated(yRot / 16.0 -180, 0.0, 1.0, 0.0);
    glRotated(zRot / 16.0 -180, 0.0, 0.0, 1.0);

    model->render();



    glPushMatrix();

    glRotated(xRot / 16.0 -180, 1.0, 0.0, 0.0);
    glRotated(yRot / 16.0 -180, 0.0, 1.0, 0.0);
    glRotated(zRot / 16.0 -180, 0.0, 0.0, 1.0);

    glLightfv(GL_LIGHT0, GL_POSITION, lightPos0);
    glLightfv(GL_LIGHT1, GL_POSITION, lightPos1);
    glLightfv(GL_LIGHT2, GL_POSITION, lightPos2);
    glLightfv(GL_LIGHT3, GL_POSITION, lightPos3);

    // ft ----
    rwforce->setPosition(model->getPosition(BRWFT)* isnl::pos(-0.05f,0.0f,0.0f));
    lwforce->setPosition(model->getPosition(BLWFT)* isnl::pos(-0.05f,0.0f,0.0f));
    rfforce->setPosition(model->getPosition(BRAR) * isnl::pos(-0.15f,0.0f,0.0f));
    lfforce->setPosition(model->getPosition(BLAR) * isnl::pos(-0.15f,0.0f,0.0f));
    rfforce->setLength(PODO_DATA.CoreSHM.FT[0].Fz/800.0);
    lfforce->setLength(PODO_DATA.CoreSHM.FT[1].Fz/800.0);
    rwforce->setLength(PODO_DATA.CoreSHM.FT[2].Fz/50.0);
    lwforce->setLength(PODO_DATA.CoreSHM.FT[3].Fz/50.0);

    globjs.render();

    glPopMatrix();

    glPopMatrix();

}

void GLWidget::resizeGL(int width, int height)
{
    int side = qMin(width, height);
    glViewport((width - side) / 2, (height - side) / 2, side, side);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glFrustum(-0.1, +0.1, -0.1, 0.1, 0.1, 60.0);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    glTranslated(0.0, 0.0, -1.0);
    // change camera coord to world coord :
    glRotatef(2.0944*R2Df, -0.5774, -0.5774, -0.5774);
}
void GLWidget::mousePressEvent(QMouseEvent *event){
    lastPos = event->pos();
}
void GLWidget::mouseMoveEvent(QMouseEvent *event){
    currPos = event->pos();
    int dx = currPos.x() - lastPos.x();
    int dy = currPos.y() - lastPos.y();

    if(event->buttons() & Qt::RightButton){
        setYRotation(yRot + 2 * dy);
        setZRotation(zRot + 2 * dx);
    }else if (event->buttons() & Qt::LeftButton){
//        setXRotation(xRot + 8 * dy);
//        setZRotation(zRot + 8 * dx);
        glTranslatef(0, dx/500.0, -dy/500.0);
    }
    lastPos = event->pos();
}
void GLWidget::wheelEvent(QWheelEvent *event){
    //if(event->modifiers().testFlag(Qt::ControlModifier)){
        glTranslatef(-event->delta()/1000.0, 0, 0);
    //}
}

void GLWidget::normalizeAngle(int *angle){
    while (*angle < 0)
        *angle += 360 * 16;
    while (*angle > 360 * 16)
        *angle -= 360 * 16;
}


// ===========================
// ===========================


HUBOModel* GLWidget::newModel(){

    Joints joints(NO_OF_JOINTS);
    Bones  bones(NBONE);
    Bones tempbone(30);
    std::vector<GLComplex*> objs(1);
    objs[0] = new GLComplex();

    // Lower body joints
    joints[RHY]     = new RevoluteZJoint("RHY");
    joints[RHR]     = new RevoluteXJoint("RHR");
    joints[RHP]     = new RevoluteYJoint("RHP");
    joints[RKN]     = new RevoluteYJoint("RKN");
    joints[RAP]     = new RevoluteYJoint("RAP");
    joints[RAR]     = new RevoluteXJoint("RAR");
    joints[LHY]     = new RevoluteZJoint("LHY");
    joints[LHR]     = new RevoluteXJoint("LHR");
    joints[LHP]     = new RevoluteYJoint("LHP");
    joints[LKN]     = new RevoluteYJoint("LKN");
    joints[LAP]     = new RevoluteYJoint("LAP");
    joints[LAR]     = new RevoluteXJoint("LAR");
    // Upper body joints
    joints[RSP]     = new RevoluteYJoint("RSP");
    joints[RSR]     = new RevoluteXJoint("RSR");
    joints[RSY]     = new RevoluteZJoint("RSY");
    joints[REB]     = new RevoluteYJoint("REB");
    joints[RWY]     = new RevoluteZJoint("RWY");
    joints[RWP]     = new RevoluteYJoint("RWP");
    joints[LSP]     = new RevoluteYJoint("LSP");
    joints[LSR]     = new RevoluteXJoint("LSR");
    joints[LSY]     = new RevoluteZJoint("LSY");
    joints[LEB]     = new RevoluteYJoint("LEB");
    joints[LWY]     = new RevoluteZJoint("LWY");
    joints[LWP]     = new RevoluteYJoint("LWP");
    // Waist, Fingers
    joints[WST]     = new RevoluteZJoint("WST");
    joints[RF1]     = new RevoluteZJoint("RF1");
    joints[RF2]     = new RevoluteZJoint("RF2");
    joints[RF3]     = new RevoluteZJoint("RF3");
    joints[RF4]     = new RevoluteZJoint("RF4");
    joints[RF5]     = new RevoluteZJoint("RF5");
    joints[LF1]     = new RevoluteZJoint("LF1");
    joints[LF2]     = new RevoluteZJoint("LF2");
    joints[LF3]     = new RevoluteZJoint("LF3");
    joints[LF4]     = new RevoluteZJoint("LF4");
    joints[LF5]     = new RevoluteZJoint("LF5");

    joints[NKY]     = new RevoluteZJoint("NKY");
    joints[NK1]     = new RevoluteYJoint("NKP1");
    joints[NK2]     = new RevoluteYJoint("NKP2");

    ((RevoluteJoint*)joints[RSR])->setOffset(-15.f*D2Rf);
    ((RevoluteJoint*)joints[LSR])->setOffset( 15.f*D2Rf);
    ((RevoluteJoint*)joints[REB])->setOffset(-20.f*D2Rf);
    ((RevoluteJoint*)joints[LEB])->setOffset(-20.f*D2Rf);


    GLObject *body_head    = newStl(0.0, 0.0, 0.0, MeshPath+"Body_Head.stl");
    GLObject *body_torso   = newStl(0.0, 0.0, 0.0, MeshPath+"Body_Torso.stl");
    GLObject *body_wst	   = newStl(0.0, 0.0, 0.0, MeshPath+"Body_Hip.stl");

    //LARM
    GLObject *body_lsp	   = newStl(0.0, 0.0, 0.0, MeshPath+"Body_LSP.stl");
    GLObject *body_lsr     = newStl(0.0, 0.0, 0.0, MeshPath+"Body_LSR.stl");
    GLObject *body_lsy     = newStl(0.0, 0.0, 0.0, MeshPath+"Body_LSY.stl");
    GLObject *body_leb     = newStl(0.0, 0.0, 0.0, MeshPath+"Body_LEB.stl");
    GLObject *body_lwy     = newStl(0.0, 0.0, 0.0, MeshPath+"Body_LWY.stl");
    GLObject *body_lwp     = newStl(0.0, 0.0, 0.0, MeshPath+"Body_LWP.stl");
    GLObject *body_lwft    = newStl(0.0, 0.0, 0.0, MeshPath+"Body_LF1.stl");

    //RARM
    GLObject *body_rsp	   = newStl(0.0, 0.0, 0.0, MeshPath+"Body_RSP.stl");
    GLObject *body_rsr     = newStl(0.0, 0.0, 0.0, MeshPath+"Body_RSR.stl");
    GLObject *body_rsy     = newStl(0.0, 0.0, 0.0, MeshPath+"Body_RSY.stl");
    GLObject *body_reb     = newStl(0.0, 0.0, 0.0, MeshPath+"Body_REB.stl");
    GLObject *body_rwy     = newStl(0.0, 0.0, 0.0, MeshPath+"Body_RWY.stl");
    GLObject *body_rwp     = newStl(0.0, 0.0, 0.0, MeshPath+"Body_RWP.stl");
    GLObject *body_rwft    = newStl(0.0, 0.0, 0.0, MeshPath+"Body_RF1.stl");

    //LLEG
    GLObject *body_lhy	   = newStl(0.0, 0.0, 0.0, MeshPath+"Body_LHY.stl");
    GLObject *body_lhr	   = newStl(0.0, 0.0, 0.0, MeshPath+"Body_LHR.stl");
    GLObject *body_lhp	   = newStl(0.0, 0.0, 0.0, MeshPath+"Body_LHP.stl");
    GLObject *body_lkn	   = newStl(0.0, 0.0, 0.0, MeshPath+"Body_LKN.stl");
    GLObject *body_lap	   = newStl(0.0, 0.0, 0.0, MeshPath+"Body_LAP.stl");
    GLObject *body_lar	   = newStl(0.0, 0.0, 0.0, MeshPath+"Body_LAR.stl");

    //RLEG
    GLObject *body_rhy	   = newStl(0.0, 0.0, 0.0, MeshPath+"Body_RHY.stl");
    GLObject *body_rhr	   = newStl(0.0, 0.0, 0.0, MeshPath+"Body_RHR.stl");
    GLObject *body_rhp	   = newStl(0.0, 0.0, 0.0, MeshPath+"Body_RHP.stl");
    GLObject *body_rkn	   = newStl(0.0, 0.0, 0.0, MeshPath+"Body_RKN.stl");
    GLObject *body_rap	   = newStl(0.0, 0.0, 0.0, MeshPath+"Body_RAP.stl");
    GLObject *body_rar	   = newStl(0.0, 0.0, 0.0, MeshPath+"Body_RAR.stl");


    body_head->setBaseColor(0.4, 0.4, 0.8);
    body_torso->setBaseColor(0.4, 0.4, 0.6);
    body_wst->setBaseColor(0.4, 0.4, 0.5);

    body_rsp->setBaseColor(0.9, 0.5, 0.4);
    body_rsr->setBaseColor(0.7, 0.5, 0.4);
    body_rsy->setBaseColor(0.7, 0.6, 0.4);
    body_reb->setBaseColor(0.7, 0.7, 0.4);
    body_rwy->setBaseColor(0.6, 0.7, 0.4);
    body_rwp->setBaseColor(0.5, 0.7, 0.4);

    body_lsp->setBaseColor(0.9, 0.5, 0.4);
    body_lsr->setBaseColor(0.7, 0.5, 0.4);
    body_lsy->setBaseColor(0.7, 0.6, 0.4);
    body_leb->setBaseColor(0.7, 0.7, 0.4);
    body_lwy->setBaseColor(0.6, 0.7, 0.4);
    body_lwp->setBaseColor(0.5, 0.7, 0.4);

    body_rhy->setBaseColor(0.9, 0.5, 0.4);
    body_rhr->setBaseColor(0.7, 0.5, 0.4);
    body_rhp->setBaseColor(0.7, 0.6, 0.4);
    body_rkn->setBaseColor(0.7, 0.7, 0.4);
    body_rap->setBaseColor(0.6, 0.7, 0.4);
    body_rar->setBaseColor(0.5, 0.7, 0.4);

    body_lhy->setBaseColor(0.9, 0.5, 0.4);
    body_lhr->setBaseColor(0.7, 0.5, 0.4);
    body_lhp->setBaseColor(0.7, 0.6, 0.4);
    body_lkn->setBaseColor(0.7, 0.7, 0.4);
    body_lap->setBaseColor(0.6, 0.7, 0.4);
    body_lar->setBaseColor(0.5, 0.7, 0.4);



    bones[B_TORSO]  = new Bone(COMP_NAME[BTORSO],  isnl::pos(0.0, 0.0, 0.0), body_torso);
    bones[B_HEAD]   = new Bone(COMP_NAME[BCAMERA], isnl::pos(0.0, 0.0, 0.0), body_head);
    bones[B_WST]    = new Bone(COMP_NAME[BWST],    isnl::pos(0.0, 0.0, 0.0), body_wst);

    bones[B_LSP]    = new Bone(COMP_NAME[BLSP],    isnl::pos(0.0, 0.0, 0.0), body_lsp);
    bones[B_LSR]    = new Bone(COMP_NAME[BLSR],    isnl::pos(0.0, 0.0, 0.0), body_lsr);
    bones[B_LSY]    = new Bone(COMP_NAME[BLSY],    isnl::pos(0.0, 0.0, 0.0), body_lsy);
    bones[B_LEB]    = new Bone(COMP_NAME[BLEB],    isnl::pos(0.0, 0.0, 0.0), body_leb);
    bones[B_LWY]    = new Bone(COMP_NAME[BLWY],    isnl::pos(0.0, 0.0, 0.0), body_lwy);
    bones[B_LWP]    = new Bone(COMP_NAME[BLWP],    isnl::pos(0.0, 0.0, 0.0), body_lwp);
    bones[B_LWFT]   = new Bone(COMP_NAME[BLWFT],   isnl::pos(0.0, 0.0, 0.0), body_lwft);

    bones[B_RSP]    = new Bone(COMP_NAME[BRSP],    isnl::pos(0.0, 0.0, 0.0), body_rsp);
    bones[B_RSR]    = new Bone(COMP_NAME[BRSR],    isnl::pos(0.0, 0.0, 0.0), body_rsr);
    bones[B_RSY]    = new Bone(COMP_NAME[BRSY],    isnl::pos(0.0, 0.0, 0.0), body_rsy);
    bones[B_REB]    = new Bone(COMP_NAME[BREB],    isnl::pos(0.0, 0.0, 0.0), body_reb);
    bones[B_RWY]    = new Bone(COMP_NAME[BRWY],    isnl::pos(0.0, 0.0, 0.0), body_rwy);
    bones[B_RWP]    = new Bone(COMP_NAME[BRWP],    isnl::pos(0.0, 0.0, 0.0), body_rwp);
    bones[B_RWFT]   = new Bone(COMP_NAME[BRWFT],   isnl::pos(0.0, 0.0, 0.0), body_rwft);

    bones[B_LHY]    = new Bone(COMP_NAME[BLHY],    isnl::pos(0.0, 0.0, 0.0), body_lhy);
    bones[B_LHR]    = new Bone(COMP_NAME[BLHR],    isnl::pos(0.0, 0.0, 0.0), body_lhr);
    bones[B_LHP]    = new Bone(COMP_NAME[BLHP],    isnl::pos(0.0, 0.0, 0.0), body_lhp);
    bones[B_LKN]    = new Bone(COMP_NAME[BLKN],    isnl::pos(0.0, 0.0, 0.0), body_lkn);
    bones[B_LAP]    = new Bone(COMP_NAME[BLAP],    isnl::pos(0.0, 0.0, 0.0), body_lap);
    bones[B_LAR]    = new Bone(COMP_NAME[BLAR],    isnl::pos(0.0, 0.0, 0.0), body_lar);

    bones[B_RHY]    = new Bone(COMP_NAME[BRHY],    isnl::pos(0.0, 0.0, 0.0), body_rhy);
    bones[B_RHR]    = new Bone(COMP_NAME[BRHR],    isnl::pos(0.0, 0.0, 0.0), body_rhr);
    bones[B_RHP]    = new Bone(COMP_NAME[BRHP],    isnl::pos(0.0, 0.0, 0.0), body_rhp);
    bones[B_RKN]    = new Bone(COMP_NAME[BRKN],    isnl::pos(0.0, 0.0, 0.0), body_rkn);
    bones[B_RAP]    = new Bone(COMP_NAME[BRAP],    isnl::pos(0.0, 0.0, 0.0), body_rap);
    bones[B_RAR]    = new Bone(COMP_NAME[BRAR],    isnl::pos(0.0, 0.0, 0.0), body_rar);

    //tree
    bones[B_WST]->setParent(NULL);

    // --- right arm ---
    tempbone[0] = new Bone("torso_2_rsp", isnl::pos(0.0122581, -0.139027, 0.0486356));
    tempbone[1] = new Bone("rsp_2_rsr", isnl::pos(0.0269, -0.072, 0));
    tempbone[2] = new Bone("rsr_2_rsy", isnl::pos(-0.0269, 0, -0.0245));
    tempbone[3] = new Bone("rsy_2_reb", isnl::pos(0.0219936, 0.018, -0.157505));
    tempbone[4] = new Bone("reb_2_rwy", isnl::pos(-0.0219906, -0.018, -0.111408));
    tempbone[5] = new Bone("rwy_2_rwp", isnl::pos(1.72973e-06, 0.01, -0.05255));

    *bones[B_TORSO] + tempbone[0] + joints[RSP] + bones[B_RSP]
                    + tempbone[1] + joints[RSR] + bones[B_RSR]
                    + tempbone[2] + joints[RSY] + bones[B_RSY]
                    + tempbone[3] + joints[REB] + bones[B_REB]
                    + tempbone[4] + joints[RWY] + bones[B_RWY]
                    + tempbone[5] + joints[RWP] + bones[B_RWP]
                    + bones[B_RWFT];

    // --- left arm ---
    tempbone[6] = new Bone("torso_2_lsp", isnl::pos(0.0122581, 0.143973, 0.0486356));
    tempbone[7] = new Bone("lsp_2_lsr", isnl::pos(0.0269, 0.072, 0));
    tempbone[8] = new Bone("lsr_2_lsy", isnl::pos(-0.0269, -0.000166249, -0.0245));
    tempbone[9] = new Bone("lsy_2_leb", isnl::pos(0.0219936, -0.018, -0.157505));
    tempbone[10]= new Bone("leb_2_lwy", isnl::pos(-0.0219906, 0.018, -0.111408));
    tempbone[11]= new Bone("lwy_2_lwp", isnl::pos(1.72973e-06, -0.00872953, -0.05255));

    *bones[B_TORSO] + tempbone[6] + joints[LSP] + bones[B_LSP]
                    + tempbone[7] + joints[LSR] + bones[B_LSR]
                    + tempbone[8] + joints[LSY] + bones[B_LSY]
                    + tempbone[9] + joints[LEB] + bones[B_LEB]
                    + tempbone[10]+ joints[LWY] + bones[B_LWY]
                    + tempbone[11]+ joints[LWP] + bones[B_LWP]
                    + bones[B_LWFT];

    // --- right leg ---
    tempbone[12]= new Bone("wst_2_rhy", isnl::pos(0.000125136, -0.0884999, -0.0764689));
    tempbone[13]= new Bone("rhy_2_rhr", isnl::pos(0.0520001, 7.3455e-05, -0.091004));
    tempbone[14]= new Bone("rhr_2_rhp", isnl::pos(-0.0529074, -0.0655748, 9.99159e-05));
    tempbone[15]= new Bone("rhp_2_rkn", isnl::pos(0.000766364, 0.0445011, -0.280007));
    tempbone[16]= new Bone("rkn_2_rap", isnl::pos(7.20248e-06, -0.0247555, -0.279942));
    tempbone[17]= new Bone("rap_2_rar", isnl::pos(0.0711787, 0.0466006, -1.04e-10));

    *bones[B_WST] + tempbone[12]+ joints[RHY] + bones[B_RHY]
                + tempbone[13] + joints[RHR] + bones[B_RHR]
                + tempbone[14] + joints[RHP] + bones[B_RHP]
                + tempbone[15] + joints[RKN] + bones[B_RKN]
                + tempbone[16] + joints[RAP] + bones[B_RAP]
                + tempbone[17] + joints[RAR] + bones[B_RAR];

    // --- left leg ---
    tempbone[18]= new Bone("wst_2_lhy", isnl::pos(0.000125136, 0.0885155, -0.0764689));
    tempbone[19]= new Bone("lhy_2_lhr", isnl::pos(0.0520001, 1.50498e-06, -0.091004));
    tempbone[20]= new Bone("lhr_2_lhp", isnl::pos(-0.0529074, 0.0655748, 9.99159e-05));
    tempbone[21]= new Bone("lhp_2_lkn", isnl::pos(0.000766364, -0.0445011, -0.280007));
    tempbone[22]= new Bone("lkn_2_lap", isnl::pos(7.20248e-06, 0.0247555, -0.279942));
    tempbone[23]= new Bone("lap_2_lar", isnl::pos(0.0711787, -0.0466006, -1.04e-10));

    *bones[B_WST] + tempbone[18]+ joints[LHY] + bones[B_LHY]
                + tempbone[19] + joints[LHR] + bones[B_LHR]
                + tempbone[20] + joints[LHP] + bones[B_LHP]
                + tempbone[21] + joints[LKN] + bones[B_LKN]
                + tempbone[22] + joints[LAP] + bones[B_LAP]
                + tempbone[23] + joints[LAR] + bones[B_LAR];

    // --- center ---
    tempbone[24] = new Bone("wst_2_torso", isnl::pos(-0.0122662, -0.00245974, 0.15262));
    tempbone[25] = new Bone("torso_2_head", isnl::pos(0.0137246-0.0014665, 0.0024526, 0.1092625+0.0876725));
    *bones[B_WST] + joints[WST] + tempbone[24] + bones[B_TORSO] + tempbone[25] + bones[B_HEAD];

    //tree


    // handling not allocated components
    for(int i=0; i<joints.size(); i++){
        if(joints[i] == NULL){
            joints[i] = new RevoluteJoint("dummy", isnl::vec3(0,0,1));
        }
    }
    for(int i=0; i<bones.size(); i++){
        if(bones[i] == NULL){
            bones[i] = new Bone("dummy", isnl::pos(0,0,0));
        }
    }

    return new HUBOModel(bones, joints, objs);
}
void ModelDialog::on_BTN_wa_clicked()
{
     playdemoSound[120]->play();
     ui->BTN_wa->setEnabled(false);
}

void ModelDialog::on_BTN_oh_clicked()
{
    playdemoSound[121]->play();
    ui->BTN_oh->setEnabled(false);
}
void ModelDialog::on_BTN_SOUND_1_clicked(){      if(ui->rad_kor->isChecked() == true) playdemoSound[1]->play(); else playdemoSound[1+50]->play();   ui->BTN_SOUND_1->setEnabled(false);
}void ModelDialog::on_BTN_SOUND_2_clicked(){     if(ui->rad_kor->isChecked() == true) playdemoSound[2]->play(); else playdemoSound[2+50]->play();   ui->BTN_SOUND_2->setEnabled(false);
}void ModelDialog::on_BTN_SOUND_3_clicked(){     if(ui->rad_kor->isChecked() == true) playdemoSound[3]->play(); else playdemoSound[3+50]->play();   ui->BTN_SOUND_3->setEnabled(false);
}void ModelDialog::on_BTN_SOUND_4_clicked(){     if(ui->rad_kor->isChecked() == true) playdemoSound[4]->play(); else playdemoSound[4+50]->play();   ui->BTN_SOUND_4->setEnabled(false);
}void ModelDialog::on_BTN_SOUND_5_clicked(){     if(ui->rad_kor->isChecked() == true) playdemoSound[5]->play(); else playdemoSound[5+50]->play();   ui->BTN_SOUND_5->setEnabled(false);
}void ModelDialog::on_BTN_SOUND_6_clicked(){     if(ui->rad_kor->isChecked() == true) playdemoSound[6]->play(); else playdemoSound[6+50]->play();   ui->BTN_SOUND_6->setEnabled(false);
}void ModelDialog::on_BTN_SOUND_7_clicked(){     if(ui->rad_kor->isChecked() == true) playdemoSound[7]->play(); else playdemoSound[7+50]->play();   ui->BTN_SOUND_7->setEnabled(false);
}void ModelDialog::on_BTN_SOUND_15_clicked(){    if(ui->rad_kor->isChecked() == true) playdemoSound[8]->play(); else playdemoSound[8+50]->play();   ui->BTN_SOUND_15->setEnabled(false);
}void ModelDialog::on_BTN_SOUND_16_clicked(){    if(ui->rad_kor->isChecked() == true) playdemoSound[9]->play(); else playdemoSound[9+50]->play();   ui->BTN_SOUND_16->setEnabled(false);
}void ModelDialog::on_BTN_SOUND_17_clicked(){    if(ui->rad_kor->isChecked() == true) playdemoSound[10]->play(); else playdemoSound[10+50]->play();     ui->BTN_SOUND_17->setEnabled(false);
}void ModelDialog::on_BTN_SOUND_18_clicked(){    if(ui->rad_kor->isChecked() == true) playdemoSound[11]->play(); else playdemoSound[11+50]->play();     ui->BTN_SOUND_18->setEnabled(false);
}void ModelDialog::on_BTN_SOUND_19_clicked(){    if(ui->rad_kor->isChecked() == true) playdemoSound[12]->play(); else playdemoSound[12+50]->play();     ui->BTN_SOUND_19->setEnabled(false);
}void ModelDialog::on_BTN_SOUND_20_clicked(){    if(ui->rad_kor->isChecked() == true) playdemoSound[13]->play(); else playdemoSound[13+50]->play();     ui->BTN_SOUND_20->setEnabled(false);
}void ModelDialog::on_BTN_SOUND_21_clicked(){    if(ui->rad_kor->isChecked() == true) playdemoSound[14]->play(); else playdemoSound[14+50]->play();     ui->BTN_SOUND_21->setEnabled(false);
}void ModelDialog::on_BTN_SOUND_22_clicked(){    if(ui->rad_kor->isChecked() == true) playdemoSound[15]->play(); else playdemoSound[15+50]->play();     ui->BTN_SOUND_22->setEnabled(false);
}void ModelDialog::on_BTN_SOUND_23_clicked(){    if(ui->rad_kor->isChecked() == true) playdemoSound[16]->play(); else playdemoSound[16+50]->play();     ui->BTN_SOUND_23->setEnabled(false);
}void ModelDialog::on_BTN_SOUND_24_clicked(){    if(ui->rad_kor->isChecked() == true) playdemoSound[17]->play(); else playdemoSound[17+50]->play();     ui->BTN_SOUND_24->setEnabled(false);
}void ModelDialog::on_BTN_SOUND_25_clicked(){    if(ui->rad_kor->isChecked() == true) playdemoSound[18]->play(); else playdemoSound[18+50]->play();     ui->BTN_SOUND_25->setEnabled(false);
}void ModelDialog::on_BTN_SOUND_26_clicked(){    if(ui->rad_kor->isChecked() == true) playdemoSound[19]->play(); else playdemoSound[19+50]->play();     ui->BTN_SOUND_26->setEnabled(false);
}void ModelDialog::on_BTN_SOUND_27_clicked(){    if(ui->rad_kor->isChecked() == true) playdemoSound[20]->play(); else playdemoSound[20+50]->play();     ui->BTN_SOUND_27->setEnabled(false);
}void ModelDialog::on_BTN_SOUND_28_clicked(){    if(ui->rad_kor->isChecked() == true) playdemoSound[21]->play(); else playdemoSound[21+50]->play();     ui->BTN_SOUND_28->setEnabled(false);
}void ModelDialog::on_BTN_SOUND_29_clicked(){    if(ui->rad_kor->isChecked() == true) playdemoSound[22]->play(); else playdemoSound[22+50]->play();     ui->BTN_SOUND_29->setEnabled(false);
}void ModelDialog::on_BTN_SOUND_30_clicked(){    if(ui->rad_kor->isChecked() == true) playdemoSound[23]->play(); else playdemoSound[23+50]->play();     ui->BTN_SOUND_30->setEnabled(false);
}void ModelDialog::on_BTN_SOUND_31_clicked(){    if(ui->rad_kor->isChecked() == true) playdemoSound[24]->play(); else playdemoSound[24+50]->play();     ui->BTN_SOUND_31->setEnabled(false);
}void ModelDialog::on_BTN_SOUND_32_clicked(){    if(ui->rad_kor->isChecked() == true) playdemoSound[25]->play(); else playdemoSound[25+50]->play();     ui->BTN_SOUND_32->setEnabled(false);
}void ModelDialog::on_BTN_SOUND_33_clicked(){    if(ui->rad_kor->isChecked() == true) playdemoSound[26]->play(); else playdemoSound[26+50]->play();     ui->BTN_SOUND_33->setEnabled(false);
}void ModelDialog::on_BTN_SOUND_34_clicked(){    if(ui->rad_kor->isChecked() == true) playdemoSound[27]->play(); else playdemoSound[27+50]->play();     ui->BTN_SOUND_34->setEnabled(false);
}void ModelDialog::on_BTN_SOUND_35_clicked(){    if(ui->rad_kor->isChecked() == true) playdemoSound[28]->play(); else playdemoSound[28+50]->play();     ui->BTN_SOUND_35->setEnabled(false);
}void ModelDialog::on_BTN_SOUND_36_clicked(){    if(ui->rad_kor->isChecked() == true) playdemoSound[29]->play(); else playdemoSound[29+50]->play();     ui->BTN_SOUND_36->setEnabled(false);
}void ModelDialog::on_BTN_SOUND_37_clicked(){    if(ui->rad_kor->isChecked() == true) playdemoSound[30]->play(); else playdemoSound[30+50]->play();     ui->BTN_SOUND_37->setEnabled(false);
}void ModelDialog::on_BTN_SOUND_38_clicked(){    if(ui->rad_kor->isChecked() == true) playdemoSound[31]->play(); else playdemoSound[31+50]->play();     ui->BTN_SOUND_38->setEnabled(false);
}void ModelDialog::on_BTN_SOUND_39_clicked(){    if(ui->rad_kor->isChecked() == true) playdemoSound[32]->play(); else playdemoSound[32+50]->play();     ui->BTN_SOUND_39->setEnabled(false);
}void ModelDialog::on_BTN_SOUND_40_clicked(){    if(ui->rad_kor->isChecked() == true) playdemoSound[33]->play(); else playdemoSound[33+50]->play();     ui->BTN_SOUND_40->setEnabled(false);
}void ModelDialog::on_BTN_SOUND_41_clicked(){    if(ui->rad_kor->isChecked() == true) playdemoSound[34]->play(); else playdemoSound[34+50]->play();     ui->BTN_SOUND_41->setEnabled(false);
}void ModelDialog::on_BTN_SOUND_42_clicked(){    if(ui->rad_kor->isChecked() == true) playdemoSound[35]->play(); else playdemoSound[35+50]->play();     ui->BTN_SOUND_42->setEnabled(false);
}void ModelDialog::on_BTN_SOUND_43_clicked(){    if(ui->rad_kor->isChecked() == true) playdemoSound[36]->play(); else playdemoSound[36+50]->play();     ui->BTN_SOUND_43->setEnabled(false);
}void ModelDialog::on_BTN_SOUND_44_clicked(){    if(ui->rad_kor->isChecked() == true) playdemoSound[37]->play(); else playdemoSound[37+50]->play();     ui->BTN_SOUND_44->setEnabled(false);
}void ModelDialog::on_BTN_SOUND_45_clicked(){    if(ui->rad_kor->isChecked() == true) playdemoSound[38]->play(); else playdemoSound[38+50]->play();     ui->BTN_SOUND_45->setEnabled(false);
}void ModelDialog::on_BTN_SOUND_46_clicked(){    if(ui->rad_kor->isChecked() == true) playdemoSound[39]->play(); else playdemoSound[39+50]->play();     ui->BTN_SOUND_46->setEnabled(false);
}void ModelDialog::on_BTN_SOUND_47_clicked(){    if(ui->rad_kor->isChecked() == true) playdemoSound[40]->play(); else playdemoSound[40+50]->play();     ui->BTN_SOUND_47->setEnabled(false);
}void ModelDialog::on_BTN_SOUND_100_clicked(){   playdemoSound[100]->play();    ui->BTN_SOUND_100->setEnabled(false);
}void ModelDialog::on_BTN_SOUND_101_clicked(){   playdemoSound[101]->play();    ui->BTN_SOUND_101->setEnabled(false);
}void ModelDialog::on_BTN_SOUND_102_clicked(){   playdemoSound[102]->play();    ui->BTN_SOUND_102->setEnabled(false);
}void ModelDialog::on_BTN_SOUND_103_clicked(){   playdemoSound[103]->play();    ui->BTN_SOUND_103->setEnabled(false);
}void ModelDialog::on_BTN_SOUND_104_clicked(){   playdemoSound[104]->play();    ui->BTN_SOUND_104->setEnabled(false);
}void ModelDialog::on_BTN_SOUND_105_clicked(){   playdemoSound[105]->play();    ui->BTN_SOUND_105->setEnabled(false);
}void ModelDialog::on_BTN_SOUND_88_clicked(){    playdemoSound[106]->play();    ui->BTN_SOUND_88->setEnabled(false);
}void ModelDialog::on_BTN_SOUND_89_clicked(){    playdemoSound[107]->play();    ui->BTN_SOUND_89->setEnabled(false);
}void ModelDialog::on_BTN_SOUND_90_clicked(){    playdemoSound[108]->play();    ui->BTN_SOUND_90->setEnabled(false);
}void ModelDialog::on_BTN_SOUND_91_clicked(){    playdemoSound[109]->play();    ui->BTN_SOUND_91->setEnabled(false);
}void ModelDialog::on_BTN_SOUND_92_clicked(){    playdemoSound[110]->play();    ui->BTN_SOUND_92->setEnabled(false);
}void ModelDialog::on_BTN_SOUND_93_clicked(){    playdemoSound[111]->play();    ui->BTN_SOUND_93->setEnabled(false);
}void ModelDialog::on_BTN_SOUND_94_clicked(){    playdemoSound[112]->play();    ui->BTN_SOUND_94->setEnabled(false);
}void ModelDialog::on_BTN_SOUND_95_clicked(){    playdemoSound[113]->play();    ui->BTN_SOUND_95->setEnabled(false);
}void ModelDialog::on_BTN_SOUND_96_clicked(){    playdemoSound[114]->play();    ui->BTN_SOUND_96->setEnabled(false);
}void ModelDialog::on_BTN_SOUND_97_clicked(){    playdemoSound[115]->play();    ui->BTN_SOUND_97->setEnabled(false);
}void ModelDialog::on_BTN_SOUND_98_clicked(){    playdemoSound[116]->play();    ui->BTN_SOUND_98->setEnabled(false);
}void ModelDialog::on_BTN_SOUND_99_clicked(){    playdemoSound[117]->play();    ui->BTN_SOUND_99->setEnabled(false);
}


void ModelDialog::on_BTN_ENABLE_clicked()
{
      ui->BTN_SOUND_1->setEnabled(true);
      ui->BTN_SOUND_2->setEnabled(true);
      ui->BTN_SOUND_3->setEnabled(true);
      ui->BTN_SOUND_4->setEnabled(true);
      ui->BTN_SOUND_5->setEnabled(true);
      ui->BTN_SOUND_6->setEnabled(true);
      ui->BTN_SOUND_7->setEnabled(true);
     ui->BTN_SOUND_15->setEnabled(true);
     ui->BTN_SOUND_16->setEnabled(true);
     ui->BTN_SOUND_17->setEnabled(true);
     ui->BTN_SOUND_18->setEnabled(true);
     ui->BTN_SOUND_19->setEnabled(true);
     ui->BTN_SOUND_20->setEnabled(true);
     ui->BTN_SOUND_21->setEnabled(true);
     ui->BTN_SOUND_22->setEnabled(true);
     ui->BTN_SOUND_23->setEnabled(true);
     ui->BTN_SOUND_24->setEnabled(true);
     ui->BTN_SOUND_25->setEnabled(true);
     ui->BTN_SOUND_26->setEnabled(true);
     ui->BTN_SOUND_27->setEnabled(true);
     ui->BTN_SOUND_28->setEnabled(true);
     ui->BTN_SOUND_29->setEnabled(true);
     ui->BTN_SOUND_30->setEnabled(true);
     ui->BTN_SOUND_31->setEnabled(true);
     ui->BTN_SOUND_32->setEnabled(true);
     ui->BTN_SOUND_33->setEnabled(true);
     ui->BTN_SOUND_34->setEnabled(true);
     ui->BTN_SOUND_35->setEnabled(true);
     ui->BTN_SOUND_36->setEnabled(true);
     ui->BTN_SOUND_37->setEnabled(true);
     ui->BTN_SOUND_38->setEnabled(true);
     ui->BTN_SOUND_39->setEnabled(true);
     ui->BTN_SOUND_40->setEnabled(true);
     ui->BTN_SOUND_41->setEnabled(true);
     ui->BTN_SOUND_42->setEnabled(true);
     ui->BTN_SOUND_43->setEnabled(true);
     ui->BTN_SOUND_44->setEnabled(true);
     ui->BTN_SOUND_45->setEnabled(true);
     ui->BTN_SOUND_46->setEnabled(true);
     ui->BTN_SOUND_47->setEnabled(true);
    ui->BTN_SOUND_100->setEnabled(true);
    ui->BTN_SOUND_101->setEnabled(true);
    ui->BTN_SOUND_102->setEnabled(true);
    ui->BTN_SOUND_103->setEnabled(true);
    ui->BTN_SOUND_104->setEnabled(true);
    ui->BTN_SOUND_105->setEnabled(true);
     ui->BTN_SOUND_88->setEnabled(true);
     ui->BTN_SOUND_89->setEnabled(true);
     ui->BTN_SOUND_90->setEnabled(true);
     ui->BTN_SOUND_91->setEnabled(true);
     ui->BTN_SOUND_92->setEnabled(true);
     ui->BTN_SOUND_93->setEnabled(true);
     ui->BTN_SOUND_94->setEnabled(true);
     ui->BTN_SOUND_95->setEnabled(true);
     ui->BTN_SOUND_96->setEnabled(true);
     ui->BTN_SOUND_97->setEnabled(true);
     ui->BTN_SOUND_98->setEnabled(true);
     ui->BTN_SOUND_99->setEnabled(true);
     ui->BTN_wa->setEnabled(true);
     ui->BTN_oh->setEnabled(true);
}

void ModelDialog::on_BTN_DISABLE_clicked()
{

    ui->BTN_SOUND_1->setEnabled(false);
    ui->BTN_SOUND_2->setEnabled(false);
    ui->BTN_SOUND_3->setEnabled(false);
    ui->BTN_SOUND_4->setEnabled(false);
    ui->BTN_SOUND_5->setEnabled(false);
    ui->BTN_SOUND_6->setEnabled(false);
    ui->BTN_SOUND_7->setEnabled(false);
   ui->BTN_SOUND_15->setEnabled(false);
   ui->BTN_SOUND_16->setEnabled(false);
   ui->BTN_SOUND_17->setEnabled(false);
   ui->BTN_SOUND_18->setEnabled(false);
   ui->BTN_SOUND_19->setEnabled(false);
   ui->BTN_SOUND_20->setEnabled(false);
   ui->BTN_SOUND_21->setEnabled(false);
   ui->BTN_SOUND_22->setEnabled(false);
   ui->BTN_SOUND_23->setEnabled(false);
   ui->BTN_SOUND_24->setEnabled(false);
   ui->BTN_SOUND_25->setEnabled(false);
   ui->BTN_SOUND_26->setEnabled(false);
   ui->BTN_SOUND_27->setEnabled(false);
   ui->BTN_SOUND_28->setEnabled(false);
   ui->BTN_SOUND_29->setEnabled(false);
   ui->BTN_SOUND_30->setEnabled(false);
   ui->BTN_SOUND_31->setEnabled(false);
   ui->BTN_SOUND_32->setEnabled(false);
   ui->BTN_SOUND_33->setEnabled(false);
   ui->BTN_SOUND_34->setEnabled(false);
   ui->BTN_SOUND_35->setEnabled(false);
   ui->BTN_SOUND_36->setEnabled(false);
   ui->BTN_SOUND_37->setEnabled(false);
   ui->BTN_SOUND_38->setEnabled(false);
   ui->BTN_SOUND_39->setEnabled(false);
   ui->BTN_SOUND_40->setEnabled(false);
   ui->BTN_SOUND_41->setEnabled(false);
   ui->BTN_SOUND_42->setEnabled(false);
   ui->BTN_SOUND_43->setEnabled(false);
   ui->BTN_SOUND_44->setEnabled(false);
   ui->BTN_SOUND_45->setEnabled(false);
   ui->BTN_SOUND_46->setEnabled(false);
   ui->BTN_SOUND_47->setEnabled(false);
  ui->BTN_SOUND_100->setEnabled(false);
  ui->BTN_SOUND_101->setEnabled(false);
  ui->BTN_SOUND_102->setEnabled(false);
  ui->BTN_SOUND_103->setEnabled(false);
  ui->BTN_SOUND_104->setEnabled(false);
  ui->BTN_SOUND_105->setEnabled(false);
   ui->BTN_SOUND_88->setEnabled(false);
   ui->BTN_SOUND_89->setEnabled(false);
   ui->BTN_SOUND_90->setEnabled(false);
   ui->BTN_SOUND_91->setEnabled(false);
   ui->BTN_SOUND_92->setEnabled(false);
   ui->BTN_SOUND_93->setEnabled(false);
   ui->BTN_SOUND_94->setEnabled(false);
   ui->BTN_SOUND_95->setEnabled(false);
   ui->BTN_SOUND_96->setEnabled(false);
   ui->BTN_SOUND_97->setEnabled(false);
   ui->BTN_SOUND_98->setEnabled(false);
   ui->BTN_SOUND_99->setEnabled(false);
   ui->BTN_wa->setEnabled(false);
   ui->BTN_oh->setEnabled(false);
}



void ModelDialog::on_BTN_QUICK_STOP_clicked()
{
    playdemoSound[120]->stop();
    playdemoSound[121]->stop();

//    for(int i=0;i<150;i++)
//    {
//        if(playdemoSound[0]->QSound())
//    }
}

