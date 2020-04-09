#ifndef js_qp_h
#define js_qp_h

#include "rbdl/rbdl.h"
#include "QuadProg++.hh"

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

class JS_QP
{
private:
    int NUMCOLS;//length of X
    int NUMEQ;
    int NUMINEQ;

    MatrixNd A_ineq,B_ineq;
    MatrixNd A_eq,B_eq;

public:
    MatrixNd H,F;
    MatrixNd X;


public:
    void setNums(int _xlength, int numEqConstraints, int numIneqConstraints)
    {
        NUMCOLS = _xlength;//should be 18+12+3*contact
        NUMEQ = numEqConstraints;//NUMCOLS(dynamics) + NUMCOLS(contactconstraints + swingleg)
        //contact to be weight?
        NUMINEQ = numIneqConstraints;//friction cone approximation -ufz<fx<ufz, -ufz<fy<ufz ->4

        //maybe matrix assignment here
        X = MatrixNd::Zero(NUMCOLS,1);
        A_eq = MatrixNd::Zero(NUMEQ, NUMCOLS);
        B_eq = MatrixNd::Zero(NUMEQ, 1);

        A_ineq = MatrixNd::Zero(NUMINEQ, NUMCOLS);
        B_ineq = MatrixNd::Zero(NUMINEQ, 1);


        H = MatrixNd::Zero(NUMCOLS,NUMCOLS);
        F = MatrixNd::Zero(NUMCOLS,1);
    }

    void make_EQ(MatrixNd A, MatrixNd b)
    {
        //Aeq*X = Beq
        A_eq = A;
        B_eq = b;
    }

    void make_IEQ(MatrixNd A, MatrixNd b)
    {
        // Aineq*X < Bineq
        A_ineq = A;
        B_ineq = b;
    }

    void make_HF(MatrixNd A, MatrixNd b)
    {
        //min (0.5* x H x + F x)
        H = A.transpose() * A + MatrixNd::Identity(NUMCOLS,NUMCOLS)*0.01;
        F = -A.transpose() * b;

    }

    MatrixNd solve_QP()
    {
        quadprogpp::Vector<double> outX;
        quadprogpp::Matrix<double> G;
        quadprogpp::Vector<double> g0;
        quadprogpp::Matrix<double> CE;
        quadprogpp::Vector<double> ce0;
        quadprogpp::Matrix<double> CI;
        quadprogpp::Vector<double> ci0;
        //min 0.5 * x G x + g0 x
        //CE^T x + ce0 = 0
        //CI^T x + ci0 >= 0

        G.resize(NUMCOLS,NUMCOLS);
        g0.resize(NUMCOLS);
        for(int i=0;i<NUMCOLS;i++)
        {
            for(int j=0;j<NUMCOLS;j++)
            {
                G[i][j] = H(i,j);
            }
            g0[i] = F(i,0);
        }
        CE.resize(NUMCOLS,NUMEQ);
        ce0.resize(NUMEQ);
        for(int i=0;i<NUMCOLS;i++)
        {
            for(int j=0;j<NUMEQ;j++)
            {
                CE[i][j] = -A_eq(j,i);
            }
        }
        for(int j=0;j<NUMEQ;j++)
        {
            ce0[j] = B_eq(j,0);
        }
        CI.resize(NUMCOLS,NUMINEQ);
        ci0.resize(NUMINEQ);
        for(int i=0;i<NUMCOLS;i++)
        {
            for(int j=0;j<NUMINEQ;j++)
            {
                CI[i][j] = -A_ineq(j,i);
            }
        }
        for(int j=0;j<NUMINEQ;j++)
        {
            ci0[j] = B_ineq(j,0);
        }
        outX.resize(NUMCOLS);

        solve_quadprog(G, g0, CE, ce0, CI, ci0, outX);
        for(int i=0;i<NUMCOLS;i++)
        {
            X(i,0) = outX[i];
        }

        return X;

    }

public:

};

class JS_RBDL
{

public:
    Model* Robot;

    //right arm
    Body b_rsp, b_rsr, b_rsy, b_reb, b_rwy, b_rwp;
    Joint j_rsp, j_rsr, j_rsy, j_reb, j_rwy, j_rwp;
    int n_rsp, n_rsr, n_rsy, n_reb, n_rwy, n_rwp;

    double m_rsp, m_rsr, m_rsy, m_reb, m_rwy, m_rwp;
    Vector3d mc_rsp, mc_rsr, mc_rsy, mc_reb, mc_rwy, mc_rwp;
    Matrix3d I_rsp, I_rsr, I_rsy, I_reb, I_rwy, I_rwp;

    //left arm
    Body b_lsp, b_lsr, b_lsy, b_leb, b_lwy, b_lwp;
    Joint j_lsp, j_lsr, j_lsy, j_leb, j_lwy, j_lwp;
    int n_lsp, n_lsr, n_lsy, n_leb, n_lwy, n_lwp;

    double m_lsp, m_lsr, m_lsy, m_leb, m_lwy, m_lwp;
    Vector3d mc_lsp, mc_lsr, mc_lsy, mc_leb, mc_lwy, mc_lwp;
    Matrix3d I_lsp, I_lsr, I_lsy, I_leb, I_lwy, I_lwp;

    //right leg
    Body b_rhy, b_rhr, b_rhp, b_rkn, b_rap, b_rar;
    Joint j_rhy, j_rhr, j_rhp, j_rkn, j_rap, j_rar;
    int n_rhy, n_rhr, n_rhp, n_rkn, n_rap, n_rar;

    double m_rhy, m_rhr, m_rhp, m_rkn, m_rap, m_rar;
    Vector3d mc_rhy, mc_rhr, mc_rhp, mc_rkn, mc_rap, mc_rar;
    Matrix3d I_rhy, I_rhr, I_rhp, I_rkn, I_rap, I_rar;

    //left leg
    Body b_lhy, b_lhr, b_lhp, b_lkn, b_lap, b_lar;
    Joint j_lhy, j_lhr, j_lhp, j_lkn, j_lap, j_lar;
    int n_lhy, n_lhr, n_lhp, n_lkn, n_lap, n_lar;

    double m_lhy, m_lhr, m_lhp, m_lkn, m_lap, m_lar;
    Vector3d mc_lhy, mc_lhr, mc_lhp, mc_lkn, mc_lap, mc_lar;
    Matrix3d I_lhy, I_lhr, I_lhp, I_lkn, I_lap, I_lar;

    //waist and pelvis
    Body b_torso, b_pel;
    Joint j_wst, j_pel;
    int n_torso, n_pel;

    double m_torso, m_pel;
    Vector3d mc_torso, mc_pel;
    Matrix3d I_torso, I_pel;




    //# of joint -> 6*4 + 6 + 1 = 31
    //jacobian

    int n_dof = 31;
    MatrixNd jacob_arm_right = MatrixNd::Zero(6,n_dof);
    MatrixNd jacob_arm_left = MatrixNd::Zero(6,n_dof);
    MatrixNd jacob_leg_right = MatrixNd::Zero(6,n_dof);
    MatrixNd jacob_leg_left = MatrixNd::Zero(6,n_dof);
    MatrixNd jacob_torso = MatrixNd::Zero(6,n_dof);
    MatrixNd jacob_pel = MatrixNd::Zero(6,n_dof);

    MatrixNd jacob_com = MatrixNd::Zero(3,n_dof);

    MatrixNd jacob_com_rsp = MatrixNd::Zero(3,n_dof);
    MatrixNd jacob_com_rsr = MatrixNd::Zero(3,n_dof);
    MatrixNd jacob_com_rsy = MatrixNd::Zero(3,n_dof);
    MatrixNd jacob_com_reb = MatrixNd::Zero(3,n_dof);
    MatrixNd jacob_com_rwy = MatrixNd::Zero(3,n_dof);
    MatrixNd jacob_com_rwp = MatrixNd::Zero(3,n_dof);

    MatrixNd jacob_com_lsp = MatrixNd::Zero(3,n_dof);
    MatrixNd jacob_com_lsr = MatrixNd::Zero(3,n_dof);
    MatrixNd jacob_com_lsy = MatrixNd::Zero(3,n_dof);
    MatrixNd jacob_com_leb = MatrixNd::Zero(3,n_dof);
    MatrixNd jacob_com_lwy = MatrixNd::Zero(3,n_dof);
    MatrixNd jacob_com_lwp = MatrixNd::Zero(3,n_dof);

    MatrixNd jacob_com_rhy = MatrixNd::Zero(3,n_dof);
    MatrixNd jacob_com_rhr = MatrixNd::Zero(3,n_dof);
    MatrixNd jacob_com_rhp = MatrixNd::Zero(3,n_dof);
    MatrixNd jacob_com_rkn = MatrixNd::Zero(3,n_dof);
    MatrixNd jacob_com_rap = MatrixNd::Zero(3,n_dof);
    MatrixNd jacob_com_rar = MatrixNd::Zero(3,n_dof);

    MatrixNd jacob_com_lhy = MatrixNd::Zero(3,n_dof);
    MatrixNd jacob_com_lhr = MatrixNd::Zero(3,n_dof);
    MatrixNd jacob_com_lhp = MatrixNd::Zero(3,n_dof);
    MatrixNd jacob_com_lkn = MatrixNd::Zero(3,n_dof);
    MatrixNd jacob_com_lap = MatrixNd::Zero(3,n_dof);
    MatrixNd jacob_com_lar = MatrixNd::Zero(3,n_dof);

    MatrixNd jacob_com_pel = MatrixNd::Zero(3,n_dof);
    MatrixNd jacob_com_torso = MatrixNd::Zero(3,n_dof);

    MatrixNd jacob_temp = MatrixNd::Zero(6,n_dof);

    JS_RBDL()
    {
        Robot = new Model();
        Robot->gravity = Vector3d(0,0,-9.81);

        //right arm
        m_rsp = 1e-6;
        m_rsr = 1e-6;
        m_rsy = 2.3147;
        m_reb = 0.5542;
        m_rwy = 1e-6;
        m_rwp = 0.2247;

        mc_rsp = MatrixNd::Zero(3,1);
        mc_rsr = MatrixNd::Zero(3,1);
        mc_rsy = Vector3d(6.2,17.8,-45.1);
        mc_reb = Vector3d(0.3,-0.6,-47);
        mc_rwy = MatrixNd::Zero(3,1);
        mc_rwp = Vector3d(3.3,-1.5,-63.5);

        I_rsp = MatrixNd::Identity(3,3)*1e-6;
        I_rsr = MatrixNd::Identity(3,3)*1e-6;
        I_rsy = MatrixNd::Identity(3,3)*1e-6;
        I_reb = MatrixNd::Identity(3,3)*1e-6;
        I_rwy = MatrixNd::Identity(3,3)*1e-6;
        I_rwp = MatrixNd::Identity(3,3)*1e-6;

        b_rsp = Body(m_rsp,mc_rsp,I_rsp); //mass (double) , mass center (vector), inertia (matrix)
        b_rsr = Body(m_rsr,mc_rsr,I_rsr);
        b_rsy = Body(m_rsy,mc_rsy,I_rsy);
        b_reb = Body(m_reb,mc_reb,I_reb);
        b_rwy = Body(m_rwy,mc_rwy,I_rwy);
        b_rwp = Body(m_rwp,mc_rwp,I_rwp);

        j_rsp = Joint(JointTypeRevoluteY);
        j_rsr = Joint(JointTypeRevoluteX);
        j_rsy = Joint(JointTypeRevoluteZ);
        j_reb = Joint(JointTypeRevoluteY);
        j_rwy = Joint(JointTypeRevoluteZ);
        j_rwp = Joint(JointTypeRevoluteY);

        //left arm
        m_lsp = 1e-6;
        m_lsr = 1e-6;
        m_lsy = 2.3147;
        m_leb = 0.5542;
        m_lwy = 1e-6;
        m_lwp = 0.2247;

        mc_lsp = MatrixNd::Zero(3,1);
        mc_lsr = MatrixNd::Zero(3,1);
        mc_lsy = Vector3d(6.2,-17.8,-45.1);
        mc_leb = Vector3d(0.3,0.6,-47);
        mc_lwy = MatrixNd::Zero(3,1);
        mc_lwp = Vector3d(3.3,1.5,-63.5);

        I_lsp = MatrixNd::Identity(3,3)*1e-6;
        I_lsr = MatrixNd::Identity(3,3)*1e-6;
        I_lsy = MatrixNd::Identity(3,3)*1e-6;
        I_leb = MatrixNd::Identity(3,3)*1e-6;
        I_lwy = MatrixNd::Identity(3,3)*1e-6;
        I_lwp = MatrixNd::Identity(3,3)*1e-6;

        b_lsp = Body(m_lsp,mc_lsp,I_lsp); //mass (double) , mass center (vector), inertia (matrix)
        b_lsr = Body(m_lsr,mc_lsr,I_lsr);
        b_lsy = Body(m_lsy,mc_lsy,I_lsy);
        b_leb = Body(m_leb,mc_leb,I_leb);
        b_lwy = Body(m_lwy,mc_lwy,I_lwy);
        b_lwp = Body(m_lwp,mc_lwp,I_lwp);

        j_lsp = Joint(JointTypeRevoluteY);
        j_lsr = Joint(JointTypeRevoluteX);
        j_lsy = Joint(JointTypeRevoluteZ);
        j_leb = Joint(JointTypeRevoluteY);
        j_lwy = Joint(JointTypeRevoluteZ);
        j_lwp = Joint(JointTypeRevoluteY);


        //right leg
        m_rhy = 1e-6;
        m_rhr = 1e-6;
        m_rhp = 6.3512;
        m_rkn = 1.9592;
        m_rap = 1e-6;
        m_rar = 2.6626;

        mc_rhy =  MatrixNd::Zero(3,1);
        mc_rhr =  MatrixNd::Zero(3,1);
        mc_rhp =  Vector3d(17.5,-9.9,-99.5);
        mc_rkn =  Vector3d(14.6,-14.6,-184.5);
        mc_rap =  MatrixNd::Zero(3,1);
        mc_rar =  Vector3d(21.6,-1.4,-16);

        I_rhy = MatrixNd::Identity(3,3)*1e-6;
        I_rhr = MatrixNd::Identity(3,3)*1e-6;
        I_rhp = MatrixNd::Identity(3,3)*1e-6;
        I_rkn = MatrixNd::Identity(3,3)*1e-6;
        I_rap = MatrixNd::Identity(3,3)*1e-6;
        I_rar = MatrixNd::Identity(3,3)*1e-6;

        b_rhy = Body(m_rhy,mc_rhy,I_rhy); //mass (double) , mass center (vector), inertia (matrix)
        b_rhr = Body(m_rhr,mc_rhr,I_rhr);
        b_rhp = Body(m_rhp,mc_rhp,I_rhp);
        b_rkn = Body(m_rkn,mc_rkn,I_rkn);
        b_rap = Body(m_rap,mc_rap,I_rap);
        b_rar = Body(m_rar,mc_rar,I_rar);


        j_rhy = Joint(JointTypeRevoluteZ);
        j_rhr = Joint(JointTypeRevoluteX);
        j_rhp = Joint(JointTypeRevoluteY);
        j_rkn = Joint(JointTypeRevoluteY);
        j_rap = Joint(JointTypeRevoluteY);
        j_rar = Joint(JointTypeRevoluteX);

        //left leg
        m_lhy = 1e-6;
        m_lhr = 1e-6;
        m_lhp = 6.3512;
        m_lkn = 1.9592;
        m_lap = 1e-6;
        m_lar = 2.6626;

        mc_lhy = MatrixNd::Zero(3,1);
        mc_lhr = MatrixNd::Zero(3,1);
        mc_lhp = Vector3d(17.5,9.9,-99.5);
        mc_lkn = Vector3d(14.6,14.6,-184.5);
        mc_lap = MatrixNd::Zero(3,1);
        mc_lar = Vector3d(21.6,1.4,-16);

        I_lhy = MatrixNd::Identity(3,3)*1e-6;
        I_lhr = MatrixNd::Identity(3,3)*1e-6;
        I_lhp = MatrixNd::Identity(3,3)*1e-6;
        I_lkn = MatrixNd::Identity(3,3)*1e-6;
        I_lap = MatrixNd::Identity(3,3)*1e-6;
        I_lar = MatrixNd::Identity(3,3)*1e-6;

        b_lhy = Body(m_lhy,mc_lhy,I_lhy); //mass (double) , mass center (vector), inertia (matrix)
        b_lhr = Body(m_lhr,mc_lhr,I_lhr);
        b_lhp = Body(m_lhp,mc_lhp,I_lhp);
        b_lkn = Body(m_lkn,mc_lkn,I_lkn);
        b_lap = Body(m_lap,mc_lap,I_lap);
        b_lar = Body(m_lar,mc_lar,I_lar);

        j_lhy = Joint(JointTypeRevoluteZ);
        j_lhr = Joint(JointTypeRevoluteX);
        j_lhp = Joint(JointTypeRevoluteY);
        j_lkn = Joint(JointTypeRevoluteY);
        j_lap = Joint(JointTypeRevoluteY);
        j_lar = Joint(JointTypeRevoluteX);

        //wasit and pelvis
        m_pel = 3.8736;
        m_torso = 7.2025;

        mc_pel = Vector3d(-11.9,0,132.3);
        mc_torso = Vector3d(-11.5,0.0,134.7);

        I_pel = MatrixNd::Identity(3,3)*1e-6;
        I_torso = MatrixNd::Identity(3,3)*1e-6;

        b_pel = Body(m_pel,mc_pel,I_pel); //mass (double) , mass center (vector), inertia (matrix)
        b_torso = Body(m_torso,mc_torso,I_torso);

        j_pel = Joint(JointTypeFloatingBase);
        j_wst = Joint(JointTypeRevoluteZ);





        Vector3d zv = Vector3d(0.0,0.0,0.0);
        Vector3d pel2torso = Vector3d(0.0,0.0,172.5);

        //pel to torso
        n_pel = Robot->AddBody(0,Xtrans(zv),j_pel,b_pel,"PEL");
        n_torso = Robot->AddBody(n_pel,Xtrans(pel2torso),j_wst,b_torso,"WST");

        //torso to RH
        Vector3d sy2eb = Vector3d(22.0,0.0,-182.0);
        Vector3d eb2wy = Vector3d(-22.0,0.0,-164.0);
        Vector3d torso2rsp = Vector3d(0,-215,195);

        n_rsp = Robot->AddBody(n_torso,Xtrans(torso2rsp),j_rsp,b_rsp,"RSP");
        n_rsr = Robot->AddBody(n_rsp,Xtrans(zv),j_rsr,b_rsr,"RSR");
        n_rsy = Robot->AddBody(n_rsr,Xtrans(zv),j_rsy,b_rsy,"RSY");
        n_reb = Robot->AddBody(n_rsy,Xtrans(sy2eb),j_reb,b_reb,"REB");
        n_rwy = Robot->AddBody(n_reb,Xtrans(eb2wy),j_rwy,b_rwy,"RWY");
        n_rwp = Robot->AddBody(n_rwy,Xtrans(zv),j_rwp,b_rwp,"RWP");

        //torso to LH
        Vector3d torso2lsp = Vector3d(0,215,195);

        n_lsp = Robot->AddBody(n_torso,Xtrans(torso2lsp),j_lsp,b_lsp,"LSP");
        n_lsr = Robot->AddBody(n_lsp,Xtrans(zv),j_lsr,b_lsr,"LSR");
        n_lsy = Robot->AddBody(n_lsr,Xtrans(zv),j_lsy,b_lsy,"LSY");
        n_leb = Robot->AddBody(n_lsy,Xtrans(sy2eb),j_leb,b_leb,"LEB");
        n_lwy = Robot->AddBody(n_leb,Xtrans(eb2wy),j_lwy,b_lwy,"LWY");
        n_lwp = Robot->AddBody(n_lwy,Xtrans(zv),j_lwp,b_lwp,"LWP");

        //pel to RF
        Vector3d pel2rhy = Vector3d(0,-88.5,0); // TBC
        Vector3d hp2kn = Vector3d(0,0,-280); // TBC
        Vector3d kn2ap = Vector3d(0,0,-278); // TBC

        n_rhy = Robot->AddBody(n_pel,Xtrans(pel2rhy),j_rhy,b_rhy,"RHY");
        n_rhr = Robot->AddBody(n_rhy,Xtrans(zv),j_rhr,b_rhr,"RHR");
        n_rhp = Robot->AddBody(n_rhr,Xtrans(zv),j_rhp,b_rhp,"RHP");
        n_rkn = Robot->AddBody(n_rhp,Xtrans(hp2kn),j_rkn,b_rkn,"RKN");
        n_rap = Robot->AddBody(n_rkn,Xtrans(kn2ap),j_rap,b_rap,"RAP");
        n_rar = Robot->AddBody(n_rap,Xtrans(zv),j_rar,b_rar,"RAR");

        //pel to LF
        Vector3d pel2lhy = Vector3d(0,88.5,0); // TBC

        n_lhy = Robot->AddBody(n_pel,Xtrans(pel2lhy),j_lhy,b_lhy,"LHY");
        n_lhr = Robot->AddBody(n_lhy,Xtrans(zv),j_lhr,b_lhr,"LHR");
        n_lhp = Robot->AddBody(n_lhr,Xtrans(zv),j_lhp,b_lhp,"LHP");
        n_lkn = Robot->AddBody(n_lhp,Xtrans(hp2kn),j_lkn,b_lkn,"LKN");
        n_lap = Robot->AddBody(n_lkn,Xtrans(kn2ap),j_lap,b_lap,"LAP");
        n_lar = Robot->AddBody(n_lap,Xtrans(zv),j_lar,b_lar,"LAR");

    }

    void Calc_Jacob_COM(VectorNd _Q, double &m_robot, Vector3d &COM)
    {
        //get COM 3D jacobian

        VectorNd _dQ = VectorNd::Zero(_Q.rows());
        Utils::CalcCenterOfMass(*Robot,_Q,_dQ,m_robot,COM,NULL,NULL,1);

        jacob_com = MatrixNd::Zero(3,n_dof);

        CalcPointJacobian(*Robot,_Q,n_pel,mc_pel,jacob_com_pel);
        CalcPointJacobian(*Robot,_Q,n_torso,mc_torso,jacob_com_torso);
        jacob_com += jacob_com_pel*m_pel/m_robot;
        jacob_com += jacob_com_torso*m_torso/m_robot;

        CalcPointJacobian(*Robot,_Q,n_rsp,mc_rsp,jacob_com_rsp);
        CalcPointJacobian(*Robot,_Q,n_rsr,mc_rsr,jacob_com_rsr);
        CalcPointJacobian(*Robot,_Q,n_rsy,mc_rsy,jacob_com_rsy);
        CalcPointJacobian(*Robot,_Q,n_reb,mc_reb,jacob_com_reb);
        CalcPointJacobian(*Robot,_Q,n_rwy,mc_rwy,jacob_com_rwy);
        CalcPointJacobian(*Robot,_Q,n_rwp,mc_rwp,jacob_com_rwp);
        jacob_com += jacob_com_rsp*m_rsp/m_robot;
        jacob_com += jacob_com_rsr*m_rsr/m_robot;
        jacob_com += jacob_com_rsy*m_rsy/m_robot;
        jacob_com += jacob_com_reb*m_reb/m_robot;
        jacob_com += jacob_com_rwy*m_rwy/m_robot;
        jacob_com += jacob_com_rwp*m_rwp/m_robot;


        CalcPointJacobian(*Robot,_Q,n_lsp,mc_lsp,jacob_com_lsp);
        CalcPointJacobian(*Robot,_Q,n_lsr,mc_lsr,jacob_com_lsr);
        CalcPointJacobian(*Robot,_Q,n_lsy,mc_lsy,jacob_com_lsy);
        CalcPointJacobian(*Robot,_Q,n_leb,mc_leb,jacob_com_leb);
        CalcPointJacobian(*Robot,_Q,n_lwy,mc_lwy,jacob_com_lwy);
        CalcPointJacobian(*Robot,_Q,n_lwp,mc_lwp,jacob_com_lwp);
        jacob_com += jacob_com_lsp*m_lsp/m_robot;
        jacob_com += jacob_com_lsr*m_lsr/m_robot;
        jacob_com += jacob_com_lsy*m_lsy/m_robot;
        jacob_com += jacob_com_leb*m_leb/m_robot;
        jacob_com += jacob_com_lwy*m_lwy/m_robot;
        jacob_com += jacob_com_lwp*m_lwp/m_robot;

        CalcPointJacobian(*Robot,_Q,n_rhy,mc_rhy,jacob_com_rhy);
        CalcPointJacobian(*Robot,_Q,n_rhr,mc_rhr,jacob_com_rhr);
        CalcPointJacobian(*Robot,_Q,n_rhp,mc_rhp,jacob_com_rhp);
        CalcPointJacobian(*Robot,_Q,n_rkn,mc_rkn,jacob_com_rkn);
        CalcPointJacobian(*Robot,_Q,n_rap,mc_rap,jacob_com_rap);
        CalcPointJacobian(*Robot,_Q,n_rar,mc_rar,jacob_com_rar);
        jacob_com += jacob_com_rhy*m_rhy/m_robot;
        jacob_com += jacob_com_rhr*m_rhr/m_robot;
        jacob_com += jacob_com_rhp*m_rhp/m_robot;
        jacob_com += jacob_com_rkn*m_rkn/m_robot;
        jacob_com += jacob_com_rap*m_rap/m_robot;
        jacob_com += jacob_com_rar*m_rar/m_robot;

        CalcPointJacobian(*Robot,_Q,n_lhy,mc_lhy,jacob_com_lhy);
        CalcPointJacobian(*Robot,_Q,n_lhr,mc_lhr,jacob_com_lhr);
        CalcPointJacobian(*Robot,_Q,n_lhp,mc_lhp,jacob_com_lhp);
        CalcPointJacobian(*Robot,_Q,n_lkn,mc_lkn,jacob_com_lkn);
        CalcPointJacobian(*Robot,_Q,n_lap,mc_lap,jacob_com_lap);
        CalcPointJacobian(*Robot,_Q,n_lar,mc_lar,jacob_com_lar);
        jacob_com += jacob_com_lhy*m_lhy/m_robot;
        jacob_com += jacob_com_lhr*m_lhr/m_robot;
        jacob_com += jacob_com_lhp*m_lhp/m_robot;
        jacob_com += jacob_com_lkn*m_lkn/m_robot;
        jacob_com += jacob_com_lap*m_lap/m_robot;
        jacob_com += jacob_com_lar*m_lar/m_robot;

    }

};

#endif
