// This JointInformation.h is for DRC-HUBO+

#ifndef JOINT_INFORMATION_H
#define JOINT_INFORMATION_H

#include <QVector>
#include <QString>

enum JointSequentialNumber
{
    RHY = 0, RHR, RHP, RKN, RAP, RAR,
    LHY, LHR, LHP, LKN, LAP, LAR,
    RSP, RSR, RSY, REB, RWY, RWP,
    LSP, LSR, LSY, LEB, LWY, LWP,
    WST,
    NKY, NK1, NK2,
    RF1, RF2, RF3, RF4, RF5,
    LF1, LF2, LF3, LF4, LF5,
    NO_OF_JOINTS
};

const QString JointNameList[NO_OF_JOINTS] = {
    "RHY", "RHR", "RHP", "RKN", "RAP", "RAR",
    "LHY", "LHR", "LHP", "LKN", "LAP", "LAR",
    "RSP", "RSR", "RSY", "REB", "RWY", "RWP",
    "LSP", "LSR", "LSY", "LEB", "LWY", "LWP",
    "WST",
    "NKY", "NK1", "NK2",
    "RF1", "RF2", "RF3", "RF4", "RF5"
    "LF1", "LF2", "LF3", "LF4", "LF5"
};

// id : sequential number at DB file (0~)
// ch : channel (0~)
const struct {
    int id;
    int ch;
} MC_ID_CH_Pairs[NO_OF_JOINTS] = {
    {0,0}, {0,1}, {1,0}, {2,0}, {3,0}, {3,1},
    {4,0}, {4,1}, {5,0}, {6,0}, {7,0}, {7,1},
    {9,0}, {9,1}, {10,0},{10,1},{13,0},{13,1},
    {11,0},{11,1},{12,0},{12,1},{14,0},{14,1},
    {8,0},
    {15,0},{15,1},{15,2},
    {16,0},{16,1},{16,2},{16,3},{16,4},
    {17,0},{17,1},{17,2},{17,3},{17,4}
};

inline int MC_GetID(int jnum){
    return MC_ID_CH_Pairs[jnum].id;
}
inline int MC_GetCH(int jnum){
    return MC_ID_CH_Pairs[jnum].ch;
}

enum JointSequentialNumber_QP
{
    FB_PEL_POS_X = 0,
    FB_PEL_POS_Y,
    FB_PEL_POS_Z,

    FB_PEL_ORI_X,
    FB_PEL_ORI_Y,
    FB_PEL_ORI_Z,

    qp_WST,


    qp_RSP, qp_RSR, qp_RSY, qp_REB, qp_RWY, qp_RWP,
    qp_LSP, qp_LSR, qp_LSY, qp_LEB, qp_LWY, qp_LWP,

    qp_RHY, qp_RHR, qp_RHP, qp_RKN, qp_RAP, qp_RAR,
    qp_LHY, qp_LHR, qp_LHP, qp_LKN, qp_LAP, qp_LAR,

    NO_OF_JOINTS_QP
};




#endif // JOINT_INFORMATION_H
