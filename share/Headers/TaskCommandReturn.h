#ifndef TASKCOMMANDRETURN_H
#define TASKCOMMANDRETURN_H

typedef enum{
    // General  [1~99]
    TCMD_GENERAL_ESTOP = 1,
    // Wheel    [100~199]
    TCMD_WHEEL_MOVE_START = 100,
    TCMD_WHEEL_MOVE_DONE,
    TCMD_WHEEL_POS_CHANGE_WH2WALK_START,
    TCMD_WHEEL_POS_CHANGE_WH2WALK_DONE,
    TCMD_WHEEL_POS_CHANGE_WALK2WH_START,
    TCMD_WHEEL_POS_CHANGE_WALK2WH_DONE,
    TCMD_WHEEL_REAL_TEST_MODE_SET,
    // Walking  [200~299]
    TCMD_WALK_MOVE_START = 200,
    TCMD_WALK_MOVE_DONE,
    // Valve    [300~399]
    TCMD_VALVE_MOVE_DONE = 300,
    TCMD_VALVE_MODE_WB,
    TCMD_VALVE_APPROACH,
    TCMD_VALVE_ROTATE,
    TCMD_VALVE_READY,
    TCMD_VALVE_RETURN,
    TCMD_VALVE_MOTION_OK,
    TCMD_VALVE_MOTION_BAD,
    TCMD_VALVE_PUSHIN_FAIL,
    TCMD_VALVE_MODECHANGE_DONE,
    TCMD_VALVE_TAKE_IMAGE,
    // Door     [400~499]
    TCMD_DOOR_GO = 400,
    TCMD_DOOR_GO_NEARGO,
    TCMD_DOOR_OPEN,
    TCMD_DOOR_CROSS,
    TCMD_DOOR_PASS,
    TCMD_PUSHDOOR_READY_POSITION_GOSTART = 450,//
    TCMD_PUSHDOOR_READY_POSITION_ARRIVE,//
    TCMD_PUSHDOOR_MOTION_1_START,//
    TCMD_PUSHDOOR_MOTION_1_END,//
    TCMD_PUSHDOOR_MOTION_1_FAIL,//
    TCMD_PUSHDOOR_MOTION_1_ESTOP,// no use
    TCMD_PUSHDOOR_MOTION_2_START,//
    TCMD_PUSHDOOR_MOTION_2_END,//
    TCMD_PUSHDOOR_MOTION_2_FAIL,// no use
    TCMD_PUSHDOOR_MOTION_2_ESTOP,//
    TCMD_PUSHDOOR_MOTION_1_EM_START,//
    TCMD_PUSHDOOR_MOTION_1_EM_STOP,//
    TCMD_PUSHDOOR_MOTION_2_EM_1_START,//
    TCMD_PUSHDOOR_MOTION_2_EM_1_STOP,//
    TCMD_PUSHDOOR_MOTION_2_EM_2_START,//
    TCMD_PUSHDOOR_MOTION_2_EM_2_STOP,//
    TCMD_WALKPUSHDOOR_WALKTHROUGH_START = 475,
    TCMD_WALKPUSHDOOR_WALKTHROUGH_DONE,
    TCMD_PUSHDOOR_MODE_CHANGED,
    TCMD_PUSHDOOR_NOBE_PARA_CHANGED,
    // Hose     [500~599]
    TCMD_HOSE_APPROACH_START = 500,
    TCMD_HOSE_APPROACH_DONE,
    TCMD_HOSE_GRAB_START,
    TCMD_HOSE_GRAB_DONE,
    TCMD_WYE_APPROACH_START,
    TCMD_WYE_APPROACH_DONE,
    TCMD_WYE_ALIGN_START,
    TCMD_WYE_ALIGN_DONE,

    TCMD_PLUG_DATA_RECEIVED = 550,
    TCMD_PLUG_MOTION_GRAB_START,
    TCMD_PLUG_MOTION_GRAB_DONE,
    TCMD_PLUG_MOTION_GRAB_FAIL,
    TCMD_PLUG_MOTION_PLUGOUTMOVE_START,
    TCMD_PLUG_MOTION_PLUGOUTMOVE_DONE,
    TCMD_PLUG_MOTION_CAM_POS_DONE,
    TCMD_PLUG_MOTION_RETURN_NORMAL_DONE,
    TCMD_PLUG_MOTION_MODIFY_DONE,
    TCMD_PLUG_MOTION_PLUG_IN_START,
    TCMD_PLUG_MOTION_PLUG_IN_DONE,
    TCMD_PLUG_MOTION_ESCAPE_START,
    TCMD_PLUG_MOTION_ESCAPE_DONE,
    TCMD_PLUG_WALKING_START,
    TCMD_PLUG_WALKING_DONE,
    TCMD_PLUG_MODE_LEFT_FIRST,
    TCMD_PLUG_MODE_RIGHT_FIRST,

    // WALKINGValve [600~699]
    TCMD_WALKINGVALVE_MOVE_DONE = 600,
    TCMD_WALKINGVALVE_MODE_WB,
    TCMD_WALKINGVALVE_APPROACH,
    TCMD_WALKINGVALVE_ROTATE,
    TCMD_WALKINGVALVE_READY,
    TCMD_WALKINGVALVE_RETURN,
    TCMD_WALKINGVALVE_MOTION_OK,
    TCMD_WALKINGVALVE_MOTION_BAD,

    // Drill        [700~799]
    TCMD_DRILL_MOVE_DONE = 700,
    TCMD_DRILL_GRAB_DONE,
    TCMD_DRILL_CUT_DONE,
    TCMD_DRILL_READY1_DONE,
    TCMD_DRILL_READY2_DONE,
    TCMD_DRILL_BACKSTEP_DONE,
    TCMD_DRILL_POS_CHANGE_START,
    TCMD_DRILL_WIDEN_DONE,
    TCMD_DRILL_NARROW_DONE,
    TCMD_DRILL_GRAB_MOTIONFAIL,
    TCMD_DRILL_CUT_MOTIONFAIL,
    TCMD_DRILL_HANDLEGRAB_FAIL,
    TCMD_DRILL_RETURN_FAIL,
    TCMD_DRILL_ABANDON_DONE,
    TCMD_DRILL_TURNON_DONE,
    TCMD_DRILL_ROTATE_WST_DONE,
    TCMD_DRILL_TURN_OFF_DONE,
    TCMD_DRILL_TURNON_TRY_DONE,
    TCMD_DRILL_GRAB_HITFAIL,
    TCMD_DRILL_MIC_NOT_WORKING,
    TCMD_DRILL_TURNON_OK,

    // Debris       [800~899]
    TCMD_DEBRIS_TASKAPPROACHEND = 800,
    TCMD_DEBRIS_SINGLEMOTIONEND,
    TCMD_DEBRIS_REMOVEMOTIONEND,
    TCMD_DEBRIS_WHEELMOTIONEND,
    TCMD_DEBRIS_POSCHANGE_START,
    TCMD_DEBRIS_POSCHANGE_DONE,
    TCMD_DEBRIS_WHEELDONE,
    TCMD_DEBRIS_MOTIONREADYDONE,
    TCMD_DEBRIS_REMOVALMOTIONDONE,

    //Terrain       [900~999]
    TCMD_TERRAIN_WALKING_DONE = 900,
    TCMD_TERRAIN_WALKING_START,
    TCMD_TERRAIN_180_WALKREADY_START,
    TCMD_TERRAIN_180_WALKREADY_DONE,

    //Car Drive     [1000~1099]
    TCMD_DRIVING_STATUS_DATA = 1000,
    TCMD_DRIVING_SENSOR_DATA,
    TCMD_DRIVING_HUBO_READY,
    TCMD_DRIVING_HUBO_FINISH,
    TCMD_DRIVING_SCAN_CHECK,

    //Car Descend   [1100~1199]
    TCMD_CAR_READY_TO_SCAN = 1100,
    TCMD_CAR_RH_GRAB_DONE,
    TCMD_CAR_DESCEND_DONE,
    TCMD_CAR_STATIC_WALKING_DONE,
    TCMD_CAR_RH_STABILIZING_DONE,
    TCMD_CAR_LH_STABILIZING_DONE,
    TCMD_CAR_WALK_REDEAY_DONE,
    TCMD_CAR_STARTING_POS_DONE,
    TCMD_CAR_LEFT_GAIN_OVER_DONE,
    TCMD_CAR_RIGHT_GAIN_OVER_DONE,
    TCMD_CAR_LWFT_NULL_DONE,
    TCMD_CAR_READY_TO_SCAN_MANUAL_ROI,
    TCMD_CAR_MANUAL_MODE_READY,
    TCMD_CAR_MANUAL_MODE_NOT_READY,
    TCMD_CAR_JOYSTICK_OFF,
    TCMD_CAR_MANUAL_ONE_HAND_START,
    TCMD_CAR_MANUAL_MODE_STOP,
    TCMD_CAR_SEND_3D_ROI_CENTER,
    TCMD_CAR_SEND_RH_GRAB_POS_VISION,
    TCMD_CAR_ANK_STABILIZATION_DONE,

    //Starts        [1200~1299]
    //Surprise      [1300~1399]
    TCMD_SURPRISE_VIA_DATA_RECIEVED = 1300,
    TCMD_SURPRISE_MOVE_DONE,
    TCMD_SURPRISE_E_STOP,
    TCMD_SURPRISE_TASK_POS,
    TCMD_SURPRISE_RIGHT_GAIN_OVER_DONE,
    TCMD_SURPRISE_RIGHT_GAIN_OVER_OFF,
    TCMD_SURPRISE_LEFT_GAIN_OVER_DONE,
    TCMD_SURPRISE_LEFT_GAIN_OVER_OFF,
    TCMD_SURPRISE_FT_E_STOP_TOGGLE,
    TCMD_SURPRISE_HAND_CONTROL

}TASK_COMMAND_RETURN;

#endif // TASKCOMMANDRETURN_H
