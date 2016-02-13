#ifndef TRANSITIONS_H
#define TRANSITIONS_H


enum class transition {
    // Wait_HandPoseGen_msg, /**wait for msg that containts the info about the object*/
    Wait_Closed_Softhand,
    Wait_Open_Softhand,
    Error_arrived,
    Grasp_Obj,
    Removed_Obj,
    Overtune_table,
    started,
    failed,

};

typedef std::pair<transition,bool> transition_type;

enum class transition_id {
    // Wait_HandPoseGen_msg, /**wait for msg that containts the info about the object*/
    Softhand_c,
    Softhand_o,
    Vito_home,
    Vito_grasp,
    Vito_Removed,
    Vito_Overtune,
    Gen_pose,
    Vito_trash,
    
};


















#endif 