#ifndef __MOTEUS_COMMAND_TYPE_HH__
#define __MOTEUS_COMMAND_TYPE_HH__

#include <vector>

struct PowerDistData {
    float voltage = 0;
    float current = 0;
    float temperature = 0;
    float energy = 0;
};

struct ImuData {
    float attitudeQuaternion[4] = {1, 0, 0, 0};
    float rotationRate[3] = {0, 0, 0};
    float accel[3] = {0, 0, 0};

    float bias[3] = {0, 0, 0};
    float biasUncertainty[3] = {0, 0, 0};
    float attitudeUncertainty[4] = {0, 0, 0, 0};
};

struct MoteusCommand {
    // sleep position
    float ref_position[3] = {0, 0, 0};
    float ref_velocity[3] = {0, 0, 0};
    float ref_ff_torque[3] = {0, 0, 0};
    float act_q_current[3] = {0, 0, 0};
    float act_d_current[3] = {0, 0, 0};
    float act_position[3] = {0, 0, 0};
    float act_velocity[3] = {0, 0, 0};
    float act_ff_torque[3] = {0, 0, 0};
    float kp_scale[3] = {0.0, 0.0, 0.0};
    float kd_scale[3] = {0.0, 0.0, 0.0};
    float max_torque = 0.0;

    // filtering coefficient: 1.0-Full filtering, no motion, 0.0-No filtering
    float alpha = 0.5;
    
    ImuData imuData;
    
    PowerDistData powerDistData;

    MoteusCommand() {}
};

#endif
