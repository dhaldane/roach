void SetupTimer5();
void setPitchControlFlag(char state);
void tailCtrlSetup();
void resetBodyAngle();
void setPitchSetpoint(long setpoint);
void setRollSetpoint(long setpoint);
void setYawSetpoint(long setpoint);
void setLegSetpoint(long length);
void setPushoffCmd(long cmd);
void updateViconAngle(long* new_vicon_angle);

//extern long body_angle[3];
