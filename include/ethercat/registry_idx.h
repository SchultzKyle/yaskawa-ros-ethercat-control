#ifndef ESA_EWDL_ETHERCAT_REGISTRY_IDX_H
#define ESA_EWDL_ETHERCAT_REGISTRY_IDX_H

#define SYNC_MANAGER_OUTPUT_PARAMETER_IDX           0x1C32
#define SYNC_MANAGER_INPUT_PARAMETER_IDX            0x1C33

#define MAX_TORQUE                                  0x6072
#define TORQUE_UNIT                                 0x2703
// CiA 402
#define ERROR_CODE_IDX                              0x603F
#define CONTROL_WORD_IDX                            0x6040
#define STATUS_WORD_IDX                             0x6041
#define MODE_OF_OPERATION_IDX                       0x6060
#define MODE_OF_OPERATION_DISPLAY_IDX               0x6061
#define POSITION_ACTUAL_VALUE_IDX                   0x6064
#define VELOCITY_ACTUAL_VALUE_IDX                   0x606C
#define TORQUE_ACTUAL_VALUE_IDX                     0x6077
#define TARGET_TORQUE_IDX                           0x6071
#define TORQUE_DEMAND_VALUE_IDX                     0x6074
#define TARGET_POSITION_IDX                         0x607A
#define HOMING_METHOD_IDX                           0x6098
#define HOMING_SPEED_IDX                            0x6099
#define HOMING_ACCELERATION_IDX                     0x609A
#define TARGET_VELOCITY_IDX                         0x60FF
// Manufacturer Specific
#define CLEAR_ALARM_IDX                             0x2006
#define STATUS_CODE_IDX                             0x200B
#define ZERO_POSITION_IDX                           0x200C
#define ALARM_CODE_IDX                              0x200F
#define TORQUE_USER_UNIT_IDX                        0x2704

#endif
