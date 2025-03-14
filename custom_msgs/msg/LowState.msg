uint16 robot_id              # SN码 原robotID
uint8 mode                  # 电机控制模式
MotorState[16] motor_state   # 12个关节电机
int16[4] foot_force          # 足端的力传感器反馈的力
int8 temperature            # 机舱内部温度
RobotError robot_error       # 机器人硬件的报错
BmsState bms_state          # 电池的状态
