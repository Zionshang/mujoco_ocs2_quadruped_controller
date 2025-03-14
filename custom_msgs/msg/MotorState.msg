float32 position     # 电机当前角度（rad）
float32 velocity     # 电机当前速度（rad/s）
float32 torque       # 电机估计力矩（N*m）
float32 temperature  # 电机温度 (℃)
int8 state           # 0-复位  1-标定  2-运行
MotorError error     # 故障