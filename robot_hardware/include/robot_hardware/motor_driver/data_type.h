#ifndef _DATA_TYPE_H_
#define _DATA_TYPE_H_

// 电机相关的定义
#define P_MIN -12.5f
#define P_MAX 12.5f
#define V_MIN -15.0f
#define V_MAX 15.0f
#define KP_MIN 0.0f
#define KP_MAX 5000.0f
#define KD_MIN 0.0f
#define KD_MAX 100.0f
#define T_MIN -120.0f
#define T_MAX 120.0f

// 下发数据的大小
#define CMD_BUFFER_SIZE 16
#define CONTROL_BUFFER_SIZE 88

union Command
{
    unsigned char data;

    struct
    {
        unsigned char state_light : 3;               // 状态灯运行方式
        unsigned char spotlight : 1;                 // 打开照明灯，持续发送
        unsigned char imu_reset : 1;                 // imu复位，1为拉低复位，0无动作
        unsigned char not_used : 1;                  // 未使用
        unsigned char motor_power_supply : 1;        // 打开电机供电，需要一直发，且第一上电需要等待4s
        unsigned char motor_charging_electrodes : 1; // 自充电极，持续发送
    } value;
};

struct RobotState
{
    union
    {
        unsigned short data;

        struct
        {
            unsigned char not_used : 7;       // 未使用部分
            unsigned char imu_reset : 1;      // imu复位 1：imu复位中 0：imu无动作
            unsigned char emergency_stop : 1; // 远程急停 1: 急停触发 0：急停无动作
            unsigned char charging_state : 1; // 自动充电状态 1：打开 0：关闭
            unsigned char fan_0_error : 1;    // 风扇一 1：故障 0：正常
            unsigned char fan_1_error : 1;    // 风扇二 1：故障 0：正常
            unsigned char low_power : 1;      // 电量低 1：低于20% 0：正常
            unsigned char power_warning : 1;  // 电量报警 1: 低于10% 0：正常
            unsigned char BMS_overtime : 1;   // 电池通信 1：异常 0：正常
            unsigned char motor_poweron : 1;  // 电机电源 1：打开 0：关闭
        } value;
    } state;

    unsigned short temp; // 板子的温度

    // 电池状态
    struct
    {
        unsigned short pack_soc;           // 剩余电量百分比
        unsigned short pack_soh;           // 电池健康状态
        unsigned short pack_vol;           // 电池组总电压，单位0.1V
        unsigned short charge_pack_vol;    // 充电口总电压，单位0.1V
        unsigned short discharge_pack_vol; // 放电口总电压，单位0.1V
        int pack_curr;                     // 电池组总电流，单位mA
        unsigned int charge_curr;          // 充电电流，单位mA
        unsigned int discharge_curr;       // 放电电流，单位mA

        // 电池反馈状态
        union
        {
            unsigned int data;

            struct
            {
                unsigned char FB_DSG_MOS : 1;        // BIT0 /*电池放电MOS，0-关闭；1-打开*/
                unsigned char FB_CHG_MOS : 1;        // BIT1 /*电池充电MOS*/
                unsigned char FB_PREDSG_MOS : 1;     // BIT2 /*电池预充MOS*/
                unsigned char FB_DCDC_MOS : 1;       // BIT3 /*电池DCDC模块开关*/
                unsigned char FB_VICEDSG_MOS : 1;    // BIT4 /*副放电口MOS开关*/
                unsigned char FB_LOOP_MOS : 1;       // BIT5 /*电池总MOS，0-关闭；1-打开*/
                unsigned char FB_DSG_STA : 1;        // BIT6 /*电池放电MOS反馈状态，0-关闭；1-打开*/
                unsigned char FB_CHG_STA : 1;        // BIT7 /*电池充电MOS反馈状态*/
                unsigned char FB_PREDSG_STA : 1;     // BIT8 /*电池预充MOS反馈状态*/
                unsigned char FB_DCDC_STA : 1;       // BIT9 /*电池DCDC模块反馈状态*/
                unsigned char FB_VICEDSG_STA : 1;    // BIT10 /*副放电口反馈状态*/
                unsigned char FB_LOOP_STA : 1;       // BIT11 /*电池总MOS反馈状态*/
                unsigned char FB_CHG_PWRON_STA : 1;  // BIT12 /*电池处于充电激活状态*/
                unsigned char FB_LED_PWRON_STA : 1;  // BIT13 /*电池处于灯板激活状态*/
                unsigned char FB_FUSE_STA : 1;       // BIT14 /*保险丝状态，0-保险丝正常；1-保险丝熔断*/
                unsigned char FB_RESERVE_1 : 1;      // BIT15
                unsigned char FB_HEAT_M_STA : 1;     // BIT16 /*主机加热膜状态，0-关闭；1-开启*/
                unsigned char FB_HEAT_S_STA : 1;     // BIT17 /*从机加热膜状态*/
                unsigned char FB_MSCOM_WARRING : 1;  // BIT18 /*主从机通信失联*/
                unsigned char FB_MSCOM_ERR : 1;      // BIT19 /*主从机通信失败即将关机*/
                unsigned char FB_MAINKEY_STA : 1;    // BIT20 /*总开关状态，0-闭合；1-断开(即将断电)*/
                unsigned char FB_SLAVEKEY_STA : 1;   // BIT21 /*副开关状态*/
                unsigned char FB_LEDKEY_STA : 1;     // BIT22 /*灯板开关状态，0-断开；1-闭合(灯板上电)*/
                unsigned char FB_STOP_PRESS : 1;     // BIT23 /*急停开关按下*/
                unsigned char FB_FORCECHG_PRESS : 1; // BIT24 /*强充开关按下*/
                unsigned char FB_CFULL_STA : 1;      // BIT25 /*满电标志位*/
                unsigned char FB_CHGINPRO_STA : 1;   // BIT26 /*充电进行中状态*/
                unsigned char FB_CHGDE_INPUT : 1;    // BIT27 /*充电器接入标志位*/
                unsigned char FB_BATCHIP_ERR : 1;    // BIT28 /*采集芯片异常*/
                unsigned char FB_PRECHG_FAIL : 1;    // BIT29 /*预充失败*/
                unsigned char FB_CUTDOWN_MAND : 1;   // BIT30 /*主机下电命令*/
                unsigned char FB_RESERVE_4 : 1;      // BIT31 /*预留*/
            } value;
        } battery_feedback_state;

        // 电池保护状态
        union
        {
            unsigned int data;

            struct
            {
                unsigned char BH_VOL_ALTER_ERR : 1;        // BIT0  /*压差过大*/
                unsigned char BH_VOL_LOW_ERR : 1;          // BIT1 /*单体低压*/
                unsigned char BH_VOL_HIGH_ERR : 1;         // BIT2 /*单体高压*/
                unsigned char BH_CHG_HIGHTMP_ERR : 1;      // BIT3 /*高温充电*/
                unsigned char BH_DSG_HIGHTMP_ERR : 1;      // BIT4 /*高温放电*/
                unsigned char BH_CHG_LOWTMP_ERR : 1;       // BIT5 /*低温充电*/
                unsigned char BH_DSG_LOWTMP_ERR : 1;       // BIT6 /*低温放电*/
                unsigned char BH_CHG_OVERCURR_ERR : 1;     // BIT7 /*充电过流*/
                unsigned char BH_DSG_OVERCURR_ERR : 1;     // BIT8 /*放电过流*/
                unsigned char BH_DSG_PACKHIGH_ERR : 1;     // BIT9 /*放电口总压过压*/
                unsigned char BH_DSG_PACKLOW_ERR : 1;      // BIT10 /*放电口总压欠压*/
                unsigned char BH_CHG_PACKHIGH_ERR : 1;     // BIT11 /*充电口总压过压*/
                unsigned char BH_VICEDSG_PACKHIGH_ERR : 1; // BIT12 /*副放电口过压*/
                unsigned char BH_VICEDSG_PACKLOW_ERR : 1;  // BIT13 /*副放电口欠压*/
                unsigned char BH_VICEDSG_OVERCURR_ERR : 1; // BIT14 /*副放电口过流*/
                unsigned char BH_VICEDSG_HIGHTEMP_ERR : 1; // BIT15 /*副放电口高温*/
                unsigned char BH_VICEDSG_LOWTEMP_ERR : 1;  // BIT16 /*副放电口低温*/
                unsigned char BH_PCB_HIGHTEMP_ERR : 1;     // BIT17 /*保护板过温*/
                unsigned char BH_MOS_HIGHTEMP_ERR : 1;     // BIT18 /*MOS过温*/
                unsigned char BH_HEAT_HIGHTEMP_ERR : 1;    // BIT19 /*加热膜过温*/
                unsigned char BH_SOC_LOW_ERR : 1;          // BIT20 /*SOC过低*/
                unsigned char BH_SOC_HIGH_ERR : 1;         // BIT21 /*SOC过高*/
                unsigned char BH_TEMP_ALTER_ERR : 1;       // BIT22 /*温差过大*/
                unsigned char BH_SHORT_CUT_ERR : 1;        // BIT23 /*短路*/
                unsigned char BH_PWROFF_VOL_ERR : 1;       // BIT24 /*低电压强制关机*/
                unsigned char BH_CUTCHG_VOL_ERR : 1;       // BIT25 /*低压禁止充电*/
                unsigned char BH_PRECHG_ERR : 1;           // BIT26 /*预充失败*/
                unsigned char BH_environment_OT_ERR : 1;   // BIT27 /*环境高温保护*/
                unsigned char BH_environment_UT_ERR : 1;   // BIT28 /*环境低温保护*/
                unsigned char : 1;                         // BIT29		备用
                unsigned char : 1;                         // BIT30		备用
                unsigned char : 1;                         // BIT31 		备用
            } value;
        } battery_protection_state;
    } bms_state;
};

union RobotErrorCode
{
    unsigned short data;

    struct Value
    {
        unsigned char motor_0 : 1; // 通信故障
        unsigned char motor_1 : 1;
        unsigned char motor_2 : 1;
        unsigned char motor_3 : 1;
        unsigned char motor_4 : 1;
        unsigned char motor_5 : 1;
        unsigned char motor_6 : 1;
        unsigned char motor_7 : 1;
        unsigned char id_error : 1;
        unsigned char motor_time_out : 1;
    } value;
};

union MotorErrorCode
{
    unsigned short data;

    struct value
    {
        // 低八位为故障码
        unsigned char under_voltage : 1;
        unsigned char over_current : 1;
        unsigned char over_heat : 1;
        unsigned char magnetic_encoder : 1;   // 磁编码故障
        unsigned char HALL_encoder : 1;       // HALL编码故障
        unsigned char encoder_calibrated : 1; // 编码器未标定
        unsigned char mode_state : 2;         // 0=复位 1=标定 2=运行
        // 高八位暂未使用
        unsigned char not_use : 8;
    } value;
};

// 电机数据
struct MotorData
{
    float torque;
    float velocity;
    float position;
    float kp;
    float kd;

    // 用于返回电机状态使用
    float temperature;
    MotorErrorCode error;
};

// 机器人数据
struct RobotData
{
    int mode;
    unsigned short feet_force[2];
    float temperature;
    RobotErrorCode error;
};

/************** 交互用 *****************/

// 指令表
struct InstructionList
{
    int state_light;
    int open_light;
    int imu_reset;
    int motor_enable;
    int motor_disenable;
    int motor_power_supply;
    int motor_charging_electrodes;
};

// 电机指令
struct MotorCmd
{
    float torque;
    float velocity;
    float position;
    float kp;
    float kd;
};

// 上位指令
struct LowCmd
{
    InstructionList order;
    int mode;
    int type;
    MotorCmd motor_cmd[16];
};

// 状态表
struct StateList
{
    int motor_connect[16];
    int id_error;
    int motor_time_out;

    // 底板上传的状态
    int imu_reset;
    int emergency_stop;
    int charging_state;
    int fan_0_error;
    int fan_1_error;
    int low_power;
    int power_warning;
    int BMS_overtime;
    int motor_poweron;
    float temp;

    // 电池状态
    float battery_soc;
    int battery_soh;
    float battery_vol;
    float charge_vol;
    float discharge_vol;
    float battery_curr;
    float charge_curr;
    float discharge_curr;

    int all_motor_poweron;
};

// 电机故障表
struct MotorErrorList
{
    int under_voltage;
    int over_current;
    int over_heat;
    int magnetic_encoder;
    int HALL_encoder;
    int encoder_calibrated;
};

// 电机状态
struct MotorState
{
    float torque;
    float velocity;
    float position;
    float temperature;
    int run_state;
    MotorErrorList error;
};

// 机器人返回状态
struct LowState
{
    int mode;
    StateList state_list;
    float inside_temperature;

    float feet_force[4];
    MotorState motor_state[16];
};

#endif // _DATA_TYPE_H_