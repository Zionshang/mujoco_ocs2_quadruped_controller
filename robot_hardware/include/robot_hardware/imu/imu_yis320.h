#ifndef _IMU_YIS320_H_
#define _IMU_YIS320_H_

#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <thread>

#include "robot_hardware/imu/imu_serial.h"
#include "sensor_msgs/msg/imu.hpp"
#define PROTOCOL_FIRST_BYTE (unsigned char)0x59
#define PROTOCOL_SECOND_BYTE (unsigned char)0x53
#define YIS_OUTPUT_MIN_BYTES 7 /*header(2B) + tid(2B) + len(1B) + CK1(1B) + CK2(1B)*/

#define SINGLE_DATA_BYTES 4

/*data id define*/
#define SENSOR_TEMP_ID (unsigned char)0x01
#define ACCEL_ID (unsigned char)0x10
#define ANGLE_ID (unsigned char)0x20
#define MAGNETIC_ID (unsigned char)0x30     /*归一化值*/
#define RAW_MAGNETIC_ID (unsigned char)0x31 /*原始值*/
#define EULER_ID (unsigned char)0x40
#define QUATERNION_ID (unsigned char)0x41
#define UTC_ID (unsigned char)0x50
#define SAMPLE_TIMESTAMP_ID (unsigned char)0x51
#define DATA_READY_TIMESTAMP_ID (unsigned char)0x52
#define LOCATION_ID (unsigned char)0x60      /*普通精度位置，米级*/
#define HRES_LOCATION_ID (unsigned char)0x68 /*高精度位置，厘米级*/
#define SPEED_ID (unsigned char)0x70
#define NAV_STATUS_ID (unsigned char)0x80

/*length for specific data id*/
#define SENSOR_TEMP_DATA_LEN (unsigned char)0x2
#define ACCEL_DATA_LEN (unsigned char)12
#define ANGLE_DATA_LEN (unsigned char)12
#define MAGNETIC_DATA_LEN (unsigned char)12
#define MAGNETIC_RAW_DATA_LEN (unsigned char)12
#define EULER_DATA_LEN (unsigned char)12
#define QUATERNION_DATA_LEN (unsigned char)16
#define UTC_DATA_LEN (unsigned char)11
#define SAMPLE_TIMESTAMP_DATA_LEN (unsigned char)4
#define DATA_READY_TIMESTAMP_DATA_LEN (unsigned char)4
#define LOCATION_DATA_LEN (unsigned char)12
#define HRES_LOCATION_DATA_LEN (unsigned char)20
#define SPEED_DATA_LEN (unsigned char)12
#define NAV_STATUS_DATA_LEN (unsigned char)1

/*factor for sensor data*/
#define NOT_MAG_DATA_FACTOR 0.000001f
#define MAG_RAW_DATA_FACTOR 0.001f
#define SENSOR_TEMP_DATA_FACTOR 0.01f

/*factor for gnss data*/
#define HRES_LONG_LAT_DATA_FACTOR 0.0000000001
#define LONG_LAT_DATA_FACTOR 0.0000001
#define ALT_DATA_FACTOR 0.001f
#define SPEED_DATA_FACTOR 0.001f

#pragma pack(1)
typedef struct
{
    unsigned char header1; /*0x59*/
    unsigned char header2; /*0x53*/
    unsigned short tid;    /*1 -- 60000*/
    unsigned char len;     /*length of payload, 0 -- 255*/
} DataHeader;

typedef struct
{
    unsigned char data_id;
    unsigned char data_len;
} PayloadData;

typedef struct
{
    float x;
    float y;
    float z;
} Axis;

typedef struct
{
    float pitch; /*unit: ° (deg)*/
    float roll;
    float yaw;

    float quaternion_data0;
    float quaternion_data1;
    float quaternion_data2;
    float quaternion_data3;
} AttitudeData;

typedef struct
{
    double latitude;  /*unit: deg*/
    double longtidue; /*unit: deg*/
    float altidue;    /*unit: m*/
} Location;

typedef struct
{
    unsigned int msecond;
    unsigned short year;
    unsigned char month;
    unsigned char day;
    unsigned char hour;
    unsigned char minute;
    unsigned char second;
} UtcTime;

typedef struct
{
    float vel_n; /*unit: m/s */
    float vel_e;
    float vel_d;
} Velocity;

typedef struct
{
    unsigned char fusion : 4;
    unsigned char gnss : 4;
} NavStatus;

typedef struct
{
    float sensor_temp; /*unit: ℃*/
    Axis accel;        /*unit: m/s2*/
    Axis angle_rate;   /*unit: ° (deg)/s*/
    Axis norm_mag;     /*unit: 归一化值*/
    Axis raw_mag;      /*unit: mGauss*/

    AttitudeData attitude;

    Location location;
    Velocity vel;
    UtcTime utc;
    NavStatus status;

    unsigned int sample_timestamp;     /*unit: us*/
    unsigned int data_ready_timestamp; /*unit: us*/
} ProtocolInfo;
#pragma pack()

class ImuYIS320
{
public:
    ImuYIS320(std::string path);
    ~ImuYIS320();

    sensor_msgs::msg::Imu imu_pub_callback();
    // 获取惯导数据
    void GetImuData(sensor_msgs::msg::Imu &imu_msg);

    /**
     * 输出协议：
     * header1(0x59) + header2(0x53) + tid(2B) + payload_len(1B) + payload_data(Nbytes) +
     * ck1(1B) + ck2(1B)
     * crc校验从TID开始到payload data的最后一个字节
     */
    void AnalysisData(unsigned char *data, int length);

    // 数据校验
    int CheckSum(unsigned char *data, unsigned short len, unsigned short *checksum);
    // 数据拼接
    short GetSignedShort(unsigned char *data);
    int GetSignedInt(unsigned char *data);
    // 通过id检查数据长度
    int CheckDataLength(PayloadData header, unsigned char *data, ProtocolInfo *info);
    // 重置IMU
    void ResetImuYaw();

    Serial2 serial_;

private:
    std::thread serial_thread_;
    bool first_;
    std::string dev_path_;

    // 解析后的数据
    ProtocolInfo info_;

    // 缓冲区
    char buffer[512];
    unsigned char buffer_copy[512];
};

#endif // _IMU_YIS320_H_