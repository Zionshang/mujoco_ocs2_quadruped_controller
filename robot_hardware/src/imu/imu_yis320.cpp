#include "robot_hardware/imu/imu_yis320.h"

ImuYIS320::ImuYIS320(std::string path)
    : serial_(),
      info_(),
      first_(true),
      dev_path_(path)
{
    serial_.OpenSerial2(dev_path_,
                       BaudRate2::RATE_921600,
                       DataSize2::SIZE_8bit,
                       CheckType2::NONE,
                       StopSize2::STOP_1bit);
    memset(buffer, 0, sizeof(buffer));
    memset(buffer_copy, 0, sizeof(buffer_copy));
}

ImuYIS320::~ImuYIS320()
{
    std::cout << "Exit the imu serial port thread..." << std::endl;
    serial_.Close2();
}

sensor_msgs::msg::Imu ImuYIS320::imu_pub_callback()
{
    sensor_msgs::msg::Imu imu_msg;
    int length = serial_.Receive2(buffer, sizeof(buffer));

    if (length < YIS_OUTPUT_MIN_BYTES) return imu_msg;

    // 第一次接收到数据，重置偏航角
    if (first_)
    {
        first_ = false;
        ResetImuYaw();
    }

    // 数据解析
    memcpy(buffer_copy, buffer, length);

    AnalysisData(buffer_copy, length);

    // 发布imu数据

    GetImuData(imu_msg);

    return imu_msg;
}

void ImuYIS320::AnalysisData(unsigned char *data, int length)
{
    unsigned short payload_len = 0;
    unsigned short check_sum = 0;
    unsigned short pos = 0;

    DataHeader *header = nullptr;
    PayloadData *payload = nullptr;

    if (data == NULL || length <= 0)
    {
        std::cout << "imu the param of AnalysisDatat error!" << std::endl;
        return;
    }

    if (length < YIS_OUTPUT_MIN_BYTES)
    {
        std::cout << "imu data length error!" << std::endl;
        return;
    }

    // 判断帧头
    if (data[0] == PROTOCOL_FIRST_BYTE && data[1] == PROTOCOL_SECOND_BYTE)
    {
        header = (DataHeader *)data;
        payload_len = header->len;

        if (payload_len + YIS_OUTPUT_MIN_BYTES > length)
        {
            std::cout << "imu data length error!" << std::endl;
            return;
        }

        // 检查校验 3 = tid(2B) + len(1B)
        CheckSum(data + 2, payload_len + 3, &check_sum);

        if (check_sum != *((unsigned short *)(data + payload_len + 2 + 3)))
        {
            std::cout << "imu crc16 check sum failed!" << std::endl;
            return;
        }

        // 解析数据
        pos = 5;

        while (payload_len > 0)
        {
            payload = (PayloadData *)(data + pos);
            int ret =
                CheckDataLength(*payload, (unsigned char *)payload + sizeof(PayloadData), &info_);
            if (ret == 0)
            {
                pos += payload->data_len + sizeof(PayloadData);
                payload_len -= payload->data_len + sizeof(PayloadData);
            }
            else
            {
                pos++;
                payload_len--;
            }
        }

        // printf("pitch: %f, roll: %f, yaw: %f , quaternion: %f, %f, %f, %f\n",
        //     info_->attitude.pitch, info_->attitude.roll, info_->attitude.yaw,
        // 	info_->attitude.quaternion_data0,info_->attitude.quaternion_data1,
        // 	info_->attitude.quaternion_data2,info_->attitude.quaternion_data3);
    }
    else
        return;
}

int ImuYIS320::CheckSum(unsigned char *data, unsigned short len, unsigned short *checksum)
{
    unsigned char check_a = 0; /*数据类型为char，在不为char的时候，某些情况下crc计算时错误的*/
    unsigned char check_b = 0;
    unsigned short i;

    if (NULL == data || 0 == len || NULL == checksum)
    {
        return -1;
    }

    for (i = 0; i < len; i++)
    {
        check_a += data[i];
        check_b += check_a;
    }

    *checksum = ((unsigned short)(check_b << 8) | check_a);

    return 0;
}

short ImuYIS320::GetSignedShort(unsigned char *data)
{
    short temp = 0;

    temp = (short)((data[1] << 8) | data[0]);

    return temp;
}

int ImuYIS320::GetSignedInt(unsigned char *data)
{
    int temp = 0;

    temp = (int)((data[3] << 24) | (data[2] << 16) | (data[1] << 8) | data[0]);

    return temp;
}

int ImuYIS320::CheckDataLength(PayloadData header, unsigned char *data, ProtocolInfo *info)
{
    unsigned char ret = 0xff;

    if (NULL == data || (unsigned char)0 == header.data_len || NULL == info)
    {
        return -1;
    }

    switch (header.data_id)
    {
        case SENSOR_TEMP_ID:
        {
            if (SENSOR_TEMP_DATA_LEN == header.data_len)
            {
                ret = 0;
                info->sensor_temp = GetSignedShort(data) * SENSOR_TEMP_DATA_FACTOR;
            }
            else
            {
                ret = -2;
            }
        }
        break;

        case ACCEL_ID:
        {
            if (ACCEL_DATA_LEN == header.data_len)
            {
                ret = 0;
                info->accel.x = GetSignedInt(data) * NOT_MAG_DATA_FACTOR;
                info->accel.y = GetSignedInt(data + SINGLE_DATA_BYTES) * NOT_MAG_DATA_FACTOR;
                info->accel.z = GetSignedInt(data + SINGLE_DATA_BYTES * 2) * NOT_MAG_DATA_FACTOR;
            }
            else
            {
                ret = -2;
            }
        }
        break;

        case ANGLE_ID:
        {
            if (ANGLE_DATA_LEN == header.data_len)
            {
                ret = 0;
                info->angle_rate.x = GetSignedInt(data) * NOT_MAG_DATA_FACTOR;
                info->angle_rate.y = GetSignedInt(data + SINGLE_DATA_BYTES) * NOT_MAG_DATA_FACTOR;
                info->angle_rate.z =
                    GetSignedInt(data + ANGLE_DATA_LEN - SINGLE_DATA_BYTES) * NOT_MAG_DATA_FACTOR;
            }
            else
            {
                ret = -2;
            }
        }
        break;

        case MAGNETIC_ID:
        {
            if (MAGNETIC_DATA_LEN == header.data_len)
            {
                ret = 0;
                info->norm_mag.x = GetSignedInt(data) * NOT_MAG_DATA_FACTOR;
                info->norm_mag.y = GetSignedInt(data + SINGLE_DATA_BYTES) * NOT_MAG_DATA_FACTOR;
                info->norm_mag.z = GetSignedInt(data + SINGLE_DATA_BYTES * 2) * NOT_MAG_DATA_FACTOR;
            }
            else
            {
                ret = -2;
            }
        }
        break;

        case RAW_MAGNETIC_ID:
        {
            if (MAGNETIC_RAW_DATA_LEN == header.data_len)
            {
                ret = 0;
                info->raw_mag.x = GetSignedInt(data) * MAG_RAW_DATA_FACTOR;
                info->raw_mag.y = GetSignedInt(data + SINGLE_DATA_BYTES) * MAG_RAW_DATA_FACTOR;
                info->raw_mag.z = GetSignedInt(data + SINGLE_DATA_BYTES * 2) * MAG_RAW_DATA_FACTOR;
            }
            else
            {
                ret = -2;
            }
        }
        break;

        case EULER_ID:
        {
            if (EULER_DATA_LEN == header.data_len)
            {
                ret = 0;
                info->attitude.pitch = GetSignedInt(data) * NOT_MAG_DATA_FACTOR;
                info->attitude.roll = GetSignedInt(data + SINGLE_DATA_BYTES) * NOT_MAG_DATA_FACTOR;
                info->attitude.yaw =
                    GetSignedInt(data + SINGLE_DATA_BYTES * 2) * NOT_MAG_DATA_FACTOR;
            }
            else
            {
                ret = -2;
            }
        }
        break;

        case QUATERNION_ID:
        {
            if (QUATERNION_DATA_LEN == header.data_len)
            {
                ret = 0;
                info->attitude.quaternion_data0 = GetSignedInt(data) * NOT_MAG_DATA_FACTOR;
                info->attitude.quaternion_data1 =
                    GetSignedInt(data + SINGLE_DATA_BYTES) * NOT_MAG_DATA_FACTOR;
                info->attitude.quaternion_data2 =
                    GetSignedInt(data + SINGLE_DATA_BYTES * 2) * NOT_MAG_DATA_FACTOR;
                info->attitude.quaternion_data3 =
                    GetSignedInt(data + SINGLE_DATA_BYTES * 3) * NOT_MAG_DATA_FACTOR;
            }
            else
            {
                ret = -2;
            }
        }
        break;

        case LOCATION_ID:
        {
            if (LOCATION_DATA_LEN == header.data_len)
            {
                ret = 0;
                info->location.latitude = GetSignedInt(data) * LONG_LAT_DATA_FACTOR;
                info->location.longtidue =
                    GetSignedInt(data + SINGLE_DATA_BYTES) * LONG_LAT_DATA_FACTOR;
                info->location.altidue =
                    GetSignedInt(data + SINGLE_DATA_BYTES * 2) * ALT_DATA_FACTOR;
            }
            else
            {
                ret = -2;
            }
        }
        break;

        case HRES_LOCATION_ID:
        {
            if (HRES_LOCATION_DATA_LEN == header.data_len)
            {
                ret = 0;
                info->location.latitude = *((long long int *)data) * HRES_LONG_LAT_DATA_FACTOR;
                info->location.longtidue =
                    *((long long int *)(data + SINGLE_DATA_BYTES * 2)) * HRES_LONG_LAT_DATA_FACTOR;
                info->location.altidue =
                    GetSignedInt(data + HRES_LOCATION_DATA_LEN - SINGLE_DATA_BYTES)
                    * ALT_DATA_FACTOR;
            }
            else
            {
                ret = -2;
            }
        }
        break;

        case SPEED_ID:
        {
            if (SPEED_DATA_LEN == header.data_len)
            {
                ret = 0;
                info->vel.vel_n = GetSignedInt(data) * SPEED_DATA_FACTOR;
                info->vel.vel_e = GetSignedInt(data + SINGLE_DATA_BYTES) * SPEED_DATA_FACTOR;
                info->vel.vel_d = GetSignedInt(data + SINGLE_DATA_BYTES * 2) * SPEED_DATA_FACTOR;
            }
            else
            {
                ret = -2;
            }
        }
        break;

        case UTC_ID:
        {
            if (UTC_DATA_LEN == header.data_len)
            {
                ret = 0;
                memcpy((unsigned char *)&info->utc.msecond, data, sizeof(UtcTime));
            }
            else
            {
                ret = -2;
            }
        }
        break;

        case NAV_STATUS_ID:
        {
            if (NAV_STATUS_DATA_LEN == header.data_len)
            {
                ret = 0;
                info->status = *((NavStatus *)data);
            }
            else
            {
                ret = -2;
            }
        }
        break;

        case SAMPLE_TIMESTAMP_ID:
        {
            if (SAMPLE_TIMESTAMP_DATA_LEN == header.data_len)
            {
                ret = 0;
                info->sample_timestamp = *((unsigned int *)data);
            }
            else
            {
                ret = -2;
            }
        }
        break;

        case DATA_READY_TIMESTAMP_ID:
        {
            if (DATA_READY_TIMESTAMP_DATA_LEN == header.data_len)
            {
                ret = 0;
                info->data_ready_timestamp = *((unsigned int *)data);
            }
            else
            {
                ret = -2;
            }
        }
        break;

        default: break;
    }

    return ret;
}

void ImuYIS320::ResetImuYaw()
{
    char data_buffer[8];
    data_buffer[0] = 0x59;
    data_buffer[1] = 0x53;
    data_buffer[2] = 0x01;
    data_buffer[3] = 0x09;
    data_buffer[4] = 0x00;
    data_buffer[5] = 0x02;
    data_buffer[6] = 0x0C;
    data_buffer[7] = 0x21;

    int result = serial_.Send2(data_buffer, sizeof(data_buffer));
    if (result < 0)
    {
        std::cerr << "reset imu error code = " << errno << std::endl;
    }
}

void ImuYIS320::GetImuData(sensor_msgs::msg::Imu &imu_msg)
{
    // sensor_msgs::Imu imu_data;
    imu_msg.header.frame_id = "imu_link";
    imu_msg.header.stamp = rclcpp::Clock().now();
    imu_msg.linear_acceleration.x = info_.accel.x;
    imu_msg.linear_acceleration.y = info_.accel.y;
    imu_msg.linear_acceleration.z = info_.accel.z;
    imu_msg.angular_velocity.x = info_.angle_rate.x * M_PI / 180.0;
    imu_msg.angular_velocity.y = info_.angle_rate.y * M_PI / 180.0;
    imu_msg.angular_velocity.z = info_.angle_rate.z * M_PI / 180.0;
    imu_msg.orientation.w = info_.attitude.quaternion_data0;
    imu_msg.orientation.x = info_.attitude.quaternion_data1;
    imu_msg.orientation.y = info_.attitude.quaternion_data2;
    imu_msg.orientation.z = info_.attitude.quaternion_data3;

    // imu_pub_.publish(imu_data);
}