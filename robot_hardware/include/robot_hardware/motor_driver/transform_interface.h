#ifndef _TRANSFORM_INTERFACE_H_
#define _TRANSFORM_INTERFACE_H_

#include <vector>
#include <thread>
#include <mutex>

class TransformInterface
{
public:
    TransformInterface() {}
    virtual ~TransformInterface() {}

    /**
     * @brief 开启线程
     */
    virtual void Start() = 0;

    /**
     * @brief 关闭线程
     */
    virtual void Stop() = 0;

    /**
     * @brief 发送数据
     * @param data: 发送的数据指针
     * @param size: 发送的数据大小
     */
    virtual void SetData(unsigned char* data, int size) = 0;

    /**
     * @brief 接收数据
     * @param data: 接收的数据指针
     * @param size: 接收的数据大小
     */
    virtual void GetData(unsigned char* data, int& size) = 0;

private:

};

#endif // _TRANSFORM_INTERFACE_H_