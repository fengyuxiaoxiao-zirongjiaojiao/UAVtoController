/*
 * @Author: vincent vincent_xjw@163.com
 * @Date: 2024-12-28 21:52:35
 * @LastEditors: vincent vincent_xjw@163.com
 * @LastEditTime: 2024-12-29 16:37:24
 * @FilePath: /UAVtoController/src/Protocol/CtrlCenter.hpp
 * @Description: 
 */
#ifndef __CTRL_CENTER_H_
#define __CTRL_CENTER_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define THIS_LITTLE_ENDIAN

void uint64ToBeBytes(uint64_t number, uint8_t* bytes, int* index);
void uint64ToLeBytes(uint64_t number, uint8_t* bytes, int* index);
void double64ToBeBytes(double number, uint8_t* bytes, int* index);
void double64ToLeBytes(double number, uint8_t* bytes, int* index);

double double64FromBeBytes(const uint8_t* bytes, int* index);
double double64FromLeBytes(const uint8_t* bytes, int* index);
uint64_t uint64FromBeBytes(const uint8_t* bytes, int* index);
uint64_t uint64FromLeBytes(const uint8_t* bytes, int* index);

uint16_t uint16FromBeBytes(const uint8_t* bytes, int* index);
uint16_t uint16FromLeBytes(const uint8_t* bytes, int* index);
int16_t int16FromBeBytes(const uint8_t* bytes, int* index);
int16_t int16FromLeBytes(const uint8_t* bytes, int* index);

#define CTRL_CENTER_MSG_PAYLOAD_LEN_MAX   128
#define CTRL_CENTER_MSG_FRAME_LEN_MAX   (CTRL_CENTER_MSG_PAYLOAD_LEN_MAX + 7)

typedef enum _Protocol_type{
    CTRL_CENTER_MSG_TYPE_POSITION = 0x01,  // 位置信息
    CTRL_CENTER_MSG_TYPE_SYS_STATUS = 0x02, // 系统状态
    CTRL_CENTER_MSG_TYPE_BATTERY = 0x03, // 能源
} CTRL_CENTER_MSG_TYPE;

// 帧结构  所有数据采用小端字节序（低地址低字节）
typedef struct _Protocol_body {
    uint16_t header;     // 2字节的帧头，固定为0xAFAF
    uint16_t lenght;     // 整帧长度，帧头到帧尾
    uint8_t type;      // 帧类型
    uint8_t payload[CTRL_CENTER_MSG_PAYLOAD_LEN_MAX];
    uint8_t checkSum;  // 1字节的累加校验和  帧长度~校验和 (不包含校验和)
    uint16_t tail;      // 帧尾 两个字节  固定为0x193F
} ctrl_center_message_t;

/// TODO ：在这下面添加负载结构体
// 位置
typedef struct __ctrl_center_position_t {
    double longitude;
    double latitude;
    double altitude;
} ctrl_center_position_t;
// 速度
typedef struct __ctrl_center_velocity_t {
    int16_t vx;
    int16_t vy;
    int16_t vz;
} ctrl_center_velocity_t;
// 加速度
typedef struct __ctrl_center_accelerated_speed_t {
    int16_t accx;
    int16_t accy;
    int16_t accz;
} ctrl_center_accelerated_speed_t;

// 指控/自主→信息模块→飞控
typedef struct __ctrl_center_command_long_t {
    uint16_t sequenceNumber;    // 时序  从0开始累加
    uint64_t timestamp;         // 时间戳
    uint8_t mode;               // 状态模式
    uint8_t trackIsValid;       // 航迹有效性

    ctrl_center_position_t position_1;          // 位置1
    ctrl_center_velocity_t velocityNED_1;       // 速度1
    ctrl_center_accelerated_speed_t accSpeed_1; // 加速度1
    uint16_t yaw_1;             // 航向角1
    int16_t yawAngleSpeed_1;    // 航向角速度1
    uint8_t trackId_1;          // 轨迹

    ctrl_center_position_t position_2;          // 位置2
    ctrl_center_velocity_t velocityNED_2;       // 速度2
    ctrl_center_accelerated_speed_t accSpeed_2; // 加速度2
    uint16_t yaw_2;                 // 航向角2
    int16_t yawAngleSpeed_2;        // 航向角速度2
    uint8_t trackId_2;              // 轨迹

    ctrl_center_position_t position_3;          // 位置3
    ctrl_center_velocity_t velocityNED_3;       // 速度3
    ctrl_center_accelerated_speed_t accSpeed_3; // 加速度3
    uint16_t yaw_3;             // 航向角3
    int16_t yawAngleSpeed_3;    // 航向角速度3
    uint8_t trackId_3;          // 轨迹

    double height;              // 高度
    int16_t frontBackVelocity;  // 前后移动速度
    int16_t leftRightVelocity;  // 左右移动速度
    int16_t upDownVelocity;     // 上下移动速度
    int16_t yawAngleSpeed_leftRight; // 航向角速度 （左右转速度）

    uint8_t emergencyCommand; // 紧急控制指令

    uint8_t reserved[32]; // 预留
} ctrl_center_command_long_t;

/// TODO : 在这下面添加打包函数

void ctrl_center_msg_position_pack(double longitude, double latitude, double altitude, ctrl_center_message_t &message);
int ctrl_center_msg_to_send_buffer(uint8_t *buf, const ctrl_center_message_t &message);
// TODO:在这里补充其他打包函数

// TODO： 在这下面添加解析函数
typedef enum {
    CTRL_CENTER_PARSE_STATE_UNINIT=0,
    CTRL_CENTER_PARSE_STATE_IDLE,
    CTRL_CENTER_PARSE_STATE_GOT_STX1,
    CTRL_CENTER_PARSE_STATE_GOT_STX2,
    CTRL_CENTER_PARSE_STATE_GOT_LENGTH1,
    CTRL_CENTER_PARSE_STATE_GOT_LENGTH2,
    CTRL_CENTER_PARSE_STATE_GOT_MSGTYPE,
    CTRL_CENTER_PARSE_STATE_GOT_PAYLOAD,
    CTRL_CENTER_PARSE_STATE_GOT_CRC,
    CTRL_CENTER_PARSE_STATE_GOT_TAIL1,
    CTRL_CENTER_PARSE_STATE_GOT_TAIL2,
} ctrl_center_parse_state_t;

typedef struct __ctrl_center_status {
    ctrl_center_parse_state_t state;
    int pack_index;
    int pack_len;
    uint8_t crc_is_ok;
} ctrl_center_status_t;

bool ctrl_center_parse_char(uint8_t c, ctrl_center_message_t *msg, ctrl_center_status_t &status);
void ctrl_center_msg_command_long_decode(const ctrl_center_message_t* msg, ctrl_center_command_long_t* command_long);



uint8_t checkSum(const uint8_t *buf, int length);

#ifdef __cplusplus
}
#endif

#endif // __CTRL_CENTER_H_