/*
 * @Author: vincent vincent_xjw@163.com
 * @Date: 2024-12-28 21:52:35
 * @LastEditors: vincent vincent_xjw@163.com
 * @LastEditTime: 2025-01-16 18:35:05
 * @FilePath: /UAVtoController/src/Protocol/CtrlCenter.cpp
 * @Description: 
 */

#include <stdint.h>
#include <memory>
#include <string.h>
#include "CtrlCenter.hpp"

#ifdef __cplusplus
extern "C" {
#endif

void uint64ToBeBytes(uint64_t number, uint8_t* bytes, int* index)
{
    // increment byte pointer for starting point
    bytes += (*index) + 7;

    *(bytes--) = (uint8_t)(number);
    number = number >> 8;
    *(bytes--) = (uint8_t)(number);
    number = number >> 8;
    *(bytes--) = (uint8_t)(number);
    number = number >> 8;
    *(bytes--) = (uint8_t)(number);
    number = number >> 8;
    *(bytes--) = (uint8_t)(number);
    number = number >> 8;
    *(bytes--) = (uint8_t)(number);
    number = number >> 8;
    *(bytes--) = (uint8_t)(number);
    number = number >> 8;
    *(bytes--) = (uint8_t)(number);

    (*index) += 8;
}

void uint64ToLeBytes(uint64_t number, uint8_t* bytes, int* index)
{
    // increment byte pointer for starting point
    bytes += (*index);

    *(bytes++) = (uint8_t)(number);
    number = number >> 8;
    *(bytes++) = (uint8_t)(number);
    number = number >> 8;
    *(bytes++) = (uint8_t)(number);
    number = number >> 8;
    *(bytes++) = (uint8_t)(number);
    number = number >> 8;
    *(bytes++) = (uint8_t)(number);
    number = number >> 8;
    *(bytes++) = (uint8_t)(number);
    number = number >> 8;
    *(bytes++) = (uint8_t)(number);
    number = number >> 8;
    *bytes = (uint8_t)(number);

    (*index) += 8;
}

void double64ToBeBytes(double number, uint8_t* bytes, int* index)
{
    union
    {
        double doubleValue;
        uint64_t integerValue;
    }field;

    field.doubleValue = number;

    uint64ToBeBytes(field.integerValue, bytes, index);
}


void double64ToLeBytes(double number, uint8_t* bytes, int* index)
{
    union
    {
        double doubleValue;
        uint64_t integerValue;
    }field;

    field.doubleValue = number;

    uint64ToLeBytes(field.integerValue, bytes, index);
}

void uint16ToBeBytes(uint16_t number, uint8_t* bytes, int* index)
{
    // increment byte pointer for starting point
    bytes += (*index) + 1;

    *(bytes--) = (uint8_t)(number);
    number = number >> 8;
    *bytes = (uint8_t)(number);

    (*index) += 2;
}

void uint16ToLeBytes(uint16_t number, uint8_t* bytes, int* index)
{
    // increment byte pointer for starting point
    bytes += (*index);

    *(bytes++) = (uint8_t)(number);
    number = number >> 8;
    *bytes = (uint8_t)(number);

    (*index) += 2;
}

void int16ToBeBytes(int16_t number, uint8_t* bytes, int* index)
{
    // increment byte pointer for starting point
    bytes += (*index) + 1;

    *(bytes--) = (uint8_t)(number);
    number = number >> 8;
    *bytes = (uint8_t)(number);

    (*index) += 2;
}

void int16ToLeBytes(int16_t number, uint8_t* bytes, int* index)
{
    // increment byte pointer for starting point
    bytes += (*index);

    *(bytes++) = (uint8_t)(number);
    number = number >> 8;
    *bytes = (uint8_t)(number);

    (*index) += 2;
}


double double64FromBeBytes(const uint8_t* bytes, int* index)
{
    union
    {
        double doubleValue;
        uint64_t integerValue;
    }field;

    field.integerValue = uint64FromBeBytes(bytes, index);

    return field.doubleValue;
}

double double64FromLeBytes(const uint8_t* bytes, int* index)
{
    union
    {
        double doubleValue;
        uint64_t integerValue;
    }field;

    field.integerValue = uint64FromLeBytes(bytes, index);

    return field.doubleValue;
}

uint64_t uint64FromBeBytes(const uint8_t* bytes, int* index)
{
    uint64_t number;

    // increment byte pointer for starting point
    bytes += *index;

    number = *(bytes++);
    number = (number << 8) | *(bytes++);
    number = (number << 8) | *(bytes++);
    number = (number << 8) | *(bytes++);
    number = (number << 8) | *(bytes++);
    number = (number << 8) | *(bytes++);
    number = (number << 8) | *(bytes++);
    number = (number << 8) | *bytes;

    (*index) += 8;

    return number;
}

uint64_t uint64FromLeBytes(const uint8_t* bytes, int* index)
{
    uint64_t number;

    // increment byte pointer for starting point
    bytes += (*index) + 7;

    number = *(bytes--);
    number = (number << 8) | *(bytes--);
    number = (number << 8) | *(bytes--);
    number = (number << 8) | *(bytes--);
    number = (number << 8) | *(bytes--);
    number = (number << 8) | *(bytes--);
    number = (number << 8) | *(bytes--);
    number = (number << 8) | *bytes;

    (*index) += 8;

    return number;
}

uint16_t uint16FromBeBytes(const uint8_t* bytes, int* index)
{
    uint16_t number;

    // increment byte pointer for starting point
    bytes += *index;

    number = *(bytes++);
    number = (number << 8) | *bytes;

    (*index) += 2;

    return number;
}


uint16_t uint16FromLeBytes(const uint8_t* bytes, int* index)
{
    uint16_t number;

    // increment byte pointer for starting point
    bytes += (*index) + 1;

    number = *(bytes--);
    number = (number << 8) | *bytes;

    (*index) += 2;

    return number;
}

int16_t int16FromBeBytes(const uint8_t* bytes, int* index)
{
    int16_t number;

    // increment byte pointer for starting point
    bytes += *index;

    number = *(bytes++);
    number = (number << 8) | *bytes;

    (*index) += 2;

    return number;
}

int16_t int16FromLeBytes(const uint8_t* bytes, int* index)
{
    int16_t number;

    // increment byte pointer for starting point
    bytes += (*index) + 1;

    number = *(bytes--);
    number = (number << 8) | *bytes;

    (*index) += 2;

    return number;
}

void ctrl_center_msg_position_pack(double longitude, double latitude, double altitude, ctrl_center_message_t &message)
{
    message.header = 0xAFAF;
    message.lenght = sizeof(ctrl_center_position_t) + 8;
    message.type = CTRL_CENTER_MSG_TYPE_POSITION;
    int index = 0;
#ifdef THIS_LITTLE_ENDIAN
    double64ToLeBytes(longitude, message.payload, &index);
    double64ToLeBytes(latitude, message.payload, &index);
    double64ToLeBytes(altitude, message.payload, &index);
#else
    double64ToBeBytes(longitude, message.payload, &index);
    double64ToBeBytes(latitude, message.payload, &index);
    double64ToBeBytes(altitude, message.payload, &index);
#endif
    message.checkSum = 0;
    message.tail = 0x3F19;
}

void ctrl_center_msg_sys_status_pack(ctrl_center_sys_status_t sysStatus, ctrl_center_message_t &message)
{
    message.header = 0xAFAF;
    message.lenght = sizeof(ctrl_center_sys_status_t) + 8;
    message.type = CTRL_CENTER_MSG_TYPE_SYS_STATUS;
    int index = 0;
#ifdef THIS_LITTLE_ENDIAN
    message.payload[index++] = sysStatus.rtkFixType;
    double64ToLeBytes(sysStatus.position.longitude, message.payload, &index);
    double64ToLeBytes(sysStatus.position.latitude, message.payload, &index);
    double64ToLeBytes(sysStatus.position.altitude, message.payload, &index);
    double64ToLeBytes(sysStatus.relativeAltitude, message.payload, &index);
    int16ToLeBytes(sysStatus.velocity.vx, message.payload, &index);
    int16ToLeBytes(sysStatus.velocity.vy, message.payload, &index);
    int16ToLeBytes(sysStatus.velocity.vz, message.payload, &index);
    int16ToLeBytes(sysStatus.accSpeed.accx, message.payload, &index);
    int16ToLeBytes(sysStatus.accSpeed.accy, message.payload, &index);
    int16ToLeBytes(sysStatus.accSpeed.accz, message.payload, &index);
    int16ToLeBytes(sysStatus.roll, message.payload, &index);
    int16ToLeBytes(sysStatus.pitch, message.payload, &index);
    uint16ToLeBytes(sysStatus.yaw, message.payload, &index);
    int16ToLeBytes(sysStatus.angleVelocity.vx, message.payload, &index);
    int16ToLeBytes(sysStatus.angleVelocity.vy, message.payload, &index);
    int16ToLeBytes(sysStatus.angleVelocity.vz, message.payload, &index);
    message.payload[index++] = sysStatus.allSensorsHealthy;
    message.payload[index++] = sysStatus.commandAck;
#else
    message.payload[index++] = sysStatus.rtkFixType;
    double64ToBeBytes(sysStatus.position.longitude, message.payload, &index);
    double64ToBeBytes(sysStatus.position.latitude, message.payload, &index);
    double64ToBeBytes(sysStatus.position.altitude, message.payload, &index);
    double64ToBeBytes(sysStatus.relativeAltitude, message.payload, &index);
    int16ToBeBytes(sysStatus.velocity.vx, message.payload, &index);
    int16ToBeBytes(sysStatus.velocity.vy, message.payload, &index);
    int16ToBeBytes(sysStatus.velocity.vz, message.payload, &index);
    int16ToBeBytes(sysStatus.accSpeed.accx, message.payload, &index);
    int16ToBeBytes(sysStatus.accSpeed.accy, message.payload, &index);
    int16ToBeBytes(sysStatus.accSpeed.accz, message.payload, &index);
    int16ToBeBytes(sysStatus.roll, message.payload, &index);
    int16ToBeBytes(sysStatus.pitch, message.payload, &index);
    uint16BoLeBytes(sysStatus.yaw, message.payload, &index);
    int16ToBeBytes(sysStatus.angleVelocity.vx, message.payload, &index);
    int16ToBeBytes(sysStatus.angleVelocity.vy, message.payload, &index);
    int16ToBeBytes(sysStatus.angleVelocity.vz, message.payload, &index);
    message.payload[index++] = sysStatus.allSensorsHealthy;
    message.payload[index++] = sysStatus.commandAck;
#endif
    message.checkSum = 0;
    message.tail = 0x3F19;
}

void ctrl_center_msg_power_pack(uint16_t power, ctrl_center_message_t &message)
{
    message.header = 0xAFAF;
    message.lenght = sizeof(ctrl_center_power_t) + 8;
    message.type = CTRL_CENTER_MSG_TYPE_POWER;
    int index = 0;
#ifdef THIS_LITTLE_ENDIAN
    uint16ToLeBytes(power, message.payload, &index);
#else
    uint16ToBeBytes(power, message.payload, &index);
#endif
    message.checkSum = 0;
    message.tail = 0x3F19;
}

int ctrl_center_msg_to_send_buffer(uint8_t *buf, const ctrl_center_message_t &message)
{
    int index = 0;
    buf[index++] = message.header & 0xFF;
    buf[index++] = (message.header >> 8) & 0xFF;
#ifdef THIS_LITTLE_ENDIAN
    uint16ToLeBytes(message.lenght, buf, &index);
#else
    uint16ToBeBytes(message.lenght, buf, &index);
#endif
    buf[index++] = message.type;
    memcpy(buf+index, message.payload, message.lenght - 8); 
    index += message.lenght - 8;
    buf[index++] = checkSum(buf + 2, index-2);
    buf[index++] = message.tail & 0xFF;
    buf[index++] = (message.tail >> 8) & 0xFF;
    return index;
}

// TODO:在这里补充其他打包函数


// TODO： 在这下面添加解析函数
bool ctrl_center_parse_char(uint8_t c, ctrl_center_message_t *msg, ctrl_center_status_t &status)
{
    switch (status.state) {
    case CTRL_CENTER_PARSE_STATE_UNINIT:
    case CTRL_CENTER_PARSE_STATE_IDLE:
    {
        if (c == 0xAF) {
            status.state = CTRL_CENTER_PARSE_STATE_GOT_STX1;
            msg->lenght = 0;
            msg->header = c;
        }
    }
        break;
    case CTRL_CENTER_PARSE_STATE_GOT_STX1:
    {
        if (c == 0xAF) {
            status.state = CTRL_CENTER_PARSE_STATE_GOT_STX2;
            msg->lenght = 0;
            msg->header |= (c << 8) & 0xFF00;
        } else {
            status.state = CTRL_CENTER_PARSE_STATE_IDLE;
        }
    }
        break;
    case CTRL_CENTER_PARSE_STATE_GOT_STX2:
    {
        status.state = CTRL_CENTER_PARSE_STATE_GOT_LENGTH1;
        status.pack_index = 0;
        status.pack_len = 0;
        msg->lenght = c;
    }
        break;
    case CTRL_CENTER_PARSE_STATE_GOT_LENGTH1:
    {
        status.state = CTRL_CENTER_PARSE_STATE_GOT_LENGTH2;
        msg->lenght |= (c << 8) & 0xFF00;
        status.pack_index = 0;
        status.pack_len = msg->lenght - 8;
    }
        break;
    case CTRL_CENTER_PARSE_STATE_GOT_LENGTH2:
    {
        status.state = CTRL_CENTER_PARSE_STATE_GOT_MSGTYPE;
        msg->type = c;
    } break;
    case CTRL_CENTER_PARSE_STATE_GOT_MSGTYPE:
    {
        if (status.pack_index < status.pack_len) {
            msg->payload[status.pack_index++] = c;
        }
        if (status.pack_index == status.pack_len) {
            status.state = CTRL_CENTER_PARSE_STATE_GOT_PAYLOAD;
        }
    } break;
    case CTRL_CENTER_PARSE_STATE_GOT_PAYLOAD:
    {
        status.state = CTRL_CENTER_PARSE_STATE_GOT_CRC;
        msg->checkSum = c;
        uint8_t buf[261] = {0};
        int len = status.pack_len + 3;
        buf[0] = msg->lenght & 0xFF;
        buf[1] = (msg->lenght >> 8) & 0xFF;
        buf[2] = msg->type;
        memcpy(buf+3, msg->payload, len);
        uint8_t crc = checkSum(buf, len);
        bool ok = (crc == msg->checkSum);
        if (!ok) {
            status.pack_len = 0;
            status.pack_index = 0;
            status.state = CTRL_CENTER_PARSE_STATE_IDLE;
        }
    } break;
    case CTRL_CENTER_PARSE_STATE_GOT_CRC:
    {
        status.state = CTRL_CENTER_PARSE_STATE_GOT_TAIL1;
        msg->tail = c;
        if (c != 0x19) {
            status.pack_len = 0;
            status.pack_index = 0;
            status.state = CTRL_CENTER_PARSE_STATE_IDLE;
        }
    } break;
    case CTRL_CENTER_PARSE_STATE_GOT_TAIL1:
    {
        status.state = CTRL_CENTER_PARSE_STATE_GOT_TAIL1;
        msg->tail |= (c << 8) & 0xFF00;
        if (c == 0x3F) {
            status.pack_len = 0;
            status.pack_index = 0;
            return true;
        }
        
        status.state = CTRL_CENTER_PARSE_STATE_IDLE;
    } break;
    default: break;
    }

    return false;
}

void ctrl_center_msg_position_decode(const ctrl_center_message_t* msg, ctrl_center_position_t* position)
{
    if (msg && position && msg->type == CTRL_CENTER_MSG_TYPE_POSITION) {
        int index = 0;
        const uint8_t *buffer = msg->payload;
#ifdef THIS_LITTLE_ENDIAN
        position->longitude = double64FromLeBytes(buffer, &index);
        position->latitude = double64FromLeBytes(buffer, &index);
        position->altitude = double64FromLeBytes(buffer, &index);
#else
        position->longitude = double64FromBeBytes(buffer, &index);
        position->latitude = double64FromBeBytes(buffer, &index);
        position->altitude = double64FromBeBytes(buffer, &index);
#endif
    }
}

void ctrl_center_msg_sys_status_decode(const ctrl_center_message_t* msg, ctrl_center_sys_status_t* sys_status)
{
    if (msg && sys_status && msg->type == CTRL_CENTER_MSG_TYPE_SYS_STATUS) {
        int index = 0;
        const uint8_t *buffer = msg->payload;
#ifdef THIS_LITTLE_ENDIAN
        sys_status->rtkFixType = buffer[index++];
        sys_status->position.longitude = double64FromLeBytes(buffer, &index);
        sys_status->position.latitude = double64FromLeBytes(buffer, &index);
        sys_status->position.altitude = double64FromLeBytes(buffer, &index);
        sys_status->relativeAltitude = double64FromLeBytes(buffer, &index);
        sys_status->velocity.vx = int16FromLeBytes(buffer, &index);
        sys_status->velocity.vy = int16FromLeBytes(buffer, &index);
        sys_status->velocity.vz = int16FromLeBytes(buffer, &index);
        sys_status->accSpeed.accx = int16FromLeBytes(buffer, &index);
        sys_status->accSpeed.accy = int16FromLeBytes(buffer, &index);
        sys_status->accSpeed.accz = int16FromLeBytes(buffer, &index);
        sys_status->roll = int16FromLeBytes(buffer, &index);
        sys_status->pitch = int16FromLeBytes(buffer, &index);
        sys_status->yaw = uint16FromLeBytes(buffer, &index);
        sys_status->angleVelocity.vx = int16FromLeBytes(buffer, &index);
        sys_status->angleVelocity.vy = int16FromLeBytes(buffer, &index);
        sys_status->angleVelocity.vz = int16FromLeBytes(buffer, &index);
        sys_status->allSensorsHealthy = buffer[index++];
        sys_status->commandAck = buffer[index++];
        memcpy(sys_status->reserved, buffer, 32);
        index += 32;
#else
        sys_status->rtkFixType = buffer[index++];
        sys_status->position.longitude = double64FromBeBytes(buffer, &index);
        sys_status->position.latitude = double64FromBeBytes(buffer, &index);
        sys_status->position.altitude = double64FromBeBytes(buffer, &index);
        sys_status->relativeAltitude = double64FromBeBytes(buffer, &index);
        sys_status->velocity.vx = int16FromBeBytes(buffer, &index);
        sys_status->velocity.vy = int16FromBeBytes(buffer, &index);
        sys_status->velocity.vz = int16FromBeBytes(buffer, &index);
        sys_status->accSpeed.accx = int16FromBeBytes(buffer, &index);
        sys_status->accSpeed.accy = int16FromBeBytes(buffer, &index);
        sys_status->accSpeed.accz = int16FromBeBytes(buffer, &index);
        sys_status->roll = int16FromBeBytes(buffer, &index);
        sys_status->pitch = int16FromBeBytes(buffer, &index);
        sys_status->yaw = uint16FromBeBytes(buffer, &index);
        sys_status->angleVelocity.vx = int16FromBeBytes(buffer, &index);
        sys_status->angleVelocity.vy = int16FromBeBytes(buffer, &index);
        sys_status->angleVelocity.vz = int16FromBeBytes(buffer, &index);
        sys_status->allSensorsHealthy = buffer[index++];
        sys_status->commandAck = buffer[index++];
        memcpy(sys_status->reserved, buffer, 32);
        index += 32;
#endif
    }
}

void ctrl_center_msg_power_decode(const ctrl_center_message_t* msg, ctrl_center_power_t* power)
{
    if (msg && power && msg->type == CTRL_CENTER_MSG_TYPE_POWER) {
        int index = 0;
#ifdef THIS_LITTLE_ENDIAN
        power->power = uint16FromLeBytes(msg->payload, &index);
#else
        power->power = uint16FromBeBytes(msg->payload, &index);
#endif
    }
}

#ifdef THIS_LITTLE_ENDIAN
void ctrl_center_msg_command_long_decode(const ctrl_center_message_t* msg, ctrl_center_command_long_t* command_long)
{
    if (msg && command_long && msg->type == CTRL_CENTER_MSG_TYPE_COMMAND) {
        int index = 0;
        const uint8_t *buffer = msg->payload;
        command_long->sequenceNumber = uint16FromLeBytes(buffer, &index);
        command_long->timestamp = uint64FromLeBytes(buffer, &index);
        command_long->mode = buffer[index++];
        command_long->trackIsValid = buffer[index++];
        
        command_long->position_1.longitude = double64FromLeBytes(buffer, &index);
        command_long->position_1.latitude = double64FromLeBytes(buffer, &index);
        command_long->position_1.altitude = double64FromLeBytes(buffer, &index);
        command_long->velocityNED_1.vx = int16FromLeBytes(buffer, &index);
        command_long->velocityNED_1.vy = int16FromLeBytes(buffer, &index);
        command_long->velocityNED_1.vz = int16FromLeBytes(buffer, &index);
        command_long->accSpeed_1.accx = int16FromLeBytes(buffer, &index);
        command_long->accSpeed_1.accy = int16FromLeBytes(buffer, &index);
        command_long->accSpeed_1.accz = int16FromLeBytes(buffer, &index);
        command_long->yaw_1 = uint16FromLeBytes(buffer, &index);
        command_long->yawAngleSpeed_1 = int16FromLeBytes(buffer, &index);
        command_long->trackId_1 = buffer[index++];

        command_long->position_2.longitude = double64FromLeBytes(buffer, &index);
        command_long->position_2.latitude = double64FromLeBytes(buffer, &index);
        command_long->position_2.altitude = double64FromLeBytes(buffer, &index);
        command_long->velocityNED_2.vx = int16FromLeBytes(buffer, &index);
        command_long->velocityNED_2.vy = int16FromLeBytes(buffer, &index);
        command_long->velocityNED_2.vz = int16FromLeBytes(buffer, &index);
        command_long->accSpeed_2.accx = int16FromLeBytes(buffer, &index);
        command_long->accSpeed_2.accy = int16FromLeBytes(buffer, &index);
        command_long->accSpeed_2.accz = int16FromLeBytes(buffer, &index);
        command_long->yaw_2 = uint16FromLeBytes(buffer, &index);
        command_long->yawAngleSpeed_2 = int16FromLeBytes(buffer, &index);
        command_long->trackId_2 = buffer[index++];

        command_long->position_3.longitude = double64FromLeBytes(buffer, &index);
        command_long->position_3.latitude = double64FromLeBytes(buffer, &index);
        command_long->position_3.altitude = double64FromLeBytes(buffer, &index);
        command_long->velocityNED_3.vx = int16FromLeBytes(buffer, &index);
        command_long->velocityNED_3.vy = int16FromLeBytes(buffer, &index);
        command_long->velocityNED_3.vz = int16FromLeBytes(buffer, &index);
        command_long->accSpeed_3.accx = int16FromLeBytes(buffer, &index);
        command_long->accSpeed_3.accy = int16FromLeBytes(buffer, &index);
        command_long->accSpeed_3.accz = int16FromLeBytes(buffer, &index);
        command_long->yaw_3 = uint16FromLeBytes(buffer, &index);
        command_long->yawAngleSpeed_3 = int16FromLeBytes(buffer, &index);
        command_long->trackId_3 = buffer[index++];

        command_long->height = double64FromLeBytes(buffer, &index);
        command_long->frontBackVelocity = int16FromLeBytes(buffer, &index);
        command_long->leftRightVelocity = int16FromLeBytes(buffer, &index);
        command_long->upDownVelocity = int16FromLeBytes(buffer, &index);
        command_long->yawAngleSpeed_leftRight = int16FromLeBytes(buffer, &index);
        command_long->emergencyCommand = buffer[index++];
    }
}
#else
void ctrl_center_msg_command_long_decode(const ctrl_center_message_t* msg, ctrl_center_command_long_t* command_long)
{
    if (msg && command_long && msg->type == CTRL_CENTER_MSG_TYPE_COMMAND) {
        int index = 0;
        const uint8_t *buffer = msg->payload;
        command_long->sequenceNumber = uint16FromBeBytes(buffer, &index);
        command_long->timestamp = uint64FromBeBytes(buffer, &index);
        command_long->mode = buffer[index++];
        command_long->trackIsValid = buffer[index++];
        
        command_long->position_1.longitude = double64FromBeBytes(buffer, &index);
        command_long->position_1.latitude = double64FromBeBytes(buffer, &index);
        command_long->position_1.altitude = double64FromBeBytes(buffer, &index);
        command_long->velocityNED_1.vx = int16FromBeBytes(buffer, &index);
        command_long->velocityNED_1.vy = int16FromBeBytes(buffer, &index);
        command_long->velocityNED_1.vz = int16FromBeBytes(buffer, &index);
        command_long->accSpeed_1.accx = int16FromBeBytes(buffer, &index);
        command_long->accSpeed_1.accy = int16FromBeBytes(buffer, &index);
        command_long->accSpeed_1.accz = int16FromBeBytes(buffer, &index);
        command_long->yaw_1 = uint16FromBeBytes(buffer, &index);
        command_long->yawAngleSpeed_1 = int16FromBeBytes(buffer, &index);
        command_long->trackId_1 = buffer[index++];

        command_long->position_2.longitude = double64FromBeBytes(buffer, &index);
        command_long->position_2.latitude = double64FromBeBytes(buffer, &index);
        command_long->position_2.altitude = double64FromBeBytes(buffer, &index);
        command_long->velocityNED_2.vx = int16FromBeBytes(buffer, &index);
        command_long->velocityNED_2.vy = int16FromBeBytes(buffer, &index);
        command_long->velocityNED_2.vz = int16FromBeBytes(buffer, &index);
        command_long->accSpeed_2.accx = int16FromBeBytes(buffer, &index);
        command_long->accSpeed_2.accy = int16FromBeBytes(buffer, &index);
        command_long->accSpeed_2.accz = int16FromBeBytes(buffer, &index);
        command_long->yaw_2 = uint16FromBeBytes(buffer, &index);
        command_long->yawAngleSpeed_2 = int16FromBeBytes(buffer, &index);
        command_long->trackId_2 = buffer[index++];

        command_long->position_3.longitude = double64FromBeBytes(buffer, &index);
        command_long->position_3.latitude = double64FromBeBytes(buffer, &index);
        command_long->position_3.altitude = double64FromBeBytes(buffer, &index);
        command_long->velocityNED_3.vx = int16FromBeBytes(buffer, &index);
        command_long->velocityNED_3.vy = int16FromBeBytes(buffer, &index);
        command_long->velocityNED_3.vz = int16FromBeBytes(buffer, &index);
        command_long->accSpeed_3.accx = int16FromBeBytes(buffer, &index);
        command_long->accSpeed_3.accy = int16FromBeBytes(buffer, &index);
        command_long->accSpeed_3.accz = int16FromBeBytes(buffer, &index);
        command_long->yaw_3 = uint16FromBeBytes(buffer, &index);
        command_long->yawAngleSpeed_3 = int16FromBeBytes(buffer, &index);
        command_long->trackId_3 = buffer[index++];

        command_long->height = double64FromBeBytes(buffer, &index);
        command_long->frontBackVelocity = int16FromBeBytes(buffer, &index);
        command_long->leftRightVelocity = int16FromBeBytes(buffer, &index);
        command_long->upDownVelocity = int16FromBeBytes(buffer, &index);
        command_long->yawAngleSpeed_leftRight = int16FromBeBytes(buffer, &index);
        command_long->emergencyCommand = buffer[index++];
    }
}
#endif

uint8_t checkSum(const uint8_t *buf, int length)
{
    uint32_t sum = 0;

    // 遍历每个字节，进行累加
    for (size_t i = 0; i < length; i++) {
        sum += buf[i];
    }

    // 将累加和取模 256，返回校验和
    return (uint8_t)(sum & 0xFF);
}

#ifdef __cplusplus
}
#endif
