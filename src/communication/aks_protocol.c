/**
 * @file aks_protocol.c
 * @brief Communication protocol implementation for AKS
 * @author AKS Development Team
 * @date 2025
 * @version 1.0.0
 */

#include "aks_protocol.h"
#include "aks_types.h"
#include <string.h>

/* Protocol Configuration */
#define AKS_PROTOCOL_START_BYTE     0xAA
#define AKS_PROTOCOL_END_BYTE       0x55
#define AKS_PROTOCOL_ESCAPE_BYTE    0x5A
#define AKS_PROTOCOL_MAX_PAYLOAD    128
#define AKS_PROTOCOL_HEADER_SIZE    6
#define AKS_PROTOCOL_FOOTER_SIZE    3

/* Protocol Message Types */
typedef enum {
    AKS_MSG_TYPE_DATA       = 0x01,
    AKS_MSG_TYPE_COMMAND    = 0x02,
    AKS_MSG_TYPE_RESPONSE   = 0x03,
    AKS_MSG_TYPE_HEARTBEAT  = 0x04,
    AKS_MSG_TYPE_ERROR      = 0x05
} aks_protocol_msg_type_t;

/* Protocol Frame Structure */
typedef struct {
    uint8_t start_byte;        /* 0xAA */
    uint8_t msg_type;          /* Message type */
    uint8_t source_id;         /* Source node ID */
    uint8_t dest_id;           /* Destination node ID */
    uint16_t length;           /* Payload length */
    uint8_t payload[AKS_PROTOCOL_MAX_PAYLOAD]; /* Payload data */
    uint16_t crc;              /* CRC16 checksum */
    uint8_t end_byte;          /* 0x55 */
} __attribute__((packed)) aks_protocol_frame_t;

/* Private variables */
static bool g_protocol_initialized = false;
static uint8_t g_node_id = 0x01;
static uint16_t g_sequence_number = 0;

/* Statistics */
static struct {
    uint32_t frames_sent;
    uint32_t frames_received;
    uint32_t crc_errors;
    uint32_t frame_errors;
    uint32_t timeouts;
} g_stats = {0};

/* Private function prototypes */
static uint16_t aks_protocol_calculate_crc16(const uint8_t* data, uint16_t length);
static aks_result_t aks_protocol_encode_frame(const aks_message_t* message, 
                                              uint8_t* buffer, uint16_t* frame_length);
static aks_result_t aks_protocol_decode_frame(const uint8_t* buffer, uint16_t length, 
                                              aks_message_t* message);
static bool aks_protocol_validate_frame(const aks_protocol_frame_t* frame);

/**
 * @brief Initialize communication protocol
 */
aks_result_t aks_protocol_init(uint8_t node_id)
{
    if (g_protocol_initialized) {
        return AKS_ERROR_BUSY;
    }
    
    g_node_id = node_id;
    g_sequence_number = 0;
    
    /* Clear statistics */
    memset(&g_stats, 0, sizeof(g_stats));
    
    g_protocol_initialized = true;
    
    return AKS_OK;
}

/**
 * @brief Deinitialize communication protocol
 */
aks_result_t aks_protocol_deinit(void)
{
    if (!g_protocol_initialized) {
        return AKS_ERROR_NOT_READY;
    }
    
    g_protocol_initialized = false;
    
    return AKS_OK;
}

/**
 * @brief Encode message into protocol frame
 */
aks_result_t aks_protocol_encode_message(const aks_message_t* message, 
                                         uint8_t* buffer, uint16_t buffer_size, 
                                         uint16_t* frame_length)
{
    if (!g_protocol_initialized) {
        return AKS_ERROR_NOT_READY;
    }
    
    if (message == NULL || buffer == NULL || frame_length == NULL) {
        return AKS_ERROR_INVALID_PARAM;
    }
    
    if (buffer_size < (AKS_PROTOCOL_HEADER_SIZE + message->length + AKS_PROTOCOL_FOOTER_SIZE)) {
        return AKS_ERROR_OVERFLOW;
    }
    
    return aks_protocol_encode_frame(message, buffer, frame_length);
}

/**
 * @brief Decode protocol frame into message
 */
aks_result_t aks_protocol_decode_message(const uint8_t* buffer, uint16_t length, 
                                         aks_message_t* message)
{
    if (!g_protocol_initialized) {
        return AKS_ERROR_NOT_READY;
    }
    
    if (buffer == NULL || message == NULL || length < AKS_PROTOCOL_HEADER_SIZE + AKS_PROTOCOL_FOOTER_SIZE) {
        return AKS_ERROR_INVALID_PARAM;
    }
    
    return aks_protocol_decode_frame(buffer, length, message);
}

/**
 * @brief Send heartbeat message
 */
aks_result_t aks_protocol_send_heartbeat(uint8_t dest_id, uint8_t* buffer, 
                                         uint16_t buffer_size, uint16_t* frame_length)
{
    if (!g_protocol_initialized) {
        return AKS_ERROR_NOT_READY;
    }
    
    if (buffer == NULL || frame_length == NULL) {
        return AKS_ERROR_INVALID_PARAM;
    }
    
    aks_message_t heartbeat = {
        .id = AKS_MSG_TYPE_HEARTBEAT,
        .length = 4,
        .timestamp = 0, /* Would be set by system tick */
        .priority = AKS_PRIORITY_LOW
    };
    
    /* Heartbeat payload: node_id + sequence_number */
    heartbeat.data[0] = g_node_id;
    heartbeat.data[1] = (g_sequence_number >> 8) & 0xFF;
    heartbeat.data[2] = g_sequence_number & 0xFF;
    heartbeat.data[3] = 0; /* Status byte */
    
    g_sequence_number++;
    
    return aks_protocol_encode_message(&heartbeat, buffer, buffer_size, frame_length);
}

/**
 * @brief Send command message
 */
aks_result_t aks_protocol_send_command(uint8_t dest_id, aks_command_t command, 
                                       const uint8_t* params, uint8_t param_length,
                                       uint8_t* buffer, uint16_t buffer_size, 
                                       uint16_t* frame_length)
{
    if (!g_protocol_initialized) {
        return AKS_ERROR_NOT_READY;
    }
    
    if (buffer == NULL || frame_length == NULL) {
        return AKS_ERROR_INVALID_PARAM;
    }
    
    if (param_length > 6) { /* Max 6 bytes for parameters */
        return AKS_ERROR_OVERFLOW;
    }
    
    aks_message_t cmd_msg = {
        .id = AKS_MSG_TYPE_COMMAND,
        .length = 2 + param_length,
        .timestamp = 0,
        .priority = AKS_PRIORITY_HIGH
    };
    
    cmd_msg.data[0] = command;
    cmd_msg.data[1] = param_length;
    
    if (params != NULL && param_length > 0) {
        memcpy(&cmd_msg.data[2], params, param_length);
    }
    
    return aks_protocol_encode_message(&cmd_msg, buffer, buffer_size, frame_length);
}

/**
 * @brief Send response message
 */
aks_result_t aks_protocol_send_response(uint8_t dest_id, uint8_t status, 
                                        const uint8_t* data, uint8_t data_length,
                                        uint8_t* buffer, uint16_t buffer_size, 
                                        uint16_t* frame_length)
{
    if (!g_protocol_initialized) {
        return AKS_ERROR_NOT_READY;
    }
    
    if (buffer == NULL || frame_length == NULL) {
        return AKS_ERROR_INVALID_PARAM;
    }
    
    if (data_length > 6) {
        return AKS_ERROR_OVERFLOW;
    }
    
    aks_message_t resp_msg = {
        .id = AKS_MSG_TYPE_RESPONSE,
        .length = 2 + data_length,
        .timestamp = 0,
        .priority = AKS_PRIORITY_NORMAL
    };
    
    resp_msg.data[0] = status;
    resp_msg.data[1] = data_length;
    
    if (data != NULL && data_length > 0) {
        memcpy(&resp_msg.data[2], data, data_length);
    }
    
    return aks_protocol_encode_message(&resp_msg, buffer, buffer_size, frame_length);
}

/**
 * @brief Get protocol statistics
 */
aks_result_t aks_protocol_get_stats(uint32_t* frames_sent, uint32_t* frames_received, 
                                    uint32_t* errors)
{
    if (!g_protocol_initialized) {
        return AKS_ERROR_NOT_READY;
    }
    
    if (frames_sent != NULL) {
        *frames_sent = g_stats.frames_sent;
    }
    
    if (frames_received != NULL) {
        *frames_received = g_stats.frames_received;
    }
    
    if (errors != NULL) {
        *errors = g_stats.crc_errors + g_stats.frame_errors + g_stats.timeouts;
    }
    
    return AKS_OK;
}

/**
 * @brief Reset protocol statistics
 */
aks_result_t aks_protocol_reset_stats(void)
{
    if (!g_protocol_initialized) {
        return AKS_ERROR_NOT_READY;
    }
    
    memset(&g_stats, 0, sizeof(g_stats));
    
    return AKS_OK;
}

/**
 * @brief Validate received frame
 */
bool aks_protocol_is_valid_frame(const uint8_t* buffer, uint16_t length)
{
    if (buffer == NULL || length < AKS_PROTOCOL_HEADER_SIZE + AKS_PROTOCOL_FOOTER_SIZE) {
        return false;
    }
    
    /* Check start and end bytes */
    if (buffer[0] != AKS_PROTOCOL_START_BYTE) {
        return false;
    }
    
    if (buffer[length - 1] != AKS_PROTOCOL_END_BYTE) {
        return false;
    }
    
    /* Check length field consistency */
    uint16_t payload_length = (buffer[4] << 8) | buffer[5];
    uint16_t expected_length = AKS_PROTOCOL_HEADER_SIZE + payload_length + AKS_PROTOCOL_FOOTER_SIZE;
    
    if (length != expected_length) {
        return false;
    }
    
    /* Verify CRC */
    uint16_t received_crc = (buffer[length - 3] << 8) | buffer[length - 2];
    uint16_t calculated_crc = aks_protocol_calculate_crc16(buffer, length - AKS_PROTOCOL_FOOTER_SIZE);
    
    return (received_crc == calculated_crc);
}

/* Private function implementations */

/**
 * @brief Calculate CRC16 checksum
 */
static uint16_t aks_protocol_calculate_crc16(const uint8_t* data, uint16_t length)
{
    uint16_t crc = 0xFFFF;
    uint16_t polynomial = 0x1021; /* CRC-16-CCITT polynomial */
    
    for (uint16_t i = 0; i < length; i++) {
        crc ^= (data[i] << 8);
        
        for (uint8_t bit = 0; bit < 8; bit++) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ polynomial;
            } else {
                crc <<= 1;
            }
        }
    }
    
    return crc;
}

/**
 * @brief Encode message into protocol frame
 */
static aks_result_t aks_protocol_encode_frame(const aks_message_t* message, 
                                              uint8_t* buffer, uint16_t* frame_length)
{
    uint16_t index = 0;
    
    /* Start byte */
    buffer[index++] = AKS_PROTOCOL_START_BYTE;
    
    /* Message type */
    buffer[index++] = AKS_MSG_TYPE_DATA; /* Default to data type */
    
    /* Source ID */
    buffer[index++] = g_node_id;
    
    /* Destination ID (broadcast for now) */
    buffer[index++] = 0xFF;
    
    /* Payload length */
    buffer[index++] = (message->length >> 8) & 0xFF;
    buffer[index++] = message->length & 0xFF;
    
    /* Payload data */
    memcpy(&buffer[index], message->data, message->length);
    index += message->length;
    
    /* Calculate CRC over header + payload */
    uint16_t crc = aks_protocol_calculate_crc16(buffer, index);
    buffer[index++] = (crc >> 8) & 0xFF;
    buffer[index++] = crc & 0xFF;
    
    /* End byte */
    buffer[index++] = AKS_PROTOCOL_END_BYTE;
    
    *frame_length = index;
    g_stats.frames_sent++;
    
    return AKS_OK;
}

/**
 * @brief Decode protocol frame into message
 */
static aks_result_t aks_protocol_decode_frame(const uint8_t* buffer, uint16_t length, 
                                              aks_message_t* message)
{
    /* Validate frame first */
    if (!aks_protocol_is_valid_frame(buffer, length)) {
        g_stats.frame_errors++;
        return AKS_ERROR_CRC;
    }
    
    /* Extract payload length */
    uint16_t payload_length = (buffer[4] << 8) | buffer[5];
    
    if (payload_length > 8) { /* aks_message_t data field is 8 bytes */
        return AKS_ERROR_OVERFLOW;
    }
    
    /* Generate message ID based on message type and source */
    uint8_t msg_type = buffer[1];
    uint8_t source_id = buffer[2];
    static uint16_t sequence_counter = 0;
    
    /* Create unique message ID: [msg_type:4][source_id:4][sequence:8] */
    message->id = ((uint16_t)(msg_type & 0x0F) << 12) | 
                  ((uint16_t)(source_id & 0x0F) << 8) | 
                  (sequence_counter++ & 0xFF);
    
    message->length = payload_length;
    memcpy(message->data, &buffer[6], payload_length);
    message->timestamp = aks_core_get_tick(); /* Set current timestamp */
    
    /* Determine priority based on message type */
    switch (msg_type) {
        case AKS_MSG_TYPE_HEARTBEAT:
            message->priority = AKS_PRIORITY_LOW;
            break;
        case AKS_MSG_TYPE_COMMAND:
        case AKS_MSG_TYPE_ERROR:
            message->priority = AKS_PRIORITY_HIGH;
            break;
        case AKS_MSG_TYPE_RESPONSE:
        case AKS_MSG_TYPE_DATA:
        default:
            message->priority = AKS_PRIORITY_NORMAL;
            break;
    }
    
    g_stats.frames_received++;
    
    return AKS_OK;
}

/**
 * @brief Validate protocol frame
 */
static bool aks_protocol_validate_frame(const aks_protocol_frame_t* frame)
{
    if (frame == NULL) {
        return false;
    }
    
    /* Check start and end bytes */
    if (frame->start_byte != AKS_PROTOCOL_START_BYTE || 
        frame->end_byte != AKS_PROTOCOL_END_BYTE) {
        return false;
    }
    
    /* Check payload length */
    if (frame->length > AKS_PROTOCOL_MAX_PAYLOAD) {
        return false;
    }
    
    /* Verify CRC */
    uint16_t calculated_crc = aks_protocol_calculate_crc16((uint8_t*)frame, 
                                                           sizeof(aks_protocol_frame_t) - 3);
    
    return (frame->crc == calculated_crc);
}
