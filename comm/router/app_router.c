/**
 * @file    app_router.c
 * @brief   Communication router — Implementation
 *
 * See app_router.h for architecture description.
 *
 * Integration guide (replace old app_com.c usage):
 *
 *   // In main.c or equivalent init:
 *   #include "uart_comm.h"
 *   #include "app_router.h"
 *
 *   uart_comm_t comm_vision;
 *   uart_comm_t comm_chassis;
 *
 *   // DMA buffers (placed in D2 SRAM on H7, any SRAM on F1/F4)
 *   static uint8_t vis_rx_buf[256] __attribute__((aligned(32)));
 *   static uint8_t vis_tx_buf[128];
 *   static uint8_t ch_rx_buf[256]  __attribute__((aligned(32)));
 *   static uint8_t ch_tx_buf[128];
 *
 *   void comm_init(void) {
 *       uart_comm_cfg_t cfg;
 *
 *       // Vision channel
 *       cfg = (uart_comm_cfg_t){
 *           .huart = &huart1, .rx_buf = vis_rx_buf, .rx_buf_size = 256,
 *           .tx_buf = vis_tx_buf, .tx_buf_size = 128, .payload_len = 5,
 *       };
 *       uart_comm_init(&comm_vision, &cfg);
 *
 *       // Chassis channel
 *       cfg.huart = &huart2; cfg.rx_buf = ch_rx_buf; cfg.tx_buf = ch_tx_buf;
 *       uart_comm_init(&comm_chassis, &cfg);
 *
 *       app_router_init(&comm_vision, &comm_chassis);
 *   }
 *
 *   // In main loop:
 *   while (1) {
 *       uart_comm_poll(&comm_vision);
 *       uart_comm_poll(&comm_chassis);
 *       app_router_poll();
 *       // ... other tasks ...
 *   }
 *
 *   // In HAL interrupt handlers (main.c):
 *   void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
 *       uart_comm_tx_cplt_handler(&comm_vision, huart);
 *       uart_comm_tx_cplt_handler(&comm_chassis, huart);
 *       // ... existing Vofa+ handler ...
 *   }
 *
 *   void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
 *       uart_comm_error_handler(&comm_vision, huart);
 *       uart_comm_error_handler(&comm_chassis, huart);
 *   }
 */

#include "app_router.h"
#include "uart_comm.h"
#include "gimbal_controller.h"
#include <string.h>

/* ================================================================
 * Static state
 * ================================================================ */

static uart_comm_t *router_vision  = NULL;
static uart_comm_t *router_chassis = NULL;

/* ================================================================
 * Internal helpers
 * ================================================================ */

/**
 * @brief Extract big-endian int16_t from payload bytes [offset..offset+1].
 * Uses memcpy for portability (safe on Cortex-M3/M4/M7).
 */
static int16_t extract_i16(const uint8_t *payload, uint8_t offset) {
    int16_t val;
    uint8_t swapped[2];
    /* Protocol uses big-endian: MSB first */
    swapped[0] = payload[offset + 1]; /* LSB */
    swapped[1] = payload[offset];     /* MSB */
    memcpy(&val, swapped, 2);
    return val;
}

/**
 * @brief Handle a single frame from the vision channel.
 * @return true if this frame was consumed locally (not forwarded).
 */
static bool handle_vision_frame(const uint8_t *payload, uint8_t len) {
    if (len == 0) return false;
    uint8_t tag = payload[0];

    switch (tag) {

    case TAG_GIMBAL_POSITION: {
        /* 0x01: Absolute gimbal position command
         * data_a = yaw target (milliradians, int16_t)
         * data_b = pitch target (milliradians, int16_t)
         */
        if (len < 5) break; /* Need 1 tag + 2 data_a + 2 data_b */
        float cmd_yaw   = (float)extract_i16(payload, 1) / 1000.0f;
        float cmd_pitch = (float)extract_i16(payload, 3) / 1000.0f;
        gimbal_ctrl_set_target(cmd_yaw, cmd_pitch);
        return true; /* Consumed locally, do not forward */
    }

    case TAG_GIMBAL_SPEED_LIMIT: {
        /* 0x03: Gimbal speed limit command
         * data_a = yaw vmax (milliradians/s, int16_t)
         * data_b = pitch vmax (milliradians/s, int16_t)
         */
        if (len < 5) break;
        float yaw_vmax   = (float)extract_i16(payload, 1) / 1000.0f;
        float pitch_vmax = (float)extract_i16(payload, 3) / 1000.0f;
        gimbal_ctrl_set_speed_limit(yaw_vmax, pitch_vmax);
        return true; /* Consumed locally */
    }

    default:
        /* Unknown tag — forward to chassis */
        break;
    }

    return false; /* Not consumed — should be forwarded */
}

/* ================================================================
 * Public API
 * ================================================================ */

void app_router_init(uart_comm_t *vision, uart_comm_t *chassis) {
    router_vision  = vision;
    router_chassis = chassis;
}

void app_router_poll(void) {
    uint8_t payload[UART_COMM_MAX_PAYLOAD];
    uint8_t len;
    bool consumed;

    /* ============================================================
     * Vision channel (Raspberry Pi → H743 → possibly chassis)
     * ============================================================ */
    if (router_vision && router_chassis) {
        while (uart_comm_get_packet(router_vision, payload, &len)) {
            consumed = handle_vision_frame(payload, len);

            if (!consumed) {
                /* Forward to chassis (transparent pass-through) */
                uart_comm_send_packet(router_chassis, payload, len);
            }
        }
    }

    /* ============================================================
     * Chassis channel (F103 → H743 → Raspberry Pi)
     * All chassis frames are transparently forwarded to vision.
     * ============================================================ */
    if (router_chassis && router_vision) {
        while (uart_comm_get_packet(router_chassis, payload, &len)) {
            uart_comm_send_packet(router_vision, payload, len);
        }
    }
}
