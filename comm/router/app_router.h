/**
 * @file    app_router.h
 * @brief   Communication router — connects uart_comm channels to gimbal_controller
 *
 * Thin routing layer. Responsibilities:
 *   1. Receive frames from vision channel (Raspberry Pi → H743)
 *   2. Dispatch gimbal commands (tag 0x01, 0x03) to gimbal_controller
 *   3. Forward all other frames to chassis channel (H743 → F103)
 *   4. Receive frames from chassis channel (F103 → H743)
 *   5. Forward all chassis frames to vision channel (H743 → Raspberry Pi)
 *
 * Dependencies:
 *   - uart_comm.h   (uart_comm_t instances for vision and chassis)
 *   - gimbal_controller.h
 */

#ifndef __APP_ROUTER_H
#define __APP_ROUTER_H

#include <stdint.h>
#include <stdbool.h>

/* Forward declaration */
struct uart_comm_s;
typedef struct uart_comm_s uart_comm_t;

/* ================================================================
 * Protocol Tag Definitions
 * ================================================================ */

/** Raspberry Pi → H743: Gimbal absolute position command */
#define TAG_GIMBAL_POSITION   0x01

/** Raspberry Pi → H743: Gimbal speed limit command */
#define TAG_GIMBAL_SPEED_LIMIT 0x03

/* ================================================================
 * Public API
 * ================================================================ */

/**
 * @brief Initialize the router with two uart_comm channels.
 *
 * Must be called after uart_comm_init() for both channels.
 *
 * @param vision   Vision channel (Raspberry Pi ↔ H743, USART1)
 * @param chassis  Chassis channel (H743 ↔ STM32F103, USART2)
 */
void app_router_init(uart_comm_t *vision, uart_comm_t *chassis);

/**
 * @brief Main loop processing — call from while(1).
 *
 * Processes all pending frames from both channels:
 *   - Vision → filter gimbal commands → forward rest to chassis
 *   - Chassis → forward all to vision
 */
void app_router_poll(void);

#endif /* __APP_ROUTER_H */
