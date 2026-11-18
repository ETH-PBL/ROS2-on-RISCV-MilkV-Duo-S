#define DEBUG_MODULE "ANGLEUART"

#include <stdint.h>
#include "FreeRTOS.h"
#include "task.h"
#include "deck.h"
#include "uart2.h"
#include "log.h"
#include "debug.h"

#define UART_BAUDRATE 115200
#define SEND_FREQUENCY_MS 10    // 100Hz

#define PACKET_HEADER 0xAA55
#define PACKET_FOOTER 0x55AA

typedef struct {
    uint16_t header;
    int16_t roll;
    int16_t pitch;
    int16_t yaw;
    uint16_t footer;
} __attribute__((packed)) AttitudePacket;

static int logIdRoll, logIdPitch, logIdYaw;
static bool logIdsInitialized = false;

static void initLogIds() {
    if (!logIdsInitialized) {
        logIdRoll = logGetVarId("stateEstimate", "roll");
        logIdPitch = logGetVarId("stateEstimate", "pitch");
        logIdYaw = logGetVarId("stateEstimate", "yaw");
        
        if (logIdRoll != -1 && logIdPitch != -1 && logIdYaw != -1) {
            logIdsInitialized = true;
            DEBUG_PRINT("ANGLE UART: Initialized\n");
        }
    }
}

static void angleUartTask(void *param) {
    TickType_t lastWakeTime = xTaskGetTickCount();
    AttitudePacket packet;
    float roll, pitch, yaw;
    
    vTaskDelay(M2T(1000));
    
    while(1) {
        if (!logIdsInitialized) {
            initLogIds();
            vTaskDelayUntil(&lastWakeTime, M2T(SEND_FREQUENCY_MS));
            continue;
        }
        
        roll = logGetFloat(logIdRoll);
        pitch = logGetFloat(logIdPitch);
        yaw = logGetFloat(logIdYaw);
        
        packet.header = PACKET_HEADER;
        packet.roll = (int16_t)(roll * 100.0f);
        packet.pitch = (int16_t)(pitch * 100.0f);
        packet.yaw = (int16_t)(yaw * 100.0f);
        packet.footer = PACKET_FOOTER;
        
        uart2SendData(sizeof(packet), (uint8_t*)&packet);
        
        vTaskDelayUntil(&lastWakeTime, M2T(SEND_FREQUENCY_MS));
    }
}

static void angleUartDeckInit(DeckInfo *info) {
    uart2Init(UART_BAUDRATE);
    DEBUG_PRINT("ANGLE UART: Started\n");
    
    xTaskCreate(angleUartTask, "ANGLEUART", 
                configMINIMAL_STACK_SIZE * 2, NULL, 
                3, NULL);
}

static bool angleUartDeckTest() {
    return true;
}

static const DeckDriver angleUartDeck = {
    .name = "bcIMUUART",
    .usedGpio = 0,
    .usedPeriph = DECK_USING_UART2,
    .init = angleUartDeckInit,
    .test = angleUartDeckTest,
};

DECK_DRIVER(angleUartDeck);
