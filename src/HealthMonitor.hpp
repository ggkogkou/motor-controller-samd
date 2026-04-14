#pragma once

#include <cstdint>

struct HealthMonitor {
        volatile uint32_t tc3EnterCounter = 0;
        volatile uint32_t tc3ValidCounter = 0;

        volatile uint32_t tc4EnterCounter = 0;
        volatile uint32_t tc4ValidCounter = 0;

        volatile uint32_t adcPairsReadyCounter = 0;
        volatile uint32_t adcValidCounter = 0;

        volatile uint32_t encoderUpdatesCounter = 0;

        volatile uint32_t tmrRepairEvents = 0;

        volatile uint32_t faultCounter = 0;

        void reset() {
                tc3EnterCounter = 0;
                tc3ValidCounter = 0;

                tc4EnterCounter = 0;
                tc4ValidCounter = 0;

                adcPairsReadyCounter = 0;
                adcValidCounter = 0;

                encoderUpdatesCounter = 0;

                tmrRepairEvents = 0;
                faultCounter = 0;
        }
};

struct HealthSnapshot {
        uint32_t tc3EnterCounter = 0;
        uint32_t tc3ValidCounter = 0;

        uint32_t tc4EnterCounter = 0;
        uint32_t tc4ValidCounter = 0;

        uint32_t adcPairsReadyCounter = 0;
        uint32_t adcValidCounter = 0;

        uint32_t encoderUpdatesCounter = 0;

        uint32_t tmrRepairEvents = 0;
        uint32_t faultCounter = 0;
};

enum class SupervisionMode : uint8_t { STARTUP, CLOSED_LOOP, FAULTED };

struct HealthDecision {
        bool healthy = false;

        bool tc3Complete = false;
        bool tc3Progress = false;

        bool tc4Complete = false;
        bool tc4Progress = false;

        bool adcComplete = false;
        bool adcProgress = false;

        bool encoderProgress = false;

        bool noFaults = false;
};

extern constinit HealthMonitor healthMonitor;
extern constinit HealthMonitor healthMonitorCopy;

HealthSnapshot takeHealthSnapshot();
HealthDecision evaluateHealthWindow(const HealthSnapshot& now, const HealthSnapshot& prev, SupervisionMode mode);
