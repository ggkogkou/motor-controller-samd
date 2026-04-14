#include "HealthMonitor.hpp"
#include "definitions.h"

constinit HealthMonitor healthMonitor;
constinit HealthMonitor healthMonitorCopy;

static inline uint32_t delta32(uint32_t now, uint32_t prev) {
        return now - prev;
}

HealthSnapshot takeHealthSnapshot() {
        __disable_irq();

        const HealthSnapshot snapshot{
                .tc3EnterCounter = healthMonitor.tc3EnterCounter,
                .tc3ValidCounter = healthMonitor.tc3ValidCounter,
                .tc4EnterCounter = healthMonitor.tc4EnterCounter,
                .tc4ValidCounter = healthMonitor.tc4ValidCounter,
                .adcPairsReadyCounter = healthMonitor.adcPairsReadyCounter,
                .adcValidCounter = healthMonitor.adcValidCounter,
                .encoderUpdatesCounter = healthMonitor.encoderUpdatesCounter,
                .tmrRepairEvents = healthMonitor.tmrRepairEvents,
                .faultCounter = healthMonitor.faultCounter,
        };

        __enable_irq();

        return snapshot;
}

HealthDecision evaluateHealthWindow(const HealthSnapshot& now, const HealthSnapshot& prev, SupervisionMode mode) {
        HealthDecision d{};

        d.tc3Complete = (now.tc3EnterCounter == now.tc3ValidCounter);
        d.tc3Progress = (delta32(now.tc3ValidCounter, prev.tc3ValidCounter) > 0u);

        d.tc4Complete = (now.tc4EnterCounter == now.tc4ValidCounter);
        d.tc4Progress = (delta32(now.tc4ValidCounter, prev.tc4ValidCounter) > 0u);

        d.adcComplete = (now.adcPairsReadyCounter == now.adcValidCounter);
        d.adcProgress = (delta32(now.adcValidCounter, prev.adcValidCounter) > 0u);

        d.encoderProgress = (delta32(now.encoderUpdatesCounter, prev.encoderUpdatesCounter) > 0u);

        d.noFaults = (delta32(now.faultCounter, prev.faultCounter) == 0u);

        switch (mode) {
        case SupervisionMode::STARTUP:
                d.healthy = d.tc3Complete && d.tc3Progress && d.tc4Complete && d.tc4Progress && d.noFaults;
                break;

        case SupervisionMode::CLOSED_LOOP:
                d.healthy = d.tc3Complete && d.tc3Progress && d.tc4Complete && d.tc4Progress && d.adcComplete && d.adcProgress &&
                        d.encoderProgress && d.noFaults;
                break;

        case SupervisionMode::FAULTED:
        default:
                d.healthy = false;
                break;
        }

        return d;
}
