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

        const uint32_t tc3EnterDelta = delta32(now.tc3EnterCounter, prev.tc3EnterCounter);
        const uint32_t tc3ValidDelta = delta32(now.tc3ValidCounter, prev.tc3ValidCounter);

        const uint32_t tc4EnterDelta = delta32(now.tc4EnterCounter, prev.tc4EnterCounter);
        const uint32_t tc4ValidDelta = delta32(now.tc4ValidCounter, prev.tc4ValidCounter);

        const uint32_t adcReadyDelta = delta32(now.adcPairsReadyCounter, prev.adcPairsReadyCounter);
        const uint32_t adcValidDelta = delta32(now.adcValidCounter, prev.adcValidCounter);

        const uint32_t encoderDelta = delta32(now.encoderUpdatesCounter, prev.encoderUpdatesCounter);
        const uint32_t faultDelta = delta32(now.faultCounter, prev.faultCounter);

        d.tc3Complete = (tc3EnterDelta == tc3ValidDelta);
        d.tc3Progress = (tc3ValidDelta > 0u);

        d.tc4Complete = (tc4EnterDelta == tc4ValidDelta);
        d.tc4Progress = (tc4ValidDelta > 0u);

        d.adcComplete = (adcReadyDelta == adcValidDelta);
        d.adcProgress = (adcValidDelta > 0u);

        d.encoderProgress = (encoderDelta > 0u);

        d.noFaults = (faultDelta == 0u);

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
