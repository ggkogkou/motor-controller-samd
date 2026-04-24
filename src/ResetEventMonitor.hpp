#pragma once

#include <cstdint>
#include "HardwareDiagnosticsLogger.hpp"
#include "definitions.h"

struct ResetEventMonitor {
        enum class ResetEventCause : uint8_t {
                SYST = 0b0100'0000,
                WDT = 0b0010'0000,
                EXT = 0b0001'0000,
                BOD33 = 0b0000'0100,
                BOD12 = 0b0000'0010,
                POR = 0b0000'0001,
                UNKNOWN = 0b0000'0000,
        };

        static_assert(PM_RCAUSE_SYST_Msk == static_cast<uint8_t>(ResetEventCause::SYST), "SYST mismatch");
        static_assert(PM_RCAUSE_WDT_Msk == static_cast<uint8_t>(ResetEventCause::WDT), "WDT mismatch");
        static_assert(PM_RCAUSE_EXT_Msk == static_cast<uint8_t>(ResetEventCause::EXT), "EXT mismatch");
        static_assert(PM_RCAUSE_BOD33_Msk == static_cast<uint8_t>(ResetEventCause::BOD33), "BOD33 mismatch");
        static_assert(PM_RCAUSE_BOD12_Msk == static_cast<uint8_t>(ResetEventCause::BOD12), "BOD12 mismatch");
        static_assert(PM_RCAUSE_POR_Msk == static_cast<uint8_t>(ResetEventCause::POR), "POR mismatch");

        static uint8_t determineResetCause() {
                const uint8_t RCAUSE_RegisterValue = PM_REGS->PM_RCAUSE;

                if (RCAUSE_RegisterValue & static_cast<uint8_t>(ResetEventCause::POR))
                        HardwareDiagnostics::diagnosticsLogger.logResetPOR();

                if (RCAUSE_RegisterValue & static_cast<uint8_t>(ResetEventCause::BOD12))
                        HardwareDiagnostics::diagnosticsLogger.writeString("RESET: BOD12\r\n");

                if (RCAUSE_RegisterValue & static_cast<uint8_t>(ResetEventCause::BOD33))
                        HardwareDiagnostics::diagnosticsLogger.logResetBOD33();

                if (RCAUSE_RegisterValue & static_cast<uint8_t>(ResetEventCause::WDT))
                        HardwareDiagnostics::diagnosticsLogger.logResetWDT();

                if (RCAUSE_RegisterValue & static_cast<uint8_t>(ResetEventCause::EXT))
                        HardwareDiagnostics::diagnosticsLogger.logResetExternal();

                if (RCAUSE_RegisterValue & static_cast<uint8_t>(ResetEventCause::SYST))
                        HardwareDiagnostics::diagnosticsLogger.logResetSoftware();

                return RCAUSE_RegisterValue;
        }
};
