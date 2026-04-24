// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Georgios Gkogkou <ggkogkou125@gmail.com>

#pragma once

#include <cstdint>

class ResetBreadcrumb {
public:
        enum class Reason : uint32_t {
                NONE = 0,
                TMR_MISMATCH = 1,
                DEFAULT_HANDLER = 2,
        };

        struct Record {
                uint32_t magic;
                uint32_t version;
                Reason reason;
                uint32_t ipsr;
                uint32_t primask;
                uint32_t magicInverse;
                uintptr_t copy1Address;
                uintptr_t copy2Address;
                uintptr_t copy3Address;
                uint32_t copy1Value;
                uint32_t copy2Value;
                uint32_t copy3Value;
        };

        static void clear();
        static void recordTMR(const void* copy1Address, const void* copy2Address, const void* copy3Address,
                              uint32_t copy1Value, uint32_t copy2Value, uint32_t copy3Value);
        static void recordDefaultHandler(uint32_t ipsr, uint32_t primask);
        static void logAndClear(uint8_t resetCause);

private:
        static constexpr uint32_t Magic = 0x52535442u;
        static constexpr uint32_t Version = 2u;

        static volatile Record breadcrumb;
        static bool isValid();
        static bool resetCauseCanCarryBreadcrumb(uint8_t resetCause);
};
