// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Georgios Gkogkou <ggkogkou125@gmail.com>

#include "ResetBreadcrumb.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include "HardwareDiagnosticsLogger.hpp"
#include "definitions.h"

namespace {

void appendChar(char*& cursor, char* const end, const char value) {
        if (cursor < end) {
                *cursor = value;
                ++cursor;
        }
}

void appendString(char*& cursor, char* const end, const char* text) {
        if (text == nullptr)
                return;

        while ((*text != '\0') && (cursor < end)) {
                *cursor = *text;
                ++cursor;
                ++text;
        }
}

void appendHex32(char*& cursor, char* const end, const uint32_t value) {
        static constexpr char Digits[] = "0123456789ABCDEF";

        appendString(cursor, end, "0x");
        for (int shift = 28; shift >= 0; shift -= 4)
                appendChar(cursor, end, Digits[(value >> shift) & 0x0Fu]);
}

void appendHexPtr(char*& cursor, char* const end, const uintptr_t value) {
        if constexpr (sizeof(uintptr_t) <= sizeof(uint32_t)) {
                appendHex32(cursor, end, static_cast<uint32_t>(value));
        } else {
                static constexpr char Digits[] = "0123456789ABCDEF";

                appendString(cursor, end, "0x");
                for (int shift = static_cast<int>((sizeof(uintptr_t) * 8U) - 4U); shift >= 0; shift -= 4)
                        appendChar(cursor, end, Digits[(value >> shift) & 0x0Fu]);
        }
}

void finalize(char*& cursor, char* const end) {
        if (cursor < end) {
                *cursor = '\0';
        } else {
                *end = '\0';
        }
}

} // namespace

volatile ResetBreadcrumb::Record ResetBreadcrumb::breadcrumb __attribute__((section(".noinit.reset_breadcrumb"), used)) =
        {};

bool ResetBreadcrumb::isValid() {
        const bool validReason =
                breadcrumb.reason == Reason::TMR_MISMATCH || breadcrumb.reason == Reason::DEFAULT_HANDLER;

        return breadcrumb.magic == Magic && breadcrumb.version == Version && breadcrumb.magicInverse == ~Magic &&
               validReason;
}

bool ResetBreadcrumb::resetCauseCanCarryBreadcrumb(const uint8_t resetCause) {
        constexpr uint8_t ColdResetMask = PM_RCAUSE_POR_Msk | PM_RCAUSE_BOD12_Msk | PM_RCAUSE_BOD33_Msk;
        constexpr uint8_t BreadcrumbResetMask = PM_RCAUSE_SYST_Msk | PM_RCAUSE_WDT_Msk;

        return (resetCause & ColdResetMask) == 0u && (resetCause & BreadcrumbResetMask) != 0u;
}

void ResetBreadcrumb::clear() {
        breadcrumb.magic = 0;
        __DMB();
        breadcrumb.version = 0;
        breadcrumb.reason = Reason::NONE;
        breadcrumb.ipsr = 0;
        breadcrumb.primask = 0;
        breadcrumb.magicInverse = 0;
        breadcrumb.copy1Address = 0;
        breadcrumb.copy2Address = 0;
        breadcrumb.copy3Address = 0;
        breadcrumb.copy1Value = 0;
        breadcrumb.copy2Value = 0;
        breadcrumb.copy3Value = 0;
}

void ResetBreadcrumb::recordTMR(const void* copy1Address, const void* copy2Address, const void* copy3Address,
                                const uint32_t copy1Value, const uint32_t copy2Value, const uint32_t copy3Value) {
        breadcrumb.magic = 0;
        breadcrumb.version = Version;
        breadcrumb.reason = Reason::TMR_MISMATCH;
        breadcrumb.ipsr = __get_IPSR();
        breadcrumb.primask = __get_PRIMASK();
        breadcrumb.magicInverse = ~Magic;
        breadcrumb.copy1Address = reinterpret_cast<uintptr_t>(copy1Address);
        breadcrumb.copy2Address = reinterpret_cast<uintptr_t>(copy2Address);
        breadcrumb.copy3Address = reinterpret_cast<uintptr_t>(copy3Address);
        breadcrumb.copy1Value = copy1Value;
        breadcrumb.copy2Value = copy2Value;
        breadcrumb.copy3Value = copy3Value;
        __DMB();
        breadcrumb.magic = Magic;
}

void ResetBreadcrumb::recordDefaultHandler(const uint32_t ipsr, const uint32_t primask) {
        breadcrumb.magic = 0;
        breadcrumb.version = Version;
        breadcrumb.reason = Reason::DEFAULT_HANDLER;
        breadcrumb.ipsr = ipsr;
        breadcrumb.primask = primask;
        breadcrumb.magicInverse = ~Magic;
        breadcrumb.copy1Address = 0;
        breadcrumb.copy2Address = 0;
        breadcrumb.copy3Address = 0;
        breadcrumb.copy1Value = 0;
        breadcrumb.copy2Value = 0;
        breadcrumb.copy3Value = 0;
        __DMB();
        breadcrumb.magic = Magic;
}

void ResetBreadcrumb::logAndClear(const uint8_t resetCause) {
        if (!isValid())
                return;

        if (!resetCauseCanCarryBreadcrumb(resetCause)) {
                clear();
                return;
        }

        std::array<char, 256> message{};
        char* cursor = message.data();
        char* const end = message.data() + message.size() - 1;

        appendString(cursor, end, "RESET BREADCRUMB: ");

        if (breadcrumb.reason == Reason::TMR_MISMATCH) {
                appendString(cursor, end, "TMR");
                appendString(cursor, end, " IPSR=");
                appendHex32(cursor, end, breadcrumb.ipsr);
                appendString(cursor, end, " PRIMASK=");
                appendHex32(cursor, end, breadcrumb.primask);
                appendString(cursor, end, " A1=");
                appendHexPtr(cursor, end, breadcrumb.copy1Address);
                appendString(cursor, end, " V1=");
                appendHex32(cursor, end, breadcrumb.copy1Value);
                appendString(cursor, end, " A2=");
                appendHexPtr(cursor, end, breadcrumb.copy2Address);
                appendString(cursor, end, " V2=");
                appendHex32(cursor, end, breadcrumb.copy2Value);
                appendString(cursor, end, " A3=");
                appendHexPtr(cursor, end, breadcrumb.copy3Address);
                appendString(cursor, end, " V3=");
                appendHex32(cursor, end, breadcrumb.copy3Value);
        } else if (breadcrumb.reason == Reason::DEFAULT_HANDLER) {
                appendString(cursor, end, "DEFAULT_HANDLER");
                appendString(cursor, end, " IPSR=");
                appendHex32(cursor, end, breadcrumb.ipsr);
                appendString(cursor, end, " PRIMASK=");
                appendHex32(cursor, end, breadcrumb.primask);
        } else {
                appendString(cursor, end, "UNKNOWN");
        }

        appendString(cursor, end, "\r\n");
        finalize(cursor, end);

        HardwareDiagnostics::diagnosticsLogger.writeString(message.data());
        clear();
}
