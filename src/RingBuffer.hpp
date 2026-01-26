// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Georgios Gkogkou <ggkogkou125@gmail.com>

/*
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

/**
 * @file   RingBuffer.hpp
 * @brief  A ring buffer implementation for USART pipelining usage
 * @author Georgios Gkogkou <ggkogkou125@gmail.com>
 */

#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <span>
#include "definitions.h"

/**
 * @brief Single-producer / single-consumer ring buffer for bytes
 *
 * Design choices:
 * - Waste one slot to distinguish full vs empty
 * - Producer updates head, consumer updates tail
 *
 * Typical use with USART DMA TX:
 * - Producer (main thread): write(...)
 * - Consumer (DMA ISR): consume(n)
 */
class RingBuffer {
public:
        /**
         * @param storage backing memory; must have size >= 2
         */
        explicit RingBuffer(std::span<uint8_t> storage)
                : buffer(storage), sizeBytes(storage.size()) {}

        void reset() {
                head = 0;
                tail = 0;
                droppedBytes = 0;
        }

        [[nodiscard]] size_t capacity() const {
                return (sizeBytes >= 2) ? (sizeBytes - 1U) : 0U;
        }

        [[nodiscard]] size_t used() const {
                const size_t h = head;
                const size_t t = tail;
                if (h >= t)
                        return h - t;
                return sizeBytes - (t - h);
        }

        [[nodiscard]] size_t free() const {
                const size_t cap = capacity();
                const size_t u = used();
                return (u <= cap) ? (cap - u) : 0U;
        }

        [[nodiscard]] bool empty() const {
                return head == tail;
        }

        [[nodiscard]] bool full() const {
                return next(head) == tail;
        }

        [[nodiscard]] uint32_t getDroppedBytes() const {
                return droppedBytes;
        }

        /**
         * @brief Enqueue as many bytes as fit; returns bytes accepted
         */
        size_t write(std::span<const uint8_t> data) {
                if (data.empty() || sizeBytes < 2)
                        return 0;

                const size_t h = head;
                const size_t t = tail;

                const size_t u = usedFromSnapshots(h, t);
                const size_t cap = sizeBytes - 1U;
                const size_t f = (u <= cap) ? (cap - u) : 0U;

                const size_t toWrite = (data.size() < f) ? data.size() : f;

                if (toWrite == 0) {
                        droppedBytes = droppedBytes + static_cast<uint32_t>(data.size());
                        return 0;
                }

                const size_t untilEnd = sizeBytes - h;
                const size_t first = (toWrite < untilEnd) ? toWrite : untilEnd;

                std::memcpy(buffer.data() + h, data.data(), first);

                if (const size_t second = toWrite - first) {
                        std::memcpy(buffer.data(), data.data() + first, second);
                }

                __DMB();

                head = wrapAdd(h, toWrite);

                if (toWrite < data.size()) {
                        droppedBytes = droppedBytes + static_cast<uint32_t>(data.size() - toWrite);
                }

                return toWrite;
        }

        /**
         * @brief Returns the next readable contiguous block starting at tail
         */
        [[nodiscard]] std::span<const uint8_t> peekContiguous() const {
                const size_t h = head;
                const size_t t = tail;

                if (h == t)
                        return {};

                const size_t len = (h > t) ? (h - t) : (sizeBytes - t);
                return std::span<const uint8_t>(buffer.data() + t, len);
        }

        /**
         * @brief Consume up to n bytes (advance tail).
         * @return Bytes consumed
         */
        size_t consume(size_t n) {
                if (const size_t avail = used(); n > avail)
                        n = avail;

                tail = wrapAdd(tail, n);
                return n;
        }

private:
        std::span<uint8_t> buffer;
        const size_t sizeBytes;

        volatile size_t head = 0;
        volatile size_t tail = 0;

        volatile uint32_t droppedBytes = 0;

        size_t next(size_t idx) const {
                ++idx;

                if (idx == sizeBytes)
                        idx = 0;

                return idx;
        }

        size_t wrapAdd(size_t idx, size_t n) const {
                idx += n;

                if (idx >= sizeBytes)
                        idx -= sizeBytes;

                return idx;
        }

        size_t usedFromSnapshots(size_t h, size_t t) const {
                if (h >= t)
                        return h - t;

                return sizeBytes - (t - h);
        }
};
