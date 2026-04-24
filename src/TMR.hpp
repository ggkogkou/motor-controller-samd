#pragma once

#include <cstdint>
#include <cstring>
#include <type_traits>
#include "ResetBreadcrumb.hpp"
#include "definitions.h"

template <typename T>
class TMR {
public:
        static_assert(std::is_trivially_copyable_v<T>);
        static_assert(sizeof(T) <= sizeof(uint32_t));

        /**
         * Constructor that assigns variables as references to the TMR variables
         * @param a Variable 1
         * @param b Variable 2
         * @param c Variable 3
         */
        TMR(T& a, T& b, T& c) : variable1(a), variable2(b), variable3(c) {}
        explicit TMR(const T&) = delete;

        /**
         * The voting function that returns the majority value and repairs the variables if needed
         *
         * @return The variable content that at least two variables have
         */
        [[nodiscard]] T read() const {
                const T copy1 = variable1;
                const T copy2 = variable2;
                const T copy3 = variable3;

                if (copy1 == copy2) {
                        if (copy1 != copy3)
                                write(copy1);

                        return copy1;
                }

                if (copy1 == copy3) {
                        write(copy1);
                        return copy1;
                }

                if (copy2 == copy3) {
                        write(copy2);
                        return copy2;
                }

                __disable_irq();
                ResetBreadcrumb::recordTMR(&variable1, &variable2, &variable3, encodeForBreadcrumb(copy1),
                                           encodeForBreadcrumb(copy2), encodeForBreadcrumb(copy3));
                NVIC_SystemReset();

                while (true) {
                }
        }

        /**
         * Write a value to the variable; it will be written to all three copies in different places in RAM
         *
         * @param value
         */
        void write(T value) const {
                variable1 = value;
                variable2 = value;
                variable3 = value;
        }

private:
        /**
         * Reference to variable 1 that goes to RAM section tmr_a
         */
        T& variable1;

        /**
         * Reference to variable 2 that goes to RAM section tmr_b
         */
        T& variable2;

        /**
         * Reference to variable 3 that goes to RAM section tmr_c
         */
        T& variable3;

        [[nodiscard]] static uint32_t encodeForBreadcrumb(const T& value) {
                uint32_t encoded = 0;
                std::memcpy(&encoded, &value, sizeof(T));
                return encoded;
        }
};
