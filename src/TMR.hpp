#pragma once

#include <type_traits>
#include "core_cm0plus.h"

template <typename T>
class TMR {
public:
        static_assert(std::is_trivially_copyable_v<T>);

        explicit TMR(const T& initial) : variable1(initial), variable2(initial), variable3(initial) {}

        /**
         * The voting function that returns the majority; repair the values if there is a corrupted variable
         *
         * @return The variable content that at least two variables have
         */
        [[nodiscard]] T read() {
                if (variable1 == variable2 && variable1 == variable3)
                        return variable1;

                if (variable1 == variable2 || variable1 == variable3) {
                        write(variable1);
                        return variable1;
                }

                if (variable2 == variable1 || variable2 == variable3) {
                        write(variable2);
                        return variable2;
                }

                if (variable3 == variable1 || variable3 == variable2) {
                        write(variable3);
                        return variable3;
                }

                NVIC_SystemReset();

                while (true) {
                }
        }

        /**
         * Write a value to the variable; it will be written to all three copies in different places in RAM
         *
         * @param value
         */
        void write(const T& value) {
                variable1 = value;
                variable2 = value;
                variable3 = value;
        }

private:
        /**
         * Variable 1 that goes to RAM section tmr_a
         */
        T variable1;

        /**
         * Variable 2 that goes to RAM section tmr_b
         */
        T variable2;

        /**
         * Variable 3 that goes to RAM section tmr_c
         */
        T variable3;
};
