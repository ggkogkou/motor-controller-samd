#pragma once

#include <type_traits>
#include "definitions.h"

template <typename T>
class TMR {
public:
        static_assert(std::is_trivially_copyable_v<T>);

        /**
         * Enum that indicates the diagnostic result of the voting process
         */
        enum class Fault : uint8_t {
                NONE = 0, /// All three copies agree
                VARIABLE_1 = 1, /// Variable 1 is considered corrupted
                VARIABLE_2 = 2, /// Variable 2 is considered corrupted
                VARIABLE_3 = 3, /// Variable 3 is considered corrupted
                UNRECOVERABLE = 4, /// No majority could be established
        };

        /**
         * Constructor that assigns variables as references to the TMR variables
         * @param a Variable 1
         * @param b Variable 2
         * @param c Variable 3
         */
        TMR(T& a, T& b, T& c) : variable1(a), variable2(b), variable3(c) {}
        explicit TMR(const T&) = delete;

        /**
         * The voting function that returns the majority value without repairing the variables
         *
         * @return The variable content that at least two variables have
         */
        [[nodiscard]] T read() const {
                if (variable1 == variable2 || variable1 == variable3)
                        return variable1;

                if (variable2 == variable3)
                        return variable2;

                __disable_irq();
                while (true) {
                        BENCHMARK_IO_Set();
                        SYSTICK_DelayMs(100);
                        BENCHMARK_IO_Clear();
                        SYSTICK_DelayMs(100);
                }

                NVIC_SystemReset();

                while (true) {
                }
        }

        /**
         * The voting function that returns the majority and reports which variable was considered faulty
         *
         * @param fault Diagnostic output that contains the faulty variable number
         * @return The variable content that at least two variables have
         */
        [[nodiscard]] T read(Fault& fault) const {
                if (variable1 == variable2 && variable1 == variable3) {
                        fault = Fault::NONE;
                        return variable1;
                }

                if (variable1 == variable2) {
                        fault = Fault::VARIABLE_3;
                        return variable1;
                }

                if (variable1 == variable3) {
                        fault = Fault::VARIABLE_2;
                        return variable1;
                }

                if (variable2 == variable3) {
                        fault = Fault::VARIABLE_1;
                        return variable2;
                }

                fault = Fault::UNRECOVERABLE;
                NVIC_SystemReset();

                while (true) {
                }
        }

        /**
         * The voting function that returns the majority; repair the values if there is a corrupted variable
         *
         * @return The variable content that at least two variables have
         */
        [[nodiscard]] T readAndRepair() {
                Fault fault = Fault::NONE;
                const T value = read(fault);

                if (fault == Fault::VARIABLE_1 || fault == Fault::VARIABLE_2 || fault == Fault::VARIABLE_3)
                        write(value);

                return value;
        }

        /**
         * The voting function that returns the majority; repair the values if there is a corrupted variable
         * and report which variable was considered faulty
         *
         * @param fault Diagnostic output that contains the faulty variable number
         * @return The variable content that at least two variables have
         */
        [[nodiscard]] T readAndRepair(Fault& fault) {
                const T value = read(fault);

                if (fault == Fault::VARIABLE_1 || fault == Fault::VARIABLE_2 || fault == Fault::VARIABLE_3)
                        write(value);

                return value;
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
};
