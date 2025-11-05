#include "foc.hpp"

void FieldOrientedControl::FOC::algorithmCallback(TC_TIMER_STATUS status, uintptr_t context) {

}

void FieldOrientedControl::FOC::encoderOffsetCalibration() {
    constexpr float Vq = EncoderAlignmentVoltageLimit;
    constexpr float Vd = 0.0f;
    constexpr float ThetaElectrical = 4.71238898038f;

    __disable_irq();

    const auto AlphaBetaFrame = MathUtilities::performInverseParkTransform(Vd, Vq, ThetaElectrical);
    const auto DutyCycles = spaceVectorPWM.compute(AlphaBetaFrame[0], AlphaBetaFrame[1]);
    const auto [dutyCycleA, dutyCycleB, dutyCycleC] = DutyCycles;

    const auto period = TCC0_PWM24bitPeriodGet();
    const auto TCC_PeriodU = period - static_cast<uint32_t>(static_cast<float>(period) * dutyCycleA);
    const auto TCC_PeriodV = period - static_cast<uint32_t>(static_cast<float>(period) * dutyCycleB);
    const auto TCC_PeriodW = period - static_cast<uint32_t>(static_cast<float>(period) * dutyCycleC);



}
