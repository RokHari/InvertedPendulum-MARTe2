#include "HomingGAM.h"

#include "AdvancedErrorManagement.h"

namespace InvertedPendulum {

HomingGAM::HomingGAM() : GAM(),
                         inputEncoderPosition(NULL_PTR(MARTe::uint32*)),
                         outputSwitchState(NULL_PTR(MARTe::uint8*)),
                         outputEncoderPositionBottom(NULL_PTR(MARTe::uint32*)),
                         encoderPositionOld1(0),
                         encoderPositionOld2(0),
                         counter(0) {
}

HomingGAM::~HomingGAM() {
}

bool HomingGAM::Setup() {
    bool ok = GetNumberOfInputSignals() == 1;
    if (!ok) {
        REPORT_ERROR(MARTe::ErrorManagement::ParametersError, "Number of input signals must be 1.");
    }
    if (ok) {
        ok = GetSignalType(MARTe::InputSignals, 0u) == MARTe::UnsignedInteger32Bit;
        if (ok) {
            inputEncoderPosition = static_cast<MARTe::uint32*>(GetInputSignalMemory(0u));
        }
        else {
            REPORT_ERROR(MARTe::ErrorManagement::InitialisationError, "First input signal shall be of type "
                    "uint32");
        }
    }
    if (ok) {
        ok = GetNumberOfOutputSignals() == 2;
        if (!ok) {
            REPORT_ERROR(MARTe::ErrorManagement::ParametersError, "Number of output signals must be 2.");
        }
    }
    if (ok) {
        ok = GetSignalType(MARTe::OutputSignals, 0u) == MARTe::UnsignedInteger8Bit;
        if (ok) {
            outputSwitchState = static_cast<MARTe::uint8*>(GetOutputSignalMemory(0u));
        }
        else {
            REPORT_ERROR(MARTe::ErrorManagement::InitialisationError, "First output signal shall be of type "
                    "uint8");
        }
    }
    if (ok) {
        ok = GetSignalType(MARTe::OutputSignals, 1u) == MARTe::UnsignedInteger32Bit;
        if (ok) {
            outputEncoderPositionBottom = static_cast<MARTe::uint32*>(GetOutputSignalMemory(1u));
        }
        else {
            REPORT_ERROR(MARTe::ErrorManagement::InitialisationError, "Second output signal shall be of "
                    "type uint32");
        }
    }
    return ok;
}

bool HomingGAM::Execute() {
    if (counter < 3) {
        // Wait to get at least 3 measurements.
        ++counter;
        *outputSwitchState = 0;
    }
    else {
        // Switch state if last 3 measurements are identical.
        if (*inputEncoderPosition == encoderPositionOld1 &&
                *inputEncoderPosition == encoderPositionOld2) {
            *outputSwitchState = 2;
            *outputEncoderPositionBottom = *inputEncoderPosition;
        }
        else {
            *outputSwitchState = 0;
        }
    }

    // Update stored values for next execution cycle.
    encoderPositionOld2 = encoderPositionOld1;
    encoderPositionOld1 = *inputEncoderPosition;

    return true;
}

bool HomingGAM::PrepareNextState(const MARTe::char8* const currentStateName,
                                 const MARTe::char8* const nextStateName) {
    counter = 0;
    return true;
}

CLASS_REGISTER(HomingGAM, "1.0");

} // namespace InvertedPendulum
