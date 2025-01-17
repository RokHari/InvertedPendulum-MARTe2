#include "SwingUpGAM.h"

#include "AdvancedErrorManagement.h"

#include <algorithm>
//TODO
#include <iostream>

namespace InvertedPendulum {

SwingUpGAM::SwingUpGAM() : GAM(),
                           inputMotorState(NULL_PTR(MARTe::uint8*)),
                           inputEncoderPosition(NULL_PTR(MARTe::uint32*)),
                           inputMotorPosition(NULL_PTR(MARTe::int32*)),
                           inputEncoderPositionBottom(NULL_PTR(MARTe::uint32*)),
                           outputCommand(NULL_PTR(MARTe::uint8*)),
                           outputCommandParam(NULL_PTR(MARTe::int32*)),
                           outputSwitchState(NULL_PTR(MARTe::uint8*)),
                           firstMove(true),
                           positiveDirection(true),
                           doNothing(false),
                           previousEncoderPosition(0u) {
}

SwingUpGAM::~SwingUpGAM() {
}

bool SwingUpGAM::Setup() {
    bool ok = GetNumberOfInputSignals() == 4;
    if (!ok) {
        REPORT_ERROR(MARTe::ErrorManagement::ParametersError, "Number of input signals must be 4.");
    }
    if (ok) {
        ok = GetSignalType(MARTe::InputSignals, 0u) == MARTe::UnsignedInteger8Bit;
        if (ok) {
            inputMotorState = static_cast<MARTe::uint8*>(GetInputSignalMemory(0u));
        }
        else {
            REPORT_ERROR(MARTe::ErrorManagement::InitialisationError, "First input signal shall be of type "
                    "uint8");
        }
    }
    if (ok) {
        ok = GetSignalType(MARTe::InputSignals, 1u) == MARTe::UnsignedInteger32Bit;
        if (ok) {
            inputEncoderPosition = static_cast<MARTe::uint32*>(GetInputSignalMemory(1u));
        }
        else {
            REPORT_ERROR(MARTe::ErrorManagement::InitialisationError, "Second input signal shall be of type "
                    "uint32");
        }
    }
    if (ok) {
        ok = GetSignalType(MARTe::InputSignals, 2u) == MARTe::SignedInteger32Bit;
        if (ok) {
            inputMotorPosition = static_cast<MARTe::int32*>(GetInputSignalMemory(2u));
        }
        else {
            REPORT_ERROR(MARTe::ErrorManagement::InitialisationError, "Third input signal shall be of type "
                    "int32");
        }
    }
    if (ok) {
        ok = GetSignalType(MARTe::InputSignals, 3u) == MARTe::UnsignedInteger32Bit;
        if (ok) {
            inputEncoderPositionBottom = static_cast<MARTe::uint32*>(GetInputSignalMemory(3u));
        }
        else {
            REPORT_ERROR(MARTe::ErrorManagement::InitialisationError, "Fourth input signal shall be of type "
                    "uint32");
        }
    }
    if (ok) {
        bool ok = GetNumberOfOutputSignals() == 3;
        if (!ok) {
            REPORT_ERROR(MARTe::ErrorManagement::ParametersError, "Number of output signals must be 3.");
        }
    }
    if (ok) {
        ok = GetSignalType(MARTe::OutputSignals, 0u) == MARTe::UnsignedInteger8Bit;
        if (ok) {
            outputCommand = static_cast<MARTe::uint8*>(GetOutputSignalMemory(0u));
        }
        else {
            REPORT_ERROR(MARTe::ErrorManagement::InitialisationError, "First output signal shall be of type "
                    "uint8");
        }
    }
    if (ok) {
        ok = GetSignalType(MARTe::OutputSignals, 1u) == MARTe::SignedInteger32Bit;
        if (ok) {
            outputCommandParam = static_cast<MARTe::int32*>(GetOutputSignalMemory(1u));
        }
        else {
            REPORT_ERROR(MARTe::ErrorManagement::InitialisationError, "Second output signal shall be of "
                    "type int32");
        }
    }
    if (ok) {
        ok = GetSignalType(MARTe::OutputSignals, 2u) == MARTe::UnsignedInteger8Bit;
        if (ok) {
            outputSwitchState = static_cast<MARTe::uint8*>(GetOutputSignalMemory(2u));
        }
        else {
            REPORT_ERROR(MARTe::ErrorManagement::InitialisationError, "Third output signal shall be of "
                    "type uint8");
        }
    }
    return ok;
}

MARTe::int32 normalizeEncoderPos(MARTe::uint32 position,
                                 MARTe::uint32 positionBottom) {
    MARTe::int32 positionNorm;
    int bottomNorm;
    if (position > positionBottom) {
        if (position - positionBottom > 2400) {
            positionNorm = static_cast<MARTe::int32>(position) - (1 << 16);
        }
        else {
            positionNorm = static_cast<MARTe::int32>(position);
        }
        bottomNorm = static_cast<MARTe::int32>(positionBottom);
    }
    else {
        if (positionBottom - position > 2400) {
            bottomNorm = static_cast<MARTe::int32>(positionBottom) - (1 << 16);
        }
        else {
            bottomNorm = static_cast<MARTe::int32>(positionBottom);
        }
        positionNorm = static_cast<MARTe::int32>(position);
    }

    return positionNorm - bottomNorm;
}

bool oppositeSigns(MARTe::int32 x, MARTe::int32 y)
{
    return ((x ^ y) >> 31) != 0;
}


bool SwingUpGAM::Execute() {
    *outputCommand = 255u;
    *outputCommandParam = 0u;
    *outputSwitchState = 0u;

    if (doNothing) {
        return true;
    }

    MARTe::int32 normPosition = normalizeEncoderPos(*inputEncoderPosition, *inputEncoderPositionBottom);

    if (firstMove) {
        // Do not move the motor if it is already moving.
        if (*inputMotorState != 8) {
            return true;
        }
        // Always give a kick to the motor at the start, to get the pendulum moving.
        firstMove = false;
        *outputCommand = 17u;
        if (!positiveDirection) {
            *outputCommandParam = *inputMotorPosition + 250;
        }
        else {
            *outputCommandParam = *inputMotorPosition - 250;
        }
    }
    else {
        // If we are close to the highest position (within 10 steps, move to the nest state.)
        if (std::abs(normPosition) > 1180) {
            doNothing = true;
            *outputSwitchState = 3u;
            return true;
        }

        // If the pendulum crossed the bottom position, add a kick.
        if (oppositeSigns(previousEncoderPosition, normPosition)) {
            // Only move the motor when it's not moving.
            if (*inputMotorState == 8) {
                *outputCommand = 17u;
                if (positiveDirection) {
                    *outputCommandParam = *inputMotorPosition + 100;
                }
                else {
                    *outputCommandParam = *inputMotorPosition - 100;
                }
            }
            positiveDirection = !positiveDirection;
        }
    }

    previousEncoderPosition = normPosition;

    return true;
}

bool SwingUpGAM::PrepareNextState(const MARTe::char8* const currentStateName,
                                  const MARTe::char8* const nextStateName) {
    firstMove = true;
    positiveDirection = true;
    doNothing = false;
    return true;
}

CLASS_REGISTER(SwingUpGAM, "1.0");

} // namespace InvertedPendulum
