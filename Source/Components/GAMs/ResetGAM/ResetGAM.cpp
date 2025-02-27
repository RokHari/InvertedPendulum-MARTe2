#include "ResetGAM.h"
#include "MotorSTM32Constants.h"

#include "AdvancedErrorManagement.h"

#include <algorithm>

namespace InvertedPendulum {

ResetGAM::ResetGAM() : GAM(),
                       outputCommand(NULL_PTR(MARTe::uint8*)),
                       outputSwitchState(NULL_PTR(MARTe::uint8*)),
                       resetDone(false) {
}

ResetGAM::~ResetGAM() {
}

bool ResetGAM::Setup() {
    // Validate output signals
    bool ok = GetNumberOfOutputSignals() == 2;
    if (!ok) {
        REPORT_ERROR(MARTe::ErrorManagement::ParametersError, "Number of output signals must be 2.");
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
        ok = GetSignalType(MARTe::OutputSignals, 1u) == MARTe::UnsignedInteger8Bit;
        if (ok) {
            outputSwitchState = static_cast<MARTe::uint8*>(GetOutputSignalMemory(1u));
        }
        else {
            REPORT_ERROR(MARTe::ErrorManagement::InitialisationError, "Second output signal shall be of type "
                    "uint8");
        }
    }
    return ok;
}

bool ResetGAM::Execute() {
    *outputCommand = MotorCommands::NoOp;
    *outputSwitchState = 0u;

    if (!resetDone) {
        *outputCommand = MotorCommands::RT_EndControl;
        *outputSwitchState = 4u;
        resetDone = true;
    }
    return true;
}

bool ResetGAM::PrepareNextState(const MARTe::char8* const currentStateName,
                                const MARTe::char8* const nextStateName) {
    resetDone = false;
    return true;
}

CLASS_REGISTER(ResetGAM, "1.0");

} // namespace InvertedPendulum
