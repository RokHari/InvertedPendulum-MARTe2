#include "StartupGAM.h"
#include "MotorSTM32Constants.h"

#include "AdvancedErrorManagement.h"

namespace InvertedPendulum {

StartupGAM::StartupGAM() : GAM(),
                           moveCount(0),
                           inputMotorState(NULL_PTR(MARTe::uint8*)),
                           outputCommand(NULL_PTR(MARTe::uint8*)),
                           outputCommandParam(NULL_PTR(MARTe::int32*)),
                           outputSwitchState(NULL_PTR(MARTe::uint8*)) {
}

StartupGAM::~StartupGAM() {
}

bool StartupGAM::Setup() {
	bool ok = GetNumberOfInputSignals() == 1;
	if (!ok) {
        REPORT_ERROR(MARTe::ErrorManagement::ParametersError, "Number of input signals must be 1.");
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

bool StartupGAM::Execute() {
    // No command is the default output.
    *outputCommand = MotorCommands::NoOp;
    *outputCommandParam = 0u;
    *outputSwitchState = 0u;

    if (*inputMotorState == MotorState::Inactive) {
        if (moveCount == 0) {
            // Move motor on startup, to kick the pendulum of balance.
            *outputCommand = MotorCommands::GoTo;
            *outputCommandParam = 60u;
            *outputSwitchState = 0u;
            ++moveCount;
        }
        else if (moveCount == 1) {
            // Move back to the 0 position.
            *outputCommand = MotorCommands::GoTo;
            *outputCommandParam = 0u;
            *outputSwitchState = 1u;
            ++moveCount;
        }
    }

    return true;
}

bool StartupGAM::PrepareNextState(const MARTe::char8* const currentStateName,
                                  const MARTe::char8* const nextStateName) {
    moveCount = 0;
    return true;
}

CLASS_REGISTER(StartupGAM, "1.0");

} // namespace InvertedPendulum
