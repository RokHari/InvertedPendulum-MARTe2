#ifndef SOURCE_COMPONENTS_GAMS_STARTUPGAM_STARTUPGAM_H_
#define SOURCE_COMPONENTS_GAMS_STARTUPGAM_STARTUPGAM_H_

#include "GAM.h"

namespace InvertedPendulum {

/**
 * @brief GAM which performs two small absolute moves to unbalance the pendulum in case it is perfectly balanced.
 *
 * @details Motor status is monitor and motor is only moved if it is idle. Once the moves were performed, the GAM
 * indicates that its job is done by setting the CommandState to 1 for a single execution cycle only.
 * 
 * The configuration syntax is (names are only given as an example):
 * <pre>
 *  +Startup = {
 *     Class = StartupGAM
 *     InputSignals = {
 *         MotorState = { // Compulsory. Current motor state (see MotorState enum for valid values).
 *             Type = uint8 // Type must be uint8.
 *         }
 *     }
 *     OutputSignals = { // Order of signals is important.
 *         Command = { // Compulsory. Command to be sent to the motor (either MotorCommands::NoOp or MotorCommands::GoTo).
 *             Type = uint8 // Type must be uint8.
 *         }
 *         CommandParameter = { // Compulsory. Requested absolute position of the motor.
 *             Type = int32 // Type must be int32.
 *         }
 *         CommandState = { // Compulsory. Set to 1 when this GAM finishes its task. Else set to 0.
 *             Type = uint8 // Type must be uint8.
 *         }
 *     }
 * }
 * </pre>
 */
class StartupGAM : public MARTe::GAM, public MARTe::StatefulI {
public:
    CLASS_REGISTER_DECLARATION()

	StartupGAM();

	virtual ~StartupGAM();

    virtual bool Setup();

    virtual bool Execute();

    virtual bool PrepareNextState(const MARTe::char8* const currentStateName,
                                  const MARTe::char8* const nextStateName);

private:
    int moveCount;
	MARTe::uint8* inputMotorState;
	MARTe::uint8* outputCommand;
	MARTe::int32* outputCommandParam;
	MARTe::uint8* outputSwitchState;
};

} // namespace InvertedPendulum

#endif // SOURCE_COMPONENTS_GAMS_STARTUPGAM_STARTUPGAM_H_
