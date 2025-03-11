#ifndef SOURCE_COMPONENTS_GAMS_RESETGAM_RESETGAM_H_
#define SOURCE_COMPONENTS_GAMS_RESETGAM_RESETGAM_H_

#include "GAM.h"

namespace InvertedPendulum {

/**
 * @brief GAM resets the real-time control of the motor. 
 *
 * @details
 * 
 * The configuration syntax is (names are only given as an example):
 * <pre>
 * +ResetMotor = {
 *     Class = ResetGAM
 *     OutputSignals = { // Order of signals is important.
 *         Command = { // Compulsory. Command to be sent to the motor (either MotorCommands::NoOp or MotorCommands::RT_EndControl).
 *             Type = uint8 // Type must be uint8.
 *         }
 *         CommandState = { // Compulsory. Set to 4 when this GAM finishes its task. Else set to 0.
 *             Type = uint8 // Type must be uint8.
 *         }
 *     }
 * }
 * </pre>
 */
class ResetGAM : public MARTe::GAM, public MARTe::StatefulI {
public:
    CLASS_REGISTER_DECLARATION()

    ResetGAM();

    virtual ~ResetGAM();

    virtual bool Setup();

    virtual bool Execute();

    virtual bool PrepareNextState(const MARTe::char8* const currentStateName,
                                  const MARTe::char8* const nextStateName);

private:
    MARTe::uint8* outputCommand;
    MARTe::uint8* outputSwitchState;
    bool resetDone;
};

} // namespace InvertedPendulum

#endif // SOURCE_COMPONENTS_GAMS_RESETGAM_RESETGAM_H_
