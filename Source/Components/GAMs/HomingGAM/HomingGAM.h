#ifndef SOURCE_COMPONENTS_GAMS_HOMINGGAM_HOMINGGAM_H_
#define SOURCE_COMPONENTS_GAMS_HOMINGGAM_HOMINGGAM_H_

#include "GAM.h"
#include "StatefulI.h"

namespace InvertedPendulum {

/**
 * @brief GAM waits for the pendulum to stop moving after which the bottom position of the pendulum is set.
 *
 * @details
 * 
 * The configuration syntax is (names are only given as an example):
 * <pre>
 * +Homing = {
 *     Class = HomingGAM
 *     InputSignals = {
 *         EncoderPosition = { // Compulsory. Current encoder/pendulum position read from STM32.
 *             Type = uint32 // Type must be uint32.
 *         }
 *     }
 *     OutputSignals = { // Order of signals is important.
 *         CommandState = { // Compulsory. Set to 2 when this GAM finishes its task. Else set to 0.
 *             Type = uint8 // Type must be uint8.
 *         }
 *         PendulumPositionBotom = { // Compulsory. Contains the bottom position of the pendulum once the homing procedure is done.
 *             Type = uint32 // Type must be uint32.
 *         }
 *     }
 * }
 * </pre>
 */
class HomingGAM : public MARTe::GAM, public MARTe::StatefulI {
public:
    CLASS_REGISTER_DECLARATION()

    HomingGAM();

    virtual ~HomingGAM();

    virtual bool Setup();

    virtual bool Execute();

    virtual bool PrepareNextState(const MARTe::char8* const currentStateName,
                                  const MARTe::char8* const nextStateName);

private:
    MARTe::uint32* inputEncoderPosition;
    MARTe::uint8* outputSwitchState;
    MARTe::uint32* outputEncoderPositionBottom;
    MARTe::uint32 encoderPositionOld1;
    MARTe::uint32 encoderPositionOld2;
    // Counter used to detect when at least 3 measurements of encoder position were done.
    int counter;
};

} // namespace InvertedPendulum

#endif // SOURCE_COMPONENTS_GAMS_HOMINGGAM_HOMINGGAM_H_
