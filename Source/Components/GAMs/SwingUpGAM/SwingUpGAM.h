#ifndef SOURCE_COMPONENTS_GAMS_SWINGUPGAM_SWINGUPGAM_H_
#define SOURCE_COMPONENTS_GAMS_SWINGUPGAM_SWINGUPGAM_H_

#include "GAM.h"

namespace InvertedPendulum {

class SwingUpGAM : public MARTe::GAM, public MARTe::StatefulI {
public:
    CLASS_REGISTER_DECLARATION()

    SwingUpGAM();

    virtual ~SwingUpGAM();

    virtual bool Setup();

    virtual bool Execute();

    virtual bool PrepareNextState(const MARTe::char8* const currentStateName,
                                  const MARTe::char8* const nextStateName);

private:
    MARTe::uint8* inputMotorState;
    MARTe::uint32* inputEncoderPosition;
    MARTe::int32* inputMotorPosition;
    MARTe::uint32* inputEncoderPositionBottom;
    MARTe::uint8* outputCommand;
    MARTe::int32* outputCommandParam;
    MARTe::uint8* outputSwitchState;
    bool firstMove;
    bool positiveDirection;
    bool doNothing;
    MARTe::int32 previousEncoderPosition;
};

} // namespace InvertedPendulum

#endif // SOURCE_COMPONENTS_GAMS_SWINGUPGAM_SWINGUPGAM_H_
