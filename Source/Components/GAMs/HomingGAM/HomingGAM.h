#ifndef SOURCE_COMPONENTS_GAMS_HOMINGGAM_HOMINGGAM_H_
#define SOURCE_COMPONENTS_GAMS_HOMINGGAM_HOMINGGAM_H_

#include "GAM.h"
#include "StatefulI.h"

namespace InvertedPendulum {

//TODO documentation
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
