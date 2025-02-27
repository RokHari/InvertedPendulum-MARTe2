#ifndef SOURCE_COMPONENTS_GAMS_RESETGAM_RESETGAM_H_
#define SOURCE_COMPONENTS_GAMS_RESETGAM_RESETGAM_H_

#include "GAM.h"

namespace InvertedPendulum {

//TODO documentation
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
