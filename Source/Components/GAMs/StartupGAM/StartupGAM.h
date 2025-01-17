#ifndef SOURCE_COMPONENTS_GAMS_STARTUPGAM_STARTUPGAM_H_
#define SOURCE_COMPONENTS_GAMS_STARTUPGAM_STARTUPGAM_H_

#include "GAM.h"

namespace InvertedPendulum {

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
