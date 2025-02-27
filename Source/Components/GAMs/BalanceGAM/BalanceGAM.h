#ifndef SOURCE_COMPONENTS_GAMS_BALANCEGAM_BALANCEGAM_H_
#define SOURCE_COMPONENTS_GAMS_BALANCEGAM_BALANCEGAM_H_

#include "GAM.h"

namespace InvertedPendulum {

//TODO documentation
class BalanceGAM : public MARTe::GAM, public MARTe::StatefulI {
public:
    CLASS_REGISTER_DECLARATION()

    BalanceGAM();

    virtual ~BalanceGAM();

    virtual bool Initialise(MARTe::StructuredDataI& data);

    virtual bool Setup();

    virtual bool Execute();

    virtual bool PrepareNextState(const MARTe::char8* const currentStateName,
                                  const MARTe::char8* const nextStateName);

private:
    void SwingUp();

    void Balance(MARTe::float32 rtPeriod);

    MARTe::uint8* inputMotorState;
    MARTe::uint32* inputEncoderPosition;
    MARTe::int32* inputMotorPosition;
    MARTe::uint64* inputAbsoluteTime;
    MARTe::uint32* inputPrevEncoderPosition;
    MARTe::int32* inputPrevMotorPosition;
    MARTe::uint64* inputPrevAbsoluteTime;
    MARTe::uint32* inputEncoderPositionBottom;
    MARTe::uint8* inputEnableBalance;
    MARTe::uint8* outputCommand;
    MARTe::int32* outputCommandParam;
    MARTe::float32* outputRtAcc;
    MARTe::float32* outputRtPeriod;
    MARTe::uint8* outputSwitchState;

    MARTe::float32 k1;
    MARTe::float32 k2;
    MARTe::float32 k3;
    MARTe::float32 k4;

    bool firstMove;
    bool positiveDirection;
    bool balanceEnabled;
    bool exit;
    MARTe::int32 swingUpKick;
};

} // namespace InvertedPendulum

#endif // SOURCE_COMPONENTS_GAMS_BALANCEGAM_BALANCEGAM_H_
