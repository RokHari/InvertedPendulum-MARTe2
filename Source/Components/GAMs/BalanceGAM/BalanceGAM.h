#ifndef SOURCE_COMPONENTS_GAMS_BALANCEGAM_BALANCEGAM_H_
#define SOURCE_COMPONENTS_GAMS_BALANCEGAM_BALANCEGAM_H_

#include "GAM.h"

namespace InvertedPendulum {

/**
 * @brief GAM swings the pendulum up and balances it.
 *
 * @details The pendulum swing up is done by moving the motor using MotorCommands::GoTo command. Once the top position
 * is reach, the balancing stage begins, which uses the MotorCommands::RT_MoveMotor. If the motors moves too far from the
 * starting position or if the pendulum is too far away from the top position, the CommandState is set to 3 and the GAM
 * no longer sends commands to STM32. If this happens a MotorCommands::RT_EndControl needs to be sent to STM32 in order
 * to renable normal (non-real-time) commands.
 * 
 * The configuration syntax is (names are only given as an example):
 * <pre>
 * +SwingUpAndBalance = {
 *     Class = BalanceGAM
 *     K1 = 65.0 // Compulsory. Motor position weight.
 *     K2 = -1500.0 // Compulsory. Encoder position weight.
 *     K3 = -0.12 // Compulsory. Motor speed weight.
 *     K4 = -20.0 // Compulsory. Encoder speed weight.
 *     InputSignals = { // Order of signals is important.
 *         MotorState = { // Compulsory. Current motor state (see MotorState enum for valid values).
 *             Type = uint8 // Type must be uint8.
 *         }
 *         EncoderPosition = { // Compulsory. Current encoder/pendulum position read from STM32.
 *             Type = uint32 // Type must be uint32.
 *         }
 *         MotorPosition = { // Compulsory. Current motor position read from STM32.
 *             Type = int32 // Type must be int32.
 *         }
 *         AbsoluteTime = { // Compulsory. Current absolute time in microseconds.
 *             Type = uint64 // Type must be uint64.
 *         }
 *         PrevEncoderPosition = {// Compulsory. Previous encoder/pendulum position.
 *             Type = uint32 // Type must be uint32.
 *         }
 *         PrevMotorPosition = { // Compulsory. Previous motor position.
 *             Type = int32 // Type must be int32.
 *         }
 *         PrevAbsoluteTime = {// Compulsory. Previous absolute time in microseconds.
 *             Type = uint64 // Type must be uint64.
 *         }
 *         PendulumPositionBotom = { // Compulsory. Bottom pendulum position.
 *             Type = uint32 // Type must be uint32.
 *         }
 *     }
 *     OutputSignals = { // Order of signals is important.
 *         Command = { // Compulsory. Command to be sent to the motor (either MotorCommands::NoOp, MotorCommands::RT_MoveMotor, or MotorCommands::GoTo).
 *             Type = uint8 // Type must be uint8.
 *         }
 *         CommandParameter = { // Compulsory. Requested absolute position of the motor when Command is MotorCommands::GoTo.
 *             Type = int32 // Type must be int32.
 *         }
 *         RtAcc = { // Compulsory. Requested motor acceleration when Command is MotorCommands::RT_MoveMotor.
 *             Type = float32 // Type must be float32.
 *         }
 *         RtPeriod = { // Compulsory. Real-time period when Command is MotorCommands::RT_MoveMotor.
 *             Type = float32 // Type must be float32.
 *         }
 *         CommandState = { // Compulsory. Set to 3 when the pendulum can no longer be balanced. Else set to 0.
 *             Type = uint8 // Type must be uint8.
 *         }
 *     }
 * }
 * </pre>
 */
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
