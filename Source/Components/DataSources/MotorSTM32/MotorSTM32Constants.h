#ifndef SOURCE_COMPONENTS_DATASOURCES_MOTORSTM32_MOTORSTM32CONSTANTS_H_
#define SOURCE_COMPONENTS_DATASOURCES_MOTORSTM32_MOTORSTM32CONSTANTS_H_

#include "CompilerTypes.h"

namespace InvertedPendulum {

namespace MotorCommands {
/**
 * @brief Available motor commands. See STM32 projects for more information regarding
 *        particular command.
 */
enum MotorCommands {
    GetAcceleration = 6,
    GetCurrentSpeed = 7,
    GetDecelaration = 8,
    GetDeviceState = 9,
    GetFwVersion = 10,
    GetMark = 11,
    GetMaxSpeed = 12,
    GetMinSpeed = 13,
    GetPosition = 14,
    GoHome = 15,
    GoMark = 16,
    GoTo = 17,
    HardStop = 18,
    Move = 19,
    ResetAllDevices = 20,
    Run = 21,
    SetAcceleration = 22,
    SetDeceleration = 23,
    SetHome = 24,
    SetMark = 25,
    SetMaxSpeed = 26,
    SetMinSpeed = 27,
    SoftStop = 28,
    WaitWhileActive = 30,
    CmdDisable = 31,
    CmdEnable = 32,
    CmdGetParam = 33,
    CmdGetStatus = 34,
    CmdSetParam = 36,
    ReadStatusRegister = 37,
    ReleaseReset = 38,
    Reset = 39,
    SelectStepMode = 40,
    SetDirection = 41,
    CmdHardHiZ = 46,
    GetNbDevices = 55,
    SetStopMode = 68,
    GetStopMode = 69,
    GetDirection = 73,
    SetAnalogValue = 82,
    GetAnalogValue = 83,
    RT_EndControl = 252,
    ReadStatus = 253,
    RT_MoveMotor = 254,
    // Command not sent to STM32.
    NoOp = 255
};

} // namespace MotorCommands

namespace MotorState {

enum MotorState {
    Accelerating = 0,
    DeceleratingToStop = 1,
    Decelerating = 2,
    Steady = 3,
    IndexAccel = 4,
    IndexRun = 5,
    IndexDecel = 6,
    IndexDwell = 7,
    Inactive = 8,
    Standby = 9,
    StandbyToInactive = 10
};

} // namespace MotorState

} // namespace InvertedPendulum

#endif /* SOURCE_COMPONENTS_DATASOURCES_MOTORSTM32_MOTORSTM32CONSTANTS_H_ */
