#ifndef SOURCE_COMPONENTS_DATASOURCES_MOTORSTM32_MOTORSTM32_H_
#define SOURCE_COMPONENTS_DATASOURCES_MOTORSTM32_MOTORSTM32_H_

#include "DataSourceI.h"

namespace InvertedPendulum {

/**
 * @brief A DataSource for sending motor commands to STM32 and reading the motor status.
 * 
 * @details The MotorSTM32 sends a command over the serial port, and if needed waits for a reponse
 * in a blocking way. The only supported commands are:
 *     - Got to absolute position (MotorCommands::GoTo)
 *     - Real-time control: apply acceleration to the motor (MotorCommands::RT_MoveMotor)
 *     - End real-time control (MotorCommands::RT_EndControl)
 * Once a real-time control command (MotorCommands::RT_MoveMotor) is issued, an end real-time control
 * command (MotorCommands::RT_EndControl) must be executed before any other motor commands can be used.
 * 
 * The configuration syntax is (names are only given as an example):
 * <pre>
 * +STM32 = {
 *     Class = MotorSTM32
 *     Port = "/dev/ttyACM0" // Compulsory. Must be set to the device file of the SMT32.
 *     BaudRate = 230400 // Compulsory. Use the same value as is used for baud rate in the STM32 code.
 *     Signals = { // Order of signals is important.
 *         Status = { // Compulsory output signal. Signal containing motor status.
 *             Type = uint8 // Type must be uint8.
 *             NumberOfElements = 9 // Number of bytes in status response.
 *             NumberOfDimensions = 1 // Dimensions must be set to 1.
 *         }
 *         Command = { // Compulsory input signal. Three supported commands (see details).
 *             Type = uint8 // Type must be uint8.
 *         }
 *         CommandParameter = { // Compulsory input signal. Parameter for the MotorCommands::GoTo command (absolute position in steps).
 *             Type = int32 // Type must be int32.
 *         }
 *         RtAcc = { // Compulsory input signal. Parameter for the MotorCommands::RT_MoveMotor command (acceleration in steps per second squared).
 *             Type = float32 // Type must be float32.
 *         }
 *         RtPeriod = { // Compulsory input signal. Parameter for the MotorCommands::RT_MoveMotor command (real-time period in seconds).
 *             Type = float32 // Type must be float32.
 *         }
 *     }
 * }
 * </pre>
 */
class MotorSTM32 : public MARTe::DataSourceI {
public:
    CLASS_REGISTER_DECLARATION();
    
    MotorSTM32();
    
    virtual ~MotorSTM32();

    virtual bool AllocateMemory();

    virtual bool Initialise(MARTe::StructuredDataI& data);
    
    virtual bool Synchronise();

    virtual bool GetSignalMemoryBuffer(const MARTe::uint32 signalIdx,
                                       const MARTe::uint32 bufferIdx,
                                       void*& signalAddress);

    virtual const MARTe::char8* GetBrokerName(MARTe::StructuredDataI& data,
                                              const MARTe::SignalDirection direction);

    virtual bool GetInputBrokers(MARTe::ReferenceContainer& inputBrokers,
                                 const MARTe::char8* const functionName,
                                 void* const gamMemPtr);

    virtual bool GetOutputBrokers(MARTe::ReferenceContainer& outputBrokers,
                                  const MARTe::char8* const functionName,
                                  void* const gamMemPtr);
    
    virtual bool SetConfiguredDatabase(MARTe::StructuredDataI& data);

    virtual bool PrepareNextState(const MARTe::char8* const currentStateName,
                                  const MARTe::char8* const nextStateName);

    /**
     * @brief Synchronise the received signals
     * 
     * Assumes that the input broker execution calls this function *before* calling Synchronise
     */
    bool RxSynchronise();

    /**
     * @brief Synchronise the transmitted signals
     * 
     * Assumes that the input broker execution calls this function *after* calling Synchronise
     */
    bool TxSynchronise();

 private:
    /**
     * @brief Read data from serial connection.
     *
     * @param destination Buffer into which the data is read.
     * @param size Number of data to read.
     * @return False if there was an error reading. Else true.
     */
    bool ReadSerialConnection(MARTe::uint8* destination,
                              unsigned int size);

    /**
     * @brief Write data to serial connection.
     *
     * @param data Buffer holding the data to be sent.
     * @param size Number of data to send.
     * @return False if there was an error writing. Else true.
     */
    bool WriteSerialConnection(MARTe::uint8* data,
                               unsigned int size);

    /**
     * @brief Flush the socket if error flag is set.
     *
     * @return False if socket flush was needed and failed. Else true.
     */
    bool ClearSocketError();

    /**
     * @param commandId Expected command ID in the response.
     * @param deviceId Expected device ID in the response.
     * @param response Response to be validated.
     *
     * @return True if response is valid, else false.
     */
    bool ValidateResponse(MARTe::uint8 commandId,
                          MARTe::uint8 deviceId,
                          MARTe::uint8* response);

    bool socketError;
    /**
     * File descriptor for the serial port
     */
    int serialFd;

    MARTe::uint8 command;
    MARTe::int32 commandParameter;
    MARTe::float32 rtAcceleration;
    MARTe::float32 rtPeriod;
    MARTe::uint8* statusBuffer;
    MARTe::uint32 statusBufferSize;
};

} // namespace InvertedPendulum

#endif // SOURCE_COMPONENTS_DATASOURCES_MOTORSTM32_MOTORSTM32_H_
