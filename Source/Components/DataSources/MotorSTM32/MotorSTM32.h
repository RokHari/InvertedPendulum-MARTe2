#ifndef SOURCE_COMPONENTS_DATASOURCES_MOTORSTM32_MOTORSTM32_H_
#define SOURCE_COMPONENTS_DATASOURCES_MOTORSTM32_MOTORSTM32_H_

#include "DataSourceI.h"

namespace InvertedPendulum {

/**
 * @brief A DataSource for reception and tranmission of ADC and DAC data frames from and to the
 * STM32 serial interface
 * 
 * @details The STM32 performs a blocking read of a serial port. It waits until a fixed number
 * of characters have been received, and then emits the characters as a input signal.
 * 
 * The DataSource shall have the following configuration (the object name `STM32` is an example - 
 * it is arbitrary):
 * <pre>
 * +STM32 = {
 *     Class = STM32
 *     Port = "/dev/serial0"
 *     BaudRate = 115200
 *     Signals = {
 *         ReceivedByteCount = {
 *             Type = uint32
 *         }
 *         DiscardedByteCount = {
 *             Type = uint32
 *         }
 *         ReadErrorCount = {
 *             Type = uint32
 *         }
 *         MessageCount = {
 *             Type = uint32
 *         }
 *         MessageRxTime = {
 *             Type = uint64
 *         }
 *         MessageTxTime = {
 *             Type = uint64
 *         }
 *         RxBufferOccupancy = {
 *             Type = uint32
 *         }
 *         ADCTime = {
 *             Type = uint32
 *         }
 *         PPS1Time = {
 *             Type = uint32
 *         }
 *         PPS2Time = {
 *             Type = uint32
 *         }
 *         ADC1Data = {
 *             Type = uint16
 *         }
 *         ADC2Data = {
 *             Type = uint16
 *         }
 *         DAC1Data = {
 *             Type = uint16
 *         }
 *         DAC2Data = {
 *             Type = uint16
 *         }
 *    }
 * }
 * </pre>
 * 
 * The Port is the path to the serial port device within the filesystem. The BaudRate is the data
 * speed that the DataSource will configure. 
 * 
 * The following signals must be defined:
 * 
 *  - ReceivedByteCount: The total number of bytes received on the serial port
 *  - DiscardedByteCount: The toal number of bytes discarded on the serial port. Bytes are discarded
 *    if the DataSource cannot assign them to a STM32 Rx data frame. This may be due to partial messages
 *    in the serial port buffer when the DataSource starts execution, or corrupted messages.
 *  - ReadErrorCount: The number of read errors received on reading the serial port. Read errors 
 *    correspond to the read system call returning a negative number. 
 *  - MessageCount: The number of STM32 data frames received
 *  - MessageRxTime: The (local) time at which the message was extracted from the receive buffer
 *  - MessageTxTime: The (local) time at which the *previous* message was sent to the STM32
 *  - RxBufferOccupancy: The number of bytes remaining in the receive buffer
 *  - PPS1Time: The STM32 timestamp of the first PPS signal
 *  - PPS2Time: The STM32 timestamp of the second PPS signal
 *  - ADC1Data: The first ADC data value
 *  - ADC2Data: The second ADC data value
 *  - DAC1Data: The first DAC data value
 *  - DAC2Data: The second DAC data value
 * 
 * Note that any characteristics of the serial port which are not covered by the configuration 
 * above (e.g. parity, number of data bits) will retain whatever default settings are defined
 * for the serial port.
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

    /**
     * @brief Read data from serial connection.
     *
     * @param destination Buffer into which the data is read.
     * @param size Number of data to read.
     */
    bool ReadSerialConnection(MARTe::uint8* destination,
                              unsigned int size);

 private:
    /**
     * File descriptor for the serial port
     */
    int serialFd;
    MARTe::uint8 command;
    MARTe::int32 commandParameter;
    MARTe::uint8* statusBuffer;
    MARTe::uint32 statusBufferSize;
};

} // namespace InvertedPendulum

#endif // SOURCE_COMPONENTS_DATASOURCES_MOTORSTM32_MOTORSTM32_H_
