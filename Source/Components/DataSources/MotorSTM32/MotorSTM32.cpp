#include <sched.h>
#include <stdlib.h>
#include <stdio.h>
#include <pthread.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <linux/serial.h>
#include <string.h>
#include <cerrno>

#include "AdvancedErrorManagement.h"
#include "MemoryMapSynchronisedInputBroker.h"
#include "MemoryMapSynchronisedOutputBroker.h"
#include "Threads.h"

#include "MotorSTM32.h"
#include "MotorSTM32Reader.h"
#include "MotorSTM32Writer.h"
#include "MotorSTM32Constants.h"

namespace InvertedPendulum {

namespace
{

void serializeUint32(MARTe::uint32 value, MARTe::uint8* dest) {
    dest[0] = (MARTe::uint8)value;
    dest[1] = (MARTe::uint8)(value >> 8);
    dest[2] = (MARTe::uint8)(value >> 16);
    dest[3] = (MARTe::uint8)(value >> 24);
}

void serializeInt32(MARTe::int32 value, MARTe::uint8* dest) {
    MARTe::uint32 unsignedValue;
    memcpy(&unsignedValue, &value, sizeof(value));
    serializeUint32(unsignedValue, dest);
}

void serializeFloat32(MARTe::float32 value, MARTe::uint8* dest) {
    MARTe::uint32 unsignedValue;
    memcpy(&unsignedValue, &value, sizeof(value));
    serializeUint32(unsignedValue, dest);
}

const MARTe::uint32 kCommandSize = 10;
const MARTe::uint32 kCommandResponseSize = 7;
const MARTe::uint32 kStatusMetadataSize = 4;
const MARTe::uint8 kDeviceId = 0;

} // namespace

MotorSTM32::MotorSTM32() : DataSourceI(),
                           socketError(false),
                           serialFd(0),
                           command(0),
                           commandParameter(0),
                           rtAcceleration(0.0f),
                           rtPeriod(0.0f),
                           statusBuffer(NULL_PTR(MARTe::uint8*)),
                           statusBufferSize(0) {
    struct sched_param sp;
    sp.sched_priority = sched_get_priority_max(SCHED_FIFO);
    sched_setscheduler(0, SCHED_FIFO, &sp);

    cpu_set_t useCPUs;
    CPU_ZERO(&useCPUs);
    CPU_SET(1, &useCPUs);

    if (pthread_setaffinity_np(pthread_self(), sizeof (useCPUs), &useCPUs)) {
        perror("main() pthread_setaffinity_np");
        exit(1);
    }
}

MotorSTM32::~MotorSTM32() {

    if (statusBuffer != NULL_PTR(MARTe::uint8*)) {
        delete statusBuffer;
    }
}

bool MotorSTM32::Initialise(MARTe::StructuredDataI& data) {
    bool ok = DataSourceI::Initialise(data);

    MARTe::StreamString port;
    MARTe::uint32 baudRate;
    if (ok) {
        ok = data.Read("Port", port);
        if (!ok) {
            REPORT_ERROR(MARTe::ErrorManagement::ParametersError, "No serial port has been "
                    "specified");
        }
    }

    if (ok) {
        ok = data.Read("BaudRate", baudRate);
        if (!ok) {
            REPORT_ERROR(MARTe::ErrorManagement::ParametersError, "No baud rate has been "
                    "specified");
        }
    }

    // Open and configure the serial port
    if (ok) {
        serialFd = open(port.Buffer(), O_RDWR);
        ok = !(serialFd < 0);
        if (!ok) {
            REPORT_ERROR_PARAMETERS(MARTe::ErrorManagement::InitialisationError, "Failed to open "
                    "serial port, errno: %d.", errno);
        }

        struct termios tty;
        ok = (tcgetattr(serialFd, &tty) == 0);
        if (!ok) {
            REPORT_ERROR_PARAMETERS(MARTe::ErrorManagement::InitialisationError, "Failed to get "
                    "serial port settings, errno: %d.", errno);
        }
        if (ok) {
            int baudRateCode = B9600;
            switch (baudRate) {
                case 9600: baudRateCode = B9600; break;
                case 115200: baudRateCode = B115200; break;
                case 230400: baudRateCode = B230400; break;
                case 460800: baudRateCode = B460800; break;
                case 921600: baudRateCode = B921600; break;
                case 3000000: baudRateCode = B3000000; break;
                default:
                    REPORT_ERROR_PARAMETERS(MARTe::ErrorManagement::InitialisationError,
                            "Baud rate %u not supported.", baudRate);
                    ok = false;
            }
            
            cfmakeraw(&tty);
            cfsetispeed(&tty, baudRateCode);
            cfsetospeed(&tty, baudRateCode);

            tty.c_cflag &= ~CSTOPB;
            tty.c_cflag &= ~CRTSCTS;
            tty.c_cflag |= CLOCAL | CREAD;
            ok = (tcsetattr(serialFd, TCSANOW, &tty) == 0);
            if (!ok) {
                REPORT_ERROR_PARAMETERS(MARTe::ErrorManagement::InitialisationError, "Failed to "
                        "set serial port settings, errno: %d.", errno);
            }
        }
      
        // Configure the serial port for low latency operation
        if (ok) {
            struct serial_struct srl;
            ioctl(serialFd, TIOCGSERIAL, &srl);
            srl.flags |= ASYNC_LOW_LATENCY;
            ioctl(serialFd, TIOCSSERIAL, &srl);
        }

        // Flush the serial connection just in case.
        if (ok) {
            tcflush(serialFd, TCIOFLUSH);
        }
    }
    
    return ok;
}

bool MotorSTM32::Synchronise() {
    return true;
}

bool MotorSTM32::AllocateMemory() {
    return true;
}

bool MotorSTM32::GetSignalMemoryBuffer(const MARTe::uint32 signalIdx,
                                       const MARTe::uint32 bufferIdx,
                                       void*& signalAddress) {
    signalAddress = NULL;

    switch (signalIdx) {
        case 0u:
            signalAddress = reinterpret_cast<void *>(statusBuffer + kStatusMetadataSize);
            return true;
        case 1u:
            signalAddress = reinterpret_cast<void *>(&command);
            return true;
        case 2u:
            signalAddress = reinterpret_cast<void *>(&commandParameter);
            return true;
        case 3u:
            signalAddress = reinterpret_cast<void *>(&rtAcceleration);
            return true;
        case 4u:
            signalAddress = reinterpret_cast<void *>(&rtPeriod);
            return true;
        default:
            return false;
    }
}

const MARTe::char8* MotorSTM32::GetBrokerName(MARTe::StructuredDataI& data,
                                              const MARTe::SignalDirection direction) {
    if (direction == MARTe::InputSignals) {
        return "MotorSTM32Reader";
    } else {
        return "MotorSTM32Writer";
    }
}

bool MotorSTM32::GetInputBrokers(ReferenceContainer& inputBrokers,
                                 const MARTe::char8* const functionName,
                                 void* const gamMemPtr) {
    MARTe::ReferenceT<MotorSTM32Reader> broker("MotorSTM32Reader");
    bool ret = broker.IsValid();
    if (ret) {
        ret = broker->Init(MARTe::InputSignals, *this, functionName, gamMemPtr);
    }
    if (ret) {
        ret = inputBrokers.Insert(broker);
    }

    return ret;
}

bool MotorSTM32::GetOutputBrokers(ReferenceContainer& outputBrokers,
                                   const MARTe::char8* const functionName,
                                  void* const gamMemPtr) {
    MARTe::ReferenceT<MotorSTM32Writer> broker("MotorSTM32Writer");
    bool ret = broker.IsValid();
    if (ret) {
        ret = broker->Init(MARTe::OutputSignals, *this, functionName, gamMemPtr);
    }
    if (ret) {
        ret = outputBrokers.Insert(broker);
    }

    return ret;
}

bool MotorSTM32::SetConfiguredDatabase(MARTe::StructuredDataI& data) {
    bool ok = DataSourceI::SetConfiguredDatabase(data);
    
    if (ok) {
        ok = (GetNumberOfSignals() == 5u);
        if (!ok) {
            REPORT_ERROR(MARTe::ErrorManagement::InitialisationError, "Exactly 5 signals shall be defined.");
        }
    }
    if (ok) {
        ok = (GetSignalType(0u) == MARTe::UnsignedInteger8Bit);
        if (!ok) {
            REPORT_ERROR(MARTe::ErrorManagement::InitialisationError, "First signal shall be of type uint8");
        }
    }
    if (ok) {
        ok = GetSignalNumberOfElements(0u, statusBufferSize);
        if (ok) {
            // Increase buffer size in order to fit metadata as well..
            statusBufferSize += kStatusMetadataSize;
            statusBuffer = new MARTe::uint8[statusBufferSize];
        }
        else {
            REPORT_ERROR(MARTe::ErrorManagement::InitialisationError, "Number of elements shall be specified"
                    " for first signal");
        }
    }
    if (ok) {
        ok = (GetSignalType(1u) == MARTe::UnsignedInteger8Bit);
        if (!ok) {
            REPORT_ERROR(MARTe::ErrorManagement::InitialisationError, "Second signal shall be of type "
                    "uint8");
        }
    }

    if (ok) {
        ok = (GetSignalType(2u) == MARTe::SignedInteger32Bit);
        if (!ok) {
            REPORT_ERROR(MARTe::ErrorManagement::InitialisationError, "Third signal shall be of type "
                    "int32");
        }
    }

    if (ok) {
        ok = (GetSignalType(3u) == MARTe::Float32Bit);
        if (!ok) {
            REPORT_ERROR(MARTe::ErrorManagement::InitialisationError, "Fourth signal shall be of type "
                    "float32");
        }
    }

    if (ok) {
        ok = (GetSignalType(4u) == MARTe::Float32Bit);
        if (!ok) {
            REPORT_ERROR(MARTe::ErrorManagement::InitialisationError, "Fifth signal shall be of type "
                    "float32");
        }
    }

    return ok;
}

bool MotorSTM32::PrepareNextState(const MARTe::char8* const currentStateName,
                                  const MARTe::char8* const nextStateName) {
    return true;
}

bool MotorSTM32::RxSynchronise() {
    if (!ClearSocketError()) {
        return false;
    }

    MARTe::uint8 dataBuff[kCommandSize];
    // Set command ID (read status).
    dataBuff[0] = MotorCommands::ReadStatus;
    // Set device ID.
    dataBuff[1] = kDeviceId;

    if (!WriteSerialConnection(dataBuff, sizeof(dataBuff))) {
        return false;
    }

    if (!ReadSerialConnection(statusBuffer, statusBufferSize)) {
        return false;
    }

    return ValidateResponse(MotorCommands::ReadStatus, kDeviceId, statusBuffer);
}

bool MotorSTM32::TxSynchronise() {
    if (!ClearSocketError()) {
        return false;
    }

    MARTe::uint8 dataBuff[kCommandSize];

    switch (command) {
        case MotorCommands::GoTo:
            serializeInt32(commandParameter, dataBuff + 2);
            break;
        case MotorCommands::RT_MoveMotor:
            serializeFloat32(rtAcceleration, dataBuff + 2);
            serializeFloat32(rtPeriod, dataBuff + 6);
            break;
        case MotorCommands::RT_EndControl:
            // No parameters needed for RT_EndControl.
            break;
        default:
            // No other command supported via input signals.
            return true;
    }

    // Set command ID.
    dataBuff[0] = command;
    // Set device ID.
    dataBuff[1] = kDeviceId;

    // Send the command.
    if (!WriteSerialConnection(dataBuff, sizeof(dataBuff))) {
        return false;
    }

    if (command == MotorCommands::RT_MoveMotor)
    {
        // No response expected on RT move command.
        return true;
    }

    // Wait for response.
    MARTe::uint8 responseBuff[kCommandResponseSize];
    if (!ReadSerialConnection(responseBuff, sizeof(responseBuff))) {
        return false;
    }

    return ValidateResponse(command, kDeviceId, responseBuff);
}

bool MotorSTM32::ReadSerialConnection(MARTe::uint8* destination,
                                      unsigned int size) {
    unsigned int dataRead = 0;
    for (int i = 0; i < 20; ++i) {
        ssize_t ret = read(serialFd, destination + dataRead, size - dataRead);
        if (ret < 0) {
            REPORT_ERROR_PARAMETERS(MARTe::ErrorManagement::CommunicationError,
                    "Error on socket read, errno: %d.", errno);
            socketError = true;
            return false;
        }
        dataRead += ret;
        if (dataRead == size) {
            return true;
        }
    }
    REPORT_ERROR(MARTe::ErrorManagement::CommunicationError, "Timeout on socket read.");
    socketError = true;
    return false;
}

bool MotorSTM32::WriteSerialConnection(MARTe::uint8* data,
                                       unsigned int size) {
    unsigned int dataWritten = 0;
    for (int i = 0; i < 20; ++i) {
        ssize_t ret = write(serialFd, data + dataWritten, size - dataWritten);
        if (ret < 0) {
            REPORT_ERROR_PARAMETERS(MARTe::ErrorManagement::CommunicationError,
                    "Error on socket write, errno: %d.", errno);
            socketError = true;
            return false;
        }
        dataWritten += ret;
        if (dataWritten == size) {
            return true;
        }
    }
    REPORT_ERROR(MARTe::ErrorManagement::CommunicationError, "Timeout on socket write.");
    socketError = true;
    return false;
}

bool MotorSTM32::ClearSocketError() {
    // Only flush the socket when error state is set.
    if (socketError) {
        if (tcflush(serialFd, TCIOFLUSH) != 0) {
            REPORT_ERROR_PARAMETERS(MARTe::ErrorManagement::CommunicationError,
                    "Failed to flush the socket, errno: %d.", errno);
            return false;
        }
        // Reset error state if flush succeeded.
        socketError = false;
    }
    return true;
}

bool MotorSTM32::ValidateResponse(MARTe::uint8 commandId,
                                  MARTe::uint8 deviceId,
                                  MARTe::uint8* response) {
    if (response[0] != commandId) {
        REPORT_ERROR_PARAMETERS(MARTe::ErrorManagement::CommunicationError, "Expected response for"
                " command %u, received response for command %u.", commandId, response[0]);
        return false;
    }
    if (response[1] != deviceId) {
        REPORT_ERROR_PARAMETERS(MARTe::ErrorManagement::CommunicationError, "Expected response for"
                " device %u, received response for device %u.", deviceId, response[1]);
        return false;
    }
    if (response[2] != 0) {
        REPORT_ERROR_PARAMETERS(MARTe::ErrorManagement::CommunicationError, "Failed to execute "
                "command %u, error %u.", commandId, response[2]);
        return false;
    }

    switch (commandId) {
        case MotorCommands::RT_EndControl:
            if (response[3] == 0) {
                REPORT_ERROR_PARAMETERS(MARTe::ErrorManagement::CommunicationError, "Command %u "
                        "responded with unexpected value: FALSE.", commandId);
                return false;
            }
            return true;
        default:
            return true;
    }
}

CLASS_REGISTER(MotorSTM32, "1.0");

} // namespace MFI
