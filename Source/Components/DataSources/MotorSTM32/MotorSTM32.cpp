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
#include <iostream>

#include "AdvancedErrorManagement.h"
#include "MemoryMapSynchronisedInputBroker.h"
#include "MemoryMapSynchronisedOutputBroker.h"
#include "Threads.h"

#include "MotorSTM32.h"
#include "MotorSTM32Reader.h"
#include "MotorSTM32Writer.h"

namespace InvertedPendulum {

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

const MARTe::uint32 kCommandSize = 10;
const MARTe::uint32 kCommandResponseSize = 7;
const MARTe::uint32 kStatusMetadataSize = 4;

MotorSTM32::MotorSTM32() : DataSourceI(),
                           serialFd(0),
						   data(0),
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
            REPORT_ERROR(MARTe::ErrorManagement::InitialisationError, "Failed to open serial "
            		"port");
        }

        struct termios tty;
        ok = (tcgetattr(serialFd, &tty) == 0);
        if (!ok) {
            REPORT_ERROR(MARTe::ErrorManagement::InitialisationError, "Failed to get serial port "
            		"settings");
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
                    REPORT_ERROR(MARTe::ErrorManagement::InitialisationError, "Baud rate %u not "
                    		"supported", baudRate);
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
                REPORT_ERROR(MARTe::ErrorManagement::InitialisationError, "Failed to set serial "
                		"port settings");
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

    if (signalIdx == 0u) {
        signalAddress = reinterpret_cast<void *>(statusBuffer + kStatusMetadataSize);
    }
    if (signalIdx == 1u) {
        signalAddress = reinterpret_cast<void *>(&data);
    }
    
    return signalAddress != NULL;
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
        ok = (GetNumberOfSignals() == 2u);
        if (!ok) {
            REPORT_ERROR(MARTe::ErrorManagement::InitialisationError, "Exatly 2 signals shall be defined.");
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
		ok = (GetSignalType(1u) == MARTe::SignedInteger32Bit);
		if (!ok) {
			REPORT_ERROR(MARTe::ErrorManagement::InitialisationError, "Second signal shall be of type "
					"int32");
		}
    }

    return ok;
}

bool MotorSTM32::PrepareNextState(const MARTe::char8* const currentStateName,
                                  const MARTe::char8* const nextStateName) {
    return true;
}

bool MotorSTM32::RxSynchronise() {
	MARTe::uint8 dataBuff[kCommandSize];
	// Set command ID (read status).
	dataBuff[0] = 253;
	// Set device ID.
	dataBuff[1] = 0;
    /*int ret = */write(serialFd, dataBuff, sizeof(dataBuff));

    /*ret =*/ read(serialFd, statusBuffer, statusBufferSize);

    return true;
/*
    static MARTe::uint32 counter = 0;
    
    //TODO
    while (1) {
    	const MARTe::uint32 kBufferSize = 256;
    	MARTe::uint8 temp_rx_buffer[kBufferSize] ; //temp_rx_buffer[STM32_BUFSIZE];
        
        // Blocking read
        int ret = read(serialFd, temp_rx_buffer, sizeof(temp_rx_buffer));

        if (ret < 0) {
            // To do: fix problem where the transfer of the updated rx_signals is skipped by this continue
            rx_signals.read_error_count++;
            continue;
        }
        
        uint32 nbytes = static_cast<uint32>(ret);
        rx_signals.received_byte_count += nbytes;
        
        uint32 nbytes_queued = rx_buffer.queue(temp_rx_buffer, nbytes);
        rx_signals.discarded_byte_count += (nbytes - nbytes_queued);

        // The rx_buffer could contain at most STM32_BUFSIZE / DataFrame::RX_FRAME_SIZE messages for 
        // processing. In principle there should be no more than one message available, since we 
        // process received bytes immediately. However, if this thread were starved of CPU time, 
        // this would not be the case, and we would need to process multiple buffered messages to
        // catch up - hence this loop
        bool new_frames_received = false;
        for (uint32 frame = 0; frame < kBufferSize / DataFrame::RX_FRAME_SIZE; frame++) {
            // Sanitise the rx_buffer - delete any leading rogue bytes that cannot be part of an STM32
            // Rx data frame
           rx_signals.discarded_byte_count += DataFrame::SanitiseRxBuffer(rx_buffer);

            //DataFrame::RxDataFrame dataframe;
            //printf("Checkinng");
            if (DataFrame::GetNextRxDataFrame(rx_buffer, rx_signals.dataframe)) {
                rx_signals.message_count++;
                rx_signals.message_rx_time = HighResolutionTimer::Counter();
                new_frames_received = true;
            } else {
                break;
            }
        }

        if (new_frames_received) {
            rx_signals.rx_buffer_occupancy = rx_buffer.count();
            counter++;
            return true;
        }
    }*/
}

bool MotorSTM32::TxSynchronise() {
	return true;//TODO
	MARTe::uint8 dataBuff[kCommandSize];
	// Set command ID.
	dataBuff[0] = 17;
	// Set device ID.
	dataBuff[1] = 0;
	serializeInt32(data, dataBuff + 2);
    /*int ret = */write(serialFd, dataBuff, sizeof(dataBuff));

	MARTe::uint8 responseBuff[kCommandResponseSize];
    /*ret =*/ read(serialFd, responseBuff, sizeof(responseBuff));

    return true;
}

CLASS_REGISTER(MotorSTM32, "1.0");

} // namespace MFI
