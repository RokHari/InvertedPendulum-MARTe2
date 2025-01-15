#include "AdvancedErrorManagement.h"

#include "MotorSTM32Reader.h"

namespace InvertedPendulum {

MotorSTM32Reader::MotorSTM32Reader() : MemoryMapSynchronisedInputBroker() {
    stm32 = NULL;
}

MotorSTM32Reader::~MotorSTM32Reader() {

}

bool MotorSTM32Reader::Init(const MARTe::SignalDirection direction,
							MARTe::DataSourceI& dataSourceIn,
							const MARTe::char8* const functionName,
							void* const gamMemoryAddress) {
    bool ret = MemoryMapSynchronisedInputBroker::Init(direction, dataSourceIn, functionName, gamMemoryAddress);
    if (ret) {
        stm32 = dynamic_cast<MotorSTM32*>(&dataSourceIn);
        ret = (stm32 != NULL);
        if (!ret) {
            REPORT_ERROR(MARTe::ErrorManagement::FatalError, "Failed dynamic_cast from "
            		"DataSourceI* to MotorSTM32*");
        }
    }

    return ret;
}

bool MotorSTM32Reader::Execute() {
    stm32->RxSynchronise();
    
    return MemoryMapSynchronisedInputBroker::Execute();
}

CLASS_REGISTER(MotorSTM32Reader, "1.0");

} // namespace InvertedPendulum
