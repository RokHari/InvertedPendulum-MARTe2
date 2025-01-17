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
    bool ok = MemoryMapSynchronisedInputBroker::Init(direction, dataSourceIn, functionName, gamMemoryAddress);
    if (ok) {
        stm32 = dynamic_cast<MotorSTM32*>(&dataSourceIn);
        ok = (stm32 != NULL);
        if (!ok) {
            REPORT_ERROR(MARTe::ErrorManagement::FatalError, "Failed dynamic_cast from "
            		"DataSourceI* to MotorSTM32*");
        }
    }

    return ok;
}

bool MotorSTM32Reader::Execute() {
    bool ok = stm32->RxSynchronise();

    if (ok) {
        ok = MemoryMapSynchronisedInputBroker::Execute();
    }
    
    return ok;
}

CLASS_REGISTER(MotorSTM32Reader, "1.0");

} // namespace InvertedPendulum
