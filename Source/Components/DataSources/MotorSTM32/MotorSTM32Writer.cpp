#include "AdvancedErrorManagement.h"

#include "MotorSTM32Writer.h"

namespace InvertedPendulum {

MotorSTM32Writer::MotorSTM32Writer() : MemoryMapSynchronisedOutputBroker() {
    stm32 = NULL;
}

MotorSTM32Writer::~MotorSTM32Writer() {

}

bool MotorSTM32Writer::Init(const MARTe::SignalDirection direction,
							MARTe::DataSourceI& dataSourceIn,
							const MARTe::char8* const functionName,
							void* const gamMemoryAddress) {
    bool ok = MemoryMapSynchronisedOutputBroker::Init(direction, dataSourceIn, functionName, gamMemoryAddress);
    if (ok) {
        stm32 = dynamic_cast<MotorSTM32*>(&dataSourceIn);
        ok = (stm32 != NULL);
        if (!ok) {
            REPORT_ERROR(MARTe::ErrorManagement::FatalError, "Failed dynamic_cast from DataSourceI* to MotorSTM32*");
        }
    }

    return ok;
}

bool MotorSTM32Writer::Execute() {
    bool ok = MemoryMapSynchronisedOutputBroker::Execute();

    if (ok) {
        ok = stm32->TxSynchronise();
    }

    return ok;
}

CLASS_REGISTER(MotorSTM32Writer, "1.0");

} // namespace InvertedPendulum
