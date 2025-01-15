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
    bool ret = MemoryMapSynchronisedOutputBroker::Init(direction, dataSourceIn, functionName, gamMemoryAddress);
    if (ret) {
        stm32 = dynamic_cast<MotorSTM32*>(&dataSourceIn);
        ret = (stm32 != NULL);
        if (!ret) {
            REPORT_ERROR(MARTe::ErrorManagement::FatalError, "Failed dynamic_cast from DataSourceI* to MotorSTM32*");
        }
    }

    return ret;
}

bool MotorSTM32Writer::Execute() {
    bool ret = MemoryMapSynchronisedOutputBroker::Execute();

    stm32->TxSynchronise();

    return ret;
}

CLASS_REGISTER(MotorSTM32Writer, "1.0");

} // namespace InvertedPendulum
