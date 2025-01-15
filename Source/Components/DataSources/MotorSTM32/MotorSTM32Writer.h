#ifndef SOURCE_COMPONENTS_DATASOURCES_MOTORSTM32_MOTORSTM32WRITER_H_
#define SOURCE_COMPONENTS_DATASOURCES_MOTORSTM32_MOTORSTM32WRITER_H_

#include "MemoryMapSynchronisedOutputBroker.h"

#include "MotorSTM32.h"

namespace InvertedPendulum {

class MotorSTM32Writer : public MARTe::MemoryMapSynchronisedOutputBroker {
 public:

    CLASS_REGISTER_DECLARATION();
    
    MotorSTM32Writer();

    virtual ~MotorSTM32Writer();

    virtual bool Init(const MARTe::SignalDirection direction,
                      MARTe::DataSourceI& dataSourceIn,
                      const MARTe::char8* const functionName,
                      void* const gamMemoryAddress);
    
    virtual bool Execute();

 private:
    MotorSTM32* stm32;
};

} // namespace InvertedPendulum

#endif // SOURCE_COMPONENTS_DATASOURCES_MOTORSTM32_MOTORSTM32WRITER_H_
