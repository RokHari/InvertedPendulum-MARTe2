#ifndef SOURCE_COMPONENTS_DATASOURCES_MOTORSTM32_MOTORSTM32READER_H_
#define SOURCE_COMPONENTS_DATASOURCES_MOTORSTM32_MOTORSTM32READER_H_

#include "MemoryMapSynchronisedInputBroker.h"

#include "MotorSTM32.h"

namespace InvertedPendulum {

class MotorSTM32Reader : public MARTe::MemoryMapSynchronisedInputBroker {
 public:
    CLASS_REGISTER_DECLARATION();
    
    MotorSTM32Reader();

    virtual ~MotorSTM32Reader();

    virtual bool Init(const MARTe::SignalDirection direction,
                      MARTe::DataSourceI& dataSourceIn,
                      const MARTe::char8* const functionName,
                      void* const gamMemoryAddress);
    
    virtual bool Execute();

 private:
    MotorSTM32* stm32;
};

} // namespace InvertedPendulum

#endif // SOURCE_COMPONENTS_DATASOURCES_MOTORSTM32_MOTORSTM32READER_H_
