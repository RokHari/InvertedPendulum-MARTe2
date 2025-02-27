OBJSX =

SPB = StartupGAM.x \
      HomingGAM.x \
      ResetGAM.x \
      BalanceGAM.x

ROOT_DIR=../../..
PACKAGE=Components

MAKEDEFAULTDIR=$(MARTe2_DIR)/MakeDefaults

include $(MAKEDEFAULTDIR)/MakeStdLibDefs.$(TARGET)

LIBRARIES_STATIC+=$(BUILD_DIR)/StartupGAM/StartupGAM$(LIBEXT)

all: $(OBJS)  $(SUBPROJ)  \
    $(BUILD_DIR)/GAMs$(LIBEXT) \
    $(BUILD_DIR)/GAMs$(DLLEXT)
	echo  $(OBJS)

include $(MAKEDEFAULTDIR)/MakeStdLibRules.$(TARGET)
