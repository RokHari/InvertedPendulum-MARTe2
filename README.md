# MARTe2 Inverted Pendulum

Repository containing MARTe2 code for balancing a rotary inverted pendulum edukit.

## Description

Project for demonstrating how to properly use MARTe2 framework for real time control applications.

## Build

Clone and build `MARTe2` and `MARTe2-components` and set the environmental variables.

Example for Linux:

```
$ git clone https://github.com/aneto0/MARTe2.git
$ cd MARTe2
$ make -f Makefile.x86-linux
$ export MARTe2_DIR=$(pwd)
```

```
$ git clone https://github.com/aneto0/MARTe2-components.git
$ cd MARTe2-components
$ make -f Makefile.x86-linux
$ export MARTe2_Components_DIR=$(pwd)
```

Clone and build this repositorty. Linux build command in the root directory of the cloned project:

```
$ make -f Makefile.x86-linux
```

## Run

Upload the corresponding STM32 code to the edukit. TODO: link to a "permanent" InvertedPendulum-STM32 repository.

Connect the STM32 to a computer with the USB cable.

Set the correct device file (`Port` configuration of the `MotorSTM32` data source in `Configurations/Pendulum.cfg`).

Navigate to `Startup/` directory withing the project and execute the following command:

`sudo -E ./Main.sh -l RealTimeLoader -f ../Configurations/Pendulum.cfg -m StateMachine::START`
