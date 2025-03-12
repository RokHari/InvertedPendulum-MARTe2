# Design Proposal

## Setup

Edukit rotary inverted pendulum.

Components:
  - Stepper motor for rotating the pendulum base.
  - Encoder for reading the position of the pendulum.
  - Nucleo expansion card (X-NUCLEO-IHM01A1) with a L6474PD motor driver. Motor and encoder are connected to the expansion card.
  - STM32F041 Nucleo board on which the expansion card is installed.
  - A computer running a MARTe2 application connected via a USB to the Nucleo board. (Not part of the Edukit).

## Existing Solutions

1) UCLA's Edukit comes with a program running on STM32 that is already capable of balancing the pendulum.
2) A MARTe2 port of the UCLA solution.

## Goal

Existing MARTe2 solution simply moved part of UCLA's code into a single custom GAM. To better demonstrate the use and functionalities provided by MARTe2 a new solution is needed.

## New Solution

This section described not only the proposed solution, but also the thought process behind it.

### DataSource

Since a general DataSource for controlling an L6474 motor driver connected to STM32 does not yet exist, a new DataSource must be written.

First, we have to decide what functionality belongs on the SMT32 and what functionality belongs in MARTe2. For example, in UCLA's solution, all the logic resides on the STM32. In the existing MARTe2 solution, majority of the logic was transferred to MARTe2, while some parts were left on the STM32 (configuration of the motor, moving the motor at initialization, waiting for the move to finish...).

If we want to create a general DataSource, which could be used for other projects using similar hardware, we must treat the STM32 as a simple motor controller. It would only expose the functionalities of the motor (move to position, read status, set speed, set acceleration, read position...), while the project specific code would be put into MARTe2.

The existing software drivers provided by ST could help us develop a general DataSource. The ST drivers already have an abstract concept of a motor with specific implementation depending on the motor driver type (in our case, that is L6474). So, in theory, we could expose this common motor interface to our DataSource, which would allow it to communicate with different motors. However, the interface provided by ST does not allow us to move the motor or change its speed if it is already moving. Waiting for the motor to stop in order to change its speed would make it more difficult or even impossible to balance the pendulum. This is most likely why the UCLA code bypasses the high-level motor interface and controls the speed of the motor using the L6474 specific functions. Unfortunately, this means that for every different hardware setup, a custom STM32 code would need to be written.

Proposed functionalities of the DataSource:
  - Support for motor configuration and control at startup. Configuration parameters are defined in MARTe2 configuration file.
  - Optionally support for changing the configuration during runtime via MARTe2 messages.
  - Custom input broker, that reads status (an array of bytes) from the STM32. It is up to the STM32 code to define the number and meaning of bytes, and up to GAMs to interpret these bytes correctly. In our case, these bytes would contain MotorState, MotorPosition, EncoderPosition.
  - Custom output broker, that sends one of multiple available move commands. Examples would be: moveRelative, moveAbsolute, setRunSpeed.

### State Machine

Balancing of the pendulum can be divided into five stages:
  - Startup - move the motor back and forth so that the pendulum "falls" if it is currently balanced.
  - Homing - wait for the pendulum to come to rest and determine the bottom pendulum point.
  - Swing up - swing the pendulum from the bottom point towards the highest position.
  - Balance - keep the pendulum balanced near the highest point.
  - Reset - if the balance is lost, reset the system and start over.

MARTe2 offers RealTimeApplication states, and at first glance, we could implement each stage in its own RealTimeApplication state. The problem however is that transition between states is not deterministic. This is not a problem for transition between startup, homing, and swing up stages. However, for the transition between swing up and balance stages, this might present a problem. If RealTimeApplication state switch takes too long, the pendulum might move too far away from the highest position, and we are no longer able to balance it. The proposed approach is therefore, to have four RealTimeApplication states with the following tasks:
  1) Move the motor to make sure the pendulum is not balanced (startup stage).
  2) Execute the homing procedure (homing stage).
  3) Swing up the pendulum and then balance it (swing up and balance stage).
  4) Reset the system if balance is lost (reset stage).

Since the swing up and the balancing of the pendulum is done within the same RealTimeApplication state, we no longer have to worry about pendulum moving too far away from the highest point before the balancing code starts executing.

Non-real-time states (1, 2, and 4) can be executed at a low frequency (about 1 Hz). While the hard real-time state (3) should run at a much higher frequency (100 Hz or higher), which should make it easier to balance the pendulum.
