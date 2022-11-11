.. _can-counter-sample:

Controller Area Network (CAN) Counter
#####################################

Overview
********

This sample demonstrates how to use the Controller Area Network (CAN) API.
Messages with standard and extended identifiers are sent over the bus.
Messages are received using message-queues and work-queues.
Reception is indicated by blinking the LED (if present) and output of
received counter values to the console.

Building and Running
********************

In loopback mode, the board receives its own messages. This could be used for
standalone testing.

The LED output pin is defined in the board's devicetree.

The sample can be built and executed for boards with a SoC that have an
integrated CAN controller or for boards with a SoC that has been augmented
with a stand alone CAN controller.

Integrated CAN controller
=========================

For the NXP TWR-KE18F board:

.. zephyr-app-commands::
   :zephyr-app: samples/drivers/can/counter
   :board: twr_ke18f
   :goals: build flash

Stand alone CAN controller
==========================

For the nrf52dk_nrf52832 board combined with the DFRobot CAN bus V2.0 shield that
provides the MCP2515 CAN controller:

.. zephyr-app-commands::
   :zephyr-app: samples/drivers/can/counter
   :board: nrf52dk_nrf52832
   :shield: dfrobot_can_bus_v2_0
   :goals: build flash

CAN FD
==========================

For the MIMXRT1060-EVK board:

.. zephyr-app-commands::
   :zephyr-app: samples/drivers/can/counter
   :board: mimxrt1060_evk
   :goals: build flash

An additional overlay file is provided in the sample which reconfigures the default CAN output from FlexCAN2 to
CANFD/FlexCAN3. Also, the output pins are swapped from FlexCAN2 to the FlexCAN3.

Sample output
=============

.. code-block:: console

   Change LED filter ID: 0
   Finished init.
   Counter filter id: 4

   uart:~$ Counter received: 0
   Counter received: 1
   Counter received: 2
   Counter received: 3

.. note:: The values shown above might differ.
