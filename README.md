# Real-time measurement of Phasor Measurement Unit (PMU) reporting latency

This software enables PMU reporting latency to be measured accurately and conveniently, whilst requiring only relatively inexpensive hardware.

## Hardware requirements

1. A compatible xCORE board with two Ethernet ports, depending on the required configuration. For example, the xCORE General Purpose sliceKIT (https://www.xmos.com/support/boards?product=15825) definitely works, and the newer xCORE-200 General Purpose sliceKIT (https://www.xmos.com/support/boards?product=19745) should work. These can be obtained relatively easy, costing about £130/$150.

## Software requirements

1. Download xTIMEcomposer from the XMOS web page and install: https://www.xmos.com/support/tools. This is free, but you do need to create an account.
2. Clone this repository.

## Usage instructions

1. Open xTIMEcomposer, with the Workspace at the root of the cloned repository.
2. Add the main project to the Project Explorer: New > xTIMEcomposer Project Based on Existing Application or Module > Select `pmu_latency_measure_main`.
3. 
