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
3. Alter the Makefile to correspond to your xCORE hardware board e.g. `sliceKIT Core Board (L16)` for the xCORE General Purpose sliceKIT.
4. Open `main.xc` and modify the various PMU parameters to match your PMU device and testing arrangement.
5. Build the project e.g. by right clicking on it in the Prject Explorer, and selecting "Build Project".
6. Ensure your xCORE hardware is connected by USB, then create a new Run Configuration using the `xCORE Application` type. You may wish to enable `Real-Time Mode` in the XScope tab. You should now be able to run the software.

## Hardware configuration

The software supports multiple configurations, depending on the availablity of a PTP clock, etc. A typical configuration is shown below, where the circle Ethernet port is connected to the PTP master clock, and the square Ethernet port is connected to the PMU under test.

![alt tag](https://raw.githubusercontent.com/stevenblair/pmu-latency-measure/master/xCORE-hardware-config.jpg)
