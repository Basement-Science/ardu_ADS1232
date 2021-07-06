# ADS1232 library
Arduino Library for the 24bit differential ADC [ADS1232](https://www.ti.com/lit/ds/symlink/ads1232.pdf). Fully featured and class-based. Easy to use.

This library supports both **polling** and **interrupt**-based _(recommended)_ communication. It implements all required logic to use **ALL features** the IC is capable of, as well as value mapping functions that make the output data easy to use.

It even supports communication with multiple ADS1232 ICs.

The ADS123**4** (4-channel variant) is NOT supported, although it shouldn't be too hard to add support for it.

Classical Arduino boards are supported, such as those based on Atmega 328.