## Linux kernel driver for Silicon Labs CP2130 USB-SPI Bridge

* datasheet: https://www.silabs.com/Support%20Documents/TechnicalDocs/CP2130.pdf

* for out-of-tree build please refer to https://www.kernel.org/doc/Documentation/kbuild/modules.txt

* sysfs API
  * irq_poll_interval: specifies the IRQ/GPIO poll interval in us.
  * channel_pdata: binary load of platform data for SPI driver. The create_pdata.c
                 program gives an example how to create binary platform data
                 for the mcp251x.
  * channel_config: attach SPI driver to certain channel, e. g. to
                  plug the mcp251x CAN driver use
                  `echo -n 0,2,6,0,0,0,1,0,0,0,0,mcp2515 > /sys/.../channel_config`
                  or
                  `echo -n 0,2,-1,1,1,1,0,0,0,0,0,spidev > /sys/.../channel_config` and
                  `echo -n 1,2,-1,1,1,1,0,0,0,0,0,spidev > /sys/.../channel_config`
  `               for 2 general purpose spidev without IRQ support.
                  For the values to be entered in the columns please
                  refer to the CP2130 datasheet.
  * otp_rom: you can read the current OTP ROM settings and their lock
           status or write a new OTP ROM configuration, please make
           sure that you do the right thing, OTP means one-time-programmable!

* GPIO chip: the driver implements a GPIO chip for all GPIOs of the CP2130,
             they can be accessed from userspace using the sysfs GPIO API
             or directly from another driver.
