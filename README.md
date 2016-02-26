# jr3 -- a 6 degree-of-freedom force-torque sensor

Driver for jr3 for Xenomai using RTDM.

Please read before proceeding

# Building
Use command 
make all

# To load driver 
insmod jr3pci-driver.ko

# To unload driver 
rmmod jr3pci


# Additional information 

# Examples
_app.c_ provide comprehensive example how to use the driver 
in user space.

