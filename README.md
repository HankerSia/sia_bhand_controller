# sia_bhand_controller
when i was using the barrett 4 dof hand, i found the required environment is hard to config, especially the workspace of catkin_bhand, and mostly, the barrett's package is not stable, it  crashed frequently, so i wrrite this package, all the package need is you installed the driver software of usb pcan, and when you pluged the pcan, you can veryfy whether the driver is installed successfully through running the command "ls /dev/pcan*", if  it shows pcanusb0 or pcan32 and so on, the package will work normally.
P.S. i used the id_data_msgs for communicating with other ros nodes, so the sia_bhand_controller need this package when building. 

libpcan: http://wiki.ros.org/libpcan
