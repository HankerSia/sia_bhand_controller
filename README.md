# sia_bhand_controller
when i was using the barrett 4 dof hand, i found the required environment is hard to config, especially the workspace of catkin_bhand, and mostly, the barrett's package is not stable, it  crashed frequently, so i wrrite this package, all the package need is you installed the driver software of usb pcan, and when you pluged the pcan, you can veryfy whether the driver is installed successfully through running the command "ls /dev/pcan*", if  it shows pcanusb0 or pcan32 and so on, the package will work normally.
P.S. i used the id_data_msgs for communicating with other ros nodes, so the sia_bhand_controller need this package when building. 

libpcan: http://wiki.ros.org/libpcan

PCAN-Light for LINUX is the easy to use software interface for CAN hardware by PEAK-System.

website: http://p103112.typo3server.info/fileadmin/media/linux/index.htm

The PCAN drivers for LINUX work with Kernel versions 2.4 up to 4.x The complete package is distributed under the GPL.

If you have any questions about the drivers please contact us.

There's a single API (Application Programming Interface) for all CAN interfaces. This simplifies the software development. The drivers are designed to work with CAN hardware by PEAK-System and 100 percent compatible hardware by OEM vendors. Take care when using third-party hardware, since the drivers use special functions.
