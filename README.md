# ROS interface for a single Schunk FT Sensor with CANbus implementing SocketCAN
##### Tested with: Ubuntu 16.04, ROS kinetic, PEAK-System PCAN-USB, SCHUNK-30064043 FTM 115
#
### Setup
It is implied that you have already created a catkin workspace, cloned this project, installed all required dependencies and built the project.
- Connect the sensor to PC via USB CAN card
- Initialize the Network Interface Card:
    ```sh
    $ sudo ip link set <CAN_DEVICE_NAME> up type can bitrate <BITRATE>
    ```
    Here `<CAN_DEVICE_NAME>` should be replaced by the corresponding name assigned to your CAN card by the system (if a single CAN card is connected to PC, the device name is usually **can0**, otherwise can be **can0, can1, can2...**).
    In order to list all available network interface cards use command:
     ```sh
    $ ifconfig -a
    ```
    `<BITRATE>` should be replaced by the baudrate of your sensor's CAN card.
#
### Launch
In order to launch the sensor node use following command:
```sh
$ roslaunch schunk_ft_sensor schunk_ft_sensor_node.launch can_device:=<CAN_DEVICE_NAME> can_node_id:=<NODE_ID>
```
As described before, `<CAN_DEVICE_NAME>` has to be replaced by the proper device name, furthermore, `<NODE_ID>` has to be replaced by the CAN node ID of your sensor's CAN card.

##### Further available arguments:
- **node_name**
A name for the ROS node (default is ***schunk_ft_sensor***). Will be used as namespace for the topics and services.
- **calibration**
Number of the calibration slot for the values of calibration matrix (refer to the sensor's manual, default is ***0***, allowed values are ***0...15***).
- **sample_count**
Number of data samples read before calculating the average value and publishing to the ROS topic (default value is ***50***, allowed values are ***1...65536***). The publishing rate of the ROS topic is inversely proportional to the value of this parameter.
- **silence_limit**
Sensor's "silence" duration limit in seconds (default is ***0.1***). If sensor does not send response to a data request within specified time, an error will be thrown with corresponding notification via ROS topic, sensor reading will be terminated.
#
### Runtime
After a successful launch you will get a ROS node with a name specified by the ***node_name*** argument in previous step and some topics and services in the corresponding namespace.
##### ROS Nodes
- **/<node_name>**
##### ROS Topics
- **/<node_name>/sensor_data**
Sensor data will be published on this topic. The publishing rate is limited by the PC's computational power and the used baudrate, as well as divided by sample_count parameter. As such, publishing is done without any specific rate or "as fast as possible".
- **/<node_name>/failure**
Any critical failure will be published on this topic followed by **interface shutdown**.
##### ROS Services
- **/<node_name>/reset_bias**
Calling this service will treat current measurement values of the sensor as bias and set measurement values to zero until they are changed. Does not require any parameter.