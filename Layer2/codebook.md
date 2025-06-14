Fremework : ROS2 Humble Hawksbill   
Surface Hardware 1: Raspberry pi400 running ROS2 Humble on Ubuntu Jammy Desktop   
Surface Hardware 2: WiFi Router   
Communication Interface : Ethernet Cat5   
ROV-WTC Hardware 1: Raspberry Pi3 running ROS2 Humble on Ubuntu Jammy Server   
ROV-WTC Hardware 2 : pca9685 16 servos controller (I2C)  


|Hardware|Surface|ROV_WTC|
|---|---|---|
|Feature|dryside|wetside|
|Workspace|control_ws|execute_ws|
|Package|control|execute|
|Programe|keyboard.cpp|servo.cpp|
|Node|keyboard|servo|
|node in rqt|/keyboard_node|/servo_controller|
|Topic /Keyboard_command|publish|subscribe|
