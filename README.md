# OBD2 DATALOGGER HTTP

This project consist of a OBD2 data logger with a .CSV file as a output, requesting via OBD2 info at real time for a ECU Vehicle and sending the file using a HTTP request with a Wi-Fi network conected.

## How to use

CAN Obd2 scanner

Author: Gabriel Bittencourt de Souza
Git_Hub:bittsouza
E-mail: gabrielbittsouza@gmail.com
   

 The following code send requests from ECU using CAN BUS. The data qnted is sent by ESP32 from ECU and the
 data receiver is processed to get Human Readable information from the Vehicle. 
 The current information acquired is:
 1) RPM - Revolutions per minute from the motor
 2) SPD - Speed of the vehicle in Km/h
 3) INT - Intake temperature in ºC
 4) TPS - Throttle position in %
 5) FUL - Fuel in the tank in %
 6) ODO - Odometer in Km
 7) LBD - Lambda sensor λ
 8) RTM - Run time engine since start
 
 Definition of DATA[]
 Frame DATA[0]
 Frame DATA[1]
 Frame DATA[2] == PID ANSWER
 Frame DATA[3] == A
 Frame DATA[4] == B
 Frame DATA[5] == C
 
All PID's, Baud rate and ID request tested in VW Polo 2018.
 
- A example log can be seen at:
    -https://datazap.me/u/bittsouza/rolling-0-88-0807?log=0&data=0-1-2

For more information of the obd2 message and PID's access:

- https://www.csselectronics.com/screen/page/obd-ii-pid-examples/language/en
- https://en.wikipedia.org/wiki/OBD-II_PIDs


### Configure the project

Open the project configuration menu (`idf.py menuconfig`). 

In the `Main file`:

* Set the Wi-Fi configuration.
    * Set `WiFi SSID`.
    * Set `WiFi Password`.
    * Set `PID's for request`.
    * Set `HTTP server IP`.

Optional: If you need, change the other options according to your requirements.

### Build and Flash

Build the project and flash it to the board, then run the monitor tool to view the serial output:

Run `idf.py -p PORT flash monitor` to build, flash and monitor the project.

(To exit the serial monitor, type ``Ctrl-]``.)

See the Getting Started Guide for all the steps to configure and use the ESP-IDF to build projects.

* [ESP-IDF Getting Started Guide on ESP32](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/index.html)
* [ESP-IDF Getting Started Guide on ESP32-S2](https://docs.espressif.com/projects/esp-idf/en/latest/esp32s2/get-started/index.html)
* [ESP-IDF Getting Started Guide on ESP32-C3](https://docs.espressif.com/projects/esp-idf/en/latest/esp32c3/get-started/index.html)

## Example Output
Note that the output, in particular the order of the output, may vary depending on the environment.

# esp_send_files
# OBD2_DATALOGGER
