# COVID-19 Detector and Transmitter Bracelet (DeTra)
The smart bracelet detects symptoms of COVID-19/ SARS-CoV-2. It can wirelessly transmit the data to help centers or warn the person automatically

![](Images/DeTra.png)

## Introduction
The COVID-19 Detector and Transmitter Bracelet is designed to detect possible COVID-19 outbreaks in crowded areas. In order to control the virus’ spread, data can either be transmitted to medical staff or advise the bracelet carrier to consult a doctor.

The low-cost architecture of the device and its use of low-power wide-area networks (LPWAN) makes it ideal for areas with limited or no internet coverage.

By using different sensors, DeTra measures vital signs such as heart rate, body temperature, and blood oxygen level, which indicate infection with COVID-19 when varying from their status quo.

The real-time data transmission occurs via long-range radio waves using an integrated antenna.

Additionally, other health problems can also be monitored using DeTra, e.g. by indicating an upcoming diabetic coma of diabetes type 1 patients.
## Housing of DeTra
To protect the measuring device from foreign bodies and contact as well as from water and
moisture, a CAD model was developed for 3D printing.
The BME680 sensor is located on the front of the housing. Trough this the acetone from the air we breathe can be detected.

The sensor is fixed by nuts on the top of the housing. These two nuts also function as a touch sensor.
Pins to control the measuring device.

On the lower side of the housing the MAX30105 sensor is located. This allows the sensor to lie directly on the skin and measure the specific parameters pulse, oxygen saturation and body temperature.

The measuring device can be charged by a Micro-USB. Care was taken to keep the design of the measuring device and the arrangement of the hardware as compact as possible. The design is therefore space-saving 

![](Images/inner_life.png)

## Monitoring
In order to visualize the data, Node-Red receives the organized data from “The Things Network” and displays it in a dashboard for easy analysis by e.g. medical authorities.

Node-Red and “The Things Network” communicate via the MQTT protocol. For further processing, the format of the messages sent by “The Things Network” are converted from JSON strings into a JSON object.

The dashboard is divided into four columns: heartbeat, Spo2, body temperature, and acetone. The first row of each group shows the history of the measured values. The second row visualizes the current value. To get a better overview of the values, the last row visualizes the average value, as well as the maximum and minimum value, which are calculated and filtered out by DeTra.

If DeTra detected a possible COVID-19 infection, a notification on the right top of the Dashboard will pop up.


![](Images/Dashboard.png)

## Repository Contents
* **/src** - Source files for the library (.cpp, .h).
* **/examples** - Example sketches for the library (.ino). Run these from the Arduino IDE.
* **/Images** - Images for the repository.
