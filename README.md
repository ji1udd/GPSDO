# GPSDO
## GPSDO for Hermes-Lite2 and a measuring instrument    
[Youtube video](https://youtu.be/AfCz8ohrafM)  

<img src="Doc/GPSDO_front_view.jpg" width="400px">  

GPSDO using a cheap used OCXO purchased from Aliexpress. It has 10MHz square wave output that can be directly connected to the external clock input of Hermes-Lite2 and 10MHz sine wave output (-10 dBm @ 50 ohm) that can be connected to a measuring instrument such as a spectrum analyzer.

<img src="Doc/GPSDO_rear_view.jpg" width="400px">  

The 10MHz input SMA connector is located on the back panel of my HL2+. Also, my GPSDO has 10MHz output SMA connector not only on the front panel but also on the back panel. Therefore, these back connectors are connected using a coaxial cable.

<img src="Doc/GPSDO_internal_view.jpg" width="400px">  

## Operation
1. After powering on, wait for the OCXO to warm up to the preset temperature. The default temperature setting is 30 degree C.
2. Next, wait for 1PPS signal from GPS.
3. OCXO frequency control is performed using 1PPS signal. This state is called normal state.
4. During normal state, display mode can be switched by pressing MODE/SAVE button shortly. Display modes are UTC display, frequency integration error display (unit: 0.1Hz) for monitor, latitude and longitude display.
5. During normal state, by pressing MODE/SAVE button longly, the current OCXO temperature minus 5 degree C is saved to STM32 non-volatile memory as the preset temperature. Also the current OCXO control parameter (PWM value) is saved as the initial PWM value. By saving the normal state OCXO temperature and control parameters, it will take less time to stabilize the frequency after cold start. On the other hand, by pressing and holding the MODE/SAVE button while turning on the power, the preset values stored in STM32 non-volatile memory is ignored.