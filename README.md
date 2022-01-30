# Project-EDI
Design of a distance scanner by infrared 
- Based on Sharp IR sensor GP2Y0A02.
- Capable of performing distance measurements obstacles in a certain direction.
- Generate a sweep to extract a map of the environment.
- Generate an alarm signal in the presence of an obstacle or optionally a message previously recorded audio
- Control the operation through a serial terminal or optionally from a Smartphone with Bluetooth connection, or via infrared remote control
- Temperature measurement and display environment

## Manual mode or automatic sweep
Selectable by a button or jamper called "Modo"

## Manual mode. Motor position control using the Key 1 and Key 2 pushbuttons
- Each press causes a 10 degree turn to the left and right respectively.
- When the desired position is reached, simply act on the button ISP (Start /Stop) to perform a measurement. The distance measurement (in cm) in this case will be shown on the display of the Mini DK2 with each press.
- _Optional_: The functionality of Key 1 and Key 2 can be implemented through a infrared remote control through two keys acting the same shape. Two other keys will do the functionality of the ISP and Mode buttons.

## Automatic scanning mode.
The scanner will sweep (0 degress - 360 degrees - 0 degrees) continuously until the mode is changed or the ISP (Stop) button is pressed, taking measurements with a certain resolution (eg 10 degrees) every 0.5 seconds, displaying them on the LCD and/or sending them in ASCII through the serial interface.

## Obstacle detection at a configurable distance (20 to 150 cm)
- Acoustic warning of the presence of the obstacle.
- Variable frequency tone depending on the programmed detection threshold: $Frequency = 5000 - Threshold(cm) x 10 [Hz] $
- _Optional_: Audio message previously recorded in memory ("_Obstacle_")

## Specifications
- Through the serial interface (UART0), an options menu will be designed that allows configuring the operating mode, modifying some control parameter and viewing the measurements obtained in ASCII format using a serial terminal program.
- _Optional_: Wireless control of the system via Bluetooth through an external commercial module (HC06) connected to a serial port and a computer with bluetooth connection, or a Smartphone with a Bluetooth terminal application (many free on GooglePlay).

## External temperature measurement and display every 2 sec
- Digital sensor DS1621