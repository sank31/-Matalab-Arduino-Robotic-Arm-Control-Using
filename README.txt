For inquiries and updates about this project, you can email me at

- Don't forget to change 'ENC_TICKS_PER_REV' in the Arduino code to reflect your encoder's ticks per revolution.
- Don't forget to change the name of the virtual serial port in the 'Robotic_Arm_GUI_OpeningFcn()'  function located in the 'Robotic_Arm_GUI.m' file before running the code, to reflect the virtual port name assigned to your Arduino board by the operating system.
- Load the Arduino sketch to your Arduino board and then run the 'Robotic_Arm_GUI.m' file in MATLAB to open the GUI.
- You can tests the MATLAB code without the robotic arm by just connecting an Arduino Board to the companion computer. The MATLAB code will still run without the robotic arm.
- Before running the system, the DC motor (joint 1) must be positioned at zero degrees.
- To move any motor 10 degrees at a time:
  * Open the Arduino IDE's serial monitor (you can use Putty or Tera Term as well).
  * Type the motor number you want to move (1, 2, 3 or 4)
  * Type '+' (without the '') to rotate +10 degrees
  * Type '-'to rotate -10 degrees
  * Type another motor number and repeat to move the new selected motor
- You can connect a UART to USB converter to pins 3 and 6 (3 is RX, 6 is TX) in the Arduino board to receive debugging messages from the embedded controller in another serial terminal emulator.

