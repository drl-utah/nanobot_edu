%%%%%%%%%%%%%
% ECE 3610
% LAB 2 -- Toolchain Intro 2 -- Experimenting with Other Peripherals
%%%%%%%%%%%%%

%%%%%%%%%%%%%
% In this lab, we will be learning how to connect external components to
% our microcontroller, such as buttons, sensors, and motors. We will also
% continue to explore the nanobot class and its functions as detailed in
% nanobot.m.
%
% Deliverables:
% Develop a MATLAB script (this one) which successfully:
% - Moves a DC motor
% - Moves a servo motor
% - Shows live plotting of of TMP36 temperature sensor data
% - Finds the mean and standard deviation of the temperature sensor data
% - Toggles an LED using a pushbutton
%
% Possible Extensions:
% - Make the LED blink when the temperature increases about room
% temperature, and go steady when it returns
% - Move the servo in 10-degree increments every time the pushbutton is
% depressed. Return to home position when the temperature reading
% increases.
%%%%%%%%%%%%%%

%% 1. CONNECT TO YOUR NANOBOT
%  Remember to replace the first input argument with text corresponding to
%  the correct serial port you found through the Arduino IDE or port_detector.m. Note that the
%  clc and clear all will clear your command window and all saved
%  variables!

clc
clear all
nb = nanobot('COM7', 115200, 'serial');

%% 2. Connecting and running the DC motor
%  First, locate where the Motor 1 output slots are on your carrier board.
%  Refer to the pinout diagram, the pins should be marked as M1+ and M1-.
%  Take your DC motor and the leads to the output. The polarity does not
%  matter (we will learn why later).
%
%  There is an ON/OFF switch on your carrier board that enables the separate
%  battery pack to drive the motors. Set the switch to the ON position so we
%  can supply power to the motors.
%
%  Is is VERY IMPORTANT that we use the motor carrier and battery to drive
%  the motors. The Arduino can only output a very small amount of current
%  through its pins, so we want the motor carrier to do the heavy lifting
%  while the Arduino supplies the control signal. Using the Arduino itself
%  to power the motor could potentially damage your board and render it
%  nonfunctional.
%
%  Next, we want to drive the motor. Find the appropriate code in
%  nanobot_demo.m to make our DC motor move and copy it here:




%% 3. Connecting and running the servo motor
%  Similarly to the DC motor hookup, locate the servo 1 output pins. Note,
%  if you check the underside of the carrier board, you will see that the
%  one of the the servo 1 output pins has a white stripe over it. This pin
%  corresponds to the brown wire connection for the servo motor.
% 
%  Connect the servo motor to servo output 1, making sure to match the
%  brown wire to the white stripe pin.
%
%  Now find the appropriate code in nanobot_demo.m and copy it here to make
%  our servo motor move. Remember to set the power switch on the carrier to
%  ON if not already set there.




%% 4. Analog temperature sensing with the TMP36
%  For this part, you will need your breadboard and a TMP36 temperature
%  sensor. If you orient the TMP36 such that the flat face is pointing
%  downward and the legs are pointed away from you, the legs from left to
%  right are 1. +Vs, 2. Vout, and 3. GND.
%    _______
%   /       \
%  | 1  2  3 |
%  -----------
%    |  |  |
%
%  Using your breadboard, connect +Vs to VCC on the Arduino, GND to GND on
%  the Arduino, and Vout to pin A1 on the Arduino. Take a look at the
%  Arduino Nano 33 IoT pinout if you need to determine where to connect.
%
%  To test the TMP36 sensor, find a section of code in nanobot_demo.m to
%  help you with live plotting an analog pin. Make sure the code specifies
%  the correct pin you have connected to the Arduino, and that said pin is
%  set to be an analog input!
%
%  Test the sensor's response to temperature change by pinching the sensor
%  with your fingers. The average plot value should increase, but not by
%  much. When you let it go, it should eventually return to its starting
%  value.




%% 5. Gathering data and statistics
%  For this section, find the appropriate section of code from
%  nanobot_demo.m to help you gather a range of analog temperature sensor
%  data. Modify this code to find the mean value and standard deviation of 
%  the data and report it in the command window.




%% 6. Reading digital values with a pushbutton
%  Grab your pushbutton and place it into the breadboard so that its pins
%  enter different rows. Connect one pin to GND on the Arduino, and connect
%  the other to Digital Pin 10. Grab code from the DIGITAL READ section of
%  nanobot_demo.m, modify the code as necessary to select the correct pin
%  for digital reading.
%
%  Now write code to turn on the onboard LED when the button is pressed.
%
%  HINT: if you wrap an if statement in a while(true) loop, the condition
%  will be checked continuously. You can also use tic and toc to have the
%  loop run continuously for a finite period of time.




%% 7. EXTENSION (optional)
%  Try these following tasks if you're up to the challenge:
% - Make the LED blink when the temperature increases about room
% temperature, and go steady when it returns.
% - Move the servo in 10-degree increments every time the pushbutton is
% depressed. Return to home position when the temperature reading
% increases.

%% X. DISCONNECT
%  Clears the workspace and command window, then
%  disconnects from the nanobot, freeing up the serial port.

clc
delete(nb);
clear('nb');
clear all