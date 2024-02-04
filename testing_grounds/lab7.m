%%%%%%%%%%%%%
% ECE 3610
% LAB 7 -- Sensors Milestone
%%%%%%%%%%%%%

%%%%%%%%%%%%%
% This is the first lab where you will work with the robots used in
% your final project. To start, we will work on line detection and
% interfacing with the IR reflectance sensors on the robot.
% As there are only 20 robots to work with, you will be working in groups to
% accomplish this lab.
%
% Deliverables:
% - Develop a formula for error that results in large values when the
% robot is not centered on the line, and near zero when it's aligned.
% - Using what you've learned, implement code that changes the color and
% brightness of an RGB LED dependent on the sign and magnitude of the
% error.
%%%%%%%%%%%%%%

%% 1. CONNECT TO YOUR NANOBOT
%  Remember to replace the first input argument with text corresponding to
%  the correct serial port you found through the Arduino IDE. Note that the
%  clc and clear all will clear your command window and all saved
%  variables!

clc
clear all
nb = nanobot('COM49', 115200, 'serial');

%% 2. Connecting to the reflectance array
% Follow the wiring diagram on Canvas to connect your Arduino to the
% reflectance arrays. Included below is a short key with the connections
% for reference.
%
% Give your reflectance sensor a test using some demo code to verify that
% everything is wired correctly.
% 
% KEY:
% Pololu Array Pin | Arduino Pin
%     1 | V_IN
%     4 | D8 (IR1)
%     9 | D10 (IR2)
%    11 | GND
%    12 | GND
%    14 | +5V
%    18 | D12 (IR4)
%    22 | D11 (IR3)
% JP1-2 | +5V

%Initialize the reflectance sensor with default pins D12, D11, D10, D8
nb.initReflectance();

%Take a single reflectance sensor reading
val = nb.reflectanceRead;

%The sensor values are saved as fields in a structure:
fprintf('one: %.2f, two: %.2f, three: %.2f four: %.2f\n', val.one, val.two, val.three, val.four);

%% 3. How the reflectance array works
% In essence, this reflectance array works using an infrared (IR) LED, a
% phototransistor, and a capacitor to detect how much IR light is able to
% reflect off of the surface beneath the sensor. Prior to a reading, the
% capacitor is charged and stands by. When a reading is taken, the
% capacitor is allowed to discharge at a rate dictated by the
% phototransistor, dependent on the light it recieves. The time it takes
% for the capacitor to discharge to a certain voltage is recorded and
% reported as a measure of the reflectance of the surface below the sensor.
% On your reflectance sensors, this time is measured in microseconds.
% Higher values correspond to less reflective surfaces, and vice versa.

% For more detailed info, https://www.pololu.com/docs/0J13/2 is a good
% resource.

%% 4. Estimating Error 
% How can we quantify how "off-the-line" we are, given the 4 values we
% record from our reflectance array? Since our robots will eventually be
% moving and following lines using line detection, it's important that we
% stay centered.
% One way of approaching this is by using error. An error value can be
% created by a simple mixture of addition and subtraction of signals, but the
% key is that the error should reduce to zero when an optimal condition is
% reached.

% Below is some code to help you experiment with error. Try to make the
% error zero when perfectly centered on the line, and large when not
% aligned. Take note of the maximum error values you tend to see. They'll
% come in handy later.

nb.initReflectance();
while(1)
    val = nb.reflectanceRead();
    error = (2*val.one) + val.two -val.three - (2*val.four);
    fprintf("Error: %d\n", error);
end
%% 5. Line detection
% Your ultimate goal is to implement line detection such that an LED lights up in
% one color corresponding to error in one way, and another color to error
% in the other direction. When implemented correctly, the LED should shift
% colors similarly to how a pair of differential drive motors would correct
% for line deviance. You have been introduced to all of the tools needed to
% do this already, so best of luck! Don't hesitate to ask questions if you
% get stuck!

% Here are some helpful hints to get you started:
% - You'll be using the error formula you developed from above, along with
% linear interpolation to get an RGB intensity value.
% - For the purposes of determining the LED intensity, use your minimum and
% maximum recorded error values.
% - Depending on whether your error is positive or negative, your color
% should change from one to the other.
% - As one color's intensity decreases, the other should increase
% proportionally. This means that when the robot is centered, both colors
% should be at equal brightness.

% General structure should follow this:
% - define error and rgb ranges
% - while loop
%   - perform read
%   - calculate error from read
%   - linearly interpolate error to RGB
%   - modify RGB value to split into two colors that change proportionally
%   - pause for short time (optional)

nb.initReflectance();
nb.initRGB('D4','D3','D2');
% Find min and max error to lerp
errorRange = [-6000, 6000];
rgbRange = [0, 255];
blueVal = 0;
redVal = 0;
while(1)
    val = nb.reflectanceRead();
    error = (2*val.one) + val.two -val.three - (2*val.four);
    fprintf("Error: %d\n", error);
    rawRGB = interp1(errorRange, rgbRange, error);
    redVal = 255 - rawRGB;
    blueVal = rawRGB;
    nb.setRGB(redVal, 0, blueVal);
    pause(0.05);
end

%% 5. DISCONNECT
%  Clears the workspace and command window, then
%  disconnects from the nanobot, freeing up the serial port.

clc
delete(nb);
clear('nb');
clear all

