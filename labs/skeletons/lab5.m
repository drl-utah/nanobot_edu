%%%%%%%%%%%%%
% ECE 3610
% LAB 5 -- Inertial Measurement Lab
%%%%%%%%%%%%%

%%%%%%%%%%%%%
% Tilt detection using an IMU (inertial measurement unit) is critical to a 
% huge number of products - the Wii-mote, quadrotors, and commercial 
% aircraft, to name a few. The three-axis MEMS accelerometer is a critical 
% component of the IMU. In this lab, we will show how the measurements 
% from your IMU can be used to calculate and visualize tilt angle in real 
% time.
% 
% Deliverables:
% - Show a 3D block which successfully tracks your breadboard tilt in an
% intuitive way
% - Demonstrate the 3D tilt matching game
%
% Extensions:
% - Change the color of the 3d block being displayed every time you press a
% pushbutton.
% - Change the color of the 3d block being displayed every time you tap 
% the accelerometer.

%%%%%%%%%%%%%%

%% 1. CONNECT TO YOUR NANOBOT
%  Remember to replace the first input argument with text corresponding to
%  the correct serial port you found through the Arduino IDE. Note that the
%  clc and clear all will clear your command window and all saved
%  variables!

clc
clear all
nb = nanobot('?', 115200, 'serial');

%% 2. Testing the onboard IMU
% Your Arduino board has a built-in accelerometer that allows tilt
% orientation to be determined. Turn on and connect to your board, and test
% your IMU using some of the nanobot_demo.m code.
%
% HINT; 'accel' stands for accelerometer or IMU

%% 3. Visualizing IMU tilt
% The following code blocks will set up a 3D block that will rotate as you change
% your tilt of the board, once you figure out all of the '?' areas. In
% addition, a game has been set up in which you must match a random
% orientation.

%% Initialize the cube (RUN ME)
xc=0; yc=0; zc=0;    % cube center coordinates
L=2;                 % cube size (length of an edge)
alpha=0.8;             % transparency (max=1=opaque)

X = [0 0 0 0 0 1; 1 0 1 1 1 1; 1 0 1 1 1 1; 0 0 0 0 0 1];
Y = [0 0 0 0 1 0; 0 1 0 0 1 1; 0 1 1 1 1 1; 0 0 1 1 1 0];
Z = [0 0 1 0 0 0; 0 0 1 0 0 0; 1 1 1 0 1 1; 1 1 1 0 1 1];

C= [0.1 0.5 0.9 0.9 0.1 0.5];   % color/face

X = L*(X-0.5) + xc;
Y = L/1.5*(Y-0.5) + yc;
Z = L/3*(Z-0.5) + zc;
V=[reshape(X,1,24); reshape(Y,1,24); reshape(Z,1,24)]; %reshape takes the 
% element of X and it fix them in only one coulomn (in this case)

%% Track IMU pose

% Offset Calibration:
calib1 = input('Press Enter once the Arduino is lying flat (IMU chip parallel to horizon)');
% Now calibrate x and y
% Here is an example of taking 10 accelerometer readings, then averaging
% each axis:
numreads = 10;
vals = zeros(3,numreads);
for i = 1:numreads
    val = nb.accelRead();
    vals(1,i) = val.x;
    vals(2,i) = val.y;
    vals(3,i) = val.z;
end
%Note the index, getting every column in a specific row for each axis:
meanOffx = mean(vals(1,:));
meanOffy = mean(vals(2,:));
meanOffz = mean(vals(3,:));
xOff = '?' - meanOffx; % What is the expected x value when the chip is flat?
yOff = '?' - meanOffy; % What is the expected y value when the chip is flat?
zOff = '?' - meanOffz; % What is the expected z value when the chip is flat?

% % IF PLAYING GAME, UNCOMMENT %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% pitchT = randi([-60, 60]);
% pitchT = pitchT * pi/180;
% rollT = randi([-60, 60]);
% rollT = rollT * pi/180;
% 
% dcm_targ = angle2dcm(0, pitchT, rollT);
% V_targ = dcm_targ*V;
% X_targ=reshape(V_targ(1,:),4,6);
% Y_targ=reshape(V_targ(2,:),4,6);
% Z_targ=reshape(V_targ(3,:),4,6);
% 
% figure(1)
% 
% fill3(X_targ,Y_targ,Z_targ,C,'FaceAlpha',alpha);
% xlim([-2 2]);
% ylim([-2 2]);
% zlim([-2 2]);
% box on;
% drawnow
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

tic; %to count the seconds


while(toc<20) % stop after this many seconds

    numreads = 3;
    IMUvals = zeros(3,numreads);
    for i = 1:numreads
        val = nb.accelRead();
        IMUvals(1,i) = val.x;
        IMUvals(2,i) = val.y;
        IMUvals(3,i) = val.z;
    end
    %Note the index, getting every column in a specific row for each axis:
    meanx = mean(IMUvals(1,:));
    meany = mean(IMUvals(2,:));
    meanz = mean(IMUvals(3,:));
    ax = meanx + xOff; 
    ay = meany + yOff;
    az = meanz + zOff;

    theta = '?'; % Find this in the readings!
    psi = '?'; % ^
    phi = '?'; % ^
    
    dcm_acc = angle2dcm('?', '?', '?'); % creates the rotation matrix, one of the parameters will be 0!

    % % IF PLAYING GAME, UNCOMMENT %%%%%%%%%%%%%%%%%%%%%%%%%%%
    % pitchCheck = psi * 180/pi;
    % rollCheck = theta * 180/pi;
    % if((pitchCheck > (rad2deg(pitchT) - 5)) & ...
    %         (pitchCheck < (rad2deg(pitchT) + 5)) & ...
    %         (rollCheck > (rad2deg(rollT) - 5)) & ...
    %         (rollCheck < (rad2deg(rollT) + 5)))
    %     fprintf('Matching desired orientation!\n');
    % else
    %     fprintf('Not matching desired orientation...\n');
    % end
    % %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    V_rot=dcm_acc*V;
    X_rot=reshape(V_rot(1,:),4,6);
    Y_rot=reshape(V_rot(2,:),4,6);
    Z_rot=reshape(V_rot(3,:),4,6);


    figure(2)

    fill3(X_rot,Y_rot,Z_rot,C,'FaceAlpha',alpha);
    xlim([-2 2]);
    ylim([-2 2]);
    zlim([-2 2]);
    box on;
    drawnow

    pause(0.1);
  
end

%% 4. EXTENSION (optional)
% - Change the color of the 3d block being displayed every time you press a
% pushbutton.
% - Change the color of the 3d block being displayed every time you tap 
% the accelerometer.

%% 5. DISCONNECT
%  Clears the workspace and command window, then
%  disconnects from the nanobot, freeing up the serial port.

clc
delete(nb);
clear('nb');
clear all