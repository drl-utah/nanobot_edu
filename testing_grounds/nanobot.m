classdef nanobot < handle
    properties
        u
        serialPort
        baudRate
        arduino
        plotFigure
        plotAxes
        robotIP
        robotPORT
        sizeofBuffer
        sendMode
        wifiData
        ready
    end

    methods
        % Constructor
        function obj = nanobot(serialPort, baudRate, connectMode)
            if connectMode == "serial"
                % Find all open serial ports
                warning('off','instrument:instrfind:FunctionToBeRemoved')
                serialPorts = instrfind('Type', 'serial');
                % Close all open serial ports
                if ~isempty(serialPorts)
                    fclose(serialPorts);
                end
                obj.serialPort = serialPort;
                obj.baudRate = baudRate;
                obj.sendMode = "serial";
                obj.arduino = serialport(serialPort, baudRate);
                fopen(obj.arduino);
                pause(1);
                obj.init('arduino',0,0)
                if isvalid(obj.arduino)
                    disp('Connection established')
                else
                    disp('Failed to establish connection')
                end
            elseif connectMode == "wifi"
                obj.robotIP = "192.168.1.100";
                obj.robotPORT = 551;
                obj.sizeofBuffer = 250;
                obj.ready = 0;
                obj.sendMode = "wifi";
                obj.u = udpport("datagram");
                flush(obj.u, "input");
                configureCallback(obj.u, "datagram", 1, @obj.udpread);  

                obj.init('wifi',0,0);
            end
        end

        function udpread(obj, varargin)
            % READ  is a callback function designed to be called whenever
            % there is a datagram available in the UDP buffer. This
            % function handles the message appropriately, and never needs
            % to be called by the user
            try
                % Read data and update status
                datagramCount = obj.u.NumDatagramsAvailable;
                uDatagram = read(obj.u, datagramCount);
            catch
                disp('UDP Error!');
            end
            obj.ready = 1;
            obj.wifiData = char(uDatagram.Data);
        end

        % Destructor
        function delete(obj)
            if obj.sendMode == "serial"
                warning('off','transportlib:legacy:DoesNotCloseConnection')
                fclose(obj.arduino);
                delete(obj.arduino);
            elseif obj.sendMode == "wifi"
                flush(obj.u, "input");
                clear obj.u
            end
        end


        % Method to perform digitalRead operation
        function value = digitalRead(obj, pin)
            value = obj.read('digital', obj.convertPin(pin));
        end

        % Method to perform analogRead operation
        function value = analogRead(obj, pin)
            value = obj.read('analog', obj.convertPin(pin));
        end

        % Method to perform digitalWrite operation
        function digitalWrite(obj, pin, value)
            obj.write('digital', obj.convertPin(pin), value);
        end

        % Method to set a motor value
        function setMotor(obj, pin, value)
            obj.write('motor', pin, value);
        end

        % Method to set a servo angle
        function setServo(obj, pin, value)
            obj.write('servo', pin, value);
        end

        % Method to perform accelRead operation
        function values = accelRead(obj)
            values = obj.read('accel', 0);
        end

        % Method to write to the onboard LED
        function ledWrite(obj, value)
            obj.write('led', 0, value);
        end

        % Method to initialize the ultrasonic rangefinder
        function initUltrasonic(obj,trigpin,echopin)
            obj.init('ultrasonic',trigpin,echopin)
        end

        % Method to take an ultrasonic distance reading
        function value = ultrasonicRead(obj)
            value = obj.read('ultrasonic',0);
        end

        % Method to take a reflectance sensor distance reading
        function values = reflectanceRead(obj)
            values = obj.read('reflectance',0);
        end

        % Method to initialize a piezo buzzer
        function initPiezo(obj,tonepin)
            obj.init('piezo',tonepin)
        end

        % Method to send a piezo tone
        function setPiezo(obj,frequency, duration)
            obj.write('piezo',frequency,duration)
        end

        function values = encoderRead(obj, num)
            values = obj.read('encoder',num);
        end

        % TODO: Method to initialize the reflectance array
        function initReflectance(obj)
            obj.init('reflectance',0);
        end

        % TODO: Method to initialize the Hall effect board



        % TODO: Method to initialize the color sensor


        % TODO: Method to read from the reflectance array


        % TODO: Method to read from the Hall effect board


        % TODO: Method to read from the reflectance array

        % Method to set pin modes on the Arduino
        function pinMode(obj, pin, mode)
            obj.init('pinmode',pin,mode)
        end

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Method to send JSON packet
        function sendJSON(obj, mode, periph, pin, value)
            % Create a struct for the JSON packet
            jsonPacket.mode = mode;
            jsonPacket.periph = periph;
            jsonPacket.pin = pin;
            jsonPacket.value = value;

            % Convert struct to JSON string
            jsonString = jsonencode(jsonPacket);

            if obj.sendMode == "serial"
                % Send the JSON string over serial
                fprintf(obj.arduino, jsonString);
            elseif obj.sendMode == "wifi"
                sendOverUDP(obj, jsonString);
            end
        end

        function [jsonReply] = receiveJSON(obj)
            if obj.sendMode == "serial"
                jsonString = fgetl(obj.arduino);
                jsonReply = jsondecode(jsonString);
            elseif obj.sendMode == "wifi"
                while obj.ready == 0
                    pause(0.010);
                end
                jsonString = obj.wifiData;
                jsonReply = jsondecode(jsonString);
                obj.ready = 0;
            end
        end

        function sendOverUDP(obj, dataString)
            % SENDOVERUDP  sends a string over UDP to the established UDP
            % host. This isn't designed to used directly by the user, but
            % rather by this class to streamline communication.
            
            write(obj.u, dataString, obj.robotIP, obj.robotPORT); %Sends the data over UDP
        end


        % Method to convert pin notation (e.g., A1, D2) to numeric value
        function pinNum = convertPin(obj, pin)
            if ischar(pin)
                if strncmpi(pin, 'A', 1)
                    pinNum = str2double(pin(2:end)) + 14; % Add an offset of 14 for analog pins
                elseif strncmpi(pin, 'D', 1)
                    pinNum = str2double(pin(2:end)); % No offset needed for digital pins
                elseif strncmpi(pin, 'M', 1)
                    pinNum = str2double(pin(2:end)); % No offset needed for motor pins
                else
                    error('Invalid pin notation. Use A1, A2, D2, D3, etc.');
                end
            else
                pinNum = pin;
            end
        end

        % Method to perform read operation
        function value = read(obj, periph, pin)
            pinNum = obj.convertPin(pin);
            obj.sendJSON('read', periph, pinNum, 0);

            % Read the JSON reply
            jsonReply = obj.receiveJSON();

            % Depending on the peripheral, read the appropriate values
            switch periph
                case 'accel'
                    % For 'accel', we expect 'x', 'y', and 'z' fields
                    if isfield(jsonReply, 'x') && isfield(jsonReply, 'y') && isfield(jsonReply, 'z')
                        value.x = jsonReply.x;
                        value.y = jsonReply.y;
                        value.z = jsonReply.z;
                    else
                        error('Invalid accelerometer values');
                    end
                case 'encoder'
                    if isfield(jsonReply, 'count') && isfield(jsonReply, 'countper')
                        value.counts = jsonReply.count;
                        value.countspersec = jsonReply.countper;
                    else
                        error('Invalid accelerometer values');
                    end
                case {'digital', 'analog', 'ultrasonic'}
                    % For 'digital', 'analog', and 'ultrasonic', we expect a 'value' field
                    if isfield(jsonReply, 'value')
                        value = jsonReply.value;
                    else
                        error('Invalid value');
                    end
                case 'reflectance'
                    % For 'reflectance', we expect 'one', 'two', 'three', and 'four' fields
                    if isfield(jsonReply, 'one') && isfield(jsonReply, 'two') && isfield(jsonReply, 'three') && isfield(jsonReply, 'four')
                        value.one = jsonReply.one;
                        value.two = jsonReply.two;
                        value.three = jsonReply.three;
                        value.four = jsonReply.four;

                    else
                        error('Invalid accelerometer values');
                    end
                otherwise
                    error('Invalid peripheral');
            end
        end

        % Method to perform write operation
        function write(obj, periph, pin, value)
            switch periph
                case 'digital'
                    pinNum = obj.convertPin(pin);
                    obj.sendJSON('write', periph, pinNum, value);
                case 'led'
                    obj.sendJSON('write', periph, 0, value);
                case 'piezo'
                    obj.sendJSON('write', periph, pin, value);
                case 'motor'
                    obj.sendJSON('write', periph, pin, value);
                case 'servo'
                    obj.sendJSON('write', periph, pin, value);
            end
            obj.waitAck()
        end

        % Method to perform init operation
        function init(obj, periph, pin, value)
            switch periph
                case 'arduino'
                    obj.sendJSON('init',periph,0,0);
                case 'ultrasonic'
                    trigPin = obj.convertPin(pin);
                    echoPin = obj.convertPin(value);
                    obj.sendJSON('init', periph, trigPin, echoPin);
                case 'piezo'
                    tonepin = obj.convertPin(pin);
                    obj.sendJSON('init', periph, tonepin, 0);
                case 'pinmode'
                    pinNum = obj.convertPin(pin);
                    obj.sendJSON('init', value, pinNum, 0);
                case 'wifi'
                    obj.sendJSON('init',periph,0,0);
                case 'reflectance'
                    obj.sendJSON('init',periph,0,0);
            end
            obj.waitAck()
        end

        % Method to wait for an ack from Arduino
        function waitAck(obj)
            jsonReply = obj.receiveJSON();
            % Check if the message field in the JSON reply is "ack"
            if isfield(jsonReply, 'message') && strcmp(jsonReply.message, 'ack')
                % Acknowledgment received, do nothing
            elseif isfield(jsonReply, 'message') && strcmp(jsonReply.message, 'error')
                % Error received, tell the user
                disp('Error received: Have you initialized the peripheral? Is this a valid pin or value?')
            else
                % No acknowledgment received, display error message
                disp('Error: No acknowledgment received from Arduino');
            end
        end

        % Method to create a live plot
        function livePlot(obj, periph, pin)
            % Create the figure and axes for the plot
            obj.plotFigure = figure;
            obj.plotAxes = axes;

            % Set the title and labels
            title('Live Plot');
            xlabel('Time (seconds)');
            ylabel('Value');

            % Set the time window in seconds for the x-axis
            timeWindow = 5;

            % Initialize the x and y data arrays
            xData = [];
            yData = [];
            yDataX = [];
            yDataY = [];
            yDataZ = [];

            % Add a 'Stop' button to the figure
            stopButton = uicontrol('Style', 'pushbutton', 'String', 'Stop',...
                'Position', [10 20 50 20],...
                'Callback', @(src, event) close(gcbf));

            % Start the live plotting
            startTime = now;
            while ishandle(obj.plotFigure)
                if strcmpi(periph, 'accel')
                    % Request accelerometer values
                    values = obj.read('accel', 0);

                    % Calculate elapsed time since the start of the live plot
                    elapsedTime = (now - startTime)* 24 * 60 * 60;

                    % Update x and y data arrays
                    xData = [xData, elapsedTime]; % Elapsed time is already in seconds
                    yDataX = [yDataX, values.x]; % Plot the X-axis value
                    yDataY = [yDataY, values.y]; % Plot the Y-axis value
                    yDataZ = [yDataZ, values.z]; % Plot the Z-axis value

                    % Update the plot
                    plot(obj.plotAxes, xData, yDataX, 'DisplayName', 'X');
                    hold on;
                    plot(obj.plotAxes, xData, yDataY, 'DisplayName', 'Y');
                    plot(obj.plotAxes, xData, yDataZ, 'DisplayName', 'Z');
                    hold off;

                else
                    % Read pin value
                    value = obj.read(periph, obj.convertPin(pin));

                    % Calculate elapsed time since the start of the live plot
                    elapsedTime = (now - startTime)* 24 * 60 * 60;

                    % Update x and y data arrays
                    xData = [xData, elapsedTime]; % Convert elapsed time to seconds
                    yData = [yData, value];

                    % Update the plot
                    plot(obj.plotAxes, xData, yData, 'DisplayName', ['Pin ', num2str(pin)]);
                end

                % Set the x-axis limits based on the latest data points
                if elapsedTime < timeWindow
                    xlim(obj.plotAxes, [0, timeWindow]);
                else
                    xlim(obj.plotAxes, [max(0, max(xData)-timeWindow), max(xData)]);
                end

                % Add legend
                legend(obj.plotAxes, 'Location', 'best');

                % Update x-axis and y-axis labels
                xlabel(obj.plotAxes, 'Time (seconds)');
                ylabel(obj.plotAxes, 'Value');


                % Pause for a short duration
                pause(0.05);
            end
        end
    end


end
