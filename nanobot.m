classdef nanobot
    properties
        serialPort
        baudRate
        arduino
        plotFigure
        plotAxes
    end

    methods
        % Constructor
        function obj = nanobot(serialPort, baudRate)
            % Find all open serial ports
            serialPorts = instrfind('Type', 'serial');
            % Close all open serial ports
            if ~isempty(serialPorts)
                fclose(serialPorts);
            end
            obj.serialPort = serialPort;
            obj.baudRate = baudRate;
            obj.arduino = serial(serialPort, 'BaudRate', baudRate);
      
            fopen(obj.arduino);
            pause(0.5);
            obj.init('arduino',0,0)
            if isvalid(obj.arduino)
                disp('Connection established')
            else
                disp('Failed to establish connection')
            end
        end

        % Destructor
        function delete(obj)
            fclose(obj.arduino);
            delete(obj.arduino);
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

            % Send the JSON string over serial
            fprintf(obj.arduino, jsonString);
        end

        % Method to convert pin notation (e.g., A1, D2) to numeric value
        function pinNum = convertPin(obj, pin)
            if ischar(pin)
                if strncmpi(pin, 'A', 1)
                    pinNum = str2double(pin(2:end)) + 14; % Add an offset of 14 for analog pins
                elseif strncmpi(pin, 'D', 1)
                    pinNum = str2double(pin(2:end)); % No offset needed for digital pins
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
            jsonString = fgetl(obj.arduino);
            jsonReply = jsondecode(jsonString);

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
            end
            obj.waitAck()
        end

        % Method to wait for an ack from Arduino
        function waitAck(obj)
            jsonString = fgetl(obj.arduino);
            jsonReply = jsondecode(jsonString);
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
