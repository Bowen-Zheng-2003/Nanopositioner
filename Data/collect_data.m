% Clear command window and workspace
clc;
clear;

% Serial port configuration - change as needed
port = '/dev/cu.usbmodem1101'; % Your specified port
baudRate = 115200;

% Output CSV file name
outputFile = 'settling_time_PID_control.csv';

% Initialize serial port object
try
    s = serialport(port, baudRate);
    configureTerminator(s, "LF"); % Line feed as terminator (Arduino's println)
    disp(['Connected to ' port ' at ' num2str(baudRate) ' baud']);
    pause(2); % Wait for connection to stabilize
catch ME
    disp(['Error connecting to serial port: ' ME.message]);
    return;
end

% Open file for writing
fid = fopen(outputFile, 'w');
if fid == -1
    disp('Error opening CSV file');
    delete(s); % Clean up serial object
    return;
end

% Write CSV header
fprintf(fid, 'Time(s),Value\n');

% Record start time
startTime = tic; % Start timer

% Main data collection loop
disp('Starting serial data collection...');
disp('Press Ctrl+C to stop');
try
    while true
        % Read line from serial port
        if s.NumBytesAvailable > 0
            data = readline(s); % Read a line as string
            data = strtrim(data); % Remove whitespace
            
            % Attempt to convert to number and record if valid
            value = str2double(data);
            if ~isnan(value) % Only record if it's a valid number
                % Calculate elapsed time in seconds since start
                elapsedTime = toc(startTime); % Time in seconds with millisecond precision
                % Write to CSV
                fprintf(fid, '%.6f,%.6f\n', elapsedTime, value);
                disp(['Recorded: ' num2str(elapsedTime, '%.3f') 's, Value ' num2str(value)]);
            end
        end
        pause(0.001); % Small pause to prevent MATLAB from locking up
    end
catch ME
    disp(['An error occurred: ' ME.message]);
end

% Cleanup
fclose(fid);
delete(s);
clear s;
disp(['Data saved to ' outputFile]);
