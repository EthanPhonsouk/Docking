function bmeasure = getSerial()
    % 1. Detect available serial ports
    % Returns an array of strings with available port names
    availablePorts = serialportlist("available");
    
    % 2. Terminate if no connections are found
    if isempty(availablePorts)
        % The error() function stops execution immediately and prints the message
        error('SerialError:NoDevice', 'No serial connections detected. Looking for file instead.');
    end

    % 3. Port selection and configuration
    targetPort = availablePorts(1); % If there are multiple, this defaults to the first one. 
    baudRate = 115200;
    fprintf('Connecting to device on %s at %d baud...\n', targetPort, baudRate);

    try
        % Open the serial connection
        openMV = serialport(targetPort, baudRate);
        configureTerminator(openMV, "LF");
        
        % 4. Preallocate the 6x3 matrix with zeros
        % The 3rd column will remain 0 until your other process updates it
        bmeasure = zeros(6, 3);
        
        % 5. Read the data
        % Assuming the device sends 6 lines of data to fill the 6 rows
        for i = 1:6
            rawData = readline(openMV);
            parsedValues = str2double(split(rawData, ','));
            
            % Extract the 3 specific numbers
            % round() is used as a safety measure to ensure the row index is an integer
            rowIndex = round(parsedValues(1)); 
            yVal = parsedValues(2);
            zVal = parsedValues(3);
            
            % Safety check: Only assign if the row index is between 1 and 6
            if rowIndex >= 1 && rowIndex <= 6
                bmeasure(rowIndex, 2) = yVal;
                bmeasure(rowIndex, 3) = zVal;
            else
                warning('Serial Warning: Received invalid row index (%d). Line ignored.', rowIndex);
            end
        end
        
        fprintf('Matrix bmeasure successfully populated.\n');
        
        % Close the connection

        clear openMV;
        catch ME
        % Clean up the port if an error occurs
        if exist('device', 'var')
            clear openMV;
        end
        rethrow(ME); 
    end
end