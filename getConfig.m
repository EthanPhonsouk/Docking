%% Author: George Hinds

function [bmeasure, Ri] = getConfig()
    % 1. Open Windows Explorer to find the configuration file
    [file, path] = uigetfile('*.csv', 'Select the Configuration CSV file');

    % 2. Terminate the program if the user hits "Cancel"
    if isequal(file, 0)
        error('Program Terminated: No configuration file was selected.');
    end

    % 3. Construct path and read the table
    fullFileName = fullfile(path, file);
    
    % 'VariableNamingRule', 'preserve' keeps headers like "X_m" from changing
    opts = detectImportOptions(fullFileName);
    opts.VariableNamingRule = 'preserve';
    configData = readtable(fullFileName, opts);

    % Column 2 to 4: X, Y, Z coordinates
    bmeasure = configData{:, 2:4}; 
    
    % Column 5 to 7: Beacon X, Y, Z coordinates
    Ri = configData{:, 5:7};
    
    fprintf('Successfully loaded configuration from: %s\n', file);
end