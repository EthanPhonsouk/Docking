%% Author: Faris Stitou

% Get config file
[bmeasure, Ri] = getConfig;
while true
    try
        % Attempt to acquire serial data
        bmeasure = getSerial();
        
    catch ME
        
        % Check if it failed specifically because no device was found
        if strcmp(ME.identifier, 'SerialError:NoDevice')
            %disp('No serial connection detected. Falling back to config file');
            
        else
            % If it failed for a different reason (e.g., the device was unplugged mid-read)
            warning('Serial read failed with error: %s\nFalling back to config file', ME.message);
            
        end
    end
    
    % For now insert the focal length here.
    focalLength = 320;
    % Check if b_unit has its last column all zeroes, if so populate b_unit
    % with the focal length
    if all(bmeasure(:, 1) == 0)
        bmeasure(:, 1) = focalLength;
    end
    b_unit = unit(bmeasure);
    b_unit = b_unit';
    b_unit = b_unit(:);
    

    %Unknowns, coordinates are filled with starting values
    % guess_Rs = randi([0,1],[3 1]);
    % disp(guess_Rs);
    guess_Rs = [0; 0; 0]; %guess where the target is
    
    %arbitrary orientation values. This is a guess
    % guess_p = randi([0,0.9],[3 1]);
    % disp(guess_p);
    guess_p = [0; 0; 0]; %guess the rotation
    
    W = eye(18); %THIS MUST CHANGE LATER DEPENDING ON HOW GOOD THE LEDs ARE (SEE 3.24)
    
    %Tolerance can be lowered/highered depending on accuracy.
    %This method is insanely accurate so tolerance doesn't 
    %have to be high possibly saving some computation.
    tolerance = 0.00001; %for now I will make it high-tolerance
    iter = 0;
    max_iters = 200;
    
    while true
        iter = iter + 1;
        
        [H, deltab] = MSM(guess_p, guess_Rs, b_unit, Ri); % Recalculate H and deltab change to bmeasure or b_unit respectively
        %disp(deltab);
        
        % deltax is the optimal differential correction that minimizes the linearized
        % residual errors. This is function 3.26 seen on p.25
        deltax = (H' * W * H) \ (H' * W * deltab); %I had to get this specific line from 
    
        guess_Rs = guess_Rs + deltax(1:3); % Update the guess for R values
        guess_p = guess_p + deltax(4:6); % Update the guess for p values
        
        %fprintf('Iteration %d | Correction Magnitude: %f\n', iter, norm(deltax));
        % disp(guess_Rs);
        %disp(guess_p);
          
    
        %Use the norm as the condition which finds the largest value and then 
        %tests it against the set tolerance. This is what decides we are at a
        %good spot to break the loop and send the coordinates over
        if norm(deltax) < tolerance
            % final_Rs = guess_Rs;
            % final_p = guess_p;
            disp(guess_Rs);
            disp(guess_p); 
            break
        end
    
        if iter >= max_iters
            disp('Initial guess caused the sysyem to diverge.');
            break;
        end
    end
end