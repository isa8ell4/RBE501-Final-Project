function [endCommand] = closenessControl(deltaP, command, threshold)
    % Function to control closeness based on a threshold
    % deltaP: 1xN or MxN array of position deltas
    % command: 1xN or MxN array of commands
    % threshold: scalar or 1xN array of thresholds
    
    % Determine the number of columns (N)
    num = size(deltaP, 2);
    
    % Initialize the output command to zeros
    endCommand = command;
    
    % Loop through each column
    for i = 1:num
        if deltaP(i) <= threshold % Use correct indexing
            endCommand(i) = 0; % Assign corresponding command
        end
    end
end

