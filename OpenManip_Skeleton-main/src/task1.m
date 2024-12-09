function task1(robot, travelTime, pos1, pos2)
    % Current-Based Position Control
    robot.writeMode('cp')
    robot.writeJoints(0); % Write joints to zero position
    pause(travelTime);
    
    % Initialize variables
    counter = 0;
    current_readings = robot.getJointsReadings();
    robot.writeCurrents([1000, 1000, 1000, 1000]);
    
    tic;
    robot.writeJoints(pos1); % Write joint values
    pause(travelTime)
    robot.writeJoints(pos2);
    
    % Prepare figure for live plotting
    figure;
    xlabel('Time (s)');
    ylabel('Current (mA)');
    title('Motor Currents in Joint 2')
    hold on;
    
    % Loop to monitor joint positions
    while toc < travelTime*2
        counter = counter + 1;
        readings = robot.getJointsReadings(); % Get current readings
        current_readings = vertcat(current_readings, readings); % Append readings
        j2_current = current_readings(:, 2); % Extract second joint data
        plot(j2_current); % Update plot dynamically
        drawnow;
    end