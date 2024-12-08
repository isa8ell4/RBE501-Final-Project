%% Setup robot
speedRate = 0.4; % Constant that determines speed percentage
travelTime = 2 / speedRate; % Defines the travel time (seconds)
acc_time = 0.5 / speedRate; % Acceleration time (seconds)
robot = Robot(); % Creates robot object
robot.writeTime(travelTime); % Write travel time
robot.writeMotorState(true); % Write position mode
%% Program 
robot.writeMode('cp')
robot.writeJoints(0); % Write joints to zero position
pause(travelTime);

% Initialize variables
counter = 0;
current_readings = robot.getJointsReadings();
robot.writeCurrents([1000, 1000, 1000, 1000]);

tic;
robot.writeJoints([-45, -2, 41.5, -39.5]); % Write joint values
pause(travelTime)
robot.writeJoints([42, 52, 42, -95]);

% Prepare figure for live plotting
figure;
xlabel('Time (s)');
ylabel('Current (mA)');
title('Motor Currents in Joint 2 at 40% Speed')
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

