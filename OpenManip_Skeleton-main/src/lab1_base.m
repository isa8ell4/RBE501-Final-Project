%% Setup robot
travelTime = 2; % Defines the travel time
robot = Robot(); % Creates robot object
robot.writeTime(travelTime); % Write travel time
robot.writeMotorState(true); % Write position mode
%% Program 

robot.writeJoints(0); % Write joints to zero position
pause(travelTime); % Wait for trajectory completion

% baseWayPoints = [-5, 5, -45]; % Define base waypoints
% robot.getJointsReadings()
% robot.writeJoints([-45, -2, 41.5, -39.5]); % Write joint values
% robot.getJointsReadings()
robot.writeJoints([-45, -2, 41.5, -39.5])
while true
    robot.getJointsReadings()
end


% for baseWayPoint = baseWayPoints % Iterate through waypoints
% 
%     robot.writeJoints([baseWayPoint, -2, 41.5, -39.5]); % Write joint values
% 
%     tic; % Start timer
% 
%     while toc < travelTime
%         disp(robot.getJointsReadings()); % Read joint values
%     end
% 
% end

robot.writeGripper(false);

pause(1);