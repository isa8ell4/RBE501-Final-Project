%% Setup robot
speedRate = 0.4; % Constant that determines speed percentage
travelTime = 2 / speedRate; % Defines the travel time (seconds)
acc_time = 0.5 / speedRate; % Acceleration time (seconds)
robot = Robot(); % Creates robot object
robot.writeTime(travelTime); % Write travel time
robot.writeMotorState(true); % Write position mode
%% Program 
% robot.writeMode('cp')
% robot.writeJoints(0); % Write joints to zero position
% pause(travelTime);
% 
% % Initialize variables
% counter = 0;
% current_readings = robot.getJointsReadings();
% robot.writeCurrents([1000, 1000, 1000, 1000]);
% 
% tic;
% robot.writeJoints([-45, -2, 41.5, -39.5]); % Write joint values
% pause(travelTime)
% robot.writeJoints([42, 52, 42, -95]);
% 
% % Prepare figure for live plotting
% figure;
% xlabel('Time (s)');
% ylabel('Current (mA)');
% title('Motor Currents in Joint 2 at 40% Speed')
% hold on;
% 
% % Loop to monitor joint positions
% while toc < travelTime*2
%     counter = counter + 1;
%     readings = robot.getJointsReadings(); % Get current readings
%     current_readings = vertcat(current_readings, readings); % Append readings
%     j2_current = current_readings(:, 2); % Extract second joint data
%     plot(j2_current); % Update plot dynamically
%     drawnow;
% end

%% Task 3
Tc = [0,0,-1,0.185;
     0,1,0,0;
     1,0,0,0.240;
     0,0,0,1];
Ta = [0,0.7071,0.7071,0.185;
     0,0.7071,-0.7071,-0.185;
     1,0,     0,      0.185;
     0,0,0,1];
M = [0,0,-1,0.2944;
     0,1,0,0;
     1,0,0,0.2625;
     0,0,0,1];

thetasA = [-45; -2.0115; 41.5415; -39.53];
thetasB = [42; 52; 42; -95];
thetasC = [0; -51.3456; 48.5069; 2.8386];


m=100;
qAC1 = lspb(thetasA(1), thetasC(1), m);
qAC2 = lspb(thetasA(2), thetasC(2), m);
qAC3 = lspb(thetasA(3), thetasC(3), m);
qAC4 = lspb(thetasA(4), thetasC(4), m);

qCB1 = lspb(thetasC(1), thetasB(1), m);
qCB2 = lspb(thetasC(2), thetasB(2), m);
qCB3 = lspb(thetasC(3), thetasB(3), m);
qCB4 = lspb(thetasC(4), thetasB(4), m);

qAC = [qAC1,qAC2,qAC3,qAC4];
qCB = [qCB1,qCB2,qCB3,qCB4];

robot.writeMode('cp')
robot.writeJoints(thetasA.')
pause(travelTime);
it = size(qAC);
i = 1;

while i < it(1)
    robot.writeJoints(qAC(i,:));
    pause(0.01);
    i = i+1;
end

pause(5);
i = 1;
while i < it(1)
    robot.writeJoints(qCB(i, :))
    pause(0.01);
    i = i+1;
end






