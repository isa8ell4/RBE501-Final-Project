%% Setup robot
speedRate = 1; % Constant that determines speed percentage
travelTime = 1 / speedRate; % Defines the travel time (seconds)
acc_time = 0.5 / speedRate; % Acceleration time (seconds)
robot = Robot(); % Creates robot object
robot.writeTime(travelTime); % Write travel time
robot.writeMotorState(true); % Write position mode

thetasA = [-45; -2.0115; 41.5415; -39.53];
thetasB = [42; 52; 42; -95];
thetasC = [0; -51.3456; 48.5069; 2.8386];
currents = [1000, 1000, 1000, 1000];

% % Task 1: Current-Based Position Control Mode
% cpControl(robot, travelTime, thetasA, thetasB, currents);

% % Task 2: LSPB
% lspbControl(robot, thetasA, thetasB, thetasC, travelTime);

% Task 3: LSPB Velocity
lspbVelocityControl(robot, thetasA, thetasB, thetasC, travelTime);


%% Task 3
% robot.writeMode('cp')
% robot.writeJoints(thetasA.')
% pause(2);
% 
% % % % calc vel from qACrobot.writeMode('cp')
% % % % calc vel from qAC
% 
% robot.writeMode('v')
% i = 1;
% deltaPA = 1000*ones(1,4);
% deltaPHistory = zeros(1,4);
% velHistory = zeros(1,4);
% while (max(deltaPA) > 1)
%     readings = robot.getJointsReadings();
%     vel = velPID(readings(1,:), thetasC.', readings(2,:), 0.3, 0.025, 0.05, 0.01, [0,0,0,0]);
%     velHistory = vertcat(velHistory, vel);
%     robot.writeVelocities(vel);
%     pause(0.01);
%     deltaPA = abs(readings(1,:)) - abs(thetasC.');
%     deltaPHistory = vertcat(deltaPHistory, deltaPA);
%     i = i+1;
% end
% disp('at pos c')
% robot.writeVelocities([0,0,0,0]);
% 
% 
% i = 1;
% deltaPB = 1000*ones(1,4);
% deltaPHistory = zeros(1,4);
% velHistory = zeros(1,4);
% pause(2)
% while (max(deltaPB) > 1)
%     readings = robot.getJointsReadings();
%     vel = velPID(readings(1,:), thetasB.', readings(2,:), 0.3, 0.005, 0.05, 0.03, [0,0,0,0]);
%     velHistory = vertcat(velHistory, vel);
%     robot.writeVelocities(vel);
%     pause(0.01);
%     deltaPB = abs(readings(1,:)) - abs(thetasB.');
%     deltaPHistory = vertcat(deltaPHistory, deltaPB);
%     i = i+1;
% end
% disp('at pos b')
% robot.writeVelocities([0,0,0,0]);

% while (deltaV > 1 || i < size(velReadingsAC, 1))
%     readings = robot.getJointsReadings();
%     if i >= size(velReadingsAC,1)
%         deltaV = max(abs(readings(1,:)) - abs(thetasC.'));
%     else
%         vel = velReadingsAC(i,:).';
%         robot.writeVelocities(vel);
%         pause(travelTime/100);
%         deltaV = max(abs(readings(1,:)) - abs(thetasC.'));
%         i = i+1;
%     end
% end

% while (max(deltaP) > 1) %|| i < size(velReadingsAC, 1))
%     readings = robot.getJointsReadings();
% 
%     if i >= size(velReadingsAC,1)
%         deltaP = abs(readings(1,:)) - abs(thetasC.');
%         deltaPHistory = vertcat(deltaPHistory, deltaP);
%     else
% %         vel = velReadingsAC(i,:);
%         vel = velPID(readings(1,:), thetasC, readings(2,:), 1, 0.5, 0.05, 0.01, [0,0,0,0]);
%         robot.writeVelocities(vel);
%         pause(0.01);
%         deltaP = abs(readings(1,:)) - abs(thetasC.');
%         deltaPHistory = vertcat(deltaPHistory, deltaP);
%         i = i+1;
%     end
% end

% % % % calc vel from qAC
% velCalc = zeros(1,4);
% prs = size(posReadingsAC);
% for i= 1:1:(prs(1)-1)
%     curr = posReadingsAC(i, :);
%     next = posReadingsAC(i+1, :);
%     time = totalTimeAC/prs(1) * ones(1,4);
%     instV = (next - curr)./time;
%     velCalc = vertcat(velCalc, instV);
% end
% velCalc = vertcat(velCalc, [0,0,0,0]);

% while (max(deltaP) > 1 || i < size(velReadingsAC, 1))
%     readings = robot.getJointsReadings();
%     if i >= size(velReadingsAC,1)
%         deltaP = abs(readings(1,:)) - abs(thetasC.');
%         deltaPHistory = vertcat(deltaPHistory, deltaP);
%     else
%         vel = velReadingsAC(i,:).';
%         robot.writeVelocities(vel);
%         pause(totalTimeAC/size(velReadingsAC, 1));
%         deltaP = abs(readings(1,:)) - abs(thetasC.');
%         deltaPHistory = vertcat(deltaPHistory, deltaP);
%         i = i+1;
%     end
% end









