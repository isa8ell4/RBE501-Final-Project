%% Setup robot
speedRate = 1; % Constant that determines speed percentage
travelTime = 1 / speedRate; % Defines the travel time (seconds)
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

%% Task 2
thetasA = [-45; -2.0115; 41.5415; -39.53];
thetasB = [42; 52; 42; -95];
thetasC = [0; -51.3456; 48.5069; 2.8386];

% % m=100;
% % qAC1 = lspb(thetasA(1), thetasC(1), m);
% % qAC2 = lspb(thetasA(2), thetasC(2), m);
% % qAC3 = lspb(thetasA(3), thetasC(3), m);
% % qAC4 = lspb(thetasA(4), thetasC(4), m);
% % 
% % qCB1 = lspb(thetasC(1), thetasB(1), m);
% % qCB2 = lspb(thetasC(2), thetasB(2), m);
% % qCB3 = lspb(thetasC(3), thetasB(3), m);
% % qCB4 = lspb(thetasC(4), thetasB(4), m);
% % 
% % qAC = [qAC1,qAC2,qAC3,qAC4];
% % qCB = [qCB1,qCB2,qCB3,qCB4];
% % 
% % robot.writeMode('cp')
% % robot.writeJoints(thetasA.')
% % pause(travelTime);
% % 
% % i = 1;
% % velReadingsAC = zeros(1, 4);
% % posReadingsAC = thetasA.';
% % 
% % % % while the getJointsReadings are not at desired position, then keep moving
% % currPos = robot.getJointsReadings();
% % delta = inf;
% % tic;
% % 
% % while (delta > 1 || i < size(qAC, 1))
% %     readings = robot.getJointsReadings();
% %     if i >= size(qAC,1)
% %         velReadingsAC = vertcat(velReadingsAC, readings(2, :));
% %         posReadingsAC = vertcat(posReadingsAC, readings(1,:));
% %         delta = max(abs(readings(1,:)) - abs(thetasC.'));
% %     else
% %         robot.writeJoints(qAC(i,:));
% %         velReadingsAC = vertcat(velReadingsAC, readings(2, :));
% %         posReadingsAC = vertcat(posReadingsAC, readings(1,:));
% %         pause(travelTime/100);
% %         delta = max(abs(readings(1,:)) - abs(thetasC.'));
% %         i = i+1;
% %     end
% % end
% % totalTime = toc;
% % disp('at pos c')
% % 
% % pause(1);

% i = 1;
% while i < it(1)
%     robot.writeJoints(qCB(i, :))
%     pause(0.01);
%     i = i+1;
% end

%% Task 3
robot.writeMode('cp')
robot.writeJoints(thetasA.')
pause(2);

% % % calc vel from qACrobot.writeMode('cp')
robot.writeJoints(thetasA.')
pause(2);

% % % calc vel from qAC

robot.writeMode('v')
i = 1;
deltaP = 1000*ones(1,4);
deltaPHistory = zeros(1,4);
velHistory = zeros(1,4);
while (max(deltaP) > 1) %|| i < size(velReadingsAC, 1))
    readings = robot.getJointsReadings();

    if i >= size(velReadingsAC,1)
        deltaP = abs(readings(1,:)) - abs(thetasC.');
        deltaPHistory = vertcat(deltaPHistory, deltaP);
    else
%         vel = velReadingsAC(i,:);
        vel = velPID(readings(1,:), thetasC.', readings(2,:), 0.25, 0.025, 0.05, 0.01, [0,0,0,0]);
        velHistory = vertcat(velHistory, vel);
        robot.writeVelocities(vel);
        pause(0.01);
        deltaP = abs(readings(1,:)) - abs(thetasC.');
        deltaPHistory = vertcat(deltaPHistory, deltaP);
        i = i+1;
    end
end
disp('at pos c')
robot.writeVelocities([0,0,0,0]);


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
%     time = totalTime/prs(1) * ones(1,4);
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
%         pause(totalTime/size(velReadingsAC, 1));
%         deltaP = abs(readings(1,:)) - abs(thetasC.');
%         deltaPHistory = vertcat(deltaPHistory, deltaP);
%         i = i+1;
%     end
% end









