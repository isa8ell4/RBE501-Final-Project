%% Setup robot
speedRate = 0.8; % Constant that determines speed percentage
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
% currentReadings = cpControl(robot, travelTime, thetasA, thetasB, currents, speedRate);

% % Task 2: LSPB
% [qAC, qCB] = lspbControl(robot, thetasA, thetasB, thetasC, travelTime);

% Task 3: LSPB Velocity
% lspbVelocityControl(robot, thetasA, thetasB, thetasC, travelTime, speedRate);



%% Task 4

% HW4 stuff
ws = [[0,0,1];
      [0,1,0];
      [0,1,0];
      [0,1,0]];
% l1 = convlength(1.50373, 'in', 'm'); % 
% l2 = convlength(3.79237, 'in', 'm'); % 
% l3 = convlength(5.12719, 'in', 'm');
% l3x = convlength(0.94488, 'in', 'm');
% l3z = convlength(5.03937, 'in', 'm');
% l4 = convlength(4.88189, 'in', 'm');
% l5 = convlength(5.76546, 'in', 'm');
l1 = 0.096326;
l2z = 0.128;
l2x = 0.024;
l3 = 0.12074;
l4 = 145.45;
slist = [[0; 0; 1; 0;            0; 0     ]...
         [0; 1; 0; -(l1);     0; 0     ]...
         [0; 1; 0; -(l1+l2z); 0; l2x   ]...
         [0; 1; 0; -(l1+l2z); 0; l2x+l3]];

M = [0,0,-1,l2x+l3+l4;
     0,1,0,0;
     1,0,0,l1+l2z;
     0,0,0,1];

M01 = [1, 0, 0, 0;...
       0, 1, 0, 0;...
       0, 0, 1, 0.0723;...
       0, 0, 0, 1];

M12 = [1, 0, 0, 0.0049;...
       0, 1, 0, 0;...
       0, 0, 1, 0.1251;...
       0, 0, 0, 1];

M23 = [1, 0, 0, 0.1129;...
       0, 1, 0, 0;...
       0, 0, 1, 0.0251;...
       0, 0, 0, 1];

M34 = [1, 0, 0, 0.0969;...
       0, 1, 0, 0;...
       0, 0, 1, 0.0025;...
       0, 0, 0, 1];

M45 = [0, 0, -1, 0.0668;...
       0, 1, 0, 0;...
       1, 0, 0, -0.00246;...
       0, 0, 0, 1];

G1 = diag([0.0001, 0.00001, 0.0001, 0.114, 0.114,0.114]); %
G2 = diag([3.5*1e-5, 1.7*1e-4, 1.7*1e-4, 0.095, 0.095, 0.095]);
G3 = diag([3.47*1e-5, 1.71*1e-4, 1.75*1e-4, 0.079, 0.079,0.079]);
G4 = diag([0.0003, 0.0002, 0.0002, 0.214, 0.214, 0.214]); %

Glist = cat(4, G1, G2, G3, G4);
Mlist = cat(4, M01, M12, M23, M34, M45);

m = 100; % number of points
qAB1 = lspb(thetasA(1), thetasB(1), m);
qAB2 = lspb(thetasA(2), thetasB(2), m);
qAB3 = lspb(thetasA(3), thetasB(3), m);
qAB4 = lspb(thetasA(4), thetasB(4), m);
qAB = [qAB1, qAB2, qAB3, qAB4];

totalTimeAC = 1;
% calc vel from trajectory
velCalcAB = getDerivative(qAB, totalTimeAC); % not sure what totalTime is suppose to be 
accCalcAB = getDerivative(velCalcAB, totalTimeAC);


g = [0; 0; -9.8];
Ftip = [0;0;0;0;0;0];

tauCalcAB = zeros(1,4);
for i = 1:(size(qAB, 1)-2)
    instTau = InverseDynamics(qAB(i,:).', velCalcAB(i, :).', accCalcAB(i, :).', g, Ftip, Mlist, Glist, slist);
    tauCalcAB = vertcat(tauCalcAB, instTau.');
end

figure;
xlabel('Time (s)');
ylabel('Torques');
title('Taulist at Joint 2')
hold on;

j2_tau = tauCalcAB(:,2);

plot(j2_tau)



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


