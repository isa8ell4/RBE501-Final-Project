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
% [j2current, allCurrentT1] = cpControl(robot, travelTime, thetasA, thetasB, currents, speedRate);
% avgAllCurrT1 = [mean(allCurrentT1(:,1)), mean(allCurrentT1(:,2)), mean(allCurrentT1(:,3)), mean(allCurrentT1(:,4))];
% % Task 2: LSPB
% [qAC, qCB] = lspbControl(robot, thetasA, thetasB, thetasC, travelTime);

% Task 3: LSPB Velocity
% lspbVelocityControl(robot, thetasA, thetasB, thetasC, travelTime, speedRate);
% [allCurrentT3AB] = lspbVelocityControlAB(robot, thetasA, thetasB, thetasC, travelTime, speedRate);
% avgAllCurrT3 = [mean(allCurrentT3AB(:,1)), mean(allCurrentT3AB(:,2)), mean(allCurrentT3AB(:,3)), mean(allCurrentT3AB(:,4))];


% % % Task 4
% clc;
% HW4 stuff
ws = [[0,0,1];
      [0,1,0];
      [0,1,0];
      [0,1,0]];

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

% get curr at pos A
% currPosA = getCurrAtPos(thetasA); % weird error once put in a helper func

% calc torque at pos A
g = [0; 0; -9.8];
Ftip = [0;0;0;0;0;0];
dthetalist = [0;0;0;0];
ddthetalist = [0;0;0;0];
tauA = InverseDynamics(deg2rad(thetasA), dthetalist, ddthetalist, g, Ftip, Mlist, Glist, slist);

% got this from spreadsheet
overallAvgCurrA = [-4.150285714,-234.9815646,-138.9430748,-44.17455782].*0.001;

% get torque-current relationship
coefficients = currentTorqueRelationship(overallAvgCurrA, tauA.');

% get current readings while applying known wrench
FtipKnown = [0;0;0;0;0;10];
tauAppliedID = InverseDynamics(deg2rad(thetasA), dthetalist, ddthetalist, g, FtipKnown, Mlist, Glist, slist);

robot.writeMode('cp')
robot.writeCurrents(currents);
% robot.writeJoints(thetasA); % Write joints to zero position
pause(5);

%%% static readings at pos A
% currReadings = zeros(1,4);
% tic;
% while toc < 20
%     readings = robot.getJointsReadings();
%     currReadings = vertcat(currReadings, readings(3, :));
% end
% currReadings = currReadings(2:end, :);
% avgCurrAppliedW = [mean(currReadings(:,1)), mean(currReadings(:,2)), mean(currReadings(:,3)), mean(currReadings(:,4))];
% 
% tauAppliedReadings = coefficients(1).*(avgCurrAppliedW) + coefficients(2);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% robot goes A to B and graph calculated torques from lspb/MR and another
% from current readings and equation
tic;
% [allCurrentT1] = lspbVelocityControlAB(robot, thetasA, thetasB, thetasC, travelTime, speedRate);
[j2current, allCurrentT1] = cpControl(robot, travelTime, thetasA, thetasB, currents, speedRate);
% avgAllCurrT1 = [mean(allCurrentT1(:,1)), mean(allCurrentT1(:,2)), mean(allCurrentT1(:,3)), mean(allCurrentT1(:,4))];
totalTime = toc;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% current graph
rowsCR = size(allCurrentT1, 1);
timeXread = zeros(rowsCR, 1);
currTime = 0;
interval = totalTime/rowsCR;
for i=1:rowsCR
    timeXread(i) = currTime;
    currTime = currTime + interval;
end

figure;
xlabel('Time (s)');
ylabel('Torque (Nm)');
%     title('Current at Joint 2 Velocity Control')
% title(sprintf('Motor Current at Joint 2 at %.0f%% Speed (Velocity Control)', percentSpeed*100));
allTausReadings = zeros(1,4);
for i=1:size(allCurrentT1,1)
    instTauR = coefficients(1).*allCurrentT1(i,:) + coefficients(2)*ones(1,4);
    allTausReadings = vertcat(allTausReadings, instTauR);
end
allTausReadings = allTausReadings(2:end, :);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

m = 100; % number of points
qAB1 = lspb(thetasA(1), thetasB(1), m);
qAB2 = lspb(thetasA(2), thetasB(2), m);
qAB3 = lspb(thetasA(3), thetasB(3), m);
qAB4 = lspb(thetasA(4), thetasB(4), m);

qAB = [qAB1,qAB2,qAB3,qAB4];

allTausCalc = zeros(1,4);
for i=1:size(qAB,1)
    instTauCalc = 1000.*InverseDynamics(deg2rad(qAB(i,:).'), dthetalist, ddthetalist, g, Ftip, Mlist, Glist, slist);
    allTausCalc = vertcat(allTausCalc, instTauCalc.');
end
allTausCalc = allTausCalc(2:end, :);

timeXCalc = zeros(size(allTausCalc, 1), 1);
currTime = 0;
interval = totalTime/size(allTausCalc, 1);
for i=1:size(allTausCalc, 1)
    timeXCalc(i) = currTime;
    currTime = currTime + interval;
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Plot the data
plot(timeXread, allTausReadings(:, 1), 'r-', 'LineWidth', 1.5);
hold on;
plot(timeXread, allTausReadings(:, 2), 'b-', 'LineWidth', 1.5);
plot(timeXread, allTausReadings(:, 3), 'g-', 'LineWidth', 1.5);
plot(timeXread, allTausReadings(:, 4), 'm-', 'LineWidth', 1.5);

plot(timeXCalc, allTausCalc(:,1), 'r-.', 'LineWidth', 1.5);
plot(timeXCalc, allTausCalc(:,2), 'b-.', 'LineWidth', 1.5);
plot(timeXCalc, allTausCalc(:, 3), 'g-.', 'LineWidth', 1.5);
plot(timeXCalc, allTausCalc(:, 4), 'm-.', 'LineWidth', 1.5);

legend({'J1 Reading', 'J2 Reading', 'J3 Reading', 'J4 Reading', ...
        'J1 Calc', 'J2 Calc', 'J3 Calc', 'J4 Calc'}, ...
        'Location', 'best');

xlabel('Time (s)');
ylabel('Joint Torques (Nm)');
title('Torque Readings vs Torque Calculations Traveling from A to B Using CP control');
grid on;
hold off;



