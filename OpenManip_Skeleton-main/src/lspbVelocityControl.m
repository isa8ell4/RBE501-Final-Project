function [currentReadings] = lspbVelocityControl(robot, thetasA, thetasB, thetasC, travelTime, percentSpeed)
    % Linear Segment with Parabolic Blend Velocity Control
    robot.writeMode('cp')
    robot.writeJoints(thetasA.')
    pause(travelTime);
    tic;
    robot.writeMode('v')   
    deltaPA = 1000*ones(1,4);
    deltaPHistory = zeros(1,4);
    velHistory = zeros(1,4);
    constVel2 = 10;
    currentReadings = zeros(1,4);
    while (max(deltaPA) > 1)
        readings = robot.getJointsReadings();
        vel = velPID(readings(1,:), thetasC.', readings(2,:), 0.3, 0.025, 0.05, 0.01, [0,0,0,0]);
        velHistory = vertcat(velHistory, vel);
        velConstantAC = [constVel2, vel(1,2), vel(1,3), vel(1,4)];
        robot.writeVelocities(velConstantAC);
        pause(0.01);
        deltaPA = abs(readings(1,:)) - abs(thetasC.');
        deltaPHistory = vertcat(deltaPHistory, deltaPA);
        currentReadings = vertcat(currentReadings, readings(3,:));
    end
    disp('at pos c')
%     robot.writeVelocities([0,0,0,0]);
    
    deltaPB = 1000*ones(1,4);
    deltaPHistory = zeros(1,4);
    velHistory = zeros(1,4);
    pause(2)
    while (max(deltaPB) > 3)
        readings = robot.getJointsReadings();
        deltaPHistory = vertcat(deltaPHistory, deltaPB);
        deltaPB = abs(abs(readings(1,:)) - abs(thetasB.'))
        currentReadings = vertcat(currentReadings, readings(3,:));

        if deltaPB(:, 1) < -12
            vel = velPID(readings(1,:), thetasB.', readings(2,:), 0.3, 0.005, 0.05, 0.03, [0,0,0,0]);
            velHistory = vertcat(velHistory, vel);
        %         robot.writeVelocities(vel);
            velConstantCB = [constVel2, vel(1,2), vel(1,3), vel(1,4)];
            robot.writeVelocities(velConstantCB);
            pause(0.01);
        else
            vel = velPID(readings(1,:), thetasB.', readings(2,:), 0.75, 0.005, 0.3, 0.03, [0,0,0,0]);
            velHistory = vertcat(velHistory, vel);
            robot.writeVelocities(vel);
            pause(0.01);

        end
    end
    disp('at pos b')
    robot.writeVelocities([0,0,0,0]);
    totalTime = toc;

    % current graph
    rowsCR = size(currentReadings, 1);
    timeX = zeros(rowsCR, 1);
    currTime = 0;
    interval = totalTime/rowsCR;
    for i=1:rowsCR
        timeX(i) = currTime;
        currTime = currTime + interval;
    end
    
    figure;
    xlabel('Time (s)');
    ylabel('Current (mA)');
%     title('Current at Joint 2 Velocity Control')
    title(sprintf('Motor Current at Joint 2 at %.0f%% Speed (Velocity Control)', percentSpeed*100));
    hold on;
    
    plot(timeX, currentReadings(:, 2))

end