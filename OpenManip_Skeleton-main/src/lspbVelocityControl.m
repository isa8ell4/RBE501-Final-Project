function lspbVelocityControl(robot, thetasA, thetasB, thetasC, travelTime)
    % Linear Segment with Parabolic Blend Velocity Control
    robot.writeMode('cp')
    robot.writeJoints(thetasA.')
    pause(travelTime);
    
    robot.writeMode('v')   
    deltaPA = 1000*ones(1,4);
    deltaPHistory = zeros(1,4);
    velHistory = zeros(1,4);
    constVel2 = 10;
    
    while (max(deltaPA) > 1)
        readings = robot.getJointsReadings();
        vel = velPID(readings(1,:), thetasC.', readings(2,:), 0.3, 0.025, 0.05, 0.01, [0,0,0,0]);
        velHistory = vertcat(velHistory, vel);
        velConstantAC = [constVel2, vel(1,2), vel(1,3), vel(1,4)];
        robot.writeVelocities(velConstantAC);
        pause(0.01);
        deltaPA = abs(readings(1,:)) - abs(thetasC.')
        deltaPHistory = vertcat(deltaPHistory, deltaPA);
    end
    disp('at pos c')
%     robot.writeVelocities([0,0,0,0]);
    
    deltaPB = 1000*ones(1,4);
    deltaPHistory = zeros(1,4);
    velHistory = zeros(1,4);
    pause(2)
    while (max(deltaPB) > 2)
        readings = robot.getJointsReadings();
        deltaPHistory = vertcat(deltaPHistory, deltaPB);
        deltaPB = abs(abs(readings(1,:)) - abs(thetasB.'))

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
    abs(max(deltaPB))
    disp('at pos b')
    robot.writeVelocities([0,0,0,0]);

end