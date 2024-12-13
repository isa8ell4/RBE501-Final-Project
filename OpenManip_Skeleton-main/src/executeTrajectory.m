function [thetaHistory] = executeTrajectory(robot, q, targetPos, travelTime)
    i = 1;
    delta = inf;
    thetaHistory = zeros(1,4);
    while (delta > 3 || i < size(q, 1))
        readings = robot.getJointsReadings();
        thetaHistory = vertcat(thetaHistory, readings(1,:));
        if i >= size(q,1)
            delta = max(abs(abs(readings(1,:)) - abs(targetPos.')));
            
        else
            robot.writeJoints(q(i,:));
            pause(travelTime/100);
            delta = max(abs(abs(readings(1,:)) - abs(targetPos.')));
            i = i + 1;
        end
    end
    thetaHistory = thetaHistory(2:end, :);

end