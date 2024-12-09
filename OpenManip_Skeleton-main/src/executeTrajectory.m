function executeTrajectory(robot, q, targetPos, travelTime)
    i = 1;
    delta = inf;
    while (delta > 1 || i < size(q, 1))
        readings = robot.getJointsReadings();
        if i >= size(q,1)
            delta = max(abs(readings(1,:)) - abs(targetPos.'));
        else
            robot.writeJoints(q(i,:));
            pause(travelTime/100);
            delta = max(abs(readings(1,:)) - abs(targetPos.'));
            i = i + 1;
        end
    end
end