function [avgCurr] = getCurrAtPos(thetas)
robot.writeMode('cp')
robot.writeJoints(thetas); % Write joints to zero position
pause(10);


currReadings = zeros(1,4);
tic;
while toc < 5
    readings = robot.getJointsReadings();
    currReadings = vertcat(currReadings, readings(3, :));
end
currReadings = currReadings(2:end, :);
avgCurrent = [mean(currReadings(:,1)), mean(currReadings(:,2)), mean(currReadings(:,3)), mean(currReadings(:,4))];

end

