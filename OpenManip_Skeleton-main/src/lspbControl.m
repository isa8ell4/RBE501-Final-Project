function [qAC, qCB] = lspbControl(robot, thetasA, thetasB, thetasC, travelTime)
    % Linear Segment with Parabolic Blend Control
    robot.writeMode('cp')
    robot.writeJoints(thetasA.')
    pause(travelTime);
    
    % Calculations for Position Parabolas
    m = 100; % number of points
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
    tic;
    thetaHistoryAC = executeTrajectory(robot, qAC, thetasC, travelTime);
    pause(0.1);
    thetaHistoryCB = executeTrajectory(robot, qCB, thetasB, travelTime);
    totalTime = toc;
    thetaHistory = [thetaHistoryAC; thetaHistoryCB];
    % % % current graph
    rowsCR = size(thetaHistory, 1);
    timeXread = zeros(rowsCR, 1);
    currTime = 0;
    interval = totalTime/rowsCR;
    for i=1:rowsCR
        timeXread(i) = currTime;
        currTime = currTime + interval;
    end
    
    figure;


    plot(timeXread, thetaHistory(:, 1), 'r-', 'LineWidth', 1.5);
    hold on;
    plot(timeXread, thetaHistory(:, 2), 'b-', 'LineWidth', 1.5);
    plot(timeXread, thetaHistory(:, 3), 'g-', 'LineWidth', 1.5);
    plot(timeXread, thetaHistory(:, 4), 'm-', 'LineWidth', 1.5);
    xlabel('Time (s)');
    ylabel('Position (deg)');
    title('Position of Joints Traveling from A to C to B');

    legend({'J1', 'J2', 'J3', 'J4'}, ...
            'Location', 'best');
    
    grid on;
    hold off;

end
    

%     i = 1;
%     velReadingsAC = zeros(1, 4);
%     posReadingsAC = thetasA.';
%     
%     % % while the getJointsReadings are not at desired position, then keep moving
%     currPos = robot.getJointsReadings();
%     delta = inf;
%     tic;
%     
%     while (delta > 1 || i < size(qAC, 1))
%         readings = robot.getJointsReadings();
%         if i >= size(qAC,1)
%             velReadingsAC = vertcat(velReadingsAC, readings(2, :));
%             posReadingsAC = vertcat(posReadingsAC, readings(1,:));
%             delta = max(abs(readings(1,:)) - abs(thetasC.'));
%         else
%             robot.writeJoints(qAC(i,:));
%             velReadingsAC = vertcat(velReadingsAC, readings(2, :));
%             posReadingsAC = vertcat(posReadingsAC, readings(1,:));
%             pause(travelTime/100);
%             delta = max(abs(readings(1,:)) - abs(thetasC.'));
%             i = i+1;
%         end
%     end
%     totalTimeAC = toc;
%     disp('at pos c')
%     
%     pause(1);
%     
%     i = 1;
%     velReadingsCB = zeros(1, 4);
%     posReadingsCB = thetasC.';
%     
%     % % while the getJointsReadings are not at desired position, then keep moving
%     delta = inf;
%     tic;
%     
%     while (delta > 1 || i < size(qCB, 1))
%         readings = robot.getJointsReadings();
%         if i >= size(qCB,1)
%             velReadingsCB = vertcat(velReadingsCB, readings(2, :));
%             posReadingsCB = vertcat(posReadingsCB, readings(1,:));
%             delta = max(abs(readings(1,:)) - abs(thetasB.'));
%         else
%             robot.writeJoints(qCB(i,:));
%             velReadingsCB = vertcat(velReadingsCB, readings(2, :));
%             posReadingsCB = vertcat(posReadingsCB, readings(1,:));
%             pause(travelTime/100);
%             delta = max(abs(readings(1,:)) - abs(thetasB.'));
%             i = i+1;
%         end
%     end
%     totalTimeCB = toc;
%     disp('at pos c')
%     
%     pause(1);