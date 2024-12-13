function [coefficients]=currentTorqueRelationship(avgCurrent,torques)
%CURRENTTORQUERELATIONSHIP Summary of this function goes here
%   Detailed explanation goes here

% Scatter plot
% figure;
% scatter(avgCurrent, torques, 'filled'); % Scatter plot with filled markers
% hold on;

% Line of best fit (linear regression)
coefficients = polyfit(avgCurrent, torques, 1);% First-order polynomial fit
line_x = linspace(min(avgCurrent), max(avgCurrent), 100); % Generate x-values for the line
line_y = polyval(coefficients, line_x); % Compute y-values for the line

% Plot the line of best fit
% plot(line_x, line_y, 'r-', 'LineWidth', 1.5);
% 
% % Display the equation of the line
% equation = sprintf('y = %.3fx + %.3f', coefficients(1), coefficients(2));
% text(mean(avgCurrent), mean(torques), equation, 'FontSize', 12, 'Color', 'blue', 'HorizontalAlignment', 'left', 'VerticalAlignment', 'top');
% 
% % Labeling the plot
% xlabel('Current(A)');
% ylabel('Torque (Nm)');
% title('Current-Torque Relationship at Joints 1-4');
% legend('Data points', 'Best fit line');
% grid on;
% 
% hold off;
end

