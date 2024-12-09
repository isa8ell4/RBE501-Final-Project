function jointVel = velPID(currentPos, desiredPos, currentVel, Kp, Ki, Kd, dt, integralError)
    % PIDControl - Computes joint velocities using a PID controller.
    %
    % Inputs:
    %   currentPos    - 1x4 matrix of current joint positions
    %   desiredPos    - 1x4 matrix of desired joint positions
    %   currentVel    - 1x4 matrix of current joint angular velocities
    %   Kp            - 1x4 matrix of proportional gains
    %   Ki            - 1x4 matrix of integral gains
    %   Kd            - 1x4 matrix of derivative gains
    %   dt            - Time step (scalar)
    %   integralError - 1x4 matrix of cumulative position errors (integral term)
    %
    % Output:
    %   jointVel      - 1x4 matrix of computed joint angular velocities

    % Compute position error
    error = desiredPos - currentPos;
    
    % Update integral term
    integralError = integralError + error * dt;
    
    % Compute derivative term
    derivativeError = -currentVel; % Angular velocity is derivative of position
    
    % PID formula for angular velocity
    jointVel = Kp .* error + Ki .* integralError + Kd .* derivativeError;
end
