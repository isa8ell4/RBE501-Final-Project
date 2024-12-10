function [dInput] = getDerivative(input, totalTime)
    s = size(input);
    
    dInput = zeros(s(1)-1, s(2));
    
    timeStep = totalTime / (s(1) - 1);
    
    for i = 1:(s(1) - 1)
        curr = input(i, :);
        next = input(i + 1, :);
        dInput(i, :) = (next - curr) / timeStep;
    end
end

