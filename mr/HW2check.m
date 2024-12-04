%HW2 check
syms t1 t2 t3
thetaList = [0;0;0];
Slist1 = [[-1; 0;  0;  0; -10;    -25],... 
       [0; -1;  0;  20; 0;    0],...
       [1; 0; 0; 0; -45; (25+30*tan(deg2rad(30)))]];
M1 = [[1, 0, 0, 0]; 
    [0, 0, -1, -(15+30*tan(deg2rad(30)))]; 
    [0, 1, 0, -55]; 
    [0, 0, 0, 1]];

% disp(FkPoe(Slist1, M1, thetaList));
disp(cross([1;0;0], [0;-25;10]));
disp(cross([0; 1; 0], [0; -(25 + 30 * tan(deg2rad(30))); -20])); 
disp(cross([-1;0;0], [0; -(25 + 30 * tan(deg2rad(30))); -45]));