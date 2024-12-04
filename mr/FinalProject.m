ws = [[0,0,1];
      [0,1,0];
      [0,1,0];
      [0,1,0]];
l1 = convlength(1.50373, 'in', 'm');
l2 = convlength(3.79237, 'in', 'm');
l3 = convlength(5.12719, 'in', 'm');
l3x = convlength(0.94488, 'in', 'm');
l3z = convlength(5.03937, 'in', 'm');
l4 = convlength(4.88189, 'in', 'm');
l5 = convlength(5.76546, 'in', 'm');


slist = [[0; 0; 1; 0;            0; 0     ]...
         [0; 1; 0; -(l1+l2);     0; 0     ]...
         [0; 1; 0; -(l1+l2+l3z); 0; l3x   ]...
         [0; 1; 0; -(l1+l2+l3z); 0; l3x+l4]];

M = [0,0,-1,l3x+l4+l5;
     0,1,0,0;
     1,0,0,l1+l2+l3z;
     0,0,0,1];
%% Task 1 part a
Ta = [0,0.7071,0.7071,0.185;
     0,0.7071,-0.7071,-0.185;
     1,0,     0,      0.185;
     0,0,0,1];

eomg = 0.001;
ev = 0.001;
thetalistGuess = [deg2rad(-45);deg2rad(-43.3925);deg2rad(71.8158);deg2rad(-28.4233)];
%to get answer uncomment
FKinSpace(M, slist, thetalistGuess)
thetalist2 = IKinSpace(slist, M, Ta, thetalistGuess, eomg, ev);
rad2deg(thetalist2)
FKinSpace(M, slist, thetalist2)

%% Task 1 part b

%intermediate step to find all thetas except for theta 1
Tb_inter = [0,0,-1,0.185;
            0,1,0,0;
            1,0,0,0.070;
            0,0,0,1];

thetaListGb = [deg2rad(0); deg2rad(-43.3925); deg2rad(71.8158);deg2rad(-28.4233)];
% find theta1 using istan's calculations and then plug into to find R of Tb
% FKinSpace(M, slist, thetaListGb)
% thetalist2b = IKinSpace(slist, M, Tb_inter, thetaListGb, eomg, ev);
% rad2deg(thetalist2b)
% FKinSpace(M, slist, thetalist2b)

% plug in R and y translation from geometry calculations
Tb_inter2 = [0,-0.6766,-0.7363,0.185;
            0,0.7363,-0.6766,0.17;
            1,0,0,0.070;
            0,0,0,1];
% plug in theta1 from calculations and other thetas from prev step
thetaListB = [deg2rad(42.58049078); deg2rad(-153.1435); deg2rad(134.7984); deg2rad(18.3452)];
FKinSpace(M, slist, thetaListB)
thetalist2b2 = IKinSpace(slist, M, Tb_inter2, thetaListB, eomg, ev);
rad2deg(thetalist2b2)
FKinSpace(M, slist, thetalist2b2)