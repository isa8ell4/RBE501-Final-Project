%% Part a
disp('Part a')
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
         [0; 1; 0; -(l1+l2+l3z); 0; l3x+l4]]

M = [0,0,-1,l3x+l4+l5;
     0,1,0,0;
     1,0,0,l1+l2+l3z;
     0,0,0,1];
%% Part b
disp('Part b')
T1 = [1,0,0,0.15;
     0,1,0,0;
     0,0,1,0.05;
     0,0,0,1];

eomg = 0.001;
ev = 0.001;
thetalist = [deg2rad(0);0;deg2rad(15);deg2rad(80)];
FKinSpace(M, slist, thetalist)
thetalist2 = IKinSpace(slist, M, T1, thetalist, eomg, ev);
rad2deg(thetalist2)
FKinSpace(M, slist, thetalist2)

%% Part d
disp('Part d')
M01 = [1, 0, 0, 0;...
       0, 1, 0, 0;...
       0, 0, 1, 0.037136;...
       0, 0, 0, 1];

M12 = [1, 0, 0, 0;...
       0, 1, 0, 0;...
       0, 0, 1, 0.0452;...
       0, 0, 0, 1];

M23 = [1, 0, 0, 0.0025;...
       0, 1, 0, -0.00125477;...
       0, 0, 1, 0.11061355;...
       0, 0, 0, 1];

M34 = [1, 0, 0, 0.106;...
       0, 1, 0, 0;...
       0, 0, 1, 0.03384191;...
       0, 0, 0, 1];

M45 = [0, 0, -1, 0.10595287;...
       0, 1, 0, 0;...
       1, 0, 0, 0.00246;...
       0, 0, 0, 1];

M_total = M01*M12*M23*M34*M45

%% Part e and f

disp('Part f')

G1 = diag([2.31*1e-4, 2.31*1e-4, 2.98*1e-4, 0.305, 0.305,0.305]);
G2 = diag([1.85*1e-5, 2.95*1e-5, 3.26*1e-5, 0.094, 0.094, 0.094]);
G3 = diag([3.5*1e-5, 1.7*1e-4, 1.7*1e-4, 0.095, 0.095, 0.095]);
G4 = diag([3.47*1e-5, 1.71*1e-4, 1.75*1e-4, 0.079, 0.079,0.079]);


dthetalist = [0;0;0;0];
ddthetalist = [0;0;0;0];
g = [0; 0; -9.8];
Ftip = [0;0;0;0;0;-10];

Glist = cat(4, G1, G2, G3, G4);
Mlist = cat(4, M01, M12, M23, M34, M45);

tauList = InverseDynamics(thetalist2, dthetalist, ddthetalist, g, Ftip, Mlist, Glist, slist)






