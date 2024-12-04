%Exam 2

ws = [[0,0,1];
      [0,1,0];
      [0,1,0];
      [1,0,0];
      [0,1,0];
      [1,0,0]];
ps = [[0,0,0.495];
      [0.175,0,0.495];
      [0.175,0,0.495+1.095];
      [0.175,0,0.495+1.095+0.175];
      [0.175+1.2305,0,0.495+1.095+0.175];
      [0.175+1.2305+0.085,0,0.495+1.095+0.175]];

slist = createSlist(ws,ps);
% disp(slist);


% foward kinematics to get guesses
% plug in guess to Inverse Kinematics
% check thetalist with forward kinematics to see if matching T
T = [[0,1,0,  -0.97];
     [0.2,0,1, 0.69];
     [1,0,0.2, 2.3];
     [0,0,0,  1]];
% T1 = [[0.0321,0.5024,0.864,  -0.3051];
%      [0.2751,-0.8355,0.4756, 1.1485];
%      [0.9609,0.2224,-0.165, 2.3195];
%      [0,0,0,  1]];
% 
% T2 = [[-0.9777,0.1944,0.079,  -1.8302];
%      [-0.1193,-0.2053,-0.9714, -0.0826];
%      [-0.1726,-0.9592,0.2239, 1.7019];
%      [0,0,0,  1]];

T3 = [[-1,0,0,  0];
     [0,1,0, 1.1243];
     [0,0,-1, -0.0462];
     [0,0,0,  1]];

M = [0,0,1,1.4905;
     0,1,0,0;
     -1,0,0,1.765;
     0,0,0,1];
eomg = 0.001;
ev = 0.001;

% thetalist_guess = [deg2rad(90); deg2rad(-45);deg2rad(45);0;deg2rad(0);deg2rad(0)];
% disp(FKinSpace(M, slist, thetalist_guess));
% thetalist2 = IKinSpace(slist, M, T3, thetalist_guess,eomg, ev);
% disp(IKinSpace(slist, M, T3, thetalist_guess,eomg, ev));
% disp(FKinSpace(M, slist, thetalist2));
JacobianSpace(slist, [0;0;0;0;0;0])*[0;10;0;0;0;0]
b1 = twist([0,1,0], [-1.765+0.495, 0, -1.4905+0.175]);
% disp(50*1.765);
