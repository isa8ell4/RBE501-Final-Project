%practice Exam 2 

%disp(twist([0,1,0], [0,0,0.29]));

ws = [[0,0,1];
      [0,1,0];
      [0,1,0];
      [1,0,0];
      [0,1,0];
      [1,0,0]];
ps = [[0,0,0];
      [0,0,0.29];
      [0,0,0.6];
      [0,0,0.67];
      [0.3,0,0.67];
      [0.372,0,0.67]];

slist = createSlist(ws, ps);
% disp(slist);


thetaList = [0;0;0;0;deg2rad(90);0];

sJ = JacobianSpace(slist, thetaList);
% disp(sJ);
% disp(rank(sJ));

% p_desired = []
M = [0,0,1,0.372;
     0,1,0,0;
     -1,0,0,0.67;
     0,0,0,1];

L = 10;
bList = [[0;0;1;0;-L;L],...
    [1;0;0;0;0;L],...
    [0;0;1;0;0;0],...
    [0;0;0;0;1;0]];
thetaList2 = [0;0;deg2rad(90);10];

disp(JacobianBody(bList, thetaList2));



