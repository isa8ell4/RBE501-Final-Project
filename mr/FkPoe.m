function T = FkPoe(twists, M, thetas)
T = M;
[~, numT] = size(twists);
for c = 1:numT
    w = twists(1:3, c);
    v = twists(4:6, c);
    t = thetas(c);
    e = [Rodrigues(w, t), Ramirez(w,v,t);
        0, 0, 0, 1];
    T = T*e;
end


end

