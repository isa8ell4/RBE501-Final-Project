function Ro = Rodrigues(w, theta)
bracketW=SkewSymmetric(w);
Ro = eye(3) + sin(theta)*bracketW + (1-cos(theta))*bracketW^2;

end