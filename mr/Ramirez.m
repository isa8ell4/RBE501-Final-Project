function Ra = Ramirez(w, v, theta)
bracketW=SkewSymmetric(w);
Ra = (eye(3)*theta + (1-cos(theta))*bracketW+(theta-sin(theta))*bracketW^2)*v;

end