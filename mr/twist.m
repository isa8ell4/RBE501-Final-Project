%create twist or screw

function twist = twist(w,p)

a = cross(-w,p);
twist = [w, a];

end