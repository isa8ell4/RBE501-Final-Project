% create slist

function Slist = createSlist(w,p)
if ~isequal(size(w), size(p))
    error('Sizes of w and p are not equal.');
end
rows = size(w, 1);
Slist = zeros(6,rows);
for i = 1: rows
    t = twist(w(i,:),p(i, :));
    Slist(:,i)=t;

end