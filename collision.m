
function ans=collision(p1,p2,r1,r2)
p2=p2(1:3);
p1=p1(1:3);
d=norm(p2-p1);
d=abs(d);

r=r1+r2;

%where collision=0 is no collision and collision=1 is collision
if r>=d
    ans=1;
else
    ans=0;   
end

end