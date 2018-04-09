function ans=self_collide(robot_num,p_r,r_robot)

d=0;
for i=1:robot_num-1
        k=i+1;
        for ii=k:robot_num
            %fprintf('counter:%d',i)
            %fprintf('%d\n',ii)
            check=collision(p_r(1:4,i),p_r(1:4,ii),r_robot(i),r_robot(ii));
            if check==1
               %fprintf('Self Collsion at %d\n:', s)
               d=1;
            end
        end
end

 if d==1
     ans=1;
 else
     ans=0;
 end 
 
end