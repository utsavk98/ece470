function ans=obj_collide(robot_num,num_ob,p_r,p_obstacle,r_robot,r_obstacle)
   d=0;
    for robot=1:robot_num
        for obstacle=1:num_ob
            check2=collision(p_r(:,robot),p_obstacle(:,obstacle),r_robot(robot),r_obstacle(obstacle));
            if check2==1
                %fprintf('Collision between %d%d\n',robot, obstacle)
                %fprintf('Collision occured at:%d\n', s)
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