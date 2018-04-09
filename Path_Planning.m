%Path Planning!
clear
clc

S = [0.00 0.00 0.00 0.00 0.00 0.00; 0.00 1.00 1.00 0.00 1.00 0.00; 0.00 0.00 0.00 -1.00 0.00 0.00; 0.00 2.00 4.00 0.00 6.00 0.00; 0.00 0.00 0.00 0.00 0.00 1.00; -1.00 -2.00 0.00 0.00 -2.00 0.00];
M = [0.00 0.00 1.00 -4.00; 1.00 0.00 0.00 0.00; 0.00 1.00 0.00 -2.00; 0.00 0.00 0.00 1.00];
p_robot = [0.00 -2.00 -2.00 0.00 0.00 -2.00 -4.00 -4.00; 0.00 0.00 0.00 0.00 0.00 0.00 0.00 0.00; 0.00 0.00 -2.00 -4.00 -6.00 -6.00 -4.00 -2.00];
r_robot = [0.90 0.90 0.90 0.90 0.90 0.90 0.90 0.90];
p_obstacle = [0.78 -1.63 1.76 -4.28 -0.43 1.65 -4.04 -2.95 2.41 -2.57 -1.26 0.96 -1.75 -1.12 -4.65; 2.48 1.69 -1.82 1.28 -1.46 -4.49 0.63 3.98 -4.95 -1.85 4.88 -1.58 3.30 -2.13 -4.22; 4.18 4.86 -2.69 4.88 2.24 4.85 2.27 3.61 -2.92 -3.78 -4.30 3.72 0.96 -3.54 1.83];
r_obstacle = [1.57 1.33 0.82 3.46 0.72 1.47 1.39 2.17 2.99 0.71 1.59 2.25 2.19 1.47 3.40];
theta_start = [0.11; 0.82; -2.80; 1.92; -2.10; 2.41];
theta_goal = [0.64; 3.07; -1.98; 1.17; -0.71; -0.36];


num_ob=15;
robot_num=8;
p_obstacle=[p_obstacle; ones(1,num_ob)];
joint_num=6;

ss1=s_screw(S(:,1));
ss2=s_screw(S(:,2));
ss3=s_screw(S(:,3));
ss4=s_screw(S(:,4));
ss5=s_screw(S(:,5));
ss6=s_screw(S(:,6));
%ss7=s_screw(S(:,7));

%Step 1 - Check a straight line path between theta start and theta goal to
%see if there is collision!
d=0;
for s=0:.01:1
    
    theta_now=(1-s)*theta_start+s*theta_goal;
    
    
    j1=expm(ss1*theta_now(1));
    j2=expm(ss2*theta_now(2));
    j3=expm(ss3*theta_now(3));
    j4=expm(ss4*theta_now(4));
    j5=expm(ss5*theta_now(5));
    j6=expm(ss6*theta_now(6));
    %j7=expm(ss7*theta_now(7));
    %
    T_end=j1*j2*j3*j4*j5*j6*M; %BE CAREFUL ON THIS LINE!!
    
    %p1 doesn't change
    %p2 doesn't change
    p_r_new=[p_robot; ones(1,robot_num)];
    p3=j1*p_r_new(1:4,3);
    p4=j1*j2*p_r_new(1:4,4);
    p5=j1*j2*j3*p_r_new(1:4,5);
    p6=j1*j2*j3*j4*p_r_new(1:4,6);
    p7=j1*j2*j3*j4*j5*p_r_new(1:4,7);
    %p8=j1*j2*j3*j4*j5*j6*p_r_new(1:4,8);
    pend=T_end(1:4,4);
    
    p_r=[p_r_new(:,1) p_r_new(:,2) p3 p4 p5 p6 p7 pend]; % BE CAREFUL ON THIS LINE
    
    %Collision Checker - if the robot is in self-collision
    check=self_collide(robot_num,p_r,r_robot); %this function checks each joint of the robot against itself at a given configuration
    if check==1
        d=1;
    end
    % Now - let's check to see if the robot hits any of the silly little
    % obstacle spheres!
    
    check2=obj_collide(robot_num,num_ob,p_r,p_obstacle,r_robot,r_obstacle); %this function checks each joint against each object at a given config
    if check2==1
        d=1;
    end
    
end

% Run Path Planner!!!
s_t=0;
e_t=0;
col1_s=1;
col2_s=2;
col1_e=1;
col2_e=2;
if d==1 %this means that the initial path did not work!
    %Set up our zero nodes - the start and goal poses
    s_line=[s_t;zeros(joint_num-1,1)];
    start_tree(:,col1_s:col2_s)=[theta_start s_line];
    s_t=s_t+1;
    col1_s=col1_s+2;
    col2_s=col2_s+2;
    e_line=[e_t;zeros(joint_num-1,1)];
    end_tree(:,col1_e:col2_e)=[theta_goal e_line];
    e_t=e_t+1;
    col1_e=col1_e+2;
    col2_e=col2_e+2;
    condition=0;
    count=0;
    while condition==0 %%% -- NEED TO HAVE A WHILE LOOP HERE -------------------------------
        count=count+1;
        %A - pick a theta value at random from -pi to + pi, 7 values
        t_r=pi*rand(joint_num,1)-pi*rand(joint_num,1);
        %t_r=theta_start*.999;
        
        %A.2 Do Forward Kinematics of this point
        j1=expm(ss1*t_r(1));
        j2=expm(ss2*t_r(2));
        j3=expm(ss3*t_r(3));
        j4=expm(ss4*t_r(4));
        j5=expm(ss5*t_r(5));
        j6=expm(ss6*t_r(6));
        %j7=expm(ss7*t_r(7));
        
        T_end=j1*j2*j3*j4*j5*j6*M; %BE CAREFUL ON THIS LINE!!
        
        p_r_new=[p_robot; ones(1,robot_num)];
        p3=j1*p_r_new(1:4,3);
        p4=j1*j2*p_r_new(1:4,4);
        p5=j1*j2*j3*p_r_new(1:4,5);
        p6=j1*j2*j3*j4*p_r_new(1:4,6);
        p7=j1*j2*j3*j4*j5*p_r_new(1:4,7);
        %p8=j1*j2*j3*j4*j5*j6*p_r_new(1:4,8);
        pend=T_end(1:4,4);
        
        p_r=[p_r_new(:,1) p_r_new(:,2) p3 p4 p5 p6 p7 pend]; % BE CAREFUL ON THIS LINE
        
        
        %B - determine if there is a collision at this value
        self_check=self_collide(robot_num,p_r,r_robot);
        ob_check=obj_collide(robot_num,num_ob,p_r,p_obstacle,r_robot,r_obstacle);
        
        if self_check==0 && ob_check==0  %C - if no collision, continue forward
            fprintf('No Collision\n')
            %D - determine which node this node is closests to
            %check start tree
            min_norms=1000;
            for i=1:s_t
                x=(i-1)+i;
                norm_s(i)=norm(t_r-start_tree(:,x));
                if norm_s(i)<min_norms
                    if norm_s(i)<min_norms
                        min_norms=norm_s(i);
                        node_s=i;
                        col_s=x;
                    end
                end
            end
            %check end tree
            min_norme=1000;
            for ii=1:e_t
                xx=(ii-1)+ii;
                norm_e(ii)=norm(t_r-end_tree(:,xx));
                if norm_e(ii)<min_norme
                    min_norme=norm_e(ii);
                    node_e=ii;
                    col_e=xx;
                end
            end
            
            alpha=min(min_norms,min_norme);
            
            if alpha==min_norms %that means the node is closer to a node on the start tree
                %the node we're comparing to is listed as node_s
                %which is located in col_s= column of the start tree
                theta_end=start_tree(:,col_s);
            else %this means the node is closer to a node on the end tree
                %the node we're comparing to is listed as node_e
                %which is located in col_e= column of the end tree
                theta_end=end_tree(:,col_e);
            end
            
            %E - determine if node can be connected via straight line to
            %closest node
            e=0;
            for s=0:.01:1
                
                theta_test=(1-s)*t_r+s*theta_end;
                
                j1=expm(ss1*theta_test(1));
                j2=expm(ss2*theta_test(2));
                j3=expm(ss3*theta_test(3));
                j4=expm(ss4*theta_test(4));
                j5=expm(ss5*theta_test(5));
                j6=expm(ss6*theta_test(6));
                %j7=expm(ss7*theta_test(7));
                %
                T_end=j1*j2*j3*j4*j5*j6*M; %BE CAREFUL ON THIS LINE!!
                
                p_r_new=[p_robot; ones(1,robot_num)];
                p3=j1*p_r_new(1:4,3);
                p4=j1*j2*p_r_new(1:4,4);
                p5=j1*j2*j3*p_r_new(1:4,5);
                p6=j1*j2*j3*j4*p_r_new(1:4,6);
                p7=j1*j2*j3*j4*j5*p_r_new(1:4,7);
                %p8=j1*j2*j3*j4*j5*j6*p_r_new(1:4,8);
                pend=T_end(1:4,4);
                
                p_r=[p_r_new(:,1) p_r_new(:,2) p3 p4 p5 p6 p7 pend]; % BE CAREFUL ON THIS LINE
                
                %Collision Checker - if the robot is in self-collision
                check3=self_collide(robot_num,p_r,r_robot); %this function checks each joint of the robot against itself at a given configuration
                if check3==1
                    e=1;
                end
                % Now - let's check to see if the robot hits any of the silly little
                % obstacle spheres!
                
                check4=obj_collide(robot_num,num_ob,p_r,p_obstacle,r_robot,r_obstacle); %this function checks each joint against each object at a given config
                if check4==1
                    e=1;
                end
                
            end
            if e==0 %there was no collision on the path
                if alpha==min_norms %attach it to the start tree & check if you can connect it to the closest end node through a straight line
                    s_line=[node_s;zeros(joint_num-1,1)];
                    start_tree(:,col1_s:col2_s)=[t_r s_line];
                    s_t=s_t+1;
                    col1_s=col1_s+2;
                    col2_s=col2_s+2;
                    f=0;
                    for s=0:.01:1
                        theta_test1=(1-s)*t_r+s*end_tree(:,col_e);
                        
                        j1=expm(ss1*theta_test1(1));
                        j2=expm(ss2*theta_test1(2));
                        j3=expm(ss3*theta_test1(3));
                        j4=expm(ss4*theta_test1(4));
                        j5=expm(ss5*theta_test1(5));
                        j6=expm(ss6*theta_test1(6));
                        %j7=expm(ss7*theta_test1(7));
                        %
                        T_end=j1*j2*j3*j4*j5*j6*j7*M; %BE CAREFUL ON THIS LINE!!
                        
                        p_r_new=[p_robot; ones(1,robot_num)];
                        p3=j1*p_r_new(1:4,3);
                        p4=j1*j2*p_r_new(1:4,4);
                        p5=j1*j2*j3*p_r_new(1:4,5);
                        p6=j1*j2*j3*j4*p_r_new(1:4,6);
                        p7=j1*j2*j3*j4*j5*p_r_new(1:4,7);
                        %p8=j1*j2*j3*j4*j5*j6*p_r_new(1:4,8);
                        pend=T_end(1:4,4);
                        
                        p_r=[p_r_new(:,1) p_r_new(:,2) p3 p4 p5 p6 p7 pend]; % BE CAREFUL ON THIS LINE
                        
                        %Collision Checker - if the robot is in self-collision
                        check3=self_collide(robot_num,p_r,r_robot); %this function checks each joint of the robot against itself at a given configuration
                        if check3==1
                            f=1;
                        end
                        % Now - let's check to see if the robot hits any of the silly little
                        % obstacle spheres!
                        
                        check4=obj_collide(robot_num,num_ob,p_r,p_obstacle,r_robot,r_obstacle); %this function checks each joint against each object at a given config
                        if check4==1
                            f=1;
                        end
                    end
                    if f==0
                        %set my condition here =to like 1
                        fprintf('It connects, cool!\n')
                        fprintf('Final node on start tree connects to node on end tree %d\n',node_e)
                        finish=1;
                    end
                    
                    
                else % attach it to the end tree & check if you can connect it to the closest start node through a straight line
                    e_line=[node_e;zeros(joint_num-1,1)];
                    end_tree(:,col1_e:col2_e)=[t_r e_line];
                    e_t=e_t+1;
                    col1_e=col1_e+2;
                    col2_e=col2_e+2;
                    g=0;
                    for s=0:.01:1
                        theta_test1=(1-s)*t_r+s*start_tree(:,col_s);
                        
                        j1=expm(ss1*theta_test1(1));
                        j2=expm(ss2*theta_test1(2));
                        j3=expm(ss3*theta_test1(3));
                        j4=expm(ss4*theta_test1(4));
                        j5=expm(ss5*theta_test1(5));
                        j6=expm(ss6*theta_test1(6));
                        %j7=expm(ss7*theta_test1(7));
                        %
                        T_end=j1*j2*j3*j4*j5*j6*M; %BE CAREFUL ON THIS LINE!!
                        
                        p_r_new=[p_robot; ones(1,robot_num)];
                        p3=j1*p_r_new(1:4,3);
                        p4=j1*j2*p_r_new(1:4,4);
                        p5=j1*j2*j3*p_r_new(1:4,5);
                        p6=j1*j2*j3*j4*p_r_new(1:4,6);
                        p7=j1*j2*j3*j4*j5*p_r_new(1:4,7);
                        %p8=j1*j2*j3*j4*j5*j6*p_r_new(1:4,8);
                        pend=T_end(1:4,4);
                        
                        p_r=[p_r_new(:,1) p_r_new(:,2) p3 p4 p5 p6 p7 pend]; % BE CAREFUL ON THIS LINE
                        
                        %Collision Checker - if the robot is in self-collision
                        check3=self_collide(robot_num,p_r,r_robot); %this function checks each joint of the robot against itself at a given configuration
                        if check3==1
                            g=1;
                        end
                        % Now - let's check to see if the robot hits any of the silly little
                        % obstacle spheres!
                        
                        check4=obj_collide(robot_num,num_ob,p_r,p_obstacle,r_robot,r_obstacle); %this function checks each joint against each object at a given config
                        if check4==1
                            g=1;
                        end
                    end
                    if g==0
                        condition=1;
                        fprintf('It connects, cool!\n')
                        fprintf('Final node on end tree connects to node on start tree %d\n',node_s)
                        finish=2;
                    end
                    
                end
                
            end
            
        else
            fprintf('Random Point Generated is In Collision\n')
        end
        
        if count==1000
            condition=1;
        end
    end %%% --- END OF THE WHILE LOOP HERE ------------------------------------
end

start_tree
end_tree




