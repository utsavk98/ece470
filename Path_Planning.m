%Path Planning!
clear
clc

S = [-1.00 0.00 0.00; 0.00 0.00 1.00; 0.00 -1.00 0.00; 0.00 0.00 2.00; -2.00 -2.00 0.00; 0.00 0.00 0.00];
M = [-1.00 0.00 0.00 0.00; 0.00 1.00 0.00 0.00; 0.00 0.00 -1.00 -4.00; 0.00 0.00 0.00 1.00];
p_robot = [0.00 0.00 -2.00 0.00 0.00; 0.00 0.00 0.00 0.00 0.00; 0.00 2.00 2.00 -2.00 -4.00];
r_robot = [0.90 0.90 0.90 0.90 0.90];
p_obstacle = [4.83 2.68 2.46 3.25 4.10 2.69 1.81 3.87 4.74 1.53 4.51 4.98 3.63 1.42 3.52; 4.10 4.98 2.66 -2.46 4.78 -1.99 1.98 3.13 2.32 2.22 -2.75 -2.72 1.28 4.34 -4.11; 4.60 4.27 -3.42 -1.91 -3.37 -2.10 -3.97 -3.87 -3.05 -2.66 -2.64 -3.15 1.96 2.68 -2.20];
r_obstacle = [1.21 2.14 1.52 0.91 4.77 2.50 1.31 1.76 2.83 1.64 2.72 4.28 2.92 1.54 0.64];
theta_start = [-2.68; 1.49; 0.26];
theta_goal = [1.60; -0.20; -1.98];


num_ob=15;
robot_num=5;
p_obstacle=[p_obstacle; ones(1,num_ob)];
joint_num=3;

ss1=skew4(S(:,1));
ss2=skew4(S(:,2));
ss3=skew4(S(:,3));
% ss4=skew4(S(:,4));
% ss5=skew4(S(:,5));
% ss6=skew4(S(:,6));
%ss7=s_screw(S(:,7));

%Step 1 - Check a straight line path between theta start and theta goal to
%see if there is collision!
d=0;
for s=0:.01:1
    
    theta_now=(1-s)*theta_start+s*theta_goal;
    
    
    j1=expm(ss1*theta_now(1));
    j2=expm(ss2*theta_now(2));
    j3=expm(ss3*theta_now(3));
%     j4=expm(ss4*theta_now(4));
%     j5=expm(ss5*theta_now(5));
%     j6=expm(ss6*theta_now(6));
%     %j7=expm(ss7*theta_now(7));
    %
    T_end=j1*j2*j3*M; %BE CAREFUL ON THIS LINE!!
    
    %p1 doesn't change
    %p2 doesn't change
    p_r_new=[p_robot; ones(1,robot_num)];
    p3=j1*p_r_new(1:4,3);
    p4=j1*j2*p_r_new(1:4,4);
    p5=j1*j2*j3*p_r_new(1:4,5);
%     p6=j1*j2*j3*j4*p_r_new(1:4,6);
%     p7=j1*j2*j3*j4*j5*p_r_new(1:4,7);
    %p8=j1*j2*j3*j4*j5*j6*p_r_new(1:4,8);
    pend=T_end(1:4,4);
    
    p_r=[p_r_new(:,1) p_r_new(:,2) p3 p4 p5 pend]; % BE CAREFUL ON THIS LINE
    
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
%         j4=expm(ss4*t_r(4));
%         j5=expm(ss5*t_r(5));
%         j6=expm(ss6*t_r(6));
        %j7=expm(ss7*t_r(7));
        
        T_end=j1*j2*j3*M; %BE CAREFUL ON THIS LINE!!
        
        p_r_new=[p_robot; ones(1,robot_num)];
        p3=j1*p_r_new(1:4,3);
        p4=j1*j2*p_r_new(1:4,4);
        p5=j1*j2*j3*p_r_new(1:4,5);
%         p6=j1*j2*j3*j4*p_r_new(1:4,6);
%         p7=j1*j2*j3*j4*j5*p_r_new(1:4,7);
        %p8=j1*j2*j3*j4*j5*j6*p_r_new(1:4,8);
        pend=T_end(1:4,4);
        
        p_r=[p_r_new(:,1) p_r_new(:,2) p3 p4 p5 pend]; % BE CAREFUL ON THIS LINE
        
        
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
%                 j4=expm(ss4*theta_test(4));
%                 j5=expm(ss5*theta_test(5));
%                 j6=expm(ss6*theta_test(6));
                %j7=expm(ss7*theta_test(7));
                %
                T_end=j1*j2*j3*M; %BE CAREFUL ON THIS LINE!!
                
                p_r_new=[p_robot; ones(1,robot_num)];
                p3=j1*p_r_new(1:4,3);
                p4=j1*j2*p_r_new(1:4,4);
                p5=j1*j2*j3*p_r_new(1:4,5);
%                 p6=j1*j2*j3*j4*p_r_new(1:4,6);
%                 p7=j1*j2*j3*j4*j5*p_r_new(1:4,7);
                %p8=j1*j2*j3*j4*j5*j6*p_r_new(1:4,8);
                pend=T_end(1:4,4);
                
                p_r=[p_r_new(:,1) p_r_new(:,2) p3 p4 p5 pend]; % BE CAREFUL ON THIS LINE
                
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
%                         j4=expm(ss4*theta_test1(4));
%                         j5=expm(ss5*theta_test1(5));
%                         j6=expm(ss6*theta_test1(6));
                        %j7=expm(ss7*theta_test1(7));
                        %
                        T_end=j1*j2*j3*M; %BE CAREFUL ON THIS LINE!!
                        
                        p_r_new=[p_robot; ones(1,robot_num)];
                        p3=j1*p_r_new(1:4,3);
                        p4=j1*j2*p_r_new(1:4,4);
                        p5=j1*j2*j3*p_r_new(1:4,5);
%                         p6=j1*j2*j3*j4*p_r_new(1:4,6);
%                         p7=j1*j2*j3*j4*j5*p_r_new(1:4,7);
                        %p8=j1*j2*j3*j4*j5*j6*p_r_new(1:4,8);
                        pend=T_end(1:4,4);
                        
                        p_r=[p_r_new(:,1) p_r_new(:,2) p3 p4 p5 pend]; % BE CAREFUL ON THIS LINE
                        
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
                        [garbage,node_s] = size(start_tree);
                        node_e = node_e*2;
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
%                         j4=expm(ss4*theta_test1(4));
%                         j5=expm(ss5*theta_test1(5));
%                         j6=expm(ss6*theta_test1(6));
                        %j7=expm(ss7*theta_test1(7));
                        %
                        T_end=j1*j2*j3*M; %BE CAREFUL ON THIS LINE!!
                        
                        p_r_new=[p_robot; ones(1,robot_num)];
                        p3=j1*p_r_new(1:4,3);
                        p4=j1*j2*p_r_new(1:4,4);
                        p5=j1*j2*j3*p_r_new(1:4,5);
%                         p6=j1*j2*j3*j4*p_r_new(1:4,6);
%                         p7=j1*j2*j3*j4*j5*p_r_new(1:4,7);
                        %p8=j1*j2*j3*j4*j5*j6*p_r_new(1:4,8);
                        pend=T_end(1:4,4);
                        
                        p_r=[p_r_new(:,1) p_r_new(:,2) p3 p4 p5 pend]; % BE CAREFUL ON THIS LINE
                        
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
                        [garbage,node_e] = size(end_tree);
                        node_s = node_s*2;
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

parent = end_tree(1,node_e);
final_tree = [end_tree(:,node_e-1)];
while parent~=0
    final_tree = [final_tree end_tree(:,parent*2-1)];
    parent = end_tree(1,parent*2);
end

parent = start_tree(1,node_s);
final_tree = [start_tree(:,node_s-1) final_tree];
while parent~=0
    final_tree = [start_tree(:,parent*2-1) final_tree];
    parent = start_tree(1,parent*2);
end


final_tree







