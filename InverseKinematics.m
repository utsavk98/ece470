%%% Inverse Kinematics
clear
clc

%Step 1: Inputs - a/q, M, goal pose (Tg)
Tg = [-0.38060736 -0.91447455 -0.13738391 7.17721470; 0.48225213 -0.32304828 0.81429276 2.16662567; -0.78903163 0.24367213 0.56396185 -3.07117349; 0.00000000 0.00000000 0.00000000 1.00000000];
M = [1.00000000 0.00000000 0.00000000 6.00000000; 0.00000000 0.00000000 1.00000000 0.00000000; 0.00000000 -1.00000000 0.00000000 8.00000000; 0.00000000 0.00000000 0.00000000 1.00000000];
S = [0.00000000 0.00000000 1.00000000 0.00000000 0.00000000 0.00000000; 1.00000000 0.00000000 0.00000000 0.00000000 1.00000000 0.00000000; 0.00000000 1.00000000 0.00000000 0.00000000 0.00000000 0.00000000; -2.00000000 0.00000000 0.00000000 0.00000000 -4.00000000 0.00000000; 0.00000000 -2.00000000 2.00000000 0.00000000 0.00000000 1.00000000; 0.00000000 0.00000000 0.00000000 1.00000000 4.00000000 0.00000000];

%Step 2: Calculate Screws & screw brackets
ss1=s_screw(S(:,1));
ss2=s_screw(S(:,2));
ss3=s_screw(S(:,3));
ss4=s_screw(S(:,4));
ss5=s_screw(S(:,5));
ss6=s_screw(S(:,6));

%Step 3: Pick start theta values
theta=[pi/4;pi/4;pi/4;pi/4;pi/4;pi/4];

%Step 4: Do loop
tol=.1;
error=20;
while error>tol
    % Step 5: find current pose (Tc)
    j1=expm(ss1*theta(1));
    j2=expm(ss2*theta(2));
    j3=expm(ss3*theta(3));
    j4=expm(ss4*theta(4));
    j5=expm(ss5*theta(5));
    j6=expm(ss6*theta(6));
    
    Tc=j1*j2*j3*j4*j5*j6*M;
    % Step 6: find [v]=log(Tg(Tc)^-1)
    vb=logm(Tg*inv(Tc));
    v=[vb(3,2);vb(1,3);vb(2,1);vb(1:3,4)];
    % Step 7: find space jacobian
    %Column 1
    C1=S(:,1);
    %Column 2
    part2=adj(j1);
    C2=part2*S(:,2);
    %Column 3
    part3=adj(j1*j2);
    C3=part3*S(:,3);
    % %Column 4
    part4=adj(j1*j2*j3);
    C4=part4*S(:,4);
    % %Column 5
    part5=adj(j1*j2*j3*j4);
    C5=part5*S(:,5);
    
    %Column 6
    part6=adj(j1*j2*j3*j4*j5);
    C6=part6*S(:,6);
    
    space=[C1 C2 C3 C4 C5 C6];
    
    % Step 8: find thetadots, td=(J^-1)*V
    td=inv(space)*v;
    % Step 9: Calculate new thetas, theta_new=theta+thetadot
    theta=theta+td;
    % Step 10: Calculate error, norm v < tolerance 
    n=norm(v);
    error=abs(n);
end
