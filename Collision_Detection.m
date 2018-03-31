%%% Collision Detection
clear
clc


S = [-1.00000000 0.00000000 1.00000000 1.00000000 1.00000000 0.00000000; 0.00000000 0.00000000 0.00000000 0.00000000 0.00000000 0.00000000; 0.00000000 0.00000000 0.00000000 0.00000000 0.00000000 0.00000000; 0.00000000 1.00000000 0.00000000 0.00000000 0.00000000 0.00000000; 0.00000000 0.00000000 0.00000000 0.00000000 0.00000000 0.00000000; 0.00000000 0.00000000 2.00000000 2.00000000 2.00000000 1.00000000];
M = [0.00000000 0.00000000 1.00000000 4.00000000; 0.00000000 -1.00000000 0.00000000 -6.00000000; 1.00000000 0.00000000 0.00000000 0.00000000; 0.00000000 0.00000000 0.00000000 1.00000000];
theta = [-1.50621059 -2.55731960 -2.46737927 -2.56880880 -2.54534041 1.01243459 2.00273778 -0.87614251 1.16206438 -2.56865465 -0.52327399; -0.86862392 3.03606526 -1.88419878 -2.76657784 1.54078699 -0.75662572 1.80850830 0.92475966 0.49674778 -2.48628064 -0.49735599; 1.07188843 0.99692381 -0.62597930 -0.41235853 0.37463562 2.44805935 -1.43984905 -0.32077675 -2.73017953 -1.34333876 0.78164942; 2.25612252 -0.14111011 -2.39933900 -0.17704249 0.01565956 2.35538239 -3.00537093 0.94819865 0.27386113 2.97307171 -1.66569084; -1.42689273 -2.93077184 -3.09958441 2.65192482 -0.37566987 1.57348725 -2.29802656 1.86640150 1.21444059 -1.39121726 1.24537123; 2.89032231 0.65207400 1.06165003 1.49614657 3.00188603 -2.22566998 -2.12494436 0.22806928 0.18182712 3.04465968 -3.11656275];
r = 0.90000000;
r1=r;
r2=r1;


ss1=s_screw(S(:,1));
ss2=s_screw(S(:,2));
ss3=s_screw(S(:,3));
ss4=s_screw(S(:,4));
ss5=s_screw(S(:,5));
ss6=s_screw(S(:,6));

%%%MANUALLY INPUT THIS!
p1=[0;0;0;1];    %base - base never moves!
p2=[-2;0;0;1];    %joint 1 - joint 1 never moves!
p3_i=[-2;-2;0;1];  %joint 2
p4_i=[0;-2;0;1];  %joint 3
p5_i=[2;-2;0;1];  %joint 4
p6_i=[4;-2;0;1];  %joint 5
p7_i=[4;-4;0;1]; %joint 6
%p8 initial is in the M matrix


%Step 1 - figure out where all the joints are - you can do this!
for index=1:11 %there are 11 different robot configurations - so better do this 11 times! 
    
    j1=expm(ss1*theta(1,index));
    j2=expm(ss2*theta(2,index));
    j3=expm(ss3*theta(3,index));
    j4=expm(ss4*theta(4,index));
    j5=expm(ss5*theta(5,index));
    j6=expm(ss6*theta(6,index));
    
    T_end=j1*j2*j3*j4*j5*j6*M;
    p8=T_end(1:3,4);
    p8=[p8;1];
    p3=j1*p3_i;
    p4=j1*j2*p4_i;
    p5=j1*j2*j3*p5_i;
    p6=j1*j2*j3*j4*p6_i;
    p7=j1*j2*j3*j4*j5*p7_i;
    
    p=[p1 p2 p3 p4 p5 p6 p7 p8];
    
    %Collision Checker
    for i=1:7
        k=i+1;
        for ii=k:8
            %fprintf('counter:%d',i)
            %fprintf('%d\n',ii)
            check=collision(p(1:4,i),p(1:4,ii),r1,r2);
            if check==1
               c(index)=1; 
            end

        end
     end
        
    
end 
    
c






check=collision(p1,p2,r1,r2);


