
clc;
clearvars;
%symbolic function
syms L1 L3 L4 theta1 d2 theta3 theta4 theta5 theta6
T01 = DH(0, 0, L1, theta1); %  passing dh table as arguemt to dh function [A] = DH(a, alpha, d,theta) 
T12 = DH(0, (pi/2), d2, 0);
T23 = DH(0, (-pi/2), L3, theta3-(pi/2));
T34 = DH(0, (pi/2), 0, theta4);
T45 = DH(L4, (-pi/2), 0, theta5);
T56 = DH(0, (pi/2), 0, theta6);

T02 = simplify((T01*T12));
T03 = simplify((T01*T12*T23));
T04 = simplify((T01*T12*T23*T34));
T05 = simplify((T01*T12*T23*T34*T45));
T06 = simplify((T01*T12*T23*T34*T45*T56));

%symbolic function
syms d2 theta1 theta3 theta4 theta5 theta6 
k1=1; k3=1; k4=1; k5=1; k6=1; %revolute joint
k2=0; %prismatic joint

%linear velocity 3x6 matrix
jv=simplify(jacobian([(d2*sin(theta1)) + L4*(sin(theta1+theta3)*cos(theta4)),- d2*cos(theta1) - L4*cos(theta1 + theta3)*cos(theta4),L1 + L3 + L4*sin(theta4)],[d2;theta1;theta3;theta4;theta5;theta6]));  
  

%angular velocity 3x6 matrix
jw_1=[k1*T01(:,3) k2*T02(:,3) k3*T03(:,3) k4*T04(:,3) k5*T05(:,3) k6*T06(:,3)];
jw=[jw_1(1:3,:)];

%jacobian containing linear and angular elements
JACOBIAN = [jv;jw]; % 6x6 matrix
L1= 250;
L3=100;
L4=80;
theta1 = input ('PLEASE ENTER THE VALUE OF THETA1 IN degree = ')*pi/180; 
d2 = input ('PLEASE ENTER THE VALUE OF d2 = '); 
theta3 = input ('PLEASE ENTER THE VALUE OF THETA3 IN degree = ')*pi/180; 
theta4 = input ('PLEASE ENTER THE VALUE OF THETA4 IN degree = ')*pi/180; 
theta5 = input ('PLEASE ENTER THE VALUE OF THETA5 IN degree = ')*pi/180; 
theta6 = input ('PLEASE ENTER THE VALUE OF THETA6 IN degree = ')*pi/180; 

J=eval(JACOBIAN) %Jacobian



% equ=det(JACOBIAN)==0;
% sol=solve(equ,[theta3,theta4,theta5],'Real',true);
% simplify(sol.theta3)*180/pi
% sol.theta4*180/pi
% sol.theta5*180/pi






