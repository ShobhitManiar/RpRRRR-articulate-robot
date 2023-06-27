%FORWARD KINEMTAICS
% syms L1 L3 L4 theta1 d2 theta3 theta4 theta5 theta6

clc;
clearvars;
%defining link parameters
L1= 250;
L3=100;
L4=80;

theta1 = input ('PLEASE ENTER THE VALUE OF THETA1 IN degree = ')*pi/180; 
d2 = input ('PLEASE ENTER THE VALUE OF d2 in mm = '); 
theta3 = input ('PLEASE ENTER THE VALUE OF THETA3 IN degree = ')*pi/180; 
theta4 = input ('PLEASE ENTER THE VALUE OF THETA4 IN degree = ')*pi/180; 
theta5 = input ('PLEASE ENTER THE VALUE OF THETA5 IN degree = ')*pi/180; 
theta6 = input ('PLEASE ENTER THE VALUE OF THETA6 IN degree = ')*pi/180; 


%forward kinematics for individual link
T01 = DH(0, 0, L1, theta1); %  passing dh table as arguemt to dh function [A] = DH(a, alpha, d,theta) 
T12 = DH(0, (pi/2), d2, 0);
T23 = DH(0, (-pi/2), L3, theta3-(pi/2));
T34 = DH(0, (pi/2), 0, theta4);
T45 = DH(L4, (-pi/2), 0, theta5);
T56 = DH(0, (pi/2), 0, theta6);


T01;
T02 = (T01*T12);
T03 = (T01*T12*T23);
T04 = (T01*T12*T23*T34);
T05 = (T01*T12*T23*T34*T45);
T06 = (T01*T12*T23*T34*T45*T56); 

P6=T06  %Final transformation matrix
r21=T06(2,1);
r11=T06(1,1);
r31=T06(3,1);
r32=T06(3,2);
r33=T06(3,3);
g32=sqrt(r32^2+r33^2);
alpha= atan2(r21,r11)*180/pi;
beta=atan2(-r31,g32)*180/pi;
gama=atan2(r32,r33)*180/pi;

Xw = T06(1,4); 
Yw = T06(2,4); 
Zw = T06(3,4); 
Ptip = [Xw;Yw;Zw;alpha;beta;gama] %XYZ alpha beta gama coordinate in space

% T02 = simplify((T01*T12))
% T03 = simplify((T01*T12*T23))
% T04 = simplify((T01*T12*T23*T34))
% T05 = simplify((T01*T12*T23*T34*T45))
% T06 = simplify((T01*T12*T23*T34*T45*T56))


