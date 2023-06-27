
function [NONRETURNFUNCTION] = INVERSE(  )
clearvars;
clc;

L1=250;
L3=100;
L4=80;

% Taking position and orientation of point in space to calculate inverse
% for
nosolution=1000;
px=input('Enter value of X = ');
py=input('Enter value of y = ');
pz=input('Enter value of Z = ');
alpha =input('Enter value of alpha = ');
beta=input('Enter value of beta = ');
gama = input('Enter value of gamma = ');


alpha = alpha * pi/180;                           %changing to radiance from degree
beta = beta * pi/180; 
gama = gama * pi/180;

% Rotational matrix 
% R60 = [cos(alpha).*cos(beta), (cos(alpha).*sin(beta).*sin(gama))- sin(alpha).*cos(gama), (cos(alpha).*sin(beta).*cos(gama)) + sin(alpha).*sin(gama) ; 
%  
%    sin(alpha).*cos(beta),   (sin(alpha).*sin(beta).*sin(gama)) + cos(alpha).*cos(gama), (sin(alpha).*sin(beta).*cos(gama)) - cos(alpha).*sin(gama) ; 
%  
%   - sin(beta),               cos(beta).*sin(gama),                               cos(beta).*sin(gama)] 


r3=(cos(alpha).*sin(beta).*cos(gama)) + sin(alpha).*sin(gama);
r6=(sin(alpha).*sin(beta).*cos(gama)) - cos(alpha).*sin(gama);
r7=- sin(beta);
r8=cos(beta).*sin(gama);
r9=cos(beta).*cos(gama);

%function that converts radiance to degree and defines joint angle singularity 
function OUT = conversion( theta,upperlimit,lowerlimit) 
upperlimit = upperlimit * pi / 180; 
lowerlimit = lowerlimit * pi / 180; 
if (theta > upperlimit) 
    OUT = nan;%('THE SOLUTION OUT OF JOINT ANGLE LIMIT'); 
elseif (theta < lowerlimit) 
    OUT = nan;%('THE SOLUTION OUT OF JOINT ANGLE LIMIT'); 
else  
OUT = round(theta * 180 / pi); 
end 
end

%sin(theta1)
function rsin = var1(theta1,theta2)
        rsin= px-(L4*sin(theta1)*cos(theta2));
        
end

%cos(theta1)
function rcos = var2(theta1,theta2)
        rcos= py+(L4*cos(theta1)*cos(theta2));
        
end

%prismatic joint calculation
function d= pris(thetaa,thetab,thetac)
        d= round(-(py+(L4*cos(thetaa)*cos(thetab)))/cos(thetac));
end

    function out=limit(d) %Singularity condition for prismatics joint
   upperlimit=200;
   lowerlimit=0;
    if (d > upperlimit) 
        out = nan;%('THE SOLUTION OUT OF JOINT ANGLE LIMIT'); 
    elseif (d < lowerlimit) 
        out = nan;%('THE SOLUTION OUT OF JOINT ANGLE LIMIT'); 
    else  
        out = d;
    end 
end
%----------------------------------------------------------------------------------------------------------------%
% Theta 4 solution
s4 = (pz-L1-L3)/L4;
a=(1-s4^2);

% checking for singularity
if a>0       
c4 = sqrt(a);
theta41=atan2(s4,c4);
theta42=atan2(s4,-c4);
Theta41= conversion(theta41,230,-50);
Theta42= conversion(theta42,230,-50);
else 
theta41=nosolution;
theta42=nosolution;
end
%---------------------------------One set of Theta 5---------------------------------------------------------------%

% Theta 5 solution
if (theta41==nosolution)
  Theta51=nan;%('GOAL OUT OF WORKSPACE, NO VAILD VALUES EXISTS');
  Theta52=nan;%('GOAL OUT OF WORKSPACE, NO VAILD VALUES EXISTS');
else

s51=r9/sin(theta41);

if 1-s51^2>0
c51=sqrt(1-s51^2);
theta51=atan2(s51,c51);
theta52=atan2(s51,-c51);
Theta51=conversion(theta51,130,-130);
Theta52=conversion(theta52,130,-130);
else
theta51=nosolution;
theta52=nosolution;
end
end
%--------------------------------Second set of Theta 5----------------------------------------------------%
if(theta42 == nosolution)
   Theta53=nosolution;%('GOAL OUT OF WORKSPACE, NO VAILD VALUES EXISTS');
   Theta54=nosolution;%('GOAL OUT OF WORKSPACE, NO VAILD VALUES EXISTS');
else
s52=r9/sin(theta42);
if 1-s52^2>0
c52=sqrt(1-s52^2);
theta53=atan2(s52,c52);
theta54=atan2(s52,-c52);
Theta53=conversion(theta53,130,-130);
Theta54=conversion(theta54,130,-130);
else
theta53=nosolution;
theta54=nosolution;  
end
end 
%----------------------------------one set of Theta6 Theta13 solution------------------------------------------------------%

if(theta41==nosolution||theta51==nosolution)
    Theta61=nan;%('GOAL OUT OF WORKSPACE, NO VAILD VALUES EXISTS');
    Theta62=nan;%('GOAL OUT OF WORKSPACE, NO VAILD VALUES EXISTS');
    Theta63=nan;%('GOAL OUT OF WORKSPACE, NO VAILD VALUES EXISTS');
    Theta64=nan;%('GOAL OUT OF WORKSPACE, NO VAILD VALUES EXISTS');
else
s61=((r7*cos(theta41))-(cos(theta51)*sin(theta41)*r8))/((cos(theta41))^2+(cos(theta51)^2*sin(theta41)^2));
c61= sqrt(1-s61^2);
theta61 = atan2(s61,c61);
theta62= atan2(s61,-c61);
Theta61=conversion(theta61,130,-130);
Theta62=conversion(theta62,130,-130);

c131=-(((r6*cos(theta41)*sin(theta51))+(r3*cos(theta51)))/((cos(theta41)^2*sin(theta51)^2)+cos(theta51)^2));
s131=sqrt(1-c131^2);
theta131=atan2(s131,c131);
theta132=atan2(-s131,c131);
%---------------------------------------------set 1: theta 1, theta3 and d2 solution-----------------------------------------------------%

s11= var1(theta131,theta41);
c11= var2(theta131,theta41);

theta11=atan2(s11,c11);
theta31=theta131-theta11;
d21=pris(theta131,theta41,theta11);
Theta11=conversion(theta11,180,-180);
Theta31=conversion(theta31,180,-180);
D21=limit(d21);


theta12=atan2(s11,-c11);
theta32=theta131-theta12;
d22=pris(theta131,theta41,theta12);
Theta12=conversion(theta12,180,-180);
Theta32=conversion(theta32,180,-180);
D22=limit(d22);
%----------------------------------------------set 2: theta 1, theta3 and d2 solution------------------------------------%
s12= var1(theta132,theta41);
c12= var2(theta132,theta41);

theta13=atan2(s12,c12);
theta33=theta132-theta13;
d23=pris(theta132,theta41,theta13);
Theta13=conversion(theta13,180,-180);
Theta33=conversion(theta33,180,-180);
D23=limit(d23);

theta14=atan2(s12,-c12);
theta34=theta132-theta14;
d24=pris(theta132,theta41,theta14);
Theta14=conversion(theta14,180,-180);
Theta34=conversion(theta34,180,-180);
D24=limit(d24);
%----------------------------------------------------------------------------------------------------------------%
%-------------------------------------------------second set of theta 6 and theta 13--------------------------------%
s62=((r7*cos(theta41))-(cos(theta52)*sin(theta41)*r8))/((cos(theta41))^2+(cos(theta52)^2*sin(theta41)^2));
c62= sqrt(1-s62^2);
theta63 = atan2(s62,c62);
theta64= atan2(s62,-c62);
Theta63=conversion(theta63,360,-360);
Theta64=conversion(theta64,360,-360);


c132=-(((r6*cos(theta41)*sin(theta52))+(r3*cos(theta52)))/((cos(theta41)^2*sin(theta52)^2)+cos(theta52)^2));
s132=sqrt(1-c132^2);
theta133=atan2(s132,c132);
theta134=atan2(-s132,c132);

%------------------------------------------------set 3: theta 1, theta3 and d2 solution-----------------------------%
s13= var1(theta133,theta41);
c13= var2(theta133,theta41);

theta15=atan2(s13,c13);
theta35=theta133-theta15;
d25=pris(theta133,theta41,theta15);
Theta15=conversion(theta15,180,-180);
Theta35=conversion(theta35,180,-180);
D25=limit(d25);

theta16=atan2(s13,c13);
theta36=theta133-theta16;
d26=pris(theta133,theta41,theta16);
Theta16=conversion(theta16,180,-180);
Theta36=conversion(theta36,180,-180);
D26=limit(d26);

%------------------------------------------set 4: theta 1, theta3 and d2 solution-------------------------------------%
s14= var1(theta134,theta41);
c14= var2(theta134,theta41);

theta17=atan2(s14,c14);
theta37=theta134-theta17;
d27=pris(theta134,theta41,theta17);
Theta17=conversion(theta17,180,-180);
Theta37=conversion(theta37,180,-180);
D27=limit(d27);

theta18=atan2(s14,-c14);
theta38=theta134-theta18;
d28=pris(theta134,theta41,theta18);
Theta18=conversion(theta18,180,-180);
Theta38=conversion(theta38,180,-180);
D28=limit(d28);
%----------------------------------------------------------------------------------------------------------------%
%----------------------------------------------------------------------------------------------------------------%
%----------------------------------------------------------------------------------------------------------------%
%----------------------------------------------third set of theta 6 an theta 13--------------------------------------%
end
if(theta42==nosolution||theta53==nosolution)
    Theta65=nan;%('GOAL OUT OF WORKSPACE, NO VAILD VALUES EXISTS');
    Theta66=nan;%('GOAL OUT OF WORKSPACE, NO VAILD VALUES EXISTS');
    Theta67=nan;%('GOAL OUT OF WORKSPACE, NO VAILD VALUES EXISTS');
    Theta68=nan;%('GOAL OUT OF WORKSPACE, NO VAILD VALUES EXISTS');
else

s63=((r7*cos(theta42))-(cos(theta53)*sin(theta42)*r8))/((cos(theta42))^2+(cos(theta53)^2*sin(theta42)^2));
c63= sqrt(1-s63^2);
theta65 = atan2(s63,c63);
theta66= atan2(s63,-c63);
Theta65=conversion(theta65,130,-130);
Theta66=conversion(theta66,130,-130);

c133=-(((r6*cos(theta42)*sin(theta53))+(r3*cos(theta53)))/((cos(theta42)^2*sin(theta53)^2)+cos(theta53)^2));
s133=sqrt(1-c133^2);
theta135=atan2(s133,c133);
theta136=atan2(-s133,c133);
%--------------------------------------------set 5: theta 1, theta3 and d2 solution-----------------------------%
s15= var1(theta135,theta42);
c15= var2(theta135,theta42);
theta19=atan2(s15,c15);
theta39=theta135-theta19;
d29=pris(theta135,theta42,theta19);
Theta19=conversion(theta19,180,-180);
Theta39=conversion(theta39,180,-180);
D29=limit(d29);

theta110=atan2(s15,-c15);
theta310=theta135-theta110;
d210=pris(theta135,theta42,theta110);
Theta110=conversion(theta110,180,-180);
Theta310=conversion(theta310,180,-180);
D210=limit(d210);
%---------------------------------------------set 6: theta 1, theta3 and d2 solution---------------------------------%

s16= var1(theta136,theta42);
c16= var2(theta136,theta42);

theta111=atan2(s16,c16);
theta311=theta136-theta111;
d211=pris(theta136,theta42,theta111);
Theta111=conversion(theta111,180,-180);
Theta311=conversion(theta311,180,-180);
D211=limit(d211);

theta112=atan2(s16,-c16);
theta312=theta136-theta112;
d212=pris(theta136,theta41,theta112);
Theta112=conversion(theta112,180,-180);
Theta312=conversion(theta312,180,-180);
D212=limit(d212);

%----------------------------------------------------------------------------------------------------------------%
%---------------------------------------fourth set of theta 6 and theta 13----------------------------------%

s64=((r7*cos(theta42))-(cos(theta54)*sin(theta42)*r8))/((cos(theta42))^2+(cos(theta54)^2*sin(theta42)^2));
c64= sqrt(1-s64^2);
theta67 = atan2(s64,c64);
theta68= atan2(s64,-c64);
Theta67=conversion(theta67,130,-130);
Theta68=conversion(theta68,130,-130);

c134=-(((r6*cos(theta42)*sin(theta54))+(r3*cos(theta54)))/((cos(theta42)^2*sin(theta54)^2)+cos(theta54)^2));
s134=sqrt(1-c134^2);
theta137=atan2(s134,c134);
theta138=atan2(-s134,c134);
%---------------------------------------------set 7: theta 1, theta3 and d2 solution---------------------------------%
s17= var1(theta137,theta42);
c17= var2(theta137,theta42);

theta113=atan2(s17,c17);
theta313=theta137-theta113;
d213=pris(theta137,theta42,theta113);
Theta113=conversion(theta113,180,-180);
Theta313=conversion(theta313,180,-180);
D213=limit(d213);

theta114=atan2(s17,-c17);
theta314=theta137-theta114;
d214=pris(theta137,theta42,theta114);
Theta114=conversion(theta114,180,-180);
Theta314=conversion(theta314,180,-180);
D214=limit(d214);
%------------------------------------------------set 8: theta 1, theta3 and d2 solution--------------------------------%
s18= var1(theta138,theta42);
c18= var2(theta138,theta42);

theta115=atan2(s18,c18);
theta315=theta138-theta115;
d215=pris(theta138,theta42,theta115);
Theta115=conversion(theta115,180,-180);
Theta315=conversion(theta315,180,-180);
D215=limit(d215);

theta116=atan2(s18,-c18);
theta316=theta138-theta116;
d216=pris(theta138,theta42,theta116);
Theta116=conversion(theta116,180,-180);
Theta316=conversion(theta316,180,-180);
D216=limit(d216);
%----------------------------------------------------------------------------------------------------------------%
%----------------------------------------------------------------------------------------------------------------%
%----------------------------------------------------------------------------------------------------------------%
%----------------------------------------------------------------------------------------------------------------%
end


% Output
disp ( ' d and THETA 1,3,4,5,6 SOLUTIONS')


if (theta41==nosolution||theta42==nosolution||theta51==nosolution||theta53==nosolution)
    disp('----Set 1 and 2----')
    disp('GOAL OUT OF WORKSPACE')
    disp('----Set 3 & 4-----')
    disp('GOAL OUT OF WORKSPACE')
else
SOL1 =[Theta11,D21,Theta31,Theta41, Theta51, Theta61];
SOL2 =[Theta12,D22,Theta32,Theta41, Theta51, Theta61];
SOL3 =[Theta13,D23,Theta33,Theta41, Theta51, Theta62];
SOL4 =[Theta14,D24,Theta34,Theta41, Theta51, Theta62];
SOL5 =[Theta15,D25,Theta35,Theta41, Theta52, Theta63];
SOL6 =[Theta16,D26,Theta36,Theta41, Theta52, Theta63];
SOL7 =[Theta17,D27,Theta37,Theta41, Theta52, Theta64];
SOL8 =[Theta18,D28,Theta38,Theta41, Theta52, Theta64];
SOL9  =[Theta19,D29,Theta39,Theta42, Theta53, Theta65];
SOL10 =[Theta110,D210,Theta310,Theta42, Theta53, Theta65];
SOL11 =[Theta111,D211,Theta311,Theta42, Theta53, Theta66];
SOL12 =[Theta112,D212,Theta312,Theta42, Theta53, Theta66];
SOL13 =[Theta113,D213,Theta313,Theta42, Theta54, Theta67];
SOL14 =[Theta114,D214,Theta314,Theta42, Theta54, Theta67];
SOL15 =[Theta115,D215,Theta315,Theta42, Theta54, Theta68];
SOL16 =[Theta116,D216,Theta316,Theta42, Theta54, Theta68];

disp ( ' --------SET 1-----------') 
Solution1=[SOL1;SOL2;SOL3;SOL4]
disp ( ' --------SET 2-----------') 
Solution2=[SOL5;SOL6;SOL7;SOL8]
disp ( ' --------SET 3-----------') 
Solution3=[SOL9;SOL10;SOL11;SOL12]
disp ( '--------SET 4------------') 
Solution4=[SOL13;SOL14;SOL15;SOL16]

end
end

