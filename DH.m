%Homogenous Transformation Matrix for Modefied DH Table
function [A] = DH(a, alpha, d,theta)
A= [    cos(theta)                       -sin(theta)                            0                      a
    sin(theta)*round(cos(alpha))    cos(theta)*round(cos(alpha))   round(-sin(alpha))     round(-sin(alpha))*d
    sin(theta)*round(sin(alpha))    cos(theta)*round(sin(alpha))   round(cos(alpha))      round(cos(alpha))*d
             0                               0                                  0                      1];
end  
