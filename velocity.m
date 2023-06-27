%velocity element of 6x1 matrix
x_dot=input('Enter value of X_dot = ');
y_dot=input('Enter value of Y_dot = ');
z_dot=input('Enter value of Z_dot = ');
thetax_dot=input('Enter value of theta_x_dot = ');
thetay_dot=input('Enter value of theta_y_dot = ');
thetaz_dot=input('Enter value of theta_z_dot = ');
vel=[x_dot;y_dot;z_dot;thetax_dot;thetay_dot;thetaz_dot];

%computes individual joint velocity (6x1 matrix)

 Angle_Velocity=eval(inv(JACOBIAN)*vel) 
