%% transformation matrices
syms d1 d2 d3 t4 t5 t6  
syms t8rf t8rb t8lf t8lb t9rf t9rb t9lf t9lb
syms d8rf d8rb d8lf d8lb d9rf d9rb d9lf d9lb
syms a7t a8t t8t
A1 = [0 0 1 0;
      1 0 0 0;
      0 1 0 d1;
      0 0 0 1];
A2 = [0 0 1 0;
      1 0 0 0;
      0 1 0 d2;
      0 0 0 1];
A3 = [0 0 1 0;
      1 0 0 0;
      0 1 0 d3;
      0 0 0 1];
A03 = A1*A2*A3;
A4 = [-sin(t4) 0 cos(t4) 0;
      cos(t4) 0 sin(t4) 0;
      0 1 0 0;
      0 0 0 1];
A5 = [-sin(t5) 0 cos(t5) 0;
      cos(t5) 0 sin(t5) 0;
      0 1 0 0;
      0 0 0 1];
A6 = [-sin(t6) 0 cos(t6) 0;
      cos(t6) 0 sin(t6) 0;
      0 1 0 0;
      0 0 0 1];
A36 = A4*A5*A6;
A03*A36;
A7 = [0 0 1 0;
      1 0 0 0;
      0 1 0 0;
      0 0 0 1];
A8RF = [-sin(t8rf) 0 -cos(t8rf) 0;
      cos(t8rf) 0 -sin(t8rf) 0;
      0 -1 0 d8rf;
      0 0 0 1];
A9RF = [cos(t9rf) -sin(t9rf) 0 0;
      sin(t9rf) cos(t9rf) 0 0;
      0 0 1 d9rf;
      0 0 0 1];
A8RB = [-sin(t8rb) 0 -cos(t8rb) 0;
      cos(t8rb) 0 -sin(t8rb) 0;
      0 -1 0 -d8rb;
      0 0 0 1];
A9RB = [cos(t9rb) -sin(t9rb) 0 0;
      sin(t9rb) cos(t9rb) 0 0;
      0 0 1 d9rb;
      0 0 0 1];
A8LF = [sin(t8lf) 0 -cos(t8lf) 0;
      cos(t8lf) 0 sin(t8lf) 0;
      0 -1 0 d8lf;
      0 0 0 1];
A9LF = [cos(t9lf) -sin(t9lf) 0 0;
      sin(t9lf) cos(t9lf) 0 0;
      0 0 1 -d9lf;
      0 0 0 1];
A8LB = [sin(t8lb) 0 -cos(t8lb) 0;
      cos(t8lb) 0 sin(t8lb) 0;
      0 -1 0 -d8lb;
      0 0 0 1];
A9LB = [cos(t9lb) -sin(t9lb) 0 0;
      sin(t9lb) cos(t9lb) 0 0;
      0 0 1 -d9lb;
      0 0 0 1];
  
A7*A8RF*A9RF;

%% body jacob

z0 = [0 0 1 0]';
J1 = [z0(1:3) ; [0 0 0]'];
z1 = A1*z0;
J2 = [z1(1:3) ; [0 0 0]'];
z2 = A1*A2*z0;
J3 = [z2(1:3) ; [0 0 0]'];
z3 = A1*A2*A3*z0;
J4 = [[0 0 0]' ; z3(1:3)];
z4 = A1*A2*A3*A4*z0;
J5 = [[0 0 0]' ; z4(1:3)];
z5 = A1*A2*A3*A4*A5*z0;
J6 = [[0 0 0]' ; z5(1:3)];
J_body = [J1 J2 J3 J4 J5 J6]

%% wing jacobs

z6 = [0 0 1 0]';
z7 = A7*z6;
z8RF = A7*A8RF*z6;
z8LF = A7*A8LF*z6;
z8RB = A7*A8RB*z6;
z8LB = A7*A8LB*z6;

o7 = [0 0 0 1]';
o8RF = A7*A8RF*o7;
o9RF = A7*A8RF*A9RF*o7;
o8LF = A7*A8LF*o7;
o9LF = A7*A8LF*A9LF*o7;
o8RB = A7*A8RB*o7;
o9RB = A7*A8RB*A9RB*o7;
o8LB = A7*A8LB*o7;
o9LB = A7*A8LB*A9LB*o7;

J8RF = [cross(z7(1:3), o9RF(1:3)) ; z7(1:3)];
J8LF = [cross(z7(1:3), o9LF(1:3)) ; z7(1:3)];
J8RB = [cross(z7(1:3), o9RB(1:3)) ; z7(1:3)];
J8LB = [cross(z7(1:3), o9LB(1:3)) ; z7(1:3)];
J9RF = [cross(z8RF(1:3), o9RF(1:3)-o8RF(1:3)) ; z8RF(1:3)];
J9LF = [cross(z8LF(1:3), o9LF(1:3)-o8LF(1:3)) ; z8LF(1:3)];
J9RB = [cross(z8RB(1:3), o9RB(1:3)-o8RB(1:3)) ; z8RB(1:3)];
J9LB = [cross(z8LB(1:3), o9LB(1:3)-o8LB(1:3)) ; z8LB(1:3)];

J_RF = [J8RF J9RF]
J_LF = [J8LF J9LF]
J_RB = [J8RB J9RB]
J_LB = [J8LB J9LB]