theta_den = a(1) - (b(1) + b(2))*(1 + 2*a(2));
phi_den = -theta_den;

A1 = [0 1 0 0];
%A2 = [0 (-b(3)+2*a(2)*b(3)-a(4)*b(4)*(1+2*a(2)))/theta_den (b(4)*(1+2*a(2))/theta_den (b(3)+2*a(2)*b(3)-a(4)*b(4)*(1+2*a(2)))/theta_den];
A2 = [0 (-b(3)+2*a(2)*b(3)-a(4)*b(4)*(1+2*a(2)))/theta_den (b(4)*(1+2*a(2)))/theta_den (b(3)+2*a(2)*b(3)-a(4)*b(4)*(1+2*a(2)))/theta_den];
A3 = [0 0 0 1];
A4 = [0 (-a(1)*b(3)-a(3)*(b(1)+b(2)))/phi_den (-a(1)*b(4))/phi_den (a(1)*b(3)+a(4)*(b(1)+b(2)))/phi_den];

B2 = (a(5)+b(5)*(1+2*a(2)))/theta_den;
B4 = (a(1)*b(5)+a(5)*(b(1)+b(2)))/phi_den;

A = [A1;A2;A3;A4];
B = [0;B2;0;B4];
C = [1 0 0 0;0 0 1 0];
D = [0;0];

[t1,t2] = ss2tf(A,B,C,D);
%sys1 = tf(t1(1,:),t2);
%sys2 = tf(t1(2,:),t2);
sys = tf(1, t2);
figure(1);
rlocus(sys);
figure(2);
bode(sys);