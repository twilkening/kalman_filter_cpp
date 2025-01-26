r = randn(1,1000);

% pendulum equations, no friction:
syms th(t) a
Dth = diff(th,t);
eqtn = diff(th,t,2) == -a*(th);
conds = [th(0) == pi/180*10, Dth(0) == 0];
sol(t) = simplify(dsolve(eqtn,conds));

g = 9.8; % m/s^2
l = 1; % m
m1 = 1; % kg
m2 = 2; % kg
a = 3*g/l*(m1/2 + m2)/(m1 + 3*m2);

sol(t) = simplify(subs(sol(t),sym('a'),a));

step = 0.01;
t = 0:step:10-step;

theta = double(sol(t))';
theta_rand = double(sol(t) + r*pi/18/5)';

xr = l*cos(theta_rand); x = l*cos(theta);
yr = l*sin(theta_rand); y = l*sin(theta);

% plot(xr,yr,'*',x,y)
% plot(t,theta_rand,'*',t,theta)

writematrix(theta_rand,"theta_rand.csv")
writematrix(theta,"theta.csv")
writematrix(t',"t.csv")

%% Practice discretization from ECE6555 HW4

A = [0 0 1 0; 0 0 0 1; zeros(2,4)];
B = [0 0; 0 0; 1 0; 0 1];
T = sym('T');
syms t;
expm(A*T)

expm(A*T)*B
int(expm(A*(T-t))*B*transpose(expm(A*(T-t))*B),sym('t'),[0,T])

%% discretization of pendulum

syms a b real
a = - g*(m1/2 + m2)/(l*(m1 + 3*m2));
b = 3/(l^2*(m1 + 3*m2));
A = [0 1; a 0];
B = [0; b];
syms T t;
simplify(expm(A*(T-t)))
simplify(expm(A*(T-t))*B)

simplify(int(expm(A*(T-t))*B*transpose(expm(A*(T-t))*B),sym('t'),[0,T]))

