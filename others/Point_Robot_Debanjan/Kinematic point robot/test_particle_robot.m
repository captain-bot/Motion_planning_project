%% Example of a paritcle robot with velocity constraints

%% assuming particle follows a vertical line start at origin
clear all
global h;
h = 0.01; % time step

global q_old;
q_old = [0;0];

global nu;
nu = [1.3;2.4];

global n d;
n = [-0.9;-0.5];
d =1;

T = 3; % time period

N = T/h;


z = zeros(3,N); % z contains q_x, q_y, c

Z = [0;0;0]; % initial guess

% infty - value of infinity constant
infty = 1e20;

% l - lower bound 
l(1:2,1) = -infty; 
l(3:3,1) = 0;

% u - upper bound
u(1:3,1) = infty;

for i=1:N    
   z(:,i) = pathmcp(Z,l,u,'mcp_funjac_particle_robot');
   
  
   Z = z(:,i); % updating the initial guess for each iteration
   q_old = z(1:2,i); % updating the beginning value for next time step
end
x = 0:0.01:5;
y = -d/n(2) -(n(1)/n(2))*x;
plot(x,y,'.');
hold on
plot(z(1,:),z(2,:),'o');