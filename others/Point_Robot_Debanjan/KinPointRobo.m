clc
clear
close all

addpath('../pathmexa64');

global a;               global contact_wrench;  global dimen;  global q_diff;
global num_of_contacts; global dista;           global h;      global kp;
global nnz;             global q_o;             global kv;
      
dimen=2;
num_of_contacts = 1;
h=0.01;
kp=1;
kv=1;
n = dimen + num_of_contacts;
nnz = 6;
tf = 1;

a=1;
for j = 1:n
    l(j) = -Inf;
end
  for j = 1:n
    u(j) = Inf;
  end
  
l(n) = 0;
q_o=[0.5; 0.9];
q_goal(1,1) = 5; q_goal(2,1) = 0.5;
z = [0,0,0];
t = 0;

k=1;
i=1;
q=1;
m=0;
n=0;

for x=0:0.5:5
if (floor(x)==x)
    y=0;
    j(i)=x;
    k(i)=y;
else
    y=0.9;
    j(i)=x;
    k(i)=y;
end
i=i+1;
end
plot (j,k)
hold on
for c=0.5:0.5:4.5
if (floor(c)==c)
    d=0.1;
    m(q)=c;
    n(q)=d;
else
    d=1;
    m(q)=c;
    n(q)=d;
end
q=q+1;
end
plot (m,n)
hold on
k=1;

while t < tf
    norm_factor = 0.01;
    q_diff = q_o - q_goal;

    for i = 1:dimen
        norm_factor = norm_factor + (q_diff(i,1)*q_diff(i,1));
    end

    q_diff = q_diff*(1/sqrt(norm_factor));
    [contact_wrench,dista]= get_collision_info(q_o);
    [z,f,J] = pathmcp(z,l,u,'mcpfuncjacEval');

    for j = 1:dimen
      q_o(j,1) =  z(j);
    end

    q_o
    t = t + h;

    if(norm(q_diff(1:dimen)) <= 1e-06)
        t = tf;
    end

    x(k)=q_o(1,1);
    y(k)=q_o(2,1);

    plot (x(k),y(k),'.')
    hold on
    k=k+1;
    t
end

i=1;
q=1;
m=0;
n=0;

clear z;
clear f;
clear l;
clear u;