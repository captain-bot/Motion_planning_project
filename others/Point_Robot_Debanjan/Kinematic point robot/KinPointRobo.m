clear all
clc

addpath('../../pathmexa64');

pi= 3.1415926535879;
global a;
global contact_wrench;
global dimen;
global q_diff;
global num_of_contacts;
global g;
global h;
global dista;
global nnz;
global kp;
global kv;
global q_o;
dimen=2;
num_of_contacts = 1;
g=9.8;
h=0.01;
kp=1;
kv=1;
n = dimen + num_of_contacts;
nnz = 6;
tf = 40;
big = 1e20;
a=1;
for j = 1:n
    l(j) = -big;
end
for j = 1:n
    u(j) = big;
end

l(n) = 0;
q_o=[0.5;.9];
q_goal(1,1) = 5; q_goal(2,1) = 0.9;
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
    
    q_diff = q_o - q_goal;
    
    q_diff = q_diff/(norm(q_diff));
    [contact_wrench,dista]= get_collision_info(q_o);
%     [z,f,J] = pathmcp(z,l,u,'mcpfuncjacEval');
    [z,f,J] = pathmcp(z,l,u,'mcpfuncjacEval1');
    for j = 1:dimen
        q_o(j,1) =  (z(j));
    end
  
    t = t + h;
    
    if((-q_o + q_goal) <= 1e-06)
        t = tf;
    end
    x(k)=q_o(1,1);
    y(k)=q_o(2,1);
    
    plot (x(k),y(k),'.')
    hold on
    k=k+1;
end