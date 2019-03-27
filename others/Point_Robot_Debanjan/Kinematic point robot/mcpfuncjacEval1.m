function [f,J, domerr]= mcpfuncjacEval1(z,jacflag)
    global contact_wrench;     global dimen;     global num_of_contacts;
    global h;                  global dista;     global q_o;
    global q_diff;
    
    z = z(:);
    f = [];
    J = [];
    domerr = 0;
    
    for i = 1:dimen
        q_n(i,1) = z(i);
    end
    
    for i = 1:num_of_contacts
        pn(i,1) = z(i + dimen);
    end
    
    A=eye(2);
    % v = -kp*q_diff;
    v(1,1)=1.3;
    v(2,1)=0;
    
    for i = 1:dimen
        P_n(i,1) = 0; %Compensation
        for j = 1:num_of_contacts
            P_n(i,1)= P_n(i,1) + pn(j,1)*contact_wrench(i,j);
        end
    end
    
    % eq_of_motion = -M*(nu_n - nu_o) + P_app + P_n;
    
    eq_of_motion = -((q_o-q_n+(A*h*v)+(h*P_n)));
    for i = 1:dimen
        f(i) = eq_of_motion(i,1);
    end
    temp = contact_wrench' * (q_n - q_o);
    f(3) = temp + (dista);
    
    if (jacflag)
        J(1,1) = 1; J(1,2) = 0; J(1,3) = -h*contact_wrench(1,1);
        J(2,1) = 0;  J(2,2) = 1; J(2,3) = -h*contact_wrench(2,1);
        J(3,1) = contact_wrench(1,1); J(3,2) = contact_wrench(2,1); J(3,3) = 0;
        J=sparse(J);
    end
end
