function [f,J, domerr]= mcpfuncjacEval(z,jacflag)
    global a;           global contact_wrench;
    global dimen;       global num_of_contacts;
    global h;           global dista;
    global kp;          global q_o;
    global q_diff;

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

    for i = 1: dimen
        v(i,1) = -(kp*q_diff(i,1));
    end

    for i = 1:dimen
        P_n(i,1) = 0; %Compensation
        for j = 1:num_of_contacts
          P_n(i,1)= P_n(i,1) + pn(j,1)*contact_wrench(i,j);
        end
    end
    
    % Multiplication by h with P_n is missing
    eq_of_motion = (q_o-q_n+(A*h*v)+(P_n));
    for i = 1:dimen  
        f(i) = eq_of_motion(i,1);
    end

    % Missing: contact_wrench' * (q(l+1) - q(l))
    temp = contact_wrench' * q_n;
    f(3) = temp(1,1) + (dista - 0.01);

    if (jacflag)
        J(1,1) = -a; J(1,2) = 0; J(1,3) = contact_wrench(1,1);
        J(2,1) = 0;  J(2,2) = -a; J(2,3) = contact_wrench(2,1);
        J(3,1) = contact_wrench(1,1); J(3,2) = contact_wrench(2,1); J(3,3) = 0;
        J=sparse(J);
    end
end
