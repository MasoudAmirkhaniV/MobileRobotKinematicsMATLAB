function [p1, p2] = ForKin(p0, q)
    L1 = 1;
    L2 = 1;

    t1 = q(1);
    t2 = q(2);

    p1 = [L1*cos(t1); L1*sin(t1); 0]+p0;
    p2 = [p1(1) + L2*cos(t1+t2); p1(2) + L2*sin(t1+t2); 0];

end
