function Tij = TransM(DH)
    
    % DH: alpha  d   l   theta

    alpha = DH(1);
    d = DH(2);
    l = DH(3);
    theta = DH(4);

    Tij = [cos(theta)  -sin(theta)*cos(alpha)  sin(theta)*sin(alpha)    l*cos(theta)
           sin(theta)   cos(theta)*cos(alpha)  -cos(theta)*sin(alpha)   l*sin(theta)
            0                   sin(alpha)              cos(alpha)          d
            0                       0                       0               1];


end