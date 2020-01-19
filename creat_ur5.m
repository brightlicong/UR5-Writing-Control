function [ur5] = creat_ur5(rn,bp)
%创建ur5机器人
clear L;
%           theta  d       a     alpha 
L(1) = Link([0     0.08916 0       pi/2],'standard') ; 
L(2) = Link([-pi/2 0       -0.425   0],'standard') ; 
L(3) = Link([0     0       -0.39225 0],'standard') ; 
L(4) = Link([-pi/2 0.10915 0       pi/2],'standard') ; 
L(5) = Link([0     0.09456 0       -pi/2],'standard') ; 
L(6) = Link([0     0.0823  0       0],'standard') ; 

ur5 = SerialLink(L,'name','UR5');

if nargin == 0
    ur5.name = 'UR5';
    ur5.base = transl(0,0,0);
elseif nargin == 1
    ur5.name = rn;
    ur5.base = transl(0,0,0);
elseif nargin == 2
    ur5.name = rn;
    ur5.base = bp;
end

end
