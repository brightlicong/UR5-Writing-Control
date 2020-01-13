function [ angle ] = ur5_ikine_single( ur5 , Tq_world )
%用于UR机器人反解，输入为正交变换矩阵，输出位八组可行解。

%Tq to world -> Tq to base
base_world = ur5.base;
Tq_base = base_world \ Tq_world;

d1=ur5.links(1).d;
a2=ur5.links(2).a;
a3=ur5.links(3).a;
d4=ur5.links(4).d;
d5=ur5.links(5).d;
d6=ur5.links(6).d;
% Tq=[-0.0660    0.9901    0.1238    0.4568;
%     0.0762   -0.1187    0.9900   -0.0278;
%     0.9949    0.0748   -0.0677    0.6233;
%          0         0         0    1.0000];

nx=Tq_base(1,1);ny=Tq_base(2,1);nz=Tq_base(3,1);
ox=Tq_base(1,2);oy=Tq_base(2,2);oz=Tq_base(3,2);
ax=Tq_base(1,3);ay=Tq_base(2,3);az=Tq_base(3,3);
px=Tq_base(1,4);py=Tq_base(2,4);pz=Tq_base(3,4);

%two kinds of theta1
theta1_1=atan2((d6*ay-py),(-px+ax*d6))-atan2(d4,sqrt((d6*ay-py)^2+(px-ax*d6)^2-d4*d4));
theta1_2=atan2((d6*ay-py),(-px+ax*d6))-atan2(d4,-sqrt((d6*ay-py)^2+(px-ax*d6)^2-d4*d4));

% modify the value of theta1,let theta1 belong to (-pi,pi]
if theta1_1>pi
    theta1_1=theta1_1-2*pi;
else
end

if theta1_1<(-pi)
    theta1_1=theta1_1+2*pi;
else
end

if theta1_2>pi
    theta1_2=theta1_2-2*pi;
else
end

if theta1_2<(-pi)
    theta1_2=theta1_2+2*pi;
else
end

%use theta1_1 , two kinds of theta5
c5_1=sin(theta1_1)*ax-cos(theta1_1)*ay;
s5_1=sqrt(1-(sin(theta1_1)*ax-cos(theta1_1)*ay)^2);
theta5_1=atan2(s5_1,c5_1);
theta5_2=atan2(-s5_1,c5_1);
%use theta1_2 , two kinds of theta5
c5_3=sin(theta1_2)*ax-cos(theta1_2)*ay;
s5_3=sqrt(1-(sin(theta1_2)*ax-cos(theta1_2)*ay)^2);
theta5_3=atan2(s5_3,c5_3);
theta5_4=atan2(-s5_3,c5_3);
%four kinds of theta6
c6_1=(sin(theta1_1)*nx-cos(theta1_1)*ny)/sin(theta5_1);
s6_1=(cos(theta1_1)*oy-sin(theta1_1)*ox)/sin(theta5_1);
theta6_1=atan2(s6_1,c6_1);

c6_2=(sin(theta1_1)*nx-cos(theta1_1)*ny)/sin(theta5_2);
s6_2=(cos(theta1_1)*oy-sin(theta1_1)*ox)/sin(theta5_2);
theta6_2=atan2(s6_2,c6_2);

c6_3=(sin(theta1_2)*nx-cos(theta1_2)*ny)/sin(theta5_3);
s6_3=(cos(theta1_2)*oy-sin(theta1_2)*ox)/sin(theta5_3);
theta6_3=atan2(s6_3,c6_3);

c6_4=(sin(theta1_2)*nx-cos(theta1_2)*ny)/sin(theta5_4);
s6_4=(cos(theta1_2)*oy-sin(theta1_2)*ox)/sin(theta5_4);
theta6_4=atan2(s6_4,c6_4);

% four kinds of beta
cbeta1=sin(theta6_1)*nz+cos(theta6_1)*oz;
sbeta1=cos(theta5_1)*(cos(theta6_1)*nz-sin(theta6_1)*oz)-sin(theta5_1)*az;
beta1=atan2(sbeta1,cbeta1);

cbeta2=sin(theta6_2)*nz+cos(theta6_2)*oz;
sbeta2=cos(theta5_2)*(cos(theta6_2)*nz-sin(theta6_2)*oz)-sin(theta5_2)*az;
beta2=atan2(sbeta2,cbeta2);

cbeta3=sin(theta6_3)*nz+cos(theta6_3)*oz;
sbeta3=cos(theta5_3)*(cos(theta6_3)*nz-sin(theta6_3)*oz)-sin(theta5_3)*az;
beta3=atan2(sbeta3,cbeta3);

cbeta4=sin(theta6_4)*nz+cos(theta6_4)*oz;
sbeta4=cos(theta5_4)*(cos(theta6_4)*nz-sin(theta6_4)*oz)-sin(theta5_4)*az;
beta4=atan2(sbeta4,cbeta4);

% eight kinds of theta2,theta3,theta4-----------1/2
L1_1=cos(theta1_1)*px+sin(theta1_1)*py+d6*cos(beta1)*sin(theta5_1)-d5*sin(beta1);
L2_1=pz-d1+d6*sin(beta1)*sin(theta5_1)+d5*cos(beta1);
theta3_1=real(acos((L1_1^2+L2_1^2-a3^2-a2^2)/2/a3/a2));
theta3_2=-real(acos((L1_1^2+L2_1^2-a3^2-a2^2)/2/a3/a2));
c2_1=(L1_1*(a3*cos(theta3_1)+a2)+L2_1*a3*sin(theta3_1))/(((a3*cos(theta3_1)+a2)^2+(a3*sin(theta3_1))^2));
s2_1=(L2_1*(a3*cos(theta3_1)+a2)-L1_1*a3*sin(theta3_1))/(((a3*cos(theta3_1)+a2)^2+(a3*sin(theta3_1))^2));
c2_2=(L1_1*(a3*cos(theta3_2)+a2)+L2_1*a3*sin(theta3_2))/(((a3*cos(theta3_2)+a2)^2+(a3*sin(theta3_2))^2));
s2_2=(L2_1*(a3*cos(theta3_2)+a2)-L1_1*a3*sin(theta3_2))/(((a3*cos(theta3_2)+a2)^2+(a3*sin(theta3_2))^2));
theta2_1=atan2(s2_1,c2_1);
theta2_2=atan2(s2_2,c2_2);
theta4_1=beta1-theta2_1-theta3_1;
theta4_2=beta1-theta2_2-theta3_2;
% eight kinds of theta2,theta3,theta4-----------3/4
L1_3=cos(theta1_1)*px+sin(theta1_1)*py+d6*cos(beta2)*sin(theta5_2)-d5*sin(beta2);
L2_3=pz-d1+d6*sin(beta2)*sin(theta5_2)+d5*cos(beta2);
theta3_3=real(acos((L1_3^2+L2_3^2-a3^2-a2^2)/2/a3/a2));
theta3_4=-real(acos((L1_3^2+L2_3^2-a3^2-a2^2)/2/a3/a2));
c2_3=(L1_3*(a3*cos(theta3_3)+a2)+L2_3*a3*sin(theta3_3))/(((a3*cos(theta3_3)+a2)^2+(a3*sin(theta3_3))^2));
s2_3=(L2_3*(a3*cos(theta3_3)+a2)-L1_3*a3*sin(theta3_3))/(((a3*cos(theta3_3)+a2)^2+(a3*sin(theta3_3))^2));
c2_4=(L1_3*(a3*cos(theta3_4)+a2)+L2_3*a3*sin(theta3_4))/(((a3*cos(theta3_4)+a2)^2+(a3*sin(theta3_4))^2));
s2_4=(L2_3*(a3*cos(theta3_4)+a2)-L1_3*a3*sin(theta3_4))/(((a3*cos(theta3_4)+a2)^2+(a3*sin(theta3_4))^2));
theta2_3=atan2(s2_3,c2_3);
theta2_4=atan2(s2_4,c2_4);
theta4_3=beta2-theta2_3-theta3_3;
theta4_4=beta2-theta2_4-theta3_4;
% eight kinds of theta2,theta3,theta4-----------5/6
L1_5=cos(theta1_2)*px+sin(theta1_2)*py+d6*cos(beta3)*sin(theta5_3)-d5*sin(beta3);
L2_5=pz-d1+d6*sin(beta3)*sin(theta5_3)+d5*cos(beta3);
theta3_5=real(acos((L1_5^2+L2_5^2-a3^2-a2^2)/2/a3/a2));
theta3_6=-real(acos((L1_5^2+L2_5^2-a3^2-a2^2)/2/a3/a2));
c2_5=(L1_5*(a3*cos(theta3_5)+a2)+L2_5*a3*sin(theta3_5))/(((a3*cos(theta3_5)+a2)^2+(a3*sin(theta3_5))^2));
s2_5=(L2_5*(a3*cos(theta3_5)+a2)-L1_5*a3*sin(theta3_5))/(((a3*cos(theta3_5)+a2)^2+(a3*sin(theta3_5))^2));
c2_6=(L1_5*(a3*cos(theta3_6)+a2)+L2_5*a3*sin(theta3_6))/(((a3*cos(theta3_6)+a2)^2+(a3*sin(theta3_6))^2));
s2_6=(L2_5*(a3*cos(theta3_6)+a2)-L1_5*a3*sin(theta3_6))/(((a3*cos(theta3_6)+a2)^2+(a3*sin(theta3_6))^2));
theta2_5=atan2(s2_5,c2_5);
theta2_6=atan2(s2_6,c2_6);
theta4_5=beta3-theta2_5-theta3_5;
theta4_6=beta3-theta2_6-theta3_6;
% eight kinds of theta2,theta3,theta4-----------7/8
L1_7=cos(theta1_2)*px+sin(theta1_2)*py+d6*cos(beta4)*sin(theta5_4)-d5*sin(beta4);
L2_7=pz-d1+d6*sin(beta4)*sin(theta5_4)+d5*cos(beta4);
theta3_7=real(acos((L1_7^2+L2_7^2-a3^2-a2^2)/2/a3/a2));
theta3_8=-real(acos((L1_7^2+L2_7^2-a3^2-a2^2)/2/a3/a2));
c2_7=(L1_7*(a3*cos(theta3_7)+a2)+L2_7*a3*sin(theta3_7))/(((a3*cos(theta3_7)+a2)^2+(a3*sin(theta3_7))^2));
s2_7=(L2_7*(a3*cos(theta3_7)+a2)-L1_7*a3*sin(theta3_7))/(((a3*cos(theta3_7)+a2)^2+(a3*sin(theta3_7))^2));
c2_8=(L1_7*(a3*cos(theta3_8)+a2)+L2_7*a3*sin(theta3_8))/(((a3*cos(theta3_8)+a2)^2+(a3*sin(theta3_8))^2));
s2_8=(L2_7*(a3*cos(theta3_8)+a2)-L1_7*a3*sin(theta3_8))/(((a3*cos(theta3_8)+a2)^2+(a3*sin(theta3_8))^2));
theta2_7=atan2(s2_7,c2_7);
theta2_8=atan2(s2_8,c2_8);
theta4_7=beta4-theta2_7-theta3_7;
theta4_8=beta4-theta2_8-theta3_8;

% modify theta4 and let it belong to (-pi,pi]
if theta4_1>pi
    theta4_1=theta4_1-2*pi;
else
end

if theta4_1<(-pi)
    theta4_1=theta4_1+2*pi;
else
end

if theta4_2>pi
    theta4_2=theta4_2-2*pi;
else
end

if theta4_2<(-pi)
    theta4_2=theta4_2+2*pi;
else
end

if theta4_3>pi
    theta4_3=theta4_3-2*pi;
else
end

if theta4_3<(-pi)
    theta4_3=theta4_3+2*pi;
else
end

if theta4_4>pi
    theta4_4=theta4_4-2*pi;
else
end

if theta4_4<(-pi)
    theta4_4=theta4_4+2*pi;
else
end

if theta4_5>pi
    theta4_5=theta4_5-2*pi;
else
end

if theta4_5<(-pi)
    theta4_5=theta4_5+2*pi;
else
end

if theta4_6>pi
    theta4_6=theta4_6-2*pi;
else
end

if theta4_6<(-pi)
    theta4_6=theta4_6+2*pi;
else
end

if theta4_7>pi
    theta4_7=theta4_7-2*pi;
else
end

if theta4_7<(-pi)
    theta4_7=theta4_7+2*pi;
else
end

if theta4_8>pi
    theta4_8=theta4_8-2*pi;
else
end

if theta4_8<(-pi)
    theta4_8=theta4_8+2*pi;
else
end

angle{1}=[theta1_1,theta2_1,theta3_1,theta4_1,theta5_1,theta6_1];
angle{2}=[theta1_1,theta2_2,theta3_2,theta4_2,theta5_1,theta6_1];
angle{3}=[theta1_1,theta2_3,theta3_3,theta4_3,theta5_2,theta6_2];
angle{4}=[theta1_1,theta2_4,theta3_4,theta4_4,theta5_2,theta6_2];
angle{5}=[theta1_2,theta2_5,theta3_5,theta4_5,theta5_3,theta6_3];
angle{6}=[theta1_2,theta2_6,theta3_6,theta4_6,theta5_3,theta6_3];
angle{7}=[theta1_2,theta2_7,theta3_7,theta4_7,theta5_4,theta6_4];
angle{8}=[theta1_2,theta2_8,theta3_8,theta4_8,theta5_4,theta6_4];

end

