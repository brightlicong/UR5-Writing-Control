function T = cordinate2T(x,z,x_length,z_length)
%UNTITLED2 �˴���ʾ�йش˺�����ժҪ
%   �˴���ʾ��ϸ˵��
zoom_x = 0.003;
zoom_z = 0.005;
x = zoom_x*(x-0.5*x_length);
z = 0.5+zoom_z*(0.5*z_length-z);
T = [1 0 0 x;
     0 1 0 0;
     0 0 1 z;
     0 0 0 1];
 

end

