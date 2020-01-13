function [step] = get_steps_by_dis(x1,y1,x2,y2)
%UNTITLED2 此处显示有关此函数的摘要
%   此处显示详细说明
dis = 500*sqrt((x1-x2)^2+(y1-y2)^2);
s = round(dis);
step_max = 50;
step_min = 2;
if s < step_min
    step = step_min;
elseif s>step_max
    step = step_max;
else
    step = s;
end

end
