function [ans_index,dis] = find_closed_point(this_point,point_list)
this_x = this_point(1,1);
this_y = this_point(1,2);
dis = 100000000000000000000000000;
index = 1 ;
for i=1:size(point_list,1)
    x = point_list(i,2);
    y = point_list(i,3);
    new_dis = (this_x-x)^2 + (this_y-y)^2;
    if new_dis < dis
        dis = new_dis;
        index = point_list(i,1);
    end
end
ans_index = index;
end

