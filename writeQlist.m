function [] = writeQlist(Q)
% 将关节角写成一个txt文件
f = fopen('move.txt','w');
length = size(Q,1);
for i = 1:length
    fprintf(f,'%s,',num2str(Q(i,1)));
    fprintf(f,'%s,',num2str(Q(i,2)));
    fprintf(f,'%s,',num2str(Q(i,3)));
    fprintf(f,'%s,',num2str(Q(i,4)));
    fprintf(f,'%s,',num2str(Q(i,5)));
    fprintf(f,'%s,',num2str(Q(i,6)));
    fprintf(f,'%s,',num2str(Q(i,7)));
    fprintf(f,'%s,',num2str(Q(i,8)));
    fprintf(f,'%s,',num2str(Q(i,9)));
    fprintf(f,'%s,',num2str(Q(i,10)));
    fprintf(f,'%s,',num2str(Q(i,11)));
    fprintf(f,'%s\n',num2str(Q(i,12)));
end

end

