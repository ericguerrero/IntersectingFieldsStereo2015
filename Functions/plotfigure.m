function plotfigure(I1,I2,set1,set2,w,name,color)

s1=size(I1,1);
s2=size(I2,1);
if s1>s2
    I1=I1(1:s2,:,:);
else
    I2=I2(1:s1,:,:);
end
I=[I1 I2];
imshow(I); hold on
if ~isempty(set1)
    for i=1:size(set1,1)
        plot([set1(i,1) set2(i,1)+w],[set1(i,2) set2(i,2)],color);
    end
    plot(set1(:,1),set1(:,2),'og');
    plot(set2(:,1)+w,set2(:,2),'or');
end
title([name ': ' num2str(size(set1,1))]);
end