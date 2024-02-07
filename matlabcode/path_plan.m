function point_sorted=path_plan(point_unsorted,pic_size)
    %{
        plan path and sort point by path
        input:
            point_unsorted: nx2 matrix of block point postion
            pic_size: picture size
        output:
            point_sorted: sorted points' postion. If the distance between 
            two points is too long, we add a [-1 -1] point a [-2 -2] point
            between two points  
    %}
    pos_sorted=[0 0];
    i=1;
    id=1;
    res=1/pic_size*0.16;
    while length(point_unsorted)~=2
        x=point_unsorted(i,1);
        y=point_unsorted(i,2);
        [d, i2]=dis(x,y,point_unsorted,i);
        x3=point_unsorted(i2,1);
        y3=point_unsorted(i2,2);
        pos_sorted(id,:)=point_unsorted(i,:);
        id=id+1;
        point_unsorted(i,:)=[];
        if d>5*res
            pos_sorted(id,:)=[-1 -1];
            id=id+1;
            pos_sorted(id,:)=[-2 -2];
            id=id+1;
            % [d, i2]=dis(0,0,point_unsorted,i);
            % i=i2;
            % continue
        end
        i=findd(x3,y3,point_unsorted);
    end
    pos_sorted=[pos_sorted;point_unsorted];
    point_sorted=pos_sorted;
end

function [d,idx]=dis(x,y,po,i)
    min=10;
    for j=1:length(po)
        if j~=i
            x2=po(j,1);
            y2=po(j,2);
            m=sqrt((x2-x)^2+(y2-y)^2);
            if m<min
                min=m;
                idx=j;
            end
        end
    end
    d=min;
end

function ind=findd(x,y,po)
    for j=1:length(po)
        if po(j,1)==x && po(j,2)==y
            ind=j;
           % fprintf("found %f %f\n",x,y);
            % fprintf("i=%4d",i);
            return
        end
    end
    ind=1;
end