function point=pic_2_point(picture,pic_size)
    %{
        read block points of picture and return a 3D ponts position matrix
        input:
            picture: a matrix of the picture
            pic_size: the size we want picture to be
        output:
            point: a nx3 matrix
    %}
    pic_gray=rgb2gray(picture);
    pic_resized=imresize(pic_gray,[pic_size pic_size]);
    highpass_filter=[-1 -1 -1;-1 15 -1;-1 -1 -1];
    pic_filted=imfilter(pic_resized,highpass_filter);
    % change block place to high value
    position_unsorted=get_pos(pic_filted,pic_size);
    position_sorted=path_plan(position_unsorted,pic_size);
    point=add_mid_point(position_sorted);

end

function point_unsorted=get_pos(picture,pic_size)
    %{
        get block point positions and limit them in a 16*16cm^2 block
        with x offset 14cm and y offset 8cm from robot arm origin
        input:
            picture: a filted picture (pic_sizexpic_size matrix)
            pic_size: size of picture (int)
        output:
            point_unsorted: a nx2 matrix contain block point position 
            with n: num of block point on the picture 2:x pos and y pos
    %}
    pic=255-picture;
    data.x=0;
    data.y=0;
    for i=pic_size:-1:1
        for j=1:pic_size
            if pic(i,j)>=255
                data.x=[data.x j];
                data.y=[data.y -i+pic_size];
            end
        end
    end
    data.x=data.x(2:end)/(pic_size/0.16)+0.14;
    data.y=data.y(2:end)/(pic_size/0.16)-0.08;
    point_unsorted=zeros(length(data.x),2);
    for i=1:length(data.x)
        point_unsorted(i,1)=data.x(i);
        point_unsorted(i,2)=data.y(i);
    end
end

function point=add_mid_point(poo)
    %add middle point between long disted points and change
    %2D position to 3D
    q=zeros(length(poo),3);
    for i=1:length(poo)
        if poo(i,1)==-1
            q(i,1)=poo(i-1,1);
            q(i,2)=poo(i-1,2);
            q(i,3)=0.02;
        elseif poo(i,1)==-2
            q(i,1)=poo(i+1,1);
            q(i,2)=poo(i+1,2);
            q(i,3)=0.02;
        else
            q(i,1)=poo(i,1);
            q(i,2)=poo(i,2);
            q(i,3)=0;
        end
    end
    point=q;
end