img2=img;
wallsize=20;
for i = 1+ wallsize/2: size(img,1)-wallsize/2
    for j=1+ wallsize/2:size(img,2)-wallsize/2
        if img(i,j)==255 && img(i+1,j)==255;
            img2(i,j-wallsize/2:j+wallsize/2)=255;
        end
        if img(i,j)==255 && img(i,j+1)==255;
            img2(i-wallsize/2:i+wallsize/2,j)=255;
        end
    end
end