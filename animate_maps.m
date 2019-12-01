function [] = animate_maps(maps)

close all;
speed = 0.1;
figure('units','normalized','outerposition',[0 0 1 1]);

%Map:
m = imagesc(maps(:,:,1));
axis square;
hold on;

for t = 1:speed:size(maps,3)
    
    set(m, 'cdata', maps(:,:,floor(t)));
    drawnow;
end

end