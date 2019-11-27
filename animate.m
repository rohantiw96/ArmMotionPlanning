function [] = animate(maps, armplan, armplanlength)

close all;
LINKLENGTH_CELLS=10;
speed = 0.1;

figure

%Map:
m = imagesc(maps(:,:,1));
axis square;
hold on;

%Arm:
x = zeros(size(armplan,2)+1, 1);
y = zeros(size(armplan,2)+1, 1);
x(1) = size(maps, 2)/2;
a = plot(x, y, 'c-');

for t = 1:speed:armplanlength
    for i = 1:size(armplan,2)
        x(floor(t)+1) = x(floor(t)) + LINKLENGTH_CELLS*cos(armplan(floor(t),i));
        y(floor(t)+1) = y(floor(t)) + LINKLENGTH_CELLS*sin(armplan(floor(t), i));
    end
    
    set(m, 'cdata', maps(:,:,floor(t)));
    set(a, 'XData', x);
    set(a, 'YData', y);
    
    drawnow;
end

end