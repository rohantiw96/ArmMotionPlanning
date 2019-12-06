function [] = animate(maps, armplan, armplanlength)

close all;
LINKLENGTH_CELLS=10;
speed = 0.1;

figure

%Map:
m = imagesc(maps(:,:,1));
m_size = size(maps, 3);
axis square;
hold on;

%Arm:
x = zeros(size(armplan,2)+1, 1);
y = zeros(size(armplan,2)+1, 1);
x(1) = size(maps, 2)/2;
a = plot(x, y, 'c-');

for t = 1:speed:armplanlength
    for i = 1:size(armplan,2)
        x(i+1) = x(i) + LINKLENGTH_CELLS*cos(armplan(floor(t),i));
        y(i+1) = y(i) + LINKLENGTH_CELLS*sin(armplan(floor(t),i));
    end
    
    set(m, 'cdata', maps(:,:,mod(floor(t),m_size)+1));
    set(a, 'XData', x);
    set(a, 'YData', y);
    
    drawnow;
end

end