function [] = animate(map, armplan, armplanlength)

close all;
LINKLENGTH_CELLS=10;
speed = 1;

figure

%Map:
m = imagesc(map(1));
axis square;

%Arm:
x = zeros(length(armplan(1,:)),1);
y = zeros(length(armplan(1,:)),1);
x(1) = size(envmap, 3)/2;
a = plot(x, y, 'c-');

for t = 1:speed:armplanlength
    for i = 1:size(armplan,2)
        x(floor(t)+1) = x(floor(t)) + LINKLENGTH_CELLS*cos(armplan(i,floor(t)));
        y(floor(t)+1) = y(floor(t)) + LINKLENGTH_CELLS*sin(armplan(i,floor(t)));
    end
    
    set(m, 'cdata', map(floor(t),:,:));
    set(a, 'XData', x);
    set(a, 'YData', y);
    
    drawnow;
end

end