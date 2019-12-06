function [] = animate(maps, armplan, armplanlength, replanned)

close all;
LINKLENGTH_CELLS=15;
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

%Text:
txt = text(5,size(maps,1)-5, "Moving");
txt.Color = 'r';
txt.FontSize = 25;

for t = 1:speed:armplanlength
    for i = 1:size(armplan,2)
        x(i+1) = x(i) + LINKLENGTH_CELLS*cos(armplan(floor(t),i));
        y(i+1) = y(i) + LINKLENGTH_CELLS*sin(armplan(floor(t),i));
    end
    
    if replanned(floor(t)) == 0
        set(txt, 'String', "Moving Arm");
    elseif replanned(floor(t)) == 1
        set(txt, 'String', "Replanning");
    end
    
    set(m, 'cdata', maps(:,:,mod(floor(t),m_size)+1));
    set(a, 'XData', x);
    set(a, 'YData', y);
    
    
    drawnow;
end

end