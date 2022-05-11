function ground_height = rough_terrain(P)
% h_ground from task description (cm)
h_ground = [0; 0; -3; 5; 2; -5; 7; -6; -4; 2;
            4; 6; 3; 0; 3; 0; 3; 0; 5; 0;
            4; 6; -5; -7; -3; -2; 5; 7; 10; 0;
            5; -5; 5; -5; 5; -5; 5; -5; 5; -5;
            0; 10; 20; 30; 40; 50; 60; 70; 80; 90;
            90; 90; 90; 90; 90; 90; 90; 90; 90; 90];
x = P(1);
y = P(2);

if y<=0.25 && y>=-0.25 && x>=0 && x<12
    k = floor(x/0.2)+1;
    ground_height = 0.01*h_ground(k);
else
    ground_height = 0;
end
end

