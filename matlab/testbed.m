%% Project testbed

%% Load coover hallway image
hallway = imread('coover_hallway.jpg');
hallway_gray = rgb2gray(hallway);

%% Find edges using Canny edge detector
hallway_edges = edge(hallway_gray, 'Sobel');
figure; imshow(hallway_edges); hold on;

%% Detect lines using Hough Transform
[H, T, R] = hough(hallway_edges);
threshPeaks = ceil(0.3*max(H(:)));
P = houghpeaks(H, 50, 'threshold', threshPeaks);
lines = houghlines(hallway_edges, T, R, P, 'FillGap', 20, 'MinLength', 40);
figure, imshow(hallway_gray), hold on;
max_len = 0;
for k = 1:length(lines)
   xy = [lines(k).point1; lines(k).point2];
   plot(xy(:,1),xy(:,2),'LineWidth',2,'Color','green');

   % Plot beginnings and ends of lines
   plot(xy(1,1),xy(1,2),'x','LineWidth',2,'Color','yellow');
   plot(xy(2,1),xy(2,2),'x','LineWidth',2,'Color','red');

   % Determine the endpoints of the longest line segment
   len = norm(lines(k).point1 - lines(k).point2);
   if ( len > max_len)
      max_len = len;
      xy_long = xy;
   end
end
