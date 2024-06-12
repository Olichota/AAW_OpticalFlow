clear all; close all;

for i=1:7
    im1 = imread(sprintf('../data/im%d.bmp', i));
    im2 = imread(sprintf('../data/im%d.bmp', i+1));
    
    im1 = rgb2gray(im1);
    im2 = rgb2gray(im2);
    
    windowSize = 9;
    halfWindow = int16((windowSize-1)/2);
    
    [rows, cols] = size(im1);
    u = zeros(rows, cols);
    v = zeros(rows, cols);
    
    % Obliczanie gradientów
    [Ix, Iy] = imgradientxy(im1);
    It = double(im2) - double(im1);
    
    % Pętla po pikselach
    for y = halfWindow+1 : rows-halfWindow
        for x = halfWindow+1 : cols-halfWindow
    
            % Wyodrębnienie okien gradientów i różnicy czasowej
            Ix_win = Ix(y-halfWindow:y+halfWindow, x-halfWindow:x+halfWindow);
            Iy_win = Iy(y-halfWindow:y+halfWindow, x-halfWindow:x+halfWindow);
            It_win = It(y-halfWindow:y+halfWindow, x-halfWindow:x+halfWindow);
    
            % Konstrukcja macierzy A i wektora b
            A = [Ix_win(:) Iy_win(:)]' * [Ix_win(:) Iy_win(:)];
            b = -[Ix_win(:) Iy_win(:)]' * It_win(:);
    
            % Rozwiązanie układu równań
            if det(A) > 0.001 % Sprawdzenie, czy macierz jest odwracalna
                uv = A \ b;
                u(y, x) = uv(1);
                v(y, x) = uv(2);
            end
        end
    end
    
    magnitude = sqrt(u.^2 + v.^2);
    angle = -atan2(v, u);
    
    magnitude = magnitude / double(halfWindow);  % Normalizacja do zakresu [0, 1]
    angle = (angle + pi) / (2 * pi);          % Normalizacja do zakresu [0, 1]
    
    % Tworzenie obrazu HSV
    hsvImage = cat(3, (angle), magnitude, ones(size(angle)));
    % hsvImage = cat(3, (angle), ones(size(angle)), ones(size(angle)));
    
    % Konwersja HSV na RGB
    rgbImage = hsv2rgb(hsvImage);
    
    % Wyświetlanie obrazu
    figure(1)
    imshow(rgbImage);
    imwrite(rgbImage, sprintf('outMATLAB/outMATLABoptFlow%d-%d.bmp', i, i+1));
end
