clear all;
close all;

daneCPU = [0.0241065
0.217176
1.90277
];
daneGPU = [0.0030917
0.0060081
0.149966
];
daneMatlab = [0.334397
0.635953
0.946129
];

x = [256, 1024, 4096];

figure;
plot(x, daneCPU, 'ro-', 'LineWidth', 2, 'MarkerSize', 10);
hold on;
plot(x, daneGPU, 'bo-', 'LineWidth', 2, 'MarkerSize', 10);
plot(x, daneMatlab, 'go-', 'LineWidth', 2, 'MarkerSize', 10);

title('Porównanie czasów obliczeń - kola r=4');
xlabel('Bok zdjęcia w px');
ylabel('Czas (s)');
legend('CPU', 'GPU', 'Matlab');
grid on;
hold off;