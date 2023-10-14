clear all;
clc;

% Parámetros
m = 1;            % Masa de las partículas dinámicas
M = 1e20;         % Masa de la partícula estática
G = 6.67e-11;     % Constante de gravitación

% Condiciones iniciales para tres partículas (roja, azul y anaranjada)
x = zeros(2, 3);  % Inicializar matriz de posición
v = zeros(2, 3);  % Inicializar matriz de velocidad

% Posiciones iniciales de las partículas
x(:, 1) = [-1000; 0];  % Posición inicial de la partícula roja
x(:, 2) = [-120; 0];   % Posición inicial de la partícula azul
x(:, 3) = [-600; 0];   % Posición inicial de la partícula anaranjada

% Velocidades iniciales para que las partículas orbiten
% v(:, 1) = [0; -sqrt(G * M / norm(x(:, 1)))];  % Velocidad inicial de la partícula roja
% v(:, 2) = [0; -sqrt(G * M / norm(x(:, 2)))];  % Velocidad inicial de la partícula azul
% v(:, 3) = [0; -sqrt(G * M / norm(x(:, 3)))];  % Velocidad inicial de la partícula anaranjada
v(:, 1) = [0; -2000];  % Velocidad inicial de la partícula roja
v(:, 2) = [0; -6880];  % Velocidad inicial de la partícula azul
v(:, 3) = [0; -2300];  % -3K Velocidad inicial de la partícula anaranjada

% Paso de integración
h = 0.01;

% Preparar la figura
f = figure(1);
hold on;

% Variables para almacenar trayectorias
trayectorias = cell(1, 3);  % Celdas para almacenar las trayectorias de las partículas
colores = ['r', 'b', 'm'];  % Colores para las partículas (rojo, azul, anaranjado)

% Bucle principal
for step = 1:150
    for i = 1:3
        % Calcular la fuerza actuante sobre la partícula i
        F = -G * (m * M / norm(x(:, i))^2) * (x(:, i) / norm(x(:, i)));
        
        % Actualizar la velocidad y la posición utilizando el método de Euler
        v(:, i) = v(:, i) + h * (F / m);
        x(:, i) = x(:, i) + h * v(:, i);
        
        % Almacenar la posición en las trayectorias de las partículas
        trayectorias{i} = [trayectorias{i} x(:, i)];
    end
    
    % Graficar las trayectorias de las partículas
    for i = 1:3
        scatter(trayectorias{i}(1, :), trayectorias{i}(2, :), colores(i), "o");
    end
    
    % Graficar la partícula estática en el centro
    scatter(0, 0, 'k', 'filled');
    
    title(['Paso: ' num2str(step)]);
    xlabel('X(m)');
    ylabel('Y(m)');
    axis([-1500 1500 -1500 1500]);
    drawnow;
end
