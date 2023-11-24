%%                        Suavizar trayectorias

%% Limpiar todo
clear all;
clc;

%% Parámetros del potencial de Lennard-Jones
epsilon = 1.0;          
sigma = 1.0;
num_particulas = 1; 
tiempo_simulacion = 8;  % Tiempo de simulación en segundos (30 segundos en este ejemplo)
paso_tiempo = 0.009;      % Tamaño del paso de tiempo en segundos
masa_particula = 1.0;   % Masa de cada partícula
espacio = 1;          % Tamaño del espacio a utilizar

% Bucle de simulación para calcular trayectorias
num_pasos = tiempo_simulacion / paso_tiempo;


%% Trayectoria con dinamica molecular
%posiciones = zeros(1,2);
%posiciones = rand(num_particulas, 2)*espacio;
posiciones = [0.5,0.5] 
posiciones0 = posiciones;
velocidades = [2,1];                      % Velocidades iniciales aleatorias en 2D
aceleraciones = zeros(num_particulas, 2); % Inicializar aceleraciones a cero en 2D
posiciones_ant = posiciones;
x_path = [];
y_path = [];
trayectoria = [];

for paso = 1:320
    % Calcular fuerzas entre partículas (utilizando el potencial de Lennard-Jones)
    fuerzas = calcular_fuerza(posiciones_ant, epsilon, sigma);
    
    % Calcular aceleraciones
    aceleraciones = fuerzas / masa_particula;
    
    % Actualizar velocidades utilizando el algoritmo de Verlet
    velocidades = velocidades + (aceleraciones * paso_tiempo);
    
    % Actualizar posiciones utilizando el algoritmo de Verlet
    posiciones = posiciones + (velocidades * paso_tiempo) + (0.5 * aceleraciones * paso_tiempo^2);
    
    % Verificar colisiones con las paredes y aplicar rebote
    for i = 1:num_particulas
        for dim = 1:2
            if posiciones(i, dim) < 0 || posiciones(i, dim) >= espacio
                % Invertir la velocidad en la dimensión correspondiente
                velocidades(i, dim) = -velocidades(i, dim);
            end
        end
    end

    % Actualizar posiciones y aceleraciones para el siguiente paso de tiempo
    posiciones_ant = posiciones;
    
    % Dibujar la    s partículas en la figura actualizada
    scatter(posiciones(:, 1), posiciones(:, 2), 10, 'filled', 'MarkerFaceAlpha', 0.5);
    x_path(end+1) = posiciones(:,1);
    y_path(end+1) = posiciones(:,2);
    trayectoria(end+1,:) = posiciones
    xlabel('X (m)');
    ylabel('Y (m)');    
    title('Simulación de Trayectorias de Partículas (2D)');
    grid on;
    axis([0, espacio, 0, espacio]);
    
    % Pausa para permitir que MATLAB actualice la figura
    pause(paso_tiempo);
end

% trayectoria = trayectoria/10;
% x_path = x_path/10;
% y_path = y_path/10;
%%
largo = length(trayectoria)/8; % longitud de datos dividido 4
xy_puntos = [];

for j = 1:largo
    xy_puntos(end+1,:) = trayectoria(8*j,:);
end
xy_puntos(end+1,:) = trayectoria(end,:);

% interpolación mediante b-splines y generando puntos cada 0.25 en x para mayor fineza
%trayec = bsplinepolytraj(xy_puntos',[0,length(xy_puntos)-1],0:0.25:length(xy_puntos)-1);
trayec = bsplinepolytraj(xy_puntos',[0,length(xy_puntos)],0:0.08:length(xy_puntos));

trayec = trayec';
%% Graficar la trayectoria
%scatter(x(1,:),x(2,:), 'green','filled');
figure(2);
hold on;
grid on;
title('Suavizado en la simulación de Trayectorias de Partículas (2D)');
plot(x_path,y_path,'green')
xlabel('X(m)');
ylabel('Y(m)');
axis([0, espacio, 0, espacio]);
drawnow

for i = 1:num_particulas
    plot(posiciones0(i, 1), posiciones0(i, 2),'o', 'MarkerEdgeColor', 'red', 'MarkerSize', 4); % Posicion inicial
    plot(trayec(:,1), trayec(:,2));
    plot(posiciones(i, 1), posiciones(i, 2),'pentagram', 'MarkerEdgeColor', 'black');
end
pbaspect([1 1 1])
hold off;

% hold on
% plot(trayec(:,1), trayec(:,2))

%% Función para calcular las fuerzas entre partículas (utilizando el potencial de Lennard-Jones)
function fuerzas = calcular_fuerza(posiciones, epsilon, sigma)
    num_particulas = size(posiciones, 1);
    dimension = size(posiciones, 2);
    fuerzas = zeros(num_particulas, dimension);
    
    for i = 1:num_particulas
        for j = 1:num_particulas
            if i ~= j 

                r_ij = posiciones(i, :) - posiciones(j, :);
                r = norm(r_ij);
                
                % Calcular la fuerza
                fuerza_ij = 24 * epsilon * (2 * (sigma / r)^12 - (sigma / r)^6) * r_ij / r^2;
                
                % Aplicar la fuerza a la partícula i
                fuerzas(i, :) = fuerzas(i, :) + fuerza_ij;
            end
        end
    end
end