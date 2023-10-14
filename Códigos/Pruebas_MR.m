% filename = '5P_sinVi.gif';          % Nombre del archivo
% DelayTime = 0.1;                    % T. en seg. que dura cada plot en el GIF
f = figure(1);                      % Creación de la figura

% Parámetros de la simulación
tiempo_simulacion = 50; % Tiempo de simulación en segundos
paso_tiempo = 0.01;      % Tamaño del paso de tiempo en segundos
masa_particula = 1.0;    % Masa de cada partícula
espacio = 2.0;           % Tamaño del espacio a utilizar

% Algunos casos específicos 2D posiblemente interesantes
% Dos partículas a una distancia de 1 
% sin velocidades iniciales
% num_particulas = 2;
% 
% posiciones = ...
%     [0.5 1 0;...
%      1.5 1 0];
% 
% velocidades = ...
%     [0 0 0;...
%      0 0 0];
% 
% aceleraciones = ...
%     [0 0 0;...
%      0 0 0];

% Cinco partículas en una configuración simétrica
% sin velocidades iniciales
% num_particulas = 5;
% 
% posiciones = ...
%    [0.5 1.0 0;...
%     1.5 1.0 0;...
%     1.0 0.5 0;...
%     1.0 1.5 0;...
%     1.0 1.0 0];
% 
% velocidades = ...
%    [0 0 0;...
%     0 0 0;...
%     0 0 0;...
%     0 0 0;...
%     0 0 0];
% 
% aceleraciones = ...
%    [0 0 0;...
%     0 0 0;...
%     0 0 0;...
%     0 0 0;...
%     0 0 0];

% Cinco partículas en una configuración simétrica
% con velocidades perpendiculares
num_particulas = 5;

posiciones = ...
   [0.5 1.0 0;...
    1.5 1.0 0;...
    1.0 0.5 0;...
    1.0 1.5 0;...
    1.0 1.0 0];

velocidades = ...
   [ 0  1 0;...
     0 -1 0;...
    -1  0 0;...
     1  0 0;...
     0  0 0];

aceleraciones = ...
   [0 0 0;...
    0 0 0;...
    0 0 0;...
    0 0 0;...
    0 0 0];

% Matrices para almacenar las coordenadas de posición de cada partícula 
% a lo largo del tiempo
trayectorias_x_acum = ...
    zeros(num_particulas, tiempo_simulacion / paso_tiempo);
trayectorias_y_acum = ...
    zeros(num_particulas, tiempo_simulacion / paso_tiempo);
trayectorias_z_acum = ...
    zeros(num_particulas, tiempo_simulacion / paso_tiempo);

% Parámetros del potencial de Lennard-Jones
epsilon = 1.0; % Parámetro epsilon (intensidad)
sigma = 1.0;   % Parámetro sigma (Alcance)

% Bucle de simulación con el algoritmo de Verlet
num_pasos = tiempo_simulacion / paso_tiempo;
posiciones_ant = posiciones;

for paso = 1:num_pasos
    % Calcular fuerzas entre partículas    
    fuerzas = calcular_fuerza(posiciones_ant, epsilon, sigma);
    
    % Calcular aceleraciones
    aceleraciones = fuerzas / masa_particula;
    
   % Actualizar velocidades utilizando el algoritmo de Verlet
    velocidades = velocidades + (aceleraciones * paso_tiempo);
    
    % Actualizar posiciones utilizando el algoritmo de Verlet
    posiciones = ...
        posiciones + (velocidades * paso_tiempo) ...
        + (0.5 * aceleraciones * paso_tiempo^2);
    
     % Verificar colisiones con las paredes
    for i = 1:num_particulas
        for dim = 1:3
            if posiciones(i, dim) < 0 || posiciones(i, dim) > espacio
                % Invertir la velocidad en la dimensión correspondiente
                velocidades(i, dim) = -velocidades(i, dim);
            end
        end
    end

    % Almacenar las coordenadas de posición de cada partícula 
    % en este paso de tiempo
    trayectorias_x_acum(:, paso) = posiciones(:, 1);
    trayectorias_y_acum(:, paso) = posiciones(:, 2);
    trayectorias_z_acum(:, paso) = posiciones(:, 3);
    
    % Actualizar posiciones y aceleraciones para el siguiente 
    % paso de tiempo
    posiciones_ant = posiciones;
    
    % Dibujar las partículas en la figura actualizada
    scatter3(posiciones(:, 1), ...
             posiciones(:, 2), ...
             posiciones(:, 3), ...
             10, 'filled', 'MarkerFaceAlpha', 0.5);
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    title('Simulación de Dinámica Molecular en 3D');
    grid on;
    % Establece los límites del cubo
    axis([0, espacio, 0, espacio, 0, espacio]); 
    
    % Pausa para permitir que MATLAB actualice la figura
    pause(0.01);
    
    
%     frame = getframe(f);
%     % Convertir la trama en una imagen RGB (3 dimensiones) 
%     im = frame2im(frame);
%     
%     % Transformar muestras de RGB a 1 dimensión con un mapa de color "cm" 
%     [imind,cm] = rgb2ind(im , 256); 
%     if paso == 1
%         % Crear el archivo GIF
%         imwrite( ...
%           imind, cm, filename, 'gif', 'DelayTime', ...
%           DelayTime , 'LoopCount' , Inf);
%     else
%         % Añadir al GIF cada nuevo plot
%         imwrite(imind, cm, filename, 'gif', 'WriteMode', ...
%           'append', 'DelayTime', DelayTime);
%     end
end

% % Inicializa la figura para la animación
% figure(1);
% hold on; 
% % Actualiza el gráfico con las trayectorias acumuladas hasta este 
% % paso de tiempo
% for i = 1:num_particulas
%     scatter3( ...
%       trayectorias_x_acum(i, 1:paso), ...
%       trayectorias_y_acum(i, 1:paso), ...
%       trayectorias_z_acum(i, 1:paso), ...
%       10, 'filled', 'MarkerFaceAlpha', 0.5);
% end
% 
% xlabel('X');
% ylabel('Y');
% zlabel('Z');
% title('Simulación de Dinámica Molecular con Trayectorias (3D)');
% grid on;
% % Establece los límites del cubo
% axis([0, espacio, 0, espacio, 0, espacio]); 
% hold off;

% Función para calcular las fuerzas entre partículas (utilizando el potencial de Lennard-Jones)
function fuerzas = calcular_fuerza(posiciones, epsilon, sigma)
    num_particulas = size(posiciones, 1);
    dimension = size(posiciones, 2);
    fuerzas = zeros(num_particulas, dimension);
    
    for i = 1:num_particulas
        for j = i+1:num_particulas
            % Calcula el vector entre las partículas
            r_ij = posiciones(i, :) - posiciones(j, :);
            % Calcular la distancia entre particulas
            r = norm(r_ij);       
            
            % Calcular la fuerza 
            fuerza_ij = ...
                24*epsilon*( 2*(sigma/r)^12 - (sigma/r)^6 )*r_ij/r^2;
            
            % Aplicar la fuerza a ambas partículas (ley de acción y reacción)
            fuerzas(i, :) = fuerzas(i, :) + fuerza_ij;
            fuerzas(j, :) = fuerzas(j, :) - fuerza_ij;
        end
    end
end
