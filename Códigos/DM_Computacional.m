%% Valerie Valdez                                        Carné: 19659
% Universidad del Valle de Guatemala                    Sección: 20

%                    Implementación de fisica granular
%                           Dinámica molecular
% 
% % Parámetros de la simulación
% num_particulas = 100;
% tiempo_simulacion = 100; % Tiempo de simulación
% paso_tiempo = 0.01;      % Tamaño del paso de tiempo
% masa_particula = 1.0;    % Masa de cada partícula
% espacio = 6.0;           % Tamaño del espacio a utilizar
% 
% % Inicialización de posiciones y velocidades aleatorias
% posiciones = rand(num_particulas, 3);   % Posiciones iniciales aleatorias
% velocidades = randn(num_particulas, 3); % Velocidades iniciales aleatorias
% fuerzas = zeros(num_particulas, 3);     % Inicializar fuerzas a cero
% 
% % Matrices para almacenar las coordenadas de posición de cada partícula a lo largo del tiempo
% trayectorias_x = zeros(num_particulas, tiempo_simulacion / paso_tiempo);
% trayectorias_y = zeros(num_particulas, tiempo_simulacion / paso_tiempo);
% trayectorias_z = zeros(num_particulas, tiempo_simulacion / paso_tiempo);
% 
% % Simulación de dinámica molecular
% num_pasos = tiempo_simulacion / paso_tiempo;
% 
% for paso = 1:num_pasos
%     % Calcular fuerzas entre partículas
%     fuerzas = calcular_fuerza(posiciones);
%     
%     % Actualizar velocidades y posiciones
%     aceleraciones = fuerzas / masa_particula;
%     posiciones_ant = posiciones;
%     posiciones = 2 * posiciones - posiciones_ant + aceleraciones * paso_tiempo^2;
%     velocidades = (posiciones - posiciones_ant) / (2 * paso_tiempo);
%     %velocidades = velocidades + aceleraciones * paso_tiempo;
%     %posiciones = posiciones + velocidades * paso_tiempo;
%    
%     
%     % Aplicar restricciones para mantener las partículas dentro del cubo
%     posiciones(posiciones < 0) = 0;
%     posiciones(posiciones > espacio) = espacio;
%     
%     % Aplicar colisiones elásticas con las paredes del cubo
%     for i = 1:num_particulas
%         for dim = 1:3
%             if posiciones(i, dim) <= 0 || posiciones(i, dim) >= espacio
%                 velocidades(i, dim) = -velocidades(i, dim); % Invertir la velocidad en esa dimensión
%             end
%         end
%     end
%     
%     % Almacenar las coordenadas de posición de cada partícula en este paso de tiempo
%     trayectorias_x(:, paso) = posiciones(:, 1);
%     trayectorias_y(:, paso) = posiciones(:, 2);
%     trayectorias_z(:, paso) = posiciones(:, 3);
% 
% end
% 
% % Visualización de resultados (trayectorias de las partículas)
% figure;
% hold on;
% for i = 1:num_particulas
%     scatter3(trayectorias_x(i,:), trayectorias_y(i,:), trayectorias_z(i,:), 10, 'filled', 'MarkerFaceAlpha', 0.5);
% end
% xlabel('X');
% ylabel('Y');
% zlabel('Z');
% title('Trayectorias de Partículas en la Simulación de Dinámica Molecular (3D)');
% grid on;                % Cuadricula tridimensional
% axis([0, espacio, 0, espacio, 0, espacio]); % Establece los límites del cubo
% hold off;
% 
% % Función para calcular las fuerzas entre partículas (ejemplo de potencial de Lennard-Jones)
% function fuerzas = calcular_fuerza(posiciones)
%     num_particulas = size(posiciones, 1);
%     dimension = size(posiciones, 2);
%     fuerzas = zeros(num_particulas, dimension);
%     
%     epsilon = 1.0; % Parámetro epsilon del potencial de Lennard-Jones
%     sigma = 1.0;   % Parámetro sigma del potencial de Lennard-Jones
%     
%     for i = 1:num_particulas    
%         for j = i+1:num_particulas
%             r = posiciones(j,:) - posiciones(i,:);
%             distancia = norm(r);
%             fuerza = 24 * epsilon * (2 * (sigma/distancia)^12 - (sigma/distancia)^6) * r / distancia^2;
%             fuerzas(i,:) = fuerzas(i,:) + fuerza;
%             fuerzas(j,:) = fuerzas(j,:) - fuerza;
%         end
%     end
% end


% %% CODIGO DOS
% % Parámetros de la simulación
% num_particulas = 100;
% tiempo_simulacion = 100; % Tiempo de simulación
% paso_tiempo = 0.01; % Tamaño del paso de tiempo
% masa_particula = 1.0; % Masa de cada partícula
% tamano_cubo = 10.0; % Tamaño del cubo
% num_particulas_por_lado = round(num_particulas^(1/3)); % Distribución uniforme
% espaciado = tamano_cubo / num_particulas_por_lado;
% amplitud_ruido = 0.1; % Ajusta según sea necesario
% 
% % Inicialización de posiciones en una cuadrícula 3D regular dentro del cubo
% posiciones = zeros(num_particulas, 3);
% index = 1;
% for x = 1:num_particulas_por_lado
%     for y = 1:num_particulas_por_lado
%         for z = 1:num_particulas_por_lado
%             posiciones(index, :) = [x, y, z] * espaciado;
%             index = index + 1;
%         end
%     end
% end
% 
% % Agregar un poco de ruido aleatorio a las posiciones iniciales
% %posiciones = posiciones + amplitud_ruido * randn(num_particulas, 3);
% 
% % Generar ruido aleatorio para cada dimensión por separado
% ruido_x = amplitud_ruido * randn(num_particulas, 1);
% ruido_y = amplitud_ruido * randn(num_particulas, 1);
% ruido_z = amplitud_ruido * randn(num_particulas, 1);
% 
% % Agregar el ruido aleatorio a las posiciones iniciales en cada dimensión
% posiciones(:, 1) = posiciones(:, 1) + ruido_x;
% posiciones(:, 2) = posiciones(:, 2) + ruido_y;
% posiciones(:, 3) = posiciones(:, 3) + ruido_z;
% 
% 
% % Inicialización de velocidades aleatorias
% velocidades = randn(num_particulas, 3);
% 
% % Matrices para almacenar las coordenadas de posición de cada partícula en 3D
% trayectorias_x = zeros(num_particulas, tiempo_simulacion / paso_tiempo);
% trayectorias_y = zeros(num_particulas, tiempo_simulacion / paso_tiempo);
% trayectorias_z = zeros(num_particulas, tiempo_simulacion / paso_tiempo);
% 
% % Simulación de dinámica molecular
% num_pasos = tiempo_simulacion / paso_tiempo;
% for paso = 1:num_pasos
%     % Calcular fuerzas entre partículas (debes definir calcular_fuerza)
%     fuerzas = calcular_fuerza(posiciones);
%     
%     % Actualizar posiciones y velocidades usando el método de Verlet
%     aceleraciones = fuerzas / masa_particula;
%     posiciones_ant = posiciones;
%     posiciones = 2 * posiciones - posiciones_ant + aceleraciones * paso_tiempo^2;
%     velocidades = (posiciones - posiciones_ant) / (2 * paso_tiempo);
%     
%     % Aplicar restricciones para mantener las partículas dentro del cubo
%     posiciones(posiciones < 0) = 0;
%     posiciones(posiciones > tamano_cubo) = tamano_cubo;
%     
%     % Aplicar colisiones elásticas con las paredes del cubo
%     for i = 1:num_particulas
%         for dim = 1:3
%             if posiciones(i, dim) <= 0 || posiciones(i, dim) >= tamano_cubo
%                 velocidades(i, dim) = -velocidades(i, dim); % Invertir la velocidad en esa dimensión
%             end
%         end
%     end
%     
%     % Almacenar las coordenadas de posición de cada partícula en este paso de tiempo
%     trayectorias_x(:, paso) = posiciones(:, 1);
%     trayectorias_y(:, paso) = posiciones(:, 2);
%     trayectorias_z(:, paso) = posiciones(:, 3);
%     
%     % Realizar cualquier otro cálculo o análisis necesario aquí
% end
% 
% % Visualización de resultados (trayectorias de las partículas en 3D)
% figure;
% hold on;
% for i = 1:num_particulas
%     scatter3(trayectorias_x(i,:), trayectorias_y(i,:), trayectorias_z(i,:), 10, 'filled', 'MarkerFaceAlpha', 0.5);
% end
% xlabel('X');
% ylabel('Y');
% zlabel('Z');
% title('Trayectorias de Partículas en la Simulación de Dinámica Molecular (3D)');
% grid on; % Activa la cuadrícula tridimensional
% axis([0, tamano_cubo, 0, tamano_cubo, 0, tamano_cubo]); % Establece los límites del cubo
% hold off;
% 
% % Función para calcular las fuerzas entre partículas (ejemplo de potencial de Lennard-Jones)
% function fuerzas = calcular_fuerza(posiciones)
%     num_particulas = size(posiciones, 1);
%     dimension = size(posiciones, 2);
%     fuerzas = zeros(num_particulas, dimension);
%     
%     epsilon = 1.0; % Parámetro epsilon del potencial de Lennard-Jones
%     sigma = 1.0;   % Parámetro sigma del potencial de Lennard-Jones
%     
%     for i = 1:num_particulas
%         for j = i+1:num_particulas
%             r = posiciones(j,:) - posiciones(i,:);
%             distancia = norm(r);
%             fuerza = 24 * epsilon * (2 * (sigma/distancia)^12 - (sigma/distancia)^6) * r / distancia^2;
%             fuerzas(i,:) = fuerzas(i,:) + fuerza;
%             fuerzas(j,:) = fuerzas(j,:) - fuerza;
%         end
%     end
% end
% 
%% Parámetros de la simulación 3 Y 4 
% num_particulas = 10;
% tiempo_simulacion = 100; % Tiempo de simulación
% paso_tiempo = 0.01;      % Tamaño del paso de tiempo
% masa_particula = 1.0;    % Masa de cada partícula
% espacio = 10.0;           % Tamaño del espacio a utilizar
% 
% % Inicialización de posiciones y velocidades aleatorias
% posiciones = rand(num_particulas, 3) * espacio;   % Posiciones iniciales aleatorias en el espacio
% velocidades = randn(num_particulas, 3); % Velocidades iniciales aleatorias
% fuerzas = zeros(num_particulas, 3);     % Inicializar fuerzas a cero
% 
% % Matrices para almacenar las coordenadas de posición de cada partícula a lo largo del tiempo
% trayectorias_x = zeros(num_particulas, tiempo_simulacion / paso_tiempo);
% trayectorias_y = zeros(num_particulas, tiempo_simulacion / paso_tiempo);
% trayectorias_z = zeros(num_particulas, tiempo_simulacion / paso_tiempo);
% 
% %Simulación de dinámica molecular
% num_pasos = tiempo_simulacion / paso_tiempo;
% 
% for paso = 1:num_pasos
%     % Calcular fuerzas entre partículas (puedes usar un potencial de Lennard-Jones u otro)
%     fuerzas = calcular_fuerza(posiciones);
%     
%     % Actualizar velocidades y posiciones
%     aceleraciones = fuerzas / masa_particula;
%     posiciones = posiciones + velocidades * paso_tiempo + 0.5 * aceleraciones * paso_tiempo^2;
%     velocidades = velocidades + aceleraciones * paso_tiempo;
%     
%     % Aplicar restricciones para mantener las partículas dentro del cubo
%     posiciones(posiciones < 0) = 0;
%     posiciones(posiciones > espacio) = espacio;
%     
%     % Almacenar las coordenadas de posición de cada partícula en este paso de tiempo
%     trayectorias_x(:, paso) = posiciones(:, 1);
%     trayectorias_y(:, paso) = posiciones(:, 2);
%     trayectorias_z(:, paso) = posiciones(:, 3);
% end
% 
% % Visualización de las trayectorias de las partículas
% figure;
% hold on;
% for i = 1:num_particulas
%     plot3(trayectorias_x(i,:), trayectorias_y(i,:), trayectorias_z(i,:));
% end
% xlabel('X');
% ylabel('Y');
% zlabel('Z');
% title('Trayectorias de Partículas en la Simulación de Dinámica Molecular (3D)');
% grid on;                % Cuadricula tridimensional
% axis([0, espacio, 0, espacio, 0, espacio]); % Establece los límites del cubo
% hold off;
% 
% % Función para calcular las fuerzas entre partículas (puedes personalizarla según tus necesidades)
% function fuerzas = calcular_fuerza(posiciones)
%     % Aquí puedes implementar el cálculo de fuerzas entre partículas
%     % Puedes utilizar un potencial de Lennard-Jones u otro potencial según tu simulación
%     % Por simplicidad, se asume que las fuerzas son nulas en este ejemplo
%     num_particulas = size(posiciones, 1);
%     fuerzas = zeros(num_particulas, 3);
% end

%% Codigo 4
% % Parámetros de la simulación
% num_particulas = 100;
% tiempo_simulacion = 100; % Tiempo de simulación en segundos
% paso_tiempo = 0.01;      % Tamaño del paso de tiempo en segundos
% masa_particula = 1.0;    % Masa de cada partícula
% espacio = 10.0;           % Tamaño del espacio a utilizar
% 
% % Inicialización de posiciones y velocidades aleatorias
% posiciones = rand(num_particulas, 3) * espacio;   % Posiciones iniciales aleatorias en el espacio
% velocidades = randn(num_particulas, 3); % Velocidades iniciales aleatorias
% aceleraciones = zeros(num_particulas, 3); % Inicializar aceleraciones a cero
% 
% % Matrices para almacenar las coordenadas de posición de cada partícula a lo largo del tiempo
% trayectorias_x = zeros(num_particulas, tiempo_simulacion / paso_tiempo);
% trayectorias_y = zeros(num_particulas, tiempo_simulacion / paso_tiempo);
% trayectorias_z = zeros(num_particulas, tiempo_simulacion / paso_tiempo);
% 
% % Bucle de simulación con el algoritmo de Verlet
% num_pasos = tiempo_simulacion / paso_tiempo;
% posiciones_ant = posiciones;
% 
% for paso = 1:num_pasos
%     % Calcular fuerzas entre partículas (utilizando el algoritmo de Verlet)
%     fuerzas = calcular_fuerza(posiciones);
%     % Calcular aceleraciones
%     aceleraciones = fuerzas / masa_particula;
%     % Actualizar posiciones utilizando el algoritmo de Verlet
%     nuevas_posiciones = 2 * posiciones - posiciones_ant + (aceleraciones * paso_tiempo^2);
%     
%     % Actualizar velocidades
%     velocidades = (nuevas_posiciones - posiciones_ant) / (2 * paso_tiempo);
%     
%     % Almacenar las coordenadas de posición de cada partícula en este paso de tiempo
%     trayectorias_x(:, paso) = posiciones(:, 1);
%     trayectorias_y(:, paso) = posiciones(:, 2);
%     trayectorias_z(:, paso) = posiciones(:, 3);
%     
%     % Actualizar posiciones y aceleraciones para el siguiente paso de tiempo
%     posiciones_ant = posiciones;
%     posiciones = nuevas_posiciones;
% end
% 
% % Visualización de las trayectorias de las partículas
% figure;
% hold on;
% for i = 1:num_particulas
%     scatter3(trayectorias_x(i,:), trayectorias_y(i,:), trayectorias_z(i,:),10, 'filled', 'MarkerFaceAlpha', 0.5);
% end
% xlabel('X');
% ylabel('Y');
% zlabel('Z');
% title('Trayectorias de Partículas en la Simulación de Dinámica Molecular con Rastro (3D)');
% grid on;                % Cuadricula tridimensional
% axis([0, espacio, 0, espacio, 0, espacio]); % Establece los límites del cubo
% hold off;
% 
% % Función para calcular las fuerzas entre partículas (puedes personalizarla según tus necesidades)
% function fuerzas = calcular_fuerza(posiciones)
%     % Aquí puedes implementar el cálculo de fuerzas entre partículas
%     % Utiliza el algoritmo de Verlet o el potencial de Lennard-Jones u otro potencial
%     % Por simplicidad, se asume que las fuerzas son nulas en este ejemplo
%     num_particulas = size(posiciones, 1);
%     dimension = size(posiciones, 2);
%     fuerzas = zeros(num_particulas, dimension);
% end

%% Codigo 5 ESTE SI SIRVE
clear all;                            % Limpiar 
clc;
%filename = '100_p.gif';          % Nombre del archivo
%DelayTime = 0.1;                      % T. en seg. que dura cada plot en el GIF
f = figure(1);

% Parámetros de la simulación
num_particulas = 100;
tiempo_simulacion = 50; % Tiempo de simulación en segundos
paso_tiempo = 0.01;      % Tamaño del paso de tiempo en segundos
masa_particula = 1.0;    % Masa de cada partícula
espacio = 10.0;           % Tamaño del espacio a utilizar

% Distribuir las partículas equidistantemente en el espacio
num_particulas_por_lado = round(num_particulas^(1/3)); % Calcula el número de partículas por lado (cúbico)
num_particulas = num_particulas_por_lado^3; % Asegura que tengas un cubo de partículas
espaciado_entre_particulas = espacio / num_particulas_por_lado;

% Inicialización de posiciones y velocidades aleatorias
posiciones = rand(num_particulas, 3) * espacio;   % Posiciones iniciales aleatorias en el espacio
velocidades = randn(num_particulas, 3); % Velocidades iniciales aleatorias
aceleraciones = zeros(num_particulas, 3); % Inicializar aceleraciones a cero

% Distribuir las partículas en el espacio
contador = 1;
for x = 1:num_particulas_por_lado
    for y = 1:num_particulas_por_lado
        for z = 1:num_particulas_por_lado
            % Calcula la posición de la partícula
            posiciones(contador, :) = [(x - 0.5) * espaciado_entre_particulas, ...
                                         (y - 0.5) * espaciado_entre_particulas, ...
                                         (z - 0.5) * espaciado_entre_particulas];
            contador = contador + 1;
        end
    end
end

% Matrices para almacenar las coordenadas de posición de cada partícula a lo largo del tiempo
trayectorias_x_acum = zeros(num_particulas, tiempo_simulacion / paso_tiempo);
trayectorias_y_acum = zeros(num_particulas, tiempo_simulacion / paso_tiempo);
trayectorias_z_acum = zeros(num_particulas, tiempo_simulacion / paso_tiempo);

% Parámetros del potencial de Lennard-Jones
epsilon = 1.0; % Parámetro epsilon del potencial de Lennard-Jones
sigma = 1.0;   % Parámetro sigma del potencial de Lennard-Jones

% Bucle de simulación con el algoritmo de Verlet
num_pasos = tiempo_simulacion / paso_tiempo;
posiciones_ant = posiciones;

for paso = 1:num_pasos
    % Calcular fuerzas entre partículas (utilizando el algoritmo de Verlet)
    fuerzas = calcular_fuerza(posiciones_ant, epsilon, sigma);
    
    % Calcular aceleraciones
    aceleraciones = fuerzas / masa_particula;
    
   % Actualizar velocidades utilizando el algoritmo de Verlet
    velocidades = velocidades + (aceleraciones * paso_tiempo);
    
    % Actualizar posiciones utilizando el algoritmo de Verlet
    posiciones = posiciones + (velocidades * paso_tiempo) + (0.5 * aceleraciones * paso_tiempo^2);
    
     % Verificar colisiones con las paredes
    for i = 1:num_particulas
        for dim = 1:3
            if posiciones(i, dim) < 0 || posiciones(i, dim) > espacio
                % Invertir la velocidad en la dimensión correspondiente
                velocidades(i, dim) = -velocidades(i, dim);
            end
        end
    end

    % Almacenar las coordenadas de posición de cada partícula en este paso de tiempo
    trayectorias_x_acum(:, paso) = posiciones(:, 1);
    trayectorias_y_acum(:, paso) = posiciones(:, 2);
    trayectorias_z_acum(:, paso) = posiciones(:, 3);
    
    % Actualizar posiciones y aceleraciones para el siguiente paso de tiempo
    posiciones_ant = posiciones;
    
    % Dibuja las partículas en la figura actualizada
    scatter3(posiciones(:, 1), posiciones(:, 2), posiciones(:, 3), 10, 'filled', 'MarkerFaceAlpha', 0.5);
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    title('Simulación de Dinámica Molecular con Trayectorias (3D)');
    grid on;
    axis([0, espacio, 0, espacio, 0, espacio]); % Establece los límites del cubo
    
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
%         imwrite(imind,cm,filename,'gif','DelayTime', DelayTime , 'LoopCount' , Inf  );
%     else
%         % Añadir al GIF cada nuevo plot
%         imwrite(imind,cm,filename,'gif','WriteMode','append','DelayTime', DelayTime );
%     end
end

% % Inicializa la figura para la animación
% figure(1);
% hold on; 
% % Actualiza el gráfico con las trayectorias acumuladas hasta este paso de tiempo
% for i = 1:num_particulas
%     scatter3(trayectorias_x_acum(i, 1:paso), trayectorias_y_acum(i, 1:paso), trayectorias_z_acum(i, 1:paso),10, 'filled', 'MarkerFaceAlpha', 0.5);
% end
% 
% xlabel('X');
% ylabel('Y');
% zlabel('Z');
% title('Simulación de Dinámica Molecular con Trayectorias (3D)');
% grid on;
% axis([0, espacio, 0, espacio, 0, espacio]); % Establece los límites del cubo
% hold off;

% Función para calcular las fuerzas entre partículas (utilizando el potencial de Lennard-Jones)
function fuerzas = calcular_fuerza(posiciones, epsilon, sigma)
    num_particulas = size(posiciones, 1);
    dimension = size(posiciones, 2);
    fuerzas = zeros(num_particulas, dimension);
    
    for i = 1:num_particulas
        for j = i+1:num_particulas
            r_ij = posiciones(i, :) - posiciones(j, :);
            r = norm(r_ij);
            
            % Calcular la fuerza
            fuerza_ij = 24 * epsilon * (2 * (sigma / r)^12 - (sigma / r)^6) * r_ij / r^2;
            
            % Aplicar la fuerza a ambas partículas (ley de acción y reacción)
            fuerzas(i, :) = fuerzas(i, :) + fuerza_ij;
            fuerzas(j, :) = fuerzas(j, :) - fuerza_ij;
        end
    end
end

