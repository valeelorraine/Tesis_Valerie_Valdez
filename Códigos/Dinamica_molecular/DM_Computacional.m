%% Valerie Valdez                                        Carné: 19659
% Universidad del Valle de Guatemala                    Sección: 20

%                    Implementación de fisica granular
%                           Dinámica molecular

clc;
clear all;                            % Limpiar 

%% Código 1
% num_particulas = 10;
% tiempo_simulacion = 100;  % Tiempo de simulación
% paso_tiempo = 0.01;       % Tamaño del paso de tiempo
% masa_particula = 1.0;     % Masa de cada partícula
% espacio = 10.0;           % Tamaño del espacio a utilizar
% 
% % Inicialización de posiciones y velocidades aleatorias
% posiciones = rand(num_particulas, 3) * espacio;   % Posiciones iniciales aleatorias en el espacio
% velocidades = randn(num_particulas, 3);           % Velocidades iniciales aleatorias
% fuerzas = zeros(num_particulas, 3);               % Inicializar fuerzas a cero
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
%     % Calcular fuerzas entre partículas
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
% grid on;                                    % Cuadricula tridimensional
% axis([0, espacio, 0, espacio, 0, espacio]); % Establecer los límites del cubo
% hold off;
% 
% % En esta simulación las fuerzas son nulas
% % Función para calcular las fuerzas entre partículas
% function fuerzas = calcular_fuerza(posiciones)
%     % FALTA IMPLEMENTAR el cálculo de fuerzas entre partículas
%     % Potencial de Lennard-Jones
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
%     % Calcular fuerzas entre partículas
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
%     scatter3(trayectorias_x(i,:), trayectorias_y(i,:), trayectorias_z(i,:),25, 'blue', 'filled', 'MarkerFaceAlpha', 0.5);
% end
% xlabel('X');
% ylabel('Y');
% zlabel('Z');
% title('Trayectorias de Partículas en la Simulación de Dinámica Molecular con Rastro (3D)');
% grid on;                % Cuadricula tridimensional
% axis([0, espacio, 0, espacio, 0, espacio]); % Establecer los límites del cubo
% hold off;
% 
% % Función para calcular las fuerzas entre partículas
% function fuerzas = calcular_fuerza(posiciones)
%     % FALTA IMPLEMENTAR el cálculo de fuerzas entre partículas
%     num_particulas = size(posiciones, 1);
%     dimension = size(posiciones, 2);
%     fuerzas = zeros(num_particulas, dimension);
% end

%% Código con simulación dinámica
% CREO QUE ESTE CODIGO LO USO
% 
% % %filename = '100_p.gif';          % Nombre del archivo
% % %DelayTime = 0.1;                 % T. en seg. que dura cada plot en el GIF
% f = figure(1);                      % Creación de la figura
% 
% % Parámetros de la simulación
% num_particulas = 100;
% tiempo_simulacion = 50; % Tiempo de simulación en segundos
% paso_tiempo = 0.01;      % Tamaño del paso de tiempo en segundos
% masa_particula = 1.0;    % Masa de cada partícula
% espacio = 10.0;           % Tamaño del espacio a utilizar
% 
% % Distribuir las partículas equidistantemente en el espacio
% num_particulas_por_lado = round(num_particulas^(1/3)); % Calcular el número de partículas por lado (cúbico)
% num_particulas = num_particulas_por_lado^3;            % Cubo de partículas
% espaciado_entre_particulas = espacio / num_particulas_por_lado;
% 
% % Inicialización de posiciones y velocidades aleatorias
% posiciones = rand(num_particulas, 3) * espacio;   % Posiciones iniciales aleatorias en el espacio
% velocidades = randn(num_particulas, 3); % Velocidades iniciales aleatorias
% aceleraciones = zeros(num_particulas, 3); % Inicializar aceleraciones a cero
% 
% % Distribuir las partículas en el espacio
% contador = 1;
% for x = 1:num_particulas_por_lado
%     for y = 1:num_particulas_por_lado
%         for z = 1:num_particulas_por_lado
%             % Calcular la posición de la partícula
%             posiciones(contador, :) = [(x - 0.5) * espaciado_entre_particulas, ...
%                                          (y - 0.5) * espaciado_entre_particulas, ...
%                                          (z - 0.5) * espaciado_entre_particulas];
%             contador = contador + 1;
%         end
%     end
% end
% 
% % Matrices para almacenar las coordenadas de posición de cada partícula a lo largo del tiempo
% trayectorias_x_acum = zeros(num_particulas, tiempo_simulacion / paso_tiempo);
% trayectorias_y_acum = zeros(num_particulas, tiempo_simulacion / paso_tiempo);
% trayectorias_z_acum = zeros(num_particulas, tiempo_simulacion / paso_tiempo);
% 
% % Parámetros del potencial de Lennard-Jones
% epsilon = 1.0; % Parámetro epsilon del potencial de Lennard-Jones (intensidad)
% sigma = 1.0;   % Parámetro sigma del potencial de Lennard-Jones (Alcance)
% 
% % Bucle de simulación con el algoritmo de Verlet
% num_pasos = tiempo_simulacion / paso_tiempo;
% posiciones_ant = posiciones;
% 
% for paso = 1:num_pasos
%     % Calcular fuerzas entre partículas
%     fuerzas = calcular_fuerza(posiciones_ant, epsilon, sigma);
%     
%     % Calcular aceleraciones
%     aceleraciones = fuerzas / masa_particula;
%     
%    % Actualizar velocidades utilizando el algoritmo de Verlet
%     velocidades = velocidades + (aceleraciones * paso_tiempo);
%     
%     % Actualizar posiciones utilizando el algoritmo de Verlet
%     posiciones = posiciones + (velocidades * paso_tiempo) + (0.5 * aceleraciones * paso_tiempo^2);
%     
%      % Verificar colisiones con las paredes
%     for i = 1:num_particulas
%         for dim = 1:3
%             if posiciones(i, dim) < 0 || posiciones(i, dim) > espacio
%                 % Invertir la velocidad en la dimensión correspondiente
%                 velocidades(i, dim) = -velocidades(i, dim);
%             end
%         end
%     end
% 
%     % Almacenar las coordenadas de posición de cada partícula en este paso de tiempo
%     trayectorias_x_acum(:, paso) = posiciones(:, 1);
%     trayectorias_y_acum(:, paso) = posiciones(:, 2);
%     trayectorias_z_acum(:, paso) = posiciones(:, 3);
%     
%     % Actualizar posiciones y aceleraciones para el siguiente paso de tiempo
%     posiciones_ant = posiciones;
%     
%     % Dibuja las partículas en la figura actualizada
%     scatter3(posiciones(:, 1), posiciones(:, 2), posiciones(:, 3), 10, 'filled', 'MarkerFaceAlpha', 0.5);
%     xlabel('X(m)');
%     ylabel('Y(m)');
%     zlabel('Z(m)');
%     title('Simulación de Dinámica Molecular en 3D');
%     grid on;
%     axis([0, espacio, 0, espacio, 0, espacio]); % Establecer los límites del cubo
%     
%     % Pausa para permitir que MATLAB actualice la figura
%     pause(0.01);
% 
% %     frame = getframe(f);
% %     % Convertir la trama en una imagen RGB (3 dimensiones) 
% %     im = frame2im(frame);
% %     
% %     % Transformar muestras de RGB a 1 dimensión con un mapa de color "cm" 
% %     [imind,cm] = rgb2ind(im , 256); 
% %     if paso == 1
% %         % Crear el archivo GIF
% %         imwrite(imind,cm,filename,'gif','DelayTime', DelayTime , 'LoopCount' , Inf  );
% %     else
% %         % Añadir al GIF cada nuevo plot
% %         imwrite(imind,cm,filename,'gif','WriteMode','append','DelayTime', DelayTime );
% %     end
% end
% 
% % % Inicializa la figura para la animación
% % figure(1);
% % hold on; 
% % % Actualizar el gráfico con las trayectorias acumuladas hasta este paso de tiempo
% % for i = 1:num_particulas
% %     scatter3(trayectorias_x_acum(i, 1:paso), trayectorias_y_acum(i, 1:paso), trayectorias_z_acum(i, 1:paso),10, 'filled', 'MarkerFaceAlpha', 0.5);
% % end
% % 
% % xlabel('X');
% % ylabel('Y');
% % zlabel('Z');
% % title('Simulación de Dinámica Molecular con Trayectorias (3D)');
% % grid on;
% % axis([0, espacio, 0, espacio, 0, espacio]); % Establece los límites del cubo
% % hold off;
% 
% % Función para calcular las fuerzas entre partículas (utilizando el potencial de Lennard-Jones)
% function fuerzas = calcular_fuerza(posiciones, epsilon, sigma)
%     num_particulas = size(posiciones, 1);
%     dimension = size(posiciones, 2);
%     fuerzas = zeros(num_particulas, dimension);
%     
%     for i = 1:num_particulas
%         for j = i+1:num_particulas
%             r_ij = posiciones(i, :) - posiciones(j, :);
%             r = norm(r_ij);       % Calcular la distancia entre particulas
%             
%             % Calcular la fuerza
%             fuerza_ij = 24 * epsilon * (2 * (sigma / r)^12 - (sigma / r)^6) * r_ij / r^2;
%             
%             % Aplicar la fuerza a ambas partículas (ley de acción y reacción)
%             fuerzas(i, :) = fuerzas(i, :) + fuerza_ij;
%             fuerzas(j, :) = fuerzas(j, :) - fuerza_ij;
%         end
%     end
% end
%% Codico con simulacion estática en 3D ayuda CHEPITO
% % Parámetros de la simulación
% num_particulas = 10;
% tiempo_simulacion = 30; % Tiempo de simulación en segundos
% paso_tiempo = 0.01;      % Tamaño del paso de tiempo en segundos
% masa_particula = 1.0;    % Masa de cada partícula
% espacio = 10.0;           % Tamaño del espacio a utilizar
% 
% % Distribuir las partículas equidistantemente en el espacio
% num_particulas_por_lado = round(num_particulas^(1/3));
% num_particulas = num_particulas_por_lado^3;
% espaciado_entre_particulas = espacio / num_particulas_por_lado;
% 
% % Inicialización de posiciones y velocidades aleatorias
% posiciones = rand(num_particulas, 3) * espacio;
% velocidades = randn(num_particulas, 3);
% aceleraciones = zeros(num_particulas, 3);
% 
% % Distribuir las partículas en el espacio
% contador = 1;
% for x = 1:num_particulas_por_lado
%     for y = 1:num_particulas_por_lado
%         for z = 1:num_particulas_por_lado
%             posiciones(contador, :) = [(x - 0.5) * espaciado_entre_particulas, ...
%                                          (y - 0.5) * espaciado_entre_particulas, ...
%                                          (z - 0.5) * espaciado_entre_particulas];
%             contador = contador + 1;
%         end
%     end
% end
% 
% posiciones_i = posiciones;
% % Matrices para almacenar las coordenadas de posición de cada partícula a lo largo del tiempo
% trayectorias_x_acum = zeros(num_particulas, tiempo_simulacion / paso_tiempo);
% trayectorias_y_acum = zeros(num_particulas, tiempo_simulacion / paso_tiempo);
% trayectorias_z_acum = zeros(num_particulas, tiempo_simulacion / paso_tiempo);
% 
% % Parámetros del potencial de Lennard-Jones
% epsilon = 1.0;
% sigma = 1.0;
% 
% % Bucle de simulación para calcular trayectorias
% num_pasos = tiempo_simulacion / paso_tiempo;
% posiciones_ant = posiciones;
% 
% for paso = 1:num_pasos
%     fuerzas = calcular_fuerza(posiciones_ant, epsilon, sigma);
%     aceleraciones = fuerzas / masa_particula;
%     velocidades = velocidades + (aceleraciones * paso_tiempo);
%     posiciones = posiciones + (velocidades * paso_tiempo) + (0.5 * aceleraciones * paso_tiempo^2);
%     posiciones_ant = posiciones;
%     
%     % Almacenar las coordenadas de posición de cada partícula en este paso de tiempo
%     trayectorias_x_acum(:, paso) = posiciones(:, 1);
%     trayectorias_y_acum(:, paso) = posiciones(:, 2);
%     trayectorias_z_acum(:, paso) = posiciones(:, 3);
% end
% 
% xlabel('X');
% ylabel('Y');
% zlabel('Z');
% title('Simulación de Trayectorias de Partículas (3D)');
% grid on;
% maxX = max(max(trayectorias_x_acum,[],2))+10;
% maxY = max(max(trayectorias_y_acum,[],2))+10;
% maxZ = max(max(trayectorias_z_acum,[],2))+10;
% axis([-maxX, maxX, -maxY, maxY, -maxZ, maxZ]);
% 
% 
% % Dibujar un marcador en la posición inicial de cada partícula
% %plot3(posiciones_i(:, 1), posiciones_i(:, 2), posiciones_i(:, 3),'d');
% 
% % Dibujar las partículas en su posición final
% %scatter3(posiciones(:, 1), posiciones(:, 2), posiciones(:, 3), 10, 'filled', 'MarkerFaceAlpha', 0.5);
% 
% % Trazar las trayectorias de todas las partículas
% hold on;
% for i = 1:num_particulas
%     plot3(posiciones_i(i, 1), posiciones_i(i, 2), posiciones_i(i, 3),'d');
%     plot3(trayectorias_x_acum(i, 2:end-1), trayectorias_y_acum(i, 2:end-1), trayectorias_z_acum(i, 2:end-1));
%     plot3(trayectorias_x_acum(i, end), trayectorias_y_acum(i, end), trayectorias_z_acum(i, end),'d');
%     %plot3(posiciones(i, 1), posiciones(i, 2), posiciones(i, 3), 10, 'filled', 'MarkerFaceAlpha', 0.5);
% end
% hold off;
% 
% % Función para calcular las fuerzas entre partículas (utilizando el potencial de Lennard-Jones)
% function fuerzas = calcular_fuerza(posiciones, epsilon, sigma)
%     num_particulas = size(posiciones, 1);
%     dimension = size(posiciones, 2);
%     fuerzas = zeros(num_particulas, dimension);
%     
%     for i = 1:num_particulas
%         for j = i+1:num_particulas
%             r_ij = posiciones(i, :) - posiciones(j, :);
%             r = norm(r_ij);
%             
%             % Calcular la fuerza
%             fuerza_ij = 24 * epsilon * (2 * (sigma / r)^12 - (sigma / r)^6) * r_ij / r^2;
%             
%             % Aplicar la fuerza a ambas partículas (ley de acción y reacción)
%             fuerzas(i, :) = fuerzas(i, :) + fuerza_ij;
%             fuerzas(j, :) = fuerzas(j, :) - fuerza_ij;
%         end
%     end
% end

%% Código con simulación estática en 3D con colisiones elasticas entre particulas
%  PRIMER CODIGO USADO EN LA TESIS
% %Parámetros de la simulación
% num_particulas = 8;
% tiempo_simulacion = 5; % Tiempo de simulación en segundos
% paso_tiempo = 0.01;      % Tamaño del paso de tiempo en segundos
% masa_particula = 1.0;    % Masa de cada partícula
% espacio = 10.0;           % Tamaño del espacio a utilizar
% 
% % Distribuir las partículas equidistantemente en el espacio
% num_particulas_por_lado = round(num_particulas^(1/3));
% %num_particulas = num_particulas_por_lado^3;
% espaciado_entre_particulas = espacio / num_particulas_por_lado;
% 
% % Inicialización de posiciones y velocidades aleatorias
% posiciones = rand(num_particulas, 3) * espacio;
% velocidades = randn(num_particulas, 3);
% aceleraciones = zeros(num_particulas, 3);
% 
% % Distribuir las partículas en el espacio
% contador = 1;
% for x = 1:num_particulas_por_lado
%     for y = 1:num_particulas_por_lado
%         for z = 1:num_particulas_por_lado
%             posiciones(contador, :) = [(x - 0.5) * espaciado_entre_particulas, ...
%                                          (y - 0.5) * espaciado_entre_particulas, ...
%                                          (z - 0.5) * espaciado_entre_particulas];
%             contador = contador + 1;
%         end
%     end
% end
% 
% posiciones_i = posiciones;
% % Matrices para almacenar las coordenadas de posición de cada partícula a lo largo del tiempo
% trayectorias_x_acum = zeros(num_particulas, tiempo_simulacion / paso_tiempo);
% trayectorias_y_acum = zeros(num_particulas, tiempo_simulacion / paso_tiempo);
% trayectorias_z_acum = zeros(num_particulas, tiempo_simulacion / paso_tiempo);
% 
% % Parámetros del potencial de Lennard-Jones
% epsilon = 1.0;
% sigma = 1.0;
% 
% % Diámetro de colisión (se asume que todas las partículas tienen el mismo diámetro)
% diametro_colision = 0.1; 
% 
% % Bucle de simulación para calcular trayectorias
% num_pasos = tiempo_simulacion / paso_tiempo;
% posiciones_ant = posiciones;
% 
% for paso = 1:num_pasos
%     fuerzas = calcular_fuerza(posiciones_ant, epsilon, sigma);
%     aceleraciones = fuerzas / masa_particula;
%     velocidades = velocidades + (aceleraciones * paso_tiempo);
%     
%     % Manejar colisiones con los límites del espacio (colisiones elásticas)
%     for i = 1:num_particulas
%         for dim = 1:3
%             if posiciones(i, dim) < 0 || posiciones(i, dim) > espacio
%                 % La partícula ha chocado con un límite en la dirección dim
%                 % Reflejar la velocidad en esa dirección (colisión elástica)
%                 velocidades(i, dim) = -velocidades(i, dim);
%             end
%         end
%     end
%     
%     % Manejar colisiones entre partículas (colisiones elásticas)
%     for i = 1:num_particulas
%         for j = i+1:num_particulas
%             distancia = norm(posiciones(i, :) - posiciones(j, :));
%             
%             if distancia < diametro_colision
%                 % Colisión detectada entre las partículas i y j
%                 % Calcular las velocidades relativas
%                 velocidad_relativa = velocidades(i, :) - velocidades(j, :);
%                 
%                 % Calcular la dirección de la colisión
%                 direccion_colision = (posiciones(i, :) - posiciones(j, :)) / distancia;
%                 
%                 % Calcular las nuevas velocidades después de la colisión elástica
%                 velocidad_i_nueva = velocidades(i, :) - (2 * masa_particula / (masa_particula + masa_particula)) * dot(velocidad_relativa, direccion_colision) * direccion_colision;
%                 velocidad_j_nueva = velocidades(j, :) + (2 * masa_particula / (masa_particula + masa_particula)) * dot(velocidad_relativa, direccion_colision) * direccion_colision;
%                 
%                 % Actualizar las velocidades de las partículas
%                 velocidades(i, :) = velocidad_i_nueva;
%                 velocidades(j, :) = velocidad_j_nueva;
%             end
%         end
%     end
%     
%     posiciones = posiciones + (velocidades * paso_tiempo) + (0.5 * aceleraciones * paso_tiempo^2);
%     posiciones_ant = posiciones;
%     
%     % Almacenar las coordenadas de posición de cada partícula en este paso de tiempo
%     trayectorias_x_acum(:, paso) = posiciones(:, 1);
%     trayectorias_y_acum(:, paso) = posiciones(:, 2);
%     trayectorias_z_acum(:, paso) = posiciones(:, 3);
% end
% 
% xlabel('X(m)');
% ylabel('Y(m)');
% zlabel('Z(m)');
% title('Simulación de Trayectorias de Partículas (3D)');
% grid on;
% axis([0, espacio, 0, espacio]);
% 
% % Trazar las trayectorias de todas las partículas
% hold on;
% for i = 1:num_particulas
%     plot3(posiciones_i(i, 1), posiciones_i(i, 2), posiciones_i(i, 3),'o', MarkerEdgeColor = 'black', MarkerSize = 4); % Posicion inicial
%     plot3(trayectorias_x_acum(i, 2:end-1), trayectorias_y_acum(i, 2:end-1), trayectorias_z_acum(i, 2:end-1));
%     plot3(trayectorias_x_acum(i, end), trayectorias_y_acum(i, end), trayectorias_z_acum(i, end),'pentagram', MarkerEdgeColor = 'black', MarkerSize = 6);
%     %plot3(posiciones(i, 1), posiciones(i, 2), posiciones(i, 3), 10, 'filled', 'MarkerFaceAlpha', 0.5);
% end
% hold off;
% 
% % Función para calcular las fuerzas entre partículas (utilizando el potencial de Lennard-Jones)
% function fuerzas = calcular_fuerza(posiciones, epsilon, sigma)
%     num_particulas = size(posiciones, 1);
%     dimension = size(posiciones, 2);
%     fuerzas = zeros(num_particulas, dimension);
%     
%     for i = 1:num_particulas
%         for j = i+1:num_particulas
%             r_ij = posiciones(i, :) - posiciones(j, :);
%             r = norm(r_ij);
%             
%             % Calcular la fuerza
%             fuerza_ij = 24 * epsilon * (2 * (sigma / r)^12 - (sigma / r)^6) * r_ij / r^2;
%             
%             % Aplicar la fuerza a ambas partículas (ley de acción y reacción)
%             fuerzas(i, :) = fuerzas(i, :) + fuerza_ij;
%             fuerzas(j, :) = fuerzas(j, :) - fuerza_ij;
%         end
%     end
% end

%% Código con simulación estática en 2D con colisiones elásticas entre partículas
% % Parámetros de la simulación
% num_particulas = 10;
% tiempo_simulacion = 8; % Tiempo de simulación en segundos
% paso_tiempo = 0.01;      % Tamaño del paso de tiempo en segundos
% masa_particula = 1.0;    % Masa de cada partícula
% espacio = 10.0;           % Tamaño del espacio a utilizar
% 
% % Distribuir las partículas equidistantemente en el espacio 2D
% num_particulas_por_lado = round(sqrt(num_particulas));
% num_particulas = num_particulas_por_lado^2;
% espaciado_entre_particulas = espacio / num_particulas_por_lado;
% 
% % Inicialización de posiciones y velocidades aleatorias en 2D
% posiciones = rand(num_particulas, 2) * espacio;
% velocidades = randn(num_particulas, 2);
% aceleraciones = zeros(num_particulas, 2);
% 
% % Distribuir las partículas en el espacio 2D
% contador = 1;
% for x = 1:num_particulas_por_lado
%     for y = 1:num_particulas_por_lado
%         posiciones(contador, :) = [(x - 0.5) * espaciado_entre_particulas, ...
%                                     (y - 0.5) * espaciado_entre_particulas];
%         contador = contador + 1;
%     end
% end
% 
% posiciones_i = posiciones;
% % Matrices para almacenar las coordenadas de posición de cada partícula a lo largo del tiempo
% trayectorias_x_acum = zeros(num_particulas, tiempo_simulacion / paso_tiempo);
% trayectorias_y_acum = zeros(num_particulas, tiempo_simulacion / paso_tiempo);
% 
% % Parámetros del potencial de Lennard-Jones
% epsilon = 1.0;
% sigma = 1.0;
% 
% % Diámetro de colisión
% diametro_colision = 0.1; 
% 
% % Bucle de simulación para calcular trayectorias en 2D
% num_pasos = tiempo_simulacion / paso_tiempo;
% posiciones_ant = posiciones;
% 
% for paso = 1:num_pasos
%     fuerzas = calcular_fuerza(posiciones_ant, epsilon, sigma);
%     aceleraciones = fuerzas / masa_particula;
%     velocidades = velocidades + (aceleraciones * paso_tiempo);
%     
%     % Manejar colisiones con los límites del espacio 2D (colisiones elásticas)
%     for i = 1:num_particulas
%         for dim = 1:2
%             if posiciones(i, dim) < 0 || posiciones(i, dim) > espacio
%                 % La partícula ha chocado con un límite en la dirección dim
%                 % Reflejar la velocidad en esa dirección (colisión elástica)
%                 velocidades(i, dim) = -velocidades(i, dim);
%             end
%         end
%     end
%     
%     % Manejar colisiones entre partículas en 2D (colisiones elásticas)
%     for i = 1:num_particulas
%         for j = i+1:num_particulas
%             distancia = norm(posiciones(i, :) - posiciones(j, :));
%             
%             if distancia < diametro_colision
%                 % Colisión detectada entre las partículas i y j en 2D
%                 % Calcular las velocidades relativas
%                 velocidad_relativa = velocidades(i, :) - velocidades(j, :);
%                 
%                 % Calcular la dirección de la colisión
%                 direccion_colision = (posiciones(i, :) - posiciones(j, :)) / distancia;
%                 
%                 % Calcular las nuevas velocidades después de la colisión elástica en 2D
%                 velocidad_i_nueva = velocidades(i, :) - (2 * masa_particula / (masa_particula + masa_particula)) * dot(velocidad_relativa, direccion_colision) * direccion_colision;
%                 velocidad_j_nueva = velocidades(j, :) + (2 * masa_particula / (masa_particula + masa_particula)) * dot(velocidad_relativa, direccion_colision) * direccion_colision;
%                 
%                 % Actualizar las velocidades de las partículas en 2D
%                 velocidades(i, :) = velocidad_i_nueva;
%                 velocidades(j, :) = velocidad_j_nueva;
%             end
%         end
%     end
%     
%     posiciones = posiciones + (velocidades * paso_tiempo) + (0.5 * aceleraciones * paso_tiempo^2);
%     posiciones_ant = posiciones;
%     
%     % Almacenar las coordenadas de posición de cada partícula en 2D en este paso de tiempo
%     trayectorias_x_acum(:, paso) = posiciones(:, 1);
%     trayectorias_y_acum(:, paso) = posiciones(:, 2);
% end
% 
% xlabel('X');
% ylabel('Y');
% title('Simulación de Trayectorias de Partículas (2D)');
% grid on;
% axis([0, espacio, 0, espacio]);
% 
% % Trazar las trayectorias de todas las partículas en 2D
% hold on;
% for i = 1:num_particulas
%     plot(posiciones_i(i, 1), posiciones_i(i, 2),'o', 'MarkerEdgeColor', 'red', 'MarkerSize', 4); % Posicion inicial
%     plot(trayectorias_x_acum(i, 2:end-1), trayectorias_y_acum(i, 2:end-1));
%     plot(posiciones(i, 1), posiciones(i, 2),'pentagram', 'MarkerEdgeColor', 'yellow');
% end
% hold off;
% 
% % Función para calcular las fuerzas entre partículas (utilizando el potencial de Lennard-Jones)
% function fuerzas = calcular_fuerza(posiciones, epsilon, sigma)
%     num_particulas = size(posiciones, 1);
%     dimension = size(posiciones, 2);
%     fuerzas = zeros(num_particulas, dimension);
%     
%     for i = 1:num_particulas
%         for j = i+1:num_particulas
%             r_ij = posiciones(i, :) - posiciones(j, :);
%             r = norm(r_ij);
%             
%             % Calcular la fuerza
%             fuerza_ij = 24 * epsilon * (2 * (sigma / r)^12 - (sigma / r)^6) * r_ij / r^2;
%             
%             % Aplicar la fuerza a ambas partículas (ley de acción y reacción)
%             fuerzas(i, :) = fuerzas(i, :) + fuerza_ij;
%             fuerzas(j, :) = fuerzas(j, :) - fuerza_ij;
%         end
%     end
% end

%% Código con simulación estática en 3D con colisiones elasticas con la pared
% Parámetros de la simulación
% num_particulas = 10;
% tiempo_simulacion = 8;   % Tiempo de simulación en segundos 
% paso_tiempo = 0.01;      % Tamaño del paso de tiempo en segundos
% masa_particula = 1.0;    % Masa de cada partícula
% espacio = 10.0;          % Tamaño del espacio a utilizar
% 
% % Distribuir las partículas equidistantemente en el espacio
% num_particulas_por_lado = round(num_particulas^(1/3));
% num_particulas = num_particulas_por_lado^3;
% espaciado_entre_particulas = espacio / num_particulas_por_lado;
% 
% % Inicialización de posiciones y velocidades aleatorias
% posiciones = rand(num_particulas, 3) * espacio;
% velocidades = randn(num_particulas, 3);
% aceleraciones = zeros(num_particulas, 3);
% 
% % Distribuir las partículas en el espacio
% contador = 1;
% for x = 1:num_particulas_por_lado
%     for y = 1:num_particulas_por_lado
%         for z = 1:num_particulas_por_lado
%             posiciones(contador, :) = [(x - 0.5) * espaciado_entre_particulas, ...
%                                          (y - 0.5) * espaciado_entre_particulas, ...
%                                          (z - 0.5) * espaciado_entre_particulas];
%             contador = contador + 1;
%         end
%     end
% end
% 
% posiciones_i = posiciones;
% % Matrices para almacenar las coordenadas de posición de cada partícula a lo largo del tiempo
% trayectorias_x_acum = zeros(num_particulas, tiempo_simulacion / paso_tiempo);
% trayectorias_y_acum = zeros(num_particulas, tiempo_simulacion / paso_tiempo);
% trayectorias_z_acum = zeros(num_particulas, tiempo_simulacion / paso_tiempo);
% 
% % Parámetros del potencial de Lennard-Jones
% epsilon = 1.0;
% sigma = 1.0;
% 
% % Diámetro de colisión 
% diametro_colision = 0.1;
% 
% % Bucle de simulación para calcular trayectorias
% num_pasos = tiempo_simulacion / paso_tiempo;
% posiciones_ant = posiciones;
% 
% for paso = 1:num_pasos
%     fuerzas = calcular_fuerza(posiciones_ant, epsilon, sigma);
%     aceleraciones = fuerzas / masa_particula;
%     velocidades = velocidades + (aceleraciones * paso_tiempo);
%     
%     % Manejar colisiones con los límites del espacio (colisiones elásticas)
%     for i = 1:num_particulas
%         for dim = 1:3
%             if posiciones(i, dim) < 0 || posiciones(i, dim) > espacio
%                 % La partícula ha chocado con un límite en la dirección dim
%                 % Reflejar la velocidad en esa dirección (colisión elástica)
%                 velocidades(i, dim) = -velocidades(i, dim);
%             end
%         end
%     end
%     
%     posiciones = posiciones + (velocidades * paso_tiempo) + (0.5 * aceleraciones * paso_tiempo^2);
%     posiciones_ant = posiciones;
%     
%     % Almacenar las coordenadas de posición de cada partícula en este paso de tiempo
%     trayectorias_x_acum(:, paso) = posiciones(:, 1);
%     trayectorias_y_acum(:, paso) = posiciones(:, 2);
%     trayectorias_z_acum(:, paso) = posiciones(:, 3);
% end
% 
% xlabel('X');
% ylabel('Y');
% zlabel('Z');
% title('Simulación de Trayectorias de Partículas (3D)');
% grid on;
% axis([0, espacio, 0, espacio]);
% 
% % Trazar las trayectorias de todas las partículas
% hold on;
% for i = 1:num_particulas
%     plot3(posiciones_i(i, 1), posiciones_i(i, 2), posiciones_i(i, 3),'o', MarkerEdgeColor = 'red', MarkerSize = 4); % Posicion inicial
%     plot3(trayectorias_x_acum(i, 2:end-1), trayectorias_y_acum(i, 2:end-1), trayectorias_z_acum(i, 2:end-1));
%     plot3(trayectorias_x_acum(i, end), trayectorias_y_acum(i, end), trayectorias_z_acum(i, end),'pentagram', MarkerEdgeColor = 'yellow');
%     %plot3(posiciones(i, 1), posiciones(i, 2), posiciones(i, 3), 10, 'filled', 'MarkerFaceAlpha', 0.5);
% end
% hold off;
% 
% % Función para calcular las fuerzas entre partículas (utilizando el potencial de Lennard-Jones)
% function fuerzas = calcular_fuerza(posiciones, epsilon, sigma)
%     num_particulas = size(posiciones, 1);
%     dimension = size(posiciones, 2);
%     fuerzas = zeros(num_particulas, dimension);
%     
%     for i = 1:num_particulas
%         for j = i+1:num_particulas
%             r_ij = posiciones(i, :) - posiciones(j, :);
%             r = norm(r_ij);
%             
%             % Calcular la fuerza
%             fuerza_ij = 24 * epsilon * (2 * (sigma / r)^12 - (sigma / r)^6) * r_ij / r^2;
%             
%             % Aplicar la fuerza a ambas partículas (ley de acción y reacción)
%             fuerzas(i, :) = fuerzas(i, :) + fuerza_ij;
%             fuerzas(j, :) = fuerzas(j, :) - fuerza_ij;
%         end
%     end
% end


 %% Codigo para 2D DINAMICA
%  %ESTE CODIGO TAMBIEN LO USE
clear all;
clc;

% Parámetros de la simulación
num_particulas = 1;
tiempo_simulacion = 30;  % Tiempo de simulación en segundos (30 segundos en este ejemplo)
paso_tiempo = 0.01;      % Tamaño del paso de tiempo en segundos
masa_particula = 1.0;    % Masa de cada partícula
espacio = 4.0;          % Tamaño del espacio a utilizar

% % Distribuir las partículas equidistantemente en el espacio
% num_particulas_por_lado = round(num_particulas^(1/2)); % Raíz cuadrada del número de partículas por lado
% num_particulas = num_particulas_por_lado^2; % cuadrado de partículas
% espaciado_entre_particulas = espacio / num_particulas_por_lado;

% Inicialización de posiciones y velocidades aleatorias
posiciones = rand(num_particulas, 2) * espacio;   % Posiciones iniciales aleatorias en el espacio 2D
velocidades = randn(num_particulas, 2); % Velocidades iniciales aleatorias en 2D
aceleraciones = zeros(num_particulas, 2); % Inicializar aceleraciones a cero en 2D

% Parámetros del potencial de Lennard-Jones
epsilon = 1.0;
sigma = 1.0;

% Bucle de simulación para calcular trayectorias
num_pasos = tiempo_simulacion / paso_tiempo;
posiciones_ant = posiciones;

for paso = 1:num_pasos
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
            if posiciones(i, dim) < 0 || posiciones(i, dim) > espacio
                % Invertir la velocidad en la dimensión correspondiente
                velocidades(i, dim) = -velocidades(i, dim);
            end
        end
    end
    
    % Almacenar las coordenadas de posición de cada partícula en este paso de tiempo
    %trayectorias_x_acum(:, paso) = posiciones(:, 1);
   % trayectorias_y_acum(:, paso) = posiciones(:, 2);
    
    % Actualizar posiciones y aceleraciones para el siguiente paso de tiempo
    posiciones_ant = posiciones;
    
    % Dibujar las partículas en la figura actualizada
    scatter(posiciones(:, 1), posiciones(:, 2), 10, 'filled', 'MarkerFaceAlpha', 0.5);
    xlabel('X');
    ylabel('Y');    
    title('Simulación de Trayectorias de Partículas (2D)');
    grid on;
    axis([0, espacio, 0, espacio]);
    
    % Pausa para permitir que MATLAB actualice la figura
    pause(0.01);
end

% Función para calcular las fuerzas entre partículas (utilizando el potencial de Lennard-Jones)
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


%% Codigo para simulación 2D estática
% clear all;
% clc;
% 
% % Parámetros de la simulación
% num_particulas = 8;
% tiempo_simulacion = 5;  % Tiempo de simulación en segundos
% paso_tiempo = 0.01;      % Tamaño del paso de tiempo en segundos
% masa_particula = 1.0;    % Masa de cada partícula
% espacio = 10.0;          % Tamaño del espacio a utilizar
% 
% % % Distribuir las partículas equidistantemente en el espacio
% % num_particulas_por_lado = round(num_particulas^(1/2)); % Raíz cuadrada del número de partículas por lado
% % num_particulas = num_particulas_por_lado^2; % cuadrado de partículas
% % espaciado_entre_particulas = espacio / num_particulas_por_lado;
% 
% % Inicialización de posiciones y velocidades aleatorias
% posiciones = rand(num_particulas, 2) * espacio;   % Posiciones iniciales aleatorias en el espacio 2D
% velocidades = randn(num_particulas, 2); % Velocidades iniciales aleatorias en 2D
% aceleraciones = zeros(num_particulas, 2); % Inicializar aceleraciones a cero en 2D
% posiciones_i = posiciones;
% 
% % Parámetros del potencial de Lennard-Jones
% epsilon = 1.0;
% sigma = 1.0;
% 
% % Bucle de simulación para calcular trayectorias
% num_pasos = tiempo_simulacion / paso_tiempo;
% posiciones_ant = posiciones;
% 
% for paso = 1:num_pasos
%     % Calcular fuerzas entre partículas (utilizando el potencial de Lennard-Jones)
%     fuerzas = calcular_fuerza(posiciones_ant, epsilon, sigma);
%     
%     % Calcular aceleraciones
%     aceleraciones = fuerzas / masa_particula;
%     
%     % Actualizar velocidades utilizando el algoritmo de Verlet
%     velocidades = velocidades + (aceleraciones * paso_tiempo);
%     
%     % Actualizar posiciones utilizando el algoritmo de Verlet
%     posiciones = posiciones + (velocidades * paso_tiempo) + (0.5 * aceleraciones * paso_tiempo^2);
%     
%     % Verificar colisiones con las paredes y aplicar rebote
%     for i = 1:num_particulas
%         for dim = 1:2
%             if posiciones(i, dim) < 0 || posiciones(i, dim) > espacio
%                 % Invertir la velocidad en la dimensión correspondiente
%                 velocidades(i, dim) = -velocidades(i, dim);
%             end
%         end
%     end
%     
%     % Almacenar las coordenadas de posición de cada partícula en este paso de tiempo
%     trayectorias_x_acum(:, paso) = posiciones(:, 1);
%     trayectorias_y_acum(:, paso) = posiciones(:, 2);
%     
%     % Actualizar posiciones y aceleraciones para el siguiente paso de tiempo
%     posiciones_ant = posiciones;
%    
%     % Pausa para permitir que MATLAB actualice la figura
%     pause(0.01);
% end
% 
% xlabel('X(m)');
% ylabel('Y(m)');
% title('Simulación de Trayectorias de Partículas (2D)');
% grid on;
% axis([0, espacio, 0, espacio]);
% 
% % Trazar las trayectorias de todas las partículas en 2D
% hold on;
% for i = 1:num_particulas
%     plot(posiciones_i(i, 1), posiciones_i(i, 2),'o', 'MarkerEdgeColor', 'black', MarkerSize = 4); % Posicion inicial
%     plot(trayectorias_x_acum(i, 2:end-1), trayectorias_y_acum(i, 2:end-1));
%     plot(posiciones(i, 1), posiciones(i, 2),'pentagram', 'MarkerEdgeColor', 'black', MarkerSize = 4);
% end
% hold off;
% 
% 
% % Función para calcular las fuerzas entre partículas (utilizando el potencial de Lennard-Jones)
% function fuerzas = calcular_fuerza(posiciones, epsilon, sigma)
%     num_particulas = size(posiciones, 1);
%     dimension = size(posiciones, 2);
%     fuerzas = zeros(num_particulas, dimension);
%     
%     for i = 1:num_particulas
%         for j = 1:num_particulas
%             if i ~= j 
% 
%                 r_ij = posiciones(i, :) - posiciones(j, :);
%                 r = norm(r_ij);
%                 
%                 % Calcular la fuerza
%                 fuerza_ij = 24 * epsilon * (2 * (sigma / r)^12 - (sigma / r)^6) * r_ij / r^2;
%                 
%                 % Aplicar la fuerza a la partícula i
%                 fuerzas(i, :) = fuerzas(i, :) + fuerza_ij;
%             end
%         end
%     end
% end
% 
% 
% % incluir vector de fuerza en cad ainstante
