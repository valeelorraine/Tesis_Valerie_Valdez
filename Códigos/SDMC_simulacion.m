% Parámetros de la simulación
num_particulas = 10;
num_pasos = 1000;
sistema = 100.0;
paso_de_tiempo = 0.1;

% Inicializar las posiciones de las partículas
posiciones = rand(1, num_particulas) * sistema;

% Inicializar una matriz para almacenar las posiciones en cada paso
posiciones_historia = zeros(num_particulas, num_pasos);

% Función para mover una partícula
mover_particula = @(posicion) posicion + (2 * rand - 1) * paso_de_tiempo;

% Simular el movimiento de las partículas en el tiempo
for paso = 1:num_pasos
    % Mover cada partícula
    for i = 1:num_particulas
        posiciones(i) = mover_particula(posiciones(i));
        
        % Aplicar condiciones de contorno periódicas
        posiciones(i) = mod(posiciones(i), sistema);
    end
    
    % Registrar las posiciones en la historia
    posiciones_historia(:, paso) = posiciones;
end

% Crear un gráfico para visualizar las trayectorias
figure;
hold on;
for i = 1:num_particulas
    plot(1:num_pasos, posiciones_historia(i, :), '-');
end
hold off;

xlabel('Paso de tiempo');
ylabel('Posición');
title('Trayectorias de las partículas');
legend('Partícula 1', 'Partícula 2', 'Partícula 3');  % Agrega las leyendas según la cantidad de partículas

% Puedes personalizar el gráfico según tus preferencias
