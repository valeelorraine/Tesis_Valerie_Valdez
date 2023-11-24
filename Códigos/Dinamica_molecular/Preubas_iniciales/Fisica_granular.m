%% Valerie Valdez                                        Carné: 19659
% Universidad del Valle de Guatemala                    Sección: 20

%                    Implementación de fisica granular
%                          Dinámica molecular

%% Creación de un sistema dinámico (tiro parabólico)
% Partícula en 2D con tiempo discreto

clear all;

% Propiedades del archivo .GIF que se va a generar
% Nombre
filename = 'DMolecular.gif';
% Tiempo en segundos que dura cada plot en el GIF
DelayTime = 0.1; 
f = figure(1)                         % Crear una figura

% Declaración de variables a utilizar
X = [];                               % Posición
v = [];                               % Velocidad
F = [];                               % Fuerza

% Parámetros a utilizar (caída libre)
m = 1;                                % Masa
g = [0;-9.81];                        % Gravedad (vector vertical hacia abajo)
h = 0.01;                             % Paso de integración (tamaño de intervalo en s.)

% Condiciones iniciales
X = [0,0];                            % Posición en el origen
v = [0.1;4];                          % Valor inicial de la velocidad

% Segunda ley de newton
F = m*g;                              % Suma de todas las F de partículas (función de v y x)

for step= 1:15                     % Iteración del programa
    %plot(X(1),X(2),'ob');
    scatter(X(1),X(2), 'green','filled');
    hold on
    title(['paso :' num2str(step)]);  % Titulo según el step
    xlabel('X(m)');
    ylabel('Y(m)');
    axis([-0.2 0.2 -1 1]);
    drawnow
   %pause(0.1)                        % Segundos
    
    %Variables temporales
    xa = X;
    va = v;
 
    % Paso de integración con método de euler explícito
    % Calcular v y x en el punto medio predicho
    vpm = va + (h/2)*(F/m);          % Velocidad calculada
    xpm = xa + (h/2)*va;             % Posición calculada
    v = va + h*(F/m);                % Velocidad nueva
    X = xa+h*vpm;                    % Posición nueva
    
    frame = getframe(f);
    % Convertir la trama en una imagen RGB (3 dimensiones) 
    im = frame2im(frame);
    
     % Transformar muestras de RGB a 1 dimensión con un mapa de color "cm" 
    [imind,cm] = rgb2ind(im , 256); 
    %if step == 1;
        % Crear el archivo GIF
        %imwrite(imind,cm,filename,'gif','DelayTime', DelayTime , 'LoopCount' , Inf  );
   % else
          % Añadir al GIF cada nuevo plot
      %  imwrite(imind,cm,filename,'gif','WriteMode','append','DelayTime', DelayTime );
    %end
end
