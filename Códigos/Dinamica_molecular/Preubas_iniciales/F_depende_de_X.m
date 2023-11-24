%% Valerie Valdez                                        Carné: 19659
% Universidad del Valle de Guatemala                    Sección: 20

%                    Implementación de fisica granular
%               (Fuerza depende de posición de la partícula)
%                              Caída libre

%% Creación de un sistema dinámico (Fuerza depende de posición de la partícula)
clear all;                            % Limpiar 
clc;

%% Crear el GIF
%filename = 'FdependeXY.gif';          % Nombre del archivo
%DelayTime = 0.1;                      % T. en seg. que dura cada plot en el GIF

% Partícula en 2D con tiempo discreto
f = figure(1)                         % Crear una figura

% Declaración de variables a utilizar
x = [];                               % Posición
v = [];                               % Velocidad
F = [];                               % Fuerza
cola = [];                            % Estela que va a dejar

% Parámetros a utilizar
% Ley de Gravitación de Newton
% F = G*m1*m2/d^2
% M > m para simular una únicapartícula
m = 1;                                % Masa dinámica
m2 = 2;                              % Masa dinámica 2
M = 1e20;                             % Masa estática (centro de la imagen)
G = 6.67e-11;                         % Constante gravitación: N*m^2/kg^2
% Gravedad (vector vertical hacia abajo)
h = 0.01;                             % Paso de integración (tamaño de intervalo en s.)

%% Condiciones iniciales para una partícula
%x = [-1e3;0];                           % Posición en el origen
%v = [0;-2e3];                           % Valor inicial de la velocidad (tangente a la recta)

% %% Condiciones iniciales para dos partículas
% x = [-1e3,-1e3;0,0];                    % Posición en el origen
% v = [0,0;-2e3,-2e3];       

% Condiciones iniciales para dos partículas con dif. condiciones iniciales
% x=[-1e3,-1e2,-0.5e3;0,0,0];           % Posición en el origen
% v=[0,0,0;-2e3,-3e2,-1e2];

x(:, 1) = [-1000; 0];  % Posición inicial de la partícula roja
x(:, 2) = [-100; 0];   % Posición inicial de la partícula azul
x(:, 3) = [-0.5e3; 0];   % Posición inicial de la partícula anaranjada

v(:, 1) = [0; -2000];  % Velocidad inicial de la partícula roja
v(:, 2) = [0; -300];  % Velocidad inicial de la partícula azul
v(:, 3) = [0; -100];  % Velocidad inicial de la partícula anaranjada

%% Ciclo para el funcionamiento
%variable externa
for step=1:150                             % Iteración del programa F = -G*(m*M/norm(x)^2)*(x/norm(x));
    scatter(x(1,:),x(2,:), 'green','filled');
    
   % set(gca,'Color','black');
    hold on
    scatter(0,0,100,'yellow','filled');
    title(['paso :' num2str(step)]);       % Titulo según el step
    xlabel('X(m)');
    ylabel('Y(m)');
    axis([-2e3 2e3 -2e3 2e3]);
    drawnow
    %pause(0.01);

    if (size(cola,2)>0)
        scatter(cola(1,:),cola(2,:));
    end  

    % Variables temporales
    xa = x;
    va = v;
    
    % Paso integración
    F = -G*(m*M/norm(xa)^2)*(xa/norm(xa));% Actualización de la fuerza
    vpm = va + (h/2)*(F/m);               % Velocidad del punto medio
    xpm = xa + (h/2)*va;

    F=-G*(m*M/norm(xpm)^2)*(xpm/norm(xpm));
    v = va + h*(F/m);
    x = xa + h*vpm;
    cola = [xa cola];
    
    frame = getframe(f);
    
    % Convertir la trama en una imagen RGB (3 dimensiones) 
    im = frame2im(frame);
    
    % Transformar muestras de RGB a 1 dimensión con un mapa de color "cm" 
    [imind,cm] = rgb2ind(im , 256); 
   % if step == 1;
        % Crear el archivo GIF
       % imwrite(imind,cm,filename,'gif','DelayTime', DelayTime , 'LoopCount' , Inf  );
    %else
     %   % Añadir al GIF cada nuevo plot
      %  imwrite(imind,cm,filename,'gif','WriteMode','append','DelayTime', DelayTime );
    %end
end