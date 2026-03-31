function dqdt = mod_pvtol(~, x, u)
    
    % variables de estado
    X      = x(1);
    Z      = x(2);
    Theta  = x(3);
    Xp     = x(4);
    Zp     = x(5);
    Thetap = x(6);
    
    q  = [X;  Z;  Theta];               
    qp = [Xp; Zp; Thetap]; 
    
    % entradas de control
    fq        = u(1);
    tau_theta = u(2);

    % parametros del sistema
    M   = 0.5;
    g   = 9.81;
    Iyy = 0.15;
    
    %% Modelo dinamico
    
    % matriz de inercia
    Mq = [M,  0,  0;
          0,  M,  0;
          0,  0, Iyy];

    % matriz de fuerzas centripetas y de Coriolis
    C = [0, 0, 0;
         0, 0, 0;
         0, 0, 0];

    % par gravitacional
    G = [0;
         M*g; 
         0];
  
    % fuerzas externas aplicadas
    Gamma = [fq*sin(Theta);
             fq*cos(Theta);
             tau_theta];
       
    %% Ecuaciones de estado
    dqdt(1:3,1) = qp;
    dqdt(4:6,1) = inv(Mq)*(Gamma - C*qp - G);

end