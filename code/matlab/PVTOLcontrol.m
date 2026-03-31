clc, clear;

Ts = 0.01;
t = 0:Ts:10;

x = [2, 2, 0, 0, 0, 0];
u = [0, 0];

% parametros del sistema
M   = 0.5;	g   = 9.81;	Iyy = 0.15;
 
A = 1;              % amplitud
T = 10;             % periodo
f = 1/T;            % frecuencia
omega = 2*pi*f;     % frecuencia de oscilacion
offset = 2;

% z deseada
zd   =  offset + A*sin(omega*t);
zdp  =  A*omega*cos(omega*t);
zdpp = -A*omega^2*sin(omega*t);

% x deseada
xd   =  offset + A*cos(omega*t);
xdp  = -A*omega*sin(omega*t);
xdpp = -A*omega^2*cos(omega*t);

%% Ganancias de control
k1_z = 40;	k1_x = 5;	k1_theta = 50;
k2_z = 15;	k2_x = 3;	k2_theta = 15;

for i=1:length(t)-1
    
    [tt,xx] = ode45(@mod_pvtol, [t(i),t(i+1)], x(i,:), [], u(i,:));
    x(i+1,:) = xx(end,:);
    
    % variables de estado
    X  = x(i+1,1);  Z  = x(i+1,2);  Theta  = x(i+1,3);
    Xp = x(i+1,4);  Zp = x(i+1,5);  Thetap = x(i+1,6);

    % Control de Z --------------------------------------------------------
    ez  = zd(i+1)  - Z;
    ezp = zdp(i+1) - Zp;
    
    u(i+1,1) = (M/cos(Theta))*(zdpp(i+1) + k2_z*ezp + k1_z*ez + g);
   
    % Control de X --------------------------------------------------------
    ex  = xd(i+1)  - X;
    exp = xdp(i+1) - Xp;
    
    alpha_x = zdpp(i+1) + k2_z*ezp + k1_z*ez + g; 
    
    % ley de control para x
    mu_x = xdpp(i+1) + k2_x*exp + k1_x*ex;
    
    % control de theta (subactuado)
    thetad(i+1)   = atan(mu_x/alpha_x); 
    thetadp(i+1)  = (thetad(i+1)  - thetad(i))/Ts;
    thetadpp(i+1) = (thetadp(i+1) - thetadp(i))/Ts;
    
    etheta(i+1)  = thetad(i+1)  - Theta;
    ethetap(i+1) = thetadp(i+1) - Thetap;
    
    u(i+1,2) = Iyy*(thetadpp(i+1) + k2_theta*ethetap(i+1) + k1_theta*etheta(i+1));
end
