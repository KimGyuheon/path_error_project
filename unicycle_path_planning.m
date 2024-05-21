t = 0;
dt = 0.1;

x = [0; 0; 0*(pi/180)];

t_arr = [];
x_arr = [];

u = zeros(2,1); % control input vector
u_max = [4.0; 1.5]; % upper-bound of control inputs
err_los_old = 0;
err_cte_old = 0;

wp_list = [ [0; 0], [20; 0], [20; 20], [0; 20], [0; 0] ];
wp_idx = 1;

R_acc = 1.0; % acceptance radius

figure; 
plot(wp_list(1,:), wp_list(2,:), 'k.', 'markersize', 14); hold on; grid on;
for i = 1 : size(wp_list,2)
    draw_wp_circle(wp_list(1:2,i), R_acc);
end

while(wp_idx < size(wp_list,2))
    t = t + dt;
    t_arr = [t_arr; t];

    ref_wp = wp_list(:,wp_idx);
    trg_wp = wp_list(:,wp_idx+1);
    wp_pos = [ref_wp, trg_wp];
     
    % control input calculation
    kp_x = 1.5;

    diff_wp2wp = [wp_pos(1,2)-wp_pos(1,1), wp_pos(2,2)-wp_pos(2,1)]';
    diff_v2wp = [wp_pos(1,2)-x(1), wp_pos(2,2)-x(2)]';

    err_dist = sqrt( diff_v2wp(1)^2 + diff_v2wp(2)^2 );
    u(1) = kp_x*err_dist;
    
    psi = x(3);
    psi_d = atan2(diff_v2wp(2), diff_v2wp(1));
    alpha = atan2(diff_wp2wp(2), diff_wp2wp(1));
    beta = psi_d - alpha;


    kp_psi_los = 5;
    kd_psi_los = 0.5;
    error_los = pi_to_pi(psi_d - psi);
    error_los_dot = (error_los - err_los_old)/dt;


    kp_psi_cte = 0.5;
    kd_psi_cte = 1.0;
    error_cte = err_dist * sin(beta);
    error_cte_dot = (error_cte - err_cte_old)/dt;

    u(2) = (kp_psi_los*error_los + kd_psi_los*error_los_dot) * (kp_psi_cte*error_cte + kd_psi_cte*error_cte_dot);
    
    err_los_old = error_los;
    err_cte_old = error_cte;

    if(u(1) > u_max(1)) u(1) = u_max(1);  
    elseif(u(1) < -u_max(1)) u(1) = -u_max(1); end
    if(u(2) > u_max(2)) u(2) = u_max(2);  
    elseif(u(2) < -u_max(2)) u(2) = -u_max(2); end
    
    % state propagation
    [x] = propagateState(x,u,dt);

    % next waypoint or not?
    dist = dist2wp_2d(trg_wp,x);
    if(dist < R_acc); wp_idx = wp_idx + 1; end
            
    % show / plot
    drawVehicle(x); hold on; grid on;
    xlabel('x [m]'); ylabel('y [m]');
    axis equal; 
    set(gca, 'ydir', 'reverse');
    drawnow;

end

%% subfunctions
function [x] = propagateState(x,u,dt)
xdot = zeros(3,1);
x(3) = x(3) + u(2)*dt;
x(3) = pi_to_pi(x(3));
xdot(1) = u(1)*cos(x(3));
xdot(2) = u(1)*sin(x(3));
x(1:2) = x(1:2) + xdot(1:2)*dt; % time integration (Euler scheme)
end

function [dist] = dist2wp_2d(trg_wp,x)
dist = sqrt((trg_wp(1,1)-x(1))^2 + (trg_wp(2,1)-x(2))^2) ;
end

function [] = draw_wp_circle(center,radius)
theta = linspace(0,2*pi,100);
x = radius*cos(theta) + center(1);
y = radius*sin(theta) + center(2);
plot(x, y, 'k:', 'linewidth', 1);
end

function [ang] = pi_to_pi(ang)
if (ang > pi); ang = ang - 2*pi;
elseif (ang < -pi); ang = ang + 2*pi; end
end

function [] = drawVehicle(x)
L = 0.2; % vehicle length
B = 0.1;   % vehicle breadth
objx = L*[1 -1 -1 1]';
objy = B*[0,1,-1,0]';
own_posx = x(1) + objx*cos(x(3))-objy*sin(x(3));
own_posy = x(2) + objx*sin(x(3))+objy*cos(x(3));
fill(own_posx,own_posy,[242 135 41]/255);
plot(own_posx,own_posy,'color',[30 67 89]/255);
end