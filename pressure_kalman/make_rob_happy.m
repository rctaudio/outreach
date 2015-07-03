%% create array
%gather datas
clc;
clear all;

filename = 'test_data/DATALOG.csv';
m = csvread(filename, 1);
col1 = m(:,1);
col2 = m(:,2);
time = (0:1:length(col1)-1);

%gather header names
fid = fopen(filename);
paramIDs = textscan(fid, '%s',1,'HeaderLines',0);
header = regexp(paramIDs{1}{1}, ',', 'split');
fclose(fid);

%% plot the relative height
time = (0:1:length(col1)-1);
plot(time,col1);


%% kalman filter area

%define the meta-variables (ie how long and often we will sample)
duration = length(col1) -1;
dt =  sum(m(:,2)) / ((length(col1) -1)*1000); %time was in millis()

%%Define update equations (Coefficent matrices)

A = [1 dt ((dt*dt)/2); 0 1 dt; 0 0 1];  %state transistion matrix: expected height of the sensor (state prediction)
B = 0;                                  %input control matrix: there is no control, so this is zero?     
C = [1 0 0];                            %measurement matrix: the expected measurement given the predicted state

%%define main variables

u = 0; %acceleration magnitude
Q = [0;0;0]; %initalized state of the sensor [position; velocity; acceleration]
Q_estimate = Q; %x_estimate of initial location of where the sensor is ( what we are updating)
Sensor_pressure_noise_mag = .2; %process noise: the variability in how fast the Quail is speeding up (stdv of acceleration: meters/sec^2)
arduino_noise_mag = 1;  %measurement noise: How mask-blinded is the Ninja (stdv of location, in meters)
Ez = arduino_noise_mag^2;% Ez convert the measurement noise (stdv) into covariance matrix
Ex = Sensor_pressure_noise_mag^2 * [0 0 0; 0 0 0; 0 0 1]; % Ex convert the process noise (stdv) into covariance matrix
P = Ex; % estimate of initial Quail position variance (covariance matrix)

%%initize result variables
% Initialize for speed
Q_loc_meas = col1; % Quail path that the Ninja sees

%%Do kalman filtering
%initize estimation variables
Q_loc_estimate = []; %  Quail position estimate
vel_estimate = []; % Quail velocity estimate
acc_estimate = []; % Quail ackbar estimate
P_estimate = P;
P_mag_estimate = [];
predic_state = [];
predic_var = [];
for t = 1:length(col1)
    % Predict next state of the quail with the last state and predicted motion.
    Q_estimate = A * Q_estimate + B * u;
%    predic_state = [predic_state; Q_estimate(1)] ;
    %predict next covariance
    P = A * P * A' + Ex;
    predic_var = [predic_var; P] ;
    % predicted Ninja measurement covariance
    % Kalman Gain
    K = P*C'*inv(C*P*C'+Ez);
    % Update the state estimate.
    Q_estimate = Q_estimate + K * (Q_loc_meas(t) - C * Q_estimate);
    % update covariance estimation.
    P =  (eye(3)-K*C)*P;
    %Store for plotting
    Q_loc_estimate = [Q_loc_estimate; Q_estimate(1)];
    vel_estimate = [vel_estimate; Q_estimate(2)];
    acc_estimate = [acc_estimate; Q_estimate(3)];
    P_mag_estimate = [P_mag_estimate; P(1)];
end

% Plot the results
figure(2);
tt = 0 : dt : (duration*dt);
plot(time,Q_loc_meas,'-k.', time,Q_loc_estimate,'-g.');
axis([0 length(time) 95 105])

figure(3);
clf
subplot(3,1,1)
hold on
tt = 0 : dt : (duration*dt);
plot(time,Q_loc_estimate,'-k.');
axis([0 5000 90 120])
subplot(3,1,2)
hold on
tt = 0 : dt : (duration*dt);
plot(time,vel_estimate,'-k.');
axis([0 5000 -3.5 3.5])
subplot(3,1,3)
hold on
tt = 0 : dt : (duration*dt);
plot(time,acc_estimate,'-k.');
axis([0 5000 -2 2])

% %plot the evolution of the distributions
% figure(3);clf
% for T = 1:length(Q_loc_estimate)
% clf
%     x = Q_loc_estimate(T)-5:.01:Q_loc_estimate(T)+5; % range on x axis
%       
%     %predicted next position of the quail     
%     hold on
%     mu = predic_state(T); % mean
%     sigma = predic_var(T); % standard deviation
%     y = normpdf(x,mu,sigma); % pdf
%     y = y/(max(y));
%     hl = line(x,y,'Color','m'); % or use hold on and normal plot
%        
%     %data measured by the ninja
%     mu = Q_loc_meas(T); % mean
%     sigma = arduino_noise_mag; % standard deviation
%     y = normpdf(x,mu,sigma); % pdf
%     y = y/(max(y));
%     hl = line(x,y,'Color','k'); % or use hold on and normal plot
%     
%     %combined position estimate
%     mu = Q_loc_estimate(T); % mean
%     sigma = P_mag_estimate(T); % standard deviation
%     y = normpdf(x,mu,sigma); % pdf
%     y = y/(max(y));
%     hl = line(x,y, 'Color','g'); % or use hold on and normal plot
%     axis([Q_loc_estimate(T)-5 Q_loc_estimate(T)+5 0 1]);     
% 
%     
%     %actual position of the quail
%     plot(Q_loc(T)); 
%     ylim=get(gca,'ylim');
%     line([Q_loc(T);Q_loc(T)],ylim.','linewidth',2,'color','b');
%     legend('state predicted','measurement','state estimate','actual Quail position')
%     %pause
% end


%% plot the raw data
time = (0:1:length(col1)-1);
for k = 1:(length(header))
    subplot(1,6,k);
    scatter(time, m(:,k), 1);
    title(header{k});
end

%% clean plot of accel
%looks like x is the axis with gravity

%compensate for gravity.
%uncomment the one you want to use for gravity compensation
% grav = 1; % x axis
% grav = 2; % y axis
% grav = 3; % z axis
%create an accel vector

accel = m(:,4);
for k = 2:12
    accel = [accel,m(:, k+3 + (fix((k-1)/3)*6))];    
end
%remove the normalized gravity from the x matrixes
if exist('grav' , 'var')    
    v = zeros(1,12);
    for k = 1:3
        v(k*grav) = -1;
    end
    v = repmat(v,length(time),1);
    accel = accel + v;
end
%plot the accel data
for k = 1:12
    subplot(4,3,k);
    scatter(time, accel(:,k), 1);
    title(header{k+3 + (fix((k-1)/3)*6)});    
end

%% clean plot of gyro

% create a gyro vector
gyro = m(:,1);
for k = 2:12
    gyro = [gyro,m(:, k + fix((k-1)/3)*6)];    
end

%plot the gyro data
for k = 1:12
    subplot(4,3,k);
    scatter(time, gyro(:,k), 1);
    title(header{k + (fix((k-1)/3)*6)});    
end

%% clean plot of compass
% create a compass vector
compass = m(:,7);
for k = 2:12
    compass = [compass,m(:, k + 6 + fix((k-1)/3)*6)];    
end

%plot the compass data
for k = 1:12
    subplot(4,3,k);
    scatter(time, compass(:,k), 1);
    title(header{k + 6 + fix((k-1)/3)*6});    
end

%% make an accel vector

%initial arrow
p0 = [0,0,0];
p = accel(:,:);
z = zeros(length(p),1);

B = 1/3*ones(3,1);
out = filter(B,1,p);


for k = 1:length(out)
%     subplot(2,2,1);
    quiver3(0,0,0, out(k,1),out(k,2), out(k,3));
    axis([-1.2 1.2 -1.2 1.2 -1.2 1.2]);
    
%     subplot(2,2,2);
%     quiver3(0,0,0, out(k,4),out(k,5), out(k,6));
%     axis([-1.2 1.2 -1.2 1.2 -1.2 1.2]);
%     
%     subplot(2,2,3);
%     quiver3(0,0,0, out(k,7),out(k,8), out(k,9));
%     axis([-1.2 1.2 -1.2 1.2 -1.2 1.2]);
%     
%     subplot(2,2,4);
%     quiver3(0,0,0, out(k,10),out(k,11), out(k,12));
%     axis([-1.2 1.2 -1.2 1.2 -1.2 1.2]);
    
    drawnow;
    
end

%% graph accel, gyro, compass on one plot
%here is the accel data

%compensate for gravity.
%uncomment the one you want to use for gravity compensation
% grav = 1; % x axis
% grav = 2; % y axis
% grav = 3; % z axis
%create an accel vector

accel = m(:,4);
for k = 2:12
    accel = [accel,m(:, k+3 + (fix((k-1)/3)*6))];    
end
%remove the normalized gravity from the x matrixes
if exist('grav' , 'var')    
    v = zeros(1,12);
    for k = 1:3
        v(k*grav) = -1;
    end
    v = repmat(v,length(time),1);
    accel = accel + v;
end

% create a gyro vector
gyro = m(:,1);
for k = 2:12
    gyro = [gyro,m(:, k + fix((k-1)/3)*6)];    
end

% create a compass vector
compass = m(:,7);
for k = 2:12
    compass = [compass,m(:, k + 6 + fix((k-1)/3)*6)];    
end


%plot the data
for k = 1:3    
    subplot(2,3,k);
    hold on
    scatter(time, accel(:,k), 1, 'b');
    scatter(time, gyro(:,k), 1 , 'r');
    ylim([-10,10]);   
end
for k = 1:3    
    subplot(2,3,k+3);
    hold on
    scatter(time, compass(:,k), 1 , 'g');
end


%% Bayesian Ninja tracking Quail using kalman filter

clear all
%% define our meta-variables (i.e. how long and often we will sample)
duration = 10  %how long the Quail flies
dt = .1;  %The Ninja continuously looks for the birdy,
%but we'll assume he's just repeatedly sampling over time at a fixed interval

%% Define update equations (Coefficent matrices): A physics based model for where we expect the Quail to be [state transition (state + velocity)] + [input control (acceleration)]
A = [1 dt; 0 1] ; % state transition matrix:  expected flight of the Quail (state prediction)
B = [dt^2/2; dt]; %input control matrix:  expected effect of the input accceleration on the state.
C = [1 0]; % measurement matrix: the expected measurement given the predicted state (likelihood)
%since we are only measuring position (too hard for the ninja to calculate speed), we set the velocity variable to
%zero.

%% define main variables
u = 1.5; % define acceleration magnitude
Q= [0; 0]; %initized state--it has two components: [position; velocity] of the Quail
Q_estimate = Q;  %x_estimate of initial location estimation of where the Quail is (what we are updating)
Sensor_pressure_noise_mag = 0.05; %process noise: the variability in how fast the Quail is speeding up (stdv of acceleration: meters/sec^2)
arduino_noise_mag = 10;  %measurement noise: How mask-blinded is the Ninja (stdv of location, in meters)
Ez = arduino_noise_mag^2;% Ez convert the measurement noise (stdv) into covariance matrix
Ex = Sensor_pressure_noise_mag^2 * [dt^4/4 dt^3/2; dt^3/2 dt^2]; % Ex convert the process noise (stdv) into covariance matrix
P = Ex; % estimate of initial Quail position variance (covariance matrix)

%% initize result variables
% Initialize for speed
Q_loc = []; % ACTUAL Quail flight path
vel = []; % ACTUAL Quail velocity
Q_loc_meas = []; % Quail path that the Ninja sees


%% simulate what the Ninja sees over time
figure(2);clf
figure(1);clf
for t = 0 : dt: duration

    % Generate the Quail flight
    QuailAccel_noise = Sensor_pressure_noise_mag * [(dt^2/2)*randn; dt*randn];
    Q= A * Q+ B * u + QuailAccel_noise;
    % Generate what the Ninja sees
    NinjaVision_noise = arduino_noise_mag * randn;
    y = C * Q+ NinjaVision_noise;
    Q_loc = [Q_loc; Q(1)];
    Q_loc_meas = [Q_loc_meas; y];
    vel = [vel; Q(2)];
    %iteratively plot what the ninja sees
    plot(0:dt:t, Q_loc, '-r.')
    plot(0:dt:t, Q_loc_meas, '-k.')
    axis([0 10 -30 80])
    hold on
    pause
end

%plot theoretical path of ninja that doesn't use kalman
plot(0:dt:t, smooth(Q_loc_meas), '-g.')

%plot velocity, just to show that it's constantly increasing, due to
%constant acceleration
%figure(2);
%plot(0:dt:t, vel, '-b.')


%% Do kalman filtering
%initize estimation variables
Q_loc_estimate = []; %  Quail position estimate
vel_estimate = []; % Quail velocity estimate
Q= [0; 0]; % re-initized state
P_estimate = P;
P_mag_estimate = [];
predic_state = [];
predic_var = [];
for t = 1:length(Q_loc)
    % Predict next state of the quail with the last state and predicted motion.
    Q_estimate = A * Q_estimate + B * u;
    predic_state = [predic_state; Q_estimate(1)] ;
    %predict next covariance
    P = A * P * A' + Ex;
    predic_var = [predic_var; P] ;
    % predicted Ninja measurement covariance
    % Kalman Gain
    K = P*C'*inv(C*P*C'+Ez);
    % Update the state estimate.
    Q_estimate = Q_estimate + K * (Q_loc_meas(t) - C * Q_estimate);
    % update covariance estimation.
    P =  (eye(2)-K*C)*P;
    %Store for plotting
    Q_loc_estimate = [Q_loc_estimate; Q_estimate(1)];
    vel_estimate = [vel_estimate; Q_estimate(2)];
    P_mag_estimate = [P_mag_estimate; P(1)]
end

% Plot the results
figure(2);
tt = 0 : dt : duration;
plot(tt,Q_loc,'-r.',tt,Q_loc_meas,'-k.', tt,Q_loc_estimate,'-g.');
axis([0 10 -30 80])


%plot the evolution of the distributions
figure(3);clf
for T = 1:length(Q_loc_estimate)
clf
    x = Q_loc_estimate(T)-5:.01:Q_loc_estimate(T)+5; % range on x axis
      
    %predicted next position of the quail     
    hold on
    mu = predic_state(T); % mean
    sigma = predic_var(T); % standard deviation
    y = normpdf(x,mu,sigma); % pdf
    y = y/(max(y));
    hl = line(x,y,'Color','m'); % or use hold on and normal plot
       
    %data measured by the ninja
    mu = Q_loc_meas(T); % mean
    sigma = arduino_noise_mag; % standard deviation
    y = normpdf(x,mu,sigma); % pdf
    y = y/(max(y));
    hl = line(x,y,'Color','k'); % or use hold on and normal plot
    
    %combined position estimate
    mu = Q_loc_estimate(T); % mean
    sigma = P_mag_estimate(T); % standard deviation
    y = normpdf(x,mu,sigma); % pdf
    y = y/(max(y));
    hl = line(x,y, 'Color','g'); % or use hold on and normal plot
    axis([Q_loc_estimate(T)-5 Q_loc_estimate(T)+5 0 1]);     

    
    %actual position of the quail
    plot(Q_loc(T)); 
    ylim=get(gca,'ylim');
    line([Q_loc(T);Q_loc(T)],ylim.','linewidth',2,'color','b');
    legend('state predicted','measurement','state estimate','actual Quail position')
    pause
end