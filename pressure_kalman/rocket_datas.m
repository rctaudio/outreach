%% create array
%gather datas
clc;
clear all;

filename = 'test_data/Failed.csv';
m = csvread(filename, 1);
c1 = m(1:end,1);
c2 = m(1:end,2);
% c3 = m(250:end,3);
% c4 = m(250:end,4);
% c5 = m(250:end,5);
time = (0:1:length(c1)-1);

%gather header names
fid = fopen(filename);
paramIDs = textscan(fid, '%s',1,'HeaderLines',0);
header = regexp(paramIDs{1}{1}, ',', 'split');
fclose(fid);

% plot the relative height
% time = (0:1:length(col1)-1);
% hold on;
% plot(col1,col2);
% plot(col1,m(:,3));


% kalman filter area

%define the meta-variables (ie how long and often we will sample)
duration = length(c1) -1;
%dt =  sum(m(:,1)) / ((length(c1) -1)*1000); %time was in millis()

dt = mean( diff( m(:,1) ) );

%%Define update equations (Coefficent matrices)

A = [1 dt ((dt*dt)/2); 0 1 dt; 0 0 1];  %state transistion matrix: expected height of the sensor (state prediction)
B = 0;                                  %input control matrix: there is no control, so this is zero?     
C = [1 0 0];                            %measurement matrix: the expected measurement given the predicted state

%%define main variables

u = 0; %acceleration magnitude
Q = [80;0;0]; %initalized state of the sensor [position; velocity; acceleration]
Q_estimate = Q; %x_estimate of initial location of where the sensor is ( what we are updating)
Sensor_pressure_noise_mag = 2; %process noise: the variability in how fast the Quail is speeding up (stdv of acceleration: meters/sec^2)
arduino_noise_mag = .5;  %measurement noise: How mask-blinded is the Ninja (stdv of location, in meters)
Ez = arduino_noise_mag^2;% Ez convert the measurement noise (stdv) into covariance matrix
Ex = Sensor_pressure_noise_mag^2 * [(dt^6)/9 (dt^5)/6 (dt^4)/3; (dt^5)/6 (dt^4)/4 (dt^3)/2; (dt^4)/3 (dt^3)/3 (dt^2)/1]; % Ex convert the process noise (stdv) into covariance matrix
P = [1000    0    0;
     0    1000    0;
     0    0    1000]; % estimate of initial Quail position variance (covariance matrix)

%%initize result variables
% Initialize for speed
Q_loc_meas = c2; % Quail path that the Ninja sees

%%Do kalman filtering
%initize estimation variables
Q_loc_estimate = []; %  Quail position estimate
vel_estimate = []; % Quail velocity estimate
acc_estimate = []; % Quail ackbar estimate
P_estimate = P;
P_mag_estimate = [];
predic_state = [];
predic_var = [];
for t = 1:length(c1)
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
clf;
tt = 0 : dt : (duration*dt);
y = time;
hold on
%plot(y,Q_loc_meas,'-k.', y,Q_loc_estimate,'-m.');
% 
% Q_loc_meas_norm = Q_loc_meas./max(Q_loc_meas);
% Q_loc_estimate_norm = Q_loc_estimate./max(Q_loc_estimate);
% c4_norm = c4./(max(c4) * 5);
% c5_norm = c5./(max(c5) * 5);
a1 = Q_loc_meas;
b1 = Q_loc_estimate;

% Q_loc_meas_norm     = ( a1 - min(a1) ) ./ ( max(a1) - min(a1) );
% Q_loc_estimate_norm = ( b1 - min(b1) ) ./ ( max(b1) - min(b1) );
% c4_norm             = ( c4 - min(c4) ) ./ ( max(c4) - min(c4) );
% c5_norm             = ( c5 - min(c5) ) ./ ( max(c5) - min(c5) );
 
 Q_loc_meas_norm     = ( a1 - min(a1) ) ./ ( max(a1) );
 Q_loc_estimate_norm = ( b1 - min(b1) ) ./ ( max(b1) );
%  c4_norm             = ( c4  ) ./ ( max(c4) );
%  c5_norm             = ( c5  ) ./ ( max(c5) );


plot(y,Q_loc_meas_norm,'-k.', y,Q_loc_estimate_norm,'-m.');
% plot(y, c4_norm, '-r.', y, c5_norm, '-b.');

% % 
% % %
% %plot the data from the arduino
% %
% 
% c2_norm     = ( c2 - min(c2) ) ./ ( max(c2) );
% c3_norm = ( c3 - min(c3) ) ./ ( max(c3) );
% c4_norm             = ( c4  ) ./ ( max(c4) );
% c5_norm             = ( c5  ) ./ ( max(c5) );
% 
% plot(y, c2_norm, '-k.', y, c3_norm, '-m.', y, c4_norm, '-r.', y, c5_norm, '-b.');

%axis([0 length(time) 95 105])

figure(3);
clf
subplot(3,1,1)
hold on
tt = 0 : dt : (duration*dt);
plot(y,Q_loc_estimate,'-k.');
axis([0 350 60 135])
subplot(3,1,2)
hold on
tt = 0 : dt : (duration*dt);
plot(y,vel_estimate,'-k.');
axis([0 350 -40 80])
subplot(3,1,3)
hold on
tt = 0 : dt : (duration*dt);
plot(y,acc_estimate,'-k.');
axis([0 350 -40 80])

SAVEME=[c1,Q_loc_estimate,vel_estimate,acc_estimate]

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

