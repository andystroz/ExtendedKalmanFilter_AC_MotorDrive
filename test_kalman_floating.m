rpm = 50;
pulses_per_rev = 120;
simulation_time = 30;
fs = 10000;
w_b = 1;
t_sc = 0.0001;
q = 2^-15;
r=1000;     
[time,sin_s,cos_s,w,theta] = getFloatingPointData(rpm,simulation_time,fs,pulses_per_rev);

%%

n=3;      %number of states
Q=q*eye(3); 
R=r*eye(2);      
f=@(x)[x(1)*cos(w_b*t_sc*x(3)) - x(2)*sin(w_b*t_sc*x(3));...
    x(1)*sin(w_b*t_sc*x(3)) + x(2)*cos(w_b*t_sc*x(3));...
    x(3)];  % state
h=@(x)[x(1);x(2)]; % measurement                            
s=[cos_s(1);sin_s(1);0]; % initial state
x=s; 
P = eye(n);                  % initial state cov

N=length(sin_s);                                   

state_estimates = zeros(n,N);          
state_values = zeros(n,N);          
measurements = zeros(2,N);

for k=1:N
  z = h([cos_s(k);sin_s(k)]); %measurement
  state_values(:,k)= s;
  measurements(:,k)  = z;                            
  [x, P] = extendedKalmanFloatingPoint(f,x,P,h,z,Q,R,t_sc,w_b); 
  state_estimates(:,k) = x;
  s = f(s)+ q*randn(3,1); % update process 
end

to_rpm = 30/pi;
figure
axis tight;
subplot(3,1,1)
plot(time, state_estimates(1,:), 'k-', time, cos_s, 'b-')
title('Extended Kalman Filter')
xlabel('Time (s)');
ylabel('cos(\theta)');
subplot(3,1,2)
plot(time, state_estimates(2,:), 'k-', time, sin_s, 'g-')
xlabel('Time (s)');
ylabel('sin(\theta)');
subplot(3,1,3)
plot(time, state_estimates(3,:)*to_rpm, 'k-',time,rpm*ones(size(time)),'r--')
xlabel('Time (s)');
ylabel('Motor Speed (rpm)');
axis tight;
grid on;
