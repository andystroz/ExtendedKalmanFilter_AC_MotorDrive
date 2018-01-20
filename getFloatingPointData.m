function [time,s,c,w,theta] = getFloatingPointData(rpm,time_high,fs,np)

%% Create Rectangular Pulses
rps = rpm/60; 
num_seconds_per_one_rev = 1/rps;
number_of_pulses_per_revoultion = np;

t = 0:1/fs:time_high;
pulse_frequency = 1/(num_seconds_per_one_rev/np);
x2 = square(2*pi*pulse_frequency*t)/2 + 0.5;
%plot(t,x2)
%ylim([-0.5 1.5])

%% Create Counted Rectangular Pulses



count = 0;
counted_pulses= [];
for i = 1:length(x2)-1
    if x2(i) == 1 && x2(i+1) == 0
        count = count + 1;
    end
    if (count == number_of_pulses_per_revoultion)
       count = 0; 
    end
    counted_pulses(i) = (count);
end
counted_pulses(length(x2)) = (counted_pulses(length(x2)-1));

%plot(t,counted_pulses,"-",t,x2,"--");

%% Calaculate Taylor's Series Representation of the Count
n = 1;for j = 1:length(counted_pulses)
    w= counted_pulses(j)/number_of_pulses_per_revoultion;
    theta(j) = w;
    x = w*2*pi;
    %fprintf("%u %u\n",j,length(counted_pulses));
    ty = 0;
    for i = 0:n
        ty = ty +(((-1)^(i))*((x^(2*i+1)))/(factorial(2*i+1)));
    end
    ts_sin(j) = ty;
    %fprintf("%.16f\n",ty);
    sin_s(j) = sin(x);
    cos_s(j) = cos(x);
    if (t(j) ~= 0)
        angular_velocity(j) = x/t(j);
    end
end

%%
%plot(t,ts_sin);
%plot(t,sin_s);
%plot(t,cos_s);
%% EKF
 s = sin_s;
 c = cos_s;
 w = angular_velocity;
 time = t; 




