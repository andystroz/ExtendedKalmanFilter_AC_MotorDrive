function [x,P]=extendedKalmanFloatingPoint(fstate,x,P,hmeas,z,Q,R,t_sc,w_b)
% returns state est, x and state cov, P for the motor speed detection
% problem
[x1,A] = partial_f(fstate,x,t_sc,w_b);
P=A*P*A'+Q;                 %partial update
[z1,H]=partial_h(hmeas,x1);
K=P*H'*inv(H*P*H'+R);       %Kalman filter gain
x=x1+K*(z-z1);            %state estimate
P=P-K*H*P;               %state covariance matrix
end

% Jacobian Matricies Manually Caluculated and Added
function [eval,partial_differentiation]=partial_f(f,x,t_s,w_b)
    eval=f(x);
    partial_differentiation = [cos(t_s*w_b*x(3)),-sin(t_s*w_b*x(3)),-x(1)*t_s*w_b*sin(t_s*w_b*x(3))-x(2)*t_s*w_b*cos(t_s*w_b*x(3));...
        sin(t_s*w_b*x(3)),cos(t_s*w_b*x(3)),x(1)*t_s*w_b*cos(t_s*w_b*x(3))-x(2)*t_s*w_b*sin(t_s*w_b*x(3));...
        0,0,1];
end

function [eval,partial_differentiation]=partial_h(h,x)
	eval=h(x);
    partial_differentiation = [1,0,0;0,1,0];
end
