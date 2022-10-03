function dU = GT_fun_3(t,U,T,B,isVertical,mf,A,dethrottle_K,dragLimit,steering,V_star)

%% Variable Explanations
V   = U(1);
Gam = U(2);
X   = U(3);
H   = U(4);
m   = U(5);
%%
dU = zeros(5,1);    % a column vector, prep for later calculations

R_M = 6371*1000; % Radius of earth (avg)
g0=9.81; %% gravitational acceleration on Earth surface (m/s^2)
C_d = 0.237; % Coefficient of drag 
g = g0*(R_M/(R_M+H))^2; % Calc of gravity at radius m/s^2
if H < 20000
	[~,~,~,rho] = atmosisa(H); % to get density of air
else
	rho = 0;
end

D = 0.5*rho*A*C_d*V.^2; % Aerodynamic drag
T;
rho;

%%  dV
if V <= V_star
	isAcc = 0;
	if m > mf % Turns thrust off if fuel is zero
		if D > dragLimit;
			T_throttle = dethrottle_K*T;
			dV = T_throttle/m - D/m - g*sin(Gam);
		else
			T_throttle = T;
			dV = T_throttle/m - D/m - g*sin(Gam);
		end
	else
		T_throttle = T;
		dV = -D/m - g*sin(Gam);
	end
else
	T_throttle = T;
	dV = -D/m - g*sin(Gam);
	isAcc = 1;
end
%% dGam
if steering == 0
	if isVertical == 1 % Avoids singularity at start and angle changes when thrusting vertical
		dGam = 0;
	else
		part2_1 = -(1/V);
		part2_2 = g-((V.^2)/(R_M+H) );
		part2_3 = cos(Gam);
		dGam = part2_1*part2_2*part2_3;
	end
else
	t_burnout = m/B;
	dGam = atan(tan(-Gam)*(1/t_burnout));
end
%% dX
dX = (R_M/(R_M+H))*V*cos(Gam);

%% dH
dH = V*sin(Gam);

%% dm
if m >= mf | isAcc == 0;  % Turns mass flow off if fuel is zero
	if T_throttle == dethrottle_K*T
		dm = -dethrottle_K*B;
	else
		dm= -B;
	end
else
	dm = 0;
end
if m <= mf
	dm = 0;
end
dU = [dV dGam dX dH dm]';

% dGam = -(1/V)*(g-((V.^2)/(R_M+H)))*cos(Gam);
% dGam = -(1/V)*(g/((R_M+H)^2))*cos(Gam) + (V*cos(Gam)/(R_M+H));
% dX = V*cos(Gam);

