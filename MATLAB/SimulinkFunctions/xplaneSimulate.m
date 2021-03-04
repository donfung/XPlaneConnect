function [pos_m, uvw_mps, latLon_deg, altMsl_m, aspd_mps, euler_deg, pqr_radps, alphaBeta_deg, gamma_deg] = xplaneSimulate(actuatorCmds, initConds)
% Inputs
%     ActuatorCmds: control array where the elements are as follows:
%           1. Elevator Stick [-1,1]
%           2. Aileron Stick [-1,1]
%           3. Pedal [-1, 1]
%           4. Throttle [-1, 1]
%           5. Gear (0=up, 1=down)
%           6. Flaps [0, 1]
%           7. Speed Brakes [-0.5, 1.5]
%
%     initConds: Initial conditions for the aircraft and simulation
%           1. initConds(1:3): Lat (deg), Lon (deg), Alt MSL (m)
%           2. initConds(4:6): u, v, w (mps)
%           3. initConds(7:9): Theta, Phi, Psi (deg)
%           4. initConds(10:12): p, q, r (degps)
%
    persistent Socket; % retain socket connection across method calls (https://blogs.mathworks.com/simulink/2014/01/08/simulink-and-matlab-objects/)
    persistent isInitialized;
    
    if isempty(Socket)
        global clients;
        if isempty(clients)    
           Socket = XPlaneConnect.openUDP();
        else
            Socket = clients(1);
        end
    end
    
    if isempty(isInitialized) || ~isInitialized 
        % Initialize XPlane:
        %    1. Position
        %    2. Velocity
        %    3. Attitude
        %    4. Angular velocity
    
        % Position and attitude
        posi0 = [initConds(1), initConds(2), initConds(3), initConds(7), initConds(8), initConds(9),      1];
        XPlaneConnect.sendPOSI(posi0, 0, Socket);

        % Velocity and angular velocity
        XPlaneConnect.sendDREF('sim/flightmodel/position/local_vx', initConds(4), Socket); % Velocities
        XPlaneConnect.sendDREF('sim/flightmodel/position/local_vy', initConds(6), Socket); % Vy in XP is vertical velocity (Vz)
        XPlaneConnect.sendDREF('sim/flightmodel/position/local_vz', initConds(5), Socket); % Vz in XP is side velocity (Vy)
        XPlaneConnect.sendDREF('sim/flightmodel/position/P', 0, Socket);
        XPlaneConnect.sendDREF('sim/flightmodel/position/Q', 0, Socket);
        XPlaneConnect.sendDREF('sim/flightmodel/position/R', 0, Socket);
           
        isInitialized = true;
    end
    NOCHANGE = -998;  % Reserved value for XPlane to indicate no change 
    eleStick = actuatorCmds(1);
    ailStick = actuatorCmds(2);
    rudStick = actuatorCmds(3);
    flapsCmd = actuatorCmds(4);
    thrCmd = actuatorCmds(5);
    XPlaneConnect.sendCTRL([eleStick, ailStick, rudStick, thrCmd, NOCHANGE, flapsCmd, NOCHANGE], 0, Socket);

%     posi = XPlaneConnect.getPOSI(0, Socket);
    % Local XYZ coordinates
    posX_m = XPlaneConnect.getDREFs('sim/flightmodel/position/local_vx', Socket);
    posZ_m = XPlaneConnect.getDREFs('sim/flightmodel/position/local_vy', Socket);
    posY_m = XPlaneConnect.getDREFs('sim/flightmodel/position/local_vz', Socket);
    pos_m = [posX_m, posY_m, posZ_m];
    
    % LLA
%     lat_deg = XPlaneConnect.getDREFs('sim/flightmodel/position/latitude', Socket);
%     lon_deg = XPlaneConnect.getDREFs('sim/flightmodel/position/longitude', Socket);
%     altMsl_m = XPlaneConnect.getDREFs('sim/flightmodel/position/elevation', Socket);
    posi = XPlaneConnect.getPOSI(0, Socket);
    lat_deg = posi(1);
    lon_deg = posi(2);
    altMsl_m = posi(1);
    latLon_deg = [lat_deg, lon_deg];
    
    % Attitude
    theta_deg = XPlaneConnect.getDREFs('sim/flightmodel/position/theta', Socket);
    phi_deg = XPlaneConnect.getDREFs('sim/flightmodel/position/phi', Socket);
    psi_deg = XPlaneConnect.getDREFs('sim/flightmodel/position/psi', Socket);
    euler_deg = [theta_deg, phi_deg, psi_deg];
    
    % Alpha Beta
    alpha_deg = XPlaneConnect.getDREFs('sim/flightmodel/position/alpha', Socket);
    beta_deg = XPlaneConnect.getDREFs('sim/flightmodel/position/beta', Socket);
    alphaBeta_deg = [alpha_deg, beta_deg];
    gamma_deg = XPlaneConnect.getDREFs('sim/flightmodel/position/vpath', Socket);  % Flight path angle

    % Airspeed
    aspdInd_mps = XPlaneConnect.getDREFs('sim/flightmodel/position/indicated_airspeed', Socket);
    aspdTrue_mps = XPlaneConnect.getDREFs('sim/flightmodel/position/true_airspeed', Socket);
    aspd_mps = aspdTrue_mps;
    
    % Body velocities
    u_mps = XPlaneConnect.getDREFs('sim/flightmodel/position/local_vx', Socket);
    w_mps = XPlaneConnect.getDREFs('sim/flightmodel/position/local_vy', Socket);
    v_mps = XPlaneConnect.getDREFs('sim/flightmodel/position/local_vz', Socket);
    uvw_mps = [u_mps, v_mps, w_mps];
    
    % Angular velocities (body frame)
    p_degps = XPlaneConnect.getDREFs('sim/flightmodel/position/P', Socket);
    q_degps = XPlaneConnect.getDREFs('sim/flightmodel/position/Q', Socket);
    r_degps = XPlaneConnect.getDREFs('sim/flightmodel/position/R', Socket);    
    p_radps = p_degps * pi/180;
    q_radps = q_degps * pi/180;
    r_radps = r_degps * pi/180;
    pqr_radps = [p_radps, q_radps, r_radps];
    
end

