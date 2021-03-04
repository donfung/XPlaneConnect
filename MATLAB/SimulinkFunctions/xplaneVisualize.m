function xplaneVisualize(posAtt)
% Inputs
%     posAtt: Position array where the elements are as follows:
%           1. Latitiude (deg)
%           2. Longitude (deg)
%           3. Altitude (m above MSL)
%           4. Roll (deg)
%           5. Pitch (deg)
%           6. True Heading (deg)
%           7. Gear (0=up, 1=down)

    persistent Socket; % retain socket connection across method calls (https://blogs.mathworks.com/simulink/2014/01/08/simulink-and-matlab-objects/)
    
    if isempty(Socket)
       Socket = XPlaneConnect.openUDP();
    end
    
    XPlaneConnect.sendPOSI(posAtt, 0, Socket);
end

