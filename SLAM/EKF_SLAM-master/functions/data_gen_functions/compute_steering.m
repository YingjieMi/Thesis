function [G,iwp]= compute_steering(x, wp, iwp, minD, G, rateG, maxG, dt)
%function [G,iwp]= compute_steering(x, wp, iwp, minD, G, rateG, maxG, dt)
%
% INPUTS:
%   x - true position
%   wp - waypoints
%   iwp - index to current waypoint
%   minD - minimum distance to current waypoint before switching to next
%   G - current steering angle
%   rateG - max steering rate (rad/s)
%   maxG - max steering angle (rad)
%   dt - timestep
%
% OUTPUTS:
%   G - new current steering angle
%   iwp - new current waypoint
%

% �ж�Ŀǰ�Ƿ񵽴����õ�
cwp= wp(:,iwp);
% �������
d2= (cwp(1)-x(1))^2 + (cwp(2)-x(2))^2;

if d2 < minD^2
    % ����ﵽ������ǰ����һ��Ŀ�ĵ�
    iwp= iwp+1; 
    if iwp > size(wp,2)
        % �������һȦ����
        iwp=0;
        return;
    end
    % ������һ��·��
    cwp= wp(:,iwp); 
end

% ������Ҫ�����ĺ���
deltaG= pi_to_pi(atan2(cwp(2)-x(2), cwp(1)-x(1)) - x(3) - G);

% ����ת���ٶ�
maxDelta= rateG*dt;
if abs(deltaG) > maxDelta
    deltaG= sign(deltaG)*maxDelta;
end

% ����ת��Ƕ�
G= G+deltaG;
if abs(G) > maxG
    G= sign(G)*maxG;
end
