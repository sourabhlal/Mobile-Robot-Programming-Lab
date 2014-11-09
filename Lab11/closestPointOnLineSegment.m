function [rad2 , po] = closestPointOnLineSegment(pi,p1,p2)
    % Find point po on a line segment p1 - p2 closest to a given
    % point pi and return the closest point and the square of
    % the distance to it. The line segment has endpoints p1
    % and p2 (column vectors) and the point is pi. If the
    % closest point is an endpoint, returns infinity for rad2
    % because such points are bad for lidar matching
    % localization.
    dx13 = pi(1)-p1(1);
    dy13 = pi(2)-p1(2);
    dx12 = p2(1)-p1(1);
    dy12 = p2(2)-p1(2);
    dx23 = pi(1)-p2(1);
    dy23 = pi(2)-p2(2);
    v1 = [dx13 ; dy13];
    v2 = [dx12 ; dy12];
    v3 = [dx23 ; dy23];
    v1dotv2 = dot(v1,v2);
    v2dotv2 = dot(v2,v2);
    v3dotv2 = dot(v3,v2);
    if v1dotv2 > 0.0 && v3dotv2 < 0.0
        % Closest is on segment
        scale = v1dotv2/v2dotv2;
        po = v2*scale + [p1(1) ; p1(2)];
        dx = pi(1)-po(1);
        dy = pi(2)-po(2);
        rad2 = dx*dx+dy*dy;
    elseif v1dotv2 <= 0.0
        % Closest is first endpoint
        po = [p1(1) ; p1(2)];
        rad2 = inf;
    else
        % Closest is second endpoint
        po = [p2(1) ; p2(2)];
        rad2 = inf;
    end
end