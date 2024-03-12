function [x,P]= EKF_update(x,P,z,R,idf, batch)
    % Inputs:
    %   x, P - SLAM state and covariance
    %   z, R - range-bearing measurements and covariances
    %   idf - feature index for each z
    %   batch - switch to specify whether to process measurements together or sequentially
    %
    % Outputs:
    %   x, P - updated state and covariance
    
    if size(z,2) == 0
        x = x;
        P = P;
        return
    end

    if batch == 1
        [x,P]= batch_update(x,P,z,R,idf);
    else
        [x,P]= single_update(x,P,z,R,idf);
    end

end


% ���θ���
function [x,P]= batch_update(x,P,z,R,idf)
    lenz= size(z,2);
    lenx= length(x);
    H= zeros(2*lenz, lenx);
    v= zeros(2*lenz, 1);
    RR= zeros(2*lenz);
    
    % �����еĹ۲�ֵ�ŵ�ͬһ������
    for i=1:lenz
        ii= 2*i + (-1:0);
        [zp,H(ii,:)]= observe_model(x, idf(i));
        
        v(ii)=      [z(1,i)-zp(1);
            pi_to_pi(z(2,i)-zp(2))];
        RR(ii,ii)= R;
    end
    
    % ������������һ���Ը���
    [x,P]= KF_cholesky_update(x,P,v,RR,H);

end


%  ��������
function [x, P] = single_update(x, P, z, R, idf)
    % ��ö��ٸ�·��
    lenz = size(z, 2);

    % ���θ���·����и���
    for i = 1:lenz
        % ���ݹ۲�ģ�ͻ�ù۲�ģ��Ԥ��۲�ֵ
        [zp, H] = observe_model(x, idf(i));

        % ��ȡԤ��۲�ֵ��ʵ�ʹ۲�ֵ֮��Ĳ�ֵ
        v = [z(1, i) - zp(1);
            pi_to_pi(z(2, i) - zp(2))];

        % ��ø���ֵ
        [x, P] = KF_cholesky_update(x, P, v, R, H);
    end
end
