function [e] = gen_color_noise(L,sigma2,c)
nc = length(c) - 1;
xik=zeros(nc,1);%��������ֵ
xi=randn(L,1)*sqrt(sigma2);%������ֵΪ0������Ϊsigma2�ĸ�˹����������
e = zeros(1,L);
for k=1:L
    e(k)=c*[xi(k);xik];%������ɫ����
    %���ݸ���
    for i=nc:-1:2
        xik(i)=xik(i-1);
    end
    xik(1)=xi(k);
end

end

