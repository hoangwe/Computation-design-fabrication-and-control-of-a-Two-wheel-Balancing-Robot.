function C(i,j,k)= Tijk(i,j,k,n)
for i =1:n
    for j=1:n
        for k =1:n
            C(i,j,k)=(0.5*(diff(M(i,j),e(k,k))+diff(M(i,k),e(j,j))-diff(M(j,k),e(i,i))));
        end
    end
end
end