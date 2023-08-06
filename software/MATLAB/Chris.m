function gamma =Chris(n,M,e)
C = zeros(n,1);
gamma=[];
chr =zeros(n,n);
for i =1:n
    chr =zeros(n,n);
    for j=1:n
        for k =1:n
          chr=0.5*(diff(M(i,j),e(k)) + diff(M(i,k),e(j)) - diff(M(j,k),e(i)));
          gamma=[gamma,chr];          
        end
    end
end
end
