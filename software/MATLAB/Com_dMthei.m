function dMthei = Com_dMthei(M, thei)

dMthei = 0.5*[ diff(M(1,1),thei), diff(M(1,2),thei), diff(M(1,3),thei), diff(M(1,4),thei), diff(M(1,5),thei), diff(M(1,6),thei);
           diff(M(2,1),thei), diff(M(2,2),thei), diff(M(2,3),thei), diff(M(2,4),thei), diff(M(2,5),thei), diff(M(2,6),thei);
           diff(M(3,1),thei), diff(M(3,2),thei), diff(M(3,3),thei), diff(M(3,4),thei), diff(M(3,5),thei), diff(M(3,6),thei);
           diff(M(4,1),thei), diff(M(4,2),thei), diff(M(4,3),thei), diff(M(4,4),thei), diff(M(4,5),thei), diff(M(4,6),thei);
           diff(M(5,1),thei), diff(M(5,2),thei), diff(M(5,3),thei), diff(M(5,4),thei), diff(M(5,5),thei), diff(M(5,6),thei);
           diff(M(6,1),thei), diff(M(6,2),thei), diff(M(6,3),thei), diff(M(6,4),thei), diff(M(6,5),thei), diff(M(6,6),thei)];