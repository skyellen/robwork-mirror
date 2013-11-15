function [ Q ] = toQ( vector )
%toQ Convert MatLab vector to RobWork Q vector.
    import dk.robwork.DoubleVector;
    vec = DoubleVector(length(vector));
    for i=1:length(vector)
        vec.set(i-1,vector(i));
    end
    Q = dk.robwork.Q(vec);
end