function G = gaussianPDF(delU, delD,CovM)
    
    % computes the gaussian for a column disparity map
    % delU = u_ij- u
    % delD = d_ij- d
    % CovM = all Measuresments covariance Matrix
    
    Epsilon = [delU ; 0 ; delD];
    G = (1/(((2*pi)^1.5)*det(CovM))) * exp(-0.5 * Epsilon'*inv(CovM) * Epsilon);

end
