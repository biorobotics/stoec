function [opt, x, f] = gp_select(opt)

%opt.xss = mvnrnd(opt.mu, opt.Sigma, opt.Ns)';

tic
[ms, ss] = gp_predict(opt.fgp, opt.xss);
disp('predict')
toc

vs = sqrt(diag(ss));

% lower confidence bound
%

% prob of improvement

ind = [];
f = [];

switch opt.sel
 case 'p'
%  [y,ind]=max(cdf(gp.fmin, ms, vs));

 case 'pi'
  if (~isempty(opt.cgp))
    [cms, css] = gp_predict(opt.cgp, opt.xss);
    cvs = sqrt(diag(css));
  
    inan = find(cvs==0);
    
    opt.fP = normcdf(opt.fmin, ms, vs);
    opt.cP = normcdf(cms, zeros(size(cms)), cvs);
        
    opt.J = opt.fP.*opt.cP;

    opt.J = opt.J/sum(opt.J);
    J = opt.J;
    
%    I = find(sqrt(sum(opt.xss.*opt.xss, 1)) < 2);
 %   J
    
    while 1
      [f,ind]=max(J);
      x = opt.xss(:,ind);
            
      % difference between all points and this one
      dx = repmat(opt.xss(:,ind), 1, size(opt.fgp.xs,2)) - opt.fgp.xs;
      if sqrt(sum(dx.*dx, 1)) < .01
        J(ind) = 0;
        continue;
      else
        break
      end
    end
  else
    [f,ind]=max(normcdf(opt.fmin, ms, vs));
  end
  
 case 'uc'
  [f,ind]=min(ms - 1.96*vs);
  
 otherwise
  disp('uknown select case')  
end

x = opt.xss(:,ind);
