function [data_new,force_new,c_new]=pick_n_best_points(data,force,c,n_last,n_total)

data_new=data(end-n_last+1:end,:);
force_new=force(end-n_last+1:end);
c_new=c(end-n_last+1:end);

idx=randperm(length(data)-n_last);
data_new=[data_new;data(idx(1:n_total-n_last),:)];
force_new=[force_new;force(idx(1:n_total-n_last))];
c_new=[c_new;c(idx(1:n_total-n_last))];

end

