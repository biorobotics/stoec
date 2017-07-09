function [Xest,residue,num_starts]=find_unique_states(Xest,residue,epsilon)
idx=[];
for ii=1:size(Xest,1)-1
    
    for jj=ii+1:size(Xest,1)
        if norm(Xest(ii,:)-Xest(jj,:))<epsilon
            idx=[idx,jj];
        end
    end
    
    
end

idx=unique(idx);
Xest(idx,:)=[];
residue(idx)=[];
num_starts=size(Xest,1);

end