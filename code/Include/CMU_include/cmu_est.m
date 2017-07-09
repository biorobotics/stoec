function [Xreg,residue,Xregsave,tiksave,p ] = cmu_est( data,force, c ,cad_points,cad_normal,Xreg)
%data is of size n*x, force is of size n*1, c is of size n*1
residue=0;
Xprev=Xreg;
Xregsave=Xreg;
tiksave=[];
n_last=10;
n_total=min(40,length(c));
[data,force,c,]=pick_n_best_points(data,force,c,n_last,n_total);



for ii=1:20
    p=[];s=[];
    tmp=1;
    tic
    for jj=1:size(data,1)
       
        r1 = data(jj,:);
        F1 =force(jj,:);
        s1 = computeTransformedPoints(r1, Xreg);
        %using knn search for closest point now. 
        %This can be replaced by seth's method. Since Seth's method doesnt 
        %work on all laptops, I am using knn search for now, which is very 
        %fast although not the most accurate
        ind1=knnsearch(cad_points,s1');
        p1=cad_points(ind1,:);
       
           ptmp= p1-cad_normal(ind1,:)*F1/c(jj);
           stmp=s1;
        
        if norm(ptmp-stmp')<50
            p(:,tmp)=ptmp;
            s(:,tmp)=stmp;
            tmp=tmp+1;
        end
        
    end
    
    if isempty(p)~=1 && isempty(s)~=1
        indx=randperm(size(p,2));
        indx=indx(1:floor(1*size(indx,2)));
        
        
        [TR,TT] = findT(p(:,indx),s(:,indx));%T*s=p
          %  display('');
        if det(TR)<0
            tmp_var=1;
            tt=1;
            while tmp_var==1
                
                ind=randperm(size(p,2));
                roundoff=floor((0.8/tt)*size(p,2));
                if roundoff<=3
                    TR=eye(3);
                    TT=zeros(3,1);
                    break;
                end
                pf=p(:,ind(1:roundoff));
                sf=s(:,ind(1:roundoff));
                [TR,TT] = findT(pf,sf);%T*s=p
                if det(TR)>0
                    tmp_var=0;
                end
                tt=tt+1;
            end
        end
        residue=0;
        for rr=1:size(p,2)
        residue=residue+norm(p(:,rr)-bsxfun(@plus,TR*s(:,rr),TT));
        end  
        Tinc=eye(4,4);
        Tinc(1:3,1:3)=TR;
        Tinc(1:3,4)=TT;
        Treg=regParamsToTransformationMatrix(Xreg);
        Treg=Tinc*Treg;
        rot=rpy(Treg(1:3,1:3));
        Xreg(4:6)=[rot(3),rot(2),rot(1)];
        Xreg(1:3)=Treg(1:3,4);
        if norm(Xreg(1:3)-Xprev(1:3))<0.01 && norm(Xreg(4:6)-Xprev(4:6))<0.005%convergence criteria
            break
        end
            Xprev=Xreg;
Xregsave=[Xregsave;Xreg];
    end
    tiksave=[tiksave,toc];
end
%display('');


end

