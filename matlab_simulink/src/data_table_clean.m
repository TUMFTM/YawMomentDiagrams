%clean data table in a way that it is easier to asses in a plot
intr=1;
intc=1;
sz=size(ay_table);
ay_last=0;
ay_curr=0;
ay_next=0;
clean_table=zeros(beta_steps,delta_steps);

%clean table for constant delta lines
%will be multiplied with data table for cutting off plot lines 
%to make the horizontal edges of the diagram easier to asses

for intc=1:sz(2)
    for intr=1:round(sz(1)/2)
        ay_curr=ay_table(intr,intc,inti_v,inti_a);

        if intr==1
            ay_last=ay_curr;
        else
            ay_last=ay_table(intr-1,intc,inti_v,inti_a);
        end

        if intr==sz(1)
            ay_next=ay_curr;
        else
            ay_next=ay_table(intr+1,intc,inti_v,inti_a);
        end

        if ay_next>ay_curr 
            clean_table(intr,intc,inti_v,inti_a)=nan;
        else
            clean_table(intr,intc,inti_v,inti_a)=1;
        end
    end
    for intr=round(sz(1)/2):sz(1)
        ay_curr=ay_table(intr,intc,inti_v,inti_a);

        if intr==1
            ay_last=ay_curr;
        else
            ay_last=ay_table(intr-1,intc,inti_v,inti_a);
        end

        if intr==sz(1)
            ay_next=ay_curr;
        else
            ay_next=ay_table(intr+1,intc,inti_v,inti_a);
        end

        if ay_last<ay_curr 
            clean_table(intr,intc,inti_v,inti_a)=nan;
        else
            clean_table(intr,intc,inti_v,inti_a)=1;
        end
    end
end



