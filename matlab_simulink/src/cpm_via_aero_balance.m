%pitch moment coefficent calculation script
a=1.744; %distance from front axle to cog
wb=2.898; %wheelbase
b=wb-a; %distance from rear axle to cog
cl=-2.2; %coefficent of lift
wd=b/wb; %weight distribution front

%AeroBalance at different speeds
ab=zeros(20); %Aerobalance front at speeds from 10 mps to 200 mps
ab=[0.3982 0.3782 0.38 0.38 0.38 0.38 0.39 0.39 0.39 0.39 0.39 0.40 0.40 0.40 0.40 0.40 0.41 0.41 0.41 0.41 0.41];

mp=-cl*((1-ab).*b-ab*a); %pitch moment
cpm=mp/wb %pitch moment coefficent
