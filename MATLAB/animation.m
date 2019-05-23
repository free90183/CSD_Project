load('sim_phi_v0');
a = 2; b = 5;
P1 = [0 0];
axis(gca, 'equal');
axis([-3 7 -1 3]);

k = 1;
figure(1);
hold on;
for t=1:500
   theta = k*(t/10);
   P2 = a*[sin(sim_phi_v0(t)) cos(sim_phi_v0(t))];
   cir = viscircles(P1, 1);
   bar = line([P1(1) P2(1)], [P1(2) P2(2)]) ;
   pause(0.1);
   delete(bar);
   delete(cir);
end
