u1 = zeros(100,1); % Un vecteur avec que des zeros
u1([25;40]) = 1; % on fixe la valeur de u1 à 1 pour les indices 55,128,302 et 50
u1(60)=1.5; %u1 vaut 1.5 pour l'indice 60 
u1(80) = -1; %u1 vaut -1 en 80
hold on
plot(u1,'.') % on regarde parce que c'est joli
axis([0 100 -8 8]) % fixe la zone de traçage (un peu comme xlim, et ylim en une seule commande

u2 = [0;0;0;5;5;5;5;4;3;2;1;0];%u2 est un signal avec trois zeros, puis une rampe qui descend de 5 jusqu'à zero
plot(u2,'r')

s = conv(u1,u2); %s est la convolution de u1 avec u2
plot(s,'color',[0.5 0 0.5]); %On précise la couleur en RGB: 50% rouge, 50% bleu

hold off