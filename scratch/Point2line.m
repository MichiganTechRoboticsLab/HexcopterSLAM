

% Plot


% Proplem: find the point on the line defined by two endpoints which is
% closest to another arbitraty point.

% Line Segment
a = [0.5 1 2];
b = [3   2 3];
L = [a; b];

% Point
p = [2.8 2.1 2.8];

% Plot
figure(1)
clf
plot3(L(:,1), L(:,2), L(:,3), '--g');
hold on
grid on
plot3(a(1), a(2), a(3), '.r', 'MarkerSize', 5);
plot3(b(1), b(2), b(3), '.g', 'MarkerSize', 5);
plot3(p(1), p(2), p(3), '.b', 'MarkerSize', 5);
title('Problem');
axis equal




%
% Translate to the origin for simplicity
%

% Point
p = p - a;

% Line Segment
b = b - a;

Org_a = a;
a = a - a;
L = [a; b];

% Plot
figure(2)
clf
plot3(L(:,1), L(:,2), L(:,3), '--g');
hold on
grid on
plot3(a(1), a(2), a(3), '.r', 'MarkerSize', 5);
plot3(b(1), b(2), b(3), '.g', 'MarkerSize', 5);
plot3(p(1), p(2), p(3), '.b', 'MarkerSize', 5);
title('Translate to Origin');
axis equal



%
% Find the closest point on the line.
%

% Change B into a normalized vector
B = b/norm(b);

% Dot Product 
d = dot(p,B);

% vector times dot product result
P = d * B;

% Line Segment
b = b - a;
a = a - a;

% Plot

figure(3)
clf
L = [a; b];
plot3(L(:,1), L(:,2),  L(:,3), '--g');
hold on
grid on

plot3(a(1), a(2),  a(3), '.r', 'MarkerSize', 5);
plot3(b(1), b(2),  b(3), '.g', 'MarkerSize', 5);
plot3(p(1), p(2),  p(3), '.b', 'MarkerSize', 5);

plot3(B(1), B(2),  B(3), '+k', 'MarkerSize', 5);
plot3(P(1), P(2),  P(3), '.k', 'MarkerSize', 10);

L = [p; P];
plot3(L(:,1), L(:,2),  L(:,3), '--k');

axis equal
title('Almost Result');



% translate back to original space

a = Org_a;
b = b + Org_a;
p = p + Org_a;
P = P + Org_a;


figure(4)
clf
L = [a; b];
plot3(L(:,1), L(:,2),  L(:,3), '--g');
hold on
grid on
plot3(a(1), a(2), a(3), '.r', 'MarkerSize', 5);
plot3(b(1), b(2), b(3), '.g', 'MarkerSize', 5);
plot3(p(1), p(2), p(3), '.b', 'MarkerSize', 5);
plot3(P(1), P(2), P(3), '.k', 'MarkerSize', 5);
plot3(P(1), P(2), P(3), 'Ok', 'MarkerSize', 15);
title('Problem Solution');
axis equal
