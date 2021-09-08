
t = 1:16;
for j =1:t
shape(j) = DecreasingBellShapedFunction(3, 3.5, 0, 1, t(j)) + IncreasingBellShapedFunction(5.5, 6, 0, 1, t(j));
end
shape
figure;
hplot = plot(t, shape);
set(hplot, 'LineWidth', 1);