function perceptron(dataset,eta,k)
if (k <2 | k>eta)
    error("the 'n' parameter dosen't fit the requirements")
end
X = dataset(1:(size(dataset)/2),:);
test_set = dataset((size(dataset)/2):end,:);
% initialization 
target = X(:,end);
n = size(X,1);
w = rand(1,3);
iteration = 0;
err = 1;

while(err <= 0.5 | iteration == 10000) % patterns  
r = X(l,:).*w;
a(l) = sgn(r);
output_err = 0.5*(target(l) - a(l));
deltaW = eta*output_err*X(l,:);
w = w + deltaW;
l = l + 1;
iteration = iteration + 1;
if(l > n)
    l = 1;
    err = abs(a - target);
    err = mean(err);
end
end

target_test = test_set(:,end);
C = zeros(size(unique(target_test),1),size(unique(target_test),1));

for e = 1:size(test_set,1)
   r = test_set(e,:).*w;
   a(e) = sgn(r);
   C(target_test(e),a) = C(target_test(e),a) +1;
end