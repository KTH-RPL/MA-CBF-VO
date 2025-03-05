function [pAgents, vAgents] = agentsDistributedInCircle(Rcircle, nAgents)
% Agents are positioned in a circle, keeping the same angle separation
% between any two consecutives

deltaTheta = 2 * pi / nAgents;

pAgents = zeros([3, nAgents]);
for i = 0:(nAgents - 1)
    theta_i = deltaTheta * i;
    pAgents(:,i+1) = [cos(theta_i); sin(theta_i); 0] * Rcircle;
end

vAgents = zeros([3, nAgents]);

end