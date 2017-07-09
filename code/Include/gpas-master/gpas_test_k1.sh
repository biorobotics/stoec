# A demonstration of upper-confidence-bound (UCB) search on a Gaussian Process (GP) using receding-horizon cross-entropy (CE) trajectory optimization

# start env node
matlab -r "opt.envFile='data/k1.ppm'; env_node(opt);" &

# start odom node
matlab -r "opt.envFile='data/k1.ppm'; odom_node(opt);" &

# start gpas node
# rule of thumb: the GP max stdev (opt.gp.s) should be chosen so that roughly the maximum value of the GP should be equal to at most twice opt.gp.s (assuming the mean is zero)
matlab -r "opt.envFile='data/k1.ppm'; opt.gp.s=.8; gpas_node(opt);" &
