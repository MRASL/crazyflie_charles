from math import sqrt


n_cf = 1*1

cf_per_side = sqrt(n_cf) # Number of CF per side

if cf_per_side % 2 == 0:
    n_radius = 0
    for n in range(1, int(cf_per_side/2 + 1)):
        n_radius += n
    
else:
    n_radius = 1
    max_range = int((cf_per_side + 1)/2 + 1)
    for n in range(1, max_range):
        n_radius += (2*n - 1)/2


print n_radius