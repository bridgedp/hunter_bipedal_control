import numpy as np
import matplotlib.pyplot as plt

# def function relaxedBarrierPenalty
def relaxedBarrierPenalty(h, mu, delta):
    """
    This function is used to calculate the value of the relaxed barrier penalty function
    Input:
        h: the value of the first variable
        mu: the value of the second variable
        delta: the value of the third variable
    Output:
        the value of the relaxed barrier penalty function
    """
    if h >= delta:
        # compute the log of h
        return -mu * np.log(h)
    else:
        return mu / 2 * (((h - 2*delta)/delta)**2 -1) - mu * np.log(delta)


# plot function relaxedBarrierPenalty
hs = np.arange(-3, 10, 0.1)
ys = []
mu = 0.1
delta = 0.1
for h in hs:
    ys.append(relaxedBarrierPenalty(h,mu,delta))
plt.plot(hs, ys)
plt.show()