# Trajectory Follower

This trajectory follower is based on the 'Carrot Chasing' algorihm. You can find more information on the following reference:

H. Perez-Leon, J. J. Acevedo, I. Maza and A. Ollero, "A 4D trajectory follower based on the ’Carrot chasing’ algorithm for UAS within the U-space context," 2020 International Conference on Unmanned Aircraft Systems (ICUAS), 2020, pp. 1860-1867, doi: 10.1109/ICUAS48674.2020.9213979.

A software package with more functionalities such as trajectory interpolation or path following can be found here:

https://github.com/hecperleo/upat_follower

## Input

You can publish your trajectories on "trajectory_to_follow" topic

## Output

The trajectory follower will compute the proper velocity to follow the trajectory. It is published in "ual/set_velocity"
