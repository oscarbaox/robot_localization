# Recorded Bags

We recorded a bag file named `mac_first_floor_filter` with the `particle_cloud` topic recorded. Though the exact take wasn't screen recorded, it was in the same vein as [this](https://olincollege-my.sharepoint.com/:v:/g/personal/xbao_olin_edu/ER4HliaNTbNLj_lNvAtA-iYBvww0dxzBqGlCYdFNTIv3Ug) take. 

From this bag file, we can see that the particle filter is behaving as expected. The filter takes a few moments to localize to the right position and heading, maintaining doubt when multiple positions looked similar. After converging, the cloud is able to update with the robot's `odom` data and follow it closely. 