ReadMe for Outlier_Detection_v01

This is what will find points outside of a desired range given a point cloud. The comments should be a good starting point but it will be explained in detail here.

Line 5 is where you input your pcd file (point cloud) generated either by model interpolation or hard coded. It is currently hard coded, as we did not get model interpolation to work.

Included are three example point clouds of cubes, one perfect, and two with errors that can be substituted in place of a pcd file for testing.

The for loop on line 41 starts to check each point against a hard coded bound in order to find an outlier. Since this concept checks for a cube, there are 6 if's, one for each face. If a point is outside any given bound, it will be added to a list for you to do some calcualtions on and return to the robot. The logic is that all points will be within the bounds if the clay is in the correct shape, and any outside are areas on the clay that need to be shaped.