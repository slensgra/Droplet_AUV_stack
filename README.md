# Code stack for the Droplet underwater assembly robot

**Author:** Samuel Lensgraf, Ph.D., research scientist at IHMC during his Ph.D. in Computer Science at Dartmouth

*Disclaimer:*  This is research prototype code that I am currently refactoring to be more production ready. What I am changing for improved readability: integration of type hints, improved use of docstrings, improved use of ROS parameters for configuration.

<img src="https://github.com/user-attachments/assets/8e9f689d-7dad-43e9-bc76-d2f780d2dae7" width=400/>

<img src="https://github.com/user-attachments/assets/585908d2-875d-49ce-a7c8-c8145cfb581e" width=400/>

Droplet is the first free-floating autonomous underwater construction system capable of using active ballasting to transport cement building blocks efficiently. It is the first free-floating autonomous construction robot to use a paired set of resources: compressed air for buoyancy and a battery for thrusters. In construction trials, our system built structures of up to 12 components and weighing up to \SI{100}{Kg} (\SI{75}{Kg} in water). Our system achieves this performance by combining a novel one-degree-of-freedom manipulator, a novel two-component cement block construction system that corrects errors in placement, and a simple active ballasting system combined with compliant placement and grasp behaviors. The passive error correcting components of the system minimize the required complexity in sensing and control. We also explore the problem of buoyancy allocation for building structures at scale by defining a convex program which allocates buoyancy to minimize the predicted energy cost for transporting blocks.

This repository contains the code base that made the Droplet system possible. I used Droplet and evolved this codebase over several years. It integrates perception, control, and assembly behaviors with some basic motion planning capabilities. Droplet also included a user interface for divers. All was integrated using ROS and python.

There are a lot of components but some of the most important are:

This codebase was developed to support the Droplet autonomous underwater assembly robot. 
The framework was used to achieve the first example of free-floating autonomous underwater construction and supported field deployments!

If you are interested in my thoughts on the full development process, check out my blog posts at https://www.samlensgraf.com

Examples of it in action: 

* https://www.youtube.com/watch?v=TMYntbHJtng doing assembly on the "heavy" chassis for underwater construction
* https://www.samlensgraf.com/static_assets/cenotes_deployment/operating_the_robot.mp4 on a BlueROV chassis in the field
* https://www.youtube.com/watch?v=Cu6AQZC3TJ8 On the blueROV chassis doing underwater assembly with light blocks

---

## Code structure.

The main entrypoint to using the robot in the field would the the launch files `breadcrumb_and_fisheye_on_robot.launch` or `controller_test_live_robot.launch`

`breadcrumb_and_fisheye_on_robot.launch` runs the robot in a mode to test the combined perception and planning algorithm developed in this paper: https://arxiv.org/abs/2310.19408

`controller_test_live_robot.launch` would be the entrypoint for some field work we did.
