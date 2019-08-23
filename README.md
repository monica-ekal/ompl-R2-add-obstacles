# ompl-R2-add-obstacles
This code uses OMPL, [The Open Motion Planning Library](http://ompl.kavrakilab.org/), for optimal planning in the $latex (R^2) space with collision avoidance.

The [Optimal Planning Tutorial](http://ompl.kavrakilab.org/optimalPlanningTutorial.html) performs checks for collision with one circular obstacle which is embedded directly in the function that calculates clearance. The code in this repository enables setting up multiple circular obstacles in the planning space through a dedicated function. Validity of the states is determined from results of collision checks with all of them.

## Examples of different obstacle setups
The red and black dots represent the start and goal points, respectively.

1. 
![alt text](/media/Figure_1.png)   


2. 
![alt text](/media/Figure_2.png)
