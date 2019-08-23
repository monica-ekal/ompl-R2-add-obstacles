/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2010, Rice University
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Rice University nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/
/* Original code from the OMPL Optimal Planning tutorial http://ompl.kavrakilab.org/optimalPlanningTutorial.html
 * Author: Luis G. Torres, Jonathan Gammell */

/* Modifications: Monica Ekal, 2019 */



#include <iostream>
#include <fstream>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/config.h>
#include <Python.h>
#include <math.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;

// Structs for obstacles of type circle and rectangle
struct Circle {
    double xc, yc;
    double radius;
};

struct Rectangle {
    double xmin, xmax;
    double ymin, ymax;
};

// The vector allObs stores all the added obstacles
class Obstacles{

    std::vector<Circle> allObs{ };

public:

    void addObstacles(Circle c){
        allObs.push_back(c);
    }

    friend class ValidityChecker;

};

// Collision checker. The robot's state space lies in [0,1]x[0,1]
class ValidityChecker : public ob::StateValidityChecker
{

    // Pointer for object of Obstacles class, so the vector allObs can be accessed for validity checking
    const Obstacles *obsPtr;

public:


    ValidityChecker(const ob::SpaceInformationPtr& si, const Obstacles* fin) :
            ob::StateValidityChecker(si) {obsPtr = fin;};

    // Returns whether the given state's position overlaps the
    // circular obstacle; valid states are ones that do not lie in the region of the obstacle

    bool isValid(const ob::State* state) const
    {
        return this->clearance(state) > 0.0;
    }
    // Returns the distance from the given state's position to the
    // boundary of the circular obstacle.
    double clearance(const ob::State* state) const
    {
        // We know we're working with a RealVectorStateSpace in this
        // example, so we downcast state into the specific type.
        const ob::RealVectorStateSpace::StateType* state2D =
                state->as<ob::RealVectorStateSpace::StateType>();
        // Extract the robot's (x,y) position from its state
        double x = state2D->values[0];
        double y = state2D->values[1];
        // Calculate clearance from all obstacles
        return this->calcClearance(x,y);
    }

    double calcClearance(double x,double y) const{

        // In case of no obstacles
        if(obsPtr->allObs.empty())
            return 1;

        // Iterate through allObs from class Obstacles and calculate clearance from each
        std::vector<double> clearanceAll {};
        for (auto it = obsPtr->allObs.begin();it != obsPtr->allObs.end();it++)
        {
            clearanceAll.push_back(calcClear( x, y,*it));
        }

        // return the least clearance
        return *std::min_element(clearanceAll.begin(),clearanceAll.end());
    }

    // Clearance from circular obstacles
    double calcClear(double x,double y, struct Circle c) const{
        return sqrt(pow(x-c.xc,2) + pow(y-c.yc,2)) - c.radius;
    }

    // To calculate clearance from rectangular obstacles. (to be added)
    double calcClear(double x,double y, struct Rectangle r) const{
        return std::max({r.xmin-x, x-r.xmax,r.ymin-y,y-r.ymax});
    }
};

// Add obstacle locations here
void setupObstacles(Obstacles& o){
    Circle c1{0.2,0.2,0.1};
    Circle c2{0.8,0.8,0.1};
    Circle c3{0.5,0.5,0.25};
    o.addObstacles(c1);
    o.addObstacles(c2);
    o.addObstacles(c3);

};

// Returns a structure representing the optimization objective to use
// for optimal motion planning. This method returns an objective which
// attempts to minimize the length in configuration space of computed
// paths.
ob::OptimizationObjectivePtr getPathLengthObjective(const ob::SpaceInformationPtr& si)
{
    return ob::OptimizationObjectivePtr(new ob::PathLengthOptimizationObjective(si));
}


int main() {

    // Construct the robot state space in which we're planning. We're
    // planning in [0,1]x[0,1], a subset of R^2.
    ob::StateSpacePtr space(new ob::RealVectorStateSpace(2));
    // Set the bounds of space to be in [0,1].
    space->as<ob::RealVectorStateSpace>()->setBounds(0.0, 1.0);

    // Construct a space information instance for this state space
    ob::SpaceInformationPtr si(new ob::SpaceInformation(space));

    // Add the location of the circular obstacles
    Obstacles o;
    setupObstacles(o);

    // Set the object used to check which states in the space are valid
    si->setStateValidityChecker(ob::StateValidityCheckerPtr(new ValidityChecker(si,&o)));
    si->setup();

    // Set our robot's starting state to be the bottom-left corner of
    // the environment, or (0,0).
    ob::ScopedState<> start(space);
    start->as<ob::RealVectorStateSpace::StateType>()->values[0] = 0.0;
    start->as<ob::RealVectorStateSpace::StateType>()->values[1] = 0.0;
    // Set our robot's goal state to be the top-right corner of the
    // environment, or (1,1).
    ob::ScopedState<> goal(space);
    goal->as<ob::RealVectorStateSpace::StateType>()->values[0] = 1.0;
    goal->as<ob::RealVectorStateSpace::StateType>()->values[1] = 1.0;

    // Create a problem instance
    ob::ProblemDefinitionPtr pdef(new ob::ProblemDefinition(si));
    // Set the start and goal states
    pdef->setStartAndGoalStates(start, goal);
    pdef->setOptimizationObjective(getPathLengthObjective(si));

    // Construct our optimizing planner using the RRTstar algorithm.
    ob::PlannerPtr optimizingPlanner(new og::RRTstar(si));
    // Set range of the RRT star algorithm
    std::dynamic_pointer_cast<og::RRTstar>(optimizingPlanner)->setRange(0.05);

    // Set the problem instance for our planner to solve
    optimizingPlanner->setProblemDefinition(pdef);
    optimizingPlanner->setup();

    // attempt to solve the planning problem within one second of
    // planning time
    ob::PlannerStatus solved = optimizingPlanner->solve(1.0);

    if (solved)
    {

        std::ofstream f("path.txt");
        ob::PathPtr path = pdef->getSolutionPath();
        std::cout << "Found solution:" << std::endl;
        // print the path to screen
        path->print(std::cout);
        // Convert the path to matrix format so that it can be printed on the screen
        std::dynamic_pointer_cast<og::PathGeometric>(path)->printAsMatrix(f);

        // Launch the python script for plotting
        /*
         Py_Initialize();
        FILE* PythonScriptFile = fopen("../plotting_python.py", "r");
        if(PythonScriptFile)
        {
            PyRun_SimpleFileEx(PythonScriptFile, "Python Scripts/Test.py",1);
        }
        Py_Finalize();
         */

    }
    else
        std::cout << "No solution found" << std::endl;


    return 0;
}