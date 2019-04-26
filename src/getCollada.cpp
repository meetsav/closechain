#include <moveit/robot_model/robot_model.h>
#include "moveit/robot_model_loader/robot_model_loader.h"
#include "std_msgs/String.h"
#include <assimp/Importer.hpp>
#include <assimp/DefaultLogger.hpp>
#include "assimp/postprocess.h"
#include <assimp/scene.h>
#include <fstream>
#include <ctime>
#include "iostream"
#include "memory"
#include "ompl/base/StateSpace.h"
#include"ompl/base/spaces/SE3StateSpace.h"
#include "ompl/base/ProblemDefinition.h"
#include "ompl/base/SpaceInformation.h"
#include "ompl/config.h"
#include "omplapp/config.h"
#include "ompl/geometric/GeneticSearch.h"
#include "ompl/geometric/planners/rrt/RRT.h"

using namespace std;
using namespace robot_model;
namespace ob = ompl::base;
namespace og = ompl::geometric;

using namespace urdf;

int main(int argc, char **argv)
{

    Assimp::Importer importer;

    Assimp::DefaultLogger::create("", Assimp::Logger::VERBOSE, aiDefaultLogStream_STDOUT);



    const aiScene* cSpace = importer.ReadFile( "/home/meet/ros_ws/src/closechain/src/Environment.dae",
                                                 aiProcess_CalcTangentSpace |
                                                 aiProcess_Triangulate |
                                                 aiProcess_JoinIdenticalVertices |
                                                 aiProcess_SortByPType);
    const aiScene* robotEnv = importer.ReadFile( "/home/meet/ros_ws/src/closechain/src/Robot.dae",
                                                 aiProcess_CalcTangentSpace |
                                                 aiProcess_Triangulate |
                                                 aiProcess_JoinIdenticalVertices |
                                                 aiProcess_SortByPType);

    Assimp::DefaultLogger::kill();

    //check if robotEnv

    if (robotEnv == NULL)
    {
        std::cout << __LINE__ << ": Error importing robotEnv. Check if file exists ! " << std::endl;
        return -1;
    }
    else
    {
        int meshIdx = robotEnv->mRootNode->mChildren[0]->mMeshes[0];
        int envIdx = cSpace->mRootNode->mChildren[0]->mMeshes[0];

        cout << __LINE__ << ": mNumFaces: " << robotEnv->mMeshes[meshIdx]->mNumFaces << std::endl;
        cout << __LINE__ << ": HasFaces(): " << robotEnv->mMeshes[meshIdx]->HasFaces() << std::endl;
        cout << __LINE__ << ": ptr mFace[0]: " << &(robotEnv->mMeshes[meshIdx]->mFaces[0]) << std::endl;

        cout << __LINE__ << ": mNumFaces: " << cSpace->mMeshes[envIdx]->mNumFaces << std::endl;
        cout << __LINE__ << ": HasFaces(): " << cSpace->mMeshes[envIdx]->HasFaces() << std::endl;
        cout << __LINE__ << ": ptr mFace[0]: " << &(cSpace->mMeshes[envIdx]->mFaces[0]) << std::endl;
    }

    //flow chart for ompl
    //define state space
    //start and goal position
    //set the upper bound for the class
    //then start with problem defination,it is possible with out the problem defination but it does require the default planner setting

    auto space(std::make_shared<ob::SE3StateSpace>());//creating SE3statespace.
    ompl::base::RealVectorBounds bound(3);//i am assuming 3D space
    bound.setLow(-1);
    bound.setHigh(1);
    space->setBounds(bound);//done with space
    auto si(std::make_shared<ob::SpaceInformation>(space));//space information for random planner

    ompl::base::ScopedState<> start(space);//defining the starting position

    start.random();

    ompl::base::ScopedState<> goal(space);//defining goal postion
    goal.random();

    auto probdef(make_shared<ompl::base::ProblemDefinition>(si));//here prblem defination will be set up

    probdef->setStartAndGoalStates(start,goal);

    //now set the planner that i want to check for ex PRM or RRT
    auto planner(make_shared<ompl::geometric::RRT>(si));
    planner->setProblemDefinition(probdef);
    planner->setup();
   std::clock_t s = std::clock();
    ompl::base::PlannerStatus is_solve = planner->ompl::base::Planner::solve(1.0);
    if(is_solve)
    {
        cout<<"time taken by finding trejectory is : "<<(clock()-s)<<endl;
        ompl::base::PathPtr path=probdef->getSolutionPath();
        path->print(cout);
    }







    return 0;
}