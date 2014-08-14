// Bring in my package's API, which is what I'm testing
#include "../src/PathPlannerNode.h"
#include "../src/PathPlanner.h"
// Bring in gtest
#include <gtest/gtest.h>

// Declare a test
TEST(Navigation, testPlannerGeneratesPath)
{
    //Create path planner and setup nodes
    PathPlanner pathPlanner = PathPlanner();

    string node1Name = "testnode1";
    string node2Name = "testnode2";
    string node3Name = "testnode3";
    string node4Name = "testnode4";
    PathPlannerNode node1 = PathPlannerNode(&node1Name,-2.5,3);
    PathPlannerNode node2 = PathPlannerNode(&node2Name,-2.5,-0);
    PathPlannerNode node3 = PathPlannerNode(&node3Name,3,0);
    PathPlannerNode node4 = PathPlannerNode(&node4Name,3,3);
    
    node1.addNeighbour(&node2);
    node2.addNeighbour(&node1);
    node2.addNeighbour(&node3);
    node3.addNeighbour(&node2);
    node3.addNeighbour(&node4);
    node4.addNeighbour(&node3);
    
    pathPlanner.addNode(&node1);
    pathPlanner.addNode(&node2);
    pathPlanner.addNode(&node3);
    pathPlanner.addNode(&node4);

    vector<PathPlannerNode*> path = pathPlanner.pathToNode(&node1, &node4);
    vector<PathPlannerNode*> correctPath;
    correctPath.insert(path.begin(), &node4);
    correctPath.insert(path.begin(), &node3);
    correctPath.insert(path.begin(), &node2);
    correctPath.insert(path.begin(), &node1);
    ASSERT_EQ(path, correctPath);
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
testing::InitGoogleTest(&argc, argv);
return RUN_ALL_TESTS();
}