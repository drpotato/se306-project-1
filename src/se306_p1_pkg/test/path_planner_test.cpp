#include "../src/PathPlanner.h"
#include "../src/PathPlannerNode.h"

// Bring in gtest
#include <gtest/gtest.h>

TEST(TestSuite, test_path_planner_constructor) {
   PathPlanner *pathPlanner = new PathPlanner();
   ASSERT_EQ(pathPlanner->nodes.size(), 0);
}

// TEST(TestSuite, test_add_node) {
//     PathPlanner *pathPlanner = new PathPlanner();

//     string nodeName = "testnode";

//     PathPlannerNode* node = new PathPlannerNode(&nodeName,-2.5,3);

//     pathPlanner->addNode(node);

//     PathPlannerNode* testNode = pathPlanner->getNode(&nodeName);

//     ASSERT_EQ(node, testNode);
// }

// TEST(TestSuite, test_remove_node) {
//     PathPlanner *pathPlanner = new PathPlanner();

//     string nodeName = "testnode";

//     PathPlannerNode* node = new PathPlannerNode(&nodeName,-2.5,3);

//     pathPlanner->addNode(node);

//     PathPlannerNode* testNode = pathPlanner->getNode(&nodeName);
//     //pathPlanner->removeNode(&nodeName);
//     //ASSERT_EQ(pathPlanner->nodes.size(), 0);
//     ASSERT_EQ(0,0);
// }

// TEST(TestSuite, test_generate_path) {
//     PathPlanner *pathPlanner = new PathPlanner();

//     string node1Name = "testnode1";
//     string node2Name = "testnode2";
//     string node3Name = "testnode3";
//     string node4Name = "testnode4";
//     string node5Name = "testnode5";
 
//     PathPlannerNode* node1 = new PathPlannerNode(&node1Name,-2.5,3);
//     PathPlannerNode* node2 = new PathPlannerNode(&node2Name,-2.5,-0);
//     PathPlannerNode* node3 = new PathPlannerNode(&node3Name,3,0);
//     PathPlannerNode* node4 = new PathPlannerNode(&node4Name,3,3);
//     PathPlannerNode* node5 = new PathPlannerNode(&node5Name, -2.5, -3);

//     node1->addNeighbour(node2);
//     node1->addNeighbour(node5);

//     node2->addNeighbour(node1);
//     node2->addNeighbour(node3);
//     node2->addNeighbour(node5);

//     node3->addNeighbour(node2);
//     node3->addNeighbour(node4);

//     node4->addNeighbour(node3);

//     node5->addNeighbour(node2);
//     node5->addNeighbour(node1);

//     pathPlanner->addNode(node1);
//     pathPlanner->addNode(node2);
//     pathPlanner->addNode(node3);
//     pathPlanner->addNode(node4);
//     pathPlanner->addNode(node5);

//     //pathPlanner->removeNode(&nodeName);
//     //ASSERT_EQ(pathPlanner->nodes.size(), 0);
//     ASSERT_EQ(0,0);
// }