// A hack so that I can access private members
#define private public
#define protected public

// Bring in my package's API, which is what I'm testing
#include "../src/PathPlannerNode.h"

// Bring in gtest
#include <gtest/gtest.h>

// Declare a test
TEST(TestSuite, test_path_planner_node_contructor) {
    PathPlannerNode *pathPlannerNode = new PathPlannerNode("test_name", 0, 0);
    ASSERT_EQ(pathPlannerNode->name, "test_name");
    ASSERT_EQ(pathPlannerNode->px, 0);
    ASSERT_EQ(pathPlannerNode->py, 0);
}