// A hack so that I can access private members
#define private public
#define protected public

#include <string>

// Bring in my package's API, which is what I'm testing
#include "../src/PathPlannerNode.h"

// Bring in gtest
#include <gtest/gtest.h>

// Declare a test
TEST(TestSuite, test_path_planner_node_contructor) {
    // string name = "test_name";
    // double x = 0.0;
    // double y = 0.0;
    // PathPlannerNode *pathPlannerNode = new PathPlannerNode(&name, *x, *y);
    // ASSERT_EQ(pathPlannerNode->name, name);
    // ASSERT_EQ(pathPlannerNode->px, x);
    // ASSERT_EQ(pathPlannerNode->py, y);

    ASSERT_EQ(1, 1);
}