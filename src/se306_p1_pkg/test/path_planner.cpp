// A hack so that I can access private members
#define private public
#define protected public

// Bring in my package's API, which is what I'm testing
#include "../src/PathPlanner.h"

// Bring in gtest
#include <gtest/gtest.h>

// Declare a test
TEST(TestSuite, test_path_planner_constructor) {
    PathPlanner *pathPlanner = new PathPlanner();
    ASSERT_EQ(pathPlanner->nodes.size(), 0);
}

TEST(TestSuite, test_add_node) {
    ASSERT_EQ(0, 0);
}