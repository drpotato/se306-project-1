// A hack so that I can access private members
#define private public
#define protected public

// Bring in my package's API, which is what I'm testing
#include "../src/GraphSearch.h"

// Bring in gtest
#include <gtest/gtest.h>

// Declare a test
TEST(GraphSearchTestSuite, test_graphsearch_singleton)
{
  GraphSearch &gs0 = GraphSearch::getInstance();
  GraphSearch &gs1 = GraphSearch::getInstance();
  
  ASSERT_EQ(&gs0, &gs1);
}

TEST(GraphSearchTestSuite, test_graphsearch_findclosest)
{
  GraphSearch &gs = GraphSearch::getInstance();
  GraphSearch::point *p0 = gs.findClosestPoint(3.1, 0.1);
  
  // Wrap these in std::strings so that ASSERT_EQ can do a proper comparison.
  ASSERT_EQ(std::string(p0->name), std::string("nodeHallwayByLivingRoom"));
}

TEST(GraphSearchTestSuite, test_graphsearch_pathfinder)
{
  GraphSearch &gs = GraphSearch::getInstance();
  vector<GraphSearch::point>*path = gs.getPath(3.1, 0.1, 1.1, 1.1);
  
  // Start --> bedroom --> hallway-outside-bedroom --> hallway-outside-bathroom --> bathroom --> end
  // 5 destinations in total.
  ASSERT_EQ(path->size(), 5);
}