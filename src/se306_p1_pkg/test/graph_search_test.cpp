// A hack so that I can access private members
#define private public
#define protected public

// Bring in my package's API, which is what I'm testing
#include "../src/GraphSearch.h"

// Bring in gtest
#include <gtest/gtest.h>

// ===== Testing involving definition-type methods
// Test defineNode(double, double)
TEST(GraphSearchTestSuite, test_graphsearch_findclosest_nodeHallwayByLivingRoom)
{
  GraphSearch &gs = GraphSearch::getInstance();
  GraphSearch::point *p0 = gs.findClosestPoint(3.1, 0.1);
  
  // Wrap these in std::strings so that ASSERT_EQ can do a proper comparison.
  ASSERT_EQ(std::string(p0->name), std::string("nodeHallwayByLivingRoom"));
}






// ===== Testing involving HALLWAY nodes
// Test that closest node to 3.1, 0.1 is 'nodeHallwayByLivingRoom"
TEST(GraphSearchTestSuite, test_graphsearch_findclosest_nodeHallwayByLivingRoom)
{
  GraphSearch &gs = GraphSearch::getInstance();
  GraphSearch::point *p0 = gs.findClosestPoint(3.1, 0.1);
  
  // Wrap these in std::strings so that ASSERT_EQ can do a proper comparison.
  ASSERT_EQ(std::string(p0->name), std::string("nodeHallwayByLivingRoom"));
}

// Test that closest node to 2, 5 is 'nodeHouseDoor"
TEST(GraphSearchTestSuite, test_graphsearch_findclosest_nodeHouseDoor)
{
  GraphSearch &gs = GraphSearch::getInstance();
  GraphSearch::point *p0 = gs.findClosestPoint(2, 5);
  
  // Wrap these in std::strings so that ASSERT_EQ can do a proper comparison.
  ASSERT_EQ(std::string(p0->name), std::string("nodeHouseDoor"));
}

// Test that closest node to -2, -1 'nodeHallwayByBedroom"
TEST(GraphSearchTestSuite, test_graphsearch_findclosest_nodeHallwayByBedroom)
{
  GraphSearch &gs = GraphSearch::getInstance();
  GraphSearch::point *p0 = gs.findClosestPoint(-2, -1);
  
  // Wrap these in std::strings so that ASSERT_EQ can do a proper comparison.
  ASSERT_EQ(std::string(p0->name), std::string("nodeHallwayByBedroom"));
}



// ===== Testing involving BATHROOM nodes
// Test that closest node to 1, 1.2 'nodeBathroomDoorInBathroom"
TEST(GraphSearchTestSuite, test_graphsearch_findclosest_nodeBathroomDoorInBathroom)
{
  GraphSearch &gs = GraphSearch::getInstance();
  GraphSearch::point *p0 = gs.findClosestPoint(1, 1.2);
  
  // Wrap these in std::strings so that ASSERT_EQ can do a proper comparison.
  ASSERT_EQ(std::string(p0->name), std::string("nodeBathroomDoorInBathroom"));
}



// ===== Testing involving KITCHEN nodes
// Test that closest node to 5.9, 3.2 'nodeKitchenStove"
TEST(GraphSearchTestSuite, test_graphsearch_findclosest_nodeKitchenStove)
{
  GraphSearch &gs = GraphSearch::getInstance();
  GraphSearch::point *p0 = gs.findClosestPoint(5.9, 3.2);
  
  // Wrap these in std::strings so that ASSERT_EQ can do a proper comparison.
  ASSERT_EQ(std::string(p0->name), std::string("nodeKitchenStove"));
}



// ===== Testing involving SOCIAL SPACE (LIVING ROOM) nodes
// Test that closest node to 4, -2.5 "nodeLivingRoomFeedingPlace"
TEST(GraphSearchTestSuite, test_graphsearch_findclosest_nodeLivingRoomFeedingPlace)
{
  GraphSearch &gs = GraphSearch::getInstance();
  GraphSearch::point *p0 = gs.findClosestPoint(4, -2.5);
  
  // Wrap these in std::strings so that ASSERT_EQ can do a proper comparison.
  ASSERT_EQ(std::string(p0->name), std::string("nodeLivingRoomFeedingPlace"));
}



// ===== Testing involving BED nodes
// Test that closest node to -6, 2.75 "nodeMasterBed"
TEST(GraphSearchTestSuite, test_graphsearch_findclosest_nodeMasterBed)
{
  GraphSearch &gs = GraphSearch::getInstance();
  GraphSearch::point *p0 = gs.findClosestPoint(-6, 2.75);
  
  // Wrap these in std::strings so that ASSERT_EQ can do a proper comparison.
  ASSERT_EQ(std::string(p0->name), std::string("nodeMasterBed"));
}










// Test path size is set correctly
TEST(GraphSearchTestSuite, test_graphsearch_pathfinder)
{
  GraphSearch &gs = GraphSearch::getInstance();
  vector<GraphSearch::point>*path = gs.getPath(3.1, 0.1, 1.1, 1.1);
  
  // Start --> bedroom --> hallway-outside-bedroom --> hallway-outside-bathroom --> bathroom --> end
  // 5 destinations in total.
  ASSERT_EQ(path->size(), 5);
}
