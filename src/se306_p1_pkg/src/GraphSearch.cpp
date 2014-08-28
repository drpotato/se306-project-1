#include "GraphSearch.h"
#include <stdlib.h>
#include <queue>


//ActorController should act as a Singleton.
GraphSearch &GraphSearch::getInstance()
{
  static GraphSearch *graphSearchInstance;

  if (!graphSearchInstance)
  {
    graphSearchInstance = new GraphSearch();
  }

  return *graphSearchInstance;
}

GraphSearch::GraphSearch()
{
	theGraph = new vector< vector<point> > ();
	defineNode(-2.5, 3, "nodeBedroomCentre");
    defineNode(-2.5, -0, "nodeHallwayByBedroom");
    defineNode(3, 0, "nodeHallwayByLivingRoom");
    defineNode(-2.5, -3, "nodeGuestBedroomCentre");
    defineNode(2.8, 5, "nodeHouseDoor");

    defineEdge("nodeBedroomCentre", "nodeHallwayBedroom");
    defineEdge("nodeHallwayBedroom", "nodeBedroomCentre");

    defineEdge("nodeBedroomCentre", "nodeGuestBedroomCentre");
    defineEdge("nodeGuestBedroomCentre", "nodeBedroomCentre");

    defineEdge("nodeHallwayBedroom", "nodeHallwayByLivingRoom");
    defineEdge("nodeHallwayByLivingRoom", "nodeHallwayBedroom");

    defineEdge("nodeHallwayBedroom", "nodeGuestBedroomCentre");
    defineEdge("nodeGuestBedroomCentre", "nodeHallwayBedroom");

    defineEdge("nodeHallwayByLivingRoom", "nodeHouseDoor");
    defineEdge("nodeHouseDoor", "nodeHallwayByLivingRoom");
}

GraphSearch::~GraphSearch()
{
	delete theGraph;
}

void GraphSearch::defineNode(double x, double y)
{
	point *p = (point*)malloc(sizeof(point));
	p->x = x;
	p->y = y;
	p->name = "";
	vector<point> *temp = new vector<point>();
	temp->push_back(*p);
	theGraph->push_back(*temp);
}
void GraphSearch::defineNode(double x, double y, string name)
{
	point *p = (point*)malloc(sizeof(point));
	p->x = x;
	p->y = y;
	p->name = name;
	vector<point> *temp = new vector<point>();
	temp->push_back(*p);
	theGraph->push_back(*temp);
}
void GraphSearch::defineEdge(double x1, double y1, double x2, double y2)
{
	point *p = (point*)malloc(sizeof(point));
	int i;
	for (i = 0; i < theGraph->size(); i++)
	{
    	if (((*theGraph)[i][0].x == x2) && ((*theGraph)[i][0].y == y2))
    	{
    		p = &(*theGraph)[i][0];
    	}		
	}
	for (i = 0; i < theGraph->size(); i++)
	{
    	if (((*theGraph)[i][0].x == x1) && ((*theGraph)[i][0].y == y1))
    	{
    		(*theGraph)[i].push_back(*p);
    	}		
	}
}
void GraphSearch::defineEdge(string name1, string name2)
{
	point *p = (point*)malloc(sizeof(point));
	p = getPoint(name2);
	int i;
	for (i = 0; i < theGraph->size(); i++)
	{
    	if (((*theGraph)[i][0].name == name1))
    	{
    		(*theGraph)[i].push_back(*p);
    	}		
	}
}
void GraphSearch::defineEdge(string name1, double x, double y)
{
	point *p = (point*)malloc(sizeof(point));
	p = getPoint(name1);
	int i;
	for (i = 0; i < theGraph->size(); i++)
	{
    	if (((*theGraph)[i][0].x == x) && ((*theGraph)[i][0].y == y))
    	{
    		(*theGraph)[i].push_back(*p);
    	}		
	}
}
vector<GraphSearch::point> GraphSearch::getPath(string name1, string name2)
{
	point *p1 = (point*)malloc(sizeof(point));
	p1 = getPoint(name1);
	point *p2 = (point*)malloc(sizeof(point));
	p2 = getPoint(name2);	
	return getPath(p1->x,p1->y,p2->x,p2->y);	
}
vector<GraphSearch::point> GraphSearch::getPath(string name1, double x, double y)
{
	point *p1 = (point*)malloc(sizeof(point));
	p1 = getPoint(name1);
	return getPath(p1->x,p1->y,x,y);	
}
vector<GraphSearch::point> GraphSearch::getPath(double x1, double y1, double x2, double y2)
{

}
vector<GraphSearch::point> GraphSearch::getPath(double x, double y, string name1)
{
	point *p1 = (point*)malloc(sizeof(point));
	p1 = getPoint(name1);
	return getPath(x,y,p1->x,p1->y);
}

GraphSearch::point* GraphSearch::getPoint(string name)
{
	int i;
	for (i = 0; i < theGraph->size(); i++)
	{
    	if (((*theGraph)[i][0].name == name))
    	{
    		return &(*theGraph)[i][0];
    	}		
	}
}

GraphSearch::point* GraphSearch::getNewPoint(string name, double x, double y)
{
	point *p1 = (point*)malloc(sizeof(point));
	p1->x = x;
	p1->y = y;
	p1->name = name;
	return p1;
}