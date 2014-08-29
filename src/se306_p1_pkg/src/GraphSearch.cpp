#include "GraphSearch.h"
#include <stdlib.h>
#include <queue>
#include <cmath>
#include <algorithm>
#include <iostream>
#include <ros/console.h>


vector< vector<GraphSearch::point> >* GraphSearch::theGraph = NULL;

void GraphSearch::setupNodes()
{

	GraphSearch::theGraph = new vector< vector<point> > ();


	defineNode(-2.5, 3, "nodeBedroomCentre");

    defineNode(-2.5, 0, "nodeHallwayByBedroom");
    defineNode(3.1, 0, "nodeHallwayByLivingRoom");
    defineNode(-2.5, -3, "nodeGuestBedroomCentre");
    defineNode(2.8, 5, "nodeHouseDoor");

    //BATHROOM NODES
    defineNode(-1.5, 2, "nodeShowerUnderHead");
    defineNode(-1.5, 1.1, "nodeInShowerNextToDoor");
    defineNode(-0.5, 1.1, "nodeOutShowerNextToDoor");
    defineNode(1, 0, "nodeBathroomDoorHallway");
    defineNode(1, 1, "nodeBathroomDoorInBathroom");
    defineNode(-1, 3, "nodeMedicationRobotHome");

    //KITCHEN NODES
    defineNode(5.8, 3, "nodeKitchenStove");
    defineNode(5.8, -2, "nodeLivingRoomMidwayPoint");
    defineNode(4, -2, "nodeLivingRoomFeedingPlace");
    defineNode(3.1, -2, "nodeLivingRoomByHallwayDoor");

    //SOCIAL TALKING SPACE IN LIVING ROOM
    defineNode(0, -2, "nodeLivingRoomByCouchHallway");
    defineNode(0, -3.5, "nodeLivingRoomByCouch");

    //BED
    defineNode(-6.33, 3.01, "nodeMasterBed");

    defineEdge("nodeMasterBed", "nodeBedroomCentre");

    defineEdge("nodeBedroomCentre", "nodeMasterBed");


    defineEdge("nodeLivingRoomByCouch", "nodeLivingRoomByCouchHallway");

    defineEdge("nodeLivingRoomByCouchHallway", "nodeLivingRoomByCouch");


    defineEdge("nodeLivingRoomByCouchHallway", "nodeLivingRoomByHallwayDoor");

    defineEdge("nodeLivingRoomByHallwayDoor", "nodeLivingRoomByCouchHallway");


    defineEdge("nodeKitchenStove", "nodeLivingRoomMidwayPoint");

    defineEdge("nodeLivingRoomMidwayPoint", "nodeKitchenStove");


    defineEdge("nodeLivingRoomMidwayPoint", "nodeLivingRoomFeedingPlace");

    defineEdge("nodeLivingRoomFeedingPlace", "nodeLivingRoomMidwayPoint");


    defineEdge("nodeLivingRoomFeedingPlace", "nodeLivingRoomByHallwayDoor");

    defineEdge("nodeLivingRoomByHallwayDoor", "nodeLivingRoomFeedingPlace");


    defineEdge("nodeLivingRoomByHallwayDoor", "nodeHallwayByLivingRoom");

    defineEdge("nodeHallwayByLivingRoom", "nodeLivingRoomByHallwayDoor");


    defineEdge("nodeShowerUnderHead", "nodeInShowerNextToDoor");

    defineEdge("nodeInShowerNextToDoor", "nodeShowerUnderHead");

    defineEdge("nodeInShowerNextToDoor", "nodeOutShowerNextToDoor");

    defineEdge("nodeOutShowerNextToDoor", "nodeInShowerNextToDoor");

    defineEdge("nodeBathroomDoorInBathroom", "nodeOutShowerNextToDoor");

    defineEdge("nodeOutShowerNextToDoor","nodeBathroomDoorInBathroom");

    defineEdge("nodeBathroomDoorInBathroom", "nodeBathroomDoorHallway");
    defineEdge("nodeBathroomDoorHallway", "nodeBathroomDoorInBathroom");

    defineEdge("nodeBathroomDoorInBathroom", "nodeMedicationRobotHome");
    defineEdge("nodeMedicationRobotHome", "nodeBathroomDoorInBathroom");


    defineEdge("nodeBathroomDoorHallway", "nodeHallwayByBedroom");

    defineEdge("nodeHallwayByBedroom", "nodeBathroomDoorHallway");


    defineEdge("nodeBathroomDoorHallway", "nodeHallwayByLivingRoom");

    defineEdge("nodeHallwayByLivingRoom", "nodeBathroomDoorHallway");

    defineEdge("nodeBedroomCentre", "nodeHallwayByBedroom");
    defineEdge("nodeHallwayByBedroom", "nodeBedroomCentre");

    defineEdge("nodeBedroomCentre", "nodeGuestBedroomCentre");

    defineEdge("nodeGuestBedroomCentre", "nodeBedroomCentre");

    defineEdge("nodeHallwayByBedroom", "nodeHallwayByLivingRoom");

    defineEdge("nodeHallwayByLivingRoom", "nodeHallwayByBedroom");

    defineEdge("nodeHallwayByBedroom", "nodeGuestBedroomCentre");

    defineEdge("nodeGuestBedroomCentre", "nodeHallwayByBedroom");

    defineEdge("nodeHallwayByLivingRoom", "nodeHouseDoor");

    defineEdge("nodeHouseDoor", "nodeHallwayByLivingRoom");

}

void GraphSearch::defineNode(double x, double y)
{
	GraphSearch::defineNode(x, y, "");
}

void GraphSearch::defineNode(double x, double y, string name)
{
	point p;
	p.x = x;
	p.y = y;
	p.name = name;
	vector<point> temp;
	temp.push_back(p);
	GraphSearch::theGraph->push_back(temp);
}

void GraphSearch::defineEdge(double x1, double y1, double x2, double y2)
{
	point *p = 0;
	for (int i = 0; i < GraphSearch::theGraph->size(); i++)
	{
    	if (((*GraphSearch::theGraph)[i][0].x == x2) && ((*GraphSearch::theGraph)[i][0].y == y2))
    	{
    		p = &(*GraphSearch::theGraph)[i][0];
    	}
	}

	if (!p) return;

	for (int i = 0; i < GraphSearch::theGraph->size(); i++)
	{
    	if (((*GraphSearch::theGraph)[i][0].x == x1) && ((*GraphSearch::theGraph)[i][0].y == y1))
    	{
    		(*GraphSearch::theGraph)[i].push_back(*p);
    	}
	}
}

void GraphSearch::defineEdge(string name1, string name2)
{
	point *p = getPoint(name2);

	if (!p) return;

	for (int i = 0; i < GraphSearch::theGraph->size(); i++)
	{
    	if (((*GraphSearch::theGraph)[i][0].name.compare(name1) == 0))
    	{
    		(*GraphSearch::theGraph)[i].push_back(*p);
    	}
	}
}

void GraphSearch::defineEdge(string name1, double x, double y)
{
	point *p = getPoint(name1);

	if (!p) return;

	for (int i = 0; i < GraphSearch::theGraph->size(); i++)
	{
    	if (((*GraphSearch::theGraph)[i][0].x == x) && ((*GraphSearch::theGraph)[i][0].y == y))
    	{
    		(*GraphSearch::theGraph)[i].push_back(*p);
    	}
	}
}
vector<GraphSearch::point> GraphSearch::getPath(string name1, string name2)
{
	point *p1 = getPoint(name1);
	point *p2 = getPoint(name2);
	return getPath(p1->x,p1->y,p2->x,p2->y);
}
vector<GraphSearch::point> GraphSearch::getPath(string name1, double x, double y)
{
	point *p1 = getPoint(name1);
	return getPath(p1->x,p1->y,x,y);
}

bool GraphSearch::checkIfInList(point *p, vector<point> *list)
{
	if (!p) return false;

	for (int i = 0; i < list->size(); i++)
	{
		if ((*list)[i].x == p->x && (*list)[i].y == p->y)
		{
			return true;
		}
	}
  return false;
}

void GraphSearch::addPointToSeen(point *p, vector<point> *list)
{
	if(p == NULL)
	{
		return;
	}
	if (!checkIfInList(p, list))
	{
		list->push_back(*p);
	}
}

bool GraphSearch::comparePointer(point *a, point *b)
{
  return a->x == b->x && a->y == b->y;
}

vector<GraphSearch::edge>* GraphSearch::getAdjacentEdges(point *t)
{
  vector<edge>* listEdges = new vector<edge>();
  int i;
  int j;
  for (i = 0; i < GraphSearch::theGraph->size(); i++)
  {
    for(j = 1; j < (*GraphSearch::theGraph)[i].size(); j++)
    {
      point *p1 = &(*GraphSearch::theGraph)[i][0];
      point *p2 = &(*GraphSearch::theGraph)[i][j];

      if (comparePointer(p1, t))
      {
        edge e;
		e.p1 = p1;
		e.p2 = p2;
        listEdges->push_back(e);
      }
    }
  }
  return listEdges;
}

GraphSearch::point* GraphSearch::getAdjacentVertex(point *t, edge *e)
{
  if (comparePointer(e->p1, t))
  {
    return e->p2;
  }
  else if (comparePointer(e->p2, t))
  {
    return e->p1;
  }
  return NULL;
}

vector<GraphSearch::point> GraphSearch::getPath(double x1, double y1, double x2, double y2)
{
	struct backPointer {
		point *p;
		backPointer *previous;
	};

	backPointer *bp = NULL;
	backPointer *prev_bp = NULL;

	bool foundVertex = false;
	queue<point> *Q = new queue<point>();
	vector<point> *V = new vector<point>();

	// Starting point
	point *v = new point; //(point*) malloc(sizeof(point));
	v->x = x1;
	v->y = y1;
	// Finish point
	point *f = new point; //(point*) malloc(sizeof(point));
	f->x = x2;
	f->y = y2;
	// temporary pointer
	point *t;
	addPointToSeen(v, V);
	Q->push(*v);
	while (!Q->empty())
	{
		t = &Q->front();
		ROS_INFO("Now looking at %s",t->name.c_str());
		Q->pop();
		if (comparePointer(t, f))
		{
                  // maintain the backPointer
                    break;
		}
		vector<edge> *E = getAdjacentEdges(t);
		int i;
		for (i = 0; i < E->size(); i++)
		{
			point *u = getAdjacentVertex(t, &(*E)[i]);
			if (!checkIfInList(u, V) && (u != NULL))
			{
				addPointToSeen(u, V);
				u->previous = t;
				Q->push(*u);
			}
		}
	}


	vector<point> path;
	point *next;
	next = t;
	while (next->previous != NULL)
	{
		ROS_INFO_STREAM("foo");
		path.push_back(*next);
		next = next->previous;
		if (comparePointer(next,v)){
			path.push_back(*v);
			break;
		}
	}
	ROS_INFO_STREAM("finished");
	reverse(path.begin(), path.end());
	return path;
}

vector<GraphSearch::point> GraphSearch::getPath(double x, double y, string name1)
{
	point *p1 = getPoint(name1);
	return getPath(x, y, p1->x, p1->y);
}

GraphSearch::point* GraphSearch::getPoint(string name)
{
	int i;
	for (i = 0; i < GraphSearch::theGraph->size(); i++)
	{
    	if (((*GraphSearch::theGraph)[i][0].name.compare(name) == 0))
    	{
    		return &(*GraphSearch::theGraph)[i][0];
    	}
	}
}

GraphSearch::point* GraphSearch::getNewPoint(string name, double x, double y)
{
	point *p1 = new point; //(point*)malloc(sizeof(point));
	p1->x = x;
	p1->y = y;
	p1->name = name;
	return p1;
}

GraphSearch::point* GraphSearch::findClosestPoint(double x, double y)
{
	int i;
	point *best = new point; //(point*)malloc(sizeof(point));
	double bestDist = 100.0;
	for (i = 0; i < GraphSearch::theGraph->size(); i++)
	{
		cout << "findClosestPointLoop ---------------------------------------";
    	double tempx = (*GraphSearch::theGraph)[i][0].x;
    	double tempy = (*GraphSearch::theGraph)[i][0].y;

    	double tempDiff = abs(x - tempx) + abs(y - tempy);

    	if (tempDiff < bestDist)
    	{
    		bestDist = tempDiff;
    		best = &(*GraphSearch::theGraph)[i][0];
    	}


	}


	return best;
}
