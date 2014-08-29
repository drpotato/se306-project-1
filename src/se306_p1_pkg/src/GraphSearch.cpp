#include "GraphSearch.h"
#include <stdlib.h>
#include <queue>
#include <cmath>
#include <algorithm>
#include <iostream>


vector< vector<GraphSearch::point> >* GraphSearch::theGraph = NULL;

void GraphSearch::setupNodes()
{  

	GraphSearch::theGraph = new vector< vector<point> > ();


	defineNode(-2.5, 3, "nodeBedroomCentre");

    defineNode(-2.5, -0, "nodeHallwayByBedroom");
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
    cout << "a\n";
    defineEdge("nodeMasterBed", "nodeGuestBedroomCentre");
    cout << "b\n";
    defineEdge("nodeGuestBedroomCentre", "nodeMasterBed");

cout << "c\n";
    defineEdge("nodeLivingRoomByCouch", "nodeLivingRoomByCouchHallway");
    cout << "d\n";
    defineEdge("nodeLivingRoomByCouchHallway", "nodeLivingRoomByCouch");
cout << "e\n";

    defineEdge("nodeLivingRoomByCouchHallway", "nodeLivingRoomByHallwayDoor");
    cout << "f\n";
    defineEdge("nodeLivingRoomByHallwayDoor", "nodeLivingRoomByCouchHallway");
    cout << "g\n";

    defineEdge("nodeKitchenStove", "nodeLivingRoomMidwayPoint");
    cout << "h\n";
    defineEdge("nodeLivingRoomMidwayPoint", "nodeKitchenStove");
    cout << "i\n";

    defineEdge("nodeLivingRoomMidwayPoint", "nodeLivingRoomFeedingPlace");
    cout << "j\n";
    defineEdge("nodeLivingRoomFeedingPlace", "nodeLivingRoomMidwayPoint");
    cout << "k\n";

    defineEdge("nodeLivingRoomFeedingPlace", "nodeLivingRoomByHallwayDoor");
    cout << "l\n";
    defineEdge("nodeLivingRoomByHallwayDoor", "nodeLivingRoomFeedingPlace");
    cout << "m\n";

    defineEdge("nodeLivingRoomByHallwayDoor", "nodeHallwayByLivingRoom");
    cout << "n\n";
    defineEdge("nodeHallwayByLivingRoom", "nodeLivingRoomByHallwayDoor");
    cout << "o\n";

    defineEdge("nodeShowerUnderHead", "nodeInShowerNextToDoor");
    cout << "p\n";
    defineEdge("nodeInShowerNextToDoor", "nodeShowerUnderHead");
    cout << "q\n";

    defineEdge("nodeInShowerNextToDoor", "nodeOutShowerNextToDoor");
    cout << "r\n";
    defineEdge("nodeOutShowerNextToDoor", "nodeInShowerNextToDoor");
cout << "s\n";
    defineEdge("nodeBathroomDoorInBathroom", "nodeOutShowerNextToDoor");
    cout << "t\n";
    defineEdge("nodeOutShowerNextToDoor","nodeBathroomDoorInBathroom");
cout << "u\n";
    defineEdge("nodeBathroomDoorInBathroom", "nodeBathroomDoorHallway");
    cout << "v\n";
    defineEdge("nodeBathroomDoorHallway", "nodeBathroomDoorInBathroom");

    defineEdge("nodeBathroomDoorInBathroom", "nodeMedicationRobotHome");
    defineEdge("nodeMedicationRobotHome", "nodeBathroomDoorInBathroom");

cout << "w\n";
    defineEdge("nodeBathroomDoorHallway", "nodeHallwayByBedroom");
    cout << "x\n";
    defineEdge("nodeHallwayByBedroom", "nodeBathroomDoorHallway");
    cout << "y\n";

    defineEdge("nodeBathroomDoorHallway", "nodeHallwayByLivingRoom");
    cout << "z\n";
    defineEdge("nodeHallwayByLivingRoom", "nodeBathroomDoorHallway");
cout << "aa\n";
    defineEdge("nodeBedroomCentre", "nodeHallwayByBedroom");
    cout << "bb\n";
    defineEdge("nodeHallwayByBedroom", "nodeBedroomCentre");
cout << "cc\n";
    defineEdge("nodeBedroomCentre", "nodeGuestBedroomCentre");
    cout << "dd\n";
    defineEdge("nodeGuestBedroomCentre", "nodeBedroomCentre");
cout << "ee\n";
    defineEdge("nodeHallwayByBedroom", "nodeHallwayByLivingRoom");
    cout << "ff\n";
    defineEdge("nodeHallwayByLivingRoom", "nodeHallwayByBedroom");
cout << "gg\n";
    defineEdge("nodeHallwayByBedroom", "nodeGuestBedroomCentre");
    cout << "hh\n";
    defineEdge("nodeGuestBedroomCentre", "nodeHallwayByBedroom");
cout << "ii\n";
    defineEdge("nodeHallwayByLivingRoom", "nodeHouseDoor");
    cout << "jj\n";
    defineEdge("nodeHouseDoor", "nodeHallwayByLivingRoom");  
cout << "kk\n";
}

void GraphSearch::defineNode(double x, double y)
{
	point *p = new point; //(point*)malloc(sizeof(point));
	p->x = x;
	p->y = y;
	p->name = "";
	vector<point> *temp = new vector<point>();
	temp->push_back(*p);
	GraphSearch::theGraph->push_back(*temp);
}
void GraphSearch::defineNode(double x, double y, string name)
{
	point *p = new point; //(point*)malloc(sizeof(point));
	p->x = x;
	p->y = y;
	p->name = name;
	vector<point> *temp = new vector<point>();
	temp->push_back(*p);
	GraphSearch::theGraph->push_back(*temp);
}
void GraphSearch::defineEdge(double x1, double y1, double x2, double y2)
{
	point *p = new point; //(point*)malloc(sizeof(point));
	int i;
	for (i = 0; i < GraphSearch::theGraph->size(); i++)
	{
    	if (((*GraphSearch::theGraph)[i][0].x == x2) && ((*GraphSearch::theGraph)[i][0].y == y2))
    	{
    		p = &(*GraphSearch::theGraph)[i][0];
    	}		
	}
	for (i = 0; i < GraphSearch::theGraph->size(); i++)
	{
    	if (((*GraphSearch::theGraph)[i][0].x == x1) && ((*GraphSearch::theGraph)[i][0].y == y1))
    	{
    		(*GraphSearch::theGraph)[i].push_back(*p);
    	}		
	}
}
void GraphSearch::defineEdge(string name1, string name2)
{
	point *p = new point; //(point*)malloc(sizeof(point));
	p = getPoint(name2);
	int i;
	for (i = 0; i < GraphSearch::theGraph->size(); i++)
	{
    	if (((*GraphSearch::theGraph)[i][0].name.compare(name1) == 0))
    	{
    		(*GraphSearch::theGraph)[i].push_back(*p);
    	}		
	}
}
void GraphSearch::defineEdge(string name1, double x, double y)
{
	point *p = new point; //(point*)malloc(sizeof(point));
	p = getPoint(name1);
	int i;
	for (i = 0; i < GraphSearch::theGraph->size(); i++)
	{
    	if (((*GraphSearch::theGraph)[i][0].x == x) && ((*GraphSearch::theGraph)[i][0].y == y))
    	{
    		(*GraphSearch::theGraph)[i].push_back(*p);
    	}		
	}
}
vector<GraphSearch::point>* GraphSearch::getPath(string name1, string name2)
{
	point *p1 = new point; //(point*)malloc(sizeof(point));
	p1 = getPoint(name1);
	point *p2 = new point; //(point*)malloc(sizeof(point));
	p2 = getPoint(name2);	
	return getPath(p1->x,p1->y,p2->x,p2->y);	
}
vector<GraphSearch::point>* GraphSearch::getPath(string name1, double x, double y)
{
	point *p1 = new point; //(point*)malloc(sizeof(point));
	p1 = getPoint(name1);
	return getPath(p1->x,p1->y,x,y);	
}

bool GraphSearch::checkIfInList(point *p, vector<point> *list)
{
	cout << "a\n";
	if (p == NULL){
		cout << "b\n";
		return false;
		cout << "c\n";
	}
	cout << "d\n";
  int i;
  cout << "e\n";
  for(i = 0; i < list->size(); i++)
  {
  	cout << "f\n";
    if ((*list)[i].x == p->x && (*list)[i].y == p->y)
    {
    	cout << "g\n";
      return true;
    }
    cout << "h\n";
  }
  return false;
}

void GraphSearch::addPointToSeen(point *p, vector<point> *list)
{
	if(p == NULL){
		return;
	}
  if (!checkIfInList(p, list)) {
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
      
      if (comparePointer(p1, t) || comparePointer(p2, t))
      {
        edge *e = (edge*) malloc(sizeof(edge));
        listEdges->push_back(*e);
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

vector<GraphSearch::point>* GraphSearch::getPath(double x1, double y1, double x2, double y2)
{
	cout << "1\n";
  struct backPointer {
    point *p;
    backPointer *previous;
  };
  cout << "3\n";
  cout << "2\n";
  cout << "4\n";
  
  backPointer *bp = NULL;
  cout << "5\n";
  backPointer *prev_bp = NULL;
  cout << "6\n";
  
  bool foundVertex = false;
  cout << "7\n";
	queue<point> *Q = new queue<point>();
	cout << "8\n";
	vector<point> *V = new vector<point>();
        cout << "9\n";
        // Starting point
        point *v = new point; //(point*) malloc(sizeof(point));
        cout << "10\n";
        v->x = x1;
        cout << "11\n";
        v->y = y1;
        cout << "12\n";
        // Finish point
        point *f = new point; //(point*) malloc(sizeof(point));
        cout << "13\n";
        f->x = x2;
        cout << "14\n";
        f->y = y2;
        cout << "15\n";
        // temporary pointer
        point *t;
        cout << "16\n";
        addPointToSeen(v, V);
        cout << "17\n";
        Q->push(*v);
        cout << "18\n";
        while (!Q->empty()) {
        	cout << "19\n";
          t = &Q->front();
          cout << "20\n";
          Q->pop();  
          cout << "21\n";
          if (comparePointer(t, f))
          {
            cout << "22\n";
            break;
          }
          cout << "23\n";
          vector<edge> *E = getAdjacentEdges(t);
          cout << "24\n";
          int i;
          cout << "25\n";
          for (i = 0; i < E->size(); i++)
          {
          	cout << "26\n";
            point *u = getAdjacentVertex(t, &(*E)[i]);
            cout << "27\n";
            if (!checkIfInList(u, V) && (u != NULL))
            {
            	cout << "28\n";
              addPointToSeen(u, V);
              cout << "29\n";
              Q->push(*u);
              cout << "30\n";
              // maintain the backPointer
              if (bp != NULL) {
              	cout << "301\n";
                prev_bp = bp;
              } else {
              	cout << "3012\n";
              	prev_bp == NULL;
              }
              cout << "302\n";
              bp = new backPointer; //(backPointer*) malloc(sizeof(backPointer));
              cout << "303\n";
              bp->p = u;
              cout << "304\n";
              bp->previous = prev_bp;
            }          
          }
        }
        cout << "31\n";
        // Build path
        vector<point> *path = new vector<point>();
        cout << "32\n";
        while (bp->previous != NULL)
        {
        	cout << "33\n";
          path->push_back(*bp->p);
          cout << "34\n";
          bp = bp->previous;
        }
        cout << "35\n";
        reverse(path->begin(), path->end());
        cout << "36\n";
        return path;
}

vector<GraphSearch::point>* GraphSearch::getPath(double x, double y, string name1)
{
	point *p1 = new point; //(point*)malloc(sizeof(point));
	p1 = getPoint(name1);
	return getPath(x,y,p1->x,p1->y);
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