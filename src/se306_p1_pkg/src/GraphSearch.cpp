#include "GraphSearch.h"
#include <stdlib.h>
#include <queue>
#include <cmath>


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
vector<GraphSearch::point>* GraphSearch::getPath(string name1, string name2)
{
	point *p1 = (point*)malloc(sizeof(point));
	p1 = getPoint(name1);
	point *p2 = (point*)malloc(sizeof(point));
	p2 = getPoint(name2);	
	return getPath(p1->x,p1->y,p2->x,p2->y);	
}
vector<GraphSearch::point>* GraphSearch::getPath(string name1, double x, double y)
{
	point *p1 = (point*)malloc(sizeof(point));
	p1 = getPoint(name1);
	return getPath(p1->x,p1->y,x,y);	
}

bool GraphSearch::checkIfInList(point *p, vector<point> *list)
{
  int i;
  for(i = 0; i < list->size(); i++)
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
  for (i = 0; i < theGraph->size(); i++)
  {
    for(j = 1; j < (*theGraph)[i].size(); j++)
    {
      point *p1 = &(*theGraph)[i][0];
      point *p2 = &(*theGraph)[i][j];
      
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
  else// if (comparePointer(e->p2, t))
  {
    return e->p1;
  }
}

vector<GraphSearch::point>* GraphSearch::getPath(double x1, double y1, double x2, double y2)
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
        point *v = (point*) malloc(sizeof(point));
        v->x = x1;
        v->y = y1;
        
        // Finish point
        point *f = (point*) malloc(sizeof(point));
        f->x = x2;
        f->y = y2;
        
        // temporary pointer
        point *t;
        
        addPointToSeen(v, V);
        Q->push(*v);
        
        while (!Q->empty()) {
          t = &Q->front();
          Q->pop();  
          
          if (comparePointer(t, f))
          {
            
            break;
          }
          
          vector<edge> *E = getAdjacentEdges(t);
          int i;
          for (i = 0; i < E->size(); i++)
          {
            point *u = getAdjacentVertex(t, &(*E)[i]);
            if (checkIfInList(u, V))
            {
              addPointToSeen(u, V);
              Q->push(*u);
              // maintain the backPointer
              if (bp != NULL) {
                prev_bp = bp;
              }
              
              bp = (backPointer*) malloc(sizeof(backPointer));
              bp->p = u;
              bp->previous = prev_bp;
            }          
          }
        }
        
        // Build path
        vector<point> *path = new vector<point>();
        while (bp->previous != NULL)
        {
          path->push_back(*bp->p);
          bp = bp->previous;
        }
        return path;
}

vector<GraphSearch::point>* GraphSearch::getPath(double x, double y, string name1)
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

GraphSearch::point* GraphSearch::findClosestPoint(double x, double y)
{
	int i;
	point *best = (point*)malloc(sizeof(point));
	double bestDist = 100.0;
	for (i = 0; i < theGraph->size(); i++)
	{
    	double tempx = (*theGraph)[i][0].x;
    	double tempy = (*theGraph)[i][0].y;

    	double tempDiff = abs(x - tempx) + abs(y - tempy);

    	if (tempDiff < bestDist)
    	{
    		bestDist = tempDiff;
    		best = &(*theGraph)[i][0];
    	}

    		
	}

	return best;
}