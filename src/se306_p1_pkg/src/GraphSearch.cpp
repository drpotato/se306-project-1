#include "GraphSearch.h"
#include <stdlib.h>

GraphSearch::GraphSearch()
{
	theGraph = new vector< vector<point> > ();
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

}
vector<GraphSearch::point> GraphSearch::getPath(string name1, double x, double y)
{
	
}
vector<GraphSearch::point> GraphSearch::getPath(double x1, double y1, double x2, double y2)
{
	
}
vector<GraphSearch::point> GraphSearch::getPath(double x, double y, string name1)
{
	
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