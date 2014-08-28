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
	//JAMIE GET THIS WORKING
	int i;
	for (i = 0; i < theGraph->size(); i++)
	{
    	if ((theGraph[i].front().x == x1) && (theGraph[i].front().y == y1))
    	{
    		point *p = (point*)malloc(sizeof(point));
			p->x = x2;
			p->y = y2;
			p->name = "";
    		theGraph[i].front()->push_back(*p);
    	}		
	}
}
void GraphSearch::defineEdge(string name1, string name2)
{
	
}
void GraphSearch::defineEdge(string name1, double x, double y)
{

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