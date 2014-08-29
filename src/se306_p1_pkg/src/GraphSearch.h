#include <vector>
#include <string>
#include <map>

#ifndef SE306P1_GRAPHSEARCH_H_DEFINED
#define SE306P1_GRAPHSEARCH_H_DEFINED

using namespace std;

class GraphSearch
{
public:
  static void setupNodes();

  struct point {
          double x;
          double y;
          string name;
          point *previous;
  };

  struct edge {
    point *p1;
    point *p2;
  };

  point* nodeBedroomCentre;
  point* nodeHallwayByBedroom;
  point* nodeHallwayByLivingRoom;
  point* nodeGuestBedroomCentre;
  point* nodeHouseDoor;


  static void defineNode(double x, double y);
  static void defineNode(double x, double y, string name);
  static void defineEdge(double x1, double y1, double x2, double y2);
  static void defineEdge(string name1, string name2);
  static void defineEdge(string name1, double x, double y);
  static vector<point> getPath(string name1, string name2);
  static vector<point> getPath(string name1, double x, double y);
  static vector<point> getPath(double x1, double y1, double x2, double y2);
  static vector<point> getPath(double x, double y, string name1);
  static point* findClosestPoint(double x, double y);

  static point* getNewPoint(string name, double x, double y);
  static point* getPoint(string name);

  static vector< vector<point> > *theGraph;

private:
  static void addPointToSeen(point *p, vector<point> *list);
  static bool checkIfInList(point *p, vector<point> *list);
  static bool comparePointer(point *a, point *b);
  static vector<edge>* getAdjacentEdges(point *t);
  static point* getAdjacentVertex(point *t, edge *e);
};

#endif
