#include <vector>
#include <string>
#include <map>

using namespace std;

class GraphSearch
{
public:
	
	~GraphSearch();

	static GraphSearch &getInstance();


	struct point {
		double x;
		double y;
		string name;
	};


	void defineNode(double x, double y);
	void defineNode(double x, double y, string name);
	void defineEdge(double x1, double y1, double x2, double y2);
	void defineEdge(string name1, string name2);
	void defineEdge(string name1, double x, double y);
	vector<point> getPath(string name1, string name2);
	vector<point> getPath(string name1, double x, double y);
	vector<point> getPath(double x1, double y1, double x2, double y2);
	vector<point> getPath(double x, double y, string name1);

	point* getPoint(string name);

	vector< vector<point> > *theGraph;

private:
  void addPointToSeen(point *p, vector<point> *list);
  bool checkIfInList(point *p, vector<point> *list);
  GraphSearch();
};
