#ifndef SE306P1_PATHPLANNERNODE_H_DEFINED
#define SE306P1_PATHPLANNERNODE_H_DEFINED

#include <vector>
#include <string>
using namespace std;

class PathPlannerNode{
    friend class PathPlanner;
    private:
        string* name;
        bool visited;
        //position
    double px;
    double py;
    vector<PathPlannerNode*> neighbours;
    public:
    PathPlannerNode(string*,double,double);
    void addNeighbour(PathPlannerNode*);
    void removeNeighbour(PathPlannerNode*);
    string* getName();
    bool isVisited();
    void setVisited(bool newVisited);
};

#endif // #ifndef SE306P1_ACTOR_VISITOR_H_DEFINED

