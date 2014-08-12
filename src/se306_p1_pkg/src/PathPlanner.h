#ifndef SE306P1_PATHPLANNER_H_DEFINED
#define SE306P1_PATHPLANNER_H_DEFINED

#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>

#include <stdint.h>
#include <iostream>
#include <vector>
#include <boost/graph/breadth_first_search.hpp>

using namespace boost;
using namespace std;

class PathPlanner
{
private:
    
public:
	void pathToNode(string,string);
};

class PathPlannerNode{
    private:
        string name;
        bool visited;
    public:
    PathPlannerNode(string inputName){
        this->name = inputName;
    }
    
    vector<PathPlannerNode> neighbours;
    
    string getName(){
        return this->name;
    }
    
    bool isVisited(){
        return visited;
    }
    
    void setVisited(bool newVisited){
        this->visited = newVisited;
    }
};

#endif // #ifndef SE306P1_ACTOR_VISITOR_H_DEFINED

