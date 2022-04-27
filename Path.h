#include <Arduino.h>
#define MAX_HELPNODES 5 //update according to map
struct Coord{
  byte x; //x value of a coordinate	
  byte y; //y value of a coordinate
};

struct Waypoint{
  String wpName; //name of waypoint
  byte x; //x value of a waypoint
  byte y; //y value of a waypoint
};

class Path{
  public:
    Waypoint wp1; //start waypoint
    Waypoint wp2; //end waypoint
    Coord helpnodes[MAX_HELPNODES]; //array of helpnodes that make the path wp1->wp2
    Coord helpnodesR[MAX_HELPNODES]; //array of helpnodes in reversed order wp2->wp1
    
	//constructor
    Path(struct Waypoint wp1, struct Waypoint wp2, struct Coord helpnodesIn[MAX_HELPNODES]);
	Path();//empty constructor for object definition/memory allocation
};