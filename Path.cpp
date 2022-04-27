#include "Path.h"
Path::Path(struct Waypoint wp1, struct Waypoint wp2, struct Coord helpnodesIn[MAX_HELPNODES]){
  this->wp1 = wp1; //starting waypoint
  this->wp2 = wp2; //ending waypoint
  for(int i = 0; i<MAX_HELPNODES; i++){
	helpnodes[i] = helpnodesIn[i]; //helpnodes from wp1->wp2
	helpnodesR[i] = helpnodesIn[-i+(MAX_HELPNODES-1)]; //helpnodes from wp2->wp1
  }
};
//empty constructor
Path::Path(){};