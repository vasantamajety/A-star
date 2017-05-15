#include "Planner.hpp"

State getStartState();
State getTargetState();

int main(){

	Map map;
	State start=getStartState();
	State target=getTargetState();

	Planner astar;

	clock_t startTime = clock();
	astar.plan(start, target, map);
	cout << double( clock() - startTime ) / (double)CLOCKS_PER_SEC<< " seconds." << endl;
}

State getStartState()
{
	//to do: read from yml file
	return State(500, 200, 72);
}

State getTargetState()
{
	//to do: read from yml file
	return State(400, 200, 18);
}
