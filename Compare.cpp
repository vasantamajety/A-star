#include "Compare.hpp"

State Compare::target;
int** Compare::obs_map;
float** Compare::shortest_2d;
int** Compare::grid_obs_map;

bool Compare::operator() (const State s1, const State s2){
	//to do: replace by heuristic+cost comparison
	return s1.cost3d+holonomic_with_obs(s1)+0.1*non_holonomic_without_obs(s1)>s2.cost3d+holonomic_with_obs(s2)+0.1*non_holonomic_without_obs(s2);
}

typedef bool (*compare2dSignature)(State, State);
bool compare2d(State a, State b)
{
   //return a.cost2d>b.cost2d;	//simple dijkstra
   return a.cost2d+abs(Compare::target.dx-a.dx)+abs(Compare::target.dy-a.dy)>b.cost2d+abs(Compare::target.dx-b.dx)+abs(Compare::target.dy-b.dy);
}

//currently uses dijkstra's algorithm in x-y space
float Compare::holonomic_with_obs(State src){

	return Compare::shortest_2d[(int)src.x*DX/MAPX][(int)src.y*DY/MAPY];
}

void Compare::runDijkstra(){

	Compare::target.dx=Compare::target.gx*DX/GX;
	Compare::target.dy=Compare::target.gy*DY/GY;
	State src=Compare::target;
	priority_queue<State, vector<State>, compare2dSignature> frontier(&compare2d);
	int vis[DX][DY];

	float** cost=new float*[DX];
	for(int i=0;i<DX;i++)
	{
		cost[i]=new float[DY];
		for(int j=0;j<DY;j++)
			cost[i][j]=10000;
	}

	memset(vis, 0, sizeof(int) * DX * DY);

	// for(int i=0;i<DX;i++)
	// 	for(int j=0;j<DY;j++)
	// 		cost[i][j]=10000;
	cost[src.dx][src.dy]=0;

	frontier.push(src);
	while(!frontier.empty()){
		State current=frontier.top();
		frontier.pop();

		int x=current.dx;
		int y=current.dy;

		if(vis[x][y])
			continue;

		vis[x][y]=1;

		for(int i=-1;i<=1;i++)
			for(int j=-1;j<=1;j++)
			{
				if(x+i<0 || x+i>=DX || y+j<0 || y+j>=DY)
					continue;
				if((i==0 && j==0) || Compare::grid_obs_map[x+i][y+j]!=0)
					continue;

				if(cost[x+i][y+j]>cost[x][y]+sqrt(i*i+j*j))
				{
					cost[x+i][y+j]=cost[x][y]+sqrt(i*i+j*j);
					State tempstate;
					tempstate.dx=current.dx+i;
					tempstate.dy=current.dy+j;
					tempstate.cost2d=cost[x+i][y+j];
					frontier.push(tempstate);
				}
			}
	}
	Compare::shortest_2d=cost;

	// Mat dist(240, 240, CV_8UC3, Scalar(255, 255, 255));
	// for(int i=0;i<240;i++)
	// 	for(int j=0;j<240;j++)
	// 	{
	// 		dist.at<Vec3b>(j,i)={255-0.6*shortest_2d[i][j], 200-0.6*shortest_2d[i][j], 200-0.6*shortest_2d[i][j]};
	// 	}
	// resize(dist, dist, Size(400, 400));
	//uncomment to check if dijkstra ran properly
	//imshow("dist", dist);
	//waitKey(0);
}

float Compare::non_holonomic_without_obs(State src){
	return 0;
	return abs(Compare::target.x-src.x)+abs(Compare::target.y-src.y)+abs(Compare::target.theta-src.theta);
}
