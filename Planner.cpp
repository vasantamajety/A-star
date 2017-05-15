#include "Planner.hpp"


State previous[GX][GY][Theta];

void Planner::plan(State start, State target, Map map){

	//initialize variables for the Compare class
	Compare::target=target;
	Compare::obs_map=map.obs_map;
	Compare::grid_obs_map=new int*[DX];
	for(int i=0;i<DX;i++)
	{
		Compare::grid_obs_map[i]=new int[DY];
		for(int j=0;j<DY;j++)
			Compare::grid_obs_map[i][j]=0;
	}
	for(int i=0;i<MAPX;i++)
		for(int j=0;j<MAPY;j++)
		{
			if(Compare::obs_map[i][j])
				Compare::grid_obs_map[i*DX/MAPX][j*DY/MAPY]=1;
		}
	Compare cmp;
	cmp.runDijkstra();


	map.initCollisionChecker();
	map.find_near_obs();

	priority_queue<State, vector<State>, Compare> pq;
	start.cost3d=0;
	pq.push(start);

	GUI display(800, 800);
	display.drawObs(map);
	display.drawCar(start);
	display.drawCar(target);

	int vis[GX][GY][Theta];
	memset(vis, 0, sizeof(int)*GX*GY*Theta);

	int iter=0;
	while(pq.size()>0)
	{
		State current=pq.top();
		pq.pop();

		if(abs(current.gx-target.gx)<=1 && abs(current.gy-target.gy)<=1 && abs(current.gtheta-target.gtheta)<=5){
			cout<<"Reached target."<<endl;

			State Dummy;
			current.change=PRIORITY_OBSTACLE_NEAR*(map.obs_dist_max-map.nearest_obstacle_distance(current))/(float)(map.obs_dist_max-1)+
						   fabs(current.steer_angle)/BOT_M_ALPHA+1;

			while(current.x!=start.x || current.y!=start.y || current.theta!=start.theta){
				current.velocity=VELOCITY_MAX/current.change;
				display.drawCar(current);
			    display.show(2000/current.velocity);//This can be removed while executing the algo
				Dummy=previous[current.gx][current.gy][current.gtheta];
				Dummy.change=PRIORITY_MOVEMENT*fabs(Dummy.steer_angle-current.steer_angle)/(2.0*BOT_M_ALPHA)+
					     PRIORITY_OBSTACLE_NEAR*(map.obs_dist_max-map.nearest_obstacle_distance(Dummy))/(float)(map.obs_dist_max-1)+
					     fabs(Dummy.steer_angle)/BOT_M_ALPHA+1;
				current=Dummy;
			}
			break;
		}

		if(vis[current.gx][current.gy][current.gtheta]){
			continue;
		}

		vis[current.gx][current.gy][current.gtheta]=1;

		vector<State> next=current.getNextStates();
		float steer_angle_change;

		for(int i=0;i<next.size();i++){
			//display.drawCar(next[i]);
			if(!map.checkCollision(next[i])){


				if(!vis[next[i].gx][next[i].gy][next[i].gtheta]){
					//display.drawCar(next[i]);
					current.next=&(next[i]);
					next[i].previous=&(current);
				  if(current.x==start.x && current.y==start.y && current.theta==start.theta) current.direction=next[i].direction;;

					steer_angle_change=ANGLE_CHANGE_WEIGHT*fabs(next[i].steer_angle-current.steer_angle)/(2*BOT_M_ALPHA);

					if(next[i].direction!=current.direction&&i==1)
					{
						next[i].cost3d=current.cost3d+COST_DIR_CHANGE_TO_FRONT+FRONT_STRAIGHT+steer_angle_change;
					}
					else
						if(next[i].direction!=current.direction&&next[i].direction==0)
						{
							next[i].cost3d=current.cost3d+COST_DIR_CHANGE_TO_FRONT+FRONT_CURVED+steer_angle_change;
						}
						else
							if(next[i].direction!=current.direction&&i==4)
							{
								next[i].cost3d=current.cost3d+COST_DIR_CHANGE_TO_BACK+BACK_STRAIGHT+steer_angle_change;
							}
							else
								if(next[i].direction!=current.direction&&next[i].direction==0)
								{
									next[i].cost3d=current.cost3d+COST_DIR_CHANGE_TO_BACK+BACK_CURVED+steer_angle_change;
								}
								else if(i==1)
								{
									next[i].cost3d=current.cost3d+FRONT_STRAIGHT+steer_angle_change;
								}
								else
									if(next[i].direction==0)
									{
										next[i].cost3d=current.cost3d+FRONT_CURVED+steer_angle_change;
									}
									else
										if(i==4)
										{
											next[i].cost3d=current.cost3d+BACK_STRAIGHT+steer_angle_change;
										}
										else
										{
											next[i].cost3d=current.cost3d+BACK_CURVED+steer_angle_change;
										}
								//next[i].cost3d=current.cost3d+1;
					pq.push(next[i]);

					previous[next[i].gx][next[i].gy][next[i].gtheta]=current;
				}
			}
		}
	}
	cout<<"Done."<<endl;
	display.show(0);

	return;
}
