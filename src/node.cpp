#include <ros/ros.h>
#include <ros/console.h>
#include <ros/callback_queue.h>

#include <octomap/octomap.h>
#include <octomap/OcTree.h>

#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>

#include <custom_msgs/gridRow.h>
#include <custom_msgs/gridMap.h>
#include <custom_msgs/gridPoint.h>
#include <custom_msgs/pointData.h>
#include <custom_msgs/position.h>

#include <custom_msgs/baseDrive.h>
#include <custom_msgs/baseRotate.h>
#include <custom_msgs/goalControl.h>
#include <custom_msgs/goalDetail.h>
#include <custom_msgs/areaReset.h>

#include <memory>
#include <mutex>
#include <condition_variable>

#include<bits/stdc++.h>

/*
// ------------------------------------- Input values ------------------------------------
*/

double x_positive;    		//Value in meters
double x_negative;			//Value in meters
double y_positive;			//Value in meters
double y_negative;			//Value in meters

double resolution;			//Value in meters

double robotRadius;			//Value in meters
double robotHeight;			//Value in meters

double minScanRange;		//Value in meters
double maxScanRange;		//Value in meters
double unBlockDistance;		//Value in meters

double clearanceDistance;	//Value in meters
double clearanceAngle;		//Value in radians

int linearIgnore;
int angularIgnore;

/*
// ---------------------------------- Derivated values ------------------------------------
*/

int padding;				// padding size is the size obstacles get inflated by
int maskside;				// filter size. this runs on the init grid and updates the paddedGrid
int unBlockCells;			// Unblock filter mask radius in cell count

double unitOffSet;			// distance to the center of a cell. treated as a offset and added in calculations
double invResolution;				// inverse of resolution. easier to multiply than divide (silly me!)
																
int gridColCount;			// number of columns in the grid
int gridRowCount;			// number of rows in the grid

int initColCount;			// number of columns in the initgrid
int initRowCount;			// number of rows in the initgrid

int paddColCount;			// number of columns in the paddedgrid
int paddRowCount;			// number of rows in the paddedgrid

double offsetX;				// x distance to the origin of initgrid from the starting position of robot
double offsetY;				// y distance to the origin of initgrid from the starting position of robot

double mapLowX;				// x coordinate of the origin of initgrid in the robots frame (for octomap searching)
double mapHighX;			// x coordinate of the farend of initgrid in the robots frame (for octomap searching)

double mapLowY;				// y coordinate of the origin of initgrid in the robots frame (for octomap searching)
double mapHighY;			// y coordinate of the farend of initgrid in the robots frame (for octomap searching)

// Description of the Grid- {1--> not occupied} {0--> occupied} 

/*
// ---------------------------------- Other variables ------------------------------------
*/

octomap::point3d currentPosition;
octomap::point3d goal;
octomap::point3d serverGoal;
octomap::point3d nextPosition;
octomap::point3d previousPosition;

ros::ServiceClient clientGoalPosition;
ros::ServiceClient clientGoalRemove;
ros::ServiceClient serverGoalRemove;
ros::ServiceClient forwardClient;
ros::ServiceClient reverseClient;
ros::ServiceClient rotateClient;

ros::Publisher grid_pub;

octomap::OcTree* tree = nullptr;

std::mutex inputMutex;
std::condition_variable mapReadyCV;
bool mapReady = false;

typedef std::pair<int, int> Pair; 
typedef std::pair<double, std::pair<int, int> > pPair;

struct cell { 
    int parent_i, parent_j;
    double f, g, h; 
}; 

bool unExplored = true;
bool gotServerGoal = false;
bool usingServerGoal = false;
bool serverCompleted = false;

double temp_x_positive;
double temp_x_negative;
double temp_y_positive;
double temp_y_negative;

double currentYaw, previousYaw;

/*
/ calculation of secondry values, this function will be called whenever area boundries are updated
*/


void updateValues(){
	padding  = (int) (robotRadius/resolution);
	maskside = padding*2 + 1;
	unBlockCells = (int) (unBlockDistance/resolution);

	unitOffSet = resolution/2;								
	invResolution    = 1/resolution;								
																	
	gridColCount = (x_negative + x_positive)*invResolution;		
	gridRowCount = (y_negative + y_positive)*invResolution;		

	initColCount = gridColCount + 2*padding;					
	initRowCount = gridRowCount + 2*padding;			

	paddColCount = gridColCount + 4*padding;		
	paddRowCount = gridRowCount + 4*padding;		

	offsetX	  	= x_negative+((padding-1)*resolution);		
	offsetY	  	= y_negative+((padding-1)*resolution);	
	
	mapLowX 	= (-1*x_negative)-((padding-1)*resolution);		
	mapHighX 	= x_positive+((padding+1)*resolution);		
	
	mapLowY 	= (-1*y_negative)-((padding-1)*resolution);	
	mapHighY 	= y_positive+((padding+1)*resolution);	
}

double round(double var) {return (((double)((int)(var * 1000 + .5)))/1000);}

/*
/ once the map is completely explored, this function will be called to save the octomap and execute ros::shutdown()
*/

void exit(){
	unExplored = false;
}

/*
/ request a new goal
*/

bool requestGoal(){

	if (gotServerGoal)
	{
		if (serverCompleted)
		{
			goal = octomap::point3d(x_negative, y_negative, 0);
			
			ROS_INFO_STREAM("planner_node : exploration ----complete---- according to server " << "goal -> x " << goal.x() << " y "<< goal.y());

			gotServerGoal = false;
			exit();
			return false;
		}
		else{
			goal = serverGoal;

			ROS_INFO_STREAM("planner_node : using goal -> x " << goal.x() << " y "<< goal.y() << " from server");

			gotServerGoal = false;
			usingServerGoal = true;

			x_positive = temp_x_positive;
    		x_negative = temp_x_negative;
    		y_positive = temp_y_positive;
    		y_negative = temp_y_negative;

			updateValues();

			return true;
		}
	}
	else
	{
		custom_msgs::goalControl srvGoal;

		srvGoal.request.execute = true;

		ROS_INFO("planner_node : requested goalCalculate service");

		if (clientGoalPosition.call(srvGoal)){

			goal = octomap::point3d(srvGoal.response.x+x_negative, srvGoal.response.y+y_negative, srvGoal.response.z);

			ROS_INFO_STREAM("planner_node : using goal -> x " << goal.x() << " y "<< goal.y() << " from explore module");
			
			if (srvGoal.response.isNull){
				ROS_INFO_STREAM("planner_node : exploration ----complete---- according to explore module");
				exit();
			}

		} else {
			ROS_ERROR("planner_node : failed to call service goalCalculate");
		}

		return false;
	}
}

/*
/ removes goal if the goal cannot be reached or a path to goal cannot be calculated
*/

void removeGoal(){
	custom_msgs::goalDetail rmvGoal;

	rmvGoal.request.x = goal.x() - x_negative;
	rmvGoal.request.y = goal.y() - y_negative;
	rmvGoal.request.z = goal.z();

	if (usingServerGoal)
	{
		if (serverGoalRemove.call(rmvGoal)){
			ROS_INFO("planner_node : requested server to remove goal");
			usingServerGoal = false;
		}
	}
	else 
	{
		if (clientGoalRemove.call(rmvGoal)){
			ROS_INFO("planner_node : requested explore module to remove goal");
		}
	}

}

/*
/ calculate the grid by iterating through the map. on ground, look for unoccupied areas which means holes and on 
/ surrounding space, looks for occupied nodes which mean obstacles. this is used to calculate the path.
*/ 

void buildMap(std::vector<std::vector<int> > &discoveredGrid, std::vector<std::vector<int> > &initialGrid, std::vector<std::vector<int> > &processedGrid)
{	
	// Description of the Grid- {1--> not occupied} {0--> occupied}
	std::vector<std::vector<int> > paddedGrid( paddRowCount, std::vector<int> (paddColCount, 1));

	// Description of the Grid- {1--> not discovered} {0--> discovered}
	std::vector<std::vector<int> > discoverGrid( initRowCount, std::vector<int> (initColCount, 1));

	//inspect surrounding
	double lower = currentPosition.z() + unitOffSet + resolution;
	double upper = currentPosition.z() + robotHeight;

	std::unique_lock<std::mutex> lock(inputMutex);
	mapReadyCV.wait(lock, []{return mapReady;});

	for (double z=lower; z<upper; z+=resolution){
        for (double x=mapLowX; x<mapHighX; x+=resolution){
            for (double y=mapLowY; y<mapHighY; y+=resolution){
                if (tree->search(x, y, z)){

                    octomap::OcTreeNode* key = tree->search(x, y, z);

					int xN = (int) ((x + offsetX)*invResolution);
					int yN = (int) ((y + offsetY)*invResolution);

					if(tree->isNodeOccupied(key)){

						initialGrid[yN][xN] = 0;

						int xlow = xN;
						int xhigh = xN+maskside;
						int ylow = yN;
						int yhigh = yN+maskside;

						for (int a=xlow; a<xhigh; a++){
							for (int b=ylow; b<yhigh; b++){
								paddedGrid[b][a] = 0;
							}
						}
					}

					discoverGrid[yN][xN] = 0;
                }
            }
        }
    }

	//inspect floor

	// double zf = currentPosition.z() + unitOffSet;

	// ROS_INFO_STREAM("started floor analysis");
	// for (double xf=mapLowX; xf<mapHighX; xf+=resolution){
    //     for (double yf=mapLowY; yf<mapHighY; yf+=resolution){
    //         if (tree->search(xf, yf, zf)){
    //             octomap::OcTreeNode* key = tree->search(xf, yf, zf);

	// 			int xfN = (int) ((xf + offsetX)*invResolution);
	// 			int yfN = (int) ((yf + offsetY)*invResolution);

	// 			if(!(tree->isNodeOccupied(key))){
	// 				octomap::point3d point (xf, yf, zf);
	// 				double distance = currentPosition.distance(point);

	// 				if ((distance>minScanRange) && (distance<maxScanRange)){
	// 					int count = 0;
	// 					double arrayM [9][2] = {{xf-resolution, yf-resolution}, {xf-resolution, yf}, {xf-resolution, yf+resolution}, 
	// 										   {xf     		 , yf-resolution}, {xf     		 , yf}, {xf     	  , yf+resolution}, 
	// 										   {xf+resolution, yf-resolution}, {xf+resolution, yf}, {xf+resolution, yf+resolution}};

	// 					for (int i=0; i<9; i++){
	// 						if (tree->search(arrayM[i][0], arrayM[i][1], zf)){
	// 							octomap::OcTreeNode* keyMask = tree->search(arrayM[i][0], arrayM[i][1], zf);
	// 							if(!(tree->isNodeOccupied(keyMask))){
	// 								count++;
	// 							}
	// 						}
	// 					}
						
	// 					if (count>4){

	// 						initialGrid[yfN][xfN] = 0;

	// 						int xflow = xfN;
	// 						int xfhigh = xfN+maskside;
	// 						int yflow = yfN;
	// 						int yfhigh = yfN+maskside;

	// 						for (int af=xflow; af<xfhigh; af++){
	// 							for (int bf=yflow; bf<yfhigh; bf++){
	// 								paddedGrid[bf][af] = 0;
	// 							}
	// 						}
	// 					}
	// 				}
	// 			}

	// 			discoverGrid[yfN][xfN] = 0;
	//			delete key;
    //         }
    //     }
    // }

	lock.unlock();
	mapReadyCV.notify_one();

	for (int i=0; i<gridRowCount; i++){
		for (int j=0; j<gridColCount; j++){
			processedGrid[i][j] = paddedGrid[i+(padding*2)][j+(padding*2)];
			discoveredGrid[i][j] = discoverGrid[i+padding][j+padding];
		}
	}
}

/*
/ Astar algorithm to calculate the path.
*/

bool isValid(int row, int col) {
	return (row >= 0) && (row < gridRowCount) && (col >= 0) && (col < gridColCount); 
} 

bool isUnBlocked(std::vector<std::vector<int> > &grid, int row, int col) { 
	if (grid[row][col] == 1) return (true); else return (false); 
}

bool isDestination(int row, int col, Pair dest) { 
	if (row == dest.first && col == dest.second) return (true); else return (false); 
} 

double calculateHValue(int row, int col, Pair dest) {
	return ((double) std::sqrt((row-dest.first)*(row-dest.first) + (col-dest.second)*(col-dest.second))); 
} 

void tracePath( std::vector<std::vector<cell> > &cellDetails, Pair dest, std::vector<octomap::point3d> &outPath){
	int row = dest.first; 
	int col = dest.second; 

	std::stack<Pair> PathStack;
	outPath.clear();

	while (!(cellDetails[row][col].parent_i == row && cellDetails[row][col].parent_j == col )) { 
		PathStack.push(std::make_pair (row, col)); 
		int temp_row = cellDetails[row][col].parent_i; 
		int temp_col = cellDetails[row][col].parent_j; 
		row = temp_row; 
		col = temp_col; 
	} 

	PathStack.push(std::make_pair (row, col)); 

	while (!PathStack.empty()) { 
		std::pair<int,int> p = PathStack.top(); 
		PathStack.pop();
        octomap::point3d position (p.second, p.first, 0);
        outPath.push_back(position); 
	}
} 

bool search(std::vector<std::vector<int> > &grid, Pair src, Pair dest, std::vector<octomap::point3d> &outPath) {
	if (isValid (src.first, src.second) == false) return false;
	if (isValid (dest.first, dest.second) == false) return false;
	if (isUnBlocked(grid, src.first, src.second) == false || isUnBlocked(grid, dest.first, dest.second) == false) return false; 
	if (isDestination(src.first, src.second, dest) == true) return false; 

	std::vector<std::vector<bool> > closedList(gridRowCount, std::vector<bool> (gridColCount, false));
	std::vector<std::vector<cell> > cellDetails(gridRowCount, std::vector<cell> (gridColCount));
	int i, j; 

	for (i=0; i<gridRowCount; i++) { 
		for (j=0; j<gridColCount; j++) { 
			cellDetails[i][j].f = FLT_MAX; 
			cellDetails[i][j].g = FLT_MAX; 
			cellDetails[i][j].h = FLT_MAX; 
			cellDetails[i][j].parent_i = -1; 
			cellDetails[i][j].parent_j = -1; 
		} 
	}

	i = src.first, j = src.second;

	cellDetails[i][j].f = 0.0; 
	cellDetails[i][j].g = 0.0; 
	cellDetails[i][j].h = 0.0; 
	cellDetails[i][j].parent_i = i; 
	cellDetails[i][j].parent_j = j; 

	std::set<pPair> openList; 
	openList.insert(std::make_pair (0.0, std::make_pair (i, j))); 
	bool foundDest = false; 

	while (!openList.empty()) { 
		pPair p = *openList.begin(); 
		openList.erase(openList.begin()); 
		i = p.second.first; 
		j = p.second.second; 
		closedList[i][j] = true; 
	
		double gNew, hNew, fNew;

        int array [8][2] = {{i-1, j}, {i+1, j}, {i, j+1}, {i, j-1}, {i-1, j+1}, {i-1, j-1}, {i+1, j+1}, {i+1, j-1}};

        for (int a = 0; a < 8; a++ ) {
            if (isValid(array[a][0], array[a][1]) == true) { 
                if (isDestination(array[a][0], array[a][1], dest) == true) {  
                    cellDetails[array[a][0]][array[a][1]].parent_i = i; 
                    cellDetails[array[a][0]][array[a][1]].parent_j = j;
                    tracePath (cellDetails, dest, outPath); 
                    foundDest = true; 
                    return true; 
                } else if (closedList[array[a][0]][array[a][1]] == false && isUnBlocked(grid, array[a][0], array[a][1]) == true) { 
                    gNew = cellDetails[i][j].g + 1.0; 
                    hNew = calculateHValue (array[a][0], array[a][1], dest); 
                    fNew = gNew + hNew; 
    
                    if (cellDetails[array[a][0]][array[a][1]].f == FLT_MAX || cellDetails[array[a][0]][array[a][1]].f > fNew) { 
                        openList.insert( std::make_pair(fNew, std::make_pair(array[a][0], array[a][1]))); 
                        cellDetails[array[a][0]][array[a][1]].f = fNew; 
                        cellDetails[array[a][0]][array[a][1]].g = gNew; 
                        cellDetails[array[a][0]][array[a][1]].h = hNew; 
                        cellDetails[array[a][0]][array[a][1]].parent_i = i; 
                        cellDetails[array[a][0]][array[a][1]].parent_j = j; 
                    } 
                } 
            }
        }  
	} 
	if (foundDest == false)	return false; 
}

/*
/ Additional functions to process the path
*/

bool isBlocked(octomap::point3d &point, std::vector<std::vector<int> > &grid){

	int col = (int) (point.x()*invResolution);
	int row = (int) (point.y()*invResolution);

	if (grid[row][col] == 0) return true; else return false; 
}

/*
/ converts the grid path into real world path
*/

void convertPath(std::vector<octomap::point3d> &inPath, std::vector<std::vector<int> > &discoveredGrid, std::vector<octomap::point3d> &outPath){
	outPath.clear();

	std::vector<octomap::point3d> intermediatePath;

	int linear_count = 0;
	int angular_count = 0;

	ROS_INFO_STREAM("Initial path nodes : " << inPath.size());

	intermediatePath.push_back(inPath[0]);

	for (int i = 1; i < inPath.size()-1; i++)
	{
		double m1 = (inPath[i].y() - inPath[i-1].y())/(inPath[i].x() - inPath[i-1].x());
		double m2 = (inPath[i+1].y() - inPath[i].y())/(inPath[i+1].x() - inPath[i].x());

		if ((m1 == m2) && (linear_count<linearIgnore) && (angular_count==0))  // intermediate points in a straight line
		{
			linear_count += 1; 
		}
		else if ((m1 == m2) && (linear_count==linearIgnore) && (angular_count==0)) // major point in straight line based on point count
		{
			intermediatePath.push_back(inPath[i]);
			linear_count = 0;
		}
		else if ((m1 == m2) && (linear_count==0) && (angular_count>0)) // starting point of straight line after zigzag
		{
			intermediatePath.push_back(inPath[i-1]);
			linear_count += 1;
			angular_count = 0;
		}
		else if ((std::abs(m1*m2 + 1) <= 0.001) && (angular_count==0) && (linear_count>0)) // end point of straight line start of zigzag
		{
			intermediatePath.push_back(inPath[i]);
			angular_count += 1;
			linear_count = 0;
		}
		else if ((std::abs(m1*m2 + 1) <= 0.001) && (angular_count<angularIgnore) && (linear_count==0)) // intermediate points in a zigzag
		{
			angular_count += 1;
		}
		else if ((std::abs(m1*m2 + 1) <= 0.001) && (angular_count==angularIgnore) && (linear_count==0)) //  major point in a zigzag
		{
			intermediatePath.push_back(inPath[i]);
			angular_count = 0;
		}
	}

	intermediatePath.push_back(inPath[inPath.size()-1]);

	ROS_INFO_STREAM("reduced path nodes : " << intermediatePath.size());

	for (int i=0; i<intermediatePath.size(); i++){

		if (discoveredGrid[intermediatePath[i].y()][intermediatePath[i].x()]==0){
			double x = (double) intermediatePath[i].x()*resolution;
			double y = (double) intermediatePath[i].y()*resolution;
			double z = (double) intermediatePath[i].z();

			octomap::point3d node (x, y, z);
			outPath.push_back(node);
		} 
		else break;
	}
	
	inPath.clear();

	ROS_INFO_STREAM("discovered area path nodes : " << outPath.size());
}

/*
/ service calls to drive the robot from currentPosition to nextPosition
*/

bool drive(octomap::point3d &nextPosition){
	custom_msgs::baseDrive srvDrive;

	srvDrive.request.x = nextPosition.x();
	srvDrive.request.y = nextPosition.y();
	srvDrive.request.z = nextPosition.z();

	ROS_INFO("planner_node : requested drive service");

	if (forwardClient.call(srvDrive)){
		if (srvDrive.response.success){
			ROS_INFO("planner_node : point reached");
			return true;
		} else {
			ROS_INFO("planner_node : failed to reach due to obstacle");
			return false;
		}

	} else {
		ROS_ERROR("planner_node : failed to call service baseC");
		return false;
	}
}

bool drive_unblocked(octomap::point3d &position, std::vector<std::vector<int> > &processedGrid, std::vector<std::vector<int> > &discoverGrid){
	custom_msgs::baseDrive srvDrive;
	octomap::point3d newPosition;

	ROS_INFO("planner_node : requested unblocking service");

    int col = (int) (position.x()*invResolution);
	int row = (int) (position.y()*invResolution);

	bool notFound = true;
	int layer = unBlockCells;

	octomap::point3d point = position;

	while ((notFound) && (row-layer>=0) && (row+layer<gridRowCount)){
		int array [4][2] = {{row+layer, col+layer}, 
							{row-layer, col+layer},
							{row-layer, col-layer}, 
							{row+layer, col-layer}};

		for (int i = 0; i < 4; i++){
			if ((processedGrid[array[i][0]][array[i][1]] == 1) && (discoverGrid[array[i][0]][array[i][1]] == 0)){
				point.x() = array[i][1]*resolution;
				point.y() = array[i][0]*resolution;
				notFound = false;
				break;
			}
		}
		newPosition = point;
		layer += 1;
	}

	ROS_INFO_STREAM("Current Position: " << position.x() << " " << position.y());
	ROS_INFO_STREAM("New Position: " << newPosition.x() << " " << newPosition.y());

	srvDrive.request.x = newPosition.x();
	srvDrive.request.y = newPosition.y();
	srvDrive.request.z = newPosition.z();

	ROS_INFO("planner_node : requested drive service");
 
	if (forwardClient.call(srvDrive)){
		ROS_INFO("planner_node : unblocked point reached");
		if (srvDrive.response.success) return true; else return false;
	} else {
		ROS_ERROR("planner_node : failed to call service baseForward");
		return false;
	}
}

void reverse(octomap::point3d &nextPosition){
	custom_msgs::baseDrive srvDrive;

	srvDrive.request.x = nextPosition.x();
	srvDrive.request.y = nextPosition.y();
	srvDrive.request.z = nextPosition.z();

	ROS_INFO("planner_node : requested reverse service");

	if (reverseClient.call(srvDrive)){
		ROS_INFO("planner_node : reversed");
	} else {
		ROS_ERROR("planner_node : failed to call service BaseReverse");
	}
}

/*
/ service call to make the robot rotate inplace in order to update the map
*/

void rotate360(){
	custom_msgs::baseRotate srvRotate;

	srvRotate.request.angle = 3.14;

	// rotate by 180 degrees
	if (rotateClient.call(srvRotate)){
		ROS_INFO("planner_node : first half rotated");
	} else {
		ROS_ERROR("planner_node : failed to call service baseRotate");
	}

	srvRotate.request.angle = 3.14;

	// rotate by 180 degrees
	if (rotateClient.call(srvRotate)){
		ROS_INFO("planner_node : second half rotated");
	} else {
		ROS_ERROR("planner_node : failed to call service baseRotate");
	}
}

/*
/ converts the gridmaps into a 'gridMap' msgs, path into 'pointDataArray' and publishes it
*/

void publish(std::vector<std::vector<int> > &discoveredGrid, std::vector<std::vector<int> > &initGrid, std::vector<std::vector<int> > &procGrid, std::vector<octomap::point3d> &path){
	custom_msgs::gridMap gridMap;

	for (int i=0; i<gridRowCount; i++){
		custom_msgs::gridRow rowArray;

		for(int j=0; j<gridColCount; j++) {
			custom_msgs::gridPoint point;
			
			point.init = initGrid[i+padding][j+padding];
			point.proc = procGrid[i][j];
			point.disc = discoveredGrid[i][j];

			rowArray.row.push_back(point);
		}
		gridMap.grid.push_back(rowArray);
	}

	for(int i=0; i<path.size(); i++){
		custom_msgs::pointData msgInstance;

		msgInstance.x = path[i].x()*invResolution;
		msgInstance.y = path[i].y()*invResolution;
		msgInstance.z = 0;

		gridMap.path.push_back(msgInstance);
	}

	gridMap.pathLength = path.size();
	gridMap.rowCount = gridRowCount;
	gridMap.colCount = gridColCount;
	
	grid_pub.publish(gridMap);
}

/*
/  The node subscribe to topics 'Octomap' at (octomap_msgs/Octomap)  and 'Position' at (custom_msgs/position). 
/  mapCallback and positionCallback will handles incoming msgs from these two topics.
*/

void positionCallback(const custom_msgs::position::ConstPtr &msg){
	currentPosition = octomap::point3d(msg->x+x_negative, msg->y+y_negative, msg->z);
	currentYaw = msg->Y;
}

void mapCallback(const octomap_msgs::Octomap::ConstPtr &msg){
    octomap::AbstractOcTree* tempTree = octomap_msgs::fullMsgToMap(*msg);

    std::unique_lock<std::mutex> lock(inputMutex);

	if (tree) delete tree;
    tree = (octomap::OcTree*) tempTree;

	mapReady = true;
	
	lock.unlock();
	mapReadyCV.notify_one();
}

bool resetAreaCallback(custom_msgs::areaReset::Request &request, custom_msgs::areaReset::Response &response)
{
	ROS_DEBUG("planner_node : reset goal request received");
		
	octomap::point3d tempGoal;

	temp_x_positive = request.x_positive;
    temp_x_negative = request.x_negative;
    temp_y_positive = request.y_positive;
    temp_y_negative = request.y_negative;

	tempGoal = octomap::point3d(request.x+temp_x_negative, request.y+temp_y_negative, request.z);
	
	gotServerGoal = true;
	serverGoal = tempGoal;

	if (!unExplored){
		unExplored = true;
		ROS_INFO("---------- Exploration Continuing ----------");
	}

	serverCompleted = request.completion;	

	response.success = true;

	ROS_DEBUG("planner_node : succesfully resetted");

	return true;
}
/*  
/    node starts itself and places server calls to both /goalCalculate and /drive. goalCalculate servercall starts the
/    goal calculation replies with goalPoint and drive servercall starts the movement of the robot and returns only after
/	 it reaches the currently specified point. it was implemented as this for maximum efficiency.
*/

int main(int argc, char **argv)
{
	ros::init (argc, argv, "Path Planner");
	ros::NodeHandle node;
	
	ROS_INFO("Initialized the planner_node");

	node.param("area/front", 					x_positive, 		5.0);
    node.param("area/back",  					x_negative, 		0.0);
    node.param("area/left",  					y_positive, 		5.0);
    node.param("area/right", 					y_negative, 		0.0);

    node.param("map/resolution", 				resolution, 		0.05);
	node.param("map/minScanRange",				minScanRange, 		0.70);
	node.param("map/maxScanRange", 				maxScanRange, 		3.50);
	node.param("map/unBlockingDistance", 		unBlockDistance, 	0.25);

	node.param("path/linearNodeIgnoreCount",	linearIgnore, 		5);
	node.param("path/angularNodeIgnoreCount", 	angularIgnore, 		0);
    node.param("path/clearanceDistance", 		clearanceDistance, 	2.50);
	node.param("path/clearanceAngle", 			clearanceAngle, 	1.05);

	node.param("robot/dimension/radius", 		robotRadius, 		0.30);
	node.param("robot/dimension/height", 		robotHeight, 		0.50);

    temp_x_positive = x_positive;
    temp_x_negative = x_negative;
    temp_y_positive = y_positive;
    temp_y_negative = y_negative;

	updateValues();

	ROS_INFO("planner_node : loaded parameters and updated values");

    ros::Subscriber map_sub = node.subscribe("octomap", 1, mapCallback);
	ros::Subscriber pos_sub = node.subscribe("position", 1, positionCallback);

	ROS_INFO("planner_node : created subscribers");

	grid_pub = node.advertise<custom_msgs::gridMap>("grid", 1, true);
	
	ROS_INFO("planner_node : created publishers");

	ros::service::waitForService("goalCalculate");
	ros::service::waitForService("goalRemove");
	ros::service::waitForService("baseForword");
	ros::service::waitForService("baseRotate");
	ros::service::waitForService("baseReverse");

	clientGoalPosition = node.serviceClient<custom_msgs::goalControlRequest, custom_msgs::goalControlResponse>("goalCalculate");
	clientGoalRemove   = node.serviceClient<custom_msgs::goalDetailRequest, custom_msgs::goalDetailResponse>("goalRemove");
	serverGoalRemove   = node.serviceClient<custom_msgs::goalDetailRequest, custom_msgs::goalDetailResponse>("goalRemoveServer");
	forwardClient      = node.serviceClient<custom_msgs::baseDriveRequest, custom_msgs::baseDriveResponse>("baseForword");
	reverseClient      = node.serviceClient<custom_msgs::baseDriveRequest, custom_msgs::baseDriveResponse>("baseReverse");
    rotateClient       = node.serviceClient<custom_msgs::baseRotateRequest, custom_msgs::baseRotateResponse>("baseRotate");
	
	ROS_INFO("planner_node : created service clients");

	ros::ServiceServer serviceResetArea = node.advertiseService<custom_msgs::areaResetRequest, custom_msgs::areaResetResponse>("areaResetPlanner", resetAreaCallback);

	ros::AsyncSpinner spinner (5);
	spinner.start();

	while (ros::ok()){
		while (unExplored){

			int index =  1;
			bool pathFound = false;
			double remainingDistance, travelledDistance = 0;
			double desiredYaw, angle = 0;

		    rotate360();

			requestGoal();

			norotate:
			// Description of the Grid- {1--> not occupied} {0--> occupied}
			std::vector<std::vector<int> > initialGrid (initRowCount, std::vector<int> (initColCount, 1));
			std::vector<std::vector<int> > processedGrid (gridRowCount, std::vector<int> (gridColCount, 1));

			// Description of the Grid- {1--> not discovered} {0--> discovered}
			std::vector<std::vector<int> > discoveredGrid (gridRowCount, std::vector<int> (gridColCount, 1));

			std::vector<octomap::point3d> path, processedPath;

			buildMap(discoveredGrid, initialGrid, processedGrid);

			//turn source and goal into points on the grid
			int srcX = (int) (currentPosition.x()*invResolution);
			int srcY = (int) (currentPosition.y()*invResolution);

			int desX = (int) (round(goal.x())*invResolution);
			int desY = (int) (round(goal.y())*invResolution);

			pathFound = search(processedGrid, std::make_pair(srcY, srcX), std::make_pair(desY, desX), path);
			//iterate till a path is found

			while (!pathFound){

				//check whether the path was not calculated due to source being blocked.

				if (isBlocked(currentPosition, processedGrid)){
					drive_unblocked(currentPosition, processedGrid, discoveredGrid); //move the robot to a free cell if source was blocked
				} else {
					removeGoal();                                    //remove goal if source is not the cause
				}

				if (requestGoal()) goto norotate;                                      //rerequest the goal
				
				//turn source and goal into points on the grid
				int srcX = (int) (currentPosition.x()*invResolution);
				int srcY = (int) (currentPosition.y()*invResolution);

				int desX = (int) (round(goal.x())*invResolution);
				int desY = (int) (round(goal.y())*invResolution);

				pathFound = search(processedGrid, std::make_pair(srcY, srcX), std::make_pair(desY, desX), path);
			}

			if (usingServerGoal) usingServerGoal = false;

			ROS_INFO_STREAM("planner_node : selected goal -> x " << goal.x() << " y "<< goal.y());

			convertPath(path, discoveredGrid, processedPath);

			publish(discoveredGrid, initialGrid, processedGrid, processedPath);

			ROS_INFO("planner_node : Path Calculated successfully");

			//calculate distance to the goal and mark current position as previous position
			remainingDistance = goal.distance(currentPosition);
			previousPosition = currentPosition;

			if (processedPath.size()>0){
				nextPosition = processedPath[index];
				previousYaw = atan2(nextPosition.y()-currentPosition.y(), nextPosition.x()-currentPosition.x());
			} else {
				previousYaw = currentYaw;
			}
			
			while ((remainingDistance >= resolution) && pathFound && (index<processedPath.size())) {
				
				nextPosition = processedPath[index];
				desiredYaw = atan2(nextPosition.y()-currentPosition.y(), nextPosition.x()-currentPosition.x());

				if ((desiredYaw>1.57)&&(previousYaw<-1.57)){
					angle = 6.28 - (desiredYaw - previousYaw);
				} else if ((desiredYaw<-1.57)&&(previousYaw>1.57)) {
					angle = 6.28 + (desiredYaw - previousYaw);
				} else {
					angle = std::abs(desiredYaw - previousYaw);
				}

				// check whether a large turn is needed which means robot is at a corner
				// which is usually unmapped. if so recalculation is needed. so exit loop

				if ((angle > clearanceAngle) && (index > 1)) break;

				// check whether a consideralble travelledDistance was reached which means robot is
				// at a corner which is usually unmapped. if so recalculation is needed. so exit loop

				if (travelledDistance > clearanceDistance) break;

				// try to move to next position

				if (!(drive(nextPosition))) {
					reverse(previousPosition);
					break;
				};

				// calculate total distance moved 
				travelledDistance += previousPosition.distance(nextPosition);

				//calculate remaining distance

				previousPosition = currentPosition;
				remainingDistance = goal.distance(currentPosition);
				index++;		
			}

			// if (index>=processedPath.size())
			// {
			// 	removeGoal();
			// }
		}
	}
	
	return 0;
}