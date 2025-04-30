''' iPath: A C++ Library of Intelligent Global Path Planners for Mobile Robots with ROS Integration. 
 * Website: http:#www.iroboapp.org/index.php?title=IPath
 * Contact: 
 *
 * Copyright (c) 2014
 * Owners: Al-Imam University/King AbdulAziz Center for Science and Technology (KACST)/Prince Sultan University
 * All rights reserved.
 *
 * License Type: GNU GPL
 *
 *   This program is free software: you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation, version 3 of the License, or
 *   (at your option) any later version.
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with self program.  If not, see <http:#www.gnu.org/licenses/>.
'''

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h> 
#include <fstream>
#include <iostream>
#include <iomanip>
#include <string>

#include "RAstar_ros.h"

#include <pluginlib/class_list_macros.h>
#register self planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(RAstar_planner.RAstarPlannerROS, nav_core.BaseGlobalPlanner)


int value
int mapSize
bool* OGM
static  INFINIT_COST = INT_MAX; #not < cost of non connected nodes
infinity = std.numeric_limits< float >.infinity()
float tBreak;  # coefficient for breaking ties
ofstream MyExcelFile ("RA_result.xlsx", ios.trunc)

def clock_gettime(self, clk_id, timespect *tp):

def diff(self, start, end):
  timespec temp
	if (end.tv_nsec-start.tv_nsec)<0:		temp.tv_sec = end.tv_sec-start.tv_sec-1
		temp.tv_nsec = 1000000000+end.tv_nsec-start.tv_nsec
	} else:
		temp.tv_sec = end.tv_sec-start.tv_sec
		temp.tv_nsec = end.tv_nsec-start.tv_nsec

	return temp


inline vector <int> findFreeNeighborCell (int CellID)

################/
################/
################/
namespace RAstar_planner

#Default Constructor
RAstarPlannerROS.RAstarPlannerROS()


###################
RAstarPlannerROS.RAstarPlannerROS(ros.NodeHandle &nh)
  ROSNodeHandle = nh


###################
RAstarPlannerROS.RAstarPlannerROS(std.string name, costmap_ros)
  initialize(name, costmap_ros)



######################################
def initialize(self, name, costmap_ros):

  if not initialized_:
    costmap_ros_ = costmap_ros
    costmap_ = costmap_ros_.getCostmap()

    ros.NodeHandle private_nh("~/" + name)

    originX = costmap_.getOriginX()
    originY = costmap_.getOriginY()



	width = costmap_.getSizeInCellsX()
	height = costmap_.getSizeInCellsY()
	resolution = costmap_.getResolution()
	mapSize = width*height
	tBreak = 1+1/(mapSize); 
	value =0


	OGM = bool [mapSize]; 
    for (unsigned iy = 0; iy < costmap_.getSizeInCellsY(); iy++)
      for (unsigned ix = 0; ix < costmap_.getSizeInCellsX(); ix++)
        unsigned cost = static_cast<int>(costmap_.getCost(ix, iy))
        #cout<<cost
        if cost == 0:
          OGM[iy*width+ix]=True
        else:
          OGM[iy*width+ix]=False




	MyExcelFile << "StartID\tStartX\tStartY\tGoalID\tGoalX\tGoalY\tPlannertime(ms)\tpathLength\tnumberOfCells\t" << endl

    ROS_INFO("RAstar planner initialized successfully")
    initialized_ = True

  else:
    ROS_WARN("This planner has already been initialized... doing nothing")



##########################################

bool RAstarPlannerROS.makePlan( geometry_msgs.PoseStamped& start, goal,
                             std.vector<geometry_msgs.PoseStamped>& plan)

  if not initialized_:
    ROS_ERROR("The planner has not been initialized, call initialize() to use the planner")
    return False


  ROS_DEBUG("Got a start: %.2f, %.2f, a goal: %.2f, %.2f", start.pose.position.x, start.pose.position.y,
            goal.pose.position.x, goal.pose.position.y)

  plan.clear()

  if goal.header.frame_id != costmap_ros_.getGlobalFrameID():
    ROS_ERROR("This planner as configured will only accept goals in the %s frame, a goal was sent in the %s frame.",
              costmap_ros_.getGlobalFrameID().c_str(), goal.header.frame_id.c_str())
    return False


  tf.Stamped < tf.Pose > goal_tf
  tf.Stamped < tf.Pose > start_tf

  poseStampedMsgToTF(goal, goal_tf)
  poseStampedMsgToTF(start, start_tf)

  # convert the start and goal positions

  startX = start.pose.position.x
  startY = start.pose.position.y

  goalX = goal.pose.position.x
  goalY = goal.pose.position.y

  getCorrdinate(startX, startY)
  getCorrdinate(goalX, goalY)

  int startCell
  int goalCell

  if isCellInsideMap(startX, startY) and isCellInsideMap(goalX, goalY):
    startCell = convertToCellIndex(startX, startY)

    goalCell = convertToCellIndex(goalX, goalY)

MyExcelFile << startCell <<"\t"<< start.pose.position.x <<"\t"<< start.pose.position.y <<"\t"<< goalCell <<"\t"<< goal.pose.position.x <<"\t"<< goal.pose.position.y


  else:
    ROS_WARN("the start or goal is out of the map")
    return False


  # call global planner

  if isStartAndGoalCellsValid(startCell, goalCell):
        vector<int> bestPath
	bestPath.clear()

    bestPath = RAstarPlanner(startCell, goalCell)

#if the global planner find a path
    if  bestPath.size()>0:

# convert the path

      for (i = 0; i < bestPath.size(); i++)

        x = 0.0
        y = 0.0

        index = bestPath[i]

        convertToCoordinate(index, x, y)

        pose = goal

        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0

        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = 0.0
        pose.pose.orientation.w = 1.0

        plan.push_back(pose)



	path_length = 0.0
	
	std.vector<geometry_msgs.PoseStamped>it = plan.begin()
	
	geometry_msgs.PoseStamped last_pose
	last_pose = *it
	it++
	for (; it!=plan.end(); ++it)	   
  
    path_length += hypot(  (*it).pose.position.x - last_pose.pose.position.x, 
		                 (*it).pose.position.y - last_pose.pose.position.y )
	   last_pose = *it

	cout <<"The global path length: "<< path_length<< " meters"<<endl
	MyExcelFile << "\t" <<path_length <<"\t"<< plan.size() <<endl
      #publish the plan

      return True



    else:
      ROS_WARN("The planner failed to find a path, other goal position")
      return False




  else:
    ROS_WARN("Not valid start or goal")
    return False




###########################
def getCorrdinate(self, x, y):

  x = x - originX
  y = y - originY




#########################/
def convertToCellIndex(self, x, y):

  int cellIndex

  newX = x / resolution
  newY = y / resolution

  cellIndex = getCellIndex(newY, newX)

  return cellIndex


##############################
def convertToCoordinate(self, index, x, y):

  x = getCellColID(index) * resolution

  y = getCellRowID(index) * resolution

  x = x + originX
  y = y + originY


#############################/7
def isCellInsideMap(self, x, y):
  valid = True

  if x > (width * resolution) or y > (height * resolution):
    valid = False

  return valid


#################################
def mapToWorld(self, mx, my, wx, wy):
   costmap = costmap_ros_.getCostmap()
    wx = costmap.getOriginX() + mx * resolution
    wy = costmap.getOriginY() + my * resolution


################################
def RAstarPlanner(self, startCell, goalCell):
   vector<int> bestPath


#float g_score [mapSize][2]
float g_score [mapSize]

for (uint i=0; i<mapSize; i++)
	g_score[i]=infinity

   timespec time1, time2
  ''' take current time here '''
   clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &time1)

  bestPath=findPath(startCell, goalCell,  g_score)

   clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &time2)


   cout<<"time to generate best global path by Relaxed A* = " << (diff(time1,time2).tv_sec)*1e3 + (diff(time1,time2).tv_nsec)*1e-6 << " microseconds" << endl
   
   MyExcelFile <<"\t"<< (diff(time1,time2).tv_sec)*1e3 + (diff(time1,time2).tv_nsec)*1e-6 

  return bestPath




'''*****************************************************************************'''
#Function Name: findPath
#Inputs: the map layout, start and the goal Cells and a boolean to indicate if we will use break ties or not
#Output: the best path
#Description: it is used to generate the robot free path
'''*******************************************************************************'''
def findPath(self, startCell, goalCell, g_score[]):
	value++
	vector<int> bestPath
	vector<int> emptyPath
	cells CP

	multiset<cells> OPL
	int currentCell

	#calculate g_score and f_score of the start position
	g_score[startCell]=0
	CP.currentCell=startCell
	CP.fCost=g_score[startCell]+calculateHCost(startCell,goalCell)

	#add the start cell to the open list
	OPL.insert(CP)
	currentCell=startCell

	#while the open list is not empty continuie the search or g_score(goalCell) is equal to infinity
	while (not OPL.empty()and g_score[goalCell]==infinity) 
		#choose the cell that has the lowest cost fCost in the open set which is the begin of the multiset
		currentCell = OPL.begin().currentCell
		#remove the currentCell from the openList
		OPL.erase(OPL.begin())
		#search the neighbors of the current Cell
		vector <int> neighborCells; 
		neighborCells=findFreeNeighborCell(currentCell)
		for(uint i=0; i<neighborCells.size(); i++) #for each neighbor v of current cell
			# if the g_score of the neighbor is equal to INF: unvisited cell
			if g_score[neighborCells[i]]==infinity:
				g_score[neighborCells[i]]=g_score[currentCell]+getMoveCost(currentCell,neighborCells[i])
				addNeighborCellToOpenList(OPL, neighborCells[i], goalCell, g_score); 
			}#end if
		}#end for
	}#end while

	if(g_score[goalCell]!=infinity)  # if g_score(goalcell)==INF : construct path 
		bestPath=constructPath(startCell, goalCell, g_score)
		return   bestPath; 

	else:
		cout << "Failure to find a path not " << endl
		return emptyPath



'''*****************************************************************************'''
#Function Name: constructPath
#Inputs: the start and the goal Cells
#Output: the best path
#Description: it is used to construct the robot path
'''*******************************************************************************'''
def constructPath(self, startCell, goalCell, g_score[]):
	vector<int> bestPath
	vector<int> path

	path.insert(path.begin()+bestPath.size(), goalCell)
	int currentCell=goalCell

	while(currentCell!=startCell)
	{ 
		vector <int> neighborCells
		neighborCells=findFreeNeighborCell(currentCell)

		vector <float> gScoresNeighbors
		for(uint i=0; i<neighborCells.size(); i++)
			gScoresNeighbors.push_back(g_score[neighborCells[i]])
		
		int posMinGScore=distance(gScoresNeighbors.begin(), min_element(gScoresNeighbors.begin(), gScoresNeighbors.end()))
		currentCell=neighborCells[posMinGScore]

		#insert the neighbor in the path
		path.insert(path.begin()+path.size(), currentCell)

	for(uint i=0; i<path.size(); i++)
		bestPath.insert(bestPath.begin()+bestPath.size(), path[path.size()-(i+1)])

	return bestPath


'''*****************************************************************************'''
#Function Name: calculateHCost
#Inputs:the cellID and the goalCell
#Output: the distance between the current cell and the goal cell
#Description: it is used to calculate the hCost 
'''*******************************************************************************'''
'''
def calculateHCost(self, cellID, goalCell):
{    
  int x1=getCellRowID(goalCell)
  int y1=getCellColID(goalCell)
  int x2=getCellRowID(cellID)
  int y2=getCellColID(cellID)
  
  #if(getNeighborNumber()==4) 
    #The diagonal shortcut distance between two grid points (x1,y1) and (x2,y2) is:
    #  return min(abs(x1-x2),abs(y1-y2))*sqrt(2) + max(abs(x1-x2),abs(y1-y2))-min(abs(x1-x2),abs(y1-y2))
  
  #else:
    #manhatten distance for 8 neighbor
    return abs(x1-x2)+abs(y1-y2)

'''
'''*****************************************************************************'''
#Function Name: addNeighborCellToOpenList
#Inputs: the open list, neighbors Cell, g_score matrix, goal cell 
#Output: 
#Description: it is used to add a neighbor Cell to the open list
'''*******************************************************************************'''
def addNeighborCellToOpenList(self, & OPL, neighborCell, goalCell, g_score[]):
	cells CP
	CP.currentCell=neighborCell; #insert the neighbor cell
	CP.fCost=g_score[neighborCell]+calculateHCost(neighborCell,goalCell)
	OPL.insert(CP)
	#multiset<cells>it = OPL.lower_bound(CP)
	#multiset<cells>it = OPL.upper_bound(CP)
	#OPL.insert( it, CP  )


  '''******************************************************************************
 * Function Name: findFreeNeighborCell
 * Inputs: the row and columun of the current Cell
 * Output: a vector of free neighbor cells of the current cell
 * Description:it is used to find the free neighbors Cells of a the current Cell in the grid
 * Check Status: Checked by Anis, and Sahar
********************************************************************************'''

vector <int> RAstarPlannerROS.findFreeNeighborCell (int CellID) 
  int rowID=getCellRowID(CellID)
  int colID=getCellColID(CellID)
  int neighborIndex
  vector <int>  freeNeighborCells

  for (int i=-1;i<=1;i++)
    for (int j=-1; j<=1;j++)      #check whether the index is valid
     if (rowID+i>=0)and(rowID+i<height)and(colID+j>=0)and(colID+j<width)and (not (i==0 and j==0)):	neighborIndex = getCellIndex(rowID+i,colID+j)
        if isFree(neighborIndex) :
	    freeNeighborCells.push_back(neighborIndex)


    return  freeNeighborCells
 


'''*****************************************************************************'''
#Function Name: isStartAndGoalCellsValid
#Inputs: the start and Goal cells
#Output: True if the start and the goal cells are valid
#Description: check if the start and goal cells are valid
'''*******************************************************************************'''
def isStartAndGoalCellsValid(self, startCell, goalCell):
{ 
 bool isvalid=True
 bool isFreeStartCell=isFree(startCell)
 bool isFreeGoalCell=isFree(goalCell)
    if startCell==goalCell:
    #cout << "The Start and the Goal cells are the same..." << endl; 
    isvalid = False

   else:
      if not isFreeStartCell and not isFreeGoalCell:
	#cout << "The start and the goal cells are obstacle positions..." << endl
        isvalid = False

      else:
	if not isFreeStartCell:
	  #cout << "The start is an obstacle..." << endl
	  isvalid = False

	else:
	    if not isFreeGoalCell:
	      #cout << "The goal cell is an obstacle..." << endl
	      isvalid = False

	    else:
	      if findFreeNeighborCell(goalCell).size()==0:
		#cout << "The goal cell is encountred by obstacles... "<< endl
		isvalid = False

	      else:
		if findFreeNeighborCell(startCell).size()==0:
		  #cout << "The start cell is encountred by obstacles... "<< endl
		  isvalid = False






 return isvalid



 float  RAstarPlannerROS.getMoveCost(int i1, j1, i2, j2)   float moveCost=INFINIT_COST;#start cost with maximum value. Change it to real cost of cells are connected
   #if cell2(i2,j2) exists in the diagonal of cell1(i1,j1)
   if (j2==j1+1 and i2==i1+1)or(i2==i1-1 and j2==j1+1) or(i2==i1-1 and j2==j1-1)or(j2==j1-1 and i2==i1+1):     #moveCost = DIAGONAL_MOVE_COST
     moveCost = 1.4

    #if cell 2(i2,j2) exists in the horizontal or vertical line with cell1(i1,j1)
   else:
     if (j2==j1 and i2==i1-1)or(i2==i1 and j2==j1-1)or(i2==i1+1 and j2==j1) or(i1==i2 and j2==j1+1):       #moveCost = MOVE_COST
       moveCost = 1


   return moveCost
 } 
 
  float  RAstarPlannerROS.getMoveCost(int CellID1, CellID2)   int i1=0,i2=0,j1=0,j2=0
    
   i1=getCellRowID(CellID1)
   j1=getCellColID(CellID1)
   i2=getCellRowID(CellID2)
   j2=getCellColID(CellID2)
    
    return getMoveCost(i1, j1, i2, j2)
 } 


 #verify if the cell(i,j) is free
 bool  RAstarPlannerROS.isFree(int i, j)   CellID = getCellIndex(i, j)
 return OGM[CellID]

 } 

  #verify if the cell(i,j) is free
 bool  RAstarPlannerROS.isFree(int CellID) return OGM[CellID]
 } 



bool operator<(cells  &c1, &c2) { return c1.fCost < c2.fCost;