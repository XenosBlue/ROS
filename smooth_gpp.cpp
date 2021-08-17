#include "smooth_gpp.h"
#include <plugin/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(smooth_plugin::smooth_planner, nav_core::BaseGlobalPlanner)

vector<vector<int>> cost_map;
namespace smooth_plugin
{

	smooth_planner::smooth_planner()
	{
	}
	
	smooth_planner::smooth_planner(ros::NodeHandle &nh)
	{
		ROSNodeHandle = nh;
	}
	
	smooth_planner::smooth_planner(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
	{
	  initialize(name, costmap_ros);
	}
	
	
	void smooth_plugin::initialize(std::string name, costmap_2d::Costmap2DROS *costmap_ros)
	{
		if (!initialized_)
		{
		    costmap_ros_ = costmap_ros;
		    costmap_ = costmap_ros_->getCostmap();

		    ros::NodeHandle private_nh("~/" + name);

		    originX = costmap_->getOriginX();
		    originY = costmap_->getOriginY();

		    width = costmap_->getSizeInCellsX();
		    height = costmap_->getSizeInCellsY();
		    
		    
		    resolution = costmap_->getResolution();
		    mapSize = width * height;
		    

		    std::vector<std::vector<int>> c_map(height, std::vector<int>(width, 0));
		    for (int i = 0; i < height; i++)
		    {
		      for (int j = 0; j < width; j++)
		      {
			int cost = static_cast<int>(costmap_->getCost(j, i));

			if (cost == 0)
			  c_map[i][j] = 0;
			else
			  c_map[i][j] = 1;
		      }
		    }
		    
		    cost_map = c_map;

		    ROS_INFO("planner initialized.");
		    initialized_ = true;
		}
		else
		  {
		    ROS_WARN("Already initialised");
		  }
	}
	
	bool RAstarPlannerROS::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan)
	{

  		if (!initialized_)
  		{
    			ROS_WARN("The planner not been initialized");
    			return false;
  		}



		plan.clear();

		tf::Stamped < tf::Pose > goal_tf;
		tf::Stamped < tf::Pose > start_tf;

		poseStampedMsgToTF(goal, goal_tf);
		poseStampedMsgToTF(start, start_tf);


		float startX = start.pose.position.x;
		float startY = start.pose.position.y;

		float goalX = goal.pose.position.x;
		float goalY = goal.pose.position.y;
		
		makeCorrdinate(startX, startY);
		makeCorrdinate(goalX, goalY);

		int startCell;
		int goalCell;
		
		goalX = getGridSquareColIndex(index) * resolution;

		goalY = getGridSquareRowIndex(index) * resolution;

		goalX = goalX + originX;
		goalX = goalX + originY;
		
		startX = getGridSquareColIndex(index) * resolution;

		startY = getGridSquareRowIndex(index) * resolution;

		startX = startX + originX;
		startY = startY + originY;
		
		find_path pathfinder(width,height,100.0f);
		
		pathfinder.cost_map = cost_map;
		pathfinder.generateNodes();
		
		auto nodePath = pathfinder.search(pathfinder.getID(startPoint),pathfinder.getID(endPoint));
		
		
		for (int i = 0; i < nodePath.size(); i++)
      		{

			float x = 0.0;
			float y = 0.0;

			float previous_x = 0.0;
			float previous_y = 0.0;

			int index = nodePath[i];
			int previous_index;
			getGridSquareCoordinates(index, x, y);

			if (i != 0)
			{
			  previous_index = bestPath[i - 1];
			}
			else
			{
			  previous_index = index;
			}

			getGridSquareCoordinates(previous_index, previous_x, previous_y);

			tf::Vector3 vectorToTarget;
			vectorToTarget.setValue(x - previous_x, y - previous_y, 0.0);
			float angle = atan2((double)vectorToTarget.y(), (double)vectorToTarget.x());

			geometry_msgs::PoseStamped pose = goal;

			pose.pose.position.x = x;
			pose.pose.position.y = y;
			pose.pose.position.z = 0.0;

			pose.pose.orientation = tf::createQuaternionMsgFromYaw(angle);

			plan.push_back(pose);
	      }

		
		

		  

		return true;
	}
	
	
	
	void RAstarPlannerROS::getCorrdinate(float& x, float& y)
	{

	  x = (int)(x - originX);
	  y = (int)(y - originY);

	}
	
	void AStarPlanner::getGridSquareCoordinates(int index, float &x, float &y)
	{

	  x = getGridSquareColIndex(index) * resolution;

	  y = getGridSquareRowIndex(index) * resolution;

	  x = x + originX;
	  y = y + originY;
	}
	
	
	
	struct Node
	{
		std::vector<std::pair<int,float>> neighbors;
		int search_id = 0;
		int parent;
		float g;
		float h;
		std::string list_mode;
	};


	struct HeapElement
	    {
		int id;
		float g;
		int f;
		bool operator<(const HeapElement& h1) const       {
		    if(abs(f - h1.f) < 0.00001f)
		    {
		        return g < h1.g;
		    }
		    return f > h1.f;
		}
	    };

	    bool heap_compare(HeapElement rhs, HeapElement lhs)
	    {
		    if(abs(lhs.f - rhs.f) < 0.00001f)
		    {
		        return lhs.g < rhs.g;
		    }
		    return lhs.f > rhs.f;
	    }


	class find_path
	{
	    public:




	    int number_of_nodes;
	    float weight;
	    int current_search = 0;


	    std::vector<Node> nodes;
	    std::vector<HeapElement> open_list;


	    int map_width;
	    int map_height;


	    std::vector<int> gradient;
	    std::vector<std::vector<int>> cost_map;


	    static constexpr const float EPSILON = 0.00001f;
	    static constexpr float INFINITE_COST = std::numeric_limits<float>::max();


	    find_path(int h,int w,float we)
	    {
		weight = we;
		map_height=h;
		map_width=w;
		number_of_nodes = map_height * map_width;
		//generateNodes();
	    }

	    void gradient_map()
	    {
		std::vector<std::vector<int>> gradx(map_height, std::vector<int>(map_width, 0));
		std::vector<std::vector<int>> grady(map_height, std::vector<int>(map_width, 0));

		//<<height<<" "<<width<<std::endl;

		int gra = gradient.size()/2;

		for(int i=0;i<cost_map.size();i++)
		{
		    for( int j=0;j<cost_map[0].size();j++)
		    {
		        for(int k=gra * -1; k<=gra;k++)
		        {
		            if(j+k<0 || j+k>=map_width)
		            {
		                continue;
		            }
		            if(cost_map[i][j]>1)
		            {
		                continue;
		            }
		            gradx[i][j] += std::min(cost_map[i][j+k],1)*gradient[k+gra];
		        }
		    }
		}
	    }

	    std::pair<int, int> getCoordinate(int a)
	    {
		std::pair<int,int> coordinate;
		coordinate.first = a / map_width;
		coordinate.second = a % map_width;
		return coordinate;
	    }

	    int getID(std::pair<int,int> coordinate)
	    {
		return coordinate.first * map_width + coordinate.second;
	    }

	    float distance(int a, int b)
	    {
		return std::sqrt(std::pow(abs(a/map_width - b/map_width),2) + std::pow(abs(a%map_width - b%map_width),2) );
	    }

	    std::vector<std::pair<int, float>> adjacent(int id)
	    {
		auto coordinate = getCoordinate(id);
		float cost = 1;

		std::vector<std::pair<int, float>> neighbors;


		if(coordinate.first != 0 && isValid({coordinate.first - 1, coordinate.second}))
		{
		    neighbors.push_back({getID({coordinate.first - 1, coordinate.second}), cost});
		}

		if(coordinate.second != 0 && isValid({coordinate.first, coordinate.second - 1}))
		{
		    neighbors.push_back({getID({coordinate.first, coordinate.second - 1}), cost});
		}

		if(coordinate.first < (map_width - 1) && isValid({coordinate.first + 1, coordinate.second}))
		{
		    neighbors.push_back({getID({coordinate.first + 1, coordinate.second}), cost});
		}


		if(coordinate.second < map_height - 1 && isValid({coordinate.first, coordinate.second + 1}))
		{
		    neighbors.push_back({getID({coordinate.first, coordinate.second + 1}), cost});
		}


		return neighbors;
	    }

	    bool isValid(std::pair<int,int> coordinate)
	    {
		return (cost_map[coordinate.first][coordinate.second]==0);
	    }

	    bool isValid(int a)
	    {
		return (cost_map[a/map_width][a%map_width] == 0);
	    }

	    bool lineOfSight(int n1, int n2)
	    {

		std::pair<int,int> l1 = getCoordinate(n1);
		std::pair<int,int> l2 = getCoordinate(n2);

		std::pair<int,int> diff = { (l2.first-l1.first) , (l2.second- l1.second)};

		int f = 0;
		std::pair<int,int> dir;
		std::pair<int,int> offset;

		if(diff.second < 0)
		{
		    diff.second = -diff.second;
		    dir.second = -1;
		    offset.second = 0;
		}
		else
		{
		    dir.second = 1;
		    offset.second = 1;
		}

		if(diff.first < 0)
		{
		    diff.first = -diff.first;
		    dir.first = -1;
		    offset.first = 0;
		}
		else
		{
		    dir.first = 1;
		    offset.first = 1;
		}

		if(diff.first >= diff.second)
		{
		    while(l1.first != l2.first)
		    {
		        f += diff.second;
		        if(f >= diff.first)
		        {
		            if (!isValid({l1.first + offset.first,l1.second+offset.second}))
		            {
		                return false;
		            }

		            l1.second += dir.second;
		            f -= diff.first;
		        }

		        if(f != 0 && !isValid({l1.first + offset.first,l1.second+offset.second}))
		        {
		            return false;
		        }

		        if (diff.second == 0 && !isValid({l1.first + offset.first, l1.second}) && !isValid({l1.first + offset.first, l1.second + 1}))
		        {
		            return false;
		        }

		        l1.first += dir.first;
		    }
		}
		else
		{
		    while (l1.second != l2.second)
		    {
		        f += diff.first;
		        if(f >= diff.second)
		        {
		            if(!isValid({l1.first + offset.first,l1.second+offset.second}))
		            {
		                return false;
		            }

		            l1.first += dir.first;
		            f -= diff.second;
		        }

		        if(f != 0 && !isValid({l1.first + offset.first,l1.second+offset.second}))
		        {
		            return false;
		        }

		        if (diff.first == 0 && !isValid({l1.first, l1.second + offset.second}) && !isValid({l1.first + 1, l1.second + offset.second}))
		        {
		            return false;
		        }

		        l1.second += dir.second;
		    }
		}

		return true;
	    }

	    void generate_state(int node, int goal)
	    {
		if(nodes[node].search_id != current_search)
		{
		    nodes[node].search_id = current_search;
		    nodes[node].h = distance(node, goal) * weight;
		    nodes[node].g = INFINITE_COST;
		    nodes[node].list_mode = "no_list";
		}
	    }

	    void append_to_open_list(int id)
	    {

		if (nodes[id].list_mode == "open_list")
		{
		    for(std::vector<HeapElement>::iterator each=open_list.begin();each!=open_list.end();each++)
		    {
		        if((*each).id==id)
		        {
		            open_list.erase(each);
		            insert_sorted(open_list, {id, nodes[id].g, nodes[id].g + nodes[id].h});
		        }
		    }

		}

		else
		{
		    nodes[id].list_mode = "open_list";
		    insert_sorted(open_list, {id, nodes[id].g, nodes[id].g + nodes[id].h});
		}

	    }

	    std::vector<HeapElement>::iterator insert_sorted( std::vector<HeapElement>& vec, HeapElement item )
	    {
		auto pos = std::upper_bound( vec.begin(), vec.end(), item );
		return vec.insert(pos,item);
	    }



	    HeapElement getMin()
	    {
		return open_list.back();
	    }

	    void popMin()
	    {
		nodes[open_list.back().id].list_mode = "closed_list";
		open_list.pop_back();

	    }

	    void generateNodes()
	    {
		nodes.clear();
		nodes.resize(number_of_nodes);

		int n = 0;
		for(auto& node : nodes)
		{
		    node.neighbors = adjacent(n++);
		}
	    }







	    std::vector<int> search(int startId, int endId)
	    {
		open_list.clear();

		current_search++;

		generate_state(startId, endId);
		generate_state(endId, endId);

		nodes[startId].g = 0;
		nodes[startId].parent = startId;

		append_to_open_list(startId);

		while(!open_list.empty() && nodes[endId].g > getMin().f + EPSILON)
		{
		    int currId = getMin().id;
		    popMin();

		    if(!lineOfSight(nodes[currId].parent, currId))
		    {
		        nodes[currId].g = INFINITE_COST;

		        for(auto neighbordInfo : nodes[currId].neighbors)
		        {
		            auto newParent = neighbordInfo.first;

		            generate_state(newParent, endId);
		            if(nodes[newParent].list_mode == "closed_list")
		            {
		                float newG = nodes[newParent].g + neighbordInfo.second;
		                if(newG < nodes[currId].g)
		                {
		                    nodes[currId].g = newG;
		                    nodes[currId].parent = newParent;
		                }
		            }
		        }
		    }

		    for(const auto neighborInfo : nodes[currId].neighbors)
		    {
		        auto neighborId = neighborInfo.first;

		        generate_state(neighborId, endId);

		        int newParent = nodes[currId].parent;

		        if(nodes[neighborId].list_mode != "closed_list")
		        {
		            float newG = nodes[newParent].g + distance(newParent, neighborId);

		            if(newG + EPSILON < nodes[neighborId].g)
		            {
		                nodes[neighborId].g = newG;
		                nodes[neighborId].parent = newParent;
		                append_to_open_list(neighborId);
		            }
		        }
		    }
		}

		std::vector<int> path;

		if(nodes[endId].g < INFINITE_COST)
		{
		    int curr = endId;
		    while(curr != startId)
		    {
		        path.push_back(curr);
		        curr = nodes[curr].parent;
		    }

		    path.push_back(curr);
		    std::reverse(path.begin(), path.end());
		}

		return path;
	    }

	};





}


