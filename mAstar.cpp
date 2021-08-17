
#include <iostream>
#include <vector>
#include <cmath>


//using namespace std;

class state
{
    public:
    float x;
    float y;
    float theta;

    state()
    {
        x=0;
        y=0;
        theta=0;
    }

    state(float _x,float _y,float t)
    {
        x    =  _x;
        y    =  _y;
        theta =  t;
    }


};

class path
{
    public:
    std::vector<state> states;
    double cost;
    float time;

    path()
    {
        cost = 0;
    }


};


class mAstar
{
    public:
    //_______________________________________Paths
    std::vector <path> paths;

    //_______________________________________Parameters
    std::vector<std::vector<int>> cost_map;
    int height;
    int width;
    state start;
    state goal;

    //_______________________________________Penalties
    float start_penalty;
    float goal_penalty;
    float time_penalty;
    float theta_penalty;
    float avoidance_penalty;
    std::vector< int > gradient;


    void mAstar_info()
    {
        return;
    }

    mAstar( std::vector< std::vector<int> > _cm, int start_x, int start_y, float theta, int goal_x, int goal_y)
    {
        cost_map = _cm;
        height = _cm.size();
        width = _cm[0].size();

        start.x = start_x;
        start.y = start_y;
        start.theta = theta;

        goal.x = goal_x;
        goal.y = goal_y;

        gradient.assign({ 4, 0, 4 });//{ 2, 8, 32, 128, 0, 128, 32, 8, 2 });


        start_penalty     = 0;
        goal_penalty      = 5;
        time_penalty      = 1;
        theta_penalty     = 0;
        avoidance_penalty = 1;
    }

    void set_penalties(float sp,float gp,float tip,float thp,float  ap)
    {
        start_penalty = sp;
        goal_penalty = gp;
        time_penalty = tip;
        theta_penalty = thp;
        avoidance_penalty = ap;
    }

    bool state_compare(path p1,path p2)
    {
        return p1.cost > p2.cost; //------------------------------------------------------->check
    }

    double calc_state_cost(path p, float time, int cell_cost)
    {
        float total_cost = 0;
        float theta_cost = 0;
        float avoidance_cost = 0;
        float time_cost = 0;
        float start_cost = 0;
        float goal_cost = 0;


        start_cost = std::sqrt( std::pow(p.states[ p.states.size() - 1 ].x - start.x, 2 ) + std::pow(p.states[ p.states.size() - 1 ].y - start.y, 2 ) ) * start_penalty;
        goal_cost  = std::sqrt( std::pow(p.states[ p.states.size() - 1 ].x - goal.x, 2 ) + std::pow(p.states[ p.states.size() - 1 ].y - goal.y, 2 ) ) * goal_penalty;


        if(p.states.size()>1)
        {
            theta_cost = theta_penalty * fabs( p.states[ p.states.size() - 1 ].theta - p.states[ p.states.size() - 2 ].theta );
        }

        time_cost = time * ( goal_cost / goal_penalty) / (std::sqrt( std::pow( start.x - goal.x, 2 ) + std::pow( start.x - goal.x, 2 ) ) );



        total_cost = start_cost + goal_cost + time_cost + theta_cost + avoidance_cost;

        return total_cost;
    }

    void gradient_map()
    {
        std::vector<std::vector<int>> gradx(height, std::vector<int>(width, 0));
        std::vector<std::vector<int>> grady(height, std::vector<int>(width, 0));

        //std::cout<<height<<" "<<width<<std::endl;

        int gra = gradient.size()/2;

        for(int i=0;i<cost_map.size();i++)
        {
            for( int j=0;j<cost_map[0].size();j++)
            {
                for(int k=gra * -1; k<=gra;k++)
                {
                    if(j+k<0 || j+k>=width)
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

        /*

        for(auto& i: gradx)
    {
        for(auto& j: i)
        {
            std::cout<<j<<"\t";
        }
        std::cout<<std::endl;
    }std::cout<<std::endl;std::cout<<std::endl;
    */

        for(int i=0;i<cost_map.size();i++)
        {
            for( int j=0;j<cost_map[0].size();j++)
            {
                for(int k=gra * -1; k<=gra;k++)
                {
                    if(i+k<0 || i+k>=height)
                    {
                        continue;
                    }
                    if(cost_map[i][j]>0)
                    {
                        continue;
                    }
                    grady[i][j] += std::min(cost_map[i+k][j],1)*gradient[k+gra];
                }
            }
        }
/*
        for(auto& i: grady)
    {
        for(auto& j: i)
        {
            std::cout<<j<<"\t";
        }
        std::cout<<std::endl;
    }std::cout<<std::endl;std::cout<<std::endl;
*/
        for(int i=0;i<cost_map.size();i++)
        {
            for(int j=0;j<cost_map[0].size();j++)
            {
                cost_map[i][j] = (int)std::sqrt(std::pow(gradx[i][j],2)+std::pow(grady[i][j],2)) + cost_map[i][j];
            }
        }
    }

};



int main()

{
    std::vector<std::vector<int>> cost_map(10, std::vector<int>(10, 0));
    mAstar m(cost_map,0,0,0,10,10);
    for(int i=4;i<7;i++){m.cost_map[i][4]=9;}

    for(auto& i: m.cost_map)
    {
        for(auto& j: i)
        {
            std::cout<<j<<"\t";
        }
        std::cout<<std::endl;
    }std::cout<<std::endl;std::cout<<std::endl;


    m.gradient_map();


    for(auto& i: m.cost_map)
    {
        for(auto& j: i)
        {
            std::cout<<j<<"\t";
        }
        std::cout<<std::endl;
    }
}
