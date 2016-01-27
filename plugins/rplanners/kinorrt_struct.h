#include <cstdlib>
#include <cassert>
#include <cmath>
#include <arc_utilities/simple_rrt_planner.hpp>

//##############################################################################
//build up the simple_rrt structures
//##############################################################################
using namespace simple_rrt_planner;

struct Configuration{
        double x;
        double y;
        double t;
        Configuration(double xs,double ys,double ts){x=xs;y=ys;t=ts;};
        Configuration(){x=-1;y=-1;t=0.0;};
};
inline std::ostream& operator<< (std::ostream& stream, const Configuration& q) {
        std::cout << "(" << q.x << "," << q.y << "," << q.t << ")";
        return stream;
}

typedef std::vector<Configuration> ConfigurationPath;
typedef std::map<std::string,double> Statistics;

typedef std::pair<ConfigurationPath, Statistics> ResultsPair;

void dump_results(const ResultsPair &res){
        ConfigurationPath tau = res.first;
        Statistics stats = res.second;
        std::cout << "---------------------------------------------------" << std::endl;
        std::cout << "Generated trajectory" << std::endl;
        std::cout << "---------------------------------------------------" << std::endl;
        std::cout << "Waypoints: " << tau.size() << std::endl;
        std::cout << "---------------------------------------------------" << std::endl;
        for ( Statistics::const_iterator it = stats.begin(); it != stats.end(); it++) {
                cout << it->first << ":" << it->second << endl;
        }
}


//##############################################################################
// (1) start - starting configuration
//##############################################################################

const Configuration start(-1,-1,0);
const Configuration goal(2,2,0);

SimpleRRTPlannerState<Configuration> plannerstate(start);

//##############################################################################
// (3) goal_reached_fn - return if a given state meets the goal 
//      conditions (for example, within a radius of the goal state)
//##############################################################################
bool goal_reached_fn( const Configuration &q ){
        double epsilon = 0.01;
        std::cout << epsilon << std::endl;
        std::cout << "START" << std::endl;
        std::cout << goal << std::endl;
        std::cout << q << std::endl;
        double dx = (goal.x-q.x)*(goal.x-q.x);
        double dy = (goal.y-q.y)*(goal.y-q.y);
        double dt = (goal.t-q.t)*(goal.t-q.t);
        double dg = sqrtf(dx+dy+dt);
        std::cout << "##################" << dx <<std::endl;
        std::cout << "##################" << dy <<std::endl;
        std::cout << "##################" << dg <<std::endl;
        return (dg < epsilon);
}

//##############################################################################
// (4) state_sampling_fn - returns a new state 
//      (randomly- or deterministically-sampled)
//      std::function<T(void)>& state_sampling_fn
//##############################################################################

double rand_interval( double a, double b){
        //return r \in [a,b]
        assert(b>=a);
        double r = ((double)rand()/(double)RAND_MAX);
        double rab = r*(b-a)+a;
        return rab;

}
Configuration state_sampling_fn(void){

        srand (time(NULL));
        const double x_min = -3.0;
        const double x_max = 3.0;
        const double y_min = -3.0;
        const double y_max = 3.0;
        const double t_min = 0.0;
        const double t_max = 2*M_PI;

        double rx = rand_interval(x_min,x_max);
        double ry = rand_interval(y_min,y_max);
        double rt = rand_interval(t_min,t_max);

        Configuration q = Configuration(rx,ry,rt);
        return q;
}

//##############################################################################
// (5) forward_propagation_fn - given the nearest neighbor and a new 
//      target state, returns the states that would grow the tree 
//      towards the target
//##############################################################################
typedef std::pair<Configuration, int64_t> ConfigurationWaypoint;

std::vector<ConfigurationWaypoint> 
forward_propagation_fn(const Configuration &nn, const Configuration &goal){
        //expand tree from nn -> goal using our transition functon
        
        double lambda = 0.05;
        double dx = lambda*(goal.x - nn.x);
        double dy = lambda*(goal.y - nn.y);
        double dt = lambda*(goal.t - nn.t);
        Configuration qn(nn.x+dx,nn.y+dy,nn.t+dt);
        ConfigurationWaypoint qnwp;
        qnwp.first = qn;
        qnwp.second = 0;
        std::vector<ConfigurationWaypoint> waypoints;
        waypoints.push_back(qnwp);
        return waypoints;
}

//##############################################################################
// (6) time_limit - limit, in seconds, for the runtime of the planner
//##############################################################################

const std::chrono::duration<double> time_limit(1000.0);



//##############################################################################
// (2) nearest_neighbor_fn - given all nodes explored so far, and a 
//      new state, return the index of the "closest" node
//##############################################################################
int64_t
nearest_neighbor_fn
(const std::vector<SimpleRRTPlannerState<Configuration>> &nodes, const Configuration &q){


}

/*
         * Arguments:
         * (1) start - starting configuration
         * (2) nearest_neighbor_fn - given all nodes explored so far, and a new state, return the index of the "closest" node
         * (3) goal_reached_fn - return if a given state meets the goal conditions (for example, within a radius of a goal state)
         * (4) state_sampling_fn - returns a new state (randomly- or deterministically-sampled)
         * (5) forward_propagation_fn - given the nearest neighbor and a new target state, returns the states that would grow the tree towards the target
         * (6) time_limit - limit, in seconds, for the runtime of the planner
         *
         * Returns:
         * std::pair<path, statistics>
         * path - vector of states corresponding to the planned path
         * statistics - map of string keys/double values of planner statistics (i.e. run time, #states explored, #states in solution
        template<typename T, typename Allocator=std::allocator<T> >
        static std::pair<std::vector<T>, std::map<std::string, double> > Plan(const T& start,
                                                                      std::function<int64_t(const std::vector<SimpleRRTPlannerState<T, Allocator> >&, const T&)>& nearest_neighbor_fn,
                                                                      std::function<bool(const T&)>& goal_reached_fn,
                                                                      std::function<T(void)>& sampling_fn,
                                                                      std::function<std::vector<std::pair<T, int64_t> >(const T&, const T&)>& forward_propagation_fn,
                                                                      const std::chrono::duration<double>& time_limit)
        */
