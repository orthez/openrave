#include <cstdlib>
#include <cassert>
#include <cmath>
#include <arc_utilities/simple_rrt_planner.hpp>

#define DEBUG 0
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
        double dist(const Configuration &qq) const{
                double dx = (x-qq.x)*(x-qq.x);
                double dy = (y-qq.y)*(y-qq.y);
                //double dt = (t-qq.t)*(t-qq.t);
                return sqrtf(dx+dy);
        }
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
        int ctr=0;
        for ( ConfigurationPath::const_iterator it = tau.begin(); it != tau.end(); it++) {
                cout << "[" << ctr++ << "]: " << (*it) << std::endl;
        }
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
        double epsilon = 0.1;
        return (q.dist(goal) < epsilon);
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
        if(DEBUG) std::cout << "SAMPLE:" << q << std::endl;
        return q;
}

//##############################################################################
// (5) forward_propagation_fn - given the nearest neighbor and a new 
//      target state, returns the states that would grow the tree 
//      towards the target
//##############################################################################
typedef std::pair<Configuration, int64_t> ConfigurationWaypoint;

Configuration one_step_forward_propagation(const Configuration &from, const Configuration &to){
        double lambda = 0.05;
        double dx = lambda*(to.x - from.x);
        double dy = lambda*(to.y - from.y);
        double dt = lambda*(to.t - from.t);
        Configuration qn(from.x+dx,from.y+dy,from.t+dt);
        return qn;
}

std::vector<ConfigurationWaypoint> 
forward_propagation_fn(const Configuration &nn, const Configuration &goal){
        //expand tree from nn -> goal using our transition functon
        //TODO: implement real version, not naive linear progress
        int wpctr = -1;

        std::vector<ConfigurationWaypoint> waypoints;

        ConfigurationWaypoint initial;
        initial.first = one_step_forward_propagation(nn,goal);
        initial.second = wpctr++;
        waypoints.push_back(initial);

        if(DEBUG) std::cout << "NN" << nn <<std::endl;

        for(int i = 1;i<5;i++){
                ConfigurationWaypoint from = waypoints[i-1];
                ConfigurationWaypoint qnwp;
                qnwp.first = one_step_forward_propagation(from.first,goal);
                if(DEBUG) std::cout << "[" << wpctr << "]" << qnwp.first <<std::endl;
                qnwp.second = wpctr++;
                waypoints.push_back(qnwp);
        }
        return waypoints;
}

//##############################################################################
// (6) time_limit - limit, in seconds, for the runtime of the planner
//##############################################################################

const std::chrono::duration<double> time_limit(10.0);



//##############################################################################
// (2) nearest_neighbor_fn - given all nodes explored so far, and a 
//      new state, return the index of the "closest" node
//##############################################################################
int64_t
nearest_neighbor_fn
(const std::vector<SimpleRRTPlannerState<Configuration>> &nodes, const Configuration &q){

        std::vector<SimpleRRTPlannerState<Configuration>>::const_iterator it;

        double min_dist = std::numeric_limits<double>::infinity();
        int64_t nnidx = 0;
        int64_t ctr = 0;

        FORIT(it, nodes){
                const Configuration qq = it->GetValueImmutable();
                double dd = qq.dist(q);
                if(dd < min_dist){
                        min_dist = dd;
                        nnidx=ctr;
                }
                ctr++;
        }
        if(DEBUG) std::cout << "NN:" << nnidx << std::endl;
        return nnidx;
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
