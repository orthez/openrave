#include <cstdlib>
#include <cassert>
#include <cmath>
#include <openrave/openrave.h>
#include <openrave/plannerparameters.h>
#include <openrave/planner.h>
#include <arc_utilities/simple_rrt_planner_vanilla.hpp>
//#include <arc_utilities/simple_rrt_planner_interface_vanilla.hpp>
#include "rplanners.h"
#include "rrt.h"

#define DEBUG 0
#define DEBUG_VISUALIZE 0


//##############################################################################
//build up the simple_rrt structures
//##############################################################################
namespace georrt{
        using namespace simple_rrt_planner;
        const double dtime = 0.01;
        const int N_FORWARD_STEPS = 10;

        const double Z_DEFAULT = 0.15; //default z value for robot, such that it is not in collision with floor (almost touching)
        const double Z_INSIDE_FLOOR = -0.1; //make sure that robot is in collision with floor
        const std::chrono::duration<double> time_limit(2000.0);
        const double epsilon = 0.01; //epsilon region around goal

        template<class T>
        std::ostream& operator << (std::ostream& os, const std::vector<T>& v) 
        {
            os << "[";
            for (typename std::vector<T>::const_iterator ii = v.begin(); ii != v.end(); ++ii)
            {
                os << " " << *ii;
            }
            os << "]";
            return os;
        }

        struct Configuration{
                double x;
                double y;
                double z;
                double t;
                double dx;
                double dy;
                double dz;
                double dt;
                //Configuration(double xs,double ys,double ts){x=xs;y=ys;z=Z_DEFAULT;t=ts;};
                Configuration(  double xs,double ys,double zs,double ts,
                                double dxs,double dys,double dzs,double dts)
                {
                        x=xs;y=ys;z=zs;t=ts;
                        dx=dxs;dy=dys;dz=dzs;dt=dts;
                };

                Configuration(){x=0.0;y=0.0;z=0.0;t=0.0;dx=0.0;dy=0.0;dz=0.0;dt=0.0;};

                double dist(const Configuration &qq) const{
                        double dx = (x-qq.x)*(x-qq.x);
                        double dy = (y-qq.y)*(y-qq.y);
                        double dz = (z-qq.z)*(z-qq.z);
                        double dt = min(fabs(t-qq.t),2*M_PI-fabs(t-qq.t));

                        double ddx = (dx-qq.dx)*(dx-qq.dx);
                        double ddy = (dy-qq.dy)*(dy-qq.dy);
                        double ddz = (dz-qq.dz)*(dz-qq.dz);
                        double ddt = min(fabs(dt-qq.dt),2*M_PI-fabs(dt-qq.dt));

                        return sqrtf(dx+dy+dz)+dt + sqrtf(ddx+ddy+ddz) + ddt;
                }
                bool IsInCollision(){
                        std::vector<double> dd = this->toDouble();
                        EnvironmentMutex::scoped_lock lock(Configuration::env->GetMutex());

                        Configuration::robot->SetDOFValues(dd);
                        CollisionReportPtr report(new CollisionReport());
                        bool collided = env->CheckCollision(robot) || robot->CheckSelfCollision();
                        return collided;
                }
                std::vector<double> toDouble() const{
                        std::vector<double> qd;
                        qd.push_back(x);
                        qd.push_back(y);
                        qd.push_back(z);
                        qd.push_back(t);
                        return qd;
                }
                static PlannerBase::PlannerParametersPtr params;
                static EnvironmentBasePtr env;
                static RobotBasePtr robot;
                static std::vector<OpenRAVE::GraphHandlePtr> handle;
                static long int samples;
                static long int samples_rejected;
        };
        PlannerBase::PlannerParametersPtr Configuration::params = NULL;
        EnvironmentBasePtr Configuration::env = NULL;
        RobotBasePtr Configuration::robot = NULL;
        std::vector<OpenRAVE::GraphHandlePtr> Configuration::handle;
        long int Configuration::samples = 0;
        long int Configuration::samples_rejected = 0;

        inline std::ostream& operator<< (std::ostream& os, const Configuration& q) 
        {
                os << "(" << q.x << "," << q.y << "," << q.z << "," << q.t << ")";
                return os;
        }


        typedef std::vector<Configuration> ConfigurationPath;
        typedef std::map<std::string,double> Statistics;
        typedef std::pair<ConfigurationPath, Statistics> ResultsPair;

        std::vector<dReal> toDouble(const ConfigurationPath &tau){

                ConfigurationPath::const_iterator it;
                std::vector<dReal> all;
                FOREACH(it,tau){
                        all.push_back(it->x);
                        all.push_back(it->y);
                        all.push_back(it->z);
                        all.push_back(it->t);
                }
                return all;
        }

        void dump_results(const ResultsPair &res){
                ConfigurationPath tau = res.first;
                Statistics stats = res.second;
                std::cout << "---------------------------------------------------" << std::endl;
                std::cout << "Generated trajectory" << std::endl;
                std::cout << "---------------------------------------------------" << std::endl;
                std::cout << "[begin] : " << tau.front() << std::endl;
                std::cout << "[end]   : " << tau.back() << std::endl;
                //for ( ConfigurationPath::const_iterator it = tau.begin(); it != tau.end(); it++) {
                        //cout << "[" << ctr++ << "]: " << (*it) << std::endl;
                //}
                std::cout << "---------------------------------------------------" << std::endl;
                std::cout << "Waypoints: " << tau.size() << std::endl;
                std::cout << "---------------------------------------------------" << std::endl;
                for ( Statistics::const_iterator it = stats.begin(); it != stats.end(); it++) {
                        cout << it->first << ":" << it->second << endl;
                }
                std::cout << "---------------------------------------------------" << std::endl;
                std::cout << "Sampling" << std::endl;
                std::cout << "---------------------------------------------------" << std::endl;
                std::cout << "Samples: " << Configuration::samples << std::endl;
                std::cout << "Rejected: " << Configuration::samples_rejected << std::endl;
                std::cout << "Percentage Rejected: " << (double)Configuration::samples_rejected/(double)Configuration::samples << std::endl;
                std::cout << "---------------------------------------------------" << std::endl;
        }

        //##############################################################################
        // (1) start - starting configuration
        //##############################################################################
        //const Configuration goal(2,2,0);

        //##############################################################################
        // (3) goal_reached_fn - return if a given state meets the goal 
        //      conditions (for example, within a radius of the goal state)
        //##############################################################################

        bool goal_reached_fn( const Configuration &q, const Configuration &goal ){
                double dx = (q.x - goal.x)*(q.x - goal.x);
                double dy = (q.y - goal.y)*(q.y - goal.y);
                bool reached = (sqrtf(dx+dy) < epsilon);
                if(reached){
                        std::cout << "goal reached" << std::endl;
                }
                return reached;
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

                std::vector<dReal> qlim_l;
                std::vector<dReal> qlim_u;

                Configuration::robot->GetActiveDOFLimits(qlim_l,qlim_u);

                const double x_min = qlim_l.at(0);
                const double x_max = qlim_u.at(0);

                const double y_min = qlim_l.at(1);
                const double y_max = qlim_u.at(1);

                // TODO: no z axis
                const double z_min = qlim_l.at(2);
                const double z_max = qlim_u.at(2);

                const double t_min = qlim_l.at(3);
                const double t_max = qlim_u.at(3);

                double rx = rand_interval(x_min,x_max);
                double ry = rand_interval(y_min,y_max);
                double rz = z_max;
                double rt = rand_interval(t_min,t_max);

                Configuration q = Configuration(rx,ry,rz,rt,0,0,0,0);

                Configuration::samples++;
                while(q.IsInCollision()){
                        Configuration::samples_rejected++;
                        rx = rand_interval(x_min,x_max);
                        ry = rand_interval(y_min,y_max);
                        rz = z_max;
                        rt = rand_interval(t_min,t_max);
                        q = Configuration(rx,ry,rz,rt,0,0,0,0);
                        Configuration::samples++;
                }
                
                if(DEBUG) std::cout << "SAMPLE:" << q << std::endl;
                return q;
        }

        //##############################################################################
        // (5) forward_propagation_fn - given the nearest neighbor and a new 
        //      target state, returns the states that would grow the tree 
        //      towards the target
        //##############################################################################

        typedef std::pair<Configuration, int64_t> ConfigurationWaypoint;

        bool one_step_forward_propagation(const Configuration &from, const Configuration &to, Configuration &next){

                double xi = from.x;
                double yi = from.y;
                double zi = from.z;
                double ti = from.t;

                double xg = to.x;
                double yg = to.y;
                double zg = to.z;
                double tg = to.t;

                double dxn = (xg - xi);
                double dyn = (yg - yi);
                double dzn = 0;
                double dtn = (tg - ti);

                double n = sqrt(dxn*dxn+dyn*dyn);

                double eta = 0.1;
                double xn = xi + eta*dxn;
                double yn = yi + eta*dyn;
                double zn = zi;
                double tn = ti + eta*dtn;

                next = Configuration(xn,yn,zn,tn,0,0,0,0);

                if( next.IsInCollision() ){
                    return false;
                }

                if (DEBUG_VISUALIZE){
                        float fwidth = 2.5f;
                        std::vector<dReal> maxvel;
                        Configuration::robot->GetActiveDOFVelocityLimits(maxvel);
                        const double MAX_VELOCITY = sqrtf(maxvel[0]*maxvel[0] + maxvel[1]*maxvel[1]);
                        double velocity = sqrtf(dxn*dxn + dyn*dyn);
                        double color_scale = velocity/MAX_VELOCITY;

                        RaveVector<float> redish(0.3,0.0,0.4,1);
                        RaveVector<float> blue(0.0,0.0,1.0,1);

                        RaveVector<float> vcolor = color_scale*redish + (1.0-color_scale)*blue;

                        vector<RaveVector<float> > vpts;
                        RaveVector<float> rf(from.x,from.y,from.z+0.05);
                        RaveVector<float> nf(next.x,next.y,next.z+0.05);
                        vpts.push_back(rf);
                        vpts.push_back(nf);
                        //Configuration::handle.push_back(Configuration::env->drawlinestrip(points, numpts, stride, fwidth, &vcolor[0]));
                        Configuration::handle.push_back(
                                        Configuration::env->drawlinestrip(&vpts[0].x, vpts.size(), sizeof(vpts[0]), fwidth, &vcolor[0])
                                        );
                }

                return true;
        }

        std::vector<ConfigurationWaypoint> 
        forward_propagation_fn(const Configuration &nn, const Configuration &goal){
                //expand tree from nn -> goal using our transition functon
                //TODO: implement real version, not naive linear progress
                int wpctr = -1;

                std::vector<ConfigurationWaypoint> waypoints;

                ConfigurationWaypoint initial;
                bool success = one_step_forward_propagation(nn,goal,initial.first);
                if(!success){ return waypoints; }
                initial.second = wpctr++;
                waypoints.push_back(initial);

                for(int i = 1;i<N_FORWARD_STEPS;i++){
                        ConfigurationWaypoint from = waypoints[i-1];
                        ConfigurationWaypoint qnwp;
                        bool success = one_step_forward_propagation(from.first,goal,qnwp.first);
                        if(!success){ break; }
                        if(DEBUG) std::cout << "[" << wpctr << "]" << qnwp.first <<std::endl;
                        qnwp.second = wpctr++;
                        waypoints.push_back(qnwp);
                }
                return waypoints;
        }


        //##############################################################################
        // (2) nearest_neighbor_fn - given all nodes explored so far, and a 
        //      new state, return the index of the "closest" node
        //##############################################################################
        int64_t
        nearest_neighbor_fn
        (const std::vector<SimpleRRTPlannerState<Configuration>> &nodes, const Configuration &q)
        {
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
}
