#include <cstdlib>
#include <cassert>
#include <cmath>
#include <openrave/openrave.h>
#include <openrave/plannerparameters.h>
#include <openrave/planner.h>
#include <arc_utilities/simple_rrt_planner_vanilla.hpp>
#include "rplanners.h"
#include "rrt.h"

#define DEBUG 0
using namespace simple_rrt_planner;
const double dtime = 0.02;
const double Z_DEFAULT = 0.1; //default z value for robot, such that it is not in collision with floor (almost touching)
const double Z_INSIDE_FLOOR = -0.1; //make sure that robot is in collision with floor
const std::chrono::duration<double> time_limit(80000.0);
const double epsilon = 0.2; //epsilon region around goal

//##############################################################################
//build up the simple_rrt structures
//##############################################################################


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

        const double x_min = -5.0;
        const double x_max = 7.0;
        const double y_min = -3.0;
        const double y_max = 3.0;
        const double z_min = Z_DEFAULT;
        const double z_max = Z_DEFAULT;
        const double t_min = 0;
        const double t_max = 2*M_PI;

        const double dx_min = -1.0;
        const double dx_max = 1.0;
        const double dy_min = -1.0;
        const double dy_max = 1.0;
        const double dz_min = 0.0;
        const double dz_max = 0.0;
        const double dt_min = -1.0;
        const double dt_max = 1.0;

        double rx = rand_interval(x_min,x_max);
        double ry = rand_interval(y_min,y_max);
        double rz = rand_interval(z_min,z_max);
        double rt = rand_interval(t_min,t_max);

        double rdx = rand_interval(dx_min,dx_max);
        double rdy = rand_interval(dy_min,dy_max);
        double rdz = rand_interval(dz_min,dz_max);
        double rdt = rand_interval(dt_min,dt_max);

        Configuration q = Configuration(rx,ry,rz,rt,rdx,rdy,rdz,rdt);

        Configuration::samples++;
        while(q.IsInCollision()){
                Configuration::samples_rejected++;
                rx = rand_interval(x_min,x_max);
                ry = rand_interval(y_min,y_max);
                rz = rand_interval(z_min,z_max);
                rt = rand_interval(t_min,t_max);
                rdx = rand_interval(dx_min,dx_max);
                rdy = rand_interval(dy_min,dy_max);
                rdz = rand_interval(dz_min,dz_max);
                rdt = rand_interval(dt_min,dt_max);
                q = Configuration(rx,ry,rz,rt,rdx,rdy,rdz,rdt);
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

        //#####################################################################
        //GET FORCES from cell in which robot currently resides
        //#####################################################################
        std::vector<double> dd = from.toDouble();
        dd[2] = Z_INSIDE_FLOOR;
        Configuration::robot->SetDOFValues(dd);
        std::vector<double> force = Configuration::env->GetForceXYZ( from.x, from.y, from.z );
        dd[2] = Z_DEFAULT;
        Configuration::robot->SetDOFValues(dd);
        //#####################################################################
        // GET RANDOM CONTROLS
        //#####################################################################
        const double a_lin_min = -0.5;
        const double a_lin_max = 0.5;
        const double a_ang_min = -0.5;
        const double a_ang_max = 0.5;
        const double a_lie_min = -0.5;
        const double a_lie_max = 0.5;

        double a1 = rand_interval(a_lin_min,a_lin_max);
        double a2 = rand_interval(a_ang_min,a_ang_max);
        double a3 = rand_interval(a_lie_min,a_lie_max);

        //#####################################################################
        // CREATE FORCE FIELDS x_{k+1} = f(x_k,u_k)
        //#####################################################################
        std::vector<double> X1; //LINEAR VEL FIELD
        X1.push_back(cos(from.t));
        X1.push_back(sin(from.t));
        X1.push_back(0.0);
        X1.push_back(0.0);

        std::vector<double> X2; //ANGULAR VEL FIELD
        X2.push_back(0.0);
        X2.push_back(0.0);
        X2.push_back(0.0);
        X2.push_back(1.0);

        std::vector<double> X3; //LIE BRACKET [X1,X2]
        X3.push_back(sin(from.t));
        X3.push_back(-cos(from.t));
        X3.push_back(0.0);
        X3.push_back(0.0);

        std::vector<double> X4; //FORCE FIELD
        X4.push_back(force.at(0));
        X4.push_back(force.at(1));
        X4.push_back(force.at(2));
        X4.push_back(0.0);
        //#####################################################################

        double ddx = a1*X1.at(0) + a2*X2.at(0) + a3*X3.at(0) + X4.at(0);
        double ddy = a1*X1.at(1) + a2*X2.at(1) + a3*X3.at(1) + X4.at(1);
        double ddz = a1*X1.at(2) + a2*X2.at(2) + a3*X3.at(2) + X4.at(2);
        double ddt = a1*X1.at(3) + a2*X2.at(3) + a3*X3.at(3) + X4.at(3);

        // x = ddx * dt^2/2 + dx * dt

        double dx = from.dx;
        double dy = from.dy;
        double dz = from.dz;
        double dt = from.dt;

        double dxn = ddx*dtime + dx;
        double dyn = ddy*dtime + dy;
        double dzn = ddz*dtime + dz;
        double dtn = ddt*dtime + dt;

        double xn = ddx*dtime*dtime/2 + dx*dtime + from.x;
        double yn = ddy*dtime*dtime/2 + dy*dtime + from.y;
        double zn = ddz*dtime*dtime/2 + dz*dtime + from.z;
        double tn = ddt*dtime*dtime/2 + dt*dtime + from.t;

        next = Configuration(xn,yn,zn,tn,dxn,dyn,dzn,dtn);

        //std::cout << "expanding: " << from.x << "->" << xn << std::endl;
        //bool success = Configuration::params->_checkpathconstraintsfn(from.toDouble(), next.toDouble(), IT_OpenStart,PlannerBase::ConfigurationListPtr());
        if( next.IsInCollision() ){
            return false;
        }

        float fwidth = 2.0f;
        RaveVector<float> vcolor(0.5,0.0,1.0,1);

        vector<RaveVector<float> > vpts;
        RaveVector<float> rf(from.x,from.y,from.z);
        RaveVector<float> nf(next.x,next.y,next.z);

        vpts.push_back(rf);
        vpts.push_back(nf);
        //Configuration::handle.push_back(Configuration::env->drawlinestrip(points, numpts, stride, fwidth, &vcolor[0]));
        Configuration::handle.push_back(
                        Configuration::env->drawlinestrip(&vpts[0].x, vpts.size(), sizeof(vpts[0]), fwidth, &vcolor[0])
                        );

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

        int N = 10;
        for(int i = 1;i<N;i++){
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
