#pragma once

#include <boost/algorithm/string.hpp>
#include "kinorrt_struct.h"
#include "rplanners.h"
#include "rrt.h"

class KinodynamicRRTPlanner : public RrtPlanner<SimpleNode>
{
public:
    KinodynamicRRTPlanner(EnvironmentBasePtr penv) : RrtPlanner<SimpleNode>(penv)
    {
        __description += "Kinodynamic RRT";
    }
    virtual ~KinodynamicRRTPlanner() {
    }

    //struct GOALPATH
    //{
    //    GOALPATH() : startindex(-1), goalindex(-1), length(0) {
    //    }
    //    vector<dReal> qall;
    //    int startindex, goalindex;
    //    dReal length;
    //};

    virtual bool InitPlan(RobotBasePtr pbase, PlannerParametersConstPtr pparams)
    {
        EnvironmentMutex::scoped_lock lock(GetEnv()->GetMutex());
        _robot = pbase;

        _parameters.reset(new KinodynamicRRTParameters());
        _parameters->copy(pparams);
        if( !RrtPlanner<SimpleNode>::_InitPlan(pbase,_parameters) ) {
            _parameters.reset();
            return false;
        }

        return true;
    }

    virtual PlannerStatus PlanPath(TrajectoryBasePtr ptraj)
    {
        //deterministic results
        srand(0);


        if(!_parameters) {
            RAVELOG_ERROR("KinodynamicRRTPlanner::PlanPath - Error, planner not initialized\n");
            return PS_Failed;
        }
        std::vector<double> init = _parameters->vinitialconfig;
        std::vector<double> goal = _parameters->vgoalconfig;

        Configuration start_config(init.at(0),init.at(1),0.0);
        Configuration goal_config(goal.at(0),goal.at(1),0.0);

        std::cout << start_config << std::endl;
        std::cout << goal_config << std::endl;

        std::cout << _robot->GetName() << std::endl;
        Configuration::env = GetEnv();
        Configuration::robot = _robot;

        std::pair<std::vector<Configuration>, std::map<std::string,double> > results;

        std::function<int64_t(const std::vector<SimpleRRTPlannerState<Configuration>>&, const Configuration&)> nn = &(nearest_neighbor_fn);
        std::function<bool(const Configuration&, const Configuration&)> gg = &(goal_reached_fn);
        std::function<Configuration()> ss = &(state_sampling_fn);
        std::function<std::vector<std::pair<Configuration, int64_t> >(const Configuration&, const Configuration&)> fp = &(forward_propagation_fn);

        EnvironmentMutex::scoped_lock lock(GetEnv()->GetMutex());


        results = SimpleHybridRRTPlanner<Configuration>::Plan( 
                        start_config, 
                        goal_config,
                        nn,
                        gg,
                        ss,
                        fp,
                        time_limit);

        dump_results(results);
        ConfigurationPath tau = results.first;

        int Nwaypoints = tau.size();
        if( Nwaypoints == 0 ){ return PS_Failed; }
        

        if( ptraj->GetConfigurationSpecification().GetDOF() == 0 ) {
            ptraj->Init(_parameters->_configurationspecification);
        }
        std::cout << "DOF:" << ptraj->GetConfigurationSpecification().GetDOF() << std::endl;
        std::cout << "Cspecification:" << _parameters->_configurationspecification << std::endl;
        std::cout << "WPS:" << Nwaypoints << std::endl;


        std::vector<dReal> tau_wps = toDouble(tau);
        std::cout<< "TAU_WPS: " << tau_wps.size() << std::endl;
        ptraj->Insert(0, tau_wps, _parameters->_configurationspecification);

        int Ndof = ptraj->GetConfigurationSpecification().GetDOF();

        for( int i = 0; i<Ndof; i++){
                std::cout << init.at(i) << "|";
        }


        return _ProcessPostPlanners(_robot,ptraj);
    }
    virtual PlannerParametersConstPtr GetParameters() const {
        return _parameters;
    }


protected:
    KinodynamicRRTParametersPtr _parameters;
};

//#ifdef RAVE_REGISTER_BOOST
//#include BOOST_TYPEOF_INCREMENT_REGISTRATION_GROUP()
//BOOST_TYPEOF_REGISTER_TYPE(KinodynamicRRTPlanner::GOALPATH)
//#endif
