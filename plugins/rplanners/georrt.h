#pragma once

#include <boost/algorithm/string.hpp>
#include "georrt_struct.h"

class GeoRRTPlanner : public RrtPlanner<SimpleNode>
{
public:
    GeoRRTPlanner(EnvironmentBasePtr penv) : RrtPlanner<SimpleNode>(penv)
    {
        __description += "geometrical RRT";
    }
    virtual ~GeoRRTPlanner() {
    }

    virtual bool InitPlan(RobotBasePtr pbase, PlannerParametersConstPtr pparams)
    {
        using namespace georrt;
        EnvironmentMutex::scoped_lock lock(GetEnv()->GetMutex());
        _robot = pbase;

        _parameters.reset(new GeoRRTParameters());
        _parameters->copy(pparams);
        if( !RrtPlanner<SimpleNode>::_InitPlan(pbase,_parameters) ) {
            _parameters.reset();
            return false;
        }

        return true;
    }

    virtual PlannerStatus PlanPath(TrajectoryBasePtr ptraj)
    {
        using namespace georrt;
        //deterministic results
        srand(0);


        if(!_parameters) {
            RAVELOG_ERROR("GeoRRTPlanner::PlanPath - Error, planner not initialized\n");
            return PS_Failed;
        }
        std::vector<double> init = _parameters->vinitialconfig;
        std::vector<double> goal = _parameters->vgoalconfig;

        std::vector<double> init_cspace;
        std::vector<double> goal_cspace;

        for(int i=0;i<4;i++){
                init_cspace.push_back(init.at(i));
                goal_cspace.push_back(goal.at(i));
        }

        Configuration start_config(init.at(0),init.at(1),init.at(2),init.at(3),init.at(4),init.at(5),init.at(6),init.at(7));
        Configuration goal_config(goal.at(0),goal.at(1),goal.at(2),goal.at(3),goal.at(4),goal.at(5),goal.at(6),goal.at(7));

        std::cout << "Start : " << start_config << std::endl;
        std::cout << "Goal  : " << goal_config << std::endl;
        std::cout << _robot->GetName() << std::endl;

        //#####################################################################
        // Set Configuration statics
        //#####################################################################

        Configuration::env = GetEnv();
        Configuration::robot = _robot;
        Configuration::params = _parameters;

        //#####################################################################
        // Set functions for simple_rrt
        //#####################################################################
        std::pair<std::vector<Configuration>, std::map<std::string,double> > results;
        std::function<int64_t(const std::vector<SimpleRRTPlannerState<Configuration>>&, const Configuration&)> nn = &(nearest_neighbor_fn);
        std::function<bool(const Configuration&, const Configuration&)> gg = &(goal_reached_fn);
        std::function<Configuration()> ss = &(state_sampling_fn);
        std::function<std::vector<std::pair<Configuration, int64_t> >(const Configuration&, const Configuration&)> fp = &(forward_propagation_fn);

        EnvironmentMutex::scoped_lock lock(GetEnv()->GetMutex());

        //#####################################################################
        // Verify initial parameters
        //#####################################################################
            //std::list<KinBodyPtr> listCheckCollisions; listCheckCollisions.push_back(robot);
            //boost::shared_ptr<DynamicsCollisionConstraint> pcollision(new DynamicsCollisionConstraint(params, listCheckCollisions, 0xffffffff&~CFO_CheckTimeBasedConstraints));
            //params->_checkpathvelocityconstraintsfn = boost::bind(&DynamicsCollisionConstraint::Check,pcollision,_1, _2, _3, _4, _5, _6, _7, _8);

        //if( !_parameters->_checkpathconstraintsfn(init_cspace,init_cspace,IT_OpenStart,ConfigurationListPtr()) ) {
        //    RAVELOG_WARN("Error: Initial configuration not in free space\n");
        //    return PS_Failed;
        //}
        if( start_config.IsInCollision() ){
            RAVELOG_WARN("initial state rejected by constraint fn\n");
            return PS_Failed;
        }
        if( goal_config.IsInCollision() ){
            RAVELOG_WARN("goal state rejected by constraint fn\n");
            return PS_Failed;
        }

        std::cout << "############################################" << std::endl;
        std::cout << "PARAMETERS" << std::endl;
        std::cout << "############################################" << std::endl;
        std::cout << "configs   : " << _parameters->_configurationspecification << std::endl;
        
            //- joint_values
            //- joint_velocities
            //- affine_transform
            //- affine_velocities
            //- grab
            //The following internal parameters will be set:
            //- _diffstatefn
            //- _distmetricfn - weights used for distance metric are retrieved at this time and stored
            //- _samplefn
            //- _sampleneighfn
            //- _setstatevaluesfn
            //- _getstatefn
            //- _neighstatefn
            //- _checkpathconstraintsfn
            //- _vConfigLowerLimit
            //- _vConfigUpperLimit
            //- _vConfigVelocityLimit
            //- _vConfigAccelerationLimit
            //- _vConfigResolution
            //- vinitialconfig
            //- _configurationspecification

        //#####################################################################
        // Planning
        //#####################################################################
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
        //return PS_HasSolution;
    }
    virtual PlannerParametersConstPtr GetParameters() const {
        return _parameters;
    }


protected:
    GeoRRTParametersPtr _parameters;
};

//#ifdef RAVE_REGISTER_BOOST
//#include BOOST_TYPEOF_INCREMENT_REGISTRATION_GROUP()
//BOOST_TYPEOF_REGISTER_TYPE(GeoRRTPlanner::GOALPATH)
//#endif
