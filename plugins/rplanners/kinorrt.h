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





        //#####################################################################
        // simple_rrt
        //#####################################################################
        

        //ptraj->Insert(ptraj->GetNumWaypoints(), itbest->qall, _parameters->_configurationspecification);
        //RAVELOG_DEBUG_FORMAT("env=%d, plan success, iters=%d, path=%d points, computation time=%fs\n", GetEnv()->GetId()%progress._iteration%ptraj->GetNumWaypoints()%(0.001f*(float)(utils::GetMilliTime()-basetime)));
        //return _ProcessPostPlanners(_robot,ptraj);
        //SimpleHybridRRTPlanner::Plan<Configuration>( 
        //                start, 
        //                nearest_neighbor_fn,
        //                goal_reached_fn, 
        //                state_sampling_fn,
        //                forward_propagation_fn,
        //                time_limit);
        
        //#####################################################################
        return true;
    }

    virtual PlannerStatus PlanPath(TrajectoryBasePtr ptraj)
    {
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

        //SimpleHybridRRTPlanner<Configuration>::setGoal(goal_config);

        srand(0);
        std::pair<std::vector<Configuration>, std::map<std::string,double> > results;

        std::function<int64_t(const std::vector<SimpleRRTPlannerState<Configuration>>&, const Configuration&)> nn = &(nearest_neighbor_fn);
        std::function<bool(const Configuration&, const Configuration&)> gg = &(goal_reached_fn);
        std::function<Configuration()> ss = &(state_sampling_fn);
        std::function<std::vector<std::pair<Configuration, int64_t> >(const Configuration&, const Configuration&)> fp = &(forward_propagation_fn);

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
        
        EnvironmentMutex::scoped_lock lock(GetEnv()->GetMutex());

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

//    virtual void _ExtractPath(GOALPATH& goalpath, NodeBase* iConnectedForward, NodeBase* iConnectedBackward)
//    {
//
//        const int dof = _parameters->GetDOF();
//        _cachedpath.resize(0);
//
//        // add nodes from the forward tree
//        SimpleNode* pforward = (SimpleNode*)iConnectedForward;
//        goalpath.startindex = -1;
//        while(1) {
//            _cachedpath.insert(_cachedpath.begin(), pforward->q, pforward->q+dof);
//            //vecnodes.push_front(pforward);
//            if(!pforward->rrtparent) {
//                goalpath.startindex = pforward->_userdata;
//                break;
//            }
//            pforward = pforward->rrtparent;
//        }
//
//        // add nodes from the backward tree
//        goalpath.goalindex = -1;
//        SimpleNode *pbackward = (SimpleNode*)iConnectedBackward;
//        while(1) {
//            //vecnodes.push_back(pbackward);
//            _cachedpath.insert(_cachedpath.end(), pbackward->q, pbackward->q+dof);
//            if(!pbackward->rrtparent) {
//                goalpath.goalindex = pbackward->_userdata;
//                break;
//            }
//            pbackward = pbackward->rrtparent;
//        }
//
//        BOOST_ASSERT( goalpath.goalindex >= 0 && goalpath.goalindex < (int)_vecGoalNodes.size() );
//        _SimpleOptimizePath(_cachedpath,10);
//        goalpath.qall.resize(_cachedpath.size());
//        std::copy(_cachedpath.begin(), _cachedpath.end(), goalpath.qall.begin());
//        goalpath.length = 0;
//        vector<dReal> vivel(dof,1.0);
//        for(size_t i = 0; i < vivel.size(); ++i) {
//            if( _parameters->_vConfigVelocityLimit.at(i) != 0 ) {
//                vivel[i] = 1/_parameters->_vConfigVelocityLimit.at(i);
//            }
//        }
//
//        // take distance scaled with respect to velocities with the first and last points only!
//        // this is because rrt paths can initially be very complex but simplify down to something simpler.
//        std::vector<dReal> vdiff(goalpath.qall.begin(), goalpath.qall.begin()+dof);
//        _parameters->_diffstatefn(vdiff, std::vector<dReal>(goalpath.qall.end()-dof, goalpath.qall.end()));
//        for(size_t i = 0; i < vdiff.size(); ++i) {
//            goalpath.length += RaveFabs(vdiff.at(i))*vivel.at(i);
//        }
//    }
//
    virtual PlannerParametersConstPtr GetParameters() const {
        return _parameters;
    }


protected:
    KinodynamicRRTParametersPtr _parameters;
    //SpatialTree< SimpleNode > _treeBackward;
    //dReal _fGoalBiasProb;
    //std::vector< NodeBase* > _vecGoalNodes;
    //size_t _nValidGoals; ///< num valid goals
    //std::vector<GOALPATH> _vgoalpaths;
};

//#ifdef RAVE_REGISTER_BOOST
//#include BOOST_TYPEOF_INCREMENT_REGISTRATION_GROUP()
//BOOST_TYPEOF_REGISTER_TYPE(KinodynamicRRTPlanner::GOALPATH)
//#endif
