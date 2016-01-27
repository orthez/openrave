#pragma once

#include <boost/algorithm/string.hpp>
#include "kinorrt_struct.h"
#include "rplanners.h"
#include "rrt.h"

class KinodynamicRRTPlanner : public RrtPlanner<SimpleNode>
{
public:
    KinodynamicRRTPlanner(EnvironmentBasePtr penv) : RrtPlanner<SimpleNode>(penv), _treeBackward(1)
    {
        __description += "Kinodynamic RRT";
        _nValidGoals = 0;
    }
    virtual ~KinodynamicRRTPlanner() {
    }

    struct GOALPATH
    {
        GOALPATH() : startindex(-1), goalindex(-1), length(0) {
        }
        vector<dReal> qall;
        int startindex, goalindex;
        dReal length;
    };

    virtual bool InitPlan(RobotBasePtr pbase, PlannerParametersConstPtr pparams)
    {
        EnvironmentMutex::scoped_lock lock(GetEnv()->GetMutex());


        _parameters.reset(new KinodynamicRRTParameters());
        _parameters->copy(pparams);
        if( !RrtPlanner<SimpleNode>::_InitPlan(pbase,_parameters) ) {
            _parameters.reset();
            return false;
        }

        _robot = pbase;




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
        Configuration x = state_sampling_fn();
        std::cout << x << std::endl;

        srand(0);

        std::pair<std::vector<Configuration>, std::map<std::string,double> > results;

        std::function<int64_t(const std::vector<SimpleRRTPlannerState<Configuration>>&, const Configuration&)> nn = &(nearest_neighbor_fn);
        std::function<bool(const Configuration&)> gg = &(goal_reached_fn);
        std::function<Configuration()> ss = &(state_sampling_fn);
        std::function<std::vector<std::pair<Configuration, int64_t> >(const Configuration&, const Configuration&)> fp = &(forward_propagation_fn);

        results = SimpleHybridRRTPlanner::Plan<Configuration>( 
                        start, 
                        nn,
                        gg,
                        ss,
                        fp,
                        time_limit);

        dump_results(results);
        
        EnvironmentMutex::scoped_lock lock(GetEnv()->GetMutex());




        //_goalindex = -1;
        //_startindex = -1;
        //if(!_parameters) {
        //    RAVELOG_ERROR("KinodynamicRRTPlanner::PlanPath - Error, planner not initialized\n");
        //    return PS_Failed;
        //}

        //EnvironmentMutex::scoped_lock lock(GetEnv()->GetMutex());
        //uint32_t basetime = utils::GetMilliTime();

        //// the main planning loop
        //PlannerParameters::StateSaver savestate(_parameters);
        //CollisionOptionsStateSaver optionstate(GetEnv()->GetCollisionChecker(),GetEnv()->GetCollisionChecker()->GetCollisionOptions()|CO_ActiveDOFs,false);

        //SpatialTreeBase* TreeA = &_treeForward;
        //SpatialTreeBase* TreeB = &_treeBackward;
        //NodeBase* iConnectedA=NULL, *iConnectedB=NULL;
        //int iter = 0;

        //bool bSampleGoal = true;
        //PlannerProgress progress;
        //PlannerAction callbackaction=PA_None;
        //while(_vgoalpaths.size() < _parameters->_minimumgoalpaths && iter < 3*_parameters->_nMaxIterations) {
        //    RAVELOG_VERBOSE_FORMAT("iter=%d, forward=%d, backward=%d", (iter/3)%_treeForward.GetNumNodes()%_treeBackward.GetNumNodes());
        //    ++iter;

        //    if( !!_parameters->_samplegoalfn ) {
        //        vector<dReal> vgoal;
        //        if( _parameters->_samplegoalfn(vgoal) ) {
        //            RAVELOG_VERBOSE(str(boost::format("inserting new goal index %d")%_vecGoalNodes.size()));
        //            _vecGoalNodes.push_back(_treeBackward.InsertNode(NULL, vgoal, _vecGoalNodes.size()));
        //            _nValidGoals++;
        //        }
        //    }
        //    if( !!_parameters->_sampleinitialfn ) {
        //        vector<dReal> vinitial;
        //        if( _parameters->_sampleinitialfn(vinitial) ) {
        //            RAVELOG_VERBOSE(str(boost::format("inserting new initial %d")%_vecInitialNodes.size()));
        //            _vecInitialNodes.push_back(_treeForward.InsertNode(NULL,vinitial, _vecInitialNodes.size()));
        //        }
        //    }

        //    _sampleConfig.resize(0);
        //    if( (bSampleGoal || _uniformsampler->SampleSequenceOneReal() < _fGoalBiasProb) && _nValidGoals > 0 ) {
        //        bSampleGoal = false;
        //        // sample goal as early as possible
        //        uint32_t bestgoalindex = -1;
        //        for(size_t testiter = 0; testiter < _vecGoalNodes.size()*3; ++testiter) {
        //            uint32_t sampleindex = _uniformsampler->SampleSequenceOneUInt32();
        //            uint32_t goalindex = sampleindex%_vecGoalNodes.size();
        //            if( !_vecGoalNodes.at(goalindex) ) {
        //                continue; // dummy
        //            }
        //            // make sure goal is not already found
        //            bool bfound = false;
        //            FOREACHC(itgoalpath,_vgoalpaths) {
        //                if( goalindex == (uint32_t)itgoalpath->goalindex ) {
        //                    bfound = true;
        //                    break;
        //                }
        //            }
        //            if( !bfound) {
        //                bestgoalindex = goalindex;
        //                break;
        //            }
        //        }
        //        if( bestgoalindex != uint32_t(-1) ) {
        //            _treeBackward.GetVectorConfig(_vecGoalNodes.at(bestgoalindex), _sampleConfig);
        //        }
        //    }

        //    if( _sampleConfig.size() == 0 ) {
        //        if( !_parameters->_samplefn(_sampleConfig) ) {
        //            continue;
        //        }
        //    }

        //    // extend A
        //    ExtendType et = TreeA->Extend(_sampleConfig, iConnectedA);

        //    // although check isn't necessary, having it improves running times
        //    if( et == ET_Failed ) {
        //        // necessary to increment iterator in case spaces are not connected
        //        if( iter > 3*_parameters->_nMaxIterations ) {
        //            RAVELOG_WARN("iterations exceeded\n");
        //            break;
        //        }
        //        continue;
        //    }

        //    et = TreeB->Extend(TreeA->GetVectorConfig(iConnectedA), iConnectedB);     // extend B toward A

        //    if( et == ET_Connected ) {
        //        // connected, process goal
        //        _vgoalpaths.push_back(GOALPATH());
        //        _ExtractPath(_vgoalpaths.back(), TreeA == &_treeForward ? iConnectedA : iConnectedB, TreeA == &_treeBackward ? iConnectedA : iConnectedB);
        //        int goalindex = _vgoalpaths.back().goalindex;
        //        int startindex = _vgoalpaths.back().startindex;
        //        if( IS_DEBUGLEVEL(Level_Debug) ) {
        //            stringstream ss; ss << std::setprecision(std::numeric_limits<dReal>::digits10+1);
        //            ss << "env=" << GetEnv()->GetId() << ", found a goal, start index=" << startindex << " goal index=" << goalindex << ", path length=" << _vgoalpaths.back().length << ", values=[";
        //            for(int i = 0; i < _parameters->GetDOF(); ++i) {
        //                ss << _vgoalpaths.back().qall.at(_vgoalpaths.back().qall.size()-_parameters->GetDOF()+i) << ", ";
        //            }
        //            ss << "]";
        //            RAVELOG_DEBUG(ss.str());
        //        }
        //        if( _vgoalpaths.size() >= _parameters->_minimumgoalpaths || _vgoalpaths.size() >= _nValidGoals ) {
        //            break;
        //        }
        //        bSampleGoal = true;
        //        // more goals requested, so make sure to remove all the nodes pointing to the current found goal
        //        _treeBackward.InvalidateNodesWithParent(_vecGoalNodes.at(goalindex));
        //    }

        //    swap(TreeA, TreeB);
        //    iter += 3;
        //    if( iter > 3*_parameters->_nMaxIterations ) {
        //        RAVELOG_WARN("iterations exceeded\n");
        //        break;
        //    }

        //    progress._iteration = iter/3;
        //    callbackaction = _CallCallbacks(progress);
        //    if( callbackaction ==  PA_Interrupt ) {
        //        return PS_Interrupted;
        //    }
        //    else if( callbackaction == PA_ReturnWithAnySolution ) {
        //        if( _vgoalpaths.size() > 0 ) {
        //            break;
        //        }
        //    }
        //}

        //if( _vgoalpaths.size() == 0 ) {
        //    RAVELOG_WARN("plan failed, %fs\n",0.001f*(float)(utils::GetMilliTime()-basetime));
        //    return PS_Failed;
        //}

        //vector<GOALPATH>::iterator itbest = _vgoalpaths.begin();
        //FOREACH(itpath,_vgoalpaths) {
        //    if( itpath->length < itbest->length ) {
        //        itbest = itpath;
        //    }
        //}
        //_goalindex = itbest->goalindex;
        //_startindex = itbest->startindex;
        //if( ptraj->GetConfigurationSpecification().GetDOF() == 0 ) {
        //    ptraj->Init(_parameters->_configurationspecification);
        //}
        //ptraj->Insert(ptraj->GetNumWaypoints(), itbest->qall, _parameters->_configurationspecification);
        //RAVELOG_DEBUG_FORMAT("env=%d, plan success, iters=%d, path=%d points, computation time=%fs\n", GetEnv()->GetId()%progress._iteration%ptraj->GetNumWaypoints()%(0.001f*(float)(utils::GetMilliTime()-basetime)));
        return _ProcessPostPlanners(_robot,ptraj);
    }

    virtual void _ExtractPath(GOALPATH& goalpath, NodeBase* iConnectedForward, NodeBase* iConnectedBackward)
    {

        const int dof = _parameters->GetDOF();
        _cachedpath.resize(0);

        // add nodes from the forward tree
        SimpleNode* pforward = (SimpleNode*)iConnectedForward;
        goalpath.startindex = -1;
        while(1) {
            _cachedpath.insert(_cachedpath.begin(), pforward->q, pforward->q+dof);
            //vecnodes.push_front(pforward);
            if(!pforward->rrtparent) {
                goalpath.startindex = pforward->_userdata;
                break;
            }
            pforward = pforward->rrtparent;
        }

        // add nodes from the backward tree
        goalpath.goalindex = -1;
        SimpleNode *pbackward = (SimpleNode*)iConnectedBackward;
        while(1) {
            //vecnodes.push_back(pbackward);
            _cachedpath.insert(_cachedpath.end(), pbackward->q, pbackward->q+dof);
            if(!pbackward->rrtparent) {
                goalpath.goalindex = pbackward->_userdata;
                break;
            }
            pbackward = pbackward->rrtparent;
        }

        BOOST_ASSERT( goalpath.goalindex >= 0 && goalpath.goalindex < (int)_vecGoalNodes.size() );
        _SimpleOptimizePath(_cachedpath,10);
        goalpath.qall.resize(_cachedpath.size());
        std::copy(_cachedpath.begin(), _cachedpath.end(), goalpath.qall.begin());
        goalpath.length = 0;
        vector<dReal> vivel(dof,1.0);
        for(size_t i = 0; i < vivel.size(); ++i) {
            if( _parameters->_vConfigVelocityLimit.at(i) != 0 ) {
                vivel[i] = 1/_parameters->_vConfigVelocityLimit.at(i);
            }
        }

        // take distance scaled with respect to velocities with the first and last points only!
        // this is because rrt paths can initially be very complex but simplify down to something simpler.
        std::vector<dReal> vdiff(goalpath.qall.begin(), goalpath.qall.begin()+dof);
        _parameters->_diffstatefn(vdiff, std::vector<dReal>(goalpath.qall.end()-dof, goalpath.qall.end()));
        for(size_t i = 0; i < vdiff.size(); ++i) {
            goalpath.length += RaveFabs(vdiff.at(i))*vivel.at(i);
        }
    }

    virtual PlannerParametersConstPtr GetParameters() const {
        return _parameters;
    }

    virtual bool _DumpTreeCommand(std::ostream& os, std::istream& is) {
        std::string filename = RaveGetHomeDirectory() + string("/kinorrtdump.txt");
        getline(is, filename);
        boost::trim(filename);
        RAVELOG_VERBOSE(str(boost::format("dumping rrt tree to %s")%filename));
        ofstream f(filename.c_str());
        f << std::setprecision(std::numeric_limits<dReal>::digits10+1);
        _treeForward.DumpTree(f);
        _treeBackward.DumpTree(f);
        return true;
    }

protected:
    KinodynamicRRTParametersPtr _parameters;
    //SpatialTree< SimpleNode > _treeBackward;
    dReal _fGoalBiasProb;
    //std::vector< NodeBase* > _vecGoalNodes;
    //size_t _nValidGoals; ///< num valid goals
    std::vector<GOALPATH> _vgoalpaths;
};

//#ifdef RAVE_REGISTER_BOOST
//#include BOOST_TYPEOF_INCREMENT_REGISTRATION_GROUP()
//BOOST_TYPEOF_REGISTER_TYPE(KinodynamicRRTPlanner::GOALPATH)
//#endif
