#pragma once

#include "rrt.h"

class KinodynamicRRTPlanner : public RrtPlanner<SimpleNode>
{
public:
    KinodynamicRRTPlanner(EnvironmentBasePtr penv) : RrtPlanner<SimpleNode>(penv)
    {
        __description = "Kino RRT planner";
        _fGoalBiasProb = dReal(0.05);
        _bOneStep = false;
    }
    virtual ~KinodynamicRRTPlanner() {
    }

    bool InitPlan(RobotBasePtr pbase, PlannerParametersConstPtr pparams)
    {
        EnvironmentMutex::scoped_lock lock(GetEnv()->GetMutex());
        _parameters.reset(new KinodynamicRRTParameters());
        _parameters->copy(pparams);
        if( !RrtPlanner<SimpleNode>::_InitPlan(pbase,_parameters) ) {
            _parameters.reset();
            return false;
        }

        CollisionOptionsStateSaver optionstate(GetEnv()->GetCollisionChecker(),GetEnv()->GetCollisionChecker()->GetCollisionOptions()|CO_ActiveDOFs,false);

        //read in all goals
        int goal_index = 0;
        vector<dReal> vgoal(_parameters->GetDOF());
        _vecGoals.resize(0);
        while(_parameters->vgoalconfig.size() > 0) {
            for(int i = 0; i < _parameters->GetDOF(); i++) {
                if(goal_index < (int)_parameters->vgoalconfig.size())
                    vgoal[i] = _parameters->vgoalconfig[goal_index];
                else {
                    RAVELOG_ERROR("KinodynamicRRT::InitPlan - Error: goals are improperly specified:\n");
                    _parameters.reset();
                    return false;
                }
                goal_index++;
            }

            if( GetParameters()->CheckPathAllConstraints(vgoal,vgoal, std::vector<dReal>(), std::vector<dReal>(), 0, IT_OpenStart) == 0 ) {
                _vecGoals.push_back(vgoal);
            }
            else {
                RAVELOG_WARN("goal in collision\n");
            }

            if(goal_index == (int)_parameters->vgoalconfig.size()) {
                break;
            }
        }

        if(( _vecGoals.size() == 0) && !_parameters->_goalfn ) {
            RAVELOG_WARN("no goals or goal function specified\n");
            _parameters.reset();
            return false;
        }

        _bOneStep = _parameters->_nRRTExtentType == 1;
        RAVELOG_DEBUG("KinodynamicRRT::InitPlan - RRT Planner Initialized\n");
        return true;
    }

    PlannerStatus PlanPath(TrajectoryBasePtr ptraj)
    {
        if(!_parameters) {
            RAVELOG_WARN("KinodynamicRRT::PlanPath - Error, planner not initialized\n");
            return PS_Failed;
        }

        EnvironmentMutex::scoped_lock lock(GetEnv()->GetMutex());
        uint32_t basetime = utils::GetMilliTime();

        NodeBase* lastnode = NULL; // the last node visited by the RRT
        NodeBase* bestGoalNode = NULL; // the best goal node found already by the RRT. If this is not NULL, then RRT succeeded
        dReal fBestGoalNodeDist = 0; // configuration distance from initial position to the goal node

        // the main planning loop
        PlannerParameters::StateSaver savestate(_parameters);
        CollisionOptionsStateSaver optionstate(GetEnv()->GetCollisionChecker(),GetEnv()->GetCollisionChecker()->GetCollisionOptions()|CO_ActiveDOFs,false);

        std::vector<dReal> vtempinitialconfig;
        PlannerAction callbackaction = PA_None;
        PlannerProgress progress;
        int iter = 0;
        _goalindex = -1; // index into vgoalconfig if the goal is found
        _startindex = -1;

        while(iter < _parameters->_nMaxIterations) {
            iter++;
            if( !!bestGoalNode && iter >= _parameters->_nMinIterations ) {
                break;
            }
            if( !!_parameters->_samplegoalfn ) {
                vector<dReal> vgoal;
                if( _parameters->_samplegoalfn(vgoal) ) {
                    RAVELOG_VERBOSE("found goal\n");
                    _vecGoals.push_back(vgoal);
                }
            }
            if( !!_parameters->_sampleinitialfn ) {
                vector<dReal> vinitial;
                if( _parameters->_sampleinitialfn(vinitial) ) {
                    RAVELOG_VERBOSE("found initial\n");
                    _vecInitialNodes.push_back(_treeForward.InsertNode(NULL, vinitial, _vecInitialNodes.size()));
                }
            }

            if( (iter == 1 || RaveRandomFloat() < _fGoalBiasProb ) && _vecGoals.size() > 0 ) {
                _sampleConfig = _vecGoals[RaveRandomInt()%_vecGoals.size()];
            }
            else if( !_parameters->_samplefn(_sampleConfig) ) {
                continue;
            }

            // extend A
            ExtendType et = _treeForward.Extend(_sampleConfig, lastnode, _bOneStep);

            if( et == ET_Connected ) {
                FOREACH(itgoal, _vecGoals) {
                    if( _parameters->_distmetricfn(*itgoal, _treeForward.GetVectorConfig(lastnode)) < 2*_parameters->_fStepLength ) {
                        SimpleNode* pforward = (SimpleNode*)lastnode;
                        while(1) {
                            if(!pforward->rrtparent) {
                                break;
                            }
                            pforward = pforward->rrtparent;
                        }
                        vtempinitialconfig = _treeForward.GetVectorConfig(pforward); // GetVectorConfig returns the same reference, so need to make a copy
                        dReal fGoalNodeDist = _parameters->_distmetricfn(vtempinitialconfig, _treeForward.GetVectorConfig(lastnode));
                        if( !bestGoalNode || fBestGoalNodeDist > fGoalNodeDist ) {
                            bestGoalNode = lastnode;
                            fBestGoalNodeDist = fGoalNodeDist;
                            _goalindex = (int)(itgoal-_vecGoals.begin());
                        }
                        if( iter >= _parameters->_nMinIterations ) {
                            RAVELOG_DEBUG_FORMAT("env=%d, found goal index: %d", GetEnv()->GetId()%_goalindex);
                            break;
                        }
                    }
                }
            }

            // check the goal heuristic more often
            if(( et != ET_Failed) && !!_parameters->_goalfn ) {
                if( _parameters->_goalfn(_treeForward.GetVectorConfig(lastnode)) <= 1e-4f ) {
                    SimpleNode* pforward = (SimpleNode*)lastnode;
                    while(1) {
                        if(!pforward->rrtparent) {
                            break;
                        }
                        pforward = pforward->rrtparent;
                    }
                    vtempinitialconfig = _treeForward.GetVectorConfig(pforward); // GetVectorConfig returns the same reference, so need to make a copy
                    dReal fGoalNodeDist = _parameters->_distmetricfn(vtempinitialconfig, _treeForward.GetVectorConfig(lastnode));
                    if( !bestGoalNode || fBestGoalNodeDist > fGoalNodeDist ) {
                        bestGoalNode = lastnode;
                        fBestGoalNodeDist = fGoalNodeDist;
                        _goalindex = -1;
                        RAVELOG_DEBUG_FORMAT("env=%d, found node at goal at dist=%f at %d iterations, computation time %fs", GetEnv()->GetId()%fBestGoalNodeDist%iter%(0.001f*(float)(utils::GetMilliTime()-basetime)));
                    }
                    if( iter >= _parameters->_nMinIterations ) {
                        break;
                    }
                }
            }

            // check if reached any goals
            if( iter > _parameters->_nMaxIterations ) {
                RAVELOG_WARN("iterations exceeded %d\n", _parameters->_nMaxIterations);
                break;
            }

            progress._iteration = iter;
            callbackaction = _CallCallbacks(progress);
            if( callbackaction ==  PA_Interrupt ) {
                return PS_Interrupted;
            }
            else if( callbackaction == PA_ReturnWithAnySolution ) {
                if( !!bestGoalNode ) {
                    break;
                }
            }
        }

        if( !bestGoalNode ) {
            RAVELOG_DEBUG_FORMAT("plan failed, %fs",(0.001f*(float)(utils::GetMilliTime()-basetime)));
            return PS_Failed;
        }

        const int dof = _parameters->GetDOF();
        _cachedpath.resize(0);

        // add nodes from the forward tree
        SimpleNode* pforward = (SimpleNode*)bestGoalNode;
        while(1) {
            _cachedpath.insert(_cachedpath.begin(), pforward->q, pforward->q+dof);
            if(!pforward->rrtparent) {
                break;
            }
            pforward = pforward->rrtparent;
        }

        _SimpleOptimizePath(_cachedpath,10);
        if( ptraj->GetConfigurationSpecification().GetDOF() == 0 ) {
            ptraj->Init(_parameters->_configurationspecification);
        }
        std::vector<dReal> vinsertvalues(_cachedpath.begin(), _cachedpath.end());
        ptraj->Insert(ptraj->GetNumWaypoints(), vinsertvalues, _parameters->_configurationspecification);

        PlannerStatus status = _ProcessPostPlanners(_robot,ptraj);
        RAVELOG_DEBUG_FORMAT("env=%d, plan success, path=%d points in %fs", GetEnv()->GetId()%ptraj->GetNumWaypoints()%((0.001f*(float)(utils::GetMilliTime()-basetime))));
        return status;
    }

    virtual PlannerParametersConstPtr GetParameters() const {
        return _parameters;
    }

    virtual bool _DumpTreeCommand(std::ostream& os, std::istream& is) {
        std::string filename = RaveGetHomeDirectory() + string("/KinodynamicRRTdump.txt");
        getline(is, filename);
        boost::trim(filename);
        RAVELOG_VERBOSE(str(boost::format("dumping rrt tree to %s")%filename));
        ofstream f(filename.c_str());
        f << std::setprecision(std::numeric_limits<dReal>::digits10+1);
        _treeForward.DumpTree(f);
        return true;
    }
protected:
    boost::shared_ptr<KinodynamicRRTParameters> _parameters;
    dReal _fGoalBiasProb;
    bool _bOneStep;
    std::vector< std::vector<dReal> > _vecGoals;
    int _nValidGoals; ///< num valid goals
};

