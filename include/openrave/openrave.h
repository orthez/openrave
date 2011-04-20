// -*- coding: utf-8 -*-
// Copyright (C) 2006-2011 Rosen Diankov (rosen.diankov@gmail.com)
//
// This file is part of OpenRAVE.
// OpenRAVE is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
/** \file rave.h
    \brief  Defines the public headers that every plugin must include in order to use openrave properly.
*/
#ifndef OPENRAVE_H
#define OPENRAVE_H

#ifndef RAVE_DISABLE_ASSERT_HANDLER
#define BOOST_ENABLE_ASSERT_HANDLER
#endif

#include <cstdio>
#include <stdarg.h>
#include <cstring>
#include <cstdlib>
#include <stdint.h>

#ifdef _MSC_VER

#pragma warning(disable:4251) // needs to have dll-interface to be used by clients of class
#pragma warning(disable:4190) // C-linkage specified, but returns UDT 'boost::shared_ptr<T>' which is incompatible with C
#pragma warning(disable:4819) //The file contains a character that cannot be represented in the current code page (932). Save the file in Unicode format to prevent data loss using native typeof

// needed to get typeof working
//#include <boost/typeof/std/string.hpp>
//#include <boost/typeof/std/vector.hpp>
//#include <boost/typeof/std/list.hpp>
//#include <boost/typeof/std/map.hpp>
//#include <boost/typeof/std/set.hpp>
//#include <boost/typeof/std/string.hpp>

#ifndef __PRETTY_FUNCTION__
#define __PRETTY_FUNCTION__ __FUNCDNAME__
#endif

#else
#endif

#include <string>
#include <vector>
#include <list>
#include <map>
#include <set>
#include <string>
#include <exception>

#include <iomanip>
#include <fstream>
#include <sstream>

#include <boost/function.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/weak_ptr.hpp>
#include <boost/tuple/tuple.hpp>
#include <boost/enable_shared_from_this.hpp> 
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>
#include <boost/thread/recursive_mutex.hpp>
#include <boost/static_assert.hpp>
#include <boost/format.hpp>
#include <boost/array.hpp>
#include <boost/multi_array.hpp>
//#include <boost/cstdint.hpp>

#if defined(_WIN32) || defined(__CYGWIN__) || defined(_MSC_VER)
  #define OPENRAVE_HELPER_DLL_IMPORT __declspec(dllimport)
  #define OPENRAVE_HELPER_DLL_EXPORT __declspec(dllexport)
  #define OPENRAVE_HELPER_DLL_LOCAL
#else
  #if __GNUC__ >= 4
    #define OPENRAVE_HELPER_DLL_IMPORT __attribute__ ((visibility("default")))
    #define OPENRAVE_HELPER_DLL_EXPORT __attribute__ ((visibility("default")))
    #define OPENRAVE_HELPER_DLL_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define OPENRAVE_HELPER_DLL_IMPORT
    #define OPENRAVE_HELPER_DLL_EXPORT
    #define OPENRAVE_HELPER_DLL_LOCAL
  #endif
#endif

// Now we use the generic helper definitions above to define OPENRAVE_API and OPENRAVE_LOCAL.
// OPENRAVE_API is used for the public API symbols. It either DLL imports or DLL exports (or does nothing for static build)
// OPENRAVE_LOCAL is used for non-api symbols.
#if defined(OPENRAVE_DLL) || defined(OPENRAVE_CORE_DLL) // defined if OpenRAVE is compiled as a DLL
  #ifdef OPENRAVE_DLL_EXPORTS // defined if we are building the OpenRAVE DLL (instead of using it)
    #define OPENRAVE_API OPENRAVE_HELPER_DLL_EXPORT
  #else
    #define OPENRAVE_API OPENRAVE_HELPER_DLL_IMPORT
  #endif // OPENRAVE_DLL_EXPORTS
  #define OPENRAVE_LOCAL OPENRAVE_HELPER_DLL_LOCAL
#else // OPENRAVE_DLL is not defined: this means OpenRAVE is a static lib.
  #define OPENRAVE_API
  #define OPENRAVE_LOCAL
#endif // OPENRAVE_DLL

#if defined(__GNUC__)
#define RAVE_DEPRECATED __attribute__((deprecated))
#else
#define RAVE_DEPRECATED
#endif

/// The entire %OpenRAVE library
namespace OpenRAVE {
    
#include <openrave/config.h>
#include <openrave/interfacehashes.h>

#if OPENRAVE_PRECISION // 1 if double precision
typedef double dReal;
#define g_fEpsilon 1e-15
#else
typedef float dReal;
#define g_fEpsilon 2e-7f
#endif

/// \brief openrave constant for PI, could be replaced by accurate precision number depending on choice of dReal.
static const dReal PI = (dReal)3.14159265358979323846;

/// Wrappers of common basic math functions, allows OpenRAVE to control the precision requirements.
/// \ingroup affine_math
//@{

/// \brief exponential
OPENRAVE_API dReal RaveExp(dReal f);
/// \brief logarithm
OPENRAVE_API dReal RaveLog(dReal f);
/// \brief cosine
OPENRAVE_API dReal RaveCos(dReal f);
/// \brief sine
OPENRAVE_API dReal RaveSin(dReal f);
/// \brief tangent
OPENRAVE_API dReal RaveTan(dReal f);
/// \brief base 2 logarithm
OPENRAVE_API dReal RaveLog2(dReal f);
/// \brief base 10 logarithm
OPENRAVE_API dReal RaveLog10(dReal f);
/// \brief arccosine
OPENRAVE_API dReal RaveAcos(dReal f);
/// \brief arcsine
OPENRAVE_API dReal RaveAsin(dReal f);
/// \brief arctangent2 covering entire circle
OPENRAVE_API dReal RaveAtan2(dReal fy, dReal fx);
/// \brief power x^y
OPENRAVE_API dReal RavePow(dReal fx, dReal fy);
/// \brief square-root
OPENRAVE_API dReal RaveSqrt(dReal f);
/// \brief absolute value
OPENRAVE_API dReal RaveFabs(dReal f);

//@}

/// %OpenRAVE error codes
enum OpenRAVEErrorCode {
    ORE_Failed=0,
    ORE_InvalidArguments=1,
    ORE_EnvironmentNotLocked=2,
    ORE_CommandNotSupported=3, ///< string command could not be parsed or is not supported
    ORE_Assert=4,
    ORE_InvalidPlugin=5, ///< shared object is not a valid plugin
    ORE_InvalidInterfaceHash=6, ///< interface hashes do not match between plugins
    ORE_NotImplemented=7, ///< function is not implemented by the interface.
};

/// \brief Exception that all OpenRAVE internal methods throw; the error codes are held in \ref OpenRAVEErrorCode.
class OPENRAVE_API openrave_exception : public std::exception
{
public:
    openrave_exception() : std::exception(), _s("unknown exception"), _error(ORE_Failed) {}
    openrave_exception(const std::string& s, OpenRAVEErrorCode error=ORE_Failed) : std::exception() {
        _error = error;
        _s = "openrave (";
        switch(_error) {
        case ORE_Failed: _s += "Failed"; break;
        case ORE_InvalidArguments: _s += "InvalidArguments"; break;
        case ORE_EnvironmentNotLocked: _s += "EnvironmentNotLocked"; break;
        case ORE_CommandNotSupported: _s += "CommandNotSupported"; break;
        case ORE_Assert: _s += "Assert"; break;
        case ORE_InvalidPlugin: _s += "InvalidPlugin"; break;
        case ORE_InvalidInterfaceHash: _s += "InvalidInterfaceHash"; break;
        case ORE_NotImplemented: _s += "NotImplemented"; break;
        default:
            _s += boost::str(boost::format("%8.8x")%static_cast<int>(_error));
            break;
        }
        _s += "): ";
        _s += s; 
    }
    virtual ~openrave_exception() throw() {}
    char const* what() const throw() { return _s.c_str(); }
    const std::string& message() const { return _s; }
    OpenRAVEErrorCode GetCode() const { return _error; }
private:
    std::string _s;
    OpenRAVEErrorCode _error;
};

class OPENRAVE_LOCAL CaseInsensitiveCompare
{
public:
    bool operator()(const std::string & s1, const std::string& s2) const
    {
        std::string::const_iterator it1=s1.begin();
        std::string::const_iterator it2=s2.begin();
        
        //has the end of at least one of the strings been reached?
        while ( (it1!=s1.end()) && (it2!=s2.end()) )  { 
            if(::toupper(*it1) != ::toupper(*it2)) { //letters differ?
                // return -1 to indicate 'smaller than', 1 otherwise
                return ::toupper(*it1) < ::toupper(*it2);
            }
            //proceed to the next character in each string
            ++it1;
            ++it2;
        }
        std::size_t size1=s1.size(), size2=s2.size();// cache lengths
        //return -1,0 or 1 according to strings' lengths
        if (size1==size2) {
            return 0;
        }
        return size1<size2;
    }
};

/// \brief base class for all user data
class OPENRAVE_API UserData
{
 public:
    virtual ~UserData() {} 
};
typedef boost::shared_ptr<UserData> UserDataPtr;

// terminal attributes
//#define RESET           0
//#define BRIGHT          1
//#define DIM             2
//#define UNDERLINE       3
//#define BLINK           4
//#define REVERSE         7
//#define HIDDEN          8
// terminal colors
//#define BLACK           0
//#define RED             1
//#define GREEN           2
//#define YELLOW          3
//#define BLUE            4
//#define MAGENTA         5
//#define CYAN            6
//#define WHITE           7

/// Change the text color (on either stdout or stderr) with an attr:fg:bg (thanks to Radu Rusu for the code)
inline std::string ChangeTextColor (int attribute, int fg, int bg)
{
    char command[13];
    sprintf (command, "%c[%d;%d;%dm", 0x1B, attribute, fg + 30, bg + 40);
    return command;
}

/// Change the text color (on either stdout or stderr) with an attr:fg (thanks to Radu Rusu for the code)
inline std::string ChangeTextColor (int attribute, int fg)
{
    char command[13];
    sprintf (command, "%c[%d;%dm", 0x1B, attribute, fg + 30);
    return command;
}

/// Reset the text color (on either stdout or stderr) to its original state (thanks to Radu Rusu for the code)
inline std::string ResetTextColor()
{
    char command[12];
    sprintf (command, "%c[0;38;48m", 0x1B);
    return command;
}

inline std::wstring ChangeTextColorW (int attribute, int fg)
{
    wchar_t command[13];
    swprintf (command, 13, L"%c[%d;%dm", 0x1B, attribute, fg + 30);
    return command;
}

inline std::wstring RavePrintTransformString(const wchar_t* fmt)
{
    std::vector<int> positions;
    std::wstring str = fmt;
    wchar_t* p = wcsstr(&str[0], L"%s");
    while(p != NULL ) {
        positions.push_back((int)(p-&str[0])+1);
        p = wcsstr(p+2, L"%s");
    }

    p = wcsstr(&str[0], L"%S");
    while(p != NULL ) {
        p[1] = 's';
        p = wcsstr(p+2, L"%S");
    }

    p = wcsstr(&str[0], L"%ls");
    while(p != NULL ) {
        p[1] = 's';
        p[2] = ' ';
        p = wcsstr(p+2, L"%ls");
    }

    for(int i = 0; i < (int)positions.size(); ++i)
        str[positions[i]] = 'S';
    return str;
}

enum DebugLevel {
    Level_Fatal=0,
    Level_Error=1,
    Level_Warn=2,
    Level_Info=3,
    Level_Debug=4,
    Level_Verbose=5
};

#define OPENRAVECOLOR_FATALLEVEL 5 // magenta
#define OPENRAVECOLOR_ERRORLEVEL 1 // red
#define OPENRAVECOLOR_WARNLEVEL 3 // yellow
#define OPENRAVECOLOR_INFOLEVEL 0 // black
#define OPENRAVECOLOR_DEBUGLEVEL 2 // green
#define OPENRAVECOLOR_VERBOSELEVEL 4 // blue

/// Random number generation
//@{
enum IntervalType {
    IT_Open=0, ///< (a,b)
    IT_OpenStart=1, ///< (a,b]
    IT_OpenEnd=2, ///< [a,b)
    IT_Closed=3, ///< [a,b]
};

OPENRAVE_API void RaveInitRandomGeneration(uint32_t seed);
/// generate a random integer, 32bit precision
OPENRAVE_API uint32_t RaveRandomInt();
/// generate n random integers, 32bit precision
OPENRAVE_API void RaveRandomInt(int n, std::vector<int>& v);

/// \brief generate a random float in 0-1
///
/// \param interval specifies inclusion of 0 and 1 in the result
OPENRAVE_API float RaveRandomFloat(IntervalType interval=IT_Closed);

/// \deprecated (10/11/27)
OPENRAVE_API void RaveRandomFloat(int n, std::vector<float>& v) RAVE_DEPRECATED;

/// \brief generate a random double in 0-1, 53bit precision
///
/// \param interval specifies inclusion of 0 and 1 in the result
OPENRAVE_API double RaveRandomDouble(IntervalType interval=IT_Closed);

/// \deprecated (10/11/27)
OPENRAVE_API void RaveRandomDouble(int n, std::vector<double>& v) RAVE_DEPRECATED;
//@}

/// Sets the global openrave debug level
OPENRAVE_API void RaveSetDebugLevel(DebugLevel level);

/// Returns the openrave debug level
OPENRAVE_API DebugLevel RaveGetDebugLevel();

/// extracts only the filename
inline const char* RaveGetSourceFilename(const char* pfilename)
{
    if( pfilename == NULL ) {
        return "";
    }
    const char* p0 = strrchr(pfilename,'/');
    const char* p1 = strrchr(pfilename,'\\');
    const char* p = p0 > p1 ? p0 : p1;
    if( p == NULL ) {
        return pfilename;
    }
    return p+1;
}

#ifdef _WIN32

#define DefineRavePrintfW(LEVEL) \
    inline int RavePrintfW##LEVEL(const wchar_t *fmt, ...) \
    { \
        /*ChangeTextColor (stdout, 0, OPENRAVECOLOR##LEVEL);*/ \
        va_list list; \
	    va_start(list,fmt); \
        int r = vwprintf(OpenRAVE::RavePrintTransformString(fmt).c_str(), list); \
        va_end(list); \
        /*ResetTextColor (stdout);*/ \
        return r; \
    }

#define DefineRavePrintfA(LEVEL) \
    inline int RavePrintfA##LEVEL(const std::string& s) \
    { \
        if( s.size() == 0 || s[s.size()-1] != '\n' ) {  \
            printf("%s\n", s.c_str()); \
        } \
        else { \
            printf ("%s", s.c_str()); \
        } \
        return s.size(); \
    } \
    \
    inline int RavePrintfA##LEVEL(const char *fmt, ...) \
    { \
        /*ChangeTextColor (stdout, 0, OPENRAVECOLOR##LEVEL);*/ \
        va_list list; \
        va_start(list,fmt); \
        int r = vprintf(fmt, list); \
        va_end(list); \
        /*if( fmt[0] != '\n' ) { printf("\n"); }*/  \
        /*ResetTextColor(stdout);*/ \
        return r; \
    }

inline int RavePrintfA(const std::string& s, DebugLevel level)
{
    if( s.size() == 0 || s[s.size()-1] != '\n' ) { // automatically add a new line
        printf("%s\n", s.c_str());
    }
    else {
        printf ("%s", s.c_str());
    }
    return s.size();
}

DefineRavePrintfW(_INFOLEVEL)
DefineRavePrintfA(_INFOLEVEL)

#else

#define DefineRavePrintfW(LEVEL) \
    inline int RavePrintfW##LEVEL(const wchar_t *wfmt, ...) \
    { \
        va_list list; \
        va_start(list,wfmt); \
        /* Allocate memory on the stack to avoid heap fragmentation */ \
        size_t allocsize = wcstombs(NULL, wfmt, 0)+32; \
        char* fmt = (char*)alloca(allocsize); \
        strcpy(fmt, ChangeTextColor(0, OPENRAVECOLOR##LEVEL,8).c_str()); \
        snprintf(fmt+strlen(fmt),allocsize-16,"%S",wfmt); \
        strcat(fmt, ResetTextColor().c_str()); \
        int r = vprintf(fmt, list);        \
        va_end(list); \
        return r; \
    }

// In linux, only wprintf will succeed, due to the fwide() call in main, so
// for programmers who want to use regular format strings without
// the L in front, we will take their regular string and widen it
// for them.
    inline int RavePrintfA_INFOLEVEL(const std::string& s)
    {
        if( s.size() == 0 || s[s.size()-1] != '\n' ) { // automatically add a new line
            printf("%s\n", s.c_str());
        }
        else {
            printf ("%s", s.c_str());
        }
        return s.size();
    }
    
    inline int RavePrintfA_INFOLEVEL(const char *fmt, ...)
    {
        va_list list;
	    va_start(list,fmt);
        int r = vprintf(fmt, list);
        va_end(list);
        //if( fmt[0] != '\n' ) { printf("\n"); }
        return r;
    }

#define DefineRavePrintfA(LEVEL) \
    inline int RavePrintfA##LEVEL(const std::string& s) \
    { \
        if( s.size() == 0 || s[s.size()-1] != '\n' ) { \
            printf ("%c[0;%d;%dm%s%c[m\n", 0x1B, OPENRAVECOLOR##LEVEL + 30,8+40,s.c_str(),0x1B); \
        } \
        else { \
            printf ("%c[0;%d;%dm%s%c[m", 0x1B, OPENRAVECOLOR##LEVEL + 30,8+40,s.c_str(),0x1B); \
        } \
        return s.size(); \
    } \
    \
    inline int RavePrintfA##LEVEL(const char *fmt, ...) \
    { \
        va_list list; \
	    va_start(list,fmt); \
        int r = vprintf((ChangeTextColor(0, OPENRAVECOLOR##LEVEL,8) + std::string(fmt) + ResetTextColor()).c_str(), list); \
        va_end(list); \
        /*if( fmt[0] != '\n' ) { printf("\n"); } */ \
        return r; \
    } \


inline int RavePrintfA(const std::string& s, DebugLevel level)
{
    if( OpenRAVE::RaveGetDebugLevel()>=level ) {
        int color = 0;
        switch(level) {
        case Level_Fatal: color = OPENRAVECOLOR_FATALLEVEL; break;
        case Level_Error: color = OPENRAVECOLOR_ERRORLEVEL; break;
        case Level_Warn: color = OPENRAVECOLOR_WARNLEVEL; break;
        case Level_Info: // print regular
            if( s.size() == 0 || s[s.size()-1] != '\n' ) { // automatically add a new line
                printf ("%s\n",s.c_str());
            }
            else {
                printf ("%s",s.c_str());
            }
            return s.size(); 
        case Level_Debug: color = OPENRAVECOLOR_DEBUGLEVEL; break;
        case Level_Verbose: color = OPENRAVECOLOR_VERBOSELEVEL; break;
        }
        if( s.size() == 0 || s[s.size()-1] != '\n' ) { // automatically add a new line
            printf ("%c[0;%d;%dm%s%c[0;38;48m\n", 0x1B, color + 30,8+40,s.c_str(),0x1B);
        }
        else {
            printf ("%c[0;%d;%dm%s%c[0;38;48m", 0x1B, color + 30,8+40,s.c_str(),0x1B);
        }
        return s.size();
    }
    return 0;
}

#endif

DefineRavePrintfW(_FATALLEVEL)
DefineRavePrintfW(_ERRORLEVEL)
DefineRavePrintfW(_WARNLEVEL)
//DefineRavePrintfW(_INFOLEVEL)
DefineRavePrintfW(_DEBUGLEVEL)
DefineRavePrintfW(_VERBOSELEVEL)

DefineRavePrintfA(_FATALLEVEL)
DefineRavePrintfA(_ERRORLEVEL)
DefineRavePrintfA(_WARNLEVEL)
//DefineRavePrintfA(_INFOLEVEL)
DefineRavePrintfA(_DEBUGLEVEL)
DefineRavePrintfA(_VERBOSELEVEL)

#define RAVEPRINTHEADER(LEVEL) OpenRAVE::RavePrintfA##LEVEL("[%s:%d] ", OpenRAVE::RaveGetSourceFilename(__FILE__), __LINE__)

// different logging levels. The higher the suffix number, the less important the information is.
// 0 log level logs all the time. OpenRAVE starts up with a log level of 0.
#define RAVELOG_LEVELW(LEVEL,level) OpenRAVE::RaveGetDebugLevel()>=(level)&&(RAVEPRINTHEADER(LEVEL)>0)&&OpenRAVE::RavePrintfW##LEVEL
#define RAVELOG_LEVELA(LEVEL,level) OpenRAVE::RaveGetDebugLevel()>=(level)&&(RAVEPRINTHEADER(LEVEL)>0)&&OpenRAVE::RavePrintfA##LEVEL

// define log4cxx equivalents (eventually OpenRAVE will move to log4cxx logging)
#define RAVELOG_FATALW RAVELOG_LEVELW(_FATALLEVEL,OpenRAVE::Level_Fatal)
#define RAVELOG_FATALA RAVELOG_LEVELA(_FATALLEVEL,OpenRAVE::Level_Fatal)
#define RAVELOG_FATAL RAVELOG_FATALA
#define RAVELOG_ERRORW RAVELOG_LEVELW(_ERRORLEVEL,OpenRAVE::Level_Error)
#define RAVELOG_ERRORA RAVELOG_LEVELA(_ERRORLEVEL,OpenRAVE::Level_Error)
#define RAVELOG_ERROR RAVELOG_ERRORA
#define RAVELOG_WARNW RAVELOG_LEVELW(_WARNLEVEL,OpenRAVE::Level_Warn)
#define RAVELOG_WARNA RAVELOG_LEVELA(_WARNLEVEL,OpenRAVE::Level_Warn)
#define RAVELOG_WARN RAVELOG_WARNA
#define RAVELOG_INFOW RAVELOG_LEVELW(_INFOLEVEL,OpenRAVE::Level_Info)
#define RAVELOG_INFOA RAVELOG_LEVELA(_INFOLEVEL,OpenRAVE::Level_Info)
#define RAVELOG_INFO RAVELOG_INFOA
#define RAVELOG_DEBUGW RAVELOG_LEVELW(_DEBUGLEVEL,OpenRAVE::Level_Debug)
#define RAVELOG_DEBUGA RAVELOG_LEVELA(_DEBUGLEVEL,OpenRAVE::Level_Debug)
#define RAVELOG_DEBUG RAVELOG_DEBUGA
#define RAVELOG_VERBOSEW RAVELOG_LEVELW(_VERBOSELEVEL,OpenRAVE::Level_Verbose)
#define RAVELOG_VERBOSEA RAVELOG_LEVELA(_VERBOSELEVEL,OpenRAVE::Level_Verbose)
#define RAVELOG_VERBOSE RAVELOG_VERBOSEA

#define IS_DEBUGLEVEL(level) (OpenRAVE::RaveGetDebugLevel()>=(level))

/// \brief Enumeration of all the interfaces.
enum InterfaceType
{
    PT_Planner=1, ///< describes \ref PlannerBase interface
    PT_Robot=2, ///< describes \ref RobotBase interface
    PT_SensorSystem=3, ///< describes \ref SensorSystemBase interface
    PT_Controller=4, ///< describes \ref ControllerBase interface
    PT_ProblemInstance=5, ///< describes \ref ProblemInstance interface
    PT_InverseKinematicsSolver=6, ///< describes \ref IkSolverBase interface
    PT_KinBody=7, ///< describes \ref KinBody
    PT_PhysicsEngine=8, ///< describes \ref PhysicsEngineBase
    PT_Sensor=9, ///< describes \ref SensorBase
    PT_CollisionChecker=10, ///< describes \ref CollisionCheckerBase
    PT_Trajectory=11, ///< describes \ref TrajectoryBase
    PT_Viewer=12,///< describes \ref ViewerBase
    PT_NumberOfInterfaces=12 ///< number of interfaces, do not forget to update
};

/// \deprecated (10/01/01)
typedef InterfaceType PluginType RAVE_DEPRECATED;

class CollisionReport;
class InterfaceBase;
class IkSolverBase;
class TrajectoryBase;
class ControllerBase;
class PlannerBase;
class RobotBase;
class ProblemInstance;
class EnvironmentBase;
class KinBody;
class SensorSystemBase;
class PhysicsEngineBase;
class SensorBase;
class CollisionCheckerBase;
class ViewerBase;
class IkParameterization;

typedef boost::shared_ptr<CollisionReport> CollisionReportPtr;
typedef boost::shared_ptr<CollisionReport const> CollisionReportConstPtr;
typedef boost::shared_ptr<InterfaceBase> InterfaceBasePtr;
typedef boost::shared_ptr<InterfaceBase const> InterfaceBaseConstPtr;
typedef boost::weak_ptr<InterfaceBase> InterfaceBaseWeakPtr;
typedef boost::shared_ptr<KinBody> KinBodyPtr;
typedef boost::shared_ptr<KinBody const> KinBodyConstPtr;
typedef boost::weak_ptr<KinBody> KinBodyWeakPtr;
typedef boost::shared_ptr<RobotBase> RobotBasePtr;
typedef boost::shared_ptr<RobotBase const> RobotBaseConstPtr;
typedef boost::weak_ptr<RobotBase> RobotBaseWeakPtr;
typedef boost::shared_ptr<CollisionCheckerBase> CollisionCheckerBasePtr;
typedef boost::shared_ptr<CollisionCheckerBase const> CollisionCheckerBaseConstPtr;
typedef boost::weak_ptr<CollisionCheckerBase> CollisionCheckerBaseWeakPtr;
typedef boost::shared_ptr<ControllerBase> ControllerBasePtr;
typedef boost::shared_ptr<ControllerBase const> ControllerBaseConstPtr;
typedef boost::weak_ptr<ControllerBase> ControllerBaseWeakPtr;
typedef boost::shared_ptr<IkSolverBase> IkSolverBasePtr;
typedef boost::shared_ptr<IkSolverBase const> IkSolverBaseConstPtr;
typedef boost::weak_ptr<IkSolverBase> IkSolverBaseWeakPtr;
typedef boost::shared_ptr<PhysicsEngineBase> PhysicsEngineBasePtr;
typedef boost::shared_ptr<PhysicsEngineBase const> PhysicsEngineBaseConstPtr;
typedef boost::weak_ptr<PhysicsEngineBase> PhysicsEngineBaseWeakPtr;
typedef boost::shared_ptr<PlannerBase> PlannerBasePtr;
typedef boost::shared_ptr<PlannerBase const> PlannerBaseConstPtr;
typedef boost::weak_ptr<PlannerBase> PlannerBaseWeakPtr;
typedef boost::shared_ptr<ProblemInstance> ProblemInstancePtr;
typedef boost::shared_ptr<ProblemInstance const> ProblemInstanceConstPtr;
typedef boost::weak_ptr<ProblemInstance> ProblemInstanceWeakPtr;
typedef boost::shared_ptr<SensorBase> SensorBasePtr;
typedef boost::shared_ptr<SensorBase const> SensorBaseConstPtr;
typedef boost::weak_ptr<SensorBase> SensorBaseWeakPtr;
typedef boost::shared_ptr<SensorSystemBase> SensorSystemBasePtr;
typedef boost::shared_ptr<SensorSystemBase const> SensorSystemBaseConstPtr;
typedef boost::weak_ptr<SensorSystemBase> SensorSystemBaseWeakPtr;
typedef boost::shared_ptr<TrajectoryBase> TrajectoryBasePtr;
typedef boost::shared_ptr<TrajectoryBase const> TrajectoryBaseConstPtr;
typedef boost::weak_ptr<TrajectoryBase> TrajectoryBaseWeakPtr;
typedef boost::shared_ptr<ViewerBase> ViewerBasePtr;
typedef boost::shared_ptr<ViewerBase const> ViewerBaseConstPtr;
typedef boost::weak_ptr<ViewerBase> ViewerBaseWeakPtr;
typedef boost::shared_ptr<EnvironmentBase> EnvironmentBasePtr;
typedef boost::shared_ptr<EnvironmentBase const> EnvironmentBaseConstPtr;
typedef boost::weak_ptr<EnvironmentBase> EnvironmentBaseWeakPtr;

///< Cloning Options for interfaces and environments
enum CloningOptions {
    Clone_Bodies = 1, ///< clone all the bodies/robots of the environment, exclude attached interfaces like sensors/controllers
    Clone_Viewer = 2, ///< clone the viewer type, although figures won't be copied, new viewer does try to match views
    Clone_Simulation = 4, ///< clone the physics engine and simulation state (ie, timesteps, gravity)
    Clone_RealControllers = 8, ///< if specified, will clone the real controllers of all the robots, otherwise each robot gets ideal controller
    Clone_Sensors = 16, ///< if specified, will clone the sensors attached to the robot and added to the environment
};

/// base class for readable interfaces
class OPENRAVE_API XMLReadable
{
public:
    XMLReadable(const std::string& xmlid) : __xmlid(xmlid) {}
	virtual ~XMLReadable() {}
    virtual const std::string& GetXMLId() const { return __xmlid; }
private:
    std::string __xmlid;
};

typedef boost::shared_ptr<XMLReadable> XMLReadablePtr;
typedef boost::shared_ptr<XMLReadable const> XMLReadableConstPtr;
typedef std::list<std::pair<std::string,std::string> > AttributesList;
/// \deprecated (11/02/18)
typedef AttributesList XMLAttributesList RAVE_DEPRECATED;

/// base class for all xml readers. XMLReaders are used to process data from
/// xml files. Custom readers can be registered through EnvironmentBase.
/// By default it can record all data that is encountered inside the xml reader
class OPENRAVE_API BaseXMLReader : public boost::enable_shared_from_this<BaseXMLReader>
{
public:
    enum ProcessElement
    {
        PE_Pass=0, ///< current tag was not supported, so pass onto another class
        PE_Support=1, ///< current tag will be processed by this class
        PE_Ignore=2, ///< current tag and all its children should be ignored
    };
    BaseXMLReader() {}
    virtual ~BaseXMLReader() {}

    /// a readable interface that stores the information processsed for the current tag
    /// This pointer is used to the InterfaceBase class registered readers
    virtual XMLReadablePtr GetReadable() { return XMLReadablePtr(); }

    /// Gets called in the beginning of each "<type>" expression. In this case, name is "type"
    /// \param name of the tag, will be always lower case
    /// \param atts string of attributes where the first std::string is the attribute name and second is the value
    /// \return true if tag is accepted and this class will process it, otherwise false
    virtual ProcessElement startElement(const std::string& name, const AttributesList& atts) = 0;

    /// Gets called at the end of each "</type>" expression. In this case, name is "type"
    /// \param name of the tag, will be always lower case
    /// \return true if XMLReader has finished parsing (one condition is that name==_fieldname) , otherwise false
    virtual bool endElement(const std::string& name) = 0;

    /// gets called for all data in between tags.
    /// \param ch a string to the data
    virtual void characters(const std::string& ch) = 0;

    /// XML filename/resource used for this class (can be empty)
    std::string _filename;
};

typedef boost::shared_ptr<BaseXMLReader> BaseXMLReaderPtr;
typedef boost::shared_ptr<BaseXMLReader const> BaseXMLReaderConstPtr;

typedef boost::function<BaseXMLReaderPtr(InterfaceBasePtr, const AttributesList&)> CreateXMLReaderFn;

/// reads until the tag ends
class OPENRAVE_API DummyXMLReader : public BaseXMLReader
{
public:
    DummyXMLReader(const std::string& pfieldname, const std::string& pparentname, boost::shared_ptr<std::ostream> osrecord = boost::shared_ptr<std::ostream>());
    virtual ProcessElement startElement(const std::string& name, const AttributesList& atts);
    virtual bool endElement(const std::string& name);
    virtual void characters(const std::string& ch);
    const std::string& GetFieldName() const { return _fieldname; }
private:
    std::string _parentname; /// XML filename
    std::string _fieldname;
    boost::shared_ptr<std::ostream> _osrecord; ///< used to store the xml data
    boost::shared_ptr<BaseXMLReader> _pcurreader;
};

} // end namespace OpenRAVE

// define the math functions
#define MATH_EXP RaveExp
#define MATH_LOG RaveLog
#define MATH_COS RaveCos
#define MATH_SIN RaveSin
#define MATH_TAN RaveTan
#define MATH_LOG2 RaveLog2
#define MATH_LOG10 RaveLog10
#define MATH_ACOS RaveAcos
#define MATH_ASIN RaveAsin
#define MATH_ATAN2 RaveAtan2
#define MATH_POW RavePow
#define MATH_SQRT RaveSqrt
#define MATH_FABS RaveFabs

#include <openrave/geometry.h>
#include <openrave/mathextra.h>

namespace OpenRAVE {
    using geometry::RaveVector;
    using geometry::RaveTransform;
    using geometry::RaveTransformMatrix;
    typedef RaveVector<dReal> Vector;
    typedef RaveTransform<dReal> Transform;
    typedef boost::shared_ptr< RaveTransform<dReal> > TransformPtr;
    typedef boost::shared_ptr< RaveTransform<dReal> const > TransformConstPtr;
    typedef RaveTransformMatrix<dReal> TransformMatrix;
    typedef boost::shared_ptr< RaveTransformMatrix<dReal> > TransformMatrixPtr;
    typedef boost::shared_ptr< RaveTransformMatrix<dReal> const > TransformMatrixConstPtr;
    typedef geometry::obb<dReal> OBB;
    typedef geometry::aabb<dReal> AABB;
    typedef geometry::ray<dReal> RAY;
    // for compatibility
    //@{
    using mathextra::dot2;
    using mathextra::dot3;
    using mathextra::dot4;
    using mathextra::normalize2;
    using mathextra::normalize3;
    using mathextra::normalize4;
    using mathextra::cross3;
    using mathextra::inv3;
    using mathextra::inv4;
    using mathextra::lengthsqr2;
    using mathextra::lengthsqr3;
    using mathextra::lengthsqr4;
    using mathextra::mult4;
    //@}

/** \brief Parameterization of basic primitives for querying inverse-kinematics solutions.

    Holds the parameterization of a geometric primitive useful for autonomous manipulation scenarios like:
    6D pose, 3D translation, 3D rotation, 3D look at direction, and ray look at direction.
*/
class OPENRAVE_API IkParameterization
{
public:
    /// \brief The types of inverse kinematics parameterizations supported.
    ///
    /// The minimum degree of freedoms required is set in the upper 4 bits of each type.
    /// The number of values used to represent the parameterization ( >= dof ) is the next 4 bits.
    /// The lower bits contain a unique id of the type.
    enum Type {
        Type_None=0,
        Type_Transform6D=0x67000001, ///< end effector reaches desired 6D transformation
        Type_Rotation3D=0x34000002, ///< end effector reaches desired 3D rotation
        Type_Translation3D=0x33000003, ///< end effector origin reaches desired 3D translation
        Type_Direction3D=0x23000004, ///< direction on end effector coordinate system reaches desired direction
        Type_Ray4D=0x46000005, ///< ray on end effector coordinate system reaches desired global ray
        Type_Lookat3D=0x23000006, ///< direction on end effector coordinate system points to desired 3D position
        Type_TranslationDirection5D=0x56000007, ///< end effector origin and direction reaches desired 3D translation and direction. Can be thought of as Ray IK where the origin of the ray must coincide.
        Type_TranslationXY2D=0x22000008, ///< 2D translation along XY plane
        Type_TranslationXYOrientation3D=0x33000009, ///< 2D translation along XY plane and 1D rotation around Z axis. The offset of the rotation is measured starting at +X, so at +X is it 0, at +Y it is pi/2.
        Type_TranslationLocalGlobal6D=0x3600000a, ///< local point on end effector origin reaches desired 3D global point
        Type_NumberOfParameterizations=10, ///< number of parameterizations (does not count Type_None)
    };

    IkParameterization() : _type(Type_None) {}
    /// \brief sets a 6D transform parameterization
    IkParameterization(const Transform& t) { SetTransform6D(t); }
    /// \brief sets a ray parameterization
    IkParameterization(const RAY& r) { SetRay4D(r); }
    /// \brief set a custom parameterization using a transform as the source of the data. Not all types are supported with this method.
    IkParameterization(const Transform& t, Type type) {
        _type=type;
        switch(_type) {
        case Type_Transform6D: SetTransform6D(t); break;
        case Type_Rotation3D: SetRotation3D(t.rot); break;
        case Type_Translation3D: SetTranslation3D(t.trans); break;
        case Type_Lookat3D: SetLookat3D(t.trans); break;
        default:
            throw openrave_exception(str(boost::format("IkParameterization constructor does not support type 0x%x")%_type));
        }
    }

    inline Type GetType() const { return _type; }
    inline const std::string& GetName() const;

    /// \brief Returns the minimum degree of freedoms required for the IK type.
    static int GetDOF(Type type) { return (type>>28)&0xf; }
    /// \brief Returns the minimum degree of freedoms required for the IK type.
    inline int GetDOF() const { return (_type>>28)&0xf; }

    /// \brief Returns the number of values used to represent the parameterization ( >= dof ). The number of values serialized is this number plus 1 for the iktype.
    static int GetNumberOfValues(Type type) { return (type>>24)&0xf; }
    /// \brief Returns the number of values used to represent the parameterization ( >= dof ). The number of values serialized is this number plus 1 for the iktype.
    inline int GetNumberOfValues() const { return (_type>>24)&0xf; }

    inline void SetTransform6D(const Transform& t) { _type = Type_Transform6D; _transform = t; }
    inline void SetRotation3D(const Vector& quaternion) { _type = Type_Rotation3D; _transform.rot = quaternion; }
    inline void SetTranslation3D(const Vector& trans) { _type = Type_Translation3D; _transform.trans = trans; }
    inline void SetDirection3D(const Vector& dir) { _type = Type_Direction3D; _transform.rot = dir; }
    inline void SetRay4D(const RAY& ray) { _type = Type_Ray4D; _transform.trans = ray.pos; _transform.rot = ray.dir; }
    inline void SetLookat3D(const Vector& trans) { _type = Type_Lookat3D; _transform.trans = trans; }
    /// \brief the ray direction is not used for IK, however it is needed in order to compute the error
    inline void SetLookat3D(const RAY& ray) { _type = Type_Lookat3D; _transform.trans = ray.pos; _transform.rot = ray.dir; }
    inline void SetTranslationDirection5D(const RAY& ray) { _type = Type_TranslationDirection5D; _transform.trans = ray.pos; _transform.rot = ray.dir; }
    inline void SetTranslationXY2D(const Vector& trans) { _type = Type_TranslationXY2D; _transform.trans.x = trans.x; _transform.trans.y = trans.y; _transform.trans.z = 0; _transform.trans.w = 0; }
    inline void SetTranslationXYOrientation3D(const Vector& trans) { _type = Type_TranslationXYOrientation3D; _transform.trans.x = trans.x; _transform.trans.y = trans.y; _transform.trans.z = trans.z; _transform.trans.w = 0; }
    inline void SetTranslationLocalGlobal6D(const Vector& localtrans, const Vector& trans) { _type = Type_TranslationLocalGlobal6D; _transform.rot.x = localtrans.x; _transform.rot.y = localtrans.y; _transform.rot.z = localtrans.z; _transform.rot.w = 0; _transform.trans.x = trans.x; _transform.trans.y = trans.y; _transform.trans.z = trans.z; _transform.trans.w = 0; }

    inline const Transform& GetTransform6D() const { return _transform; }
    inline const Vector& GetRotation3D() const { return _transform.rot; }
    inline const Vector& GetTranslation3D() const { return _transform.trans; }
    inline const Vector& GetDirection3D() const { return _transform.rot; }
    inline const RAY GetRay4D() const { return RAY(_transform.trans,_transform.rot); }
    inline const Vector& GetLookat3D() const { return _transform.trans; }
    inline const Vector& GetLookat3DDirection() const { return _transform.rot; }
    inline const RAY GetTranslationDirection5D() const { return RAY(_transform.trans,_transform.rot); }
    inline const Vector& GetTranslationXY2D() const { return _transform.trans; }
    inline const Vector& GetTranslationXYOrientation3D() const { return _transform.trans; }
    inline std::pair<Vector,Vector> GetTranslationLocalGlobal6D() const { return std::make_pair(_transform.rot,_transform.trans); }

    /// \deprecated (11/02/15)
    //@{
    inline void SetTransform(const Transform& t) RAVE_DEPRECATED { SetTransform6D(t); }
    inline void SetRotation(const Vector& quaternion) RAVE_DEPRECATED { SetRotation3D(quaternion); }
    inline void SetTranslation(const Vector& trans) RAVE_DEPRECATED { SetTranslation3D(trans); }
    inline void SetDirection(const Vector& dir) RAVE_DEPRECATED { SetDirection3D(dir); }
    inline void SetRay(const RAY& ray) RAVE_DEPRECATED { SetRay4D(ray); }
    inline void SetLookat(const Vector& trans) RAVE_DEPRECATED { SetLookat3D(trans); }
    inline void SetTranslationDirection(const RAY& ray) RAVE_DEPRECATED { SetTranslationDirection5D(ray); }
    inline const Transform& GetTransform() const RAVE_DEPRECATED { return _transform; }
    inline const Vector& GetRotation() const RAVE_DEPRECATED { return _transform.rot; }
    inline const Vector& GetTranslation() const RAVE_DEPRECATED { return _transform.trans; }
    inline const Vector& GetDirection() const RAVE_DEPRECATED { return _transform.rot; }
    inline const Vector& GetLookat() const RAVE_DEPRECATED { return _transform.trans; }
    inline const RAY GetRay() const RAVE_DEPRECATED { return RAY(_transform.trans,_transform.rot); }
    inline const RAY GetTranslationDirection() const RAVE_DEPRECATED { return RAY(_transform.trans,_transform.rot); }
    //@}

protected:
    Transform _transform;
    Type _type;

    friend IkParameterization operator* (const Transform& t, const IkParameterization& ikparam);
    friend std::ostream& operator<<(std::ostream& O, const IkParameterization& ikparam);
    friend std::istream& operator>>(std::istream& I, IkParameterization& ikparam);
};

inline IkParameterization operator* (const Transform& t, const IkParameterization& ikparam)
{
    IkParameterization local;
    switch(ikparam.GetType()) {
    case IkParameterization::Type_Transform6D:
        local.SetTransform6D(t * ikparam.GetTransform6D());
        break;
    case IkParameterization::Type_Rotation3D:
        local.SetRotation3D(quatMultiply(quatInverse(t.rot),ikparam.GetRotation3D()));
        break;
    case IkParameterization::Type_Translation3D:
        local.SetTranslation3D(t*ikparam.GetTranslation3D());
        break;
    case IkParameterization::Type_Direction3D:
        local.SetDirection3D(t.rotate(ikparam.GetDirection3D()));
        break; 
    case IkParameterization::Type_Ray4D:
        local.SetRay4D(RAY(t*ikparam.GetRay4D().pos,t.rotate(ikparam.GetRay4D().dir)));
        break;
    case IkParameterization::Type_Lookat3D:
        local.SetLookat3D(RAY(t*ikparam.GetLookat3D(),t.rotate(ikparam.GetLookat3DDirection())));
        break;
    case IkParameterization::Type_TranslationDirection5D:
        local.SetTranslationDirection5D(RAY(t*ikparam.GetTranslationDirection5D().pos,t.rotate(ikparam.GetTranslationDirection5D().dir)));
        break;
    case IkParameterization::Type_TranslationXY2D:
        local.SetTranslationXY2D(t*ikparam.GetTranslationXY2D());
        break;
    case IkParameterization::Type_TranslationXYOrientation3D: {
        Vector v = ikparam.GetTranslationXYOrientation3D();
        Vector voldtrans(v.x,v.y,0);
        Vector vnewtrans = t*voldtrans;
        dReal zangle = -normalizeAxisRotation(Vector(0,0,1),t.rot).first;
        local.SetTranslationXYOrientation3D(Vector(vnewtrans.y,vnewtrans.y,v.z+zangle));
        break;
    }
    case IkParameterization::Type_TranslationLocalGlobal6D:
        local.SetTranslationLocalGlobal6D(ikparam.GetTranslationLocalGlobal6D().first, t*ikparam.GetTranslationLocalGlobal6D().second);
        break;
    default:
        throw openrave_exception(str(boost::format("does not support parameterization %d")%ikparam.GetType()));
    }
    return local;
}
 
inline std::ostream& operator<<(std::ostream& O, const IkParameterization& ikparam)
{
    O << ikparam._type << " ";
    switch(ikparam._type) {
    case IkParameterization::Type_Transform6D:
        O << ikparam.GetTransform6D();
        break;
    case IkParameterization::Type_Rotation3D:
        O << ikparam.GetRotation3D();
        break;
    case IkParameterization::Type_Translation3D: {
        Vector v = ikparam.GetTranslation3D();
        O << v.x << " " << v.y << " " << v.z << " ";
        break;
    }
    case IkParameterization::Type_Direction3D: {
        Vector v = ikparam.GetDirection3D();
        O << v.x << " " << v.y << " " << v.z << " ";
        break;
    }
    case IkParameterization::Type_Ray4D: {
        O << ikparam.GetRay4D();
        break;
    }
    case IkParameterization::Type_Lookat3D: {
        Vector v = ikparam.GetLookat3D();
        O << v.x << " " << v.y << " " << v.z << " ";
        break;
    }
    case IkParameterization::Type_TranslationDirection5D:
        O << ikparam.GetTranslationDirection5D();
        break;
    case IkParameterization::Type_TranslationXY2D: {
        Vector v = ikparam.GetTranslationXY2D();
        O << v.x << " " << v.y << " ";
        break;
    }
    case IkParameterization::Type_TranslationXYOrientation3D: {
        Vector v = ikparam.GetTranslationXYOrientation3D();
        O << v.x << " " << v.y << " " << v.z << " ";
        break;
    }
    case IkParameterization::Type_TranslationLocalGlobal6D: {
        std::pair<Vector,Vector> p = ikparam.GetTranslationLocalGlobal6D();
        O << p.first.x << " " << p.first.y << " " << p.first.z << " " << p.second.x << " " << p.second.y << " " << p.second.z << " ";
        break;
    }
    default:
        throw openrave_exception(str(boost::format("does not support parameterization %d")%ikparam.GetType()));
    }
    return O;
}

inline std::istream& operator>>(std::istream& I, IkParameterization& ikparam)
{
    int type=IkParameterization::Type_None;
    I >> type;
    ikparam._type = static_cast<IkParameterization::Type>(type);
    switch(ikparam._type) {
    case IkParameterization::Type_Transform6D: { Transform t; I >> t; ikparam.SetTransform6D(t); break; }
    case IkParameterization::Type_Rotation3D: { Vector v; I >> v; ikparam.SetRotation3D(v); break; }
    case IkParameterization::Type_Translation3D: { Vector v; I >> v.x >> v.y >> v.z; ikparam.SetTranslation3D(v); break; }
    case IkParameterization::Type_Direction3D: { Vector v; I >> v.x >> v.y >> v.z; ikparam.SetDirection3D(v); break; }
    case IkParameterization::Type_Ray4D: { RAY r; I >> r; ikparam.SetRay4D(r); break; }
    case IkParameterization::Type_Lookat3D: { Vector v; I >> v.x >> v.y >> v.z; ikparam.SetLookat3D(v); break; }
    case IkParameterization::Type_TranslationDirection5D: { RAY r; I >> r; ikparam.SetTranslationDirection5D(r); break; }
    case IkParameterization::Type_TranslationXY2D: { Vector v; I >> v.y >> v.y; ikparam.SetTranslationXY2D(v); break; }
    case IkParameterization::Type_TranslationXYOrientation3D: { Vector v; I >> v.y >> v.y >> v.z; ikparam.SetTranslationXYOrientation3D(v); break; }
    case IkParameterization::Type_TranslationLocalGlobal6D: { Vector localtrans, trans; I >> localtrans.x >> localtrans.y >> localtrans.z >> trans.x >> trans.y >> trans.z; ikparam.SetTranslationLocalGlobal6D(localtrans,trans); break; }
    default: throw openrave_exception(str(boost::format("does not support parameterization %d")%ikparam.GetType()));
    }
    return I;
}

}

#include <openrave/plugininfo.h>
#include <openrave/interface.h>
#include <openrave/kinbody.h>
#include <openrave/trajectory.h>
#include <openrave/problems.h>
#include <openrave/collisionchecker.h>
#include <openrave/sensor.h>
#include <openrave/robot.h>
#include <openrave/iksolver.h>
#include <openrave/planner.h>
#include <openrave/controller.h>
#include <openrave/physicsengine.h>
#include <openrave/sensorsystem.h>
#include <openrave/viewer.h>
#include <openrave/environment.h>

namespace OpenRAVE {

/// \name Global Functionality - Interface Creation, Plugin Management, Logging
/// \anchor global_functionality
//@{

/// \brief Returns the a 16 character null-terminated string specifying a hash of the interfaces used for checking changes.
inline const char* RaveGetInterfaceHash(InterfaceType type)
{
    switch(type) {
    case PT_Planner: return OPENRAVE_PLANNER_HASH;
    case PT_Robot: return OPENRAVE_ROBOT_HASH;
    case PT_SensorSystem: return OPENRAVE_SENSORSYSTEM_HASH;
    case PT_Controller: return OPENRAVE_CONTROLLER_HASH;
    case PT_ProblemInstance: return OPENRAVE_PROBLEM_HASH;
    case PT_InverseKinematicsSolver: return OPENRAVE_IKSOLVER_HASH;
    case PT_KinBody: return OPENRAVE_KINBODY_HASH;
    case PT_PhysicsEngine: return OPENRAVE_PHYSICSENGINE_HASH;
    case PT_Sensor: return OPENRAVE_SENSOR_HASH;
    case PT_CollisionChecker: return OPENRAVE_COLLISIONCHECKER_HASH;
    case PT_Trajectory: return OPENRAVE_TRAJECTORY_HASH;
    case PT_Viewer: return OPENRAVE_VIEWER_HASH;
    default:
        throw openrave_exception("failed to find openrave interface type",ORE_InvalidArguments);
        return NULL;
    }
}

/// safely casts from the base interface class to an openrave interface using static_pointer_cast.
/// The reason why dynamic_pointer_cast cannot be used is because interfaces might be created by different plugins, and the runtime type information will be different.
template <typename T>
inline boost::shared_ptr<T> RaveInterfaceCast(InterfaceBasePtr pinterface)
{
    if( !!pinterface ) {
        if( pinterface->GetInterfaceType() == T::GetInterfaceTypeStatic() ) {
            return boost::static_pointer_cast<T>(pinterface);
        }
        // encode special cases
        if( pinterface->GetInterfaceType() == PT_Robot && T::GetInterfaceTypeStatic() == PT_KinBody ) {
            return boost::static_pointer_cast<T>(pinterface);
        }
    }
    return boost::shared_ptr<T>();
}

/// safely casts from the base interface class to an openrave interface using static_pointer_cast.
/// The reason why dynamic_pointer_cast cannot be used is because interfaces might be created by different plugins, and the runtime type information will be different.
template <typename T>
inline boost::shared_ptr<T const> RaveInterfaceConstCast(InterfaceBaseConstPtr pinterface)
{
    if( !!pinterface ) {
        if( pinterface->GetInterfaceType() == T::GetInterfaceTypeStatic() ) {
            return boost::static_pointer_cast<T const>(pinterface);
        }
        // encode special cases
        if( pinterface->GetInterfaceType() == PT_Robot && T::GetInterfaceTypeStatic() == PT_KinBody ) {
            return boost::static_pointer_cast<T const>(pinterface);
        }
    }
    return boost::shared_ptr<T>();
}

/// \brief returns a lower case string of the interface type
OPENRAVE_API const std::map<InterfaceType,std::string>& RaveGetInterfaceNamesMap();
OPENRAVE_API const std::string& RaveGetInterfaceName(InterfaceType type);

/// \brief returns a string of the ik parameterization type names (can include upper case in order to match IkParameterization::Type)
OPENRAVE_API const std::map<IkParameterization::Type,std::string>& RaveGetIkParameterizationMap();

/// \brief Returns the openrave home directory where settings, cache, and other files are stored.
///
/// On Linux/Unix systems, this is usually $HOME/.openrave, on Windows this is $HOMEPATH/.openrave
OPENRAVE_API std::string RaveGetHomeDirectory();

/// \brief Searches for a filename in the database and returns a full path/URL to it
///
/// \param filename the relative filename in the database
/// \param bRead if true will only return a file if it exists. If false, will return the filename of the first valid database directory.
/// \return a non-empty string if a file could be found.
OPENRAVE_API std::string RaveFindDatabaseFile(const std::string& filename, bool bRead=true);

/// \brief Explicitly initializes the global OpenRAVE state (optional).
///
/// Optional function to initialize openrave plugins and logging.
/// Although environment creation will automatically make sure this function is called, users might want
/// explicit control of when this happens.
/// \param bLoadAllPlugins If true will load all the openrave plugins automatically that can be found in the OPENRAVE_PLUGINS environment path
/// \return 0 if successful, otherwise an error code
OPENRAVE_API int RaveInitialize(bool bLoadAllPlugins=true, DebugLevel level = Level_Info);

/// \brief Initializes the global state from an already loaded OpenRAVE environment.
///
/// Because of shared object boundaries, it is necessary to pass the global state pointer
/// around. If using plugin.h, this function is automatically called by \ref CreateInterfaceValidated.
/// It is also called by and every InterfaceBase constructor.
/// \param[in] globalstate 
OPENRAVE_API void RaveInitializeFromState(UserDataPtr globalstate);

/// \brief A pointer to the global openrave state
/// \return a managed pointer to the state.
OPENRAVE_API UserDataPtr RaveGlobalState();

/// \brief Destroys the entire OpenRAVE state and all loaded environments. 
///
/// This functions should be always called before program shutdown in order to assure all
/// resources are relased appropriately.
OPENRAVE_API void RaveDestroy();

/// \brief Get all the loaded plugins and the interfaces they support.
///
/// \param plugins A list of plugins. Each entry has the plugin name and the interfaces it supports
OPENRAVE_API void RaveGetPluginInfo(std::list< std::pair<std::string, PLUGININFO> >& plugins);

/// \brief Get a list of all the loaded interfaces.
OPENRAVE_API void RaveGetLoadedInterfaces(std::map<InterfaceType, std::vector<std::string> >& interfacenames);

/// \brief Reloads all the plugins.
///
/// The interfaces currently created remain will continue using the old plugins, so this function is safe in that plugins currently loaded remain loaded until the last interface that uses them is released.
OPENRAVE_API void RaveReloadPlugins();

/// \brief Load a plugin and its interfaces.
///
/// If the plugin is already loaded, will reload it.
/// \param name the filename of the plugin to load
OPENRAVE_API bool RaveLoadPlugin(const std::string& libraryname);

/// \brief Returns true if interface can be created, otherwise false.
OPENRAVE_API bool RaveHasInterface(InterfaceType type, const std::string& interfacename);

OPENRAVE_API InterfaceBasePtr RaveCreateInterface(EnvironmentBasePtr penv, InterfaceType type,const std::string& interfacename);
OPENRAVE_API RobotBasePtr RaveCreateRobot(EnvironmentBasePtr penv, const std::string& name="");
OPENRAVE_API PlannerBasePtr RaveCreatePlanner(EnvironmentBasePtr penv, const std::string& name);
OPENRAVE_API SensorSystemBasePtr RaveCreateSensorSystem(EnvironmentBasePtr penv, const std::string& name);
OPENRAVE_API ControllerBasePtr RaveCreateController(EnvironmentBasePtr penv, const std::string& name);
OPENRAVE_API ProblemInstancePtr RaveCreateProblem(EnvironmentBasePtr penv, const std::string& name);
OPENRAVE_API IkSolverBasePtr RaveCreateIkSolver(EnvironmentBasePtr penv, const std::string& name);
OPENRAVE_API PhysicsEngineBasePtr RaveCreatePhysicsEngine(EnvironmentBasePtr penv, const std::string& name);
OPENRAVE_API SensorBasePtr RaveCreateSensor(EnvironmentBasePtr penv, const std::string& name);
OPENRAVE_API CollisionCheckerBasePtr RaveCreateCollisionChecker(EnvironmentBasePtr penv, const std::string& name);
OPENRAVE_API ViewerBasePtr RaveCreateViewer(EnvironmentBasePtr penv, const std::string& name);
OPENRAVE_API  KinBodyPtr RaveCreateKinBody(EnvironmentBasePtr penv, const std::string& name="");
/// \brief Return an empty trajectory instance initialized to nDOF degrees of freedom. Will be deprecated soon
OPENRAVE_API TrajectoryBasePtr RaveCreateTrajectory(EnvironmentBasePtr penv, int nDOF);
/// \brief Return an empty trajectory instance.
OPENRAVE_API TrajectoryBasePtr RaveCreateTrajectory(EnvironmentBasePtr penv, const std::string& name="");

/** \brief Registers a function to create an interface, this allows the interface to be created by other modules.

    \param type interface type
    \param name interface name
    \param interfacehash the hash of the interface being created (use the global defines OPENRAVE_X_HASH)
    \param envhash the hash of the environment (use the global define OPENRAVE_ENVIRONMENT_HASH)
    \param createfn functions to create the interface it takes two parameters: the environment and an istream to the rest of the interface creation arguments.
    \return a handle if function is successfully registered. By destroying the handle, the interface will be automatically unregistered.
    \throw openrave_exception Will throw with ORE_InvalidInterfaceHash if hashes do not match
 */
OPENRAVE_API boost::shared_ptr<void> RaveRegisterInterface(InterfaceType type, const std::string& name, const char* interfacehash, const char* envhash, const boost::function<InterfaceBasePtr(EnvironmentBasePtr, std::istream&)>& createfn);

/** \brief Registers a custom xml reader for a particular interface.
    
    Once registered, anytime an interface is created through XML and
    the xmltag is seen, the function CreateXMLReaderFn will be called to get a reader for that tag
    \param xmltag the tag specified in xmltag is seen in the interface, the the custom reader will be created.
    \param fn CreateXMLReaderFn(pinterface,atts) - passed in the pointer to the interface where the tag was seen along with the list of attributes
    \return a pointer holding the registration, releasing the pointer will unregister the XML reader
*/
OPENRAVE_API boost::shared_ptr<void> RaveRegisterXMLReader(InterfaceType type, const std::string& xmltag, const CreateXMLReaderFn& fn);

/// \brief return the environment's unique id, returns 0 if environment could not be found or not registered
OPENRAVE_API int RaveGetEnvironmentId(EnvironmentBasePtr penv);

/// \brief get the environment from its unique id
/// \param id the unique environment id returned by \ref RaveGetEnvironmentId
OPENRAVE_API EnvironmentBasePtr RaveGetEnvironment(int id);

/// \brief Return all the created OpenRAVE environments.
OPENRAVE_API void RaveGetEnvironments(std::list<EnvironmentBasePtr>& listenvironments);

/// \brief Returns the current registered reader for the interface type/xmlid
///
/// \throw openrave_exception Will throw with ORE_InvalidArguments if registered function could not be found.
OPENRAVE_API BaseXMLReaderPtr RaveCallXMLReader(InterfaceType type, const std::string& xmltag, InterfaceBasePtr pinterface, const AttributesList& atts);

//@}

/// \brief separates the directories from a string and returns them in a vector
inline bool RaveParseDirectories(const char* pdirs, std::vector<std::string>& vdirs)
{
    vdirs.resize(0);
    if( !pdirs ) {
        return false;
    }
    // search for all directories separated by ':'
    std::string tmp = pdirs;
    std::string::size_type pos = 0, newpos=0;
    while( pos < tmp.size() ) {
#ifdef _WIN32
        newpos = tmp.find(';', pos);
#else
		newpos = tmp.find(':', pos);
#endif
        std::string::size_type n = newpos == std::string::npos ? tmp.size()-pos : (newpos-pos);
        vdirs.push_back(tmp.substr(pos, n));
        if( newpos == std::string::npos ) {
            break;
        }
        pos = newpos+1;
    }
    return true;
}

/// \brief Create the interfaces, see \ref CreateInterfaceValidated.
/// \ingroup plugin_exports
typedef InterfaceBasePtr (*PluginExportFn_OpenRAVECreateInterface)(InterfaceType type, const std::string& name, const char* pluginhash, const char* envhash, EnvironmentBasePtr penv);

/// \brief Called to fill information about the plugin, see \ref GetPluginAttributesValidated.
/// \ingroup plugin_exports
typedef bool (*PluginExportFn_OpenRAVEGetPluginAttributes)(PLUGININFO* pinfo, int size, const char* infohash);

/// \brief Called before plugin is unloaded from openrave. See \ref DestroyPlugin. 
/// \ingroup plugin_exports
typedef void (*PluginExportFn_DestroyPlugin)();

/// \deprecated
typedef InterfaceBasePtr (*PluginExportFn_CreateInterface)(InterfaceType type, const std::string& name, const char* pluginhash, EnvironmentBasePtr penv);

/// \deprecated
typedef bool (*PluginExportFn_GetPluginAttributes)(PLUGININFO* pinfo, int size);

// define inline functions
const std::string& IkParameterization::GetName() const
{
    std::map<IkParameterization::Type,std::string>::const_iterator it = RaveGetIkParameterizationMap().find(_type);
    if( it != RaveGetIkParameterizationMap().end() ) {
        return it->second;
    }
    throw openrave_exception(str(boost::format("IkParameterization iktype 0x%x not supported")));
}

} // end namespace OpenRAVE

#if !defined(RAVE_DISABLE_ASSERT_HANDLER) && defined(BOOST_ENABLE_ASSERT_HANDLER)
/// Modifications controlling %boost library behavior.
namespace boost
{
inline void assertion_failed(char const * expr, char const * function, char const * file, long line)
{
    throw OpenRAVE::openrave_exception(boost::str(boost::format("[%s:%d] -> %s, expr: %s")%file%line%function%expr),OpenRAVE::ORE_Assert);
}
}
#endif

BOOST_STATIC_ASSERT(OPENRAVE_VERSION_MAJOR>=0&&OPENRAVE_VERSION_MAJOR<=255);
BOOST_STATIC_ASSERT(OPENRAVE_VERSION_MINOR>=0&&OPENRAVE_VERSION_MINOR<=255);
BOOST_STATIC_ASSERT(OPENRAVE_VERSION_PATCH>=0&&OPENRAVE_VERSION_PATCH<=255);

// register for typeof (MSVC only)
#ifdef RAVE_REGISTER_BOOST
#include BOOST_TYPEOF_INCREMENT_REGISTRATION_GROUP()
BOOST_TYPEOF_REGISTER_TYPE(OpenRAVE::InterfaceType)
BOOST_TYPEOF_REGISTER_TYPE(OpenRAVE::UserData)

BOOST_TYPEOF_REGISTER_TYPE(OpenRAVE::ProblemInstance)
BOOST_TYPEOF_REGISTER_TYPE(OpenRAVE::ControllerBase)
BOOST_TYPEOF_REGISTER_TYPE(OpenRAVE::PlannerBase)
BOOST_TYPEOF_REGISTER_TYPE(OpenRAVE::PlannerBase::PlannerParameters)
BOOST_TYPEOF_REGISTER_TYPE(OpenRAVE::IkSolverBase)
BOOST_TYPEOF_REGISTER_TYPE(OpenRAVE::SensorBase)
BOOST_TYPEOF_REGISTER_TYPE(OpenRAVE::SensorBase::SensorData)
BOOST_TYPEOF_REGISTER_TYPE(OpenRAVE::SensorSystemBase)
BOOST_TYPEOF_REGISTER_TYPE(OpenRAVE::SimpleSensorSystem)
BOOST_TYPEOF_REGISTER_TYPE(OpenRAVE::SimpleSensorSystem::XMLData)
BOOST_TYPEOF_REGISTER_TYPE(OpenRAVE::IkSolverBase)
BOOST_TYPEOF_REGISTER_TYPE(OpenRAVE::ViewerBase)
BOOST_TYPEOF_REGISTER_TYPE(OpenRAVE::GraphHandle)
BOOST_TYPEOF_REGISTER_TYPE(OpenRAVE::IkParameterization)
BOOST_TYPEOF_REGISTER_TEMPLATE(OpenRAVE::RaveVector, 1)
BOOST_TYPEOF_REGISTER_TEMPLATE(OpenRAVE::RaveTransform, 1)
BOOST_TYPEOF_REGISTER_TEMPLATE(OpenRAVE::RaveTransformMatrix, 1)

BOOST_TYPEOF_REGISTER_TYPE(OpenRAVE::KinBody)
BOOST_TYPEOF_REGISTER_TYPE(OpenRAVE::KinBody::Joint)
BOOST_TYPEOF_REGISTER_TYPE(OpenRAVE::KinBody::Joint::MIMIC)
BOOST_TYPEOF_REGISTER_TYPE(OpenRAVE::KinBody::Link)
BOOST_TYPEOF_REGISTER_TYPE(OpenRAVE::KinBody::Link::GEOMPROPERTIES)
BOOST_TYPEOF_REGISTER_TYPE(OpenRAVE::KinBody::Link::TRIMESH)
BOOST_TYPEOF_REGISTER_TYPE(OpenRAVE::KinBody::KinBodyStateSaver)
BOOST_TYPEOF_REGISTER_TYPE(OpenRAVE::KinBody::BodyState)
BOOST_TYPEOF_REGISTER_TYPE(OpenRAVE::KinBody::ManageData)

BOOST_TYPEOF_REGISTER_TYPE(OpenRAVE::RobotBase)
BOOST_TYPEOF_REGISTER_TYPE(OpenRAVE::RobotBase::Manipulator)
BOOST_TYPEOF_REGISTER_TYPE(OpenRAVE::RobotBase::AttachedSensor)
BOOST_TYPEOF_REGISTER_TYPE(OpenRAVE::RobotBase::GRABBED)
BOOST_TYPEOF_REGISTER_TYPE(OpenRAVE::RobotBase::RobotStateSaver)

BOOST_TYPEOF_REGISTER_TYPE(OpenRAVE::TrajectoryBase)
BOOST_TYPEOF_REGISTER_TYPE(OpenRAVE::TrajectoryBase::TPOINT)
BOOST_TYPEOF_REGISTER_TYPE(OpenRAVE::TrajectoryBase::TSEGMENT)

BOOST_TYPEOF_REGISTER_TYPE(OpenRAVE::PLUGININFO)
BOOST_TYPEOF_REGISTER_TYPE(OpenRAVE::XMLReadable)
BOOST_TYPEOF_REGISTER_TYPE(OpenRAVE::InterfaceBase)
BOOST_TYPEOF_REGISTER_TYPE(OpenRAVE::BaseXMLReader)

BOOST_TYPEOF_REGISTER_TYPE(OpenRAVE::EnvironmentBase)
BOOST_TYPEOF_REGISTER_TYPE(OpenRAVE::EnvironmentBase::BODYSTATE)

BOOST_TYPEOF_REGISTER_TYPE(OpenRAVE::RAY)
BOOST_TYPEOF_REGISTER_TYPE(OpenRAVE::AABB)
BOOST_TYPEOF_REGISTER_TYPE(OpenRAVE::OBB)
BOOST_TYPEOF_REGISTER_TYPE(OpenRAVE::TRIANGLE)
#endif


#endif