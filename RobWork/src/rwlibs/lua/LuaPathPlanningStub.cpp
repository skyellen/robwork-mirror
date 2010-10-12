/*
** Lua binding: LuaPathPlanning
** Generated automatically by tolua++-1.0.92 on 10/05/10 22:20:09.
*/

#ifndef __cplusplus
#include "stdlib.h"
#endif
#include "string.h"

#include "tolua++.h"

/* Exported function */
TOLUA_API int  tolua_LuaPathPlanning_open (lua_State* tolua_S);

#include <rw/math.hpp>
#include <rw/trajectory.hpp>
#include <rw/pathplanning.hpp>
#include <rwlibs/pathplanners/rrt/RRTPlanner.hpp>
#include <rwlibs/pathplanners/sbl/SBLPlanner.hpp>
#include <rwlibs/pathplanners/prm/PRMPlanner.hpp>
#include <rwlibs/pathoptimization/clearance/ClearanceOptimizer.hpp>
#include <rwlibs/pathoptimization/clearance/MinimumClearanceCalculator.hpp>
#include <rwlibs/pathoptimization/pathlength/PathLengthOptimizer.hpp>
#include "LuaKinematics.hpp"
#include "LuaTrajectory.hpp"
#include "LuaPathPlanning.hpp"

/* function to release collected object via destructor */
#ifdef __cplusplus

static int tolua_collect_rwlibs__lua__StopCriteriaPtr (lua_State* tolua_S)
{
 rwlibs::lua::StopCriteriaPtr* self = (rwlibs::lua::StopCriteriaPtr*) tolua_tousertype(tolua_S,1,0);
	Mtolua_delete(self);
	return 0;
}
#endif


/* function to register type */
static void tolua_reg_types (lua_State* tolua_S)
{
 tolua_usertype(tolua_S,"rwlibs::lua::Q");
 tolua_usertype(tolua_S,"rwlibs::lua::StopCriteria");
 tolua_usertype(tolua_S,"rwlibs::lua::QSampler");
 tolua_usertype(tolua_S,"rwlibs::lua::RRTPlanner");
 tolua_usertype(tolua_S,"std::vector<rwlibs::lua::Q>");
 tolua_usertype(tolua_S,"rwlibs::lua::PlannerConstraint");
 tolua_usertype(tolua_S,"rwlibs::lua::QToQPlanner");
}

/* method: stopAfter of class  rwlibs::lua::StopCriteria */
#ifndef TOLUA_DISABLE_tolua_LuaPathPlanning_rwlibs_lua_StopCriteria_stopAfter00
static int tolua_LuaPathPlanning_rwlibs_lua_StopCriteria_stopAfter00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertable(tolua_S,1,"rwlibs::lua::StopCriteria",0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  double time = ((double)  tolua_tonumber(tolua_S,2,0));
  {
   rwlibs::lua::StopCriteriaPtr tolua_ret = (rwlibs::lua::StopCriteriaPtr)  rwlibs::lua::StopCriteria::stopAfter(time);
   {
#ifdef __cplusplus
    void* tolua_obj = Mtolua_new((rwlibs::lua::StopCriteriaPtr)(tolua_ret));
     tolua_pushusertype(tolua_S,tolua_obj,"rwlibs::lua::StopCriteriaPtr");
    tolua_register_gc(tolua_S,lua_gettop(tolua_S));
#else
    void* tolua_obj = tolua_copy(tolua_S,(void*)&tolua_ret,sizeof(rwlibs::lua::StopCriteriaPtr));
     tolua_pushusertype(tolua_S,tolua_obj,"rwlibs::lua::StopCriteriaPtr");
    tolua_register_gc(tolua_S,lua_gettop(tolua_S));
#endif
   }
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'stopAfter'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: stopNever of class  rwlibs::lua::StopCriteria */
#ifndef TOLUA_DISABLE_tolua_LuaPathPlanning_rwlibs_lua_StopCriteria_stopNever00
static int tolua_LuaPathPlanning_rwlibs_lua_StopCriteria_stopNever00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertable(tolua_S,1,"rwlibs::lua::StopCriteria",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  {
   rwlibs::lua::StopCriteriaPtr tolua_ret = (rwlibs::lua::StopCriteriaPtr)  rwlibs::lua::StopCriteria::stopNever();
   {
#ifdef __cplusplus
    void* tolua_obj = Mtolua_new((rwlibs::lua::StopCriteriaPtr)(tolua_ret));
     tolua_pushusertype(tolua_S,tolua_obj,"rwlibs::lua::StopCriteriaPtr");
    tolua_register_gc(tolua_S,lua_gettop(tolua_S));
#else
    void* tolua_obj = tolua_copy(tolua_S,(void*)&tolua_ret,sizeof(rwlibs::lua::StopCriteriaPtr));
     tolua_pushusertype(tolua_S,tolua_obj,"rwlibs::lua::StopCriteriaPtr");
    tolua_register_gc(tolua_S,lua_gettop(tolua_S));
#endif
   }
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'stopNever'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: stopNow of class  rwlibs::lua::StopCriteria */
#ifndef TOLUA_DISABLE_tolua_LuaPathPlanning_rwlibs_lua_StopCriteria_stopNow00
static int tolua_LuaPathPlanning_rwlibs_lua_StopCriteria_stopNow00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertable(tolua_S,1,"rwlibs::lua::StopCriteria",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  {
   rwlibs::lua::StopCriteriaPtr tolua_ret = (rwlibs::lua::StopCriteriaPtr)  rwlibs::lua::StopCriteria::stopNow();
   {
#ifdef __cplusplus
    void* tolua_obj = Mtolua_new((rwlibs::lua::StopCriteriaPtr)(tolua_ret));
     tolua_pushusertype(tolua_S,tolua_obj,"rwlibs::lua::StopCriteriaPtr");
    tolua_register_gc(tolua_S,lua_gettop(tolua_S));
#else
    void* tolua_obj = tolua_copy(tolua_S,(void*)&tolua_ret,sizeof(rwlibs::lua::StopCriteriaPtr));
     tolua_pushusertype(tolua_S,tolua_obj,"rwlibs::lua::StopCriteriaPtr");
    tolua_register_gc(tolua_S,lua_gettop(tolua_S));
#endif
   }
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'stopNow'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: stopByFlag of class  rwlibs::lua::StopCriteria */
#ifndef TOLUA_DISABLE_tolua_LuaPathPlanning_rwlibs_lua_StopCriteria_stopByFlag00
static int tolua_LuaPathPlanning_rwlibs_lua_StopCriteria_stopByFlag00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertable(tolua_S,1,"rwlibs::lua::StopCriteria",0,&tolua_err) ||
     !tolua_isboolean(tolua_S,2,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  bool stop = ((bool)  tolua_toboolean(tolua_S,2,0));
  {
   rwlibs::lua::StopCriteriaPtr tolua_ret = (rwlibs::lua::StopCriteriaPtr)  rwlibs::lua::StopCriteria::stopByFlag(&stop);
   {
#ifdef __cplusplus
    void* tolua_obj = Mtolua_new((rwlibs::lua::StopCriteriaPtr)(tolua_ret));
     tolua_pushusertype(tolua_S,tolua_obj,"rwlibs::lua::StopCriteriaPtr");
    tolua_register_gc(tolua_S,lua_gettop(tolua_S));
#else
    void* tolua_obj = tolua_copy(tolua_S,(void*)&tolua_ret,sizeof(rwlibs::lua::StopCriteriaPtr));
     tolua_pushusertype(tolua_S,tolua_obj,"rwlibs::lua::StopCriteriaPtr");
    tolua_register_gc(tolua_S,lua_gettop(tolua_S));
#endif
   }
   tolua_pushboolean(tolua_S,(bool)stop);
  }
 }
 return 2;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'stopByFlag'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: stopCnt of class  rwlibs::lua::StopCriteria */
#ifndef TOLUA_DISABLE_tolua_LuaPathPlanning_rwlibs_lua_StopCriteria_stopCnt00
static int tolua_LuaPathPlanning_rwlibs_lua_StopCriteria_stopCnt00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertable(tolua_S,1,"rwlibs::lua::StopCriteria",0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  int cnt = ((int)  tolua_tonumber(tolua_S,2,0));
  {
   rwlibs::lua::StopCriteriaPtr tolua_ret = (rwlibs::lua::StopCriteriaPtr)  rwlibs::lua::StopCriteria::stopCnt(cnt);
   {
#ifdef __cplusplus
    void* tolua_obj = Mtolua_new((rwlibs::lua::StopCriteriaPtr)(tolua_ret));
     tolua_pushusertype(tolua_S,tolua_obj,"rwlibs::lua::StopCriteriaPtr");
    tolua_register_gc(tolua_S,lua_gettop(tolua_S));
#else
    void* tolua_obj = tolua_copy(tolua_S,(void*)&tolua_ret,sizeof(rwlibs::lua::StopCriteriaPtr));
     tolua_pushusertype(tolua_S,tolua_obj,"rwlibs::lua::StopCriteriaPtr");
    tolua_register_gc(tolua_S,lua_gettop(tolua_S));
#endif
   }
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'stopCnt'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: stopEither of class  rwlibs::lua::StopCriteria */
#ifndef TOLUA_DISABLE_tolua_LuaPathPlanning_rwlibs_lua_StopCriteria_stopEither00
static int tolua_LuaPathPlanning_rwlibs_lua_StopCriteria_stopEither00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertable(tolua_S,1,"rwlibs::lua::StopCriteria",0,&tolua_err) ||
     (tolua_isvaluenil(tolua_S,2,&tolua_err) || !tolua_isusertype(tolua_S,2,"const rwlibs::lua::StopCriteriaPtr",0,&tolua_err)) ||
     (tolua_isvaluenil(tolua_S,3,&tolua_err) || !tolua_isusertype(tolua_S,3,"const rwlibs::lua::StopCriteriaPtr",0,&tolua_err)) ||
     !tolua_isnoobj(tolua_S,4,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  const rwlibs::lua::StopCriteriaPtr* a = ((const rwlibs::lua::StopCriteriaPtr*)  tolua_tousertype(tolua_S,2,0));
  const rwlibs::lua::StopCriteriaPtr* b = ((const rwlibs::lua::StopCriteriaPtr*)  tolua_tousertype(tolua_S,3,0));
  {
   rwlibs::lua::StopCriteriaPtr tolua_ret = (rwlibs::lua::StopCriteriaPtr)  rwlibs::lua::StopCriteria::stopEither(*a,*b);
   {
#ifdef __cplusplus
    void* tolua_obj = Mtolua_new((rwlibs::lua::StopCriteriaPtr)(tolua_ret));
     tolua_pushusertype(tolua_S,tolua_obj,"rwlibs::lua::StopCriteriaPtr");
    tolua_register_gc(tolua_S,lua_gettop(tolua_S));
#else
    void* tolua_obj = tolua_copy(tolua_S,(void*)&tolua_ret,sizeof(rwlibs::lua::StopCriteriaPtr));
     tolua_pushusertype(tolua_S,tolua_obj,"rwlibs::lua::StopCriteriaPtr");
    tolua_register_gc(tolua_S,lua_gettop(tolua_S));
#endif
   }
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'stopEither'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: query of class  rwlibs::lua::QToQPlanner */
#ifndef TOLUA_DISABLE_tolua_LuaPathPlanning_rwlibs_lua_QToQPlanner_query00
static int tolua_LuaPathPlanning_rwlibs_lua_QToQPlanner_query00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"rwlibs::lua::QToQPlanner",0,&tolua_err) ||
     (tolua_isvaluenil(tolua_S,2,&tolua_err) || !tolua_isusertype(tolua_S,2,"const rwlibs::lua::Q",0,&tolua_err)) ||
     (tolua_isvaluenil(tolua_S,3,&tolua_err) || !tolua_isusertype(tolua_S,3,"const rwlibs::lua::Q",0,&tolua_err)) ||
     (tolua_isvaluenil(tolua_S,4,&tolua_err) || !tolua_isusertype(tolua_S,4,"std::vector<rwlibs::lua::Q>",0,&tolua_err)) ||
     (tolua_isvaluenil(tolua_S,5,&tolua_err) || !tolua_isusertype(tolua_S,5,"const rwlibs::lua::StopCriteria",0,&tolua_err)) ||
     !tolua_isnoobj(tolua_S,6,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  rwlibs::lua::QToQPlanner* self = (rwlibs::lua::QToQPlanner*)  tolua_tousertype(tolua_S,1,0);
  const rwlibs::lua::Q* from = ((const rwlibs::lua::Q*)  tolua_tousertype(tolua_S,2,0));
  const rwlibs::lua::Q* to = ((const rwlibs::lua::Q*)  tolua_tousertype(tolua_S,3,0));
  std::vector<rwlibs::lua::Q>* path = ((std::vector<rwlibs::lua::Q>*)  tolua_tousertype(tolua_S,4,0));
  const rwlibs::lua::StopCriteria* stop = ((const rwlibs::lua::StopCriteria*)  tolua_tousertype(tolua_S,5,0));
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'query'", NULL);
#endif
  {
   bool tolua_ret = (bool)  self->query(*from,*to,*path,*stop);
   tolua_pushboolean(tolua_S,(bool)tolua_ret);
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'query'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: query of class  rwlibs::lua::QToQPlanner */
#ifndef TOLUA_DISABLE_tolua_LuaPathPlanning_rwlibs_lua_QToQPlanner_query01
static int tolua_LuaPathPlanning_rwlibs_lua_QToQPlanner_query01(lua_State* tolua_S)
{
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"rwlibs::lua::QToQPlanner",0,&tolua_err) ||
     (tolua_isvaluenil(tolua_S,2,&tolua_err) || !tolua_isusertype(tolua_S,2,"const rwlibs::lua::Q",0,&tolua_err)) ||
     (tolua_isvaluenil(tolua_S,3,&tolua_err) || !tolua_isusertype(tolua_S,3,"const rwlibs::lua::Q",0,&tolua_err)) ||
     (tolua_isvaluenil(tolua_S,4,&tolua_err) || !tolua_isusertype(tolua_S,4,"std::vector<rwlibs::lua::Q>",0,&tolua_err)) ||
     !tolua_isnumber(tolua_S,5,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,6,&tolua_err)
 )
  goto tolua_lerror;
 else
 {
  rwlibs::lua::QToQPlanner* self = (rwlibs::lua::QToQPlanner*)  tolua_tousertype(tolua_S,1,0);
  const rwlibs::lua::Q* from = ((const rwlibs::lua::Q*)  tolua_tousertype(tolua_S,2,0));
  const rwlibs::lua::Q* to = ((const rwlibs::lua::Q*)  tolua_tousertype(tolua_S,3,0));
  std::vector<rwlibs::lua::Q>* path = ((std::vector<rwlibs::lua::Q>*)  tolua_tousertype(tolua_S,4,0));
  double time = ((double)  tolua_tonumber(tolua_S,5,0));
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'query'", NULL);
#endif
  {
   bool tolua_ret = (bool)  self->query(*from,*to,*path,time);
   tolua_pushboolean(tolua_S,(bool)tolua_ret);
  }
 }
 return 1;
tolua_lerror:
 return tolua_LuaPathPlanning_rwlibs_lua_QToQPlanner_query00(tolua_S);
}
#endif //#ifndef TOLUA_DISABLE

/* method: query of class  rwlibs::lua::QToQPlanner */
#ifndef TOLUA_DISABLE_tolua_LuaPathPlanning_rwlibs_lua_QToQPlanner_query02
static int tolua_LuaPathPlanning_rwlibs_lua_QToQPlanner_query02(lua_State* tolua_S)
{
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"rwlibs::lua::QToQPlanner",0,&tolua_err) ||
     (tolua_isvaluenil(tolua_S,2,&tolua_err) || !tolua_isusertype(tolua_S,2,"const rwlibs::lua::Q",0,&tolua_err)) ||
     (tolua_isvaluenil(tolua_S,3,&tolua_err) || !tolua_isusertype(tolua_S,3,"const rwlibs::lua::Q",0,&tolua_err)) ||
     (tolua_isvaluenil(tolua_S,4,&tolua_err) || !tolua_isusertype(tolua_S,4,"std::vector<rwlibs::lua::Q>",0,&tolua_err)) ||
     !tolua_isnoobj(tolua_S,5,&tolua_err)
 )
  goto tolua_lerror;
 else
 {
  rwlibs::lua::QToQPlanner* self = (rwlibs::lua::QToQPlanner*)  tolua_tousertype(tolua_S,1,0);
  const rwlibs::lua::Q* from = ((const rwlibs::lua::Q*)  tolua_tousertype(tolua_S,2,0));
  const rwlibs::lua::Q* to = ((const rwlibs::lua::Q*)  tolua_tousertype(tolua_S,3,0));
  std::vector<rwlibs::lua::Q>* path = ((std::vector<rwlibs::lua::Q>*)  tolua_tousertype(tolua_S,4,0));
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'query'", NULL);
#endif
  {
   bool tolua_ret = (bool)  self->query(*from,*to,*path);
   tolua_pushboolean(tolua_S,(bool)tolua_ret);
  }
 }
 return 1;
tolua_lerror:
 return tolua_LuaPathPlanning_rwlibs_lua_QToQPlanner_query01(tolua_S);
}
#endif //#ifndef TOLUA_DISABLE

/* Open function */
TOLUA_API int tolua_LuaPathPlanning_open (lua_State* tolua_S)
{
 tolua_open(tolua_S);
 tolua_reg_types(tolua_S);
 tolua_module(tolua_S,NULL,0);
 tolua_beginmodule(tolua_S,NULL);
  tolua_module(tolua_S,"rwlibs",0);
  tolua_beginmodule(tolua_S,"rwlibs");
   tolua_module(tolua_S,"lua",0);
   tolua_beginmodule(tolua_S,"lua");
    tolua_cclass(tolua_S,"QSampler","rwlibs::lua::QSampler","",NULL);
    tolua_beginmodule(tolua_S,"QSampler");
    tolua_endmodule(tolua_S);
    tolua_cclass(tolua_S,"PlannerConstraint","rwlibs::lua::PlannerConstraint","",NULL);
    tolua_beginmodule(tolua_S,"PlannerConstraint");
    tolua_endmodule(tolua_S);
    tolua_cclass(tolua_S,"StopCriteria","rwlibs::lua::StopCriteria","",NULL);
    tolua_beginmodule(tolua_S,"StopCriteria");
     tolua_function(tolua_S,"stopAfter",tolua_LuaPathPlanning_rwlibs_lua_StopCriteria_stopAfter00);
     tolua_function(tolua_S,"stopNever",tolua_LuaPathPlanning_rwlibs_lua_StopCriteria_stopNever00);
     tolua_function(tolua_S,"stopNow",tolua_LuaPathPlanning_rwlibs_lua_StopCriteria_stopNow00);
     tolua_function(tolua_S,"stopByFlag",tolua_LuaPathPlanning_rwlibs_lua_StopCriteria_stopByFlag00);
     tolua_function(tolua_S,"stopCnt",tolua_LuaPathPlanning_rwlibs_lua_StopCriteria_stopCnt00);
     tolua_function(tolua_S,"stopEither",tolua_LuaPathPlanning_rwlibs_lua_StopCriteria_stopEither00);
    tolua_endmodule(tolua_S);
    tolua_cclass(tolua_S,"QToQPlanner","rwlibs::lua::QToQPlanner","",NULL);
    tolua_beginmodule(tolua_S,"QToQPlanner");
     tolua_function(tolua_S,"query",tolua_LuaPathPlanning_rwlibs_lua_QToQPlanner_query00);
     tolua_function(tolua_S,"query",tolua_LuaPathPlanning_rwlibs_lua_QToQPlanner_query01);
     tolua_function(tolua_S,"query",tolua_LuaPathPlanning_rwlibs_lua_QToQPlanner_query02);
    tolua_endmodule(tolua_S);
    tolua_cclass(tolua_S,"RRTPlanner","rwlibs::lua::RRTPlanner","",NULL);
    tolua_beginmodule(tolua_S,"RRTPlanner");
    tolua_endmodule(tolua_S);
   tolua_endmodule(tolua_S);
  tolua_endmodule(tolua_S);
 tolua_endmodule(tolua_S);
 return 1;
}


#if defined(LUA_VERSION_NUM) && LUA_VERSION_NUM >= 501
 TOLUA_API int luaopen_LuaPathPlanning (lua_State* tolua_S) {
 return tolua_LuaPathPlanning_open(tolua_S);
};
#endif

