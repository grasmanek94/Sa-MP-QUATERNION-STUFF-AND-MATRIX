#include <iostream>
#include <plugin.h>
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

const float MATH_PI = 3.14159265359f;
const float RAD_TO_DEG = 180.0f/MATH_PI;
const float DEG_TO_RAD = 1.0f/RAD_TO_DEG;

extern void* pAMXFunctions;

PLUGIN_EXPORT unsigned int PLUGIN_CALL Supports() 
{
	return SUPPORTS_VERSION | SUPPORTS_AMX_NATIVES;
}

PLUGIN_EXPORT bool PLUGIN_CALL Load( void **ppData ) 
{
	pAMXFunctions = ppData[PLUGIN_DATA_AMX_EXPORTS];
	return true;
}

PLUGIN_EXPORT void PLUGIN_CALL Unload( )
{
}

inline void AmxSetQuat(AMX * amx, cell * &params, unsigned char startpos , glm::quat quat)
{
	cell* addr;
	amx_GetAddr(amx,params[startpos],&addr);
	*addr = amx_ftoc(quat.w);
	amx_GetAddr(amx,params[startpos+1],&addr);
	*addr = amx_ftoc(quat.x);
	amx_GetAddr(amx,params[startpos+2],&addr);
	*addr = amx_ftoc(quat.y);
	amx_GetAddr(amx,params[startpos+3],&addr);
	*addr = amx_ftoc(quat.z);
}

inline void AmxSetVector3(AMX * amx, cell * &params, unsigned char startpos, glm::vec3 vector)
{
	cell* addr;
	amx_GetAddr(amx,params[startpos],&addr);
	*addr = amx_ftoc(vector.x);
	amx_GetAddr(amx,params[startpos+1],&addr);
	*addr = amx_ftoc(vector.y);
	amx_GetAddr(amx,params[startpos+2],&addr);
	*addr = amx_ftoc(vector.z);
}

inline void AmxSetVector3Inverse(AMX * amx, cell * &params, unsigned char startpos, glm::vec3 vector)
{
	cell* addr;
	amx_GetAddr(amx,params[startpos],&addr);
	*addr = amx_ftoc(vector.z);
	amx_GetAddr(amx,params[startpos+1],&addr);
	*addr = amx_ftoc(vector.y);
	amx_GetAddr(amx,params[startpos+2],&addr);
	*addr = amx_ftoc(vector.x);
}

inline void AmxSetVector2(AMX * amx, cell * &params, unsigned char startpos, glm::vec2 vector)
{
	cell* addr;
	amx_GetAddr(amx,params[startpos],&addr);
	*addr = amx_ftoc(vector.x);
	amx_GetAddr(amx,params[startpos+1],&addr);
	*addr = amx_ftoc(vector.y);
}

inline void AmxSetVector2Inverse(AMX * amx, cell * &params, unsigned char startpos, glm::vec2 vector)
{
	cell* addr;
	amx_GetAddr(amx,params[startpos],&addr);
	*addr = amx_ftoc(vector.y);
	amx_GetAddr(amx,params[startpos+1],&addr);
	*addr = amx_ftoc(vector.x);
}

inline void AmxSetFloat(AMX * amx,cell &param, float val)
{
	cell* addr;
	amx_GetAddr(amx,param,&addr);
	*addr = amx_ftoc(val);
}

inline float AmxGetFloat(cell &param)
{
	return amx_ctof(param);
}

inline glm::quat AmxGetQuat(cell * &params, unsigned char startpos)
{
	return glm::quat(AmxGetFloat(params[startpos]),AmxGetFloat(params[startpos+1]),AmxGetFloat(params[startpos+2]),AmxGetFloat(params[startpos+3]));
}

inline glm::vec3 AmxGetVector3(cell * &params, unsigned char startpos)
{
	return glm::vec3(AmxGetFloat(params[startpos]),AmxGetFloat(params[startpos+1]),AmxGetFloat(params[startpos+2]));
}

inline glm::vec2 AmxGetVector2(cell * &params, unsigned char startpos)
{
	return glm::vec2(AmxGetFloat(params[startpos]),AmxGetFloat(params[startpos+1]));
}

inline glm::vec2 GetPitchYawBetweenCoords(const glm::vec3 &source, const glm::vec3 &target)
{
	glm::vec2 output;
	glm::vec3 result = target-source;
	output.x = atan2( result.y, result.x ) * RAD_TO_DEG;//yaw?
	output.y = atan2( result.z, sqrt( result.x * result.x + result.y * result.y ) ) * RAD_TO_DEG;//pitch?
	return output;
}

inline void GetPitchYawBetweenCoords(const glm::vec3 &source, const glm::vec3 &target, glm::vec2 &output)
{
	glm::vec3 result = target-source;
	output.x = atan2( result.y, result.x ) * RAD_TO_DEG;//yaw
	output.y = atan2( result.z, sqrt( result.x * result.x + result.y * result.y ) ) * RAD_TO_DEG;//pitch
}

static cell AMX_NATIVE_CALL QuatToEuler( AMX* amx, cell* params )
{
	AmxSetVector3(amx,params,1, glm::vec3( glm::eulerAngles( AmxGetQuat(params,4) ) ) );
	return 1;
}

static cell AMX_NATIVE_CALL EulerToQuat( AMX* amx, cell* params )
{
	AmxSetQuat(amx,params,4,glm::quat(AmxGetVector3(params,1)*DEG_TO_RAD));
	return 1;
}

static cell AMX_NATIVE_CALL GetPitchYawBetweenPositions( AMX* amx, cell* params )
{
	glm::vec2 rot;
	GetPitchYawBetweenCoords(AmxGetVector3(params,1),AmxGetVector3(params,4),rot);
	AmxSetVector2Inverse(amx,params,7,rot);
	return 1;
}

static cell AMX_NATIVE_CALL GetPitchYawRollBetweenPositions( AMX* amx, cell* params )
{
	glm::vec3 points[3] = {AmxGetVector3(params,4),AmxGetVector3(params,1),AmxGetVector3(params,7)};
	glm::vec2 YawPitchRotations[2];
	GetPitchYawBetweenCoords(points[0]+((points[1]-points[0])/2.0f),points[2],YawPitchRotations[0]);
	GetPitchYawBetweenCoords(points[0],points[1],YawPitchRotations[1]);
	AmxSetVector3Inverse(amx,params,10,glm::vec3(YawPitchRotations[1].y,YawPitchRotations[0].x,YawPitchRotations[0].y));
	return 1;
}

static cell AMX_NATIVE_CALL GetQuatRotBetweenPositions3D( AMX* amx, cell* params )
{
	glm::vec3 points[3] = {AmxGetVector3(params,4),AmxGetVector3(params,1),AmxGetVector3(params,7)};
	glm::vec2 YawPitchRotations[2];
	GetPitchYawBetweenCoords(points[0]+((points[1]-points[0])/2.0f),points[2],YawPitchRotations[0]);
	GetPitchYawBetweenCoords(points[0],points[1],YawPitchRotations[1]);
	glm::vec3 eulerangles(YawPitchRotations[0].y,YawPitchRotations[0].x,YawPitchRotations[1].x);
	AmxSetQuat(amx,params,10,glm::quat(eulerangles*DEG_TO_RAD));
	return 1;
}

static cell AMX_NATIVE_CALL GetQuatRotBetweenPositions2D( AMX* amx, cell* params )
{
	glm::vec2 rot;
	GetPitchYawBetweenCoords(AmxGetVector3(params,1),AmxGetVector3(params,4),rot);
	glm::vec3 eulerangles(rot.y,rot.x,0.0f);
	AmxSetQuat(amx,params,7,glm::quat(eulerangles*DEG_TO_RAD));
	return 1;
}

//gets the correct quaternion between two positions FOR A VEHICLE
static cell AMX_NATIVE_CALL GetQuatRotForVehBetweenCoords2D( AMX* amx, cell* params )
{
	glm::vec2 rot;
	GetPitchYawBetweenCoords(AmxGetVector3(params,1),AmxGetVector3(params,4),rot);
	rot.x = std::fmod(450.0f-rot.x,360.0f);
	if(rot.x < 0.0f)
		rot.x += 360.0f;
	AmxSetQuat(amx,params,7,glm::quat(glm::vec3(-rot.y*cos(rot.x*DEG_TO_RAD), -rot.y*sin(-rot.x*DEG_TO_RAD), rot.x)*DEG_TO_RAD));
	return 1;
}

//now the freaking most advanced thing of all shit here in this plugin
//this is not perfect but the closes I can get to a quaternion representing all axes (Pitch,Roll,Yaw) based on 3 points
static cell AMX_NATIVE_CALL GetQuatRotForVehBetweenCoords3D( AMX* amx, cell* params )
{
	glm::vec3 points[3] = {AmxGetVector3(params,4),AmxGetVector3(params,1),AmxGetVector3(params,7)};
	glm::vec2 YawPitchRotations[2];
	GetPitchYawBetweenCoords(points[0]+((points[1]-points[0])/2.0f),points[2],YawPitchRotations[0]);
	GetPitchYawBetweenCoords(points[0],points[1],YawPitchRotations[1]);
	//AmxSetVector3Inverse(amx,params,10,glm::vec3(YawPitchRotations[1].y,YawPitchRotations[0].x,YawPitchRotations[0].y));
	YawPitchRotations[0].x = std::fmod(450.0f-YawPitchRotations[0].x,360.0f);
	if(YawPitchRotations[0].x < 0.0f)
		YawPitchRotations[0].x += 360.0f;

	float PitchDc = -YawPitchRotations[0].y*cos(YawPitchRotations[0].x*DEG_TO_RAD);
	float PitchDs = -YawPitchRotations[0].y*sin(-YawPitchRotations[0].x*DEG_TO_RAD);
	float RollDc = YawPitchRotations[1].y*cos(YawPitchRotations[0].x*DEG_TO_RAD);
	float RollDs = YawPitchRotations[1].y*sin(YawPitchRotations[0].x*DEG_TO_RAD);
	AmxSetQuat
	(
		amx,params,10,
		glm::quat
		(
			glm::vec3
			(
				RollDs+PitchDc, 
				RollDc+PitchDs,
				YawPitchRotations[0].x
			)
			*DEG_TO_RAD
		)
	);
	return 1;
}

#define native(name) {#name,name}

AMX_NATIVE_INFO AMXNatives[ ] =
{
	native(QuatToEuler),
	native(EulerToQuat),
	native(GetQuatRotBetweenPositions2D),
	native(GetQuatRotBetweenPositions3D),
	native(GetPitchYawBetweenPositions),
	native(GetPitchYawRollBetweenPositions),
	native(GetQuatRotForVehBetweenCoords2D),
	native(GetQuatRotForVehBetweenCoords3D),
	{0,                0}
};

PLUGIN_EXPORT int PLUGIN_CALL AmxLoad( AMX *amx ) 
{
	return amx_Register( amx, AMXNatives, -1 );
}

PLUGIN_EXPORT int PLUGIN_CALL AmxUnload( AMX *amx ) 
{
	return AMX_ERR_NONE;
}

