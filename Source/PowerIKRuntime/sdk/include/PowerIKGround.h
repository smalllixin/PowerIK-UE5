/*
Copyright Power Animated Inc., All Rights Reserved.
Unauthorized copying, selling or distribution of this software is strictly prohibited.
*/
#pragma once

#include "PowerIKMath.h"
#include <vector>


namespace PowerIK
{

struct GroundFoot
{
	Transform Transform;
	Vec3 Normal;
};

struct GroundSettings
{
	/** Vector that is opposite gravity. This is usually constant. Default is +Z. */
	Vec3 WorldUpNormal = Vec3(0.0f, 0.0f, 1.0f);
	/** Maximum speed that ground normal is allowed to rotate in degrees per second. */
	float MaxNormalAngularSpeed = 75.0f;
	/** Normalized vector that describes the direction of travel. */
	Vec3 StrideDirection = Vec3(1.0f, 0.0f, 0.0f);
	/** Maximum angle relative to gravity vector to apply leaning and stride scaling. */
	float MaxGroundAngle = 45.0f;

	/** If true, entire skeleton is rotated to align to the ground. Useful for quadrupeds.*/
	bool OrientToGround = true;
	/** Orient entire skeleton to the PITCH direction of ground angle. Range is -1 to 1. Default 0.9. */
	float OrientToPitch = 0.8f;
	/** Orient entire skeleton to the ROLL direction of ground angle. Range is -1 to 1. Default 0.1. */
	float OrientToRoll = 0.0f;

	/** If true, stride of IsGrounded effectors will be scaled in Stride Direction. */
	bool ScaleStride = true;
	/** Amount to scale stride when going uphill. Range is 0-1. Default 0.4.*/
	float UphillStrideScale = 0.4f;
	/** Amount to scale stride when going downhill. Range is 0-1. Default 0.2.*/
	float DownhillStrideScale = 0.2f;
	/** Amount to scale stride when on a side-hill. Range is 0-1. Default 1.0.*/
	float SidehillStrideScale = 1.0f;
	/** Amount (in centimeters) to push outside feet outwards on sidehills. Default is 20.0.*/
	float SidehillPushOuterFeet = 20.0f;
	/** Amount (in centimeters) to push inside feet outwards on sidehills. Default is 10.0.*/
	float SidehillPushInnerFeet = 10.0f;

	/** If True, character root bone is rotated in opposite direction of ground normal. Useful for bipeds. */
	bool Lean = true;
	/** Amount to lean forward when going uphill. Range is 0-1. Default 0.3.*/
	float UphillLean = 0.3f;
	/** Amount to lean backward when going downhill. Range is 0-1. Default 0.5.*/
	float DownhillLean = 0.5f;
	/** Amount to lean sideways into side-hills. Range is 0-1. Default 0.0.*/
	float SidehillLean = 0.0f;

	/** If True, will counter lean CounterLeanBone. Usually this is the head. */
	bool CounterLean = true;
	/** Name of base of counter lean bone. Usually this is the base Head bone.*/
	const char* CounterLeanBoneName = "head";
	/** Amount to counter lean backward when going uphill. Range is 0-1. Default 0.8.*/
	float UphillCounterLean = 0.8f;
	/** Amount to counter lean backward when going downhill. Range is 0-1. Default 1.0.*/
	float DownhillCounterLean = 1.0f;
	/** Amount to counter lean sideways into side-hills. Range is 0-1. Default 0.0.*/
	float SidehillCounterLean = 0.0f;

	/** If True, character root bone position is gradually pushed vertically and horizontally. */
	bool MoveRoot = true;
	/** Amount (in centimeters) to push the character root up/down when going UP hill. Default 10.0.*/
	float UphillVertOffset = 10.0f;
	/** Amount (in centimeters) to push the character root forward/back when going UP hill. Default -20.0.*/
	float UphillHorizOffset = -20.0f;
	/** Amount (in centimeters) to push the character root up/down when going DOWN hill. Default 10.0.*/
	float DownhillVertOffset = 10.0f;
	/** Amount (in centimeters) to push the character root forward/back when going DOWN hill. Default 20.0.*/
	float DownhillHorizOffset = 20.0f;
	/** Amount (in centimeters) to push the character root up/down when on a SIDE hill. Default -10.0.*/
	float SidehillVertOffset = -10.0f;
	/** Amount (in centimeters) to push the character root forward/back when on a SIDE hill. Default -10.0.*/
	float SidehillHorizOffset = -10.0f;

	/** If True, feet are oriented to ground normal. */
	bool RotateFootToGround = true;
	/** Amount to pitch foot to orient to up/down hills. Range is 0-1. Default 1.0.*/
	float PitchFootAmount = 1.0f;
	/** Amount to roll foot to orient to sidehills. Range is 0-1. Default 0.6.*/
	float RollFootAmount = 0.6f;

	/** If True, applies static offset to foot positions along normal direction. Useful for nudging feet into contact with ground. */
	bool OffsetFeetPositions = false;
	/** Amount in centimeters to offset the foot in the direction of the foot's ground normal. */
	float StaticFootOffset = 4.0f;
};


/** 
 */
class GroundAlign
{
public:
	
	/**
	* This class calculates several modifications to be made to a skeleton
	* based on the slope of the ground it is walking on. These modifications are then
	* applied before the core solver is run.
	*
	* This is an example of how to create a system that interacts with
	* the core PowerIK solver. The GroundAlign class takes feet transforms as input
	* and produces a root transform and modified feet transforms as output.
	*
	* The output FEET transforms are applied directly to the PowerIK effectors before Solve() is called.
	* The output ROOT transform is also sent directly to PowerIK via:
	* Solver::TranslateSkeletonBeforeSolve()
	* Solver::RotateSkeletonBeforeSolve()
	*
	* These functions move the entire skeleton at an early stage in the solver and will override
	* the input pose. This allows for full-body skeletal adjustments without the need for a separate node.
	*
	* You can use this as a starting place for your own project ground alignment solution,
	* or as an example for other animation systems that prepare the skeleton
	* and/or effectors procedurally before running Solve().
	* 
	* @param: Root - the solver root bone transform in character space
	* @param: OutRootOffset - OUT offset to apply to the solver root
	* @param: OutFeet - IN/OUT array of GroundFoot structs, 1 per foot effector
	* @param: NumFeet - number of elements of OutFeet
	* @param: OutCounterLeanRotation - OUTPUT relative rotation to apply to counter lean bone
	* @param: AngularDeltaSmoothSpeed - speed in degrees per second that ground normal delta can change 
	* @param: DeltaTime - time in seconds since last tick
	*/
	void CalculateGroundAlignment(
		const Transform& Root,
		Transform& OutRootOffset,
		GroundFoot *OutFeet,
		unsigned int NumFeet,
		Quat& OutCounterLeanRotation,
		float DeltaTime);

	GroundSettings Settings;

private:

	Quat PrevRotationDelta = Quat::Identity();
};

};