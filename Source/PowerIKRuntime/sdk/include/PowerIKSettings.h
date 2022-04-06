/*
Copyright Power Animated Inc., All Rights Reserved.
Unauthorized copying, selling or distribution of this software is strictly prohibited.
*/

#pragma once

#include "PowerIK.h"
#include "PowerIKMath.h"

namespace PowerIK
{
	/** Error code returned by all PowerIK SDK functions. */
	enum class PowerIKErrorCode : unsigned int 
	{
		SUCCESS,
		FAILURE,
		OUT_OF_BOUNDS
	};

	/** The settings used for the core solver. These setting only have to be set once. Though they can all be changed at runtime if desired. */
	struct SolverSettings
	{
		/** Number of iterations to determine squashed limb shape. Default is 6. */
		unsigned int SquashIterations = 6;
		/** Number of iterations to determine stretched limb shape. Default is 6. */
		unsigned int StretchIterations = 6;
		/** Number of iterations to determine stretched limb shape. Default is 3. */
		unsigned int FinalIterations = 3;
		/** Up direction of world. Used for Center of Gravity and Grounding. Default is +Z (Unreal).*/
		Vec3 WorldUpNormal = Vec3(0.0f, 0.0f, 1.0f); 
		/** Amount to rotate the root in response to effectors. Default is 1.0. Value as high as 3 or 4 may be necessary in some cases.*/
		float RootRotationMultiplier = 1.0f;
		/** If true, bone lengths will be updated each frame based on the input pose. This allows bone chains to 'stretch' but has a small additional cost to support.*/
		bool AllowBoneTranslation = false;
	};

	/** The settings used to control the inertial smoothing of the character's body (parts not controlled by effectors). */
	struct InertiaSettings
	{
		/** Whether to use inertia to smooth body motion. Default is false.*/
		bool ApplyInertiaToBody = false;

		/** Amount to smooth the body motion as it adapts to effectors. Typical values between 2 and 20. Higher values reduce the body motion. */
		float SmoothFactor = 5.0f;

		/** Alternative to SmoothFactor, uses a spring to pull the body towards the effectors (springs are allowed to overshoot).*/
		bool UseSpring = false;
		/** The strength of the spring attached the root to the solved position. */
		float SpringStrength = 100.0f;
		/** Dampen the motion over time. Range is 0-1. Default is 0.2. )*/
		float SpringDamping = 0.01f;
	};

	/** The settings used to control the center of gravity constraint. */
	struct CenterOfGravitySettings
	{
		/** Amount to apply center of gravity constraint. Range is 0-1. Default is 0.*/
		float Alpha = 0.0f;
		/** Amount to counteract root position in horizontal plane (relative to gravity). Range is 0-inf. Default is 1.2.*/
		float HorizAmount = 1.2f;
		/** Amount to counteract root position vertically (opposite to gravity). Range is 0-inf. Default is 0.3.*/
		float VertAmount = 0.3f;
		/** Amount to pull the rest of the body connected to the root. Range is 0-inf. Default is 0.4.*/
		float PullBodyAmount = 0.4f;
	};

	/** The settings used control the position and rotation smoothing of effectors. */
	struct EffectorSmoothingSettings
	{
		/** If true, the position of the effector will be smoothed over time.*/
		bool SmoothPositionOverTime = false;
		/** Maximum speed an effector can move in centimeters per second.*/
		float MaxPositionSpeed = 200.0f;
		/** Maximum distance an effector can be from it's input position in centimeters.*/
		float MaxPositionDistance = 1000.0f;
		/** If true, the rotation of the effector will be smoothed over time.*/
		bool SmoothRotationOverTime = false;
		/** Maximum speed an effector can rotate in degrees per second.*/
		float MaxDegreesSpeed = 90.0f;
		/** Maximum distance an effector angle can be from it's input rotation in degrees.*/
		float MaxDegreesDistance = 90.0f;
	};

	/** The modes for a PowerIK effector Pole Vector. */
	enum class PoleVectorMode : unsigned int
	{
		NONE,
		POSITION,
		BONE,
		ANGLE_OFFSET
	};

	/** The settings that define the Pole Vector behavior for an Effector. */
	struct EffectorPoleVector
	{
		/** Type of Pole Vector. Either None, Position, Bone or Parameter. */
		PoleVectorMode Mode = PoleVectorMode::NONE;
		/** The Character Space position that the limb should point in. This may be continuously updated.*/
		Vec3 Position = Vec3(0.0f, 100.0f, 0.0f);
		/** The index of the bone that the pole vector should point towards. This cannot be changed after first call to Solve(). */
		unsigned int BoneIndex = 0;
		/** An angle to offset the direction of the pole vector. This may be continuously updated. */
		float AngleOffset = 0.0f;
	};
}