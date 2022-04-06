/*
Copyright Power Animated Inc., All Rights Reserved.
Unauthorized copying, selling or distribution of this software is strictly prohibited.
*/
#pragma once

/*
#ifdef _WIN32
	#define POWERIK_LIB_API __declspec(dllexport)
#endif
#ifdef __linux__
	#define POWERIK_LIB_API __attribute__((visibility("default")))
#endif
*/

//#define POWERIK_LIB_API DLLEXPORT


namespace PowerIK
{
	struct Vec3;
	struct Quat;
	struct Core;
	struct SolverSettings;
	struct CenterOfGravitySettings;
	struct InertiaSettings;
	struct GroundSettings;
	struct EffectorSmoothingSettings;
	struct EffectorPoleVector;
	enum class PowerIKErrorCode : unsigned int;

/** The core solver class for PowerIK. */
class Solver 
{

public:

	/** Creates a new instance of a PowerIK solver. Do this once per animated skeleton. */
	Solver();
	~Solver();

	/**
	 * Call this every tick to generate a new pose based on the input pose and the effectors.
	 * @param DeltaTime - the time (in seconds since the last call to Solve(). PowerIK is deterministic, this is only needed for effector smoothing.
	 * @param Alpha - blend the entire effect of the solver off and on (0-1). At 0, the solve is skipped entirely and the input pose is completely unchanged.
	 * @warning Call this after adding Bones via AddBone() and setting the Bone and Effector transforms
	 */
	void Solve(const float DeltaTime, const float Alpha) const;

	/**
	 * Call this to reset all internal data structures. Will not deallocate. Useful for recycling solver memory.
	 * @warning Adding a greater number of bones or effector (than before reset) may result in a new allocation.
	 */
	void Reset() const;

	/**
	 * Check if the solver has been initialized yet.
	 */
	bool IsInitialized() const;

	/**
	 * Call this on a new Solver() or only AFTER calling Reset() to expand/allocate space for a new skeleton.
	 * @warning Adding a greater number of bones or effectors (than before reset) may result in a new allocation.
	 */
	void SetNumBones(int NumBones) const;

	/** Get how many bones have been added to the Solver.
	 * @return - int, number of bones in the solver
	 */
	int GetNumBones() const;

	/**
	 * Call this for each bone AFTER calling SetNumBones(). Supplies hierarchy information to solver.
	 * @param BoneIndex - the index of the bone. Index must be >= 0 and less than NumBones.
	 * @param BoneName - the string name of the Bone. This name must match the name passed to AddEffector()
	 * @param ParentIndex - the index of the bone this bone is parented to or -1 for the root (no parent).
	 * @return - SUCCESS or OUT_OF_BOUNDS if either BoneIndex or ParentIndex are invalid
	 */
	PowerIKErrorCode AddBone(
		const int BoneIndex,
		const char* BoneName,
		const int ParentIndex) const;

	/**
	 * Call this AFTER called AddBone() for the whole skeleton. Adds an effector to a given bone (by name).
	 * @param BoneName - the string name of the Bone to affect. This name must match the name passed to AddBone().
	 * @param OutEffectorIndex - the index of the newly created effector. Store this and use it in SetEffectorTransform()
	 * @return - SUCCESS or FAILURE if BoneName is invalid
	 */
	PowerIKErrorCode AddEffector(
		const char* BoneName, 
		int& OutEffectorIndex) const;

	/**
	 * Get how many effectors have been added to the solver.
	 * @return - int, number of effectors in the solver
	 */
	int GetNumEffectors() const;

	/**
	 * Get name of the bone associated with the given effector.
	 * @return - a pointer to array of char holding the name of the bone for the effector. Returns empty string if index is out of range.
	 */
	const char* GetEffectorName(const unsigned int EffectorIndex);

	/**
	 * Tell the solver what the root bone is. All bones ABOVE the root are culled from the solver entirely.
	 * @param BoneIndex - all of the ground settings
	 * @return - SUCCESS or OUT_OF_BOUNDS if BoneIndex is invalid
	 */
	PowerIKErrorCode SetRootBone(const int BoneIndex) const;

	/**
	 * Get name of the currently set root bone.
	 * @return - a pointer to array of char holding the name of the root bone. Returns empty string if no root has been set.
	 */
	const char* GetRootBoneName() const;

	/**
	 * Optional method to modify internal solver settings like iteration counts.
	 * @param Settings - all the solver settings
	 */
	void SetSolverSettings(const SolverSettings& Settings) const;

	/**
	 * Optional method to setup Center of Gravity constraint on root bone.
	 * @param Settings - all of the center of gravity settings
	 */
	void SetCOGSettings(const CenterOfGravitySettings& Settings) const;

	/**
	 * Optional method to add inertial smoothing to root motion.
	 * @param Settings - all of the root inertia settings
	 */
	void SetInertiaSettings(const InertiaSettings& Settings) const;

	/**
	 * Call this every tick, for each bone, before Solve() to update the input pose.
	 * @param BoneIndex - the index of the bone. Index must be >= 0 and less than NumBones.
	 * @param Position - the Character Space 3d position of the bone
	 * @param Rotation - the Character Space rotation of the bone as a quaternion
	 * @warning This function performs no bounds checking for efficiency
	 */
	void SetBoneTransform(
		const int BoneIndex,
		const Vec3& Position,
		const Quat& Rotation) const;

	/**
	 * Call this every tick, for each bone, AFTER Solve() to get the generated output pose.
	 * @param BoneIndex - the index of the bone. Index must be >= 0 and less than NumBones.
	 * @param OutPosition - the Character Space 3d position of the bone
	 * @param OutRotation - the Character Space rotation of the bone as a quaternion
	 * @warning This function performs no bounds checking for efficiency
	 */
	void GetBoneTransform(
		const int BoneIndex,
		Vec3& OutPosition,
		Quat& OutRotation) const;

	/**
	 * Optionally call this every tick, for each effector, before Solve() to update the effector transform settings.
	 * @param EffectorIndex - the index of the effector (as returned from AddEffector)
	 * @param Position - the Character Space 3d position of the effector
	 * @param Rotation - the Character Space rotation of the effector as a quaternion
	 * @param RotateBone - if True, affected Bone will rotate to match effector rotation
	 * @param RotateLimb - if True, Bones near this effector will be rotated with the effector
	 * @return - SUCCESS or OUT_OF_BOUNDS if EffectorIndex is invalid
	 */
	PowerIKErrorCode SetEffectorTransform(
		const int EffectorIndex,
		const Vec3& Position,
		const Quat& Rotation,
		const bool RotateBone,
		const bool RotateLimb) const;

	/**
	 * Optionally call this every tick, for each effector, before Solve() to update the effector weight settings.
	 * @param EffectorIndex - the index of the effector (as returned from AddEffector)
	 * @param Alpha - used to blend the effect of the effector on/off. Ranges from 0 to 1.
	 * @param PullWeight - determines the degree of effect this effector has on the rest of the body. Normalized relative to all other PullWeight values.
	 * @param NormalizePulling - if True, total PullWeight on body is normalized to 1 to avoid over/under shooting.
	 * @param PositivePullFactor - multiplies effect of this effector on the rest of the body in the positive directions
	 * @param NegativePullFactor - multiplies effect of this effector on the rest of the body in the negative directions
	 * @return - SUCCESS or OUT_OF_BOUNDS if EffectorIndex is invalid
	 */
	PowerIKErrorCode SetEffectorWeights(
		const int EffectorIndex,
		const float Alpha,
		const float PullWeight,
		const bool NormalizePulling,
		const Vec3& PositivePullFactor,
		const Vec3& NegativePullFactor) const;

	/**
	 * Optionally call this every tick, for each effector, before Solve() to update the effector weight settings.
	 * @param EffectorIndex - the index of the effector (as returned from AddEffector)
	 * @param DeltaSmoothSpeed - speed in centimeters per second that this effector adjusts to changing positions.
* 	 * @param AngularDeltaSmoothSpeed - speed in degrees per second that this effector adjusts to changing rotations.
	 * @param AffectsCenterOfGravity - multiplies effect of this effector on the rest of the body in the negative directions
	 * @return - SUCCESS or OUT_OF_BOUNDS if EffectorIndex is invalid
	 */
	PowerIKErrorCode SetEffectorSettings(
		const int EffectorIndex,
		const float DeltaSmoothSpeed,
		const float AngularDeltaSmoothSpeed,
		const bool AffectsCenterOfGravity) const;

	/**
	 * Optionally call this to smooth an effector's position and/or rotation over time
	 * @param EffectorIndex - the index of the effector (as returned from AddEffector)
	 * @param Settings - the position and rotation smoothing settings
	 * @return - SUCCESS or OUT_OF_BOUNDS if EffectorIndex is invalid
	 */
	PowerIKErrorCode SetEffectorSmoothing(
		const int EffectorIndex,
		const EffectorSmoothingSettings& Settings) const;

	/**
	 * Optionally call this to specify an Effector's pole vector settings
	 * @param EffectorIndex - the index of the effector (as returned from AddEffector)
	 * @param PoleVector - the settings defining the Pole Vector behavior
	 * @return - SUCCESS or OUT_OF_BOUNDS if EffectorIndex is invalid
	 */
	PowerIKErrorCode SetEffectorPoleVector(
		const int EffectorIndex,
		const EffectorPoleVector& PoleVector) const;

	/**
	 * Optionally call this to specify a custom bending direction for a single bone
	 * @param BoneIndex - the index of the bone (as passed to AddBone)
	 * @param Direction - a 3d vector in the local space of the bone that is the direction the bone will move in when bending
	 * @return - SUCCESS or OUT_OF_BOUNDS if BoneIndex is invalid
	 */
	PowerIKErrorCode SetBoneBendDirection(
		const int BoneIndex,
		const Vec3& Direction) const;

	/**
	 * EXPERIMENTAL! Optionally call this to specify a set of limits and stiffness for a bone
	 * @param BoneIndex - the index of the bone (as passed to AddBone)
	 * @param Stiffness - ranges from 0 to 1. Default is 0. At 1 the bone is completely rigid and will be removed from the solve entirely.
	 * @param UseLimits - turn limits on/off.
	 * @param MinAngles - minimum allowable X, Y and Z rotations relative to parent
	 * @param MaxAngles - maximum allowable X, Y and Z rotations relative to parent
	 * @param NegateXAxis - consider limits in opposite direction along X-axis
	 * @return - SUCCESS or OUT_OF_BOUNDS if BoneIndex is invalid
	 */
	PowerIKErrorCode SetBoneLimits(
		const int BoneIndex,
		const float Stiffness,
		const bool UseLimits,
		const Vec3& MinAngles,
		const Vec3& MaxAngles,
		const bool NegateXAxis) const;

	/**
	 * Optionally call this to get the limits for a bone. If the bone has no limits
	 * @param BoneIndex - the index of the bone
	 * @param OutMinLimits - minimum allowable X, Y and Z rotations relative to parent
	 * @param OutMaxLimits - maximum allowable X, Y and Z rotations relative to parent
	 * @param OutNegateXAxis - consider limits in opposite direction along X-axis
	 * @param HasLimits - True if the bone has limits, false otherwise (all Out values will be untouched)
	 * @return - SUCCESS or OUT_OF_BOUNDS if BoneIndex is invalid
	 */
	PowerIKErrorCode GetBoneLimits(
		const int BoneIndex,
		Vec3& OutMinLimits,
		Vec3& OutMaxLimits,
		bool& OutNegateXAxis,
		bool& HasLimits) const;

	/**
	 * Get the root of a given limb. This is the top-most solved bone before a branch in the hierarchy.
	 * @param BoneIndex - a bone anywhere in the limb.
	 * @param OutBoneIndex - the top-most bone in the same limb.
	 */
	PowerIKErrorCode GetLimbRoot(
		const int BoneIndex,
		int &OutBoneIndex) const;

	/**
	 * Exclude the given bone. It will be treated as 100% stiff and the solver will ignore it.
	 * @warning This will only take effect if called before the first call to Solve()
	 * @param BoneIndex - the index of the bone (as passed to AddBone)
	 * @return - SUCCESS or OUT_OF_BOUNDS if BoneIndex is invalid
	 */
	PowerIKErrorCode ExcludeBone(const int BoneIndex) const;

	/**
	 * Convenience method for providing global offsets to the skeleton
	 * to be applied internally before the main solver pass.
	 * @param PositionOffset - the amount to translate the entire skeleton
	 */
	void TranslateSkeletonBeforeSolve(const Vec3& PositionOffset) const;

	/**
	 * Convenience method for providing global offsets to the skeleton
	 * to be applied internally before the main solver pass.
	 * @param RotationOffset - the amount to rotate the entire skeleton
	 */
	void RotateSkeletonBeforeSolve(const Quat& RotationOffset) const;

	/**
	 * Convenience method for providing local rotate offsets to specific bones
	 * to be applied internally before the main solver pass.
	 * @param RotationOffset - character space rotation offset applied relative to input pose
	 * @param BoneIndex - the index of the bone (as passed to AddBone)
	 * @return - SUCCESS or OUT_OF_BOUNDS if BoneIndex is invalid
	 */
	PowerIKErrorCode RotateBoneBeforeSolve(
		const Quat& RotationOffset,
		const int BoneIndex) const;

private:

	Core* PIK;
};

} // namespace
