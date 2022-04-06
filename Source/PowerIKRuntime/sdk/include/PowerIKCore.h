/*
Copyright Power Animated Inc., All Rights Reserved.
Unauthorized copying, selling or distribution of this software is strictly prohibited.
*/
#pragma once

#include "PowerIKMath.h"
#include "PowerIKSettings.h"

#include <vector>
#include <string>


namespace PowerIK
{

struct Effector;
struct SubChain;

struct Bone
{
	std::string Name = "";
	Bone* Parent = nullptr;
	int ParentIndex = -1;
	Effector* AttachedEffector = nullptr;
	SubChain* ChildSubChain = nullptr; // null on all except root bone of subchain
	std::vector<Bone*> Children;
	std::vector<Bone*> AllChildren;

	Vec3 Position = Vec3::Zero();
	Quat Rotation = Quat::Identity();

	Vec3 PositionFromPose = Vec3::Zero();
	Quat RotationFromPose = Quat::Identity();
	Vec3 LocalPositionFromPose = Vec3::Zero();
	Quat LocalRotationFromPose = Quat::Identity();

	float Stiffness = 0.0f;
	bool UseLimits = false;
	Vec3 MinLimit = Vec3(-20.0f, -20.0f, -20.0f);
	Vec3 MaxLimit = Vec3(20.0f, 20.0f, 20.0f);
	bool NegateXAxis = false;

	bool UseBendDirection = false;
	Vec3 BendDirection = Vec3(0.0f, 1.0f, 0.0f);

	float DistToParent = 0.0f;
	bool IsSolved = false;
	bool IsExcluded = false;
	bool IsChildOfSolvedFork = false;
	bool IsSolvedFork = false;
	bool IsSolverRoot = false;

	Quat PreSolveRotationOffset = Quat::Identity();
	bool ApplyPreSolveRotationOffset = false;
};

struct Effector
{
    Bone *Bone = nullptr;
	std::string BoneName = "";

	Vec3 Position = Vec3::Zero();
	Quat Rotation = Quat::Identity();

	float Alpha = 1.0f;
	float PullWeight = 1.0f;
	bool NormalizePulling = true;
	Vec3 PositivePullFactor = Vec3::One();
	Vec3 NegativePullFactor = Vec3::One();
	bool RotateLimb = false;
	bool RotateBone = false;

	bool AffectsCenterOfGravity = false;

	float DeltaSmoothSpeed = 100.0f;
	float AngularDeltaSmoothSpeed = 0.0f;
	EffectorSmoothingSettings SmoothSettings;

	EffectorPoleVector PoleVector;
	Vec3 PoleTargetRelativeToStartOrig = Vec3::One();

	Vec3 PositionDelta;
	Quat RotationDelta;
	Vec3 PrevPosition = Vec3::Zero();
	Quat PrevRotation = Quat::Identity();
	Vec3 PrevPositionDelta = Vec3::Zero();
	Quat PrevRotationDelta = Quat::Identity();
};

struct BendConstraint
{
	Bone* A;
	Bone* B;
	Bone* C;
	bool AIsLocked;
	bool BIsLocked;
	bool CIsLocked;
	float CentroidToBOrig;	// orig dist from ABC center to b
	float ACOrig;			// orig dist from a to c
	float K;				// spring constant

	void Initialize()
	{
		// called after A, B, and C pointers are set
		K = 1.0f;
		AIsLocked = true ? A->AttachedEffector || A->IsSolvedFork || A->IsSolverRoot || A->IsChildOfSolvedFork : false;
		BIsLocked = true ? B->AttachedEffector || B->IsSolvedFork || B->IsSolverRoot || B->IsChildOfSolvedFork : false;
		CIsLocked = true ? C->AttachedEffector || C->IsSolvedFork || C->IsSolverRoot || C->IsChildOfSolvedFork : false;
	}

	void UpdateInputState()
	{
		Vec3 Center = (A->Position + B->Position + C->Position) * 0.3333333333333f;
		CentroidToBOrig = (B->Position - Center).Length();
		ACOrig = (C->Position - A->Position).Length();
	}

	Vec3 CentroidToB()
	{
		Vec3 Centroid = (A->Position + B->Position + C->Position) * 0.3333333333333f;
		return B->Position - Centroid;
	}
};

struct SubChain
{
	// all bones in this chain, sorted from tip to root (including root and tip)
	std::vector<Bone*> Bones;

	// depth in the sub-chain hierarchy (for debug use)
	unsigned int Depth = 0;		

	// bending constraints in this sub-chain
	std::vector<BendConstraint> BendingConstraints;

	// squash/stretch data
	// this data is used to squash sub-chain bones
	Bone* SquashStartBone = nullptr;
	std::vector<Vec3> BonesRelativeToEnd;
	std::vector<Vec3> BonesRelativeToStart;
	Vec3 ChainAxisOrigStart;
	Vec3 ChainAxisOrigEnd;
	std::vector<float> BoneDistancesToChain;
	std::vector<float> BoneParams;
	float LineLengthOrig = 0;

	// FABRIK data
	// these positions are averaged at the end of the FABRIK solve
	std::vector<Vec3> FABRIKPositions;

	// pointers to neighboring subchains
	// these are used to rotate sub-roots and for COG constraint
	std::vector<SubChain*> NeighborChains;

	// smooth weight data
	// these are used to weight the effectors to the body
	std::vector<float> EffectorPullWeights;
	std::vector<float> EffectorDistWeights;
	std::vector<float> EffectorDistances;

	// total length of bones from root to tip of chain
	float ChainLength = 0;

	// inertia memory
	Vec3 Velocity;
	Vec3 Position;
};

struct Core
{
	void Solve(const float DeltaTime, const float Alpha);
	
	void Reset();

	void SetNumBones(int NumBones);

	int GetNumBones() const;

	PowerIKErrorCode AddBone(
		const int Index,
		const char* BoneName,
		const int& ParentIndex);

	PowerIKErrorCode AddEffector(
		const char* BoneName, 
		int& OutEffectorIndex);

	int GetNumEffectors() const;

	const char* GetEffectorName(const unsigned int EffectorIndex);

	PowerIKErrorCode SetRootBone(const int BoneIndex);

	const char* GetRootBoneName();

	void SetSolverSettings(const SolverSettings& Settings);

	void SetCOGSettings(const CenterOfGravitySettings& Settings);

	void SetInertiaSettings(const InertiaSettings& Settings);
	
	void SetBoneTransform(
		const int BoneIndex,
		const Vec3& Position,
		const Quat& Rotation);

	void GetBoneTransform(
		const int BoneIndex,
		Vec3& OutPosition,
		Quat& OutRotation);

	PowerIKErrorCode SetEffectorTransform(
		const int EffectorIndex,
		const Vec3 Position,
		const Quat Rotation,
		const bool RotateBone,
		const bool RotateLimb);

	PowerIKErrorCode SetEffectorWeights(
		const int EffectorIndex,
		const float Alpha,
		const float PullWeight,
		const bool NormalizePulling,
		const Vec3& PositivePullMultiplier,
		const Vec3& NegativePullMultiplier);

	PowerIKErrorCode SetEffectorSettings(
		const int EffectorIndex,
		const float DeltaSmoothSpeed,
		const float AngularDeltaSmoothSpeed,
		const bool AffectsCenterOfGravity);

	PowerIKErrorCode SetEffectorPoleVector(
		const int EffectorIndex,
		const EffectorPoleVector& PoleVector);

	PowerIKErrorCode SetEffectorSmoothing(
		const int EffectorIndex,
		const EffectorSmoothingSettings& Settings);

	PowerIKErrorCode SetBoneBendDirection(
		const int BoneIndex,
		const Vec3 Direction);

	PowerIKErrorCode SetBoneLimits(
		const int BoneIndex,
		const float Stiffness,
		const bool UseLimits,
		const Vec3 MinAngles,
		const Vec3 MaxAngles,
		const bool NegateXAxis);

	PowerIKErrorCode GetBoneLimits(
		const int Index,
		Vec3& OutMinLimits,
		Vec3& OutMaxLimits,
		bool& OutNegateXAxis,
		bool& OutHasLimits);

	PowerIKErrorCode GetLimbRoot(const int BoneIndex, int& OutBoneIndex);

	PowerIKErrorCode ExcludeBone(const int BoneIndex);

	void TranslateSkeletonBeforeSolve(const Vec3 &PositionOffset);
	
	void RotateSkeletonBeforeSolve(const Quat &RotationOffset);
	
	PowerIKErrorCode RotateBoneBeforeSolve(
		const Quat &RotationOffset, 
		const int BoneIndex);
	
private:

	bool GetBoneExists(const char* BoneName);
	void CacheInputState();
	void CacheStartPose();
	void NormalizeEffectorWeights();

	void Initialize();
	void InitBones();
	void InitEffectors();
	void InitSubChains();
	void InitSmoothWeights();
	void InitBendingConstraints();
	void InitPoleVectors();

	void ApplyPreSolveOffsets();
	void SmoothEffectorsWithAlpha();
	void SmoothlyMoveSubRoots();
	void ConstrainCOG();
	void UpdateInertia();
	void SquashSubChains();
	void RotateSubChainRoots();
	//void ApplyStiffness();
	
	void SolveSubRootPullConstraints();
    void SolveSubChainDistanceConstraints();
	void SolveSubChainBendingConstraints(const int& Iteration);
	void SolveFABRIKForward();
	void SolveFABRIKBackward();
	void MoveBoneDuringBackwardPass(Bone* B, Vec3 TargetPosition, Effector* TipEftr);
	void MoveBoneDuringForwardPass(Bone* B, Vec3 TargetPosition);
	void MoveSubChainTipBackPass(SubChain* Chain);
	void MoveSubChainRootFwdPass(SubChain* Chain);
	void UpdateSolvedForkRotations();
	void UpdateSolvedBoneRotations();
	void UpdatePoleVectorRotations();
	void UpdateNonSolvedSkeletonBelowBone(Bone& Bone);

	Vec3 ScaleDeltaByEffectorPullWeights(
		const Vec3& Delta,
		const float& EffectorWeight,
		const Effector& Eftr);

	std::vector<Bone> Bones;
	std::vector<Effector> Effectors;
	std::vector<SubChain> SubChains;
	SolverSettings MainSettings;
	CenterOfGravitySettings COGSettings;
	InertiaSettings Inertia;
	
	bool ReNormalizeEffectorWeights = true;

	Bone* Root;
	bool ApplySkeletonOffsets = false;
	Vec3 PreSolveSkeletonPositionOffset = Vec3::Zero();
	Quat PreSolveSkeletonRotationOffset = Quat::Identity();

	float DeltaTime = 0.0f;
	float SolverAlpha = 1.0f;

public:

	bool IsInitialized = false;
	static bool AreBonesConnected(Bone* A, Bone* B);
	static void SetChildrenToLocalPositionsFromPose(Bone* Fork);
	static float DistanceBetweenBones(Bone* A, Bone* B);
};

} // namespace PowerIK
