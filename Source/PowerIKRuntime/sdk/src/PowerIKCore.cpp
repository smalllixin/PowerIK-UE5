/*
Copyright Power Animated Inc., All Rights Reserved.
Unauthorized copying, selling or distribution of this software is strictly prohibited.
*/
#include "PowerIKCore.h"

#include <algorithm>


namespace PowerIK
{
	using PowerIK::Core;
	using PowerIK::Bone;
	using PowerIK::Effector;
	using PowerIK::Quat;
	using PowerIK::Vec3;


void Core::Reset()
{
	Bones.clear();
	Effectors.clear();
	SubChains.clear();
	IsInitialized = false;
	Root = nullptr;
}

void Core::SetNumBones(int NumBones)
{
	Bones.resize(NumBones);
}

int Core::GetNumBones() const
{
	return (int)Bones.size();
}

PowerIKErrorCode Core::AddBone(
	const int BoneIndex,
	const char* BoneName,
	const int& ParentIndex)
{
	if (BoneIndex < 0 || 
        BoneIndex >= (int)Bones.size() ||
		ParentIndex < 0 ||
        ParentIndex >= (int)Bones.size() ||
		BoneIndex == ParentIndex)
	{
		return PowerIKErrorCode::OUT_OF_BOUNDS;
	}
		
	Bone& B = Bones[BoneIndex];
	B.Name = BoneName;
	B.ParentIndex = ParentIndex;
	IsInitialized = false;

	return PowerIKErrorCode::SUCCESS;
}

PowerIKErrorCode Core::AddEffector(
	const char* BoneName, 
	int& OutEffectorIndex)
{
	if (!GetBoneExists(BoneName))
	{
		OutEffectorIndex = -1;
		return PowerIKErrorCode::FAILURE;
	}

	Effector E;
	E.BoneName = BoneName;
	Effectors.push_back(E);
	OutEffectorIndex = (int)Effectors.size() - 1;

	IsInitialized = false;
	ReNormalizeEffectorWeights = true;

	return PowerIKErrorCode::SUCCESS;
}

int Core::GetNumEffectors() const
{
	return (int)Effectors.size();
}

const char* Core::GetEffectorName(const unsigned int EffectorIndex)
{
	if (EffectorIndex < 0 || EffectorIndex >= (int)Effectors.size())
	{
		return "";
	}

	return Effectors[EffectorIndex].BoneName.c_str();
}

PowerIKErrorCode Core::SetRootBone(const int BoneIndex)
{
    if (BoneIndex < 0 || BoneIndex >= (int)Bones.size())
	{
		Root = nullptr;
		return PowerIKErrorCode::OUT_OF_BOUNDS;
	}

	for (unsigned int i = 0; i < Bones.size(); ++i)
	{
		Bones[i].IsSolverRoot = BoneIndex == (int)i;
		if (Bones[i].IsSolverRoot)
		{
			Root = &Bones[i];
		}
	}

	return PowerIKErrorCode::SUCCESS;
}

const char* Core::GetRootBoneName()
{
	for (unsigned int i = 0; i < Bones.size(); ++i)
	{
		if (Bones[i].IsSolverRoot)
		{
			return Bones[i].Name.c_str();
		}
	}

	return "";
}

void Core::SetSolverSettings(const SolverSettings& InSettings)
{
	MainSettings = InSettings;

	// clamp squash iterations
	MainSettings.SquashIterations = std::max((unsigned int)0, MainSettings.SquashIterations);
	MainSettings.SquashIterations = std::min((unsigned int)1000, MainSettings.SquashIterations);

	// clamp stretch iterations
	MainSettings.StretchIterations = std::max((unsigned int)0, MainSettings.StretchIterations);
	MainSettings.StretchIterations = std::min((unsigned int)1000, MainSettings.StretchIterations);
	
	// clamp final iterations
	MainSettings.FinalIterations = std::max((unsigned int)0, MainSettings.FinalIterations);
	MainSettings.FinalIterations = std::min((unsigned int)1000, MainSettings.FinalIterations);

	// make sure world up is normalized
	MainSettings.WorldUpNormal = InSettings.WorldUpNormal.Normalized();
}

void Core::SetCOGSettings(const CenterOfGravitySettings& Settings)
{
	COGSettings = Settings;
}

void Core::SetInertiaSettings(const InertiaSettings& Settings)
{
	Inertia = Settings;
}

void Core::SetBoneTransform(
	const int BoneIndex,
	const Vec3& Position,
	const Quat& Rotation)
{
	Bone& B = Bones[BoneIndex];
	B.Position = Position;
	B.Rotation = Rotation;
}

void Core::GetBoneTransform(
	const int BoneIndex,
	Vec3& OutPosition,
	Quat& OutRotation)
{
	Bone& B = Bones[BoneIndex];
	OutPosition = B.Position;
	OutRotation = B.Rotation;
}

PowerIKErrorCode Core::SetEffectorTransform(
	const int EffectorIndex,
	const Vec3 Position,
	const Quat Rotation,
	const bool RotateBone,
	const bool RotateLimb)
{
    if (EffectorIndex < 0 || EffectorIndex >= (int)Effectors.size())
	{
		return PowerIKErrorCode::OUT_OF_BOUNDS;
	}

	Effector& Eftr = Effectors[EffectorIndex];
	Eftr.Position = Position;
	Eftr.Rotation = Rotation;
	Eftr.RotateBone = RotateBone;
	Eftr.RotateLimb = RotateLimb;

	return PowerIKErrorCode::SUCCESS;
}

PowerIKErrorCode Core::SetEffectorWeights(
	const int EffectorIndex,
	const float Alpha,
	const float PullWeight,
	const bool NormalizePulling,
	const Vec3& PositivePullMultiplier,
	const Vec3& NegativePullMultiplier)
{
    if (EffectorIndex < 0 || EffectorIndex >= (int)Effectors.size())
	{
		return PowerIKErrorCode::OUT_OF_BOUNDS;
	}

	Effector& Eftr = Effectors[EffectorIndex];

	// tell solver to re-normalize weights when PullWeight updated
	if (Eftr.PullWeight != PullWeight)
	{
		ReNormalizeEffectorWeights = true;
	}

	Eftr.Alpha = Alpha;
	Eftr.PullWeight = PullWeight;
	Eftr.NormalizePulling = NormalizePulling;
	Eftr.PositivePullFactor = PositivePullMultiplier;
	Eftr.NegativePullFactor = NegativePullMultiplier;

	return PowerIKErrorCode::SUCCESS;
}

PowerIKErrorCode Core::SetEffectorSettings(
	const int EffectorIndex,
	const float DeltaSmoothSpeed,
	const float AngularDeltaSmoothSpeed,
	const bool AffectsCenterOfGravity)
{
    if (EffectorIndex < 0 || EffectorIndex >= (int)Effectors.size())
	{
		return PowerIKErrorCode::OUT_OF_BOUNDS;
	}

	Effector& Eftr = Effectors[EffectorIndex];
	Eftr.DeltaSmoothSpeed = DeltaSmoothSpeed;
	Eftr.AngularDeltaSmoothSpeed = AngularDeltaSmoothSpeed;
	Eftr.AffectsCenterOfGravity = AffectsCenterOfGravity;

	return PowerIKErrorCode::SUCCESS;
}

PowerIKErrorCode Core::SetEffectorPoleVector(
	const int EffectorIndex, 
	const EffectorPoleVector& PoleVector)
{
	if (EffectorIndex < 0 || EffectorIndex >= (int)Effectors.size())
	{
		return PowerIKErrorCode::OUT_OF_BOUNDS;
	}

	Effector& Eftr = Effectors[EffectorIndex];
	Eftr.PoleVector = PoleVector;

	return PowerIKErrorCode::SUCCESS;
}

PowerIKErrorCode Core::SetEffectorSmoothing(
	const int EffectorIndex,
	const EffectorSmoothingSettings& Settings)
{
    if (EffectorIndex < 0 || EffectorIndex >= (int)Effectors.size())
	{
		return PowerIKErrorCode::OUT_OF_BOUNDS;
	}

	Effector& Eftr = Effectors[EffectorIndex];
	Eftr.SmoothSettings = Settings;

	return PowerIKErrorCode::SUCCESS;
}

PowerIKErrorCode Core::SetBoneBendDirection(
	const int BoneIndex,
	const Vec3 Direction)
{
    if (BoneIndex < 0 || BoneIndex >= (int)Bones.size())
	{
		return PowerIKErrorCode::OUT_OF_BOUNDS;
	}

	Bone& B = Bones[BoneIndex];
	B.UseBendDirection = true;
	B.BendDirection = Direction.NormalizedSafe();

	return PowerIKErrorCode::SUCCESS;
}

PowerIKErrorCode Core::SetBoneLimits(
	const int BoneIndex,
	const float Stiffness,
	const bool UseLimits,
	const Vec3 MinAngles,
	const Vec3 MaxAngles,
	const bool NegateXAxis)
{
    if (BoneIndex < 0 || BoneIndex >= (int)Bones.size())
	{
		return PowerIKErrorCode::OUT_OF_BOUNDS;
	}

	Bone& Bone = Bones[BoneIndex];
	Bone.Stiffness = std::max(std::min(1.0f, Stiffness), 0.0f);
	Bone.UseLimits = UseLimits;
	Bone.MinLimit = MinAngles;
	Bone.MaxLimit = MaxAngles;
	Bone.NegateXAxis = NegateXAxis;

	return PowerIKErrorCode::SUCCESS;
}

PowerIKErrorCode Core::GetBoneLimits(
	const int BoneIndex,
	Vec3& OutMinLimits,
	Vec3& OutMaxLimits,
	bool& OutNegateXAxis,
	bool& OutHasLimits)
{
    if (BoneIndex < 0 || BoneIndex >= (int)Bones.size())
	{
		return PowerIKErrorCode::OUT_OF_BOUNDS;
	}

	Bone& B = Bones[BoneIndex];
	OutMinLimits = B.MinLimit;
	OutMaxLimits = B.MaxLimit;
    OutNegateXAxis = B.NegateXAxis;
	OutHasLimits = B.UseLimits;

	return PowerIKErrorCode::SUCCESS;
}

PowerIKErrorCode Core::GetLimbRoot(const int BoneIndex, int& OutBoneIndex)
{
	if (BoneIndex < 0 || BoneIndex >= (int)Bones.size())
	{
		return PowerIKErrorCode::OUT_OF_BOUNDS;
	}

	if (!IsInitialized)
	{
		return PowerIKErrorCode::FAILURE;
	}

	Bone *B = &Bones[BoneIndex];
	while (true)
	{
		if (!B)
		{
			return PowerIKErrorCode::FAILURE;
		}

		if (B->IsChildOfSolvedFork || B->IsSolverRoot)
		{
			break;
		}

		B = B->Parent;
	}

	for (unsigned int i = 0; i < Bones.size(); ++i)
	{
		if (&Bones[i] == B)
		{
			OutBoneIndex = i;
			return PowerIKErrorCode::SUCCESS;
		}
	}

	return PowerIKErrorCode::FAILURE;
}

PowerIKErrorCode Core::ExcludeBone(const int BoneIndex)
{
    if (BoneIndex < 0 || BoneIndex >= (int)Bones.size())
	{
		return PowerIKErrorCode::OUT_OF_BOUNDS;
	}

	Bone& B = Bones[BoneIndex];
	B.IsExcluded = true;

	return PowerIKErrorCode::SUCCESS;
}

void Core::TranslateSkeletonBeforeSolve(
	const Vec3 &PositionOffset)
{
	PreSolveSkeletonPositionOffset = PositionOffset;
	ApplySkeletonOffsets = true;
}

void Core::RotateSkeletonBeforeSolve(
	const Quat &RotationOffset)
{
	PreSolveSkeletonRotationOffset = RotationOffset;
	ApplySkeletonOffsets = true;
}

PowerIKErrorCode Core::RotateBoneBeforeSolve(
	const Quat &RotationOffset, 
	const int BoneIndex)
{
	if (BoneIndex < 0 || BoneIndex >= (int)Bones.size())
	{
		return PowerIKErrorCode::OUT_OF_BOUNDS;
	}

	Bone& B = Bones[BoneIndex];
	B.PreSolveRotationOffset = RotationOffset;
	B.ApplyPreSolveRotationOffset = true;

	return PowerIKErrorCode::SUCCESS;
}

} // namespace PowerIK
