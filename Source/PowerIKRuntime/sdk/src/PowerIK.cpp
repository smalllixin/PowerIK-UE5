/*
Copyright Power Animated Inc., All Rights Reserved.
Unauthorized copying, selling or distribution of this software is strictly prohibited.
*/
#include "PowerIK.h"
#include "PowerIKCore.h"


namespace PowerIK
{
    Solver::Solver()
    {
        PIK = new PowerIK::Core();
    }

	Solver::~Solver()
	{
		delete PIK;
	}

	void Solver::Solve(
		const float DeltaTime,
		const float Alpha) const
	{
		PIK->Solve(DeltaTime, Alpha);
	}

	void Solver::Reset() const
	{
		PIK->Reset();
	}

	bool Solver::IsInitialized() const
	{
		return PIK->IsInitialized;
	}

	void Solver::SetNumBones(int NumBones) const
	{
        PIK->SetNumBones(NumBones);
	}

	int Solver::GetNumBones() const
	{
        return PIK->GetNumBones();
	}

    PowerIKErrorCode Solver::AddBone(
        const int BoneIndex, 
        const char* BoneName, 
        const int ParentIndex) const
	{
		return PIK->AddBone(
			BoneIndex, 
			BoneName, 
			ParentIndex);
	}

	PowerIKErrorCode Solver::AddEffector(
		const char* BoneName,
		int& OutEffectorIndex) const
	{
		return PIK->AddEffector(
			BoneName, 
			OutEffectorIndex);
	}

	int Solver::GetNumEffectors() const
	{
		return PIK->GetNumEffectors();
	}

	const char* Solver::GetEffectorName(const unsigned int EffectorIndex)
	{
		return PIK->GetEffectorName(EffectorIndex);
	}

	PowerIKErrorCode Solver::SetRootBone(const int BoneIndex) const
	{
		return PIK->SetRootBone(BoneIndex);
	}
	
	const char* Solver::GetRootBoneName() const
	{
		return PIK->GetRootBoneName();
	}

	void Solver::SetSolverSettings(const SolverSettings& Settings) const
	{
		PIK->SetSolverSettings(Settings);
	}

	void Solver::SetCOGSettings(const CenterOfGravitySettings& Settings) const
	{
		PIK->SetCOGSettings(Settings);
	}

	void Solver::SetInertiaSettings(const InertiaSettings& Settings) const
	{
		PIK->SetInertiaSettings(Settings);
	}

	void Solver::SetBoneTransform(
		const int BoneIndex,
		const Vec3& Position,
		const Quat& Rotation) const
	{
		PIK->SetBoneTransform(
			BoneIndex,
			Position,
			Rotation);
	}

	void Solver::GetBoneTransform(
		const int BoneIndex,
		Vec3& OutPosition,
		Quat& OutRotation) const
	{
		PIK->GetBoneTransform(
			BoneIndex,
			OutPosition,
			OutRotation);
	}

	PowerIKErrorCode Solver::SetEffectorTransform(
		const int EffectorIndex,
		const Vec3& Position,
		const Quat& Rotation,
		const bool RotateBone,
		const bool RotateLimb) const
	{
		return PIK->SetEffectorTransform(
			EffectorIndex,
			Position,
			Rotation,
			RotateBone,
			RotateLimb);
	}

	PowerIKErrorCode Solver::SetEffectorWeights(
		const int EffectorIndex,
		const float Alpha,
		const float PullWeight,
		const bool NormalizePulling,
		const Vec3& PositivePullFactor,
		const Vec3& NegativePullFactor) const
	{
		return PIK->SetEffectorWeights(
			EffectorIndex,
			Alpha,
			PullWeight,
			NormalizePulling,
			PositivePullFactor,
			NegativePullFactor);
	}

	PowerIKErrorCode Solver::SetEffectorSettings(
		const int EffectorIndex,
		const float DeltaSmoothSpeed,
		const float AngularDeltaSmoothSpeed,
		const bool AffectsCenterOfGravity) const
	{
		return PIK->SetEffectorSettings(
			EffectorIndex,
			DeltaSmoothSpeed,
			AngularDeltaSmoothSpeed,
			AffectsCenterOfGravity);
	}

	PowerIKErrorCode Solver::SetEffectorSmoothing(
		const int EffectorIndex,
		const EffectorSmoothingSettings& Settings) const
	{
		return PIK->SetEffectorSmoothing(
			EffectorIndex,
			Settings);
	}

	PowerIKErrorCode Solver::SetEffectorPoleVector(
		const int EffectorIndex, 
		const EffectorPoleVector& PoleVector) const
	{
		return PIK->SetEffectorPoleVector(
			EffectorIndex, 
			PoleVector);
	}

	PowerIKErrorCode Solver::SetBoneBendDirection(
		const int BoneIndex,
		const Vec3& Direction) const
	{
		return PIK->SetBoneBendDirection(BoneIndex, Direction);
	}

	PowerIKErrorCode Solver::SetBoneLimits(
		const int BoneIndex,
		const float Stiffness,
		const bool UseLimits,
		const Vec3& MinAngles,
		const Vec3& MaxAngles,
		const bool NegateXAxis) const
	{
		return PIK->SetBoneLimits(
			BoneIndex,
			Stiffness,
			UseLimits,
			MinAngles,
			MaxAngles,
			NegateXAxis);
	}

	PowerIKErrorCode Solver::GetBoneLimits(
		const int BoneIndex,
		Vec3& OutMinLimits,
		Vec3& OutMaxLimits,
		bool& OutNegateXAxis,
		bool& HasLimits) const
	{
		return PIK->GetBoneLimits(
			BoneIndex,
			OutMinLimits,
			OutMaxLimits,
			OutNegateXAxis,
			HasLimits);
	}

	PowerIKErrorCode Solver::GetLimbRoot(const int BoneIndex, int& OutBoneIndex) const
	{
		return PIK->GetLimbRoot(BoneIndex, OutBoneIndex);
	}

	PowerIKErrorCode Solver::ExcludeBone(const int BoneIndex) const
	{
		return PIK->ExcludeBone(BoneIndex);
	}

	void Solver::TranslateSkeletonBeforeSolve(const Vec3& PositionOffset) const
	{
		PIK->TranslateSkeletonBeforeSolve(PositionOffset);
	}

	void Solver::RotateSkeletonBeforeSolve(const Quat& RotationOffset) const
	{
		PIK->RotateSkeletonBeforeSolve(RotationOffset);
	}

	PowerIKErrorCode Solver::RotateBoneBeforeSolve(
		const Quat& RotationOffset, 
		const int BoneIndex) const
	{
		return PIK->RotateBoneBeforeSolve(RotationOffset, BoneIndex);
	}

} // namespace
