/*
Copyright Power Animated Inc., All Rights Reserved.
Unauthorized copying, selling or distribution of this software is strictly prohibited.
*/

//#pragma optimize( "", off )

#include "PowerIKGround.h"
#include "PowerIKMath.h"

#include <algorithm>


namespace  PowerIK
{

void GroundAlign::CalculateGroundAlignment(
	const Transform& Root,
	Transform& OutRootOffset,
	GroundFoot* OutFeet,
	unsigned int NumFeet,
	Quat& OutCounterLeanRotation,
	float DeltaTime)
{
	// must have at least 1 foot
	if (NumFeet <= 0)
	{
		return;
	}

	// clamp feet normals
	for (unsigned int i = 0; i < NumFeet; ++i)
	{
		ClampAngleBetweenNormals(Settings.WorldUpNormal, Settings.MaxGroundAngle, OutFeet[i].Normal);
	}
	
	// average all the foot normals to arrive at the ground normal
	Vec3 AverageFootNormal = Vec3::Zero();
	for (unsigned int i = 0; i < NumFeet; ++i)
	{
		AverageFootNormal += OutFeet[i].Normal;
	}
	float InvNumGroundedEffectors = 1.0f / (float)NumFeet;
	AverageFootNormal *= InvNumGroundedEffectors;
	AverageFootNormal.NormalizeSafe();

	// smooth the ROTATION DELTA of the normal over time
	Quat RotationDelta = Quat::FromToRotation(Settings.WorldUpNormal, AverageFootNormal);
	if (Settings.MaxNormalAngularSpeed > 0.001f)
	{
		RotationDelta = RotateTowards(
			PrevRotationDelta,
			RotationDelta,
			Settings.MaxNormalAngularSpeed,
			DeltaTime);
	}
	PrevRotationDelta = RotationDelta;
	Vec3 Normal = RotationDelta.Rotate(Settings.WorldUpNormal).Normalized();

	// decompose ground normal into pitch and yaw
	Vec3 Fwd = Settings.StrideDirection;
	Vec3 Up = Settings.WorldUpNormal;
	Vec3 Side;
	Vec3 VertNormal;
	Vec3 SideNormal;
	Quat PitchRotation;
	Quat RollRotation;
	Vec3 PitchedFwdVec;
	Vec3 RolledSideVec;

	PitchYawFromNormal(
		Fwd,
		Up,
		Normal,
		Side,
		VertNormal,
		SideNormal,
		PitchRotation,
		RollRotation,
		PitchedFwdVec,
		RolledSideVec);

	// get clamped inv max ground angle
	float ClampedMaxAngle = std::max(1.0f, Settings.MaxGroundAngle);
	ClampedMaxAngle = std::min(89.0f, Settings.MaxGroundAngle);
	float InvMaxAngle = 1.0f / ClampedMaxAngle;

	// calculate percentage of forward/back angle
	float FwdBackAngle = Vec3::AngleBetweenNormals(VertNormal, Up);
	FwdBackAngle = std::min(89.0f, FwdBackAngle);
	float FwdBackPercent = FwdBackAngle * InvMaxAngle;

	// calculate percentage of side hill
	float SideAngle = Vec3::AngleBetweenNormals(SideNormal, Up);
	SideAngle = std::min(89.0f, SideAngle);
	float SideHillPercent = SideAngle * InvMaxAngle;

	// leaning forward or backward?
	bool GoingUphill = Vec3::Dot(Normal, Fwd) <= 0.0f;
	// tilted left or right?
	float TiltedLeft = Vec3::Dot(Normal, Side) <= 0.0f;

	// orig hips position
	Vec3 OrigHipsPosition = Root.Position;

	OutRootOffset.Position = Vec3::Zero();
	OutRootOffset.Rotation = Quat::Identity();

	if (Settings.OrientToGround)
	{
		// move and rotate the entire skeleton with separate control over pitch and roll
		Quat PitchAmount = Quat::Lerp(Quat::Identity(), PitchRotation, Settings.OrientToPitch);
		Quat RollAmount = Quat::Lerp(Quat::Identity(), RollRotation, Settings.OrientToRoll);
		Quat OrientOffset = PitchAmount * RollAmount;

		OutRootOffset.Position += OrientOffset.Rotate(Root.Position) - Root.Position;
		OutRootOffset.Rotation = OutRootOffset.Rotation * OrientOffset;
		OutRootOffset.Rotation.Normalize();
	}

	if (Settings.Lean)
	{
		// factor-in separate forward/back leaning amounts
		float FwdBackLeanFactor = (FwdBackPercent * GoingUphill) ? Settings.UphillLean : Settings.DownhillLean;
		Quat LeanFwdBackRot = Quat::Lerp(Quat::Identity(), PitchRotation, FwdBackLeanFactor);

		// factor-in separate side-to-side leaning amounts
		float SideLeanFactor = SideHillPercent * Settings.SidehillLean;
		Quat LeanSideRot = Quat::Lerp(Quat::Identity(), RollRotation, SideLeanFactor);

		// apply leaning rotation to bone
		OutRootOffset.Rotation = OutRootOffset.Rotation * LeanFwdBackRot.Inverse() * LeanSideRot.Inverse();
		OutRootOffset.Rotation.Normalize();
	}
	
	if (Settings.CounterLean)
	{
		// factor-in separate forward/back leaning amounts
		float FwdBackCounterLeanFactor = (FwdBackPercent * GoingUphill) ? Settings.UphillCounterLean : Settings.DownhillCounterLean;
		Quat CounterPitchRotation = Quat::Lerp(Quat::Identity(), PitchRotation, FwdBackCounterLeanFactor);

		// factor-in separate side-to-side leaning amounts
		float SideCounterLeanFactor = SideHillPercent * Settings.SidehillCounterLean;
		Quat CounterRollRotation = Quat::Lerp(Quat::Identity(), RollRotation, SideCounterLeanFactor);

		// store relative rotate=ion to lean in opposite direction
		OutCounterLeanRotation = CounterPitchRotation * CounterRollRotation;
		OutCounterLeanRotation.Normalize();
	}

	if (Settings.MoveRoot)
	{
		// amount to move vertically based on uphill
		float VerticalOffsetFwdBack = GoingUphill ? Settings.UphillVertOffset : Settings.DownhillVertOffset;
		VerticalOffsetFwdBack *= FwdBackPercent;
		// amount to move vertically based on side-hill
		float VerticalOffsetSide = Settings.SidehillVertOffset * SideHillPercent;
		// use the greatest vertical offset
		float VerticalOffsetAmount = VerticalOffsetFwdBack + VerticalOffsetSide;
		// calculate final vertical offset
		Vec3 VerticalOffset = Up * VerticalOffsetAmount;

		// amount to move horiz based on Up/Downhill
		float HorizOffsetFwdBackAmount = GoingUphill ? Settings.UphillHorizOffset : Settings.DownhillHorizOffset;
		HorizOffsetFwdBackAmount *= FwdBackPercent;
		// Up/Downhill horizontal offset is in direction of stride ROTATED by FwdBackRot
		Vec3 HorizOffsetFwdBack = PitchedFwdVec * HorizOffsetFwdBackAmount;

		// amount to move horiz based on Sidehill
		float HorizOffsetSideAmount = TiltedLeft ? Settings.SidehillHorizOffset : -Settings.SidehillHorizOffset;
		HorizOffsetSideAmount *= SideHillPercent;
		// Sidehill horizontal offset is in direction of side vector ROTATED by SideRot
		Vec3 HorizOffsetSidehill = RolledSideVec * HorizOffsetSideAmount;

		// apply offsets to bone position
		OutRootOffset.Position += VerticalOffset + HorizOffsetFwdBack + HorizOffsetSidehill;
	}

	if (Settings.ScaleStride)
	{
		// calculate amount to scale stride fwd/back
		float StrideScaleFwdBack = GoingUphill ? Settings.UphillStrideScale : Settings.DownhillStrideScale;
		StrideScaleFwdBack = ScalarLerp(1.0f, StrideScaleFwdBack, FwdBackPercent);
		// calculate amount to scale stride from sidehill
		float StrideScaleSidehill = ScalarLerp(1.0f, Settings.SidehillStrideScale, SideHillPercent);
		// use whichever stride scale is smallest
		float TotalStrideScale = StrideScaleSidehill < StrideScaleFwdBack ? StrideScaleSidehill : StrideScaleFwdBack;

		// scale all grounded effectors along rotated stride direction
		for (unsigned int f = 0; f < NumFeet; ++f)
		{
			GroundFoot *Foot = &OutFeet[f];

			// scale stride
			Vec3 ProjOnStride = Vec3::Project(Foot->Transform.Position, PitchedFwdVec);
			Vec3 StrideProjToOrig = Foot->Transform.Position - ProjOnStride;
			Vec3 StrideScaledPos = (ProjOnStride * TotalStrideScale) + StrideProjToOrig;

			// push feet outwards on sidehill
			Vec3 HipsToFoot = (Foot->Transform.Position - OrigHipsPosition).NormalizeSafe();
			float NegateSideVec = Vec3::Dot(HipsToFoot, Side) >= 0.0f ? 1.0f : -1.0f;
			Vec3 SignedSideVec = NegateSideVec * RolledSideVec;
			//float PushFeetAmount = Vec3::Dot(SignedSideVec, Normal) >= 0.0f ? Settings.SidehillPushOuterFeet : Settings.SidehillPushInnerFeet;
			float PushFeetAmount = Settings.SidehillPushOuterFeet;
			PushFeetAmount *= SideHillPercent;
			Vec3 PushSidewaysOffset = SignedSideVec * PushFeetAmount;

			// move the foot
			Foot->Transform.Position = StrideScaledPos + PushSidewaysOffset;
		}
	}

	if (Settings.OffsetFeetPositions)
	{
		// nudge feet positions a static amount in the direction of the normal
		for (unsigned int f = 0; f < NumFeet; ++f)
		{
			GroundFoot* Foot = &OutFeet[f];
			Foot->Transform.Position += Foot->Normal * Settings.StaticFootOffset;
		}
	}

	if (Settings.RotateFootToGround)
	{
		for (unsigned int f = 0; f < NumFeet; ++f)
		{
			GroundFoot *Foot = &OutFeet[f];

			// decompose foot normal into pitch and yaw
			Vec3 FootSide;
			Vec3 FootVertNormal;
			Vec3 FootSideNormal;
			Quat FootPitchRotation;
			Quat FootRollRotation;
			Vec3 FootPitchedFwdVec;
			Vec3 FootRolledSideVec;

			PitchYawFromNormal(
				Fwd,
				Up,
				Foot->Normal,
				FootSide,
				FootVertNormal,
				FootSideNormal,
				FootPitchRotation,
				FootRollRotation,
				FootPitchedFwdVec,
				FootRolledSideVec);

			Quat FootRoll = Quat::Lerp(Quat::Identity(), FootRollRotation, Settings.RollFootAmount);
			Quat FootPitch = Quat::Lerp(Quat::Identity(), FootPitchRotation, Settings.PitchFootAmount);
			Foot->Transform.Rotation = FootRoll * FootPitch * Foot->Transform.Rotation;
			Foot->Transform.Rotation.Normalize();
		}
	}
}

}