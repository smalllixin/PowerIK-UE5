/*
Copyright Power Animated Inc., All Rights Reserved.
Unauthorized copying, selling or distribution of this software is strictly prohibited.
*/
#include "PowerIKMath.h"

#include <math.h>
#include <algorithm>


namespace  PowerIK
{

//
// Vec3 implementation
//

Vec3::Vec3() : X(0), Y(0), Z(0)
{
}

Vec3::Vec3(const Vec3& V)
{
	X = V.X;
	Y = V.Y;
	Z = V.Z;
}

Vec3::Vec3(float X1, float Y1, float Z1)
{
	X = X1;
	Y = Y1;
	Z = Z1;
}

float Vec3::operator[](int Index) const
{
	Index = std::min<int>(std::max<int>(0, Index), 2);
	return (&X)[Index];
}

Vec3 Vec3::operator+(const Vec3& V) const
{
	return Vec3(X + V.X, Y + V.Y, Z + V.Z);
}

Vec3& Vec3::operator+=(const Vec3& V)
{
	X += V.X;
	Y += V.Y;
	Z += V.Z;
	return *this;
}

Vec3 Vec3::operator-(const Vec3& V) const
{
	return Vec3(X - V.X, Y - V.Y, Z - V.Z);
}

Vec3& Vec3::operator-=(const Vec3& V)
{
	X -= V.X;
	Y -= V.Y;
	Z -= V.Z;
	return *this;
}

Vec3 Vec3::operator*(float Scalar) const
{
	return Vec3(X * Scalar, Y * Scalar, Z * Scalar);
}

Vec3& Vec3::operator*=(float Scalar)
{
	X *= Scalar;
	Y *= Scalar;
	Z *= Scalar;
	return *this;
}

Vec3 Vec3::operator/(float Scalar) const
{
	return Vec3(X / Scalar, Y / Scalar, Z / Scalar);
}

Vec3& Vec3::operator/=(float Scalar)
{
	float Multiplier = 1.0f / Scalar;
	X *= Multiplier;
	Y *= Multiplier;
	Z *= Multiplier;
	return *this;
}

Vec3& Vec3::operator=(const Vec3& V)
{
	X = V.X;
	Y = V.Y;
	Z = V.Z;
	return *this;
}

void Vec3::Set(float X1, float Y1, float Z1)
{
	X = X1;
	Y = Y1;
	Z = Z1;
}

float Vec3::Dot(const Vec3& V) const
{
	return X * V.X + Y * V.Y + Z * V.Z;
}

Vec3 Vec3::Cross(const Vec3& V) const
{
	float X1 = Y * V.Z - Z * V.Y;
	float Y1 = Z * V.X - X * V.Z;
	float Z1 = X * V.Y - Y * V.X;
	return Vec3(X1, Y1, Z1);
}

Vec3 Vec3::Project(const Vec3& A, const Vec3& OnNormal)
{
	float LenSq = Dot(OnNormal, OnNormal);
	if (LenSq <= NEAR_ZERO)
	{
		return Vec3::Zero();
	}

	float ADotNorm = Dot(A, OnNormal);
	float Scale = ADotNorm / LenSq;
	return OnNormal * Scale;
}

Vec3 Vec3::ProjectPointOnPlane(
	const Vec3& A, 
	const Vec3& PlaneOrigin, 
	const Vec3& PlaneNormal)
{
	Vec3 OriginToA = A - PlaneOrigin;
	Vec3 AOnNormal = Project(OriginToA, PlaneNormal);
	return PlaneOrigin + (A - (PlaneOrigin + AOnNormal));
}

float Vec3::AngleBetweenNormals(const Vec3& A, const Vec3& B)
{
	return acosf(Vec3::Dot(A, B)) * RAD_TO_DEG;
}

float Vec3::Length() const
{
	return sqrtf(X * X + Y * Y + Z * Z);
}

float Vec3::LengthSq() const
{
	return X * X + Y * Y + Z * Z;
}

float Vec3::Distance(const Vec3& V) const
{
	float X1 = X - V.X;
	float Y1 = Y - V.Y;
	float Z1 = Z - V.Z;
	return sqrtf(X1 * X1 + Y1 * Y1 + Z1 * Z1);
}

Vec3 Vec3::Normalized() const
{
	float InvLength = 1.0f / Length();
	return Vec3(X * InvLength, Y * InvLength, Z * InvLength);
}

Vec3 Vec3::NormalizedSafe(float Tolerance) const
{
	const float LengthSq = X * X + Y * Y + Z * Z;

	if (LengthSq == 1.f)
	{
		return Vec3(X, Y, Z); // already normalized
	}
		
	if (LengthSq < Tolerance)
	{
		return Vec3(1.0f, 0.0f, 0.0f);	// avoid divide by zero
	}

	const float Multiplier = 1.0f / sqrtf(LengthSq); // TODO optimize with fast inv sqrt
	return Vec3(X * Multiplier, Y * Multiplier, Z * Multiplier);
}

Vec3& Vec3::NormalizeSafe(float Tolerance)
{
	const float LengthSq = X * X + Y * Y + Z * Z;

	if (LengthSq == 1.f)
	{
		return *this; // already normalized
	}

	if (LengthSq < Tolerance)
	{
		return *this; // avoid divide by zero
	}

	const float Multiplier = 1.0f / sqrtf(LengthSq); // TODO optimize with fast inv sqrt
	X *= Multiplier;
	Y *= Multiplier;
	Z *= Multiplier;

	return *this;
}

void Vec3::GetDirectionAndLength(Vec3& OutDirection, float& OutLength, float Tolerance)
{
	OutLength = Length();
	if (OutLength > Tolerance)
	{
		float InvLength = 1.0f / OutLength;
		OutDirection = Vec3(X * InvLength, Y * InvLength, Z * InvLength);
	}
	else
	{
		OutDirection = Vec3::Zero();
	}
}

Vec3 operator*(const float Scalar, const Vec3& A)
{
	return Vec3(A.X * Scalar, A.Y * Scalar, A.Z * Scalar);
}


Vec3 Vec3::Lerp(const Vec3& A, const Vec3& B, const float T)
{
	const float X = A.X + T * (B.X - A.X);
	const float Y = A.Y + T * (B.Y - A.Y);
	const float Z = A.Z + T * (B.Z - A.Z);
	return Vec3(X, Y, Z);
}

float Vec3::Dot(const Vec3& A, const Vec3& B)
{
	return A.X * B.X + A.Y * B.Y + A.Z * B.Z;
}

Vec3 Vec3::Cross(const Vec3& A, const Vec3& B)
{
	const float X1 = A.Y * B.Z - A.Z * B.Y;
	const float Y1 = A.Z * B.X - A.X * B.Z;
	const float Z1 = A.X * B.Y - A.Y * B.X;
	return Vec3(X1, Y1, Z1);
}

//
// Quat implementation
//

Quat::Quat() : X(0), Y(0), Z(0), W(1)
{
}

Quat::Quat(const Quat& Q)
{
	X = Q.X;
	Y = Q.Y;
	Z = Q.Z;
	W = Q.W;
}

Quat::Quat(float InX, float InY, float InZ, float InW)
{
	X = InX;
	Y = InY;
	Z = InZ;
	W = InW;
}

Quat Quat::operator+(const Quat& Q) const
{
	return Quat(X + Q.X, Y + Q.Y, Z + Q.Z, W + Q.W);
}

void Quat::Invert()
{
	X = -X;
	Y = -Y;
	Z = -Z;
}

Quat Quat::Inverse()
{
	return Quat(-X, -Y, -Z, W);
}

Quat Quat::operator*(const Quat& Q) const
{
	Quat Result;
	Result.X = (W * Q.X) + (X * Q.W) + (Y * Q.Z) - (Z * Q.Y);
	Result.Y = (W * Q.Y) - (X * Q.Z) + (Y * Q.W) + (Z * Q.X);
	Result.Z = (W * Q.Z) + (X * Q.Y) - (Y * Q.X) + (Z * Q.W);
	Result.W = (W * Q.W) - (X * Q.X) - (Y * Q.Y) - (Z * Q.Z);
	return Result;
}

Quat Quat::operator*(const float Multiplier) const
{
	return Quat(Multiplier * X, Multiplier * Y, Multiplier * Z, Multiplier * W);
}

Quat& Quat::Normalize(float Tolerance)
{
	const float LengthSq = X * X + Y * Y + Z * Z + W * W;

	if (LengthSq >= Tolerance)
	{
		const float Multiplier = 1.0f / sqrtf(LengthSq); // TODO optimize with fast inv sqrt
		X *= Multiplier;
		Y *= Multiplier;
		Z *= Multiplier;
		W *= Multiplier;
	}
	else
	{
		// set to identity
		X = 0;
		Y = 0;
		Z = 0;
		W = 1.0f;
	}

	return *this;
}

Vec3 Quat::Rotate(const Vec3& V) const
{
	// http://people.csail.mit.edu/bkph/articles/Quaternions.pdf
	// V' = V + 2w(Q x V) + (2Q x (Q x V))
	// refactor:
	// V' = V + w(2(Q x V)) + (Q x (2(Q x V)))
	// T = 2(Q x V);
	// V' = V + w*(T) + (Q x T)

	const Vec3 Q(X, Y, Z);
	const Vec3 T = 2.f * Vec3::Cross(Q, V);
	const Vec3 Result = V + (W * T) + Vec3::Cross(Q, T);
	return Result;
	
	/*
	Quat Vec(V.X, V.Y, V.Z, 0.0f);
	Quat A = *this * Vec;
	Quat B = A * Quat(-X, -Y, -Z, W);
	return Vec3(B.X, B.Y, B.Z);*/
}

Quat Quat::FromAxisAngleDegrees(const Vec3& Axis, const float AngleDegrees)
{
	return FromAxisAngleRadians(Axis, AngleDegrees * DEG_TO_RAD);
}

Quat Quat::FromAxisAngleRadians(const Vec3& Axis, const float AngleRadians)
{
	float HalfAngle = 0.5f * AngleRadians;
	float SinHalfAngle = sinf(HalfAngle);
	Quat Q;
	Q.W = cosf(HalfAngle);
	Q.X = (SinHalfAngle * Axis.X);
	Q.Y = (SinHalfAngle * Axis.Y);
	Q.Z = (SinHalfAngle * Axis.Z);
	return Q;
}

Quat Quat::Lerp(const Quat& A, const Quat& B, const float Alpha)
{
	// a "good enough" approximation of SLERP but much faster.
	//
	// output is NOT normalized for efficiency, 
	// but if A and B are normalized then output will be too

	// take dot product to see if Quat are pointing in same or opposite direction
	// use dot to flip quaternion to take shortest path
	const float Bias = Quat::Dot(A, B) >= 0.0f ? 1.0f : -1.0f;
	return (B * Alpha) + (A * (Bias * (1.f - Alpha)));
}

Quat Quat::FromToRotation(const Vec3& A, const Vec3& B)
{
	// Based on:
	// http://lolengine.net/blog/2014/02/24/quaternion-from-two-vectors-final
	// http://www.euclideanspace.com/maths/algebra/vectors/angleBetween/index.htm

	Quat Result;
	const float NormAB = sqrtf(A.LengthSq() * B.LengthSq());
	float W = NormAB + Vec3::Dot(A, B);
	if (W >= 1e-6f * NormAB)
	{
		//Axis = CrossProduct(A, B);
		Result.X = A.Y * B.Z - A.Z * B.Y;
		Result.Y = A.Z * B.X - A.X * B.Z;
		Result.Z = A.X * B.Y - A.Y * B.X;
		Result.W = W;
	}
	else
	{
		// A and B point in opposite directions
		W = 0.f;
		if (fabs(A.X) > fabs(A.Y))
		{
			Result = Quat(-A.Z, 0.f, A.X, W);
		}
		else
		{
			Result = Quat(0.f, -A.Z, A.Y, W);
		}
	}

	Result.Normalize();
	return Result;
}

float Quat::Dot(const Quat& A, const Quat& B)
{
	return A.X * B.X + A.Y * B.Y + A.Z * B.Z + A.W * B.W;
}

bool Quat::AreClose(const Quat& A, const Quat& B)
{
	return Quat::Dot(A, B) > 0.0f;
}

float Quat::AngleDegrees(const Quat& A, const Quat& B)
{
	float DotAB = Quat::Dot(A, B);
    float AbsClampedDot = std::min((float)fabs(DotAB), 1.0f);
	return (acosf(AbsClampedDot) * 2.0f) * RAD_TO_DEG;
}

float ScalarLerp(const float A, const float B, const float T)
{
	return A + T * (B - A);
}


void PitchYawFromNormal(
	const Vec3& Fwd,
	const Vec3& Up,
	const Vec3& Normal,
	Vec3& OutSide,
	Vec3& OutVertNormal,
	Vec3& OutSideNormal,
	Quat& OutPitchRotation,
	Quat& OutRollRotation,
	Vec3& OutPitchedFwdVec,
	Vec3& OutRolledSideVec)
{
	// decompose normal into forward/back
	OutSide = Vec3::Cross(Up, Fwd);
	OutVertNormal = Normal - Vec3::Project(Normal, OutSide);
	OutVertNormal.NormalizeSafe();
	OutPitchRotation = Quat::FromToRotation(Up, OutVertNormal);
	OutPitchedFwdVec = OutPitchRotation.Rotate(Fwd).Normalized();

	// decompose normal into side-to-side
	OutSideNormal = Normal - Vec3::Project(Normal, Fwd);
	OutSideNormal.NormalizeSafe();
	OutRollRotation = Quat::FromToRotation(Up, OutSideNormal);
	OutRolledSideVec = OutRollRotation.Rotate(OutSide).Normalized();
}

void ClampAngleBetweenNormals( 
	const Vec3& RelativeTo, 
	const float MaxAngle,
	Vec3& OutVector)
{
	if (Vec3::AngleBetweenNormals(RelativeTo, OutVector) <= MaxAngle)
		return;

	Vec3 Axis = Vec3::Cross(RelativeTo, OutVector).NormalizedSafe();
	Quat Rotation = Quat::FromAxisAngleDegrees(Axis, MaxAngle);
	OutVector = Rotation.Rotate(RelativeTo);
}

Vec3 MoveTowards(
	const Vec3& Current,
	const Vec3& Target,
	const float MaxSpeed,
	const float MaxDistance,
	const float DeltaTime)
{
	Vec3 ToTgt = Target - Current;
	float DistToTgt;
	ToTgt.GetDirectionAndLength(ToTgt, DistToTgt);

	// clamp current position to always be within 'max delta ever' of target
	if (DistToTgt > MaxDistance)
	{
		Vec3 Start = Target - (ToTgt * MaxDistance);
		ToTgt = Target - Start;
		ToTgt.GetDirectionAndLength(ToTgt, DistToTgt);
	}

	// if we are within "max delta per frame" of target, just move there
	const float MaxDelta = DeltaTime * MaxSpeed;
	if (DistToTgt <= MaxDelta || DistToTgt == 0.0f)
	{
		return Target;
	}

	// move towards target "max delta" amount
	return Current + ToTgt * MaxDelta;
}

PowerIK::Vec3 MoveTowardsSmooth(const Vec3& Current, const Vec3& Target, const float SlowFactor)
{
	//http://sol.gfxile.net/interpolation/index.html#s5
	//v = ( (v * (N - 1) ) + w) / N; 
	Vec3 Result;
	Result = ((Current * (SlowFactor - 1)) + Target) / SlowFactor;
	return Result;
}

Quat RotateTowards(
	const Quat& Current,
	const Quat& Target,
	const float MaxAngularSpeed,
	const float DeltaTime)
{
	// check if already at or very near Target
	float AngleBetween = Quat::AngleDegrees(Current, Target);
	if (AngleBetween <= 0.1f)
	{
		return Target;
	}

	float MaxDegrees = MaxAngularSpeed * DeltaTime;
	float T = std::min(1.0f, MaxDegrees / AngleBetween);
	Quat Result = Quat::Lerp(Current, Target, T);
	Result.Normalize();
	return Result;
}

} // PowerIK namespace
