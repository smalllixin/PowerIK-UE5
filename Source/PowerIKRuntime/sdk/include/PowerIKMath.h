/*
Copyright Power Animated Inc., All Rights Reserved.
Unauthorized copying, selling or distribution of this software is strictly prohibited.
*/
#pragma once

#include "PowerIK.h"

#define NEAR_ZERO (1.e-4f)
#define RAD_TO_DEG (57.2957801818848f)
#define DEG_TO_RAD (0.01745329252f)


namespace PowerIK
{

struct Vec3
{
	float X;
	float Y;
	float Z;

	/**default construction, zero*/
	Vec3();
	/** construct from other Vec3 */
    Vec3(const Vec3& V);
	/** construct from 3-floats */
    Vec3(float X1, float Y1, float Z1);
	
	/** index into it */
    float operator[](int Index)const;
	/** addition */
    Vec3 operator+(const Vec3& V)const;
	/** addition in-place */
    Vec3& operator+=(const Vec3& V);
	/** subtraction */
    Vec3 operator-(const Vec3& V)const;
	/** subtraction in-place */
    Vec3& operator-=(const Vec3& V);
	/** multiplication */
    Vec3 operator*(float Scalar)const;
	/** multiplication in-place */
    Vec3& operator*=(float Scalar);
	/** division */
    Vec3 operator/(float Scalar)const;
	/** division in-place */
    Vec3& operator/=(float Scalar);
	/** assignment */
    Vec3& operator=(const Vec3& V);
	
	/** set component Scalars */
    void Set(float X1, float Y1, float Z1);
	/** dot product w/ other vector */
    float Dot(const Vec3& V)const;
	/** cross product w/ other vector */
    Vec3 Cross(const Vec3& V)const;
	/** magnitude of the vector */
    float Length()const;
	/** squared magnitude */
    float LengthSq()const;
	/** distance to other vector */
    float Distance(const Vec3& V)const;
	/** get normalized vector (fast/unsafe, no Zero length check!) */
    Vec3 Normalized()const;
	/** get normalized vector, checks for zero length */
    Vec3 NormalizedSafe(float Tolerance = NEAR_ZERO)const;
	/** normalize in-place, checks for zero length */
    Vec3& NormalizeSafe(float Tolerance = NEAR_ZERO);
	/** get a normalized direction vector and length */
	void GetDirectionAndLength(Vec3& OutDirection, float& OutLength, float Tolerance = NEAR_ZERO);
	
	/** lerp between two vectors */
	static Vec3 Lerp(const Vec3& A, const Vec3& B, const float t);
	/** dot product of two vectors */
	static float Dot(const Vec3& A, const Vec3& B);
	/** cross product of two vectors */
	static Vec3 Cross(const Vec3& A, const Vec3& B);
	/** create new zero vector */
	static Vec3 Zero() { return Vec3(0.0f, 0.0f, 0.0f); }
	/** create new one vector */
	static Vec3 One() { return Vec3(1.0f, 1.0f, 1.0f); }
	/** project A onto a normal vector */
	static Vec3 Project(const Vec3& A, const Vec3& OnNormal);
	/** project A onto a plane */
	static Vec3 ProjectPointOnPlane(const Vec3& A, const Vec3& PlaneOrigin, const Vec3& PlaneNormal);
	/** get angle in degrees between A and B (assumed to be normalized) */
	static float AngleBetweenNormals(const Vec3& A, const Vec3& B);
};

Vec3 operator*(const float Scalar, const Vec3& A);


struct Quat
{
	float X;
	float Y;
	float Z;
	float W;

	/** default construction, uninitialized for speed */
    Quat();
	/** construct from other Quat */
    Quat(const Quat& Q);
	/** construct from 4-floats */
    Quat(float InX, float InY, float InZ, float InW);
	
	/** add quaternions component-wise */
    Quat operator+(const Quat& Q) const;
	/** multiply by scalar */
    Quat operator*(const float Multiplier) const;
	/** quaternion multiplication (represents a composed rotation of this, followed by Q) */
    Quat operator*(const Quat& Q) const;
	
	/** inverse, in-place */
    void Invert();
	/** inverse, return copy */
    Quat Inverse();
	/** normalize in-place */
    Quat& Normalize(float Tolerance = NEAR_ZERO);
	/** rotate a vector by this */
    Vec3 Rotate(const Vec3& V) const;
	
	/** set this quaternion based on an axis and angle (in degrees)*/
	static Quat FromAxisAngleDegrees(const Vec3& Axis, const float AngleDegrees);
	/** set this quaternion based on an axis and angle (in radians)*/
	static Quat FromAxisAngleRadians(const Vec3& Axis, const float AngleRadians);
	/** lerp between two quaternions */
	static Quat Lerp(const Quat& A, const Quat& B, const float Alpha);
	/** create quaternion that rotates from A to B */
	static Quat FromToRotation(const Vec3& A, const Vec3& B);
	/** dot product of two vectors */
	static float Dot(const Quat& A, const Quat& B);
	/** are two quaternions close to each other? */
	static bool AreClose(const Quat& A, const Quat& B);
	/** create new zero vector */
	static Quat Identity() { return Quat(0.0f, 0.0f, 0.0f, 1.0f); }
	/** angle in degrees between two quaternions */
	static float AngleDegrees(const Quat& A, const Quat& B);
};

struct Transform
{
	Vec3 Position;
	Quat Rotation;

	Transform()
	{
		Position = Vec3::Zero();
		Rotation = Quat::Identity();
	}
};

/**
* Linearly interpolate between two float values
* 
* @param: A - start value
* @param: B - end value
* @param: T - parameter (0 at A and 1 at B)
* @return: float - result interpolated from A to B
*/
float ScalarLerp(const float A, const float B, const float T);


/**
* Move a position towards a target position with linear velocity
* (will not overshoot target)
* 
* @param: Current - current position
* @param: Target - position to move towards
* @param: MaxSpeed - speed in cm/second to move towards
* @param: MaxDistance - clamp maximum distance between current and target
* @param: DeltaTime - seconds since last tick
* @return: PowerIK::Vec3 - the new position moved towards target
*/
Vec3 MoveTowards(
	const Vec3& Current,
	const Vec3& Target,
	const float MaxSpeed,
	const float MaxDistance,
	const float DeltaTime);

/**
* Move a position towards a target position with non-linear velocity decreasing towards target
* (will not overshoot target)
*
* @param: Current - current position
* @param: Target - position to move towards
* @param: MaxSpeed - speed in cm/second to move towards
* @param: MaxDistance - clamp maximum distance between current and target
* @param: DeltaTime - seconds since last tick
* @return: PowerIK::Vec3 - the new position moved towards target
*/
Vec3 MoveTowardsSmooth(
	const Vec3& Current,
	const Vec3& Target,
	const float SlowFactor);
	
/**
* Rotate a quaternion towards a target quaternion with linear angular velocity
* (will not overshoot target)
* 
* @param: Current - current rotation
* @param: Target - rotation to rotate towards
* @param: MaxAngularSpeed - speed in degrees/second to rotate towards target
* @param: DeltaTime - seconds since last tick
* @return: PowerIK::Quat - the new rotation, closer to target
*/
Quat RotateTowards(
	const Quat& Current,
	const Quat& Target,
	const float MaxAngularSpeed,
	const float DeltaTime);
	
/**
* Decompose rotation between two normals into separate pitch/yaw, relative to a forward vector
* 
* @param: Fwd - normalized vector pointing in forward direction
* @param: Up - normalized vector pointing up 
* @param: Normal - normalized vector pointing away from up (the orientation normal)
* @param: OutSide - outputs vector perpendicular to normal and forward (sideways vector)
* @param: OutVertNormal - rotated up vector projected into plane of pitch
* @param: OutSideNormal - rotated side vector projected into plane of roll
* @param: OutPitchRotation - quaternion with only pitch
* @param: OutRollRotation - quaternion with only roll
* @param: OutPitchedFwdVec - the forward vector rotated by pitch
* @param: OutRolledSideVec - the side vector rotated by roll
*/
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
	Vec3& OutRolledSideVec);

/**
* Clamp the input vector to be within MaxAngle of "RelativeTo" vector
*
* @param: RelativeTo - normalized vector to measure angle relative to
* @param: OutVector -normalized  vector to clamp within MaxAngle of RelativeTo
* @param: MaxAngle - maximum angle, in degrees, that OutVector can deviate from RelativeTo
*/
void ClampAngleBetweenNormals(
	const Vec3& RelativeTo,
	const float MaxAngle,
	Vec3& OutVector);
	
} // namespace

