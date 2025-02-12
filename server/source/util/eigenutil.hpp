/**
AsterTrack Optical Tracking System
Copyright (C)  2025 Seneral <contact@seneral.dev> and contributors

MIT License

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#ifndef EIGEN_UTIL_H
#define EIGEN_UTIL_H

#include "util/eigendef.hpp"

/**
 * Utility functions using Eigen
 */


/* Functions */

/**
 * Return x,y,z angles in XYZ convention
 */
template<typename Derived>
inline Eigen::Matrix<typename Derived::Scalar,3,1> getEulerXYZ(const Eigen::MatrixBase<Derived> &rotMat)
{
	static_assert(3 == Derived::RowsAtCompileTime);
	static_assert(3 == Derived::ColsAtCompileTime);
	return rotMat.canonicalEulerAngles(2,1,0).reverse();
}

/**
 * Return x,y,z angles in ZYX convention
 */
template<typename Derived>
inline Eigen::Matrix<typename Derived::Scalar,3,1> getEulerZYX(const Eigen::MatrixBase<Derived> &rotMat)
{
	static_assert(3 == Derived::RowsAtCompileTime);
	static_assert(3 == Derived::ColsAtCompileTime);
	return rotMat.canonicalEulerAngles(0,1,2); // Decompose in order x,y,z
}

/**
 * Get angle axis from x,y,z angles in XYZ convention
 */
template<typename Derived>
inline Eigen::Quaternion<typename Derived::Scalar> getQuaternionXYZ(const Eigen::MatrixBase<Derived> &eulerAngles)
{
	typedef typename Derived::Scalar Scalar;
	static_assert(3 == Derived::RowsAtCompileTime);
	static_assert(1 == Derived::ColsAtCompileTime);
	return Eigen::AngleAxis<Scalar>(eulerAngles.z(), Eigen::Matrix<Scalar,3,1>::UnitZ())
		* Eigen::AngleAxis<Scalar>(eulerAngles.y(), Eigen::Matrix<Scalar,3,1>::UnitY())
		* Eigen::AngleAxis<Scalar>(eulerAngles.x(), Eigen::Matrix<Scalar,3,1>::UnitX());
}

/**
 * Get angle axis from x,y,z angles in ZYX convention
 */
template<typename Derived>
inline Eigen::Quaternion<typename Derived::Scalar> getQuaternionZYX(const Eigen::MatrixBase<Derived> &eulerAngles)
{
	typedef typename Derived::Scalar Scalar;
	static_assert(3 == Derived::RowsAtCompileTime);
	static_assert(1 == Derived::ColsAtCompileTime);
	return Eigen::AngleAxis<Scalar>(eulerAngles.x(), Eigen::Matrix<Scalar,3,1>::UnitX())
		* Eigen::AngleAxis<Scalar>(eulerAngles.y(), Eigen::Matrix<Scalar,3,1>::UnitY())
		* Eigen::AngleAxis<Scalar>(eulerAngles.z(), Eigen::Matrix<Scalar,3,1>::UnitZ());
}

/**
 * Get rotation matrix from x,y,z angles in XYZ convention
 */
template<typename Derived>
inline Eigen::Matrix<typename Derived::Scalar,3,3> getRotationXYZ(const Eigen::MatrixBase<Derived> &eulerAngles)
{
	return Eigen::Matrix<typename Derived::Scalar,3,3>(getQuaternionXYZ(eulerAngles));
}

/**
 * Get rotation matrix from x,y,z angles in ZYX convention
 */
template<typename Derived>
inline Eigen::Matrix<typename Derived::Scalar,3,3> getRotationZYX(const Eigen::MatrixBase<Derived> &eulerAngles)
{
	return Eigen::Matrix<typename Derived::Scalar,3,3>(getQuaternionZYX(eulerAngles));
}

/**
 * Convert Modified Rodrigues Parameters to Quaternions
 */
template<typename Derived>
Eigen::Quaternion<typename Derived::Scalar> MRP2Quat(const Eigen::MatrixBase<Derived> &mrp)
{
	auto a = mrp.squaredNorm();
	auto factor = 1.0 / (16 + a);
	auto base = 16 - a;
	base *= factor;
	Eigen::Matrix<typename Derived::Scalar,3,1> vec = mrp * 8*factor;
	return Eigen::Quaternion<typename Derived::Scalar>(base, vec.x(), vec.y(), vec.z());
}

/**
 * Convert Quaternions to Modified Rodrigues Parameters
 */
template<typename Scalar>
Eigen::Matrix<Scalar,3,1> Quat2MRP(const Eigen::Quaternion<Scalar> quat)
{
	return quat.vec() * (4/(1+quat.w()));
}

/**
 * Decode rotation from an encoded vector of size 3
 */
template<typename Scalar, typename Derived>
Eigen::Matrix<Scalar, 3, 3> DecodeAARot(const Eigen::MatrixBase<Derived> &rotVec)
{
	Eigen::Matrix<Scalar,3,1> axis = rotVec.template cast<Scalar>();
	Scalar angle = axis.norm();
	Eigen::Transform<Scalar, 3, Eigen::Isometry> pose;
	return Eigen::AngleAxis<Scalar>(angle - (Scalar)1, axis / angle).toRotationMatrix();
}

/**
 * Encodes rotation to an encoded rotation vector of size 3
 */
template<typename Scalar>
Eigen::Matrix<Scalar,3,1> EncodeAARot(const Eigen::Matrix<Scalar,3,3> &rot)
{
	Eigen::AngleAxis<Scalar> aa(rot);
	return (aa.angle() + (Scalar)1) * aa.axis();
}

/**
 * Decode pose from an encoded pose vector of size 6
 */
template<typename Scalar, typename Derived>
Eigen::Transform<Scalar, 3, Eigen::Isometry> DecodeAAPose(const Eigen::MatrixBase<Derived> &poseVec)
{
	Eigen::Transform<Scalar, 3, Eigen::Isometry> pose;
	pose.linear() = DecodeAARot<Scalar>(poseVec.template tail<3>());
	pose.translation() = poseVec.template head<3>().template cast<Scalar>();
	return pose;
}

/**
 * Encode pose to an encoded pose vector of size 6
 */
template<typename Scalar>
Eigen::Matrix<Scalar,6,1> EncodeAAPose(const Eigen::Transform<Scalar, 3, Eigen::Isometry> &pose)
{
	Eigen::Matrix<Scalar,6,1> poseVec;
	poseVec.template tail<3>() = EncodeAARot<Scalar>(pose.rotation());
	poseVec.template head<3>() = pose.translation();
	return poseVec;
}

/**
 * Create projection matrix using default clip planes and given field of view, so that projected points are clipped to ([-1,1], [-1,1])
 */
inline Eigen::Projective3f createProjectionMatrixGL(const CameraCalib &calib, const CameraMode &mode)
{
	float zN = 0.1f, zF = 100.0f; // 0.1m-100m probably
	float a = -(float)((zF+zN)/(zF-zN)), b = (float)(2*zF*zN/(zF-zN));
	float sX = (float)calib.f*mode.factorW, sY = (float)(calib.f*mode.factorH);
	// Projection - negative z because of blender-style camera coordinate system
	Eigen::Projective3f p;
	p.matrix() << 
		sX, 0, (float)calib.principalPoint.x()*mode.factorW, 0,
		0, sY, (float)calib.principalPoint.y()*mode.factorH, 0,
		0, 0, a, b,
		0, 0, 1, 0;
	return p;
}

/**
 * Create projection matrix given field of view
 */
template<typename Scalar = CVScalar>
inline Eigen::Transform<Scalar,3,Eigen::Projective> createProjectionMatrixCV(const CameraCalib &calib)
{
	Eigen::Transform<Scalar,3,Eigen::Projective> p;
	p.matrix() << 
		calib.f, 0, calib.principalPoint.x(), 0,
		0, calib.f, calib.principalPoint.y(), 0,
		0, 0, 1, 0,
		0, 0, 1, 0;
	return p;
}

/**
 * Turn field of view in degrees to camera fInv (1/f)
 */
template<typename Scalar>
inline Scalar fInvFromFoV(Scalar fov)
{
	return std::tan(fov / 2 / 180.0f * PI);
}

/**
 * Get the mathematical horizontal field of view in degrees of the camera
 */
inline double getFoVH(const CameraCalib &calib)
{
	return 2 * std::atan(calib.fInv) / PI * 180.0;
}

/**
 * Get the mathematical horizontal field of view in degrees of the camera
 */
inline double getFoVH(const CameraCalib &calib, const CameraMode &mode)
{
	return 2 * std::atan(calib.fInv*mode.sizeW) / PI * 180.0;
}

/**
 * Get the mathematical vertical field of view in degrees of the camera
 */
inline double getFoVV(const CameraCalib &calib, const CameraMode &mode)
{
	return 2 * std::atan(calib.fInv*mode.sizeH) / PI * 180.0;
}

/**
 * Get the mathematical diagonal field of view in degrees of the camera
 */
inline double getFoVD(const CameraCalib &calib, const CameraMode &mode)
{
	return 2 * std::atan(calib.fInv*std::sqrt(mode.sizeH*mode.sizeH+mode.sizeW*mode.sizeW)) / PI * 180.0;
}

/**
 * Get the effective horizontal field of view in degrees of the camera
 */
template<typename Scalar = double>
inline Scalar getEffectiveFoVH(const CameraCalib &calib)
{
	float effFInv = undistortPoint(calib, Eigen::Matrix<Scalar,2,1>(1,0)).norm()*(Scalar)calib.fInv;
	return 2 * std::atan(effFInv) / PI * 180.0;
}

/**
 * Get the effective horizontal field of view in degrees of the camera
 */
template<typename Scalar = double>
inline Scalar getEffectiveFoVH(const CameraCalib &calib, const CameraMode &mode)
{
	float effFInv = undistortPoint(calib, Eigen::Matrix<Scalar,2,1>(mode.sizeW,0)).norm()*(Scalar)calib.fInv;
	return 2 * std::atan(effFInv) / PI * 180.0;
}

/**
 * Get the effective vertical field of view in degrees of the camera
 */
template<typename Scalar = double>
inline Scalar getEffectiveFoVV(const CameraCalib &calib, const CameraMode &mode)
{
	float effFInv = undistortPoint(calib, Eigen::Matrix<Scalar,2,1>(0,mode.sizeH)).norm()*(Scalar)calib.fInv;
	return 2 * std::atan(effFInv) / PI * 180.0;
}

/**
 * Get the effective diagonal field of view in degrees of the camera
 */
template<typename Scalar = double>
inline Scalar getEffectiveFoVD(const CameraCalib &calib, const CameraMode &mode)
{
	float effFInv = undistortPoint(calib, Eigen::Matrix<Scalar,2,1>(mode.sizeW,mode.sizeH)).norm()*(Scalar)calib.fInv;
	return 2 * std::atan(effFInv) / PI * 180.0;
}

/**
 * Create model matrix using given transformations
 */
inline Eigen::Affine3f createModelMatrix(const Eigen::Vector3f &translation, const Eigen::Matrix3f &rotation, float scale)
{
	Eigen::Affine3f m;
	m.linear() = rotation*scale;
	m.translation() = translation;
	return m;
}

/**
 * Create model matrix using given transformations
 */
inline Eigen::Isometry3f createModelMatrix(const Eigen::Vector3f &translation, const Eigen::Matrix3f &rotation)
{
	Eigen::Isometry3f m;
	m.linear() = rotation;
	m.translation() = translation;
	return m;
}

/**
 * Update the view and camera matrix when transform or intrinsiv parameters of cameras changed
 */
template<typename Scalar>
inline void CameraCalib_t<Scalar>::UpdateDerived()
{
	view = transform.inverse(); // Very cheap inverse 
	projection = createProjectionMatrixCV<Scalar>(*this);
	camera = projection * view;
}

/**
 * Projects the 3D point and normalises it. Z includes depth (+/-)
 */
template<typename Derived, typename CalibScalar>
inline Eigen::Matrix<typename Derived::Scalar,3,1> projectPoint(const Eigen::Transform<CalibScalar,3,Eigen::Projective> &projection, const Eigen::MatrixBase<Derived> &point)
{
	Eigen::Matrix<typename Derived::Scalar,4,1> projected = projection.template cast<typename Derived::Scalar>() * point.homogeneous();
 	projected.template head<2>() /= projected.w();
	return projected.template head<3>();
	//return projected.hnormalized(); // Would also work, but looses depth information
}

/**
 * Projects the 3D point and discards depth information, including whether it is in front or behind the view
 */
template<typename Derived, typename CalibScalar>
inline Eigen::Matrix<typename Derived::Scalar,2,1> projectPoint2D(const Eigen::Transform<CalibScalar,3,Eigen::Projective> &projection, const Eigen::MatrixBase<Derived> &point)
{
	Eigen::Matrix<typename Derived::Scalar,4,1> projected = projection.template cast<typename Derived::Scalar>() * point.homogeneous();
	return projected.template head<2>() / projected.w(); // Discard z, assumes z to be positive
}

/**
 * Projects the 3D point and discards depth information, including whether it is in front or behind the view
 */
template<typename Derived, typename CalibScalar>
inline Eigen::Matrix<typename Derived::Scalar,2,1> applyProjection2D(const CameraCalib_t<CalibScalar> &calib, const Eigen::MatrixBase<Derived> &point)
{
	return point.template head<2>()*((typename Derived::Scalar)calib.f/point.z()) + calib.principalPoint.template cast<typename Derived::Scalar>();
}

/**
 * Reprojects the 2D point in image space into camera space as a hnormalized 2D point
 */
template<typename Derived, typename CalibScalar>
inline Eigen::Matrix<typename Derived::Scalar,2,1> applyReprojection2D(const CameraCalib_t<CalibScalar> &calib, const Eigen::MatrixBase<Derived> &point)
{
	return (point - calib.principalPoint.template cast<typename Derived::Scalar>()) * (typename Derived::Scalar)calib.fInv;
}

/**
 * Infer the pose of a marker given it's image points and the known camera position to transform into world space
 */
template<typename Scalar, typename PointScalar, typename CalibScalar = CVScalar>
static inline Ray3_t<Scalar> castRay(const Eigen::Matrix<PointScalar,2,1> &point2D, const CameraCalib_t<CalibScalar> &camera)
{
	Eigen::Matrix<Scalar,2,1> pt = applyReprojection2D(camera, point2D.template cast<Scalar>());
	Eigen::Matrix<Scalar,3,1> dir = (camera.transform.linear().template cast<Scalar>() * pt.homogeneous()).normalized();
	return { camera.transform.translation().template cast<Scalar>()+dir, dir };
}

template<typename Scalar>
static void getRayIntersect(const Ray3_t<Scalar> &ray1, const Ray3_t<Scalar> &ray2, Scalar *sec1, Scalar *sec2)
{
	// Pseudo ray intersection by computing the closest point of two lines
	// Due to usual camera placements, for most cases the following assumptions can be made:
	// Parallel case can be ignored, and rays can safely be considered as lines
	// Reference: http://geomalgorithms.com/a07-_distance.html

	Scalar a = ray1.dir.dot(ray1.dir);
	Scalar b = ray1.dir.dot(ray2.dir);
	Scalar c = ray2.dir.dot(ray2.dir);

	Scalar d = ray1.dir.dot(ray1.pos-ray2.pos);
	Scalar e = ray2.dir.dot(ray1.pos-ray2.pos);

	Scalar s = a*c - b*b;
	*sec1 = (b*e - c*d) / s;
	*sec2 = (a*e - b*d) / s;
}

template<typename Scalar, typename PointScalar>
static Scalar getRaySection(const Ray3_t<Scalar> &ray, const Eigen::Matrix<PointScalar,3,1> &point3D)
{
	return ray.dir.dot(point3D - ray.pos) / ray.dir.dot(ray.dir);
}

template<typename Scalar, typename CalibScalar = CVScalar>
inline Eigen::Matrix<Scalar,3,3> calculateFundamentalMatrix(const CameraCalib_t<CalibScalar> &camA, const CameraCalib_t<CalibScalar> &camB)
{
	// Copy so that rows are mapped as 1,2,0,1
	Eigen::Matrix<CalibScalar,4,4> calibA, calibB;
	calibA.template topRows<2>() = camA.camera.matrix().template middleRows<2>(1);
	calibA.template bottomRows<2>() = camA.camera.matrix().template topRows<2>();
	calibB.template topRows<2>() = camB.camera.matrix().template middleRows<2>(1);
	calibB.template bottomRows<2>() = camB.camera.matrix().template topRows<2>();

	Eigen::Matrix<Scalar,3,3> FM;
	Eigen::Matrix<CalibScalar,4,4> ab;
	for (int i = 0; i < 3; i++)
	{
		ab.template bottomRows<2>() = calibB.template middleRows<2>(i);
		for (int j = 0; j < 3; j++)
		{
			ab.template topRows<2>() = calibA.template middleRows<2>(j);
			FM(i,j) = ab.determinant();
		}
	}
	return FM;
}

/**
 * Calculates translational [mm] and rotational [°] difference between poseA and poseB
 */
template<typename Scalar>
inline std::pair<Scalar,Scalar> calculatePoseError(const Eigen::Transform<Scalar,3,Eigen::Isometry> &poseA, const Eigen::Transform<Scalar,3,Eigen::Isometry> &poseB)
{
	Eigen::Matrix<Scalar,3,1> tDiff = poseB.translation() - poseA.translation();
	Eigen::Matrix<Scalar,3,3> rDiff = poseA.rotation() * poseB.rotation().transpose();
	Scalar tError = tDiff.norm()*1000, rError = Eigen::AngleAxis<Scalar>(rDiff).angle()/PI*180;
	return { tError, rError };
}

/**
 * Convert the given point in pixel space to camera space
 */
template<typename Scalar>
inline Eigen::Matrix<Scalar,2,1> pix2cam(const CameraMode &camera, Eigen::Matrix<Scalar,2,1> point)
{
	return Eigen::Matrix<Scalar,2,1>(
		+(point.x() / (Scalar)camera.widthPx * 2 - 1) * (Scalar)camera.sizeW,
		-(point.y() / (Scalar)camera.heightPx * 2 - 1) * (Scalar)camera.sizeH);
	// Inverting y to convert from pixel coordinates (top left origin) to our coordinate system (bottom left origin)
	// The first is great for camera image processing, but for linear algebra the second is more commonly used
}
/**
 * Convert the given point in normalised pixel space (0-1 on both axis) to camera space
 */
template<typename Scalar>
inline Eigen::Matrix<Scalar,2,1> npix2cam(const CameraMode &camera, Eigen::Matrix<Scalar,2,1> point)
{
	return Eigen::Matrix<Scalar,2,1>(
		+(point.x() * 2 - 1) * (Scalar)camera.sizeW,
		-(point.y() * 2 - 1) * (Scalar)camera.sizeH);
	// Inverting y to convert from pixel coordinates (top left origin) to our coordinate system (bottom left origin)
	// The first is great for camera image processing, but for linear algebra the second is more commonly used
}
/**
 * Convert the given point in camera space to pixel space
 */
template<typename Scalar>
inline Eigen::Matrix<Scalar,2,1> cam2pix(const CameraMode &camera, Eigen::Matrix<Scalar,2,1> point)
{
	return Eigen::Matrix<Scalar,2,1>(
		(+point.x()/(Scalar)camera.sizeW + 1) / 2 * (Scalar)camera.widthPx, 
		(-point.y()/(Scalar)camera.sizeH + 1) / 2 * (Scalar)camera.heightPx);
	// Inverting y to convert from pixel coordinates (top left origin) to our coordinate system (bottom left origin)
	// The first is great for camera image processing, but for linear algebra the second is more commonly used
}
/**
 * Convert the given point in camera space to normalised pixel space (0-1 on both axis)
 */
template<typename Scalar>
inline Eigen::Matrix<Scalar,2,1> cam2npix(const CameraMode &camera, Eigen::Matrix<Scalar,2,1> point)
{
	return Eigen::Matrix<Scalar,2,1>(
		(+point.x()/(Scalar)camera.sizeW + 1) / 2, 
		(-point.y()/(Scalar)camera.sizeH + 1) / 2);
	// Inverting y to convert from pixel coordinates (top left origin) to our coordinate system (bottom left origin)
	// The first is great for camera image processing, but for linear algebra the second is more commonly used
}

/**
 * Distort the given point in image space
 */
template<typename Scalar, typename CalibScalar = double>
inline Eigen::Matrix<Scalar,2,1> distortPointUnstable(const CameraCalib_t<CalibScalar> &camera, const Eigen::Matrix<Scalar,2,1> &point, int iterations = 20, Scalar tolerance = 0.01f*PixelSize)
{
	auto &dist = camera.distortion;
	// As long as it's consistent, should be able to save the f multiplication here
	typedef Eigen::Matrix<Scalar,2,1> Vec2;
	Vec2 p = (point - camera.principalPoint.template cast<Scalar>()) * camera.fInv;
	Vec2 pu = p;
	for (int i = 0; i < iterations; i++)
	{ // This iteration may not converge for some distortion values
		Scalar rsq = p.squaredNorm();
		Scalar rd = (Scalar)1 + rsq * (dist.k1 + rsq * (dist.k2 + (rsq*dist.k3)));
		Scalar dx = (Scalar)2*dist.p1*p.x()*p.y() + dist.p2*(rsq+(Scalar)2*p.x()*p.x());
		Scalar dy = dist.p1*(rsq+(Scalar)2*p.y()*p.y()) + (Scalar)2*dist.p2*p.x()*p.y();
		Vec2 pd = p*rd + Vec2(dx, dy);
		if ((pd-pu).squaredNorm() < tolerance*tolerance)
			break;
		p = (pu - Vec2(dx, dy)) / rd;
	}
	return camera.principalPoint.template cast<Scalar>() + p * camera.f;
}

/**
 * Undistorts the given point in image space
 */
template<typename Scalar, typename CalibScalar = double>
inline Eigen::Matrix<Scalar,2,1> undistortPoint(const CameraCalib_t<CalibScalar> &camera, const Eigen::Matrix<Scalar,2,1> &point)
{
	// NOTE: This is different from most theory
	// Usually you multiply with fInv
	// And in other libraries (OpenCV), distort and undistort is switched
	// As there, undistortion is done by a lookup into a table (which is generated with the distort function)
	// In the end, what exact distortion model is used doesn't really matter
	// Optimisation will try to fit it to the data either way
	auto &dist = camera.distortion;
	typedef Eigen::Matrix<Scalar,2,1> Vec2;
	Vec2 p = (point - camera.principalPoint.template cast<Scalar>()) * camera.fInv;
	Scalar rsq = p.squaredNorm();
	Scalar rd = (Scalar)1 + rsq * (dist.k1 + rsq * (dist.k2 + (rsq*dist.k3)));
	Scalar dx = (Scalar)2*dist.p1*p.x()*p.y() + dist.p2*(rsq+(Scalar)2*p.x()*p.x());
	Scalar dy = dist.p1*(rsq+(Scalar)2*p.y()*p.y()) + (Scalar)2*dist.p2*p.x()*p.y();
	p = p*rd + Vec2(dx, dy);
	return camera.principalPoint.template cast<Scalar>() + p * camera.f;
}

#endif // EIGEN_UTIL_H