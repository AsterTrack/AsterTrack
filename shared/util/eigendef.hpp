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

#ifndef EIGEN_DEF_H
#define EIGEN_DEF_H

#include "Eigen/Core"
#include "Eigen/Geometry"

/**
 * Utility definitions using Eigen
 */


/* Structures */

typedef double CVScalar; // Can switch quality of calibration and most internal geometric calculations

template<typename Scalar>
using Isometry3 = Eigen::Transform<Scalar, 3, Eigen::Isometry>;
template<typename Scalar>
using Affine3 = Eigen::Transform<Scalar, 3, Eigen::Affine>;
template<typename Scalar>
using Projective3 = Eigen::Transform<Scalar, 3, Eigen::Projective>;

template<typename Scalar>
using MatrixX = Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>;
template<typename Scalar>
using Matrix4 = Eigen::Matrix<Scalar, 4, 4>;
template<typename Scalar>
using Matrix3 = Eigen::Matrix<Scalar, 3, 3>;

template<typename Scalar>
using VectorX = Eigen::Matrix<Scalar, Eigen::Dynamic, 1>;
template<typename Scalar>
using Vector4 = Eigen::Matrix<Scalar, 4, 1>;
template<typename Scalar>
using Vector3 = Eigen::Matrix<Scalar, 3, 1>;
template<typename Scalar>
using Vector2 = Eigen::Matrix<Scalar, 2, 1>;

typedef int CameraID;
const CameraID CAMERA_ID_NONE = 0;

/**
 * Calibration data of a camera
 */
template<typename Scalar>
struct CameraCalib_t
{
	CameraID id = CAMERA_ID_NONE;
	int index = -1; // Merely for algorithms to organise that only have access to camera calibrations
	Eigen::Transform<Scalar,3,Eigen::Isometry> transform; // Source camera transform
	Eigen::Transform<Scalar,3,Eigen::Isometry> view; // := Inverse transform
	Eigen::Transform<Scalar,3,Eigen::Projective> projection; // := Projection(f, principalPoint)
	Eigen::Transform<Scalar,3,Eigen::Projective> camera; // := projection * view
	Scalar f; // f=1.0/tan(fovH/2), fovH = 2*atan(1.0/f)/PI*180.0, projection(0,0) = f, f=2*focalLen/sensorX
	Scalar fInv; // fInv=tan(fovH/2), fovH = 2*atan(fInv)/PI*180.0, projection(0,0) = 1/fInv, fInv=sensorX/focalLen/2
	Eigen::Matrix<Scalar,2,1> principalPoint;
	struct {
		Scalar k1;
		Scalar k2;
		Scalar p1;
		Scalar p2;
		Scalar k3;
	} distortion;

	/**
	 * Update the view and camera matrix when transform or intrinsiv parameters of cameras changed
	 */
	inline void UpdateDerived();

	inline bool valid() const { return id != CAMERA_ID_NONE; }
	inline bool invalid() const { return id == CAMERA_ID_NONE; }

	template<typename T>
	operator CameraCalib_t<T>() const
	{ // Conversion
		CameraCalib_t<T> other;
		other.id = id;
		other.index = index;
		other.transform = transform.template cast<T>();
		other.view = view.template cast<T>();
		other.projection = projection.template cast<T>();
		other.camera = camera.template cast<T>();
		other.f = (T)f;
		other.fInv = (T)fInv;
		other.principalPoint = principalPoint.template cast<T>();
		other.distortion.k1 = (T)distortion.k1;
		other.distortion.k2 = (T)distortion.k2;
		other.distortion.p1 = (T)distortion.p1;
		other.distortion.p2 = (T)distortion.p2;
		other.distortion.k3 = (T)distortion.k3;
		return other;
	}

	CameraCalib_t()
	{ // Conversion
		transform.setIdentity();
		f = 1;
		fInv = 1;
		principalPoint.setZero();
		distortion.k1 = 0;
		distortion.k2 = 0;
		distortion.p1 = 0;
		distortion.p2 = 0;
		distortion.k3 = 0;
		UpdateDerived();
	}
};
typedef CameraCalib_t<CVScalar> CameraCalib; 

/**
 * Physical data of a camera and its mode
 */
struct CameraMode
{
	int widthPx, heightPx;
	int binningX, binningY;
	int sensorWidth, sensorHeight;
	// Derivatives
	CVScalar cropWidth, cropHeight;
	CVScalar aspect;
	CVScalar sizeW, sizeH;
	CVScalar factorW, factorH;
	Eigen::Vector<CVScalar,2> crop;
	Eigen::Vector<CVScalar,2> size;
	Eigen::Vector<CVScalar,2> factor;
	// TODO: Rework CameraMode completely
	// Has a lot of "support" for different camera modes, but was never actually tested
	// Need to correct CameraCalib for full sensor if calibrated in a cropped mode
	// And gotta work on the layout/structure, perhaps use eigen
	// Maybe even exchange/join some stuff with CameraCalib since they are often needed together and highly dependent on each other

	CameraMode() {}

	CameraMode(int width, int height) : widthPx(width), heightPx(height), binningX(1), binningY(1), sensorWidth(width), sensorHeight(height)
	{
		update();
	}

	void update()
	{
		cropWidth = (CVScalar)(widthPx*binningX) / sensorWidth;
		cropHeight = (CVScalar)(heightPx*binningY) / sensorHeight;
		// Assuming square pixels, assumption made in calibration system as well
		aspect = (CVScalar)(heightPx*binningY) / (widthPx*binningX);
		sizeW = cropWidth;
		sizeH = cropHeight*aspect;
		factorW = (CVScalar)1.0 / sizeW;
		factorH = (CVScalar)1.0 / sizeH;
		crop = Eigen::Vector<CVScalar,2>(cropWidth, cropHeight);
		size = Eigen::Vector<CVScalar,2>(sizeW, sizeH);
		factor = Eigen::Vector<CVScalar,2>(factorW, factorH);
		// TODO: Convert to getter functions for lesser used stuff?
	}
};

template<typename Scalar>
struct Ray3_t
{
	Eigen::Matrix<Scalar,3,1> pos;
	Eigen::Matrix<Scalar,3,1> dir;
};
typedef Ray3_t<float> Ray3f;
typedef Ray3_t<double> Ray3d;

template<typename Scalar>
struct Bounds2
{
	Scalar minX, minY;
	Scalar maxX, maxY;

	Bounds2() 
	{
		minX = minY = std::numeric_limits<Scalar>::max();
		maxX = maxY = std::numeric_limits<Scalar>::lowest();
	}
	Bounds2(Scalar MinX, Scalar MinY, Scalar MaxX, Scalar MaxY)
		: minX(MinX), minY(MinY), maxX(MaxX), maxY(MaxY) {}
	Bounds2(Eigen::Matrix<Scalar,2,1> center, Eigen::Matrix<Scalar,2,1> size) 
	{
		minX = center.x()-size.x()/2;
		maxX = center.x()+size.x()/2;
		minY = center.y()-size.y()/2;
		maxY = center.y()+size.y()/2;
	}
	template<typename T>
	Bounds2<T> cast() const
	{ // Conversion
		Bounds2<T> other;
		other.minX = minX;
		other.minY = minY;
		other.maxX = maxX;
		other.maxY = maxY;
		return other;
	}


	template<typename ScalarPrec = Scalar>
	inline Eigen::Matrix<ScalarPrec,2,1> center() const
	{
		return Eigen::Matrix<ScalarPrec,2,1>((ScalarPrec)(maxX+minX)/(ScalarPrec)2, (ScalarPrec)(maxY+minY)/(ScalarPrec)2);
	};
	inline Eigen::Matrix<Scalar,2,1> extends() const
	{
		return Eigen::Matrix<Scalar,2,1>(
			maxX > minX? maxX-minX : 0, 
			maxY > minY? maxY-minY : 0);
	};
	inline Eigen::Matrix<Scalar,2,1> min() const
	{
		return Eigen::Matrix<Scalar,2,1>(minX, minY);
	};
	inline Eigen::Matrix<Scalar,2,1> max() const
	{
		return Eigen::Matrix<Scalar,2,1>(maxX, maxY);
	};
	inline bool overlaps(const Bounds2<Scalar> other) const
	{
		return maxX >= other.minX && other.maxX >= minX && maxY >= other.minY && other.maxY >= minY;
	};
	inline void overlapWith(const Bounds2<Scalar> other)
	{
		minX = std::max(minX, other.minX);
		maxX = std::min(maxX, other.maxX);
		minY = std::max(minY, other.minY);
		maxY = std::min(maxY, other.maxY);
	}
	inline bool includes(const Bounds2<Scalar> other) const
	{
		return maxX >= other.maxX && other.minX >= minX && maxY >= other.maxY && other.minY >= minY;
	};
	inline bool includes(const Eigen::Matrix<Scalar,2,1> point) const
	{
		return point.x() < maxX && point.x() >= minX && point.y() < maxY && point.y() >= minY;
	};
	inline void include(const Eigen::Matrix<Scalar,2,1> point)
	{
		minX = std::min(minX, point.x());
		maxX = std::max(maxX, point.x());
		minY = std::min(minY, point.y());
		maxY = std::max(maxY, point.y());
	};
	inline void include(const Bounds2<Scalar> other)
	{
		minX = std::min(minX, other.minX);
		minY = std::min(minY, other.minY);
		maxX = std::max(maxX, other.maxX);
		maxY = std::max(maxY, other.maxY);
	}
	inline Eigen::Matrix<Scalar,2,1> clamp(const Eigen::Matrix<Scalar,2,1> point)
	{
		return Eigen::Matrix<Scalar,2,1>(
			std::max(minX, std::min(maxX, point.x())),
			std::max(minY, std::min(maxY, point.y()))
		);
	}
	inline void extendBy(const Eigen::Matrix<Scalar,2,1> size)
	{
		minX -= size.x();
		maxX += size.x();
		minY -= size.y();
		maxY += size.y();
	};
	inline void extendBy(Scalar size)
	{
		minX -= size;
		maxX += size;
		minY -= size;
		maxY += size;
	};
	inline Bounds2 extendedBy(const Eigen::Matrix<Scalar,2,1> size) const
	{
		Bounds2 other = *this;
		other.extendBy(size);
		return other;
	};
	inline Bounds2 extendedBy(Scalar size) const
	{
		Bounds2 other = *this;
		other.extendBy(size);
		return other;
	};
	inline bool operator==(const Bounds2<Scalar> other) const
	{
		return maxX == other.maxX && other.minX == minX && maxY == other.maxY && other.minY == minY;
	};
	inline Scalar size() const
	{
		return extends().prod();
	};
};
typedef Bounds2<int> Bounds2i;
typedef Bounds2<float> Bounds2f;


template<typename Scalar>
struct Bounds3
{
	Scalar minX, minY, minZ;
	Scalar maxX, maxY, maxZ;

	Bounds3() 
	{
		minX = minY = minZ = std::numeric_limits<Scalar>::max();
		maxX = maxY = maxZ = std::numeric_limits<Scalar>::lowest();
	}
	Bounds3(Eigen::Matrix<Scalar,3,1> center, Eigen::Matrix<Scalar,3,1> size) 
	{
		minX = center.x()-size.x()/2;
		maxX = center.x()+size.x()/2;
		minY = center.y()-size.y()/2;
		maxY = center.y()+size.y()/2;
		minZ = center.z()-size.z()/2;
		maxZ = center.z()+size.z()/2;
	}
	inline Eigen::Matrix<Scalar,3,1> center() const
	{
		return Eigen::Matrix<Scalar,3,1>((maxX+minX)/2.0f, (maxY+minY)/2.0f, (maxZ+minZ)/2.0f);
	};
	inline Eigen::Matrix<Scalar,3,1> extends() const
	{
		return Eigen::Matrix<Scalar,3,1>(maxX-minX, maxY-minY, maxZ-minZ);
	};
	inline Eigen::Matrix<Scalar,3,1> min() const
	{
		return Eigen::Matrix<Scalar,3,1>(minX, minY, minZ);
	};
	inline Eigen::Matrix<Scalar,3,1> max() const
	{
		return Eigen::Matrix<Scalar,3,1>(maxX, maxY, maxZ);
	};
	inline bool overlaps(const Bounds3<Scalar> other) const
	{
		return maxX >= other.minX && other.maxX >= minX && maxY >= other.minY && other.maxY >= minY && maxZ >= other.minZ && other.maxZ >= minZ;
	};
	inline bool includes(const Eigen::Matrix<Scalar,3,1> point) const
	{
		return point.x() <= maxX && point.x() >= minX && point.y() <= maxY && point.y() >= minY && point.z() <= maxZ && point.z() >= minZ;
	};
	inline bool includes(const Bounds3<Scalar> other) const
	{
		return maxX >= other.maxX && other.minX >= minX && maxY >= other.maxY && other.minY >= minY && maxZ >= other.maxZ && other.minZ >= minZ;
	};
	inline void include(const Eigen::Matrix<Scalar,3,1> point)
	{
		if (point.x() < minX) minX = point.x();
		if (point.x() > maxX) maxX = point.x();
		if (point.y() < minY) minY = point.y();
		if (point.y() > maxY) maxY = point.y();
		if (point.z() < minZ) minZ = point.z();
		if (point.z() > maxZ) maxZ = point.z();
	};
	inline void extendBy(const Eigen::Matrix<Scalar,3,1> size)
	{
		minX -= size.x();
		maxX += size.x();
		minY -= size.y();
		maxY += size.y();
		minZ -= size.z();
		maxZ += size.z();
	};
	inline Bounds3 extendBy(Scalar size) const
	{
		extendBy(Eigen::Matrix<Scalar,3,1>::Constant(size));
	};
	inline Bounds3 extendedBy(const Eigen::Matrix<Scalar,3,1> size) const
	{
		Bounds3 other = *this;
		other.extendBy(size);
		return other;
	};
	inline Bounds3 extendedBy(Scalar size) const
	{
		Bounds3 other = *this;
		other.extendBy(Eigen::Matrix<Scalar,3,1>::Constant(size));
		return other;
	};
	inline bool operator==(const Bounds3<Scalar> other) const
	{
		return maxX == other.maxX && other.minX == minX && maxY == other.maxY && other.minY == minY && maxZ == other.maxZ && other.minZ == minZ;
	};
};
typedef Bounds3<int> Bounds3i;
typedef Bounds3<float> Bounds3f;


/* Constants */

static const double PI = 3.14159265358979323846;

// This is arbitrarily chosen and is only for visualisation and some easy-to-read limits
static const float PixelFactor = 1280/2.0f;
static const float PixelSize = 2.0f/1280;

#endif // EIGEN_DEF_H