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

typedef uint32_t CameraID;
const CameraID CAMERA_ID_NONE = 0;

template<typename Scalar>
struct CameraCalib_t;

/**
 * Distortion preset of a known lens
 */
template<typename Scalar>
struct LensCalib_t
{
	int id;
	std::string label;
	Scalar f;
	Scalar k1;
	Scalar k2;
	Scalar k3;

	LensCalib_t() = default;
	LensCalib_t(int ID, CameraCalib_t<Scalar> calib)
	{
		id = ID;
		label = "New Lens";
		f = calib.f;
		k1 = calib.distortion.k1;
		k2 = calib.distortion.k2;
		k3 = calib.distortion.k3;
	}
};
typedef LensCalib_t<CVScalar> LensCalib;

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
	int lensID;

	/**
	 * Update the view and camera matrix when transform or intrinsiv parameters of cameras changed
	 */
	void UpdateDerived();

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
		other.lensID = lensID;
		return other;
	}

	CameraCalib_t()
	{
		transform.setIdentity();
		f = 1;
		fInv = 1;
		principalPoint.setZero();
		distortion.k1 = 0;
		distortion.k2 = 0;
		distortion.p1 = 0;
		distortion.p2 = 0;
		distortion.k3 = 0;
		lensID = -1;
		UpdateDerived();
	}

	CameraCalib_t(LensCalib_t<Scalar> lens)
	{
		transform.setIdentity();
		f = lens.f;
		fInv = 1.0f/lens.f;
		principalPoint.setZero();
		distortion.k1 = lens.k1;
		distortion.k2 = lens.k2;
		distortion.p1 = 0;
		distortion.p2 = 0;
		distortion.k3 = lens.k3;
		lensID = lens.id;
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

template<typename Scalar, int N>
struct Bounds
{
	using VEC = Eigen::Matrix<Scalar,N,1>;
	VEC min;
	VEC max;

	Bounds() 
	{
		min.setConstant(std::numeric_limits<Scalar>::max());
		max.setConstant(std::numeric_limits<Scalar>::lowest());
	}

	Bounds(Scalar MinX, Scalar MinY, Scalar MaxX, Scalar MaxY)
		: min(MinX, MinY), max(MaxX, MaxY)
	{ static_assert(N == 2); }

	Bounds(Scalar MinX, Scalar MinY, Scalar MinZ, Scalar MaxX, Scalar MaxY, Scalar MaxZ)
		: min(MinX, MinY, MinZ), max(MaxX, MaxY, MaxZ)
	{ static_assert(N == 3); }

	Bounds(VEC center, VEC size) 
	{
		min = center-size/2;
		max = center+size/2;
	}

	template<typename T>
	Bounds<T,N> cast() const
	{ // Conversion
		Bounds<T,N> other;
		other.min = min.template cast<T>();
		other.max = max.template cast<T>();
		return other;
	}

	template<typename ScalarPrec = Scalar>
	inline Eigen::Matrix<ScalarPrec,N,1> center() const
	{
		return (max.template cast<ScalarPrec>()+min.template cast<ScalarPrec>())/2;
	};
	inline VEC extends() const
	{
		return (max - min).cwiseMax(VEC::Zero());
	};
	inline bool overlaps(const Bounds<Scalar,N> other) const
	{
		return (other.min.array() - max.array()).maxCoeff() <= 0 && (other.max.array() - min.array()).minCoeff() >= 0;

	};
	inline void overlapWith(const Bounds<Scalar,N> other)
	{
		min = min.cwiseMax(other.min);
		max = max.cwiseMin(other.max);
	}
	inline bool includes(const Bounds<Scalar,N> other) const
	{
		return (other.max.array() - max.array()).maxCoeff() <= 0 && (other.min.array() - min.array()).minCoeff() >= 0;
	};
	inline bool includes(const VEC point) const
	{
		return (point.array() - max.array()).maxCoeff() < 0 && (point.array() - min.array()).minCoeff() >= 0;
	};
	inline void include(const VEC point)
	{
		min = min.cwiseMin(point);
		max = max.cwiseMax(point);
	};
	inline void include(const Bounds<Scalar,N> other)
	{
		min = min.cwiseMin(other.min);
		max = max.cwiseMax(other.max);
	}
	inline VEC clamp(const VEC point)
	{
		return point.cwiseMax(min).cwiseMin(max);
	}
	inline void extendBy(const VEC size)
	{
		min -= size;
		max += size;
	};
	inline void extendBy(Scalar size)
	{
		min -= VEC::Constant(size);
		max += VEC::Constant(size);
	};
	inline Bounds extendedBy(const VEC size) const
	{
		Bounds other = *this;
		other.extendBy(size);
		return other;
	};
	inline Bounds extendedBy(Scalar size) const
	{
		Bounds other = *this;
		other.extendBy(size);
		return other;
	};
	inline bool operator==(const Bounds<Scalar,N> other) const
	{
		return min == other.min && max == other.max;
	};
	inline Scalar size() const
	{
		return extends().prod();
	};
};

template<typename Scalar>
using Bounds2 = Bounds<Scalar,2>;

typedef Bounds2<int> Bounds2i;
typedef Bounds2<float> Bounds2f;

template<typename Scalar>
using Bounds3 = Bounds<Scalar,3>;

typedef Bounds3<int> Bounds3i;
typedef Bounds3<float> Bounds3f;


/* Constants */

static const double PI = 3.14159265358979323846;

// This is arbitrarily chosen and is only for visualisation and some easy-to-read limits
static const float PixelFactor = 1280/2.0f;
static const float PixelSize = 2.0f/1280;

#endif // EIGEN_DEF_H