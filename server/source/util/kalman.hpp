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

#ifndef KALMAN_H
#define KALMAN_H

#include "kalman/LinearizedSystemModel.hpp"
#include "kalman/LinearizedMeasurementModel.hpp"
#include "kalman/ExtendedKalmanFilter.hpp"
#include "kalman/UnscentedKalmanFilter.hpp"

/**
 * Implementation of several Kalman filters using a kalman library
 */


/**
 * Parameter controlling the timestep
 */
template<typename T>
class TimeControl : public Kalman::Vector<T, 1>
{
public:
	KALMAN_VECTOR(TimeControl, T, 1)

	TimeControl(T time) { dt() = time; }
	const T dt() const { return (*this)[0]; }
	T& dt() { return (*this)[0]; }
};

/**
 * State of a 3DOF object (position with acceleration-model)
 */
template<typename T>
class State3DOF : public Kalman::Vector<T, 9>
{
public:
	KALMAN_VECTOR(State3DOF, T, 9)

	typedef Eigen::Matrix<T,9,1> VectorBase;
	typedef Eigen::VectorBlock<VectorBase, 3> VectorSegment;
	typedef const Eigen::VectorBlock<const VectorBase, 3> ConstVectorSegment;

	ConstVectorSegment pos() const { return this->template segment<3>(0); }
	ConstVectorSegment vel() const { return this->template segment<3>(3); }
	ConstVectorSegment acc() const { return this->template segment<3>(6); }
	VectorSegment pos() { return this->template segment<3>(0); }
	VectorSegment vel() { return this->template segment<3>(3); }
	VectorSegment acc() { return this->template segment<3>(6); }
};

/**
 * Model of the behaviour of a 3DOF object (position with acceleration-model)
 */
template<typename T = float>
class SystemModel3DOF : public Kalman::LinearizedSystemModel<State3DOF<T>, TimeControl<T>>
{
public:
	typedef State3DOF<T> S;
	typedef TimeControl<T> C;

	SystemModel3DOF()
	{
		// P Covariance (noise) of system process f
		this->P.setZero();
		// Set up later depending on situation

		// W = df/dw (w.r.t. noise)
		// E.g. which state variables are stronger affected by noise
		this->W.setIdentity();

		// F = df/dx
		// Each parameter w.r.t. itself
		this->F.setIdentity();
		// Rest dynamic
	}

	void setExpectedError(float expectedError)
	{
		this->P.template block<3,3>(6,6).diagonal().setConstant(expectedError*expectedError);
	}

	S f(const S& x, const C& u) const
	{
		S nx;
		nx.acc() = x.acc();
		nx.vel() = x.vel() + x.acc() * u.dt();
		nx.pos() = x.pos() + x.vel() * u.dt() + x.acc() * u.dt()*u.dt()/2;
		return nx;
	}
	
	void updateJacobians(const S& x, const C& u)
	{
		// F = df/dx
		// Each parameter w.r.t. itself
		//this->F.setIdentity();
		// Vel w.r.t. Acc
		this->F.template block<3,3>(3,6).diagonal().setConstant(u.dt());
		// Pos w.r.t. Vel
		this->F.template block<3,3>(0,3).diagonal().setConstant(u.dt());
		// Pos w.r.t. Acc
		this->F.template block<3,3>(0,6).diagonal().setConstant(u.dt()*u.dt()/2);
	}
};

/**
 * Model of the observation of a 3DOF object (position with acceleration-model)
 */
template<typename T>
class MeasurementModel3DOF : public Kalman::LinearizedMeasurementModel<State3DOF<T>, Kalman::Vector<T, 3>>
{
public:
	typedef State3DOF<T> S;
	typedef Kalman::Vector<T, 3> M;

	MeasurementModel3DOF()
	{
		// P Covariance (noise) of measurement process h
		this->P.setZero();
		// Set on demand

		// H = dh/dx
		this->H.setZero();
		this->H.template block<3,3>(0,0).diagonal().setConstant(1);
		// V = dh/dw (w.r.t. noise)
		this->V.setIdentity();
	}

	M h(const S& x) const
	{
		M measurement;
		measurement = x.pos();
		return measurement;
	}

	void updateJacobians(const S& x)
	{
		// Perhaps update noise
		//this->P.template block<3,3>(0,0).diagonal().setConstant(0.1*0.1); // Noise in cm
	}

	/**
	 */
	void setExpectedError(float expectedError)
	{
		this->P.template block<3,3>(0,0).diagonal().setConstant(expectedError*expectedError); // Noise in cm
	}
};


/**
 * State of a 3DOF object (position with velocity-model)
 */
template<typename T>
class State3DOFV : public Kalman::Vector<T, 6>
{
public:
	KALMAN_VECTOR(State3DOFV, T, 6)

	typedef Eigen::Matrix<T,6,1> VectorBase;
	typedef Eigen::VectorBlock<VectorBase, 3> VectorSegment;
	typedef const Eigen::VectorBlock<const VectorBase, 3> ConstVectorSegment;

	ConstVectorSegment pos() const { return this->template segment<3>(0); }
	ConstVectorSegment vel() const { return this->template segment<3>(3); }
	VectorSegment pos() { return this->template segment<3>(0); }
	VectorSegment vel() { return this->template segment<3>(3); }
};

/**
 * Model of the behaviour of a 3DOF object (position with velocity-model)
 */
template<typename T = float>
class SystemModel3DOFV : public Kalman::LinearizedSystemModel<State3DOFV<T>, TimeControl<T>>
{
public:
	typedef State3DOFV<T> S;
	typedef TimeControl<T> C;

	SystemModel3DOFV()
	{
		// P Covariance (noise) of system process f
		this->P.setZero();
		// Set up later depending on situation

		// W = df/dw (w.r.t. noise)
		// E.g. which state variables are stronger affected by noise
		this->W.setIdentity();

		// F = df/dx
		// Each parameter w.r.t. itself
		this->F.setIdentity();
		// Rest dynamic
	}

	void setExpectedError(float expectedError)
	{
		this->P.template block<3,3>(3,3).diagonal().setConstant(expectedError*expectedError);
	}

	S f(const S& x, const C& u) const
	{
		S nx;
		nx.vel() = x.vel();
		nx.pos() = x.pos() + x.vel() * u.dt();
		return nx;
	}
	
	void updateJacobians(const S& x, const C& u)
	{
		// F = df/dx
		// Each parameter w.r.t. itself
		//this->F.setIdentity();
		// Pos w.r.t. Vel
		this->F.template block<3,3>(0,3).diagonal().setConstant(u.dt());
	}
};

/**
 * Model of the observation of a 3DOF object (position with velocity-model)
 */
template<typename T>
class MeasurementModel3DOFV : public Kalman::LinearizedMeasurementModel<State3DOFV<T>, Kalman::Vector<T, 3>>
{
public:
	typedef State3DOFV<T> S;
	typedef Kalman::Vector<T, 3> M;

	MeasurementModel3DOFV()
	{
		// P Covariance (noise) of measurement process h
		this->P.setZero();
		// Set on demand

		// H = dh/dx
		this->H.setZero();
		this->H.template block<3,3>(0,0).diagonal().setConstant(1);
		// V = dh/dw (w.r.t. noise)
		this->V.setIdentity();
	}

	M h(const S& x) const
	{
		M measurement;
		measurement = x.pos();
		return measurement;
	}

	void updateJacobians(const S& x)
	{
		// Perhaps update noise
		//this->P.template block<3,3>(0,0).diagonal().setConstant(0.1*0.1); // Noise in cm
	}

	/**
	 */
	void setExpectedError(float expectedError)
	{
		this->P.template block<3,3>(0,0).diagonal().setConstant(expectedError*expectedError); // Noise in cm
	}
};

/**
 * State of a 3DOF rotation object (rotation with velocity-model as quaternions)
 * Not a suitable representation for a Kalman Filter
 */
template<typename T>
class State3DOF_Quat : public Kalman::Vector<T, 8>
{
public:
	KALMAN_VECTOR(State3DOF_Quat, T, 8)

	typedef Eigen::Matrix<T,8,1> VectorBase;
	typedef Eigen::VectorBlock<VectorBase, 4> QuatSegment;
	typedef const Eigen::VectorBlock<const VectorBase, 4> ConstQuatSegment;

	ConstQuatSegment quatR() const { return this->template segment<4>(0); }
	ConstQuatSegment quatV() const { return this->template segment<4>(4); }
	QuatSegment quatR() { return this->template segment<4>(0); }
	QuatSegment quatV() { return this->template segment<4>(4); }

	const Eigen::Quaternion<T> getQuatR() const { return Eigen::Quaternion<T>(quatR()); }
	void setQuatR(const Eigen::Quaternion<T> &quat) { quatR() = quat.coeffs(); }
	const Eigen::Quaternion<T> getQuatV() const { return Eigen::Quaternion<T>(quatV()); }
	void setQuatV(const Eigen::Quaternion<T> &quat) { quatV() = quat.coeffs(); }
};

/**
 * State of a 6DOF object (position with velocity-model + rotation with velocity-model as quaternions)
 * Not a suitable representation for a Kalman Filter
 */
template<typename T>
class State6DOF_Quat : public Kalman::Vector<T, 14>
{
public:
	KALMAN_VECTOR(State6DOF_Quat, T, 14)

	typedef Eigen::Matrix<T,14,1> VectorBase;
	typedef Eigen::VectorBlock<VectorBase, 3> VectorSegment;
	typedef Eigen::VectorBlock<VectorBase, 4> QuatSegment;
	typedef const Eigen::VectorBlock<const VectorBase, 3> ConstVectorSegment;
	typedef const Eigen::VectorBlock<const VectorBase, 4> ConstQuatSegment;

	ConstVectorSegment pos() const { return this->template segment<3>(0); }
	ConstVectorSegment vel() const { return this->template segment<3>(3); }
	ConstQuatSegment quatR() const { return this->template segment<4>(6); }
	ConstQuatSegment quatV() const { return this->template segment<4>(10); }
	VectorSegment pos() { return this->template segment<3>(0); }
	VectorSegment vel() { return this->template segment<3>(3); }
	QuatSegment quatR() { return this->template segment<4>(6); }
	QuatSegment quatV() { return this->template segment<4>(10); }

	const Eigen::Quaternion<T> getQuatR() const { return Eigen::Quaternion<T>(quatR()); }
	void setQuatR(const Eigen::Quaternion<T> &quat) { quatR() = quat.coeffs(); }
	const Eigen::Quaternion<T> getQuatV() const { return Eigen::Quaternion<T>(quatV()); }
	void setQuatV(const Eigen::Quaternion<T> &quat) { quatV() = quat.coeffs(); }
};


/**
 * State of a 6DOF object (position with acceleration-model + rotation with velocity-model implemented as MEKF)
 */
template<typename T>
class State6DOF_MEKF : public Kalman::Vector<T, 15>
{
public:
	KALMAN_VECTOR(State6DOF_MEKF, T, 15)

	typedef Eigen::Matrix<T,15,1> VectorBase;
	typedef Eigen::VectorBlock<VectorBase, 3> VectorSegment;
	typedef const Eigen::VectorBlock<const VectorBase, 3> ConstVectorSegment;

	// Note on choice of rotation representation
	// Since deltaT is varying (e.g. angle of velocity/acceleration has to change)
	// Only a rotation representation exposing the angle is useful
	// That includes AngleAxis, which can be compressed to 3 values (EncodeAARot/DecodeAARot)
	// And converted to quaternion (two trigonometry functions)

	ConstVectorSegment pos() const { return this->template segment<3>(0); }
	ConstVectorSegment vel() const { return this->template segment<3>(3); }
	ConstVectorSegment acc() const { return this->template segment<3>(6); }
	ConstVectorSegment rDel() const { return this->template segment<3>(9); }
	ConstVectorSegment rVel() const { return this->template segment<3>(12); }
	VectorSegment pos() { return this->template segment<3>(0); }
	VectorSegment vel() { return this->template segment<3>(3); }
	VectorSegment acc() { return this->template segment<3>(6); }
	VectorSegment rDel() { return this->template segment<3>(9); }
	VectorSegment rVel() { return this->template segment<3>(12); }

	/* MEKF-specific implementation */

	Eigen::Quaternion<T> referenceQuat;
	void reset()
	{
		referenceQuat = quat();
		rDel().setZero();
	}

	const Eigen::Quaternion<T> getRDelQuat() const { return MRP2Quat(rDel()); }
	void setRDelQuat(const Eigen::Quaternion<T> &quat) { rDel() = Quat2MRP(quat); }
	const Eigen::Quaternion<T> getRVelQuat() const { return MRP2Quat(rVel()); }
	void setRVelQuat(const Eigen::Quaternion<T> &quat) { rVel() = Quat2MRP(quat); }

	const Eigen::Quaternion<T> quat() const
	{
		return referenceQuat * getRDelQuat();
	}
};

/**
 * Model of the behaviour of a 6DOF object (position with acceleration-model + rotation with velocity-model implemented as MEKF)
 */
template<typename T = float>
class SystemModel6DOF_MEKF : public Kalman::LinearizedSystemModel<State6DOF_MEKF<T>, TimeControl<T>>
{
public:
	typedef State6DOF_MEKF<T> S;
	typedef TimeControl<T> C;

	SystemModel6DOF_MEKF()
	{
		// P Covariance (noise) of system process f
		this->P.setZero();
		// Set up later depending on situation

		// W = df/dw (w.r.t. noise)
		// E.g. which state variables are stronger affected by noise
		this->W.setIdentity();

		// F = df/dx
		// Each parameter w.r.t. itself
		this->F.setIdentity();
		// Rest dynamic
	}

	void setExpectedError(float expectedErrorPos, float expectedErrorRot)
	{
		this->P.template block<3,3>(6,6).diagonal().setConstant(expectedErrorPos*expectedErrorPos);
		this->P.template block<3,3>(12,12).diagonal().setConstant(expectedErrorRot*expectedErrorRot);
	}

	S f(const S& x, const C& u) const
	{
		S nx;
		nx.referenceQuat = x.referenceQuat;

		nx.acc() = x.acc();
		nx.vel() = x.vel() + x.acc() * u.dt();
		nx.pos() = x.pos() + x.vel() * u.dt() + x.acc() * u.dt()*u.dt()/2;

		nx.rVel() = x.rVel();
		nx.setRDelQuat(x.getRDelQuat() * x.getRVelQuat());
		return nx;
	}

	void updateJacobians(const S& x, const C& u)
	{
		// F = df/dx
		// Each parameter w.r.t. itself
		//this->F.setIdentity();
		// Vel w.r.t. Acc
		this->F.template block<3,3>(3,6).diagonal().setConstant(u.dt());
		// Pos w.r.t. Vel
		this->F.template block<3,3>(0,3).diagonal().setConstant(u.dt());
		// Pos w.r.t. Acc
		this->F.template block<3,3>(0,6).diagonal().setConstant(u.dt()*u.dt()/2);

		/* THEORY NOTES for
		nx.setRVelQuat(x.getRVelQuat() * x.getRAccQuat());

		// Conversion from MRP to Quat for i = 1,2
		ui = (ri, si, ti)
		ai = ri*ri + si*si + ti*ti;
		wi = (16 - ai) / (16 + ai);
		vi = ui * 8 / (16 + ai)
		vi = (xi, yi, zi)
		// Derivation helpers
		mi = 16+ai
		mi2 = (16+ai)^2
		ai/ui = 2ui
		// Derivation for conversion
		wi/ui = ui * -64/mi2
		vi/ui = - 8(2*ui^2-mi)/mi2
		vi/ui (diff) = - 16(vi*ui)/m2
		// Quaternion product, derivation easy
		//v = w1 * v2 + w2*v1 - v1.cross(v2);
		x = w1*x2 + x1*w2 - y1*z2 + z1*y2;
		y = w1*y2 + y1*w2 - z1*x2 + x1*z2;
		z = w1*z2 + z1*w2 - x1*y2 + y1*x2;
		w = w1*w2 - x1*x2 - y1*y2 - z1*z2;
		// Convert quaternion product to MRP
		q = v * 4/(1+w);
		*/

		// Conversion from MRP to Quat, intermediate values
		Eigen::Matrix<T,3,1> u1 = x.rDel(), u2 = x.rVel();
		T a1 = u1.squaredNorm(), a2 = u2.squaredNorm();
		T m1 = 16+a1, m2 = 16+a2;
		T m12 = m1*m1, m22 = m2*m2;
		T f1 = 8/m12, f2 = 8/m22;
		T h1 = 8/m1, h2 = 8/m2;
		Eigen::Matrix<T,4,1> q1 (u1.x()*h1, u1.y()*h1, u1.z()*h1, (16-a1)/m1);
		Eigen::Matrix<T,4,1> q2 (u2.x()*h2, u2.y()*h2, u2.z()*h2, (16-a2)/m2);

		// Conversion from MRP to Quat for source 1, derivative
		Eigen::Matrix<T,4,3> q1d;
		q1d(0,0) = -(2*u1.x()*u1.x() - m1) * f1;
		q1d(1,1) = -(2*u1.y()*u1.y() - m1) * f1;
		q1d(2,2) = -(2*u1.z()*u1.z() - m1) * f1;
		q1d(0,1) = -2*u1.x()*u1.y() * f1;
		q1d(0,2) = -2*u1.x()*u1.z() * f1;
		q1d(1,2) = -2*u1.y()*u1.z() * f1;
		q1d(1,0) = q1d(0,1);
		q1d(2,0) = q1d(0,2);
		q1d(2,1) = q1d(1,2);
		q1d.row(3) = u1 * -8*f1;

		// Conversion from MRP to Quat for source 2, derivative
		Eigen::Matrix<T,4,3> q2d;
		q2d(0,0) = -(2*u2.x()*u2.x() - m2) * f2;
		q2d(1,1) = -(2*u2.y()*u2.y() - m2) * f2;
		q2d(2,2) = -(2*u2.z()*u2.z() - m2) * f2;
		q2d(0,1) = -2*u2.x()*u2.y() * f2;
		q2d(0,2) = -2*u2.x()*u2.z() * f2;
		q2d(1,2) = -2*u2.y()*u2.z() * f2;
		q2d(1,0) = q2d(0,1);
		q2d(2,0) = q2d(0,2);
		q2d(2,1) = q2d(1,2);
		q2d.row(3) = u2 * -8*f2;

		// Quaternion product
		Eigen::Matrix<T,4,1> q;
		q.x() = q1.w()*q2.x() + q1.x()*q2.w() - q1.y()*q2.z() + q1.z()*q2.y();
		q.y() = q1.w()*q2.y() + q1.y()*q2.w() - q1.z()*q2.x() + q1.x()*q2.z();
		q.z() = q1.w()*q2.z() + q1.z()*q2.w() - q1.x()*q2.y() + q1.y()*q2.x();
		q.w() = q1.w()*q2.w() - q1.x()*q2.x() - q1.y()*q2.y() - q1.z()*q2.z();

		// Quaternion product, derivative for source 1
		Eigen::Matrix<T,4,3> qd1;
		qd1.row(0) = q1d.row(3)*q2(0) + q1d.row(0)*q2(3) - q1d.row(1)*q2(2) + q1d.row(2)*q2(1);
		qd1.row(1) = q1d.row(3)*q2(1) + q1d.row(1)*q2(3) - q1d.row(2)*q2(0) + q1d.row(0)*q2(2);
		qd1.row(2) = q1d.row(3)*q2(2) + q1d.row(2)*q2(3) - q1d.row(0)*q2(1) + q1d.row(1)*q2(0);
		qd1.row(3) = q1d.row(3)*q2(3) - q1d.row(0)*q2(0) - q1d.row(1)*q2(1) - q1d.row(2)*q2(2);

		// Quaternion product, derivative for source 2
		Eigen::Matrix<T,4,3> qd2;
		qd2.row(0) = q1(3)*q2d.row(0) + q1(0)*q2d.row(3) - q1(1)*q2d.row(2) + q1(2)*q2d.row(1);
		qd2.row(1) = q1(3)*q2d.row(1) + q1(1)*q2d.row(3) - q1(2)*q2d.row(0) + q1(0)*q2d.row(2);
		qd2.row(2) = q1(3)*q2d.row(2) + q1(2)*q2d.row(3) - q1(0)*q2d.row(1) + q1(1)*q2d.row(0);
		qd2.row(3) = q1(3)*q2d.row(3) - q1(0)*q2d.row(0) - q1(1)*q2d.row(1) - q1(2)*q2d.row(2);


		T w = q.w() + 1;
		T wI = 4 / (w*w);

		Eigen::Matrix<T,3,1> mrp;
		mrp = q.template block<3,1>(0,0) * 4/w;

		// Conversion back to MRP, derivative for source 1
		Eigen::Matrix<T,3,3> mrp1;
		mrp1.row(0) = (w * qd1.row(0) - q(0) * qd1.row(3)) * wI;
		mrp1.row(1) = (w * qd1.row(1) - q(1) * qd1.row(3)) * wI;
		mrp1.row(2) = (w * qd1.row(2) - q(2) * qd1.row(3)) * wI;

		// Conversion back to MRP, derivative for source 2
		Eigen::Matrix<T,3,3> mrp2;
		mrp2.row(0) = (w * qd2.row(0) - q(0) * qd2.row(3)) * wI;
		mrp2.row(1) = (w * qd2.row(1) - q(1) * qd2.row(3)) * wI;
		mrp2.row(2) = (w * qd2.row(2) - q(2) * qd2.row(3)) * wI;

		/*
		T w1 = (16-a1)/m1, w2 = (16-a2)/m2;
		T x1 = u1.x()*h1, x2 = u2.x()*8/m2;
		T y1 = u1.y()*h1, y2 = u2.y()*8/m2;
		T z1 = u1.z()*h1, z2 = u2.z()*8/m2;
		*/

		/*
		Eigen::Vector<T,4,1> q;
		q.x() = w1*x2 + x1*w2 - y1*z2 + z1*y2;
		q.y() = w1*y2 + y1*w2 - z1*x2 + x1*z2;
		q.z() = w1*z2 + z1*w2 - x1*y2 + y1*x2;
		q.w() = w1*w2 - x1*x2 - y1*y2 - z1*z2;

		Eigen::Matrix<T,4,3> qd1;
		qd1.row(0) = q1d.row(3)*x2 + q1d.row(0)*w2 - q1d.row(1)*z2 + q1d.row(2)*y2;
		qd1.row(1) = q1d.row(3)*y2 + q1d.row(1)*w2 - q1d.row(2)*x2 + q1d.row(0)*z2;
		qd1.row(2) = q1d.row(3)*z2 + q1d.row(2)*w2 - q1d.row(0)*y2 + q1d.row(1)*x2;
		qd1.row(3) = q1d.row(3)*w2 + q1d.row(0)*x2 - q1d.row(1)*y2 + q1d.row(2)*z2;

		Eigen::Matrix<T,4,3> qd2;
		qd2.row(0) = w1*q2d.row(0) + x1*q2d.row(3) - y1*q2d.row(2) + z1*q2d.row(1);
		qd2.row(1) = w1*q2d.row(1) + y1*q2d.row(3) - z1*q2d.row(0) + x1*q2d.row(2);
		qd2.row(2) = w1*q2d.row(2) + z1*q2d.row(3) - x1*q2d.row(1) + y1*q2d.row(0);
		qd2.row(3) = w1*q2d.row(3) - x1*q2d.row(0) - y1*q2d.row(1) - z1*q2d.row(2);
		*/

		/*Eigen::Matrix<T,4,3> qd1;
		for (int i = 0; i < 3; i++)
		{
			qd1(0,i) = q1d(3,i)*x2 + q1d(0,i)*w2 - q1d(1,i)*z2 + q1d(2,i)*y2;
			qd1(1,i) = q1d(3,i)*y2 + q1d(1,i)*w2 - q1d(2,i)*x2 + q1d(0,i)*z2;
			qd1(2,i) = q1d(3,i)*z2 + q1d(2,i)*w2 - q1d(0,i)*y2 + q1d(1,i)*x2;
			qd1(3,i) = q1d(3,i)*w2 + q1d(0,i)*x2 - q1d(1,i)*y2 + q1d(2,i)*z2;
		}

		Eigen::Matrix<T,4,3> qd2;
		for (int i = 0; i < 3; i++)
		{
			qd2(0,i) = w1*q2d(0,i) + x1*q2d(3,i) - y1*q2d(2,i) + z1*q2d(1,i);
			qd2(1,i) = w1*q2d(1,i) + y1*q2d(3,i) - z1*q2d(0,i) + x1*q2d(2,i);
			qd2(2,i) = w1*q2d(2,i) + z1*q2d(3,i) - x1*q2d(1,i) + y1*q2d(0,i);
			qd2(3,i) = w1*q2d(3,i) - x1*q2d(0,i) - y1*q2d(1,i) - z1*q2d(2,i);
		}*/

		// Rot w.r.t. Rot
		this->F.template block<3,3>(9,9) = mrp1;
		// Rot w.r.t. RotVel
		this->F.template block<3,3>(9,12) = mrp2;




		Eigen::Quaternionf quat1 = MRP2Quat(u1), quat2 = MRP2Quat(u2);
		Eigen::Quaternionf quat = quat1 * quat2;
		Eigen::Matrix<T,3,1> mrpcp = Quat2MRP(quat);

		float d = 0.0001f;
		Eigen::Matrix<T,3,1> x1 = u1, x2 = u2;

/*		LOG(LTrackingFilter, LTrace, "(%.3f %.3f %.3f) x (%.3f %.3f %.3f)\n", u1.x(), u1.y(), u1.z(), u2.x(), u2.y(), u2.z());
		LOG(LTrackingFilter, LTrace, "Quats (%.3f, %.3f %.3f %.3f) x (%.3f %.3f %.3f %.3f)\n", quat1.x(), quat1.y(), quat1.z(), quat1.w(), quat2.x(), quat2.y(), quat2.z(), quat2.w());
		LOG(LTrackingFilter, LTrace, "Q1Q2 (%.3f, %.3f %.3f %.3f) x (%.3f %.3f %.3f %.3f)\n", q1.x(), q1.y(), q1.z(), q1.w(), q2.x(), q2.y(), q2.z(), q2.w());
		LOG(LTrackingFilter, LTrace, "Q1 d / pos\n");
		LOG(LTrackingFilter, LTrace, "%.3f %.3f %.3f\n", q1d(0,0), q1d(0,1), q1d(0,2));
		LOG(LTrackingFilter, LTrace, "%.3f %.3f %.3f\n", q1d(1,0), q1d(1,1), q1d(1,2));
		LOG(LTrackingFilter, LTrace, "%.3f %.3f %.3f\n", q1d(2,0), q1d(2,1), q1d(2,2));
		LOG(LTrackingFilter, LTrace, "%.3f %.3f %.3f\n", q1d(3,0), q1d(3,1), q1d(3,2));
		LOG(LTrackingFilter, LTrace, "Q2 d / vel\n");
		LOG(LTrackingFilter, LTrace, "%.3f %.3f %.3f\n", q2d(0,0), q2d(0,1), q2d(0,2));
		LOG(LTrackingFilter, LTrace, "%.3f %.3f %.3f\n", q2d(1,0), q2d(1,1), q2d(1,2));
		LOG(LTrackingFilter, LTrace, "%.3f %.3f %.3f\n", q2d(2,0), q2d(2,1), q2d(2,2));
		LOG(LTrackingFilter, LTrace, "%.3f %.3f %.3f\n", q2d(3,0), q2d(3,1), q2d(3,2));


		Eigen::Matrix<T,4,3> q1diff;
		Eigen::Matrix<T,4,3> q2diff;

		{
			x1 = u1; x2 = u2; x1(0) += d;
			Eigen::Matrix<T,4,1> q1dd = MRP2Quat(x1).coeffs();
			q1diff.col(0) = (q1dd-q1)/d;
		}
		{
			x1 = u1; x2 = u2; x1(1) += d;
			Eigen::Matrix<T,4,1> q1dd = MRP2Quat(x1).coeffs();
			q1diff.col(1) = (q1dd-q1)/d;
		}
		{
			x1 = u1; x2 = u2; x1(2) += d;
			Eigen::Matrix<T,4,1> q1dd = MRP2Quat(x1).coeffs();
			q1diff.col(2) = (q1dd-q1)/d;
		}
		{
			x1 = u1; x2 = u2; x2(0) += d;
			Eigen::Matrix<T,4,1> q2dd = MRP2Quat(x2).coeffs();
			q2diff.col(0) = (q2dd-q2)/d;
		}
		{
			x1 = u1; x2 = u2; x2(1) += d;
			Eigen::Matrix<T,4,1> q2dd = MRP2Quat(x2).coeffs();
			q2diff.col(1) = (q2dd-q2)/d;
		}
		{
			x1 = u1; x2 = u2; x2(2) += d;
			Eigen::Matrix<T,4,1> q2dd = MRP2Quat(x2).coeffs();
			q2diff.col(2) = (q2dd-q2)/d;
		}

		LOG(LTrackingFilter, LTrace, "Q1 Differential / vel\n");
		LOG(LTrackingFilter, LTrace, "%.3f %.3f %.3f\n", q1diff(0,0), q1diff(0,1), q1diff(0,2));
		LOG(LTrackingFilter, LTrace, "%.3f %.3f %.3f\n", q1diff(1,0), q1diff(1,1), q1diff(1,2));
		LOG(LTrackingFilter, LTrace, "%.3f %.3f %.3f\n", q1diff(2,0), q1diff(2,1), q1diff(2,2));
		LOG(LTrackingFilter, LTrace, "%.3f %.3f %.3f\n", q1diff(3,0), q1diff(3,1), q1diff(3,2));
		LOG(LTrackingFilter, LTrace, "Q2 Differential / vel\n");
		LOG(LTrackingFilter, LTrace, "%.3f %.3f %.3f\n", q2diff(0,0), q2diff(0,1), q2diff(0,2));
		LOG(LTrackingFilter, LTrace, "%.3f %.3f %.3f\n", q2diff(1,0), q2diff(1,1), q2diff(1,2));
		LOG(LTrackingFilter, LTrace, "%.3f %.3f %.3f\n", q2diff(2,0), q2diff(2,1), q2diff(2,2));
		LOG(LTrackingFilter, LTrace, "%.3f %.3f %.3f\n", q2diff(3,0), q2diff(3,1), q2diff(3,2));


		LOG(LTrackingFilter, LTrace, "Q (%.3f %.3f %.3f %.3f)\n", q.x(), q.y(), q.z(), q.w());
		LOG(LTrackingFilter, LTrace, "Quat (%.3f %.3f %.3f %.3f)\n", quat.x(), quat.y(), quat.z(), quat.w());
		LOG(LTrackingFilter, LTrace, "Q / pos / vel\n");
		LOG(LTrackingFilter, LTrace, "%.3f %.3f %.3f  %.3f %.3f %.3f\n", qd1(0,0), qd1(0,1), qd1(0,2), qd2(0,0), qd2(0,1), qd2(0,2));
		LOG(LTrackingFilter, LTrace, "%.3f %.3f %.3f  %.3f %.3f %.3f\n", qd1(1,0), qd1(1,1), qd1(1,2), qd2(1,0), qd2(1,1), qd2(1,2));
		LOG(LTrackingFilter, LTrace, "%.3f %.3f %.3f  %.3f %.3f %.3f\n", qd1(2,0), qd1(2,1), qd1(2,2), qd2(2,0), qd2(2,1), qd2(2,2));
		LOG(LTrackingFilter, LTrace, "%.3f %.3f %.3f  %.3f %.3f %.3f\n", qd1(3,0), qd1(3,1), qd1(3,2), qd2(3,0), qd2(3,1), qd2(3,2));


		Eigen::Matrix<T,4,3> quatd1;
		Eigen::Matrix<T,4,3> quatd2;

		{
			x1 = u1; x2 = u2; x1(0) += d;
			Eigen::Matrix<T,4,1> qdd = (MRP2Quat(x1) * MRP2Quat(x2)).coeffs();
			quatd1.col(0) = (qdd-q)/d;
		}
		{
			x1 = u1; x2 = u2; x1(1) += d;
			Eigen::Matrix<T,4,1> qdd = (MRP2Quat(x1) * MRP2Quat(x2)).coeffs();
			quatd1.col(1) = (qdd-q)/d;
		}
		{
			x1 = u1; x2 = u2; x1(2) += d;
			Eigen::Matrix<T,4,1> qdd = (MRP2Quat(x1) * MRP2Quat(x2)).coeffs();
			quatd1.col(2) = (qdd-q)/d;
		}
		{
			x1 = u1; x2 = u2; x2(0) += d;
			Eigen::Matrix<T,4,1> qdd = (MRP2Quat(x1) * MRP2Quat(x2)).coeffs();
			quatd2.col(0) = (qdd-q)/d;
		}
		{
			x1 = u1; x2 = u2; x2(1) += d;
			Eigen::Matrix<T,4,1> qdd = (MRP2Quat(x1) * MRP2Quat(x2)).coeffs();
			quatd2.col(1) = (qdd-q)/d;
		}
		{
			x1 = u1; x2 = u2; x2(2) += d;
			Eigen::Matrix<T,4,1> qdd = (MRP2Quat(x1) * MRP2Quat(x2)).coeffs();
			quatd2.col(2) = (qdd-q)/d;
		}
		LOG(LTrackingFilter, LTrace, "Q Differential / pos / vel\n");
		LOG(LTrackingFilter, LTrace, "%.3f %.3f %.3f  %.3f %.3f %.3f\n", quatd1(0,0), quatd1(0,1), quatd1(0,2), quatd2(0,0), quatd2(0,1), quatd2(0,2));
		LOG(LTrackingFilter, LTrace, "%.3f %.3f %.3f  %.3f %.3f %.3f\n", quatd1(1,0), quatd1(1,1), quatd1(1,2), quatd2(1,0), quatd2(1,1), quatd2(1,2));
		LOG(LTrackingFilter, LTrace, "%.3f %.3f %.3f  %.3f %.3f %.3f\n", quatd1(2,0), quatd1(2,1), quatd1(2,2), quatd2(2,0), quatd2(2,1), quatd2(2,2));
		LOG(LTrackingFilter, LTrace, "%.3f %.3f %.3f  %.3f %.3f %.3f\n", quatd1(3,0), quatd1(3,1), quatd1(3,2), quatd2(3,0), quatd2(3,1), quatd2(3,2));


		LOG(LTrackingFilter, LTrace, "MRP (%.3f %.3f %.3f)\n", mrp.x(), mrp.y(), mrp.z());
		LOG(LTrackingFilter, LTrace, "MRP CP (%.3f %.3f %.3f)\n", mrpcp.x(), mrpcp.y(), mrpcp.z());
		LOG(LTrackingFilter, LTrace, "MRP / pos / vel\n");
		LOG(LTrackingFilter, LTrace, "%.3f %.3f %.3f  %.3f %.3f %.3f\n", mrp1(0,0), mrp1(0,1), mrp1(0,2), mrp2(0,0), mrp2(0,1), mrp2(0,2));
		LOG(LTrackingFilter, LTrace, "%.3f %.3f %.3f  %.3f %.3f %.3f\n", mrp1(1,0), mrp1(1,1), mrp1(1,2), mrp2(1,0), mrp2(1,1), mrp2(1,2));
		LOG(LTrackingFilter, LTrace, "%.3f %.3f %.3f  %.3f %.3f %.3f\n", mrp1(2,0), mrp1(2,1), mrp1(2,2), mrp2(2,0), mrp2(2,1), mrp2(2,2));

*/


		Eigen::Matrix<T,3,3> mrpd1, mrpd2;
		{
			x1 = u1; x2 = u2; x1(0) += d;
			Eigen::Matrix<T,3,1> mrpd = Quat2MRP(MRP2Quat(x1) * MRP2Quat(x2));
			mrpd1.col(0) = (mrpd-mrpcp)/d;
		}
		{
			x1 = u1; x2 = u2; x1(1) += d;
			Eigen::Matrix<T,3,1> mrpd = Quat2MRP(MRP2Quat(x1) * MRP2Quat(x2));
			mrpd1.col(1) = (mrpd-mrpcp)/d;
		}
		{
			x1 = u1; x2 = u2; x1(2) += d;
			Eigen::Matrix<T,3,1> mrpd = Quat2MRP(MRP2Quat(x1) * MRP2Quat(x2));
			mrpd1.col(2) = (mrpd-mrpcp)/d;
		}
		{
			x1 = u1; x2 = u2; x2(0) += d;
			Eigen::Matrix<T,3,1> mrpd = Quat2MRP(MRP2Quat(x1) * MRP2Quat(x2));
			mrpd2.col(0) = (mrpd-mrpcp)/d;
		}
		{
			x1 = u1; x2 = u2; x2(1) += d;
			Eigen::Matrix<T,3,1> mrpd = Quat2MRP(MRP2Quat(x1) * MRP2Quat(x2));
			mrpd2.col(1) = (mrpd-mrpcp)/d;
		}
		{
			x1 = u1; x2 = u2; x2(2) += d;
			Eigen::Matrix<T,3,1> mrpd = Quat2MRP(MRP2Quat(x1) * MRP2Quat(x2));
			mrpd2.col(2) = (mrpd-mrpcp)/d;
		}
/*		LOG(LTrackingFilter, LTrace, "MRP Differential / pos / vel\n");
		LOG(LTrackingFilter, LTrace, "%.3f %.3f %.3f  %.3f %.3f %.3f\n", mrpd1(0,0), mrpd1(0,1), mrpd1(0,2), mrpd2(0,0), mrpd2(0,1), mrpd2(0,2));
		LOG(LTrackingFilter, LTrace, "%.3f %.3f %.3f  %.3f %.3f %.3f\n", mrpd1(1,0), mrpd1(1,1), mrpd1(1,2), mrpd2(1,0), mrpd2(1,1), mrpd2(1,2));
		LOG(LTrackingFilter, LTrace, "%.3f %.3f %.3f  %.3f %.3f %.3f\n", mrpd1(2,0), mrpd1(2,1), mrpd1(2,2), mrpd2(2,0), mrpd2(2,1), mrpd2(2,2));

*/
		// Rot w.r.t. Rot
		this->F.template block<3,3>(9,9) = mrpd1;
		// Rot w.r.t. RotVel
		this->F.template block<3,3>(9,12) = mrpd2;
	}
};

/**
 * Model of the observation of a 6DOF object (position with acceleration-model + rotation with velocity-model implemented as MEKF)
 */
template<typename T>
class MeasurementModel6DOF_MEKF : public Kalman::LinearizedMeasurementModel<State6DOF_MEKF<T>, Kalman::Vector<T, 6>>
{
public:
	typedef State6DOF_MEKF<T> S;
	typedef Kalman::Vector<T, 6> M;

	MeasurementModel6DOF_MEKF()
	{
		// P Covariance (noise) of measurement process h
		this->P.setZero();
		// Set on demand

		// H = dh/dx
		this->H.setZero();
		this->H.template block<3,3>(0,0).diagonal().setConstant(1);
		this->H.template block<3,3>(3,9).diagonal().setConstant(1);
		// V = dh/dw (w.r.t. noise)
		this->V.setIdentity();
	}

	M h(const S& x) const
	{
		M measurement;
		measurement.template segment<3>(0) = x.pos();
		measurement.template segment<3>(3) = x.rDel();
		return measurement;
	}

	void setExpectedError(float expectedErrorPos, float expectedErrorRot)
	{
		this->P.template block<3,3>(0,0).diagonal().setConstant(expectedErrorPos*expectedErrorPos); // Noise in cm
		this->P.template block<3,3>(3,3).diagonal().setConstant(expectedErrorRot*expectedErrorRot); // Noise in radians
	}
};






#if 0 // Rotation acceleration model incomplete


/**
 * Parameters for the state of a 6DOF object (position with acceleration-model + rotation with acceleration-model implemented as MEKF)
 */
template<typename T>
class State6DOF_MEKF : public Kalman::Vector<T, 18>
{
public:
	KALMAN_VECTOR(State6DOF_MEKF, T, 18)

	typedef Eigen::Matrix<T,18,1> VectorBase;
	typedef Eigen::VectorBlock<VectorBase, 3> VectorSegment;
	typedef const Eigen::VectorBlock<const VectorBase, 3> ConstVectorSegment;

	// Note on choice of rotation representation
	// Since deltaT is varying (e.g. angle of velocity/acceleration has to change)
	// Only a rotation representation exposing the angle is useful
	// That includes AngleAxis, which can be compressed to 3 values (EncodeAARot/DecodeAARot)
	// And converted to quaternion (two trigonometry functions)

	ConstVectorSegment pos() const { return this->template segment<3>(0); }
	ConstVectorSegment vel() const { return this->template segment<3>(3); }
	ConstVectorSegment acc() const { return this->template segment<3>(6); }
	ConstVectorSegment rDel() const { return this->template segment<3>(9); }
	ConstVectorSegment rVel() const { return this->template segment<3>(12); }
	ConstVectorSegment rAcc() const { return this->template segment<3>(15); }
	VectorSegment pos() { return this->template segment<3>(0); }
	VectorSegment vel() { return this->template segment<3>(3); }
	VectorSegment acc() { return this->template segment<3>(6); }
	VectorSegment rDel() { return this->template segment<3>(9); }
	VectorSegment rVel() { return this->template segment<3>(12); }
	VectorSegment rAcc() { return this->template segment<3>(15); }

	Eigen::Quaternion<T> referenceQuat;
	void reset()
	{
		referenceQuat = quat();
		rDel().setZero();
	}

	const Eigen::Quaternion<T> getRDelQuat() const { return MRP2Quat(rDel()); }
	void setRDelQuat(const Eigen::Quaternion<T> &quat) { rDel() = Quat2MRP(quat); }
	const Eigen::Quaternion<T> getRVelQuat() const { return MRP2Quat(rVel()); }
	void setRVelQuat(const Eigen::Quaternion<T> &quat) { rVel() = Quat2MRP(quat); }
	const Eigen::Quaternion<T> getRAccQuat() const { return MRP2Quat(rAcc()); }
	void setRAccQuat(const Eigen::Quaternion<T> &quat) { rAcc() = Quat2MRP(quat); }

	Eigen::Quaternion<T> quat()
	{
		return referenceQuat * getRDelQuat();
	}
};

/**
 * Model of the behaviour of a 6DOF object (position with acceleration-model + rotation with acceleration-model implemented as MEKF)
 */
template<typename T = float>
class SystemModel6DOF_MEKF : public Kalman::LinearizedSystemModel<State6DOF_MEKF<T>, TimeControl<T>>
{
public:
	typedef State6DOF_MEKF<T> S;
	typedef TimeControl<T> C;

	SystemModel6DOF_MEKF()
	{
		// P Covariance (noise) of system process f
		this->P.setZero();
		// Set up later depending on situation

		// W = df/dw (w.r.t. noise)
		// E.g. which state variables are stronger affected by noise
		this->W.setIdentity();

		// F = df/dx
		// Each parameter w.r.t. itself
		this->F.setIdentity();
		// Rest dynamic
	}

	/**
	 * Scale expected error from 0 to maximally humanly possible (assuming initialised state)
	 */
	void setExpectedErrorScale(float expectedErrorPos, float expectedErrorRot)
	{
		// Values for a jab in boxing (Source: Kimm and Thiel, Hand Speed Measurements in Boxing)
		// Around 20g / 200m/s^2 peak acceleration within <40ms 
		const float maxAccChange = 500000; // Estimate cm/s^3
		// Currently estimated as 10-20 times faster to change in ° vs cm
		const float maxAccRotChange = 10000000; // TODO: Estimate radians/s^3

		if (expectedErrorPos > 0)
		{ // Scale from lesser to extreme normal operation
			float accError = expectedErrorPos * maxAccChange;
			this->P.template block<3,3>(6,6).diagonal().setConstant(accError*accError);
		}

		if (expectedErrorRot > 0)
		{ // Scale from lesser to extreme normal operation
			float accError = expectedErrorRot * maxAccRotChange;
			this->P.template block<3,3>(15,15).diagonal().setConstant(accError*accError);
		}
	}

	/**
	 * Scale expected error from 0 to maximally humanly possible (assuming initialised state)
	 */
	void setExpectedError(float expectedErrorPos, Eigen::Matrix<T,3,1> expectedErrorRotAcc)
	{
		// Values for a jab in boxing (Source: Kimm and Thiel, Hand Speed Measurements in Boxing)
		// Around 20g / 200m/s^2 peak acceleration within <40ms 
		const float maxAccChange = 500000; // Estimate cm/s^3
		// Currently estimated as 10-20 times faster to change in ° vs cm
		const float maxAccRotChange = 10000000; // TODO: Estimate radians/s^3

		if (expectedErrorPos > 0)
		{ // Scale from lesser to extreme normal operation
			float accError = expectedErrorPos * maxAccChange;
			this->P.template block<3,3>(6,6).diagonal().setConstant(accError*accError);
		}
		this->P.template block<3,3>(15,15).diagonal() = expectedErrorRotAcc;
	}

	S f(const S& x, const C& u) const
	{
		S nx;
		nx.referenceQuat = x.referenceQuat;

		nx.acc() = x.acc();
		nx.vel() = x.vel() + x.acc() * u.dt();
		nx.pos() = x.pos() + x.vel() * u.dt() + x.acc() * u.dt()*u.dt()/2;

		nx.rVel() = x.rVel();
		nx.setRDelQuat(x.getRVelQuat() * x.getRAccQuat());
		/*nx.rAcc() = x.rAcc();
		nx.setRVelQuat(x.getRVelQuat() * x.getRAccQuat());

		Eigen::Quaternion<T> halfAcc = x.getRAccQuat();
		halfAcc.vec() /= 2;
		halfAcc.w() = 1 - ((1-halfAcc.w()) / 2);
		halfAcc.normalise();
		nx.setRDelQuat(x.getRDelQuat() * x.getRVelQuat() * halfAcc);*/
		return nx;
	}

	void updateJacobians(const S& x, const C& u)
	{
		// F = df/dx
		// Each parameter w.r.t. itself
		//this->F.setIdentity();
		// Vel w.r.t. Acc
		this->F.template block<3,3>(3,6).diagonal().setConstant(u.dt());
		// Pos w.r.t. Vel
		this->F.template block<3,3>(0,3).diagonal().setConstant(u.dt());
		// Pos w.r.t. Acc
		this->F.template block<3,3>(0,6).diagonal().setConstant(u.dt()*u.dt()/2);

		/* THEORY NOTES for
		nx.setRVelQuat(x.getRVelQuat() * x.getRAccQuat());

		// Conversion from MRP to Quat for i = 1,2
		ui = (ri, si, ti)
		ai = ri*ri + si*si + ti*ti;
		wi = (16 - ai) / (16 + ai);
		vi = ui * 8 / (16 + ai)
		vi = (xi, yi, zi)
		// Derivation helpers
		mi = 16+ai
		mi2 = (16+ai)^2
		ai/ui = 2ui
		// Derivation for conversion
		wi/ui = ui * -64/mi2
		vi/ui = - 8(2*ui^2-mi)/mi2
		vi/ui (diff) = - 16(vi*ui)/m2
		// Quaternion product, derivation easy
		//v = w1 * v2 + w2*v1 - v1.cross(v2);
		x = w1*x2 + x1*w2 - y1*z2 + z1*y2;
		y = w1*y2 + y1*w2 - z1*x2 + x1*z2;
		z = w1*z2 + z1*w2 - x1*y2 + y1*x2;
		w = w1*w2 - x1*x2 - y1*y2 - z1*z2;
		// Convert quaternion product to MRP
		q = v * 4/(1+w);
		*/

		// Conversion from MRP to Quat, intermediate values
		Eigen::Matrix<T,3,1> u1 = x.rVel(), u2 = x.rAcc();
		T a1 = u1.squaredNorm(), a2 = u2.sqaredNorm();
		T m1 = 16+a1, m2 = 16+a2;
		T m12 = m1*m1, m22 = m2*m2;
		T f1 = 8/m12, f2 = 8/m22;
		T h1 = 8/m1, h2 = 8/m2;
		Eigen::Vector<T,4,1> q1 ((16-a1)/m1, u1.x()*h1, u1.y()*h1, u1.z()*h1);
		Eigen::Vector<T,4,1> q2 ((16-a2)/m2, u2.x()*h2, u2.y()*h2, u2.z()*h2);

		// Conversion from MRP to Quat for source 1, derivative
		Eigen::Matrix<T,4,3> q1d;
		q1d(0,0) = -(2*u1.x()*u1.x() - m1) * f1;
		q1d(1,1) = -(2*u1.y()*u1.y() - m1) * f1;
		q1d(2,2) = -(2*u1.z()*u1.z() - m1) * f1;
		q1d(0,1) = -2*(u1.x()*u1.y()) * f1;
		q1d(0,2) = -2*(u1.x()*u1.z()) * f1;
		q1d(1,2) = -2*(u1.y()*u1.z()) * f1;
		q1d(1,0) = q1d(0,1);
		q1d(2,0) = q1d(0,2);
		q1d(2,1) = q1d(1,2);
		q1d.rows(3) = u1 * -64*f1;

		// Conversion from MRP to Quat for source 2, derivative
		Eigen::Matrix<T,4,3> q2d;
		q2d(0,0) = -(2*u2.x()*u2.x() - m2) * f2;
		q2d(1,1) = -(2*u2.y()*u2.y() - m2) * f2;
		q2d(2,2) = -(2*u2.z()*u2.z() - m2) * f2;
		q2d(0,1) = -2*(u2.x()*u2.y()) * f2;
		q2d(0,2) = -2*(u2.x()*u2.z()) * f2;
		q2d(1,2) = -2*(u2.y()*u2.z()) * f2;
		q2d(1,0) = q2d(0,1);
		q2d(2,0) = q2d(0,2);
		q2d(2,1) = q2d(1,2);
		q2d.rows(3) = u2 * -64*f2;

		// Quaternion product
		Eigen::Vector<T,4,1> q;
		q.x() = q1.w()*q2.x() + q1.x()*q2.w() - q1.y()*q2.z() + q1.z()*q2.y();
		q.y() = q1.w()*q2.y() + q1.y()*q2.w() - q1.z()*q2.x() + q1.x()*q2.z();
		q.z() = q1.w()*q2.z() + q1.z()*q2.w() - q1.x()*q2.y() + q1.y()*q2.x();
		q.w() = q1.w()*q2.w() - q1.x()*q2.x() - q1.y()*q2.y() - q1.z()*q2.z();

		// Quaternion product, derivative for source 1
		Eigen::Matrix<T,4,3> qd1;
		qd1.row(0) = q1d.row(3)*q2(0) + q1d.row(0)*q2(3) - q1d.row(1)*q2(2) + q1d.row(2)*q2(1);
		qd1.row(1) = q1d.row(3)*q2(1) + q1d.row(1)*q2(3) - q1d.row(2)*q2(0) + q1d.row(0)*q2(2);
		qd1.row(2) = q1d.row(3)*q2(2) + q1d.row(2)*q2(3) - q1d.row(0)*q2(1) + q1d.row(1)*q2(0);
		qd1.row(3) = q1d.row(3)*q2(3) + q1d.row(0)*q2(0) - q1d.row(1)*q2(1) + q1d.row(2)*q2(2);

		// Quaternion product, derivative for source 2
		Eigen::Matrix<T,4,3> qd2;
		qd2.row(0) = q1(3)*q2d.row(0) + q1(0)*q2d.row(3) - q1(1)*q2d.row(2) + q1(2)*q2d.row(1);
		qd2.row(1) = q1(3)*q2d.row(1) + q1(1)*q2d.row(3) - q1(2)*q2d.row(0) + q1(0)*q2d.row(2);
		qd2.row(2) = q1(3)*q2d.row(2) + q1(2)*q2d.row(3) - q1(0)*q2d.row(1) + q1(1)*q2d.row(0);
		qd2.row(3) = q1(3)*q2d.row(3) - q1(0)*q2d.row(0) - q1(1)*q2d.row(1) - q1(2)*q2d.row(2);


		T wI = 4 / ((1+w)*(1+w));

		// Conversion back to MRP, derivative for source 1
		Eigen::Matrix<T,3,3> mrp1;
		mrp1.row(0) = ((w+1) * qd1.row(0) - q(0) * qd1.row(3)) * wI;
		mrp1.row(1) = ((w+1) * qd1.row(1) - q(1) * qd1.row(3)) * wI;
		mrp1.row(2) = ((w+1) * qd1.row(2) - q(2) * qd1.row(3)) * wI;

		// Conversion back to MRP, derivative for source 2
		Eigen::Matrix<T,3,3> mrp2;
		mrp2.row(0) = ((w+1) * qd2.row(0) - q(0) * qd2.row(3)) * wI;
		mrp2.row(1) = ((w+1) * qd2.row(1) - q(1) * qd2.row(3)) * wI;
		mrp2.row(2) = ((w+1) * qd2.row(2) - q(2) * qd2.row(3)) * wI;


		/*
		T w1 = (16-a1)/m1, w2 = (16-a2)/m2;
		T x1 = u1.x()*h1, x2 = u2.x()*8/m2;
		T y1 = u1.y()*h1, y2 = u2.y()*8/m2;
		T z1 = u1.z()*h1, z2 = u2.z()*8/m2;
		*/

		/*
		Eigen::Vector<T,4,1> q;
		q.x() = w1*x2 + x1*w2 - y1*z2 + z1*y2;
		q.y() = w1*y2 + y1*w2 - z1*x2 + x1*z2;
		q.z() = w1*z2 + z1*w2 - x1*y2 + y1*x2;
		q.w() = w1*w2 - x1*x2 - y1*y2 - z1*z2;

		Eigen::Matrix<T,4,3> qd1;
		qd1.row(0) = q1d.row(3)*x2 + q1d.row(0)*w2 - q1d.row(1)*z2 + q1d.row(2)*y2;
		qd1.row(1) = q1d.row(3)*y2 + q1d.row(1)*w2 - q1d.row(2)*x2 + q1d.row(0)*z2;
		qd1.row(2) = q1d.row(3)*z2 + q1d.row(2)*w2 - q1d.row(0)*y2 + q1d.row(1)*x2;
		qd1.row(3) = q1d.row(3)*w2 + q1d.row(0)*x2 - q1d.row(1)*y2 + q1d.row(2)*z2;

		Eigen::Matrix<T,4,3> qd2;
		qd2.row(0) = w1*q2d.row(0) + x1*q2d.row(3) - y1*q2d.row(2) + z1*q2d.row(1);
		qd2.row(1) = w1*q2d.row(1) + y1*q2d.row(3) - z1*q2d.row(0) + x1*q2d.row(2);
		qd2.row(2) = w1*q2d.row(2) + z1*q2d.row(3) - x1*q2d.row(1) + y1*q2d.row(0);
		qd2.row(3) = w1*q2d.row(3) - x1*q2d.row(0) - y1*q2d.row(1) - z1*q2d.row(2);
		*/

		/*Eigen::Matrix<T,4,3> qd1;
		for (int i = 0; i < 3; i++)
		{
			qd1(0,i) = q1d(3,i)*x2 + q1d(0,i)*w2 - q1d(1,i)*z2 + q1d(2,i)*y2;
			qd1(1,i) = q1d(3,i)*y2 + q1d(1,i)*w2 - q1d(2,i)*x2 + q1d(0,i)*z2;
			qd1(2,i) = q1d(3,i)*z2 + q1d(2,i)*w2 - q1d(0,i)*y2 + q1d(1,i)*x2;
			qd1(3,i) = q1d(3,i)*w2 + q1d(0,i)*x2 - q1d(1,i)*y2 + q1d(2,i)*z2;
		}

		Eigen::Matrix<T,4,3> qd2;
		for (int i = 0; i < 3; i++)
		{
			qd2(0,i) = w1*q2d(0,i) + x1*q2d(3,i) - y1*q2d(2,i) + z1*q2d(1,i);
			qd2(1,i) = w1*q2d(1,i) + y1*q2d(3,i) - z1*q2d(0,i) + x1*q2d(2,i);
			qd2(2,i) = w1*q2d(2,i) + z1*q2d(3,i) - x1*q2d(1,i) + y1*q2d(0,i);
			qd2(3,i) = w1*q2d(3,i) - x1*q2d(0,i) - y1*q2d(1,i) - z1*q2d(2,i);
		}*/

		// RotVel w.r.t. RotAcc
		this->F.template block<3,3>(12,15).diagonal().setConstant(1);
		// Rot w.r.t. RotVel
		this->F.template block<3,3>(9,12).diagonal().setConstant(1);
		// Rot w.r.t. RotAcc
		this->F.template block<3,3>(9,15).diagonal().setConstant(1/2);

/*		LOG(LTrackingFilter, LTrace, "pos       vel      acc\n");
		int r = 0;
		LOG(LTrackingFilter, LTrace, "%.1f %.1f %.1f %.1f %.1f %.1f %.1f %.1f %.1f\n", this->F(r,0), this->F(r,1), this->F(r,2), this->F(r,3), this->F(r,4), this->F(r,5), this->F(r,6), this->F(r,7), this->F(r,8));
		r = 1;
		LOG(LTrackingFilter, LTrace, "%.1f %.1f %.1f %.1f %.1f %.1f %.1f %.1f %.1f\n", this->F(r,0), this->F(r,1), this->F(r,2), this->F(r,3), this->F(r,4), this->F(r,5), this->F(r,6), this->F(r,7), this->F(r,8));
		r = 2;
		LOG(LTrackingFilter, LTrace, "%.1f %.1f %.1f %.1f %.1f %.1f %.1f %.1f %.1f\n", this->F(r,0), this->F(r,1), this->F(r,2), this->F(r,3), this->F(r,4), this->F(r,5), this->F(r,6), this->F(r,7), this->F(r,8));
		r = 3;
		LOG(LTrackingFilter, LTrace, "%.1f %.1f %.1f %.1f %.1f %.1f %.1f %.1f %.1f\n", this->F(r,0), this->F(r,1), this->F(r,2), this->F(r,3), this->F(r,4), this->F(r,5), this->F(r,6), this->F(r,7), this->F(r,8));
		r = 4;
		LOG(LTrackingFilter, LTrace, "%.1f %.1f %.1f %.1f %.1f %.1f %.1f %.1f %.1f\n", this->F(r,0), this->F(r,1), this->F(r,2), this->F(r,3), this->F(r,4), this->F(r,5), this->F(r,6), this->F(r,7), this->F(r,8));
		r = 5;
		LOG(LTrackingFilter, LTrace, "%.1f %.1f %.1f %.1f %.1f %.1f %.1f %.1f %.1f\n", this->F(r,0), this->F(r,1), this->F(r,2), this->F(r,3), this->F(r,4), this->F(r,5), this->F(r,6), this->F(r,7), this->F(r,8));
		r = 6;
		LOG(LTrackingFilter, LTrace, "%.1f %.1f %.1f %.1f %.1f %.1f %.1f %.1f %.1f\n", this->F(r,0), this->F(r,1), this->F(r,2), this->F(r,3), this->F(r,4), this->F(r,5), this->F(r,6), this->F(r,7), this->F(r,8));
		r = 7;
		LOG(LTrackingFilter, LTrace, "%.1f %.1f %.1f %.1f %.1f %.1f %.1f %.1f %.1f\n", this->F(r,0), this->F(r,1), this->F(r,2), this->F(r,3), this->F(r,4), this->F(r,5), this->F(r,6), this->F(r,7), this->F(r,8));
		r = 8;
		LOG(LTrackingFilter, LTrace, "%.1f %.1f %.1f %.1f %.1f %.1f %.1f %.1f %.1f\n", this->F(r,0), this->F(r,1), this->F(r,2), this->F(r,3), this->F(r,4), this->F(r,5), this->F(r,6), this->F(r,7), this->F(r,8));*/
	}
};

/**
 * Model of the observed pose of a 6DOF object (position with acceleration-model + rotation with acceleration-model implemented as MEKF)
 */
template<typename T>
class MeasurementModel6DOF_MEKF : public Kalman::LinearizedMeasurementModel<State6DOF_MEKF<T>, Kalman::Vector<T, 6>>
{
public:
	typedef State6DOF_MEKF<T> S;
	typedef Kalman::Vector<T, 6> M;

	MeasurementModel6DOF_MEKF()
	{
		// P Covariance (noise) of measurement process h
		this->P.setZero();
		// Set on demand

		// H = dh/dx
		this->H.setZero();
		this->H.template block<3,3>(0,0).diagonal().setConstant(1);
		this->H.template block<3,3>(3,9).diagonal().setConstant(1);
		// V = dh/dw (w.r.t. noise)
		this->V.setIdentity();
	}

	M h(const S& x) const
	{
		M measurement;
		measurement.template segment<3>(0) = x.pos();
		measurement.template segment<3>(3) = x.rDel();
		return measurement;
	}

	/**
	 */
	void setExpectedError(float expectedErrorPos, float expectedErrorRot)
	{
		this->P.template block<3,3>(0,0).diagonal().setConstant(expectedErrorPos*expectedErrorPos); // Noise in cm
		this->P.template block<3,3>(3,3).diagonal().setConstant(expectedErrorRot*expectedErrorRot); // Noise in radians
	}
};

#endif

#endif // KALMAN_H