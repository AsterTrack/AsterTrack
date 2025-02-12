// The MIT License (MIT)
//
// Copyright (c) 2015 Markus Herb
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.
#ifndef KALMAN_EXTENDEDKALMANFILTER_HPP_
#define KALMAN_EXTENDEDKALMANFILTER_HPP_

#include "KalmanFilterBase.hpp"
#include "StandardFilterBase.hpp"
#include "LinearizedSystemModel.hpp"
#include "LinearizedMeasurementModel.hpp"

#include "util/util.hpp" // Log
#include "util/eigenutil.hpp" // Log
namespace Kalman {
    
    /**
     * @brief Extended Kalman Filter (EKF)
     * 
     * This implementation is based upon [An Introduction to the Kalman Filter](https://www.cs.unc.edu/~welch/media/pdf/kalman_intro.pdf)
     * by Greg Welch and Gary Bishop.
     *
     * @param StateType The vector-type of the system state (usually some type derived from Kalman::Vector)
     */
    template<class StateType>
    class ExtendedKalmanFilter : public KalmanFilterBase<StateType>,
                                 public StandardFilterBase<StateType>
    {
    public:
        //! Kalman Filter base type
        typedef KalmanFilterBase<StateType> KalmanBase;
        //! Standard Filter base type
        typedef StandardFilterBase<StateType> StandardBase;
        
        //! Numeric Scalar Type inherited from base
        using typename KalmanBase::T;
        
        //! State Type inherited from base
        using typename KalmanBase::State;
        
        //! Linearized Measurement Model Type
        template<class Measurement, template<class> class CovarianceBase>
        using MeasurementModelType = LinearizedMeasurementModel<State, Measurement, CovarianceBase>;
        
        //! Linearized System Model Type
        template<class Control, template<class> class CovarianceBase>
        using SystemModelType = LinearizedSystemModel<State, Control, CovarianceBase>;
        
    protected:
        //! Kalman Gain Matrix Type
        template<class Measurement>
        using KalmanGain = Kalman::KalmanGain<State, Measurement>;
        
    protected:
        //! State Estimate
        using KalmanBase::x;
        //! State Covariance Matrix
        using StandardBase::P;
        
    public:
        /**
         * @brief Constructor
         */
        ExtendedKalmanFilter()
        {
            // Setup state and covariance
            P.setIdentity();
        }
        
        /**
         * @brief Perform filter prediction step using system model and no control input (i.e. \f$ u = 0 \f$)
         *
         * @param [in] s The System model
         * @return The updated state estimate
         */
        template<class Control, template<class> class CovarianceBase>
        const State& predict( SystemModelType<Control, CovarianceBase>& s )
        {
            // predict state (without control)
            Control u;
            u.setZero();
            return predict( s, u );
        }
        
        /**
         * @brief Perform filter prediction step using control input \f$u\f$ and corresponding system model
         *
         * @param [in] s The System model
         * @param [in] u The Control input vector
         * @return The updated state estimate
         */
        template<class Control, template<class> class CovarianceBase>
        const State& predict( SystemModelType<Control, CovarianceBase>& s, const Control& u )
        {
            s.updateJacobians( x, u );
            
            // predict state
            x = s.f(x, u);
            
/*            Eigen::Matrix<T,3,1> rotStdDev = P.template block<3,3>(9,9).diagonal();
            Eigen::Matrix<T,3,1> posStdDev = P.template block<3,3>(0,0).diagonal();*/
            //PRINT("P %dx%d: \n%s", P.rows(), P.cols(), printMatrix(P));

            // predict covariance
//            auto corr = ( s.W * s.getCovariance() * s.W.transpose() ).eval();
            P  = ( s.F * P * s.F.transpose() ) + ( s.W * s.getCovariance() * s.W.transpose() );

            //PRINT("C %dx%d: \n%s", corr.rows(), corr.cols(), printMatrix(corr));
            //PRINT("F %dx%d: \n%s", s.F.rows(), s.F.cols(), printMatrix(s.F));
            //PRINT("P = F*P*F^T + C %dx%d: \n%s", P.rows(), P.cols(), printMatrix(P));

/*            Eigen::Matrix<T,3,1> rotStdDevCorr = corr.template block<3,3>(9,9).diagonal();
            Eigen::Matrix<T,3,1> rotStdDevCorred = P.template block<3,3>(9,9).diagonal();
            PRINT("Propagating rot variance from (%.3f, %.3f, %.3f) by (%.3f, %.3f, %.3f) to (%.3f, %.3f, %.3f)!", rotStdDev.x(), rotStdDev.y(), rotStdDev.z(), rotStdDevCorr.x(), rotStdDevCorr.y(), rotStdDevCorr.z(), rotStdDevCorred.x(), rotStdDevCorred.y(), rotStdDevCorred.z());


            Eigen::Matrix<T,3,1> posStdDevCorr = corr.template block<3,3>(0,0).diagonal();
            Eigen::Matrix<T,3,1> posStdDevCorred = P.template block<3,3>(0,0).diagonal();
            PRINT("Propagating pos variance from (%.3f, %.3f, %.3f) by (%.3f, %.3f, %.3f) to (%.3f, %.3f, %.3f)!", posStdDev.x(), posStdDev.y(), posStdDev.z(), posStdDevCorr.x(), posStdDevCorr.y(), posStdDevCorr.z(), posStdDevCorred.x(), posStdDevCorred.y(), posStdDevCorred.z());*/
            
            // return state prediction
            return this->getState();
        }
        
        /**
         * @brief Perform filter update step using measurement \f$z\f$ and corresponding measurement model
         *
         * @param [in] m The Measurement model
         * @param [in] z The measurement vector
         * @return The updated state estimate
         */
        template<class Measurement, template<class> class CovarianceBase>
        const State& update( MeasurementModelType<Measurement, CovarianceBase>& m, const Measurement& z )
        {
            m.updateJacobians( x );
            
            // COMPUTE KALMAN GAIN
            // compute innovation covariance
            Covariance<Measurement> S = ( m.H * P * m.H.transpose() ) + ( m.V * m.getCovariance() * m.V.transpose() );
            
            // compute kalman gain
            KalmanGain<Measurement> K = P * m.H.transpose() * S.inverse();
            
            // UPDATE STATE ESTIMATE AND COVARIANCE
            // Update state using computed kalman gain and innovation
            x += K * ( z - m.h( x ) );

            // Update covariance
            /*auto d = (m.H * P).eval();
            auto corr = (K * d).eval();

            Eigen::Matrix<T,3,1> rotStdDev = P.template block<3,3>(9,9).diagonal();

            Eigen::Matrix<T,3,1> posStdDev = P.template block<3,3>(0,0).diagonal();*/
            
            //PRINT("K %dx%d: \n%s", K.rows(), K.cols(), printMatrix(K));
            //PRINT("H %dx%d: \n%s", m.H.rows(), m.H.cols(), printMatrix(m.H));
            //PRINT("P %dx%d: \n%s", P.rows(), P.cols(), printMatrix(P));*/

            P -= K * m.H * P;

            //PRINT("H * P %dx%d: \n%s", d.rows(), d.cols(), printMatrix(d));
            //PRINT("K * H * P %dx%d: \n%s", corr.rows(), corr.cols(), printMatrix(corr));
            //PRINT("P -= K * H * P %dx%d: \n%s", P.rows(), P.cols(), printMatrix(P));

            /*Eigen::Matrix<T,3,1> rotStdDevCorr = corr.template block<3,3>(9,9).diagonal();
            Eigen::Matrix<T,3,1> rotStdDevCorred = P.template block<3,3>(9,9).diagonal();
            Eigen::Matrix<T,3,1> rotStdDevD = d.template block<3,3>(3,9).diagonal();
            Eigen::Matrix<T,3,1> rotStdDevGain = K.template block<3,3>(9,3).diagonal();
            PRINT("Correcting rot variance from (%.3f, %.3f, %.3f) by (%.3f, %.3f, %.3f) to (%.3f, %.3f, %.3f) with measurement cov (%.3f, %.3f, %.3f) (size %dx%d) and gain (%.3f, %.3f, %.3f) (size %dx%d!", rotStdDev.x(), rotStdDev.y(), rotStdDev.z(), rotStdDevCorr.x(), rotStdDevCorr.y(), rotStdDevCorr.z(), rotStdDevCorred.x(), rotStdDevCorred.y(), rotStdDevCorred.z(), rotStdDevD.x(), rotStdDevD.y(), rotStdDevD.z(), d.rows(), d.cols(), rotStdDevGain.x(), rotStdDevGain.y(), rotStdDevGain.z(), K.rows(), K.cols());

            Eigen::Matrix<T,3,1> posStdDevCorr = corr.template block<3,3>(0,0).diagonal();
            Eigen::Matrix<T,3,1> posStdDevCorred = P.template block<3,3>(0,0).diagonal();
            Eigen::Matrix<T,3,1> posStdDevD = d.template block<3,3>(0,0).diagonal();
            Eigen::Matrix<T,3,1> posStdDevGain = K.template block<3,3>(0,0).diagonal();
            PRINT("Correcting pos variance from (%.3f, %.3f, %.3f) by (%.3f, %.3f, %.3f) to (%.3f, %.3f, %.3f) with measurement cov (%.3f, %.3f, %.3f) (size %dx%d) and gain (%.3f, %.3f, %.3f) (size %dx%d!", posStdDev.x(), posStdDev.y(), posStdDev.z(), posStdDevCorr.x(), posStdDevCorr.y(), posStdDevCorr.z(), posStdDevCorred.x(), posStdDevCorred.y(), posStdDevCorred.z(), posStdDevD.x(), posStdDevD.y(), posStdDevD.z(), d.rows(), d.cols(), posStdDevGain.x(), posStdDevGain.y(), posStdDevGain.z(), K.rows(), K.cols());*/

            // return updated state estimate
            return this->getState();
        }
    };
}

#endif
