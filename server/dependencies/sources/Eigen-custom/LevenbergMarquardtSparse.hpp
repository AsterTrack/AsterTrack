// -*- coding: utf-8
// vim: set fileencoding=utf-8

// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// Copyright (C) 2009 Thomas Capricelli <orzel@freehackers.org>
//
// This Source Code Form is subject to the terms of the Mozilla
// Public License v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at http://mozilla.org/MPL/2.0/.

#ifndef EIGEN_LEVENBERGMARQUARDT_SPARSE__H
#define EIGEN_LEVENBERGMARQUARDT_SPARSE__H

// IWYU pragma: private
#include "unsupported/Eigen/src/NonLinearOptimization/InternalHeaderCheck.h"

#include <Eigen/SparseCore>
#include <Eigen/SparseQR>

#include "lmparSparse.hpp"

#include <chrono>
#include <stop_token>

namespace Eigen { 

/**
  * \ingroup NonLinearOptimization_Module
  * \brief Performs non linear optimization over a non-linear function,
  * using a variant of the Levenberg Marquardt algorithm.
  *
  * Check wikipedia for more information.
  * http://en.wikipedia.org/wiki/Levenberg%E2%80%93Marquardt_algorithm
  */
template<typename FunctorType, typename Scalar=double>
class LevenbergMarquardtSparse
{
    static Scalar sqrt_epsilon()
    {
      using std::sqrt;
      return sqrt(NumTraits<Scalar>::epsilon());
    }
    
public:
    LevenbergMarquardtSparse(FunctorType &_functor)
        : functor(_functor) { nfev = njev = iter = 0;  fnorm = gnorm = 0.; useExternalScaling=false; }

    typedef DenseIndex Index;
    
    struct Parameters {
        Parameters()
            : factor(Scalar(100.))
            , maxfev(400)
            , ftol(sqrt_epsilon())
            , xtol(sqrt_epsilon())
            , gtol(Scalar(0.))
            , epsfcn(Scalar(0.)) {}
        Scalar factor;
        Index maxfev;   // maximum number of function evaluation
        Scalar ftol;
        Scalar xtol;
        Scalar gtol;
        Scalar epsfcn;
    };

    typedef Matrix< Scalar, Dynamic, 1 > FVectorType;
    //typedef typename FunctorType::JacobianType JacobianType;
    typedef SparseMatrix< Scalar > JacobianType;

    LevenbergMarquardtSpace::Status lmder1(
            FVectorType &x,
            const Scalar tol = sqrt_epsilon()
            );

    LevenbergMarquardtSpace::Status minimize(FVectorType &x);
    LevenbergMarquardtSpace::Status minimizeInit(FVectorType &x);
    LevenbergMarquardtSpace::Status minimizeOneStep(FVectorType &x);

    static LevenbergMarquardtSpace::Status lmdif1(
            FunctorType &functor,
            FVectorType &x,
            Index *nfev,
            const Scalar tol = sqrt_epsilon()
            );

    void resetParameters(void) { parameters = Parameters(); }

    Parameters parameters;
    FVectorType  fvec, qtf, diag;
    JacobianType fjac;
    PermutationMatrix<Dynamic,Dynamic> permutation;
    Index nfev;
    Index njev;
    Index iter;
    Scalar fnorm, gnorm;
    bool useExternalScaling; 

    std::stop_token stopToken;

    Scalar lm_param(void) { return par; }
private:
    
    FunctorType &functor;
    Index n;
    Index m;
    FVectorType wa1, wa2, wa3, wa4;

    Scalar par, sum;
    Scalar temp, temp1, temp2;
    Scalar delta;
    Scalar ratio;
    Scalar pnorm, xnorm, fnorm1, actred, dirder, prered;

    LevenbergMarquardtSparse& operator=(const LevenbergMarquardtSparse&);
};

template<typename FunctorType, typename Scalar>
LevenbergMarquardtSpace::Status
LevenbergMarquardtSparse<FunctorType,Scalar>::lmder1(
        FVectorType  &x,
        const Scalar tol
        )
{
    n = x.size();
    m = functor.values();

    /* check the input parameters for errors. */
    if (n <= 0 || m < n || tol < 0.)
        return LevenbergMarquardtSpace::ImproperInputParameters;

    resetParameters();
    parameters.ftol = tol;
    parameters.xtol = tol;
    parameters.maxfev = 100*(n+1);

    return minimize(x);
}


template<typename FunctorType, typename Scalar>
LevenbergMarquardtSpace::Status
LevenbergMarquardtSparse<FunctorType,Scalar>::minimize(FVectorType  &x)
{
    LevenbergMarquardtSpace::Status status = minimizeInit(x);
    if (status==LevenbergMarquardtSpace::ImproperInputParameters)
        return status;
    do {
        status = minimizeOneStep(x);
    } while (status==LevenbergMarquardtSpace::Running);
    return status;
}

template<typename FunctorType, typename Scalar>
LevenbergMarquardtSpace::Status
LevenbergMarquardtSparse<FunctorType,Scalar>::minimizeInit(FVectorType  &x)
{
    n = x.size();
    m = functor.values();

    wa1.resize(n); wa2.resize(n); wa3.resize(n);
    wa4.resize(m);
    fvec.resize(m);
    fjac.resize(m, n);
    if (!useExternalScaling)
        diag.resize(n);
    eigen_assert( (!useExternalScaling || diag.size()==n) && "When useExternalScaling is set, the caller must provide a valid 'diag'");
    qtf.resize(n);

    /* Function Body */
    nfev = 0;
    njev = 0;

    /*     check the input parameters for errors. */
    if (n <= 0 || m < n || parameters.ftol < 0. || parameters.xtol < 0. || parameters.gtol < 0. || parameters.maxfev <= 0 || parameters.factor <= 0.)
        return LevenbergMarquardtSpace::ImproperInputParameters;

    if (useExternalScaling)
        for (Index j = 0; j < n; ++j)
            if (diag[j] <= 0.)
                return LevenbergMarquardtSpace::ImproperInputParameters;

    /*     evaluate the function at the starting point */
    /*     and calculate its norm. */
    nfev = 1;
    if ( functor(x, fvec) < 0)
        return LevenbergMarquardtSpace::UserAsked;
    fnorm = fvec.stableNorm();

    /*     initialize levenberg-marquardt parameter and iteration counter. */
    par = 0.;
    iter = 1;

    return LevenbergMarquardtSpace::NotStarted;
}

template<typename Derived>
static inline Matrix<typename Derived::RealScalar, Derived::ColsAtCompileTime, 1>
colwise_blueNorm_impl(const EigenBase<Derived>& _vec)
{
  typedef typename Derived::RealScalar RealScalar;
  typedef Matrix< typename Derived::RealScalar, Derived::ColsAtCompileTime, 1> RealVec;
  using std::pow;
  using std::sqrt;
  using std::abs;

  // This program calculates the machine-dependent constants
  // bl, b2, slm, s2m, relerr overfl
  // from the "basic" machine-dependent numbers
  // nbig, ibeta, it, iemin, iemax, rbig.
  // The following define the basic machine-dependent constants.
  // For portability, the PORT subprograms "ilmaeh" and "rlmach"
  // are used. For any specific computer, each of the assignment
  // statements can be replaced
  static const int ibeta = std::numeric_limits<RealScalar>::radix;  // base for floating-point numbers
  static const int it    = NumTraits<RealScalar>::digits();  // number of base-beta digits in mantissa
  static const int iemin = NumTraits<RealScalar>::min_exponent();  // minimum exponent
  static const int iemax = NumTraits<RealScalar>::max_exponent();  // maximum exponent
  static const RealScalar rbig   = NumTraits<RealScalar>::highest();  // largest floating-point number
  static const RealScalar b1     = RealScalar(pow(RealScalar(ibeta),RealScalar(-((1-iemin)/2))));  // lower boundary of midrange
  static const RealScalar b2     = RealScalar(pow(RealScalar(ibeta),RealScalar((iemax + 1 - it)/2)));  // upper boundary of midrange
  static const RealScalar s1m    = RealScalar(pow(RealScalar(ibeta),RealScalar((2-iemin)/2)));  // scaling factor for lower range
  static const RealScalar s2m    = RealScalar(pow(RealScalar(ibeta),RealScalar(- ((iemax+it)/2))));  // scaling factor for upper range
  static const RealScalar eps    = RealScalar(pow(double(ibeta), 1-it));
  static const RealScalar relerr = sqrt(eps);  // tolerance for neglecting asml

  const Derived& vec(_vec.derived());
  Index n = vec.size();
  RealScalar ab2 = b2 / RealScalar(n);
  RealVec asml = RealVec::Zero(vec.cols());
  RealVec amed = RealVec::Zero(vec.cols());
  RealVec abig = RealVec::Zero(vec.cols());

  for(Index j=0; j<vec.outerSize(); ++j)
  {
    for(typename Derived::InnerIterator iter(vec, j); iter; ++iter)
    {
      RealScalar ax = abs(iter.value());
      if(ax > ab2)     abig(iter.col()) += numext::abs2(ax*s2m);
      else if(ax < b1) asml(iter.col()) += numext::abs2(ax*s1m);
      else             amed(iter.col()) += numext::abs2(ax);
    }
  }

  RealVec aout = RealVec(vec.cols());
  for(Index j=0; j<vec.cols(); ++j)
  {
    if(amed(j)!=amed(j))
    {
        aout(j) = amed(j);  // we got a NaN
        continue;
    }
    if(abig(j) > RealScalar(0))
    {
      abig(j) = sqrt(abig(j));
      if(abig(j) > rbig) // overflow, or *this contains INF values
      {
        aout(j) = abig(j);  // return INF
        continue;
      }
      if(amed(j) > RealScalar(0))
      {
        abig(j) = abig(j)/s2m;
        amed(j) = sqrt(amed(j));
      }
      else
      {
        aout(j) = abig(j)/s2m;
        continue;
      }
    }
    else if(asml(j) > RealScalar(0))
    {
      if (amed(j) > RealScalar(0))
      {
        abig(j) = sqrt(amed(j));
        amed(j) = sqrt(asml(j)) / s1m;
      }
      else
      {
        aout(j) = sqrt(asml(j))/s1m;
        continue;
      }
    }
    else
    {
      aout(j) = sqrt(amed(j));
      continue;
    }
    asml(j) = numext::mini(abig(j), amed(j));
    abig(j) = numext::maxi(abig(j), amed(j));
    if(asml(j) <= abig(j)*relerr)
      aout(j) = abig(j);
    else
      aout(j) = abig(j) * sqrt(RealScalar(1) + numext::abs2(asml(j)/abig(j)));
  }
  return aout;
}

template<typename FunctorType, typename Scalar>
LevenbergMarquardtSpace::Status
LevenbergMarquardtSparse<FunctorType,Scalar>::minimizeOneStep(FVectorType  &x)
{
    using std::abs;
    using std::sqrt;

    eigen_assert(x.size()==n); // check the caller is not cheating us

    /* calculate the jacobian matrix. */
    Index df_ret = functor.dfs(x, fjac);
    if (df_ret<0)
        return LevenbergMarquardtSpace::UserAsked;
    if (df_ret>0)
        // numerical diff, we evaluated the function df_ret times
        nfev += df_ret;
    else njev++;

    if (stopToken.stop_requested())
        return LevenbergMarquardtSpace::UserAsked;

    /* compute the qr factorization of the jacobian. */
    wa2 = colwise_blueNorm_impl(fjac);

    if (stopToken.stop_requested())
        return LevenbergMarquardtSpace::UserAsked;

    typedef Eigen::SparseQR<JacobianType, Eigen::COLAMDOrdering<int>> QRFAC;
    QRFAC qrfac(fjac);
    if (stopToken.stop_requested())
        return LevenbergMarquardtSpace::UserAsked;
    fjac = qrfac.matrixR();
    permutation = qrfac.colsPermutation();

    /* on the first iteration and if external scaling is not used, scale according */
    /* to the norms of the columns of the initial jacobian. */
    if (iter == 1) {
        if (!useExternalScaling)
            for (Index j = 0; j < n; ++j)
                diag[j] = (wa2[j]==0.)? 1. : wa2[j];

        /* on the first iteration, calculate the norm of the scaled x */
        /* and initialize the step bound delta. */
        xnorm = diag.cwiseProduct(x).stableNorm();
        delta = parameters.factor * xnorm;
        if (delta == 0.)
            delta = parameters.factor;
    }

    /* form (q transpose)*fvec and store the first n components in */
    /* qtf. */
    wa4 = qrfac.matrixQ().adjoint() * fvec;
    qtf = wa4.head(n);

    /* compute the norm of the scaled gradient. */
    gnorm = 0.;
    if (fnorm != 0.)
        for (Index j = 0; j < n; ++j)
            if (wa2[permutation.indices()[j]] != 0.)
                gnorm = (std::max)(gnorm, abs( fjac.col(j).head(j+1).dot(qtf.head(j+1)/fnorm) / wa2[permutation.indices()[j]]));

    /* test for convergence of the gradient norm. */
    if (gnorm <= parameters.gtol)
        return LevenbergMarquardtSpace::CosinusTooSmall;

    /* rescale if necessary. */
    if (!useExternalScaling)
        diag = diag.cwiseMax(wa2);

    do {

        if (stopToken.stop_requested())
            return LevenbergMarquardtSpace::UserAsked;

        /* determine the levenberg-marquardt parameter. */
        internal::lmpar2Sparse<Scalar, QRFAC>(qrfac, diag, qtf, delta, par, wa1);

        if (stopToken.stop_requested())
            return LevenbergMarquardtSpace::UserAsked;

        /* store the direction p and x + p. calculate the norm of p. */
        wa1 = -wa1;
        wa2 = x + wa1;
        pnorm = diag.cwiseProduct(wa1).stableNorm();

        /* on the first iteration, adjust the initial step bound. */
        if (iter == 1)
            delta = (std::min)(delta,pnorm);

        /* evaluate the function at x + p and calculate its norm. */
        if ( functor(wa2, wa4) < 0)
            return LevenbergMarquardtSpace::UserAsked;
        ++nfev;
        fnorm1 = wa4.stableNorm();

        /* compute the scaled actual reduction. */
        actred = -1.;
        if (Scalar(.1) * fnorm1 < fnorm)
            actred = 1. - numext::abs2(fnorm1 / fnorm);

        /* compute the scaled predicted reduction and */
        /* the scaled directional derivative. */
        wa3 = fjac.template triangularView<Upper>() * (qrfac.colsPermutation().inverse() *wa1);
        temp1 = numext::abs2(wa3.stableNorm() / fnorm);
        temp2 = numext::abs2(sqrt(par) * pnorm / fnorm);
        prered = temp1 + temp2 / Scalar(.5);
        dirder = -(temp1 + temp2);

        /* compute the ratio of the actual to the predicted */
        /* reduction. */
        ratio = 0.;
        if (prered != 0.)
            ratio = actred / prered;

        /* update the step bound. */
        if (ratio <= Scalar(.25)) {
            if (actred >= 0.)
                temp = Scalar(.5);
            if (actred < 0.)
                temp = Scalar(.5) * dirder / (dirder + Scalar(.5) * actred);
            if (Scalar(.1) * fnorm1 >= fnorm || temp < Scalar(.1))
                temp = Scalar(.1);
            /* Computing MIN */
            delta = temp * (std::min)(delta, pnorm / Scalar(.1));
            par /= temp;
        } else if (!(par != 0. && ratio < Scalar(.75))) {
            delta = pnorm / Scalar(.5);
            par = Scalar(.5) * par;
        }

        /* test for successful iteration. */
        if (ratio >= Scalar(1e-4)) {
            /* successful iteration. update x, fvec, and their norms. */
            x = wa2;
            wa2 = diag.cwiseProduct(x);
            fvec = wa4;
            xnorm = wa2.stableNorm();
            fnorm = fnorm1;
            ++iter;
        }

        /* tests for convergence. */
        if (abs(actred) <= parameters.ftol && prered <= parameters.ftol && Scalar(.5) * ratio <= 1. && delta <= parameters.xtol * xnorm)
            return LevenbergMarquardtSpace::RelativeErrorAndReductionTooSmall;
        if (abs(actred) <= parameters.ftol && prered <= parameters.ftol && Scalar(.5) * ratio <= 1.)
            return LevenbergMarquardtSpace::RelativeReductionTooSmall;
        if (delta <= parameters.xtol * xnorm)
            return LevenbergMarquardtSpace::RelativeErrorTooSmall;

        /* tests for termination and stringent tolerances. */
        if (nfev >= parameters.maxfev)
            return LevenbergMarquardtSpace::TooManyFunctionEvaluation;
        if (abs(actred) <= NumTraits<Scalar>::epsilon() && prered <= NumTraits<Scalar>::epsilon() && Scalar(.5) * ratio <= 1.)
            return LevenbergMarquardtSpace::FtolTooSmall;
        if (delta <= NumTraits<Scalar>::epsilon() * xnorm)
            return LevenbergMarquardtSpace::XtolTooSmall;
        if (gnorm <= NumTraits<Scalar>::epsilon())
            return LevenbergMarquardtSpace::GtolTooSmall;

    } while (ratio < Scalar(1e-4));

    return LevenbergMarquardtSpace::Running;
}

template<typename FunctorType, typename Scalar>
LevenbergMarquardtSpace::Status
LevenbergMarquardtSparse<FunctorType,Scalar>::lmdif1(
        FunctorType &functor,
        FVectorType  &x,
        Index *nfev,
        const Scalar tol
        )
{
    Index n = x.size();
    Index m = functor.values();

    /* check the input parameters for errors. */
    if (n <= 0 || m < n || tol < 0.)
        return LevenbergMarquardtSpace::ImproperInputParameters;

    NumericalDiff<FunctorType> numDiff(functor);
    // embedded LevenbergMarquardtSparse
    LevenbergMarquardtSparse<NumericalDiff<FunctorType>, Scalar > lm(numDiff);
    lm.parameters.ftol = tol;
    lm.parameters.xtol = tol;
    lm.parameters.maxfev = 200*(n+1);

    LevenbergMarquardtSpace::Status info = LevenbergMarquardtSpace::Status(lm.minimize(x));
    if (nfev)
        * nfev = lm.nfev;
    return info;
}

} // end namespace Eigen

#endif // EIGEN_LEVENBERGMARQUARDT_SPARSE__H

//vim: ai ts=4 sts=4 et sw=4
