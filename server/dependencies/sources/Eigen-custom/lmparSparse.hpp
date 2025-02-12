// IWYU pragma: private
#include "unsupported/Eigen/src/NonLinearOptimization/InternalHeaderCheck.h"

#include <Eigen/SparseCore>

#include "qrsolvSparse.hpp"

namespace Eigen { 

namespace internal {

template <typename Scalar, typename QRFAC>
void lmpar2Sparse(
        const QRFAC &qr,
        const Matrix< Scalar, Dynamic, 1 >  &diag,
        const Matrix< Scalar, Dynamic, 1 >  &qtb,
        Scalar delta,
        Scalar &par,
        Matrix< Scalar, Dynamic, 1 >  &x)

{
    using std::sqrt;
    using std::abs;
    typedef DenseIndex Index;

    /* Local variables */
    Index j;
    Scalar fp;
    Scalar parc, parl;
    Index iter;
    Scalar temp, paru;
    Scalar gnorm;
    Scalar dxnorm;


    /* Function Body */
    const Scalar dwarf = (std::numeric_limits<Scalar>::min)();
    const Index n = qr.matrixR().cols();
    eigen_assert(n==diag.size());
    eigen_assert(n==qtb.size());

    Matrix< Scalar, Dynamic, 1 >  wa1, wa2;

    /* compute and store in x the gauss-newton direction. if the */
    /* jacobian is rank-deficient, obtain a least squares solution. */

//    const Index rank = qr.nonzeroPivots(); // exactly double(0.)
    const Index rank = qr.rank(); // use a threshold
    wa1 = qtb;
    wa1.tail(n-rank).setZero();
    wa1.head(rank) = qr.matrixR().topLeftCorner(rank, rank).template triangularView<Upper>().solve(wa1.head(rank));

    x = qr.colsPermutation()*wa1;

    /* initialize the iteration counter. */
    /* evaluate the function at the origin, and test */
    /* for acceptance of the gauss-newton direction. */
    iter = 0;
    wa2 = diag.cwiseProduct(x);
    dxnorm = wa2.blueNorm();
    fp = dxnorm - delta;
    if (fp <= Scalar(0.1) * delta) {
        par = 0;
        return;
    }

    /* if the jacobian is not rank deficient, the newton */
    /* step provides a lower bound, parl, for the zero of */
    /* the function. otherwise set this bound to zero. */
    parl = 0.;
    if (rank==n) {
        wa1 = qr.colsPermutation().inverse() *  diag.cwiseProduct(wa2)/dxnorm;
        qr.matrixR().topLeftCorner(n, n).transpose().template triangularView<Lower>().solveInPlace(wa1);
        temp = wa1.blueNorm();
        parl = fp / delta / temp / temp;
    }

    /* calculate an upper bound, paru, for the zero of the function. */
    for (j = 0; j < n; ++j)
        wa1[j] = qr.matrixR().col(j).head(j+1).dot(qtb.head(j+1)) / diag[qr.colsPermutation().indices()(j)];

    gnorm = wa1.stableNorm();
    paru = gnorm / delta;
    if (paru == 0.)
        paru = dwarf / (std::min)(delta,Scalar(0.1));

    /* if the input par lies outside of the interval (parl,paru), */
    /* set par to the closer endpoint. */
    par = (std::max)(par,parl);
    par = (std::min)(par,paru);
    if (par == 0.)
        par = gnorm / dxnorm;

    /* beginning of an iteration. */
    int c = qr.matrixR().cols();
    SparseMatrix< Scalar, RowMajor > sS = qr.matrixR().topLeftCorner(c,c);
    while (true) {
        ++iter;

        /* evaluate the function at the current value of par. */
        if (par == 0.)
            par = (std::max)(dwarf,Scalar(.001) * paru); /* Computing MAX */
        wa1 = sqrt(par)* diag;

        auto sC = sS;
        qrsolvSparse<Scalar>(sC, qr.colsPermutation().indices(), wa1, qtb, x);
        // Only wa1 is changing in magnitude (due to par) from iteration to iteration

        wa2 = diag.cwiseProduct(x);
        dxnorm = wa2.blueNorm();
        temp = fp;
        fp = dxnorm - delta;

        /* if the function is small enough, accept the current value */
        /* of par. also test for the exceptional cases where parl */
        /* is zero or the number of iterations has reached 10. */
        if (abs(fp) <= Scalar(0.1) * delta || (parl == 0. && fp <= temp && temp < 0.) || iter == 10)
            break;

        /* compute the newton correction. */
        wa1 = qr.colsPermutation().inverse() * diag.cwiseProduct(wa2/dxnorm);
        // Original minpack code suggest we should be able to use this here, but doesn't work
        //sC.template triangularView<Upper>().solveInPlace(wa1);
        for (j = 0; j < n; ++j) {
            typename Eigen::SparseMatrix<Scalar, RowMajor>::InnerIterator it(sC, j);
            wa1[j] /= it.value(); // Diagonal
            ++it; // Skip diagonal
            for (; it; ++it)
                wa1[it.col()] -= it.value() * wa1[j];
        }
        temp = wa1.blueNorm();
        parc = fp / delta / temp / temp;

        /* depending on the sign of the function, update parl or paru. */
        if (fp > 0.)
            parl = (std::max)(parl,par);
        if (fp < 0.)
            paru = (std::min)(paru,par);

        /* compute an improved estimate for par. */
        par = (std::max)(parl,par+parc);
    }
    if (iter == 0)
        par = 0.;
    return;
}

} // end namespace internal

} // end namespace Eigen
