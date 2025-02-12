// IWYU pragma: private
#include "unsupported/Eigen/src/NonLinearOptimization/InternalHeaderCheck.h"

#include <Eigen/SparseCore>

namespace Eigen { 

namespace internal {

/**
 * Original qrsolv description in minpack:
 * 
 * 
 * 
!  given an m by n matrix a, an n by n diagonal matrix d,
!  and an m-vector b, the problem is to determine an x which
!  solves the system
!```
!        a*x = b ,     d*x = 0 ,
!```
!  in the least squares sense.
!
!  this subroutine completes the solution of the problem
!  if it is provided with the necessary information from the
!  qr factorization, with column pivoting, of a. that is, if
!  a*p = q*r, where p is a permutation matrix, q has orthogonal
!  columns, and r is an upper triangular matrix with diagonal
!  elements of nonincreasing magnitude, then qrsolv expects
!  the full upper triangle of r, the permutation matrix p,
!  and the first n components of (q transpose)*b. the system
!  a*x = b, d*x = 0, is then equivalent to
!```
!               t       t
!        r*z = q *b ,  p *d*p*z = 0 ,
!```
!  where x = p*z. if this system does not have full rank,
!  then a least squares solution is obtained. on output qrsolv
!  also provides an upper triangular matrix s such that
!```
!         t   t               t
!        p *(a *a + d*d)*p = s *s .
!```
!  s is computed within qrsolv and may be of separate interest.
 * 
 * 
 * 
 * In other words:
 * A custom Sparse QR solver that solves
 *     a*x = b, d*x = 0
 * reformulated as 
 *     r*z = q^t*b ,  p^t*d*p*z = 0 ,
 * where x = p*z is output
 * Put in terms of the code / parameters:
 *     r*wa = qtb, p^t * diag * p * wa
 *     x = p*wa
 * 
 * In contrast to the original dense qesolv, where s held the original r in the upper triangular,
 * which was unmodified, and held s in the strict lower triangular plus it's diagonal in sdiag,
 * in the sparse version s is the input r, and modified to be output s, since they share the same sparse structure
 * However, it MUST be a upper triangular matrix, the strict lower triangular must be empty (no non-zero elements)
 */
template <typename Scalar>
void qrsolvSparse(
        SparseMatrix< Scalar, RowMajor > &s,
        const VectorXi &ipvt,
        const Matrix< Scalar, Dynamic, 1 >  &diag,
        const Matrix< Scalar, Dynamic, 1 >  &qtb,
        Matrix< Scalar, Dynamic, 1 >  &x)

{
    typedef DenseIndex Index;

    /* Local variables */
    Index i, j, k, l;
    Scalar temp;
    Index n = s.cols();
    Matrix< Scalar, Dynamic, 1 > wa = qtb;
    Matrix< Scalar, Dynamic, 1 > tdiag(n);
    JacobiRotation<Scalar> givens;

    /* Function Body */

    /*     eliminate the diagonal matrix d using a givens rotation. */
    for (j = 0; j < n; ++j) {

        /*        prepare the row of d to be eliminated, locating the */
        /*        diagonal element using p from the qr factorization. */
        l = ipvt[j];
        if (diag[l] == 0.)
            break;
        tdiag.tail(n-j).setZero();
        tdiag[j] = diag[l];

        /*        the transformations to eliminate the row of d */
        /*        modify only a single element of (q transpose)*b */
        /*        beyond the first n, which is initially zero. */
        Scalar qtbpj = 0.;
        for (k = j; k < n; ++k) {
            Scalar &sdiag = s.coeffRef(k,k);

            /*           determine a givens rotation which eliminates the */
            /*           appropriate element in the current row of d. */
            givens.makeGivens(-sdiag, tdiag[k]);

            /*           compute the modified diagonal element of r and */
            /*           the modified element of ((q transpose)*b,0). */
            sdiag = givens.c() * sdiag + givens.s() * tdiag[k];
            temp = givens.c() * wa[k] + givens.s() * qtbpj;
            qtbpj = -givens.s() * wa[k] + givens.c() * qtbpj;
            wa[k] = temp;

            /*           accumulate the transformation in the row of s. */
            typename Eigen::SparseMatrix<Scalar, RowMajor>::InnerIterator it(s,k);
            ++it; // Skip diagonal
            for (; it; ++it) {
                temp = givens.c() * it.value() + givens.s() * tdiag[it.col()];
                tdiag[it.col()] = -givens.s() * it.value() + givens.c() * tdiag[it.col()];
                it.valueRef() = temp;
            }
        }
    }

    /*     solve the triangular system for z. if the system is */
    /*     singular, then obtain a least squares solution. */
    Index nsing;
    for(nsing=0; nsing<n && tdiag[nsing]!=0; nsing++) {}

    wa.tail(n-nsing).setZero();
    wa.head(nsing) = s.topLeftCorner(nsing, nsing).template triangularView<Upper>().solve(wa.head(nsing));

    /*     permute the components of z back to components of x. */
    for (j = 0; j < n; ++j) x[ipvt[j]] = wa[j];
}

} // end namespace internal

} // end namespace Eigen
