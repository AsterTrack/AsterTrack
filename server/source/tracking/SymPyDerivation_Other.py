from sympy import *

def dep(r, x):
    reps = {}
    for v, e in r:
        if e.xreplace(reps).has(x):
            reps[v] = Function(str(v))(x)
    return reps

def differentiate_with_cse(expr, x, backsub=False):
    r, e = cse(expr)
    reps = dep(r, x)
    print(reps)
    dr = []
    for v, s in r:
        if v in reps:
            f = reps[v]
            df = Derivative(f, x).doit()
            ds = s.xreplace(reps).diff(x)
            dr.append((df, ds))
    dexpr = e[0].subs(reps).diff(x).subs(dr).expand()
    for k, v in reversed(reps.items()):
        dexpr = dexpr.subs(v, k)
    return dexpr.subs(list(reversed(r))) if backsub else dexpr,dict(r)

incRot_c = symbols('aX aY aZ')
incRot = Matrix(3, 1, incRot_c)
theta = sqrt((incRot.T @ incRot)[0,0])
incQuat = Quaternion.from_axis_angle(incRot/theta, theta*2)

baseQuat_c = symbols('qX qY qZ qW')
baseQuat = Quaternion(*baseQuat_c)
poseQuat = incQuat * baseQuat

posePos = MatrixSymbol('posePos', 3, 1).as_mutable()
camMat = MatrixSymbol('camMat', 4, 4).as_mutable()
tgtPt = MatrixSymbol('tgtPt', 3, 1).as_mutable()

scnPt = Matrix(3, 1, Quaternion.rotate_point(tgtPt, poseQuat)) + posePos
tmpPt = camMat * Matrix(4, 1, [ scnPt[0], scnPt[1], scnPt[2], 1 ])
camPt = Matrix(3, 1, [ tmpPt[0], tmpPt[1], tmpPt[2] ]) / tmpPt[3]

derivation = differentiate_with_cse(camPt, incRot_c[0])
#print(derivation)