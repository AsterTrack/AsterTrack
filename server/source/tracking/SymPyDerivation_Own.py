from sympy import *
import re
from sympy.matrices.expressions.matexpr import MatrixElement

def find_derivatives(expression):
    derivatives = []
    if isinstance(expression, Derivative):
        #print(expression)
        derivatives.append(expression)
    elif isinstance(expression, Basic):
        for a in expression.args:
            derivatives += find_derivatives(a)
    elif isinstance(expression, MatrixBase):
        for i in range(expression.rows):
            for j in range(expression.cols):
                derivatives += find_derivatives(expression[i, j])
    return derivatives

def derive_recursively(expression_list, derive_done, derive_todo):
    newly_derived = {}
    simple_derived = {}
    for s, e in derive_todo.items():
        print("Handling derivatives in " + str(s))
        derivatives = find_derivatives(e)
        for d in derivatives:
            if d in simple_derived:
                print("Found derivative " + str(d) + " in simple list, already handled!")
                e = e.subs(d, simple_derived[d])
                derive_todo[s] = e
                continue
            if d in newly_derived:
                #print("Found derivative " + str(d) + " in done list, already handled!")
                continue
            if d in derive_todo:
                #print("Found derivative " + str(d) + " in todo list, already handling!")
                continue
            if d in expression_list:
                #print("Found derivative " + str(d) + " in past list, already handled!")
                continue
            if d.expr in expression_list:
                expression = expression_list[d.expr]
                print("  Deriving " + str(d.expr) + " w.r.t. " + str(d.variables))
                print("    Expression: " + str(expression))
                derivative = Derivative(expression, *d.variable_count).simplify().doit().simplify()
                print("    Derivative: " + str(derivative))
                if isinstance(derivative, AtomicExpr) or isinstance(derivative, Symbol) or isinstance(derivative, MatrixElement):
                    e = e.subs(d, derivative)
                    derive_todo[s] = e
                    simple_derived[s] = derivative
                    print("        Replacing main expression with: " + str(e))
                    continue
                newly_derived[d] = derivative
                continue
            print("Did NOT find base expression " + str(d.expr) + " in provided expression list!")
    derive_done = derive_todo | derive_done
    if len(newly_derived) == 0:
        return derive_done
    return derive_recursively(expression_list, derive_done, newly_derived)

incRot_c = symbols('aX aY aZ')
incRot_s = Matrix(3, 1, incRot_c)/2
theta_s = Function("theta")(*incRot_c)
theta_e = sqrt((incRot_s.T @ incRot_s)[0,0])
incQuat_c = [ Function(f"iQ{i}")(*incRot_c) for i in "WXYZ" ]
incQuat_s = Quaternion(*incQuat_c)
incQuat_e = Quaternion(cos(theta_s), *(sin(theta_s)/theta_s*incRot_s))
baseQuat_c = symbols('bQW bQX bQY bQZ')
baseQuat_s = Quaternion(*baseQuat_c)
poseQuat_c = [ Function(f"pQ{i}")(*incRot_c) for i in "WXYZ" ]
poseQuat_s = Quaternion(*poseQuat_c)
# Could also do it like this and in expressions just refer poseQuat_s to poseQuat_e, but output is less readable
#poseQuat_s = Function(f"pq")(*incRot_c, *baseQuat_c)
poseQuat_e = incQuat_s * baseQuat_s

posePos_c = symbols('posX posY posZ')
posePos = Matrix(3, 1, posePos_c)
#posePos = MatrixSymbol('posePos', 3, 1).as_explicit()
#camMat_c = symbols('cm:16')
#camMat = Matrix(4, 4, camMat_c)
camMat = MatrixSymbol('cm', 4, 4).as_explicit()
tgtPt_c = symbols('tgtX tgtY tgtZ')
tgtPt = Matrix(3, 1, tgtPt_c)
#tgtPt = MatrixSymbol('tgtPt', 3, 1).as_explicit()
scnPt_c = [ Function(f"s{i}")(*posePos_c, *incRot_c) for i in "XYZ" ]
scnPt_s = Matrix(3, 1, scnPt_c)
scnPt_e = Matrix(3, 1, Quaternion.rotate_point(tgtPt, poseQuat_s)) + posePos
tmpPt_c = [ Function(f"t{i}")(*posePos_c, *incRot_c) for i in "XYZW" ]
tmpPt_s = Matrix(4, 1, tmpPt_c)
tmpPt_e = camMat * Matrix(4, 1, [ scnPt_s[0], scnPt_s[1], scnPt_s[2], 1 ])
camPt_c = [ Function(f"c{i}")(*posePos_c, *incRot_c) for i in "XY" ]
camPt_s = Matrix(2, 1, camPt_c)
camPt_e = Matrix(2, 1, [ tmpPt_s[0], tmpPt_s[1] ]) / tmpPt_s[3]

expressions = \
    { camPt_c[i]: camPt_e[i] for i in range(2) } | \
    { tmpPt_c[i]: tmpPt_e[i] for i in range(4) } | \
    { scnPt_c[i]: scnPt_e[i] for i in range(3) } | \
    { theta_s: theta_e } | \
    { incQuat_c[i]: incQuat_e.to_Matrix()[i] for i in range(4) } | \
    { poseQuat_c[i]: poseQuat_e.to_Matrix()[i] for i in range(4) }

derivatives = derive_recursively(expressions, {}, { 
    symbols('dPX'): diff(camPt_s, posePos_c[0]),
    symbols('dPY'): diff(camPt_s, posePos_c[1]),
    symbols('dPZ'): diff(camPt_s, posePos_c[2]),
    symbols('dAX'): diff(camPt_s, incRot_c[0]),
    symbols('dAY'): diff(camPt_s, incRot_c[1]),
    symbols('dAZ'): diff(camPt_s, incRot_c[2])
})

derStr = str(derivatives).replace("(aX, aY, aZ)", "").replace("(posX, posY, posZ, aX, aY, aZ)", "")
derStr = re.sub(r"Derivative\((\w+),\s?(\w+)\)", r"d_\1_\2", derStr)
derStr = derStr.replace(", d_", ";\ndouble d_").replace(": ", " = ")
derStr = re.sub(r"cm\[([0-9]),\s?([0-9])\]", r"cm(\1, \2)", derStr)
derStr = derStr.replace("cos(theta)", "ct")
derStr = derStr.replace("sin(theta)", "st")
derStr = re.sub(r"/([a-zA-z_]+)\b\*\*2", r"/(\1*\1)", derStr)
derStr = re.sub(r"\b([a-zA-z_]+)\b\*\*2", r"\1*\1", derStr)
print(derStr)

#for s,d in derivatives.items():
#    print(str(s) + " : " + str(type(d)))

#elements = cse([ Eq(s, d) for s,d in derivatives.items() ])
#print(elements)

#elements = cse([ Eq(s, d) for s,d in derivatives.items() if len(str(d)) > 1000 ])