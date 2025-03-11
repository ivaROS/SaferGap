# The MIT License (MIT)
#
# Copyright 2020 Barbara Barros Carlos, Tommaso Sartor
#
# This file is part of crazyflie_nmpc.
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.


from acados_template import *
from casadi import SX, vertcat, sin, cos, if_else

def export_unicycle_keyhole_mpc_model() -> AcadosModel:

    model_name = 'unicycle_keyhole_mpc'

    # states (f_exp)
    xq = SX.sym('xq')
    yq = SX.sym('yq')
    thetaq = SX.sym('thetaq')
    x1 = SX.sym('x1')
    x2 = SX.sym('x2')
    x = vertcat(xq, yq, thetaq, x1, x2)

    # controls
    v = SX.sym('v')
    w = SX.sym('w')
    vdot = SX.sym('vdot')
    wdot = SX.sym('wdot')
    u = vertcat(v, w, vdot, wdot) # motor speed

    # for f_impl
    xq_dot = SX.sym('xq_dot')
    yq_dot = SX.sym('yq_dot')
    thetaq_dot = SX.sym('thetaq_dot')
    x1_dot = SX.sym('x1_dot')
    x2_dot = SX.sym('x2_dot')
    xdot = vertcat(xq_dot, yq_dot, thetaq_dot, x1_dot, x2_dot)

    # Model equations
    dxq = v * cos(thetaq)
    dyq = v * sin(thetaq)
    dthetaq = w
    dx1 = vdot
    dx2 = wdot

    # Explicit and Implicit functions
    f_expl = vertcat(dxq, dyq, dthetaq, dx1, dx2)
    f_impl = xdot - f_expl

    # parameters
    c1_x = SX.sym('c1_x')
    c1_y = SX.sym('c1_y')
    d1 = SX.sym('d1')
    c2_x = SX.sym('c2_x')
    c2_y = SX.sym('c2_y')
    d2 = SX.sym('d2')
    c3_x = SX.sym('c3_x')
    c3_y = SX.sym('c3_y')
    d3 = SX.sym('d3')
    c4_x = SX.sym('c4_x')
    c4_y = SX.sym('c4_y')
    d4 = SX.sym('d4')
    c5_x = SX.sym('c5_x')
    c5_y = SX.sym('c5_y')
    d5 = SX.sym('d5')

    xc_x = SX.sym('xc_x')
    xc_y = SX.sym('xc_y')
    r = SX.sym('r')

    alpha1 = SX.sym('alpha1')
    alpha2 = SX.sym('alpha2')
    alpha3 = SX.sym('alpha3')
    alpha4 = SX.sym('alpha4')
    alpha5 = SX.sym('alpha5')
    alpha6 = SX.sym('alpha6')
    alpha7 = SX.sym('alpha7')
    alpha8 = SX.sym('alpha8')
    alpha9 = SX.sym('alpha9')
    alpha10 = SX.sym('alpha10')
    alpha11 = SX.sym('alpha11')
    alpha12 = SX.sym('alpha12')
    alpha13 = SX.sym('alpha13')
    alpha14 = SX.sym('alpha14')
    alpha15 = SX.sym('alpha15')
    b = SX.sym('b')
    
    p = vertcat(c1_x, c1_y, d1, c2_x, c2_y, d2, c3_x, c3_y, d3, c4_x, c4_y, d4, c5_x, c5_y, d5, xc_x, xc_y, r, 
                alpha1, alpha2, alpha3, alpha4, alpha5, alpha6, alpha7, alpha8, alpha9, alpha10, alpha11, alpha12, alpha13, alpha14, alpha15, b)

    # Constraints
    val1 = c1_x * xq + c2_y * yq + d1
    relu1 = if_else(val1 < 0, 0, val1)
    val2 = c2_x * xq + c2_y * yq + d2
    relu2 = if_else(val2 < 0, 0, val2)
    val3 = c3_x * xq + c3_y * yq + d3
    relu3 = if_else(val3 < 0, 0, val3)
    val4 = c4_x * xq + c4_y * yq + d4
    relu4 = if_else(val4 < 0, 0, val4)
    val5 = c5_x * xq + c5_y * yq + d5
    relu5 = if_else(val5 < 0, 0, val5)

    circ_dist = (xc_x - xq) ** 2 + (xc_y - yq) ** 2
    circ_val = -(circ_dist - r ** 2)
    circ = if_else(circ_val < 0., 0., circ_val)

    keyhole_const = alpha1 * relu1 + alpha2 * relu2 + alpha3 * relu3 + alpha4 * relu1 * relu2 + \
                    alpha5 * relu1 * relu2 * relu3 + alpha6 * relu1 * relu4 * relu5 + alpha7 * relu2 * relu4 * relu5 + \
                    alpha8 * circ * relu1 * relu4 + alpha9 * circ * relu2 * relu4 + \
                    alpha10 * circ * relu1 + alpha11 * circ * relu2 + alpha12 * circ * relu3 + \
                    alpha13 * circ * relu1 * relu2 + alpha14 * circ * relu1 * relu2 * relu3 + \
                    alpha15 * circ + b

    # algebraic variables
    z = []

    # dynamics
    model = AcadosModel()

    model.f_impl_expr = f_impl
    model.f_expl_expr = f_expl
    model.con_h_expr = keyhole_const
    model.con_h_expr_0 = keyhole_const
    model.con_h_expr_e = keyhole_const
    model.x = x
    model.xdot = xdot
    model.u = u
    model.z = z
    model.p = p
    model.name = model_name

    return model
