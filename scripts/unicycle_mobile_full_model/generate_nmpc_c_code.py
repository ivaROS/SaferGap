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
import acados_template as at
from unicycle_keyhole_mpc_model import *
import numpy as np
import scipy.linalg
import math
from ctypes import *
from os.path import dirname, join, abspath

ACADOS_PATH = "/home/shiyu/Documents/SDK/acados"

# create render arguments
ra = AcadosOcp()

# export model
model = export_unicycle_keyhole_mpc_model()

Tf = 0.2
N = 6
nx = model.x.size()[0]
nu = model.u.size()[0]
ny = nx + nu
ny_e = nx
n_param = model.p.size()[0]

# set ocp_nlp_dimensions
nlp_dims     = ra.dims
nlp_dims.nx  = nx
nlp_dims.ny  = ny
nlp_dims.ny_e = ny_e
nlp_dims.nbx = 0
nlp_dims.nbu = nu
nlp_dims.nbx_e = 0
nlp_dims.nu  = nu
nlp_dims.N   = N
nlp_dims.np = n_param

# set weighting matrices
nlp_cost = ra.cost
Q = np.eye(nx)
Q[0,0] = 10      # x
Q[1,1] = 10      # y
Q[2,2] = 0       # theta
Q[3,3] = 0
Q[4,4] = 0

R = np.eye(nu)
R[0,0] = 1    # v
R[1,1] = 1    # w
R[2,2] = 0    # vdot
R[3,3] = 0    # wdot

nlp_cost.W = scipy.linalg.block_diag(Q, R)

Vx = np.zeros((ny, nx))
Vx[0,0] = 1.0
Vx[1,1] = 1.0
Vx[2,2] = 1.0
nlp_cost.Vx = Vx

Vu = np.zeros((ny, nu))
Vu[5,0] = 1.0
Vu[6,1] = 1.0
nlp_cost.Vu = Vu

nlp_cost.W_e = 5*Q

Vx_e = np.zeros((ny_e, nx))
Vx_e[0,0] = 1.0
Vx_e[1,1] = 1.0
Vx_e[2,2] = 1.0
nlp_cost.Vx_e = Vx_e

nlp_cost.yref   = np.array([0, 0, 0, 0, 0, 0, 0, 0, 0])
nlp_cost.yref_e = np.array([0, 0, 0, 0, 0])

nlp_con = ra.constraints

nlp_con.lbu = np.array([0, -4, -10, -1.78])
nlp_con.ubu = np.array([0.5, 4, 10, 1.78])
nlp_con.lh = np.array([0])
nlp_con.lh_0 = np.array([0])
nlp_con.lh_e = np.array([0])
nlp_con.uh = np.array([1e15])
nlp_con.uh_0 = np.array([1e15])
nlp_con.uh_e = np.array([1e15])
nlp_con.x0  = np.array([0, 0, 0, 0, 0])
nlp_con.idxbu = np.array([0, 1, 2, 3])

ra.parameter_values = np.zeros((n_param, 1))

## set QP solver
#ra.solver_options.qp_solver = 'FULL_CONDENSING_QPOASES'
ra.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM'
ra.solver_options.hessian_approx = 'GAUSS_NEWTON'
ra.solver_options.integrator_type = 'ERK'

# set prediction horizon
ra.solver_options.tf = Tf
ra.solver_options.nlp_solver_type = 'SQP_RTI'
#ra.solver_options.nlp_solver_type = 'SQP'

# set header path
ra.acados_include_path  = f'{ACADOS_PATH}/include'
ra.acados_lib_path      = f'{ACADOS_PATH}/lib'

ra.model = model

acados_solver = AcadosOcpSolver(ra, json_file = 'acados_ocp.json')

print('>> NMPC exported')
