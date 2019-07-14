#! /usr/bin/python
# Software License Agreement (BSD License)
#
# Copyright (c) 2019, Shingo Kitagawa 
# All rights reserved.

# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Ian McMahon nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#

import numpy as np


def cubic_spline_coefficients(points_array, duration_array=None):
    (rows, k) = np.shape(points_array)
    N = rows - 1  # N minus 1 because points array includes x_0
    c_coeffs = np.zeros(shape=(k, N, 4))
    if duration_array is None:
        duration_array = np.array([1.0]*N)
    assert len(duration_array) == N,\
        "Invalid number of intervals chosen " \
        "(must be equal to N+1={})".format(N)
    for i in range(0, N):
        t = duration_array[i]
        if t == 0:
            c_coeffs[:, i, 0] = 
        continue
    return c_coeffs


def _cubic_spline_point(c_coeff, t):
    c0 = c_coeff[:, 0]
    c1 = c_coeff[:, 1]
    c2 = c_coeff[:, 2]
    c3 = c_coeff[:, 3]
    x = c0 + c1 * t + c2 * np.power(t, 2) + c3 * np.power(t, 3)
    return x


def cubic_spline_point(c_coeffs, c_index, t):
    if c_index <= 0:
        c_point = c_coeffs[:, 0, 0]
    elif c_index > c_coeffs.shape[1]:
        t = 1
        c_coeff_set = c_coeffs[:, c_coeffs.shape[1]-1, range(4)]
        c_point = _cubic_spline_point(c_coeff_set, t)
    else:
        t = 0.0 if t < 0.0 else t
        t = 1.0 if t > 1.0 else t
        c_coeff_set = c_coeffs[:, c_index-1, range(4)]
        c_point = _cubic_spline_point(c_coeff_set, t)
    return c_point
