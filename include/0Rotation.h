//
// Created by ale on 19-11-25.
//

// Ceres Solver - A fast non-linear least squares minimizer
// Copyright 2015 Google Inc. All rights reserved.
// http://ceres-solver.org/
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
// * Neither the name of Google Inc. nor the names of its contributors may be
//   used to endorse or promote products derived from this software without
//   specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// Author: keir@google.com (Keir Mierle)
//         sameeragarwal@google.com (Sameer Agarwal)
//
// Templated functions for manipulating rotations. The templated
// functions are useful when implementing functors for automatic
// differentiation.


#ifndef MY3DPHOTO_0ROTATION_H
#define MY3DPHOTO_0ROTATION_H

#include <algorithm>
#include <cmath>
#include <limits>

namespace m3d
{
    //mainly from ceres library -> rotation.h.

    template<typename T, int row_stride, int col_stride>
    struct MatrixAdapter {
        T* pointer_;
        explicit MatrixAdapter(T* pointer)
                : pointer_(pointer)
        {}

        T& operator()(int r, int c) const {
            return pointer_[r * row_stride + c * col_stride];
        }
    };

    template <typename T>
    MatrixAdapter<T, 1, 3> ColumnMajorAdapter3x3(T* pointer) {
        return MatrixAdapter<T, 1, 3>(pointer);
    }

    template <typename T>
    MatrixAdapter<T, 3, 1> RowMajorAdapter3x3(T* pointer) {
        return MatrixAdapter<T, 3, 1>(pointer);
    }

// xy = x cross y;
    template<typename T> inline
    void CrossProduct(const T x[3], const T y[3], T x_cross_y[3]) {
        x_cross_y[0] = x[1] * y[2] - x[2] * y[1];
        x_cross_y[1] = x[2] * y[0] - x[0] * y[2];
        x_cross_y[2] = x[0] * y[1] - x[1] * y[0];
    }

    template<typename T> inline
    T DotProduct(const T x[3], const T y[3]) {
        return (x[0] * y[0] + x[1] * y[1] + x[2] * y[2]);
    }

// The conversion of a rotation matrix to the angle-axis form is
// numerically problematic when then rotation angle is close to zero
// or to Pi. The following implementation detects when these two cases
// occurs and deals with them by taking code paths that are guaranteed
// to not perform division by a small number.
    template <typename T>
    inline void RotationMatrixToAngleAxis(const T* R, T* angle_axis) {
        RotationMatrixToAngleAxis(ColumnMajorAdapter3x3(R), angle_axis);
    }

    template <typename T, int row_stride, int col_stride>
    void RotationMatrixToAngleAxis(
            const MatrixAdapter<const T, row_stride, col_stride>& R,
            T* angle_axis) {
        T quaternion[4];
        RotationMatrixToQuaternion(R, quaternion);
        QuaternionToAngleAxis(quaternion, angle_axis);
        return;
    }

    template <typename T>
    inline void AngleAxisToRotationMatrix(const T* angle_axis, T* R) {
        AngleAxisToRotationMatrix(angle_axis, ColumnMajorAdapter3x3(R));
    }

    template <typename T, int row_stride, int col_stride>
    void AngleAxisToRotationMatrix(
            const T* angle_axis,
            const MatrixAdapter<T, row_stride, col_stride>& R) {
        static const T kOne = T(1.0);
        const T theta2 = DotProduct(angle_axis, angle_axis);
        if (theta2 > T(std::numeric_limits<double>::epsilon())) {
            // We want to be careful to only evaluate the square root if the
            // norm of the angle_axis vector is greater than zero. Otherwise
            // we get a division by zero.
            const T theta = sqrt(theta2);
            const T wx = angle_axis[0] / theta;
            const T wy = angle_axis[1] / theta;
            const T wz = angle_axis[2] / theta;

            const T costheta = cos(theta);
            const T sintheta = sin(theta);

            R(0, 0) =     costheta   + wx*wx*(kOne -    costheta);
            R(1, 0) =  wz*sintheta   + wx*wy*(kOne -    costheta);
            R(2, 0) = -wy*sintheta   + wx*wz*(kOne -    costheta);
            R(0, 1) =  wx*wy*(kOne - costheta)     - wz*sintheta;
            R(1, 1) =     costheta   + wy*wy*(kOne -    costheta);
            R(2, 1) =  wx*sintheta   + wy*wz*(kOne -    costheta);
            R(0, 2) =  wy*sintheta   + wx*wz*(kOne -    costheta);
            R(1, 2) = -wx*sintheta   + wy*wz*(kOne -    costheta);
            R(2, 2) =     costheta   + wz*wz*(kOne -    costheta);
        } else {
            // Near zero, we switch to using the first order Taylor expansion.
            R(0, 0) =  kOne;
            R(1, 0) =  angle_axis[2];
            R(2, 0) = -angle_axis[1];
            R(0, 1) = -angle_axis[2];
            R(1, 1) =  kOne;
            R(2, 1) =  angle_axis[0];
            R(0, 2) =  angle_axis[1];
            R(1, 2) = -angle_axis[0];
            R(2, 2) = kOne;
        }
    }


}

#endif //MY3DPHOTO_0ROTATION_H
