//
// Created by ale on 19-11-25.
//

#ifndef MY3DPHOTO_0TRANSFORM_H
#define MY3DPHOTO_0TRANSFORM_H

#include <opencv2/opencv.hpp>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

namespace m3d
{
    //globalPose[6], relativePose[6], res[6].

    //assume world pose of src: Rs, ts; world pose of dst: Rd, td.
    //assume relative pose from src to dst: R, t.

    //if globalPose is dst(fSrc2Dst is true).
    //[ R*src + t -> dst.]      [ Rd*dst + td -> world.]        ------->      [ Rd*R*src + Rd*t + td -> world. ]     -------->    R1 = Rd*R, t1 = Rd*t + td.
    //if globalPose is src(fSrc2Dst is false).
    //[ R-1*dst + (-R-1*t) -> src.]    [ Rs*src + ts -> world.]    -------->        [ Rs*R-1*dst + ts - Rs*R-1*t -> world.]        --------> R2 = Rs*R-1, t2 = - Rs*R-1*t + ts.

    void RelativeToGlobal(const double * const globalPose, const double * const relativePose, double res[6], bool fSrc2Dst = true);

    void transformMatrixFromExtrinsics(const double * const extrinsics, glm::mat4 & transformMat);

}

#endif //MY3DPHOTO_0TRANSFORM_H
