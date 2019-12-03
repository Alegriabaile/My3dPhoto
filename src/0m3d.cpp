//
// Created by ale on 19-11-21.
//

#include "0m3d.h"

namespace m3d
{
    IntrinsicD Frame::intrinsicD;
    const size_t Frame::PANO_W = 4096;
    const size_t Frame::PANO_H = 2048;

    const double Edge::c = 100;
    const double Edge::k = 0.5;
}