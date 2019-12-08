//
// Created by ale on 19-12-7.
//

#include "9generateTrianglesFromPanorama.h"

namespace m3d {

    size_t generateTrianglesFromPanorama(const cv::Mat &pano_depth_, std::vector<float> &out)
    {
        cv::Mat pano_depth = pano_depth_.clone();
        if(pano_depth.type() != CV_16UC1)
            pano_depth.convertTo(pano_depth, CV_16UC1);

        const float MAX_DIFF = 0.05;//相邻像素深度值可相差的最大比率

        const float H = pano_depth.rows;
        const float W = pano_depth.cols;

        out.clear();
        out.reserve((H - 1) * (W - 1) * 3);//* 6 * 3);

        float x, y, r;//3d coordinates and uv texture coordinates
        for (float h = 0; h < H - 1; ++h) {
            for (float w = 0; w < W - 1; ++w) {
                //参考了 https://github.com/simonfuhrmann/mve/blob/master/libs/mve/depthmap.cc#L211
                //0 1
                //2 3
                float d[4] = {0.0f, 0.0f, 0.0f, 0.0f};
                d[0] = pano_depth.at<ushort>(h, w);//depth of the current point
                d[1] = pano_depth.at<ushort>((int) h, (int) w + 1);//depth of the right
                d[2] = pano_depth.at<ushort>((int) h + 1, (int) w);//depth of the bottom
                d[3] = pano_depth.at<ushort>((int) h + 1, (int) w + 1);//depth of the right bottom
                //At least three valid depth values are required.
                int positive_d_n = int(d[0] > 0) + int(d[1] > 0) + int(d[2] > 0) + int(d[3] > 0);
                if (positive_d_n < 3) continue;
                //3,1,2;  0,2,3;  3,1,0;  0,2,1
                int tri[4] = {0, 0, 0, 0};
                for (int i = 0; i < 4; ++i)//if positive_d_n==3,then
                {
                    if (!(d[i] > 0)) {
                        tri[i] = 1;
                        break;
                    }
                }

                if (positive_d_n == 4) {
                    if (fabs(d[0] - d[3]) > fabs(d[1] - d[2]))
                        tri[0] = tri[3] = 1;
                    else
                        tri[1] = tri[2] = 1;
                }

                int tris[4][3] =
                        {
                                {3, 1, 2},
                                {0, 2, 3},
                                {3, 1, 0},
                                {0, 2, 1}
                        };

                for (int i = 0; i < 4; ++i) {
                    if (tri[i] == 0)
                        continue;

                    int in0 = tris[i][0];
                    int in1 = tris[i][1];
                    int in2 = tris[i][2];
                    if (fabs(d[in0] - d[in1]) < MAX_DIFF * fmax(d[in0], d[in1])
                        && fabs(d[in1] - d[in2]) < MAX_DIFF * fmax(d[in1], d[in2])
                        && fabs(d[in2] - d[in0]) < MAX_DIFF * fmax(d[in2], d[in0])) {
                        for (int j = 0; j < 3; ++j) {
                            r = d[tris[i][j]];
                            x = w + tris[i][j] % 2;
                            y = h + tris[i][j] / 2;
//                        x = z * (w + tris[i][j] % 2 - cx) / fx;
//                        y = z * (h + tris[i][j] / 2 - cy) / fy;
//                        u = (w + 0.5 + tris[i][j] % 2) / (float) (W );
//                        v = 1 - (h + 0.5 + tris[i][j] / 2) / (float) (H);
                            out.push_back(x);
                            out.push_back(y);
                            out.push_back(r);
                        }
                    }
                }//end of current(h,w) position's triangle generation

            }//W
        }//H, end of whole loop

        //三角形数目
        return out.size() / (3 * 3);
    }


}
