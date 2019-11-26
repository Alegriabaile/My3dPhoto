//
// Created by ale on 19-11-6.
//

#ifndef MY3DPHOTO_0CAMERAPARAMETERS_H
#define MY3DPHOTO_0CAMERAPARAMETERS_H

namespace m3d
{
    template <typename T>
    class Intrinsic
    {
    private:
        T intri_c[4];//fx, fy, ppx, ppy;
        T intri_d[4];
    public:
//        Intrinsic():fx(0), fy(0), ppx(0), ppy(0) {}
        Intrinsic(T _fx_c = 0, T _fy_c = 0, T _ppx_c = 0, T _ppy_c = 0)
        {
            intri_c[0] = _fx_c;
            intri_c[1] = _fy_c;
            intri_c[2] = _ppx_c;
            intri_c[3] = _ppy_c;

            for(int i=0; i<4; ++i)
                intri_d[i] = intri_c[i];
        }
        Intrinsic(T _intrinsic_c[4], T _intrinsic_d[4])
        {
            for(int i=0; i<4; ++i)
                intri_c[i] = _intrinsic_c[i];
            for(int i=0; i<4; ++i)
                intri_d[i] = _intrinsic_d[i];
        }
        virtual ~Intrinsic(){}

        void setIntrinsic(T _fx_c = 0, T _fy_c = 0, T _ppx_c = 0, T _ppy_c = 0)
        {
            intri_c[0] = _fx_c;
            intri_c[1] = _fy_c;
            intri_c[2] = _ppx_c;
            intri_c[3] = _ppy_c;

            for(int i=0; i<4; ++i)
                intri_d[i] = intri_c[i];
        }

        void setIntrinsic(T _intrinsic_c[4], T _intrinsic_d[4])
        {
            for(int i=0; i<4; ++i)
                intri_c[i] = _intrinsic_c[i];
            for(int i=0; i<4; ++i)
                intri_d[i] = _intrinsic_d[i];
        }

        T& fx_c(){return intri_c[0];}
        T& fy_c(){return intri_c[1];}
        T& ppx_c(){return intri_c[2];}
        T& ppy_c(){return intri_c[3];}
        T& fx_d(){return intri_d[0];}
        T& fy_d(){return intri_d[1];}
        T& ppx_d(){return intri_d[2];}
        T& ppy_d(){return intri_d[3];}

        T* intrinsic_c(){ return intri_c;}
        T* intrinsic_d(){ return intri_d;}

    };
    typedef Intrinsic<double> IntrinsicD;


    template <typename T>
    class Extrinsic
    {
    private:
        //data
//        T a[3];//angleAxis
//        T t[3];//translate
        
    public:
        T rts[6];
        //constructor, deconstructor
        Extrinsic(T _ax = 0, T _ay = 0, T _az = 0,
                  T _tx = 0, T _ty = 0, T _tz = 0)
        {
            rts[0] = _ax; rts[1] = _ay; rts[2] = _az;
            rts[3] = _tx; rts[4] = _ty; rts[5] = _tz;
        }

        Extrinsic(T _a[3], T _t[3])
        {
            for(int i=0; i<3; ++i)
            {
                rts[i] = _a[i];
                rts[i+3] = _t[i];
            }
        }
        virtual ~Extrinsic(){}

        //functions
        T* angleAxis(){ return rts;}
        T* translate(){ return rts+3;}
        T* transform(){ return rts;}

    };
    typedef Extrinsic<double> ExtrinsicD;
}

#endif //MY3DPHOTO_0CAMERAPARAMETERS_H
