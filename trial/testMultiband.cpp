//
// Created by ale on 19-12-26.
//

#include <iostream>

class A
{
public:
    union{
        size_t data[4];
        struct{
            size_t d1, d2, d3, d4;
        };
    };

    A(size_t data_[4]){
        for(size_t i = 0; i < 4; ++i)
            data[i] = data_[i];
    }
};

int main()
{
    size_t data_[4] = {4, 3, 2, 1};
    A a(data_);

    printf("data: %ld, %ld, %ld, %ld \n", a.d1, a.d2, a.d3, a.d4);

    return 0;
}