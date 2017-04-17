#include <iostream>
#include <vector>

class FooBar {
    public:
        int *arr;


        FooBar(int *arr_) {
            arr = arr_;

            for (int i = 0; i < 3; i++) {
                std::cout << arr[i] << " ";
            }
            std::cout << std::endl;
        }

        int * getArr() {
            return arr;
        }
};


class FooBar2 {
    public:
        double **matrix;

        FooBar2(double *matrix_) {
            matrix = new double*[3];

            for (int i = 0; i < 3; i++) {
                matrix[i] = new double[3];

                for (int j = 0; j < 3; j++) {
                    matrix[i][j] = matrix_[i*3+j];
                }
            }

            for (int i = 0; i < 3; i++) {
                for (int j = 0; j < 3; j++) {
                    std::cout << matrix[i][j];
             
                }
                std::cout << std::endl;
            }
        }

        double ** getMatrix() {
            return matrix;
        }
};

int main() {}


extern "C" {
    FooBar * FB_new(int *arr) { return new FooBar(arr); }
    int * FB_getArr(FooBar *fb) { return fb->arr; }

    FooBar2 * FB_new2(double *matrix) { return new FooBar2(matrix); }
    double ** FB_getMatrix(FooBar2 *fb) { return fb->getMatrix(); }
}


/*

g++ -c -std=c++11 -fPIC fooBar_2.cpp
g++ -std=c++11 -fPIC fooBar_2.cpp
g++ -shared -Wl,-soname,libfb.so -o libfb.so fooBar_2.o
./test_fb_2.py

*/