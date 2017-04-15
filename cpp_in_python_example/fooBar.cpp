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
        int **matrix;

        FooBar2(int *matrix_) {
            matrix = new int*[3];

            for (int i = 0; i < 3; i++) {
                matrix[i] = new int[3];

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

        int ** getMatrix() {
            return matrix;
        }
};

int main() {}


extern "C" {
    FooBar * FB_new(int *arr) { return new FooBar(arr); }
    int * FB_getArr(FooBar *fb) { return fb->arr; }

    FooBar2 * FB_new2(int *matrix) { return new FooBar2(matrix); }
    int ** FB_getMatrix(FooBar2 *fb) { return fb->getMatrix(); }
}


/*

g++ -c -std=c++11 -fPIC fooBar.cpp
g++ -std=c++11 -fPIC fooBar.cpp
g++ -shared -Wl,-soname,libfb.so -o libfb.so fooBar.o
./test_fb.py

*/