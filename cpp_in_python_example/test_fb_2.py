#!/usr/bin/env python
from fbWrapper_2 import *
import numpy as np
import ctypes


if __name__ == '__main__':

    matrix = [[1, 2, 3], [4, 5, 6], [7, 8, 9]]
    matrix = np.array(matrix).astype(c_double).ctypes.data_as(POINTER(POINTER(c_double)))
    fb = FooBar2(matrix)

    matr_ret = fb.getMatrix()
    print matr_ret[0][0]
    print matr_ret[0][1]
    print matr_ret[0][2]
    print matr_ret[1][0]
    print matr_ret[1][1]
    print matr_ret[1][2]
    print matr_ret[2][0]
    print matr_ret[2][1]
    print matr_ret[2][2]


























    """
    #matrix = np.array(matrix)
    #matrix = np.array(matrix).astype(c_int).ctypes.data_as(POINTER(POINTER(c_int)))
    """




    """
    arr = [200, 100, 250]
    arr = np.array(arr).astype(c_int)

    print arr

    fb = FooBar(arr);
    arr_ret = fb.getArr()

    a = np.fromiter(arr_ret, dtype=np.int, count=3).tolist()
    print a
    """


    """
    arr = [200, 100, 250]
    arr_ = (c_int * 3)()

    for i in range(3):
        arr_[i] = arr[i]

    fb = FooBar(arr_)
    """


    """
    matrix = [[1, 2, 3], [4, 5, 6], [7, 8, 9]]
    matrix = np.array(matrix)
    matrix = np.array(matrix).astype(c_int).ctypes.data_as(POINTER(POINTER(c_int)))
    fb = FooBar2(matrix)
    """


    """
    matrix = [[1, 2, 3], [4, 5, 6], [7, 8, 9]]
    matrix_ = (c_int * 3 * 3)()

    for i in range(3):
        for j in range(3):
            matrix_[i][j] = matrix[i][j]

    fb = FooBar2(matrix_)
    """

    """
    matrix = [[1, 2, 3], [4, 5, 6], [7, 8, 9]]
    matrix_ = int_matr()

    for i in range(3):
        for j in range(3):
            matrix_[i][j] = matrix[i][j]

    fb = FooBar2(matrix_)
    
    matr_ret = fb.getMatrix()
    a = np.fromiter(matr_ret, dtype=np.int, count=9).tolist()
    print a
    """