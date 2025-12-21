/**
 ******************************************************************************
 * @file    matrix.h
 * @brief   Matrix/vector calculation. 矩阵/向量运算
 * @author  Spoon Guan
 ******************************************************************************
 * Copyright (c) 2023 Team JiaoLong-SJTU
 * All rights reserved.
 ******************************************************************************
 */

#ifndef MATRIX_H
#define MATRIX_H

#include "main.h"
#include "arm_math.h"

// Matrix class
template <int _rows, int _cols>
class Mat_f32
{
public:
    // Constructor without input data
    Mat_f32() : rows_(_rows), cols_(_cols)
    {
        arm_mat_init_f32(&arm_mat_, _rows, _cols, this->data_);
    }

    // Constructor with input data
    explicit Mat_f32(const float data[_rows * _cols]) : Mat_f32()
    {
        memcpy(this->data_, data, _rows * _cols * sizeof(float));
    }

    // Copy constructor
    Mat_f32(const Mat_f32<_rows, _cols>& mat) : Mat_f32()
    {
        memcpy(this->data_, mat.data_, _rows * _cols * sizeof(float));
    }

    // Destructor
    ~Mat_f32() = default;

    // Row size
    static int rows() { return _rows; }
    // Column size
    static int cols() { return _cols; }

    // Element
    float& operator()(const int r, const int c) { return data_[r * _cols + c]; }
    float* operator[](const int& row) { return &this->data_[row * _cols]; }

    // Operators
    Mat_f32<_rows, _cols>& operator=(const Mat_f32<_rows, _cols> mat)
    {
        memcpy(this->data_, mat.data_, _rows * _cols * sizeof(float));
        return *this;
    }

    Mat_f32<_rows, _cols>& operator+=(const Mat_f32<_rows, _cols> mat)
    {
        arm_status s;
        s = arm_mat_add_f32(&this->arm_mat_, &mat.arm_mat_, &this->arm_mat_);
        return *this;
    }

    Mat_f32<_rows, _cols>& operator-=(const Mat_f32<_rows, _cols> mat)
    {
        arm_status s;
        s = arm_mat_sub_f32(&this->arm_mat_, &mat.arm_mat_, &this->arm_mat_);
        return *this;
    }

    Mat_f32<_rows, _cols>& operator*=(const float& val)
    {
        arm_status s;
        s = arm_mat_scale_f32(&this->arm_mat_, val, &this->arm_mat_);
        return *this;
    }

    Mat_f32<_rows, _cols>& operator/=(const float& val)
    {
        arm_status s;
        s = arm_mat_scale_f32(&this->arm_mat_, 1.f / val, &this->arm_mat_);
        return *this;
    }

    Mat_f32<_rows, _cols> operator+(const Mat_f32<_rows, _cols>& mat)
    {
        arm_status            s;
        Mat_f32<_rows, _cols> res;
        s = arm_mat_add_f32(&this->arm_mat_, &mat.arm_mat_, &res.arm_mat_);
        return res;
    }

    Mat_f32<_rows, _cols> operator-(const Mat_f32<_rows, _cols>& mat)
    {
        arm_status            s;
        Mat_f32<_rows, _cols> res;
        s = arm_mat_sub_f32(&this->arm_mat_, &mat.arm_mat_, &res.arm_mat_);
        return res;
    }

    Mat_f32<_rows, _cols> operator*(const float& val)
    {
        arm_status            s;
        Mat_f32<_rows, _cols> res;
        s = arm_mat_scale_f32(&this->arm_mat_, val, &res.arm_mat_);
        return res;
    }

    friend Mat_f32<_rows, _cols> operator*(const float&                 val,
                                           const Mat_f32<_rows, _cols>& mat)
    {
        arm_status            s;
        Mat_f32<_rows, _cols> res;
        s = arm_mat_scale_f32(&mat.arm_mat_, val, &res.arm_mat_);
        return res;
    }

    Mat_f32<_rows, _cols> operator/(const float& val)
    {
        arm_status            s;
        Mat_f32<_rows, _cols> res;
        s = arm_mat_scale_f32(&this->arm_mat_, 1.f / val, &res.arm_mat_);
        return res;
    }

    // Matrix multiplication
    template <int cols>
    friend Mat_f32<_rows, cols> operator*(const Mat_f32<_rows, _cols>& mat1,
                                          const Mat_f32<_cols, cols>&  mat2)
    {
        arm_status           s;
        Mat_f32<_rows, cols> res;
        s = arm_mat_mult_f32(&mat1.arm_mat_, &mat2.arm_mat_, &res.arm_mat_);
        return res;
    }

    // Submatrix
    template <int rows, int cols>
    Mat_f32<rows, cols> block(const int& start_row, const int& start_col)
    {
        Mat_f32<rows, cols> res;
        for (int row = start_row; row < start_row + rows; row++)
        {
            memcpy(static_cast<float*>(res[0]) + (row - start_row) * cols,
                   static_cast<float*>(this->data_) + row * _cols + start_col,
                   cols * sizeof(float));
        }
        return res;
    }

    // Specific row
    Mat_f32<1, _cols> row(const int& row) { return block<1, _cols>(row, 0); }
    // Specific column
    Mat_f32<_rows, 1> col(const int& col) { return block<_rows, 1>(0, col); }

    // Transpose
    Mat_f32<_cols, _rows> trans()
    {
        Mat_f32<_cols, _rows> res;
        arm_mat_trans_f32(&arm_mat_, &res.arm_mat_);
        return res;
    }

    // Trace
    float trace()
    {
        float res = 0;
        for (int i = 0; i < fmin(_rows, _cols); i++)
        {
            res += (*this)[i][i];
        }
        return res;
    }

    // Norm
    float norm() { return sqrtf((this->trans() * *this)[0][0]); }

    // clear
    void clear() { memset(data_, 0, _rows * _cols * sizeof(float)); }

public:
    // arm matrix instance
    arm_matrix_instance_f32 arm_mat_{};

protected:
    // size
    int rows_, cols_;
    // data
    float data_[_rows * _cols]{};
};

// Matrix functions
namespace MatrixF32
{
    // Special Matrices
    // Zero matrix
    template <int _rows, int _cols>
    Mat_f32<_rows, _cols> zeros()
    {
        float data[_rows * _cols] = {0};
        return Mat_f32<_rows, _cols>(data);
    }

    // Ones matrix
    template <int _rows, int _cols>
    Mat_f32<_rows, _cols> ones()
    {
        float data[_rows * _cols] = {0};
        for (int i = 0; i < _rows * _cols; i++)
        {
            data[i] = 1;
        }
        return Mat_f32<_rows, _cols>(data);
    }

    // Identity matrix
    template <int _rows, int _cols>
    Mat_f32<_rows, _cols> eye()
    {
        float data[_rows * _cols] = {0};
        for (int i = 0; i < fmin(_rows, _cols); i++)
        {
            data[i * _cols + i] = 1;
        }
        return Mat_f32<_rows, _cols>(data);
    }

    // Diagonal matrix
    template <int _rows, int _cols>
    Mat_f32<_rows, _cols> diag(Mat_f32<_rows, 1> vec)
    {
        Mat_f32<_rows, _cols> res = MatrixF32::zeros<_rows, _cols>();
        for (int i = 0; i < fmin(_rows, _cols); i++)
        {
            res[i][i] = vec[i][0];
        }
        return res;
    }

    // Inverse
    template <int _dim>
    Mat_f32<_dim, _dim> inv(Mat_f32<_dim, _dim> mat)
    {
        arm_status s;
        // extended matrix [A|I]
        Mat_f32<_dim, 2 * _dim> ext_mat = MatrixF32::zeros<_dim, 2 * _dim>();
        for (int i = 0; i < _dim; i++)
        {
            memcpy(ext_mat[i], mat[i], _dim * sizeof(float));
            ext_mat[i][_dim + i] = 1;
        }
        // elimination
        for (int i = 0; i < _dim; i++)
        {
            // find maximum absolute value in the first column in lower right block
            float abs_max     = fabs(ext_mat[i][i]);
            int   abs_max_row = i;
            for (int row = i; row < _dim; row++)
            {
                if (abs_max < fabs(ext_mat[row][i]))
                {
                    abs_max     = fabs(ext_mat[row][i]);
                    abs_max_row = row;
                }
            }
            if (abs_max < 1e-12f)
            {
                // singular
                return MatrixF32::zeros<_dim, _dim>();
                s = ARM_MATH_SINGULAR;
            }
            if (abs_max_row != i)
            {
                // row exchange
                float                tmp;
                Mat_f32<1, 2 * _dim> row_i       = ext_mat.row(i);
                Mat_f32<1, 2 * _dim> row_abs_max = ext_mat.row(abs_max_row);
                memcpy(ext_mat[i], row_abs_max[0], 2 * _dim * sizeof(float));
                memcpy(ext_mat[abs_max_row], row_i[0], 2 * _dim * sizeof(float));
            }
            float k = 1.f / ext_mat[i][i];
            for (int col = i; col < 2 * _dim; col++)
            {
                ext_mat[i][col] *= k;
            }
            for (int row = 0; row < _dim; row++)
            {
                if (row == i)
                {
                    continue;
                }
                k = ext_mat[row][i];
                for (int j = i; j < 2 * _dim; j++)
                {
                    ext_mat[row][j] -= k * ext_mat[i][j];
                }
            }
        }
        // inv = ext_mat(:,n+1:2n)
        s = ARM_MATH_SUCCESS;
        Mat_f32<_dim, _dim> res;
        for (int i = 0; i < _dim; i++)
        {
            memcpy(res[i], &ext_mat[i][_dim], _dim * sizeof(float));
        }
        return res;
    }
} // namespace matrixf

namespace Vector3F32
{
    // hat of vector
    inline Mat_f32<3, 3> hat(Mat_f32<3, 1> vec)
    {
        float hat[9] = {
            0, -vec[2][0], vec[1][0],
            vec[2][0], 0, -vec[0][0],
            -vec[1][0], vec[0][0], 0
        };
        return Mat_f32<3, 3>(hat);
    }

    // cross product
    inline Mat_f32<3, 1> cross(const Mat_f32<3, 1>& vec1, const Mat_f32<3, 1>& vec2)
    {
        return Vector3F32::hat(vec1) * vec2;
    }
} // namespace vector3f

#endif  // MATRIX_H
