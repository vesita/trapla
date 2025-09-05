#ifndef MAT_HPP
#define MAT_HPP

template<typename T> class MatUnit;
template<typename T> class Mat;

#include <vector>
#include <stdexcept>
#include <iostream>

template<typename T>
class Mat {
public:
    std::vector<MatUnit<T>> mats;
    std::vector<int> edges;
    int size;

    Mat();

    static Mat<T> desig(const std::vector<MatUnit<T>>& mus);

    void put(const MatUnit<T>& mu);

    Mat<T> result() const;

    Mat<T> derect() const;

    bool is_empty() const;

private:
    MatUnit<T> construct_result(const std::vector<std::vector<int>>& split, int row, int col) const;
};

template<typename T>
class MatUnit {
public:
    int rows;
    int cols;
    std::vector<std::vector<T>> data;

    MatUnit(int rows = 0, int cols = 0);

    MatUnit(int rows, int cols, const T& initial_value);

    static MatUnit<T> desig(const std::vector<std::vector<T>>& mat);

    bool operator==(const MatUnit<T>& other) const;

    bool can_multiply(const MatUnit<T>& other) const;

    MatUnit<T> multiply(const MatUnit<T>& other) const;

    void set_all(const T& value);

    int get_rows() const;

    int get_cols() const;

    T& operator()(int row, int col);
    const T& operator()(int row, int col) const;
};

// 矩阵旋转函数声明
MatUnit<double> rot_x(double angle);
MatUnit<double> rot_y(double angle);
MatUnit<double> rot_z(double angle);

#endif