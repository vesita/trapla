#include "utils/mat.hpp"

template<typename T>
MatUnit<T>::MatUnit(int rows, int cols) : rows(rows), cols(cols) {
    data.resize(rows, std::vector<T>(cols));
}

template<typename T>
MatUnit<T>::MatUnit(int rows, int cols, const T& initial_value) : rows(rows), cols(cols) {
    data.resize(rows, std::vector<T>(cols, initial_value));
}

template<typename T>
MatUnit<T> MatUnit<T>::desig(const std::vector<std::vector<T>>& mat) {
    if (mat.empty()) {
        return MatUnit<T>(0, 0);
    }
    
    int rows = mat.size();
    int cols = mat[0].size();
    MatUnit<T> result(rows, cols);
    result.data = mat;
    return result;
}

template<typename T>
bool MatUnit<T>::operator==(const MatUnit<T>& other) const {
    return rows == other.rows && cols == other.cols && data == other.data;
}

template<typename T>
bool MatUnit<T>::can_multiply(const MatUnit<T>& other) const {
    return cols == other.rows;
}

template<typename T>
MatUnit<T> MatUnit<T>::multiply(const MatUnit<T>& other) const {
    if (!can_multiply(other)) {
        throw std::invalid_argument("Matrix dimensions do not match for multiplication");
    }

    MatUnit<T> result(rows, other.cols);
    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < other.cols; j++) {
            T sum = T(0);
            for (int k = 0; k < cols; k++) {
                sum += data[i][k] * other.data[k][j];
            }
            result.data[i][j] = sum;
        }
    }
    return result;
}

template<typename T>
void MatUnit<T>::set_all(const T& value) {
    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            data[i][j] = value;
        }
    }
}

template<typename T>
int MatUnit<T>::get_rows() const {
    return rows;
}

template<typename T>
int MatUnit<T>::get_cols() const {
    return cols;
}

template<typename T>
T& MatUnit<T>::operator()(int row, int col) {
    return data[row][col];
}

template<typename T>
const T& MatUnit<T>::operator()(int row, int col) const {
    return data[row][col];
}

template<typename T>
Mat<T>::Mat() : size(0) {}

template<typename T>
Mat<T> Mat<T>::desig(const std::vector<MatUnit<T>>& mus) {
    Mat<T> mat;
    mat.mats = mus;
    mat.size = mus.size();
    
    if (!mus.empty()) {
        mat.edges.resize(mus.size() + 1);
        mat.edges[0] = mus[0].rows;
        for (size_t i = 1; i < mus.size(); i++) {
            mat.edges[i] = mus[i].cols;
        }
        mat.edges[mus.size()] = mus[mus.size() - 1].cols;
    }
    
    return mat;
}

template<typename T>
void Mat<T>::put(const MatUnit<T>& mu) {
    mats.push_back(mu);
    size = mats.size();
    
    if (edges.empty()) {
        edges.push_back(mu.rows);
        edges.push_back(mu.cols);
    } else {
        edges.push_back(mu.cols);
    }
}

template<typename T>
bool Mat<T>::is_empty() const {
    return size == 0;
}

template<typename T>
MatUnit<T> Mat<T>::construct_result(const std::vector<std::vector<int>>& split, int row, int col) const {
    if (row == col) {
        return mats[row];
    }
    
    int k = split[row][col];
    MatUnit<T> left = construct_result(split, row, k);
    MatUnit<T> right = construct_result(split, k + 1, col);
    return left.multiply(right);
}

template<typename T>
Mat<T> Mat<T>::result() const {
    if (size == 0) {
        return Mat<T>();
    }
    
    if (size == 1) {
        std::vector<MatUnit<T>> result_mats = {mats[0]};
        return Mat<T>::desig(result_mats);
    }
    

    std::vector<std::vector<T>> mem(size, std::vector<T>(size, T(0)));
    std::vector<std::vector<int>> split(size, std::vector<int>(size, 0));
    

    for (int chain = 2; chain <= size; chain++) {
        for (int row = 0; row < size - chain + 1; row++) {
            int col = row + chain - 1;
            
            mem[row][col] = std::numeric_limits<T>::max();
            split[row][col] = row;
            

            for (int next = row; next < col; next++) {
                T cost = mem[row][next] + mem[next + 1][col];
                T product = T(edges[row] * edges[next + 1] * edges[col + 1]);
                T total_cost = cost + product;
                
                if (total_cost < mem[row][col]) {
                    mem[row][col] = total_cost;
                    split[row][col] = next;
                }
            }
        }
    }
    

    MatUnit<T> result_matrix = construct_result(split, 0, size - 1);
    std::vector<MatUnit<T>> result_mats = {result_matrix};
    return Mat<T>::desig(result_mats);
}

template<typename T>
Mat<T> Mat<T>::derect() const {
    if (is_empty()) {
        throw std::invalid_argument("Cannot multiply matrices: list is empty");
    }
    
    MatUnit<T> result = mats[0];
    for (int i = 1; i < size; i++) {
        result = result.multiply(mats[i]);
    }
    
    std::vector<MatUnit<T>> result_mats = {result};
    return Mat<T>::desig(result_mats);
}

MatUnit<double> rot_x(double angle) {
    MatUnit<double> result(4, 4);
    double cos_a = cos(angle);
    double sin_a = sin(angle);
    
    result(0, 0) = 1.0; result(0, 1) = 0.0;   result(0, 2) = 0.0;   result(0, 3) = 0.0;
    result(1, 0) = 0.0; result(1, 1) = cos_a; result(1, 2) = -sin_a; result(1, 3) = 0.0;
    result(2, 0) = 0.0; result(2, 1) = sin_a; result(2, 2) = cos_a;  result(2, 3) = 0.0;
    result(3, 0) = 0.0; result(3, 1) = 0.0;   result(3, 2) = 0.0;   result(3, 3) = 1.0;
    
    return result;
}

MatUnit<double> rot_y(double angle) {
    MatUnit<double> result(4, 4);
    double cos_a = cos(angle);
    double sin_a = sin(angle);
    
    result(0, 0) = cos_a;  result(0, 1) = 0.0; result(0, 2) = sin_a; result(0, 3) = 0.0;
    result(1, 0) = 0.0;    result(1, 1) = 1.0; result(1, 2) = 0.0;   result(1, 3) = 0.0;
    result(2, 0) = -sin_a; result(2, 1) = 0.0; result(2, 2) = cos_a; result(2, 3) = 0.0;
    result(3, 0) = 0.0;    result(3, 1) = 0.0; result(3, 2) = 0.0;   result(3, 3) = 1.0;
    
    return result;
}

MatUnit<double> rot_z(double angle) {
    MatUnit<double> result(4, 4);
    double cos_a = cos(angle);
    double sin_a = sin(angle);
    
    result(0, 0) = cos_a; result(0, 1) = -sin_a; result(0, 2) = 0.0; result(0, 3) = 0.0;
    result(1, 0) = sin_a; result(1, 1) = cos_a;  result(1, 2) = 0.0; result(1, 3) = 0.0;
    result(2, 0) = 0.0;   result(2, 1) = 0.0;    result(2, 2) = 1.0; result(2, 3) = 0.0;
    result(3, 0) = 0.0;   result(3, 1) = 0.0;    result(3, 2) = 0.0; result(3, 3) = 1.0;
    
    return result;
}

template class MatUnit<int>;
template class MatUnit<double>;
template class MatUnit<float>;

template class Mat<int>;
template class Mat<double>;
template class Mat<float>;