#ifndef MATRIX_HPP
#define MATRIX_HPP

#include <algorithm>
#include <arm_math.h>
#include <cassert>
#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <initializer_list>
#include <limits>
#include <type_traits>

namespace gdut {

template <typename Mat, std::size_t Rows, std::size_t Cols>
struct redefine_matrix {};

template <std::size_t Rows, std::size_t Cols,
          template <typename, std::size_t, std::size_t> class Mat, typename Ty,
          std::size_t OtherRows, std::size_t OtherCols>
struct redefine_matrix<Mat<Ty, OtherRows, OtherCols>, Rows, Cols> {
  using type = Mat<Ty, Rows, Cols>;
};

template <typename Mat, std::size_t Rows, std::size_t Cols>
using redefine_matrix_t = typename redefine_matrix<Mat, Rows, Cols>::type;

template <typename Mat> struct matrix_parameters {};

template <template <typename T, std::size_t, std::size_t> class Mat,
          typename Ty, std::size_t Rows, std::size_t Cols>
struct matrix_parameters<Mat<Ty, Rows, Cols>> {
  using value_type = Ty;
  static constexpr std::size_t row_value = Rows;
  static constexpr std::size_t col_value = Cols;
};

template <typename Ty>
inline constexpr bool is_scalar_v =
    std::is_integral_v<Ty> || std::is_floating_point_v<Ty>;

template <typename Derived> class base_matrix {};

template <typename Mat, typename = void> struct is_matrix : std::false_type {};

template <typename Mat>
struct is_matrix<Mat, std::void_t<typename matrix_parameters<Mat>::value_type,
                                   decltype(matrix_parameters<Mat>::row_value),
                                   decltype(matrix_parameters<Mat>::col_value)>>
    : std::bool_constant<std::is_base_of_v<base_matrix<Mat>, Mat>> {};

template <typename Mat>
inline constexpr bool is_matrix_v = is_matrix<Mat>::value;

template <template <typename, std::size_t, std::size_t> class Matrix,
          typename T, std::size_t Rows, std::size_t Cols>
class base_matrix<Matrix<T, Rows, Cols>> {
  static_assert(is_scalar_v<T>,
                "Matrix value type must be a scalar (integral or floating-point)");
  static_assert(Rows > 0, "Matrix must have at least one row");
  static_assert(Cols > 0, "Matrix must have at least one column");
  
  using Derived = Matrix<T, Rows, Cols>;

public:
  constexpr base_matrix() = default;
  constexpr ~base_matrix() noexcept = default;
  constexpr base_matrix(const base_matrix &) = default;
  constexpr base_matrix(base_matrix &&) = default;
  base_matrix &operator=(const base_matrix &) = default;
  base_matrix &operator=(base_matrix &&) = default;

  Derived add(const Derived &other) const {
    return get_derived()->add_impl(other);
  }

  Derived sub(const Derived &other) const {
    return get_derived()->sub_impl(other);
  }

  template <typename Ty, typename = std::enable_if_t<is_scalar_v<Ty>>>
  Derived mult(Ty val) const {
    return get_derived()->mult_impl(val);
  }

  template <typename Mat>
  Matrix<T, Rows, matrix_parameters<Mat>::col_value>
  mult(const Mat &other) const {
    return get_derived()->mult_impl(other);
  }

  T det() const {
    static_assert(Rows == Cols, "Determinant only defined for square matrices");
    if constexpr (Rows == 1) {
      return get_value(0, 0);
    } else if constexpr (Rows == 2) {
      const T a = get_value(0, 0);
      const T b = get_value(0, 1);
      const T c = get_value(1, 0);
      const T d = get_value(1, 1);
      return a * d - b * c;
    } else if constexpr (Rows == 3) {
      const T a = get_value(0, 0);
      const T b = get_value(0, 1);
      const T c = get_value(0, 2);
      const T d = get_value(1, 0);
      const T e = get_value(1, 1);
      const T f = get_value(1, 2);
      const T g = get_value(2, 0);
      const T h = get_value(2, 1);
      const T i = get_value(2, 2);
      return a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g);
    } else {
      constexpr std::size_t N = Rows;
      T lu[N * N];
      int sign = 1;
      std::copy_n(get(), N * N, lu);

      for (std::size_t k = 0; k < N; ++k) {
        T max_val = std::abs(lu[k * N + k]);
        std::size_t pivot = k;
        for (std::size_t i = k + 1; i < N; ++i) {
          T val = std::abs(lu[i * N + k]);
          if (val > max_val) {
            max_val = val;
            pivot = i;
          }
        }
        if (max_val <= std::numeric_limits<T>::epsilon()) {
          return static_cast<T>(0);
        }
        if (pivot != k) {
          std::swap_ranges(lu + k * N, lu + (k + 1) * N, lu + pivot * N);
          sign = -sign;
        }

        for (std::size_t i = k + 1; i < N; ++i) {
          lu[i * N + k] /= lu[k * N + k];
          for (std::size_t j = k + 1; j < N; ++j) {
            lu[i * N + j] -= lu[i * N + k] * lu[k * N + j];
          }
        }
      }

      T det = static_cast<T>(sign);
      for (std::size_t i = 0; i < N; ++i) {
        det *= lu[i * N + i];
      }
      return det;
    }
  }

  bool is_invertible() const {
    static_assert(Rows == Cols,
                  "Invertibility only defined for square matrices");
    return std::abs(det()) > std::numeric_limits<T>::epsilon();
  }

  Derived inverse() const {
    static_assert(Rows == Cols, "Inverse only defined for square matrices");
    return get_derived()->inverse_impl();
  }

  T norm() const {
    T sum = T{};
    for (std::size_t i = 0; i < size(); ++i) {
      sum += get()[i] * get()[i];
    }
    return std::sqrt(sum);
  }

  Derived normalized() const {
    T n = norm();
    if (n <= std::numeric_limits<T>::epsilon()) {
      return *get_derived();
    }
    return mult(static_cast<T>(1) / n);
  }

  redefine_matrix_t<Derived, Cols, Rows> transpose() const {
    return get_derived()->transpose_impl();
  }

  T *get() { return get_derived()->get_impl(); }

  const T *get() const { return get_derived()->get_impl(); }

  constexpr std::size_t size() const { return Rows * Cols; }
  constexpr std::size_t rows() const { return Rows; }
  constexpr std::size_t cols() const { return Cols; }

  T &get_value(std::size_t i, std::size_t j) {
    return get_derived()->get_value_impl(i, j);
  }

  const T &get_value(std::size_t i, std::size_t j) const {
    return get_derived()->get_value_impl(i, j);
  }

  auto get_handle() const { return get_derived()->get_handle_impl(); }

  T &operator[](std::size_t i, std::size_t j) { return get_value(i, j); }

  const T &operator[](std::size_t i, std::size_t j) const {
    return get_value(i, j);
  }

  T &operator[](std::size_t i) {
    static_assert(Cols == 1, "Single index operator only defined for vectors");
    return get_value(i, 0);
  }

  const T &operator[](std::size_t i) const {
    static_assert(Cols == 1, "Single index operator only defined for vectors");
    return get_value(i, 0);
  }

  static Derived identity() {
    static_assert(Rows == Cols,
                  "Identity matrix only defined for square matrices");
    Derived res;
    for (std::size_t i = 0; i < Rows; ++i) {
      res.get_value(i, i) = static_cast<T>(1);
    }
    return res;
  }

  friend Derived operator+(const Derived &a, const Derived &b) {
    return a.add(b);
  }

  friend Derived operator-(const Derived &a, const Derived &b) {
    return a.sub(b);
  }

protected:
  Derived *get_derived() { return static_cast<Derived *>(this); }

  const Derived *get_derived() const {
    return static_cast<const Derived *>(this);
  }
};

template <typename MatA, typename MatB,
          typename = std::enable_if_t<is_matrix_v<MatA> && is_matrix_v<MatB>>>
inline redefine_matrix_t<MatA, matrix_parameters<MatA>::row_value,
                         matrix_parameters<MatB>::col_value>
operator*(const MatA &a, const MatB &b) {
  static_assert(std::is_same_v<typename matrix_parameters<MatA>::value_type,
                               typename matrix_parameters<MatB>::value_type>,
                "Matrix multiplication requires both matrices to have the same "
                "value type");
  static_assert(
      matrix_parameters<MatA>::col_value == matrix_parameters<MatB>::row_value,
      "Matrix multiplication requires the number of columns of the first "
      "matrix to be equal to the number of rows of the second matrix");
  return a.mult(b);
}

template <typename Mat, typename = std::enable_if_t<is_matrix_v<Mat>>>
Mat operator*(
    const Mat &a,
    std::type_identity_t<typename matrix_parameters<Mat>::value_type> val) {
  return a.mult(val);
}

template <typename Mat, typename = std::enable_if_t<is_matrix_v<Mat>>>
Mat operator*(
    std::type_identity_t<typename matrix_parameters<Mat>::value_type> val,
    const Mat &a) {
  return a.mult(val);
}

template <typename T, std::size_t Rows, std::size_t Cols> class matrix {};

template <std::size_t Rows, std::size_t Cols>
class matrix<float, Rows, Cols>
    : public base_matrix<matrix<float, Rows, Cols>> {
public:
  using value_type = base_matrix<matrix<float, Rows, Cols>>::value_type;
  static constexpr std::size_t row_value = Rows;
  static constexpr std::size_t col_value = Cols;

  constexpr matrix() = default;
  constexpr ~matrix() noexcept = default;

  constexpr matrix(const matrix &other) {
    std::copy_n(other.m_data, Rows * Cols, m_data);
  }

  constexpr matrix(matrix &&other) noexcept {
    std::copy_n(other.m_data, Rows * Cols, m_data);
  }

  explicit constexpr matrix(std::initializer_list<value_type> list) {
    assert(list.size() <= Rows * Cols &&
           "Initializer list size exceeds matrix capacity");
    std::size_t iter = 0;
    for (const value_type &d : list) {
      if (iter >= Rows * Cols) {
        break;
      }
      m_data[iter++] = d;
    }
  }

protected:
  friend base_matrix<matrix<value_type, Rows, Cols>>;

  matrix add_impl(const matrix &other) const {
    auto a = this->get_handle();
    auto b = other.get_handle();
    matrix res;
    auto c = res.get_handle();
    arm_mat_add_f32(&a, &b, &c);
    return res;
  }

  matrix sub_impl(const matrix &other) const {
    auto a = this->get_handle();
    auto b = other.get_handle();
    matrix res;
    auto c = res.get_handle();
    arm_mat_sub_f32(&a, &b, &c);
    return res;
  }

  template <typename Ty, typename = std::enable_if_t<is_scalar_v<Ty>>>
  matrix mult_impl(Ty val) const {
    auto a = this->get_handle();
    matrix res;
    auto c = res.get_handle();
    arm_mat_scale_f32(&a, static_cast<value_type>(val), &c);
    return res;
  }

  template <std::size_t ResCols>
  matrix<value_type, Rows, ResCols>
  mult_impl(const matrix<value_type, Cols, ResCols> &other) const {
    auto a = this->get_handle();
    auto b = other.get_handle();
    matrix<value_type, Rows, ResCols> res;
    auto c = res.get_handle();
    arm_mat_mult_f32(&a, &b, &c);
    return res;
  }

  matrix inverse_impl() const {
    matrix tmp{*this};
    matrix res;
    auto a = tmp.get_handle();
    auto c = res.get_handle();
    arm_status status = arm_mat_inverse_f32(&a, &c);
    assert(status == ARM_MATH_SUCCESS &&
           "Matrix inversion failed - matrix may be singular");
    if (status != ARM_MATH_SUCCESS) {
      return matrix{};
    }
    return res;
  }

  matrix<value_type, Cols, Rows> transpose_impl() const {
    auto a = this->get_handle();
    matrix<value_type, Cols, Rows> res;
    auto c = res.get_handle();
    arm_mat_trans_f32(&a, &c);
    return res;
  }

  value_type *get_impl() { return m_data; }

  const value_type *get_impl() const { return m_data; }

  arm_matrix_instance_f32 get_handle_impl() const {
    return {Rows, Cols, const_cast<value_type *>(m_data)};
  }

  value_type &get_value_impl(std::size_t i, std::size_t j) {
    return m_data[Cols * i + j];
  }

  const value_type &get_value_impl(std::size_t i, std::size_t j) const {
    return m_data[Cols * i + j];
  }

private:
  value_type m_data[Rows * Cols] = {};
};

template <std::size_t Rows, std::size_t Cols>
class matrix<double, Rows, Cols>
    : public base_matrix<matrix<double, Rows, Cols>> {
public:
  using value_type = double;
  constexpr matrix() = default;
  constexpr ~matrix() noexcept = default;

  constexpr matrix(const matrix &other) {
    std::copy_n(other.m_data, Rows * Cols, m_data);
  }

  constexpr matrix(matrix &&other) noexcept {
    std::copy_n(other.m_data, Rows * Cols, m_data);
  }

  explicit constexpr matrix(std::initializer_list<value_type> list) {
    assert(list.size() <= Rows * Cols &&
           "Initializer list size exceeds matrix capacity");
    std::size_t iter = 0;
    for (const value_type &d : list) {
      if (iter >= Rows * Cols) {
        break;
      }
      m_data[iter++] = d;
    }
  }

protected:
  friend base_matrix<matrix<value_type, Rows, Cols>>;

  matrix add_impl(const matrix &other) const {
    auto a = this->get_handle();
    auto b = other.get_handle();
    matrix res;
    auto c = res.get_handle();
    arm_mat_add_f64(&a, &b, &c);
    return res;
  }

  matrix sub_impl(const matrix &other) const {
    auto a = this->get_handle();
    auto b = other.get_handle();
    matrix res;
    auto c = res.get_handle();
    arm_mat_sub_f64(&a, &b, &c);
    return res;
  }

  template <typename Ty, typename = std::enable_if_t<is_scalar_v<Ty>>>
  matrix mult_impl(Ty val) const {
    auto a = this->get_handle();
    matrix res;
    auto c = res.get_handle();
    arm_mat_scale_f64(&a, static_cast<value_type>(val), &c);
    return res;
  }

  template <std::size_t ResCols>
  matrix<value_type, Rows, ResCols>
  mult_impl(const matrix<value_type, Cols, ResCols> &other) const {
    auto a = this->get_handle();
    auto b = other.get_handle();
    matrix<value_type, Rows, ResCols> res;
    auto c = res.get_handle();
    arm_mat_mult_f64(&a, &b, &c);
    return res;
  }

  matrix inverse_impl() const {
    matrix tmp{*this};
    matrix res;
    auto a = tmp.get_handle();
    auto c = res.get_handle();
    arm_status status = arm_mat_inverse_f64(&a, &c);
    assert(status == ARM_MATH_SUCCESS &&
           "Matrix inversion failed - matrix may be singular");
    if (status != ARM_MATH_SUCCESS) {
      return matrix{};
    }
    return res;
  }

  matrix<value_type, Cols, Rows> transpose_impl() const {
    auto a = this->get_handle();
    matrix<value_type, Cols, Rows> res;
    auto c = res.get_handle();
    arm_mat_trans_f64(&a, &c);
    return res;
  }

  value_type *get_impl() { return m_data; }

  const value_type *get_impl() const { return m_data; }

  arm_matrix_instance_f64 get_handle_impl() const {
    return {Rows, Cols, const_cast<value_type *>(m_data)};
  }

  value_type &get_value_impl(std::size_t i, std::size_t j) {
    return m_data[Cols * i + j];
  }

  const value_type &get_value_impl(std::size_t i, std::size_t j) const {
    return m_data[Cols * i + j];
  }

private:
  value_type m_data[Rows * Cols] = {};
};

template <typename T, std::size_t Rows> using vector = matrix<T, Rows, 1>;

template <typename T, std::size_t Rows>
inline constexpr T dot(const vector<T, Rows> &a, const vector<T, Rows> &b) {
  T result = T{};
  for (std::size_t i = 0; i < Rows; ++i) {
    result += a[i] * b[i];
  }
  return result;
}

template <typename T, std::size_t Rows>
inline constexpr T operator*(const vector<T, Rows> &a,
                             const vector<T, Rows> &b) {
  return dot(a, b);
}

template <typename T>
inline constexpr vector<T, 3> cross(const vector<T, 3> &a,
                                    const vector<T, 3> &b) {
  vector<T, 3> res;
  res[0] = a[1] * b[2] - a[2] * b[1];
  res[1] = a[2] * b[0] - a[0] * b[2];
  res[2] = a[0] * b[1] - a[1] * b[0];
  return res;
}

template <typename T>
inline constexpr matrix<T, 4, 4> make_scale(std::type_identity_t<T> scale) {
  matrix<T, 4, 4> res;
  res[0, 0] = scale;
  res[1, 1] = scale;
  res[2, 2] = scale;
  res[3, 3] = 1;
  return res;
}

template <typename T>
inline constexpr matrix<T, 4, 4> make_translate(const vector<T, 3> &vec) {
  matrix<T, 4, 4> res = matrix<T, 4, 4>::identity();
  res[0, 3] = vec[0];
  res[1, 3] = vec[1];
  res[2, 3] = vec[2];
  res[3, 3] = 1;
  return res;
}

template <typename T>
inline constexpr matrix<T, 4, 4> make_rotate(const vector<T, 3> &vec,
                                             std::type_identity_t<T> angle) {
  matrix<T, 4, 4> res;
  vector<T, 3> temp = vec.normalized();
  T c = std::cos(angle);
  T s = std::sin(angle);
  T osc = 1 - c;
  res[0, 0] = c + temp[0] * temp[0] * osc;
  res[0, 1] = temp[0] * temp[1] * osc - temp[2] * s;
  res[0, 2] = temp[0] * temp[2] * osc + temp[1] * s;
  res[1, 0] = temp[1] * temp[0] * osc + temp[2] * s;
  res[1, 1] = c + temp[1] * temp[1] * osc;
  res[1, 2] = temp[1] * temp[2] * osc - temp[0] * s;
  res[2, 0] = temp[2] * temp[0] * osc - temp[1] * s;
  res[2, 1] = temp[2] * temp[1] * osc + temp[0] * s;
  res[2, 2] = c + temp[2] * temp[2] * osc;
  res[3, 3] = 1;
  return res;
}

} // namespace gdut

#endif // MATRIX_HPP
