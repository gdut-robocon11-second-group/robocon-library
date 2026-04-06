# DSP 矩阵模块（`matrix.hpp`）

## 概览

这个模块提供编译期固定维度的矩阵和向量类型，位于 `namespace gdut`。实现方式是：

- 以 `matrix<T, Rows, Cols>` 作为核心类型
- 以 `vector<T, Rows>` 作为列向量别名
- 对 `float` 和 `double` 两个特化版本提供完整实现
- 逆矩阵计算对接 CMSIS-DSP，常规加减乘/转置使用模板循环实现
- 通过 CRTP 基类 `base_matrix<Matrix<T, Rows, Cols>>` 提供通用接口

矩阵数据存放在栈上定长数组中，采用行优先（row-major）布局，尺寸和类型约束都在编译期完成。

## 设计结构

### `base_matrix<Derived>`

`base_matrix` 是一个 CRTP 基类，派生类需要实现若干 `*_impl` 接口，基类再统一提供高层 API。

当前基类对外提供的能力包括：

| 功能 | 方法 | 说明 |
|------|------|------|
| 加法 | `add(other)` / `+` | 同维度矩阵逐元素相加 |
| 减法 | `sub(other)` / `-` | 同维度矩阵逐元素相减 |
| 标量乘法 | `mult(scalar)` / `*` | 每个元素乘以标量 |
| 矩阵乘法 | `mult(other)` / `*` | `(M×K) * (K×N) -> (M×N)` |
| 行列式 | `det()` | 仅方阵可用 |
| 可逆判断 | `is_invertible()` | 使用 `|det| > epsilon` 判断 |
| 逆矩阵 | `inverse()` | 仅方阵可用，交给 CMSIS-DSP 求逆 |
| 转置 | `transpose()` | 返回转置后的新矩阵 |
| 范数 | `norm()` | Frobenius 范数 |
| 归一化 | `normalized()` | 按 Frobenius 范数归一化，范数过小时直接返回自身副本 |
| 单位矩阵 | `identity()` | 静态成员函数，仅方阵可用 |
| 全零矩阵 | `zeros()` | 静态成员函数，返回同维度零矩阵 |
| 全一矩阵 | `ones()` | 静态成员函数，返回同维度全一矩阵 |
| 元素访问 | `get_value(i, j)` / `operator[](i, j)` | 二维访问，零起始 |
| 向量访问 | `operator[](i)` | 仅列向量可用 |
| 原始数据 | `get()` | 返回内部数组指针 |
| CMSIS 句柄 | `get_handle()` | 返回对应的 `arm_matrix_instance_f32/f64` |

### `matrix<T, Rows, Cols>`

`matrix` 只有两个实际可用的特化：

- `matrix<float, Rows, Cols>`
- `matrix<double, Rows, Cols>`

其它类型的主模板只是占位声明，不提供实际成员。

构造和存储特性：

- 默认构造后数据为零初始化
- 支持 `build_and_clean_mat` / `build_but_not_clean_mat` 构造标签控制初始化策略
- 支持用 `std::initializer_list` 进行线性初始化
- 数据按行优先顺序存储在 `m_data[Rows * Cols]` 中
- `get_handle()` 返回的 CMSIS 句柄直接指向内部数据

### `vector<T, Rows>`

```cpp
template <typename T, std::size_t Rows>
using vector = matrix<T, Rows, 1>;
```

也就是说，向量本质上是列矩阵。对向量可使用单下标访问 `v[i]`，它等价于 `v[i, 0]`。

### 行列式算法

| 维度 | 算法 |
|------|------|
| `1 × 1` | 直接返回元素 |
| `2 × 2` | `ad - bc` |
| `3 × 3` | 按展开式直接计算 |
| `4 × 4` | 展开后的显式公式 |
| `N × N`，`N >= 5` | 带部分主元选取的 LU 分解，复杂度 `O(N^3)` |

## 向量与变换函数

### 点积与叉积

```cpp
T dot(const vector<T, Rows>& a, const vector<T, Rows>& b);
T operator*(const vector<T, Rows>& a, const vector<T, Rows>& b); // 仅 Rows != 1
vector<T, 3> cross(const vector<T, 3>& a, const vector<T, 3>& b);
```

- `dot` 和 `vector * vector` 都表示点积
- `vector * vector` 点积仅对 `Rows != 1` 启用
- `cross` 只对 3 维向量提供叉积

### 4×4 变换矩阵

```cpp
matrix<T, 4, 4> make_scale(std::type_identity_t<T> scale);
matrix<T, 4, 4> make_translate(const vector<T, 3>& vec);
matrix<T, 4, 4> make_rotate(const vector<T, 3>& axis, std::type_identity_t<T> radian);
```

这些函数都返回 `matrix<T, 4, 4>`：

- `make_scale`：均匀缩放，前三个对角元素为缩放值，右下角为 `1`
- `make_translate`：平移矩阵，基于单位矩阵，在最后一列填入位移
- `make_rotate`：绕任意轴旋转，内部会先对轴向量做归一化，再套用 Rodrigues 公式

组合变换时使用右乘约定：

$$\mathbf{v}' = M_{\text{translate}} \cdot M_{\text{rotate}} \cdot M_{\text{scale}} \cdot \mathbf{v}$$

## 使用示例

### 基本矩阵运算

```cpp
#include "matrix.hpp"

gdut::matrix<float, 3, 4> a{1, 2, 3, 4,
                            5, 6, 7, 8,
                            9, 10, 11, 12};

gdut::matrix<float, 4, 2> b{1, 2, 3, 4, 5, 6, 7, 8};
auto c = a * b;

auto d = c * 2.0f;
auto e = 2.0f * c;

auto f = c + d;
auto g = c - d;

auto z = gdut::matrix<float, 2, 3>::zeros();
auto o = gdut::matrix<float, 2, 3>::ones();

// 构造时不初始化矩阵，由用户负责填充数据
// 同时提高运算性能，避免不必要的内存写入
gdut::matrix<float, 2, 2> fast_tmp{gdut::build_but_not_clean_mat};
fast_tmp[0, 0] = 1.0f;
fast_tmp[0, 1] = 2.0f;
fast_tmp[1, 0] = 3.0f;
fast_tmp[1, 1] = 4.0f;
```

### 方阵操作

```cpp
auto m = gdut::matrix<float, 3, 3>{1, 2, 3,
                                   0, 1, 4,
                                   5, 6, 0};

float det_value = m.det();

if (m.is_invertible()) {
    auto inv = m.inverse();
    auto eye = m * inv;
}

auto t = m.transpose();
auto i = gdut::matrix<float, 4, 4>::identity();
```

### 元素访问

```cpp
auto m = gdut::matrix<float, 2, 3>{1, 2, 3, 4, 5, 6};

float val = m[0, 2];
m[1, 0] = 10.0f;

const auto& cm = m;
float cval = cm[1, 2];
```

### 向量与叉积

```cpp
using vec3f = gdut::vector<float, 3>;

vec3f x{1.0f, 0.0f, 0.0f};
vec3f y{0.0f, 1.0f, 0.0f};

float dp = gdut::dot(x, y);
vec3f cp = gdut::cross(x, y);

float n = x.norm();

gdut::vector<float, 1> v1{2.0f};
gdut::vector<float, 1> v2{3.0f};
// 1维向量使用 dot，避免与 1x1 矩阵乘法冲突
float dp1 = gdut::dot(v1, v2);
```

### 3D 变换

```cpp
#include "matrix.hpp"
#include <numbers>

auto s = gdut::make_scale<float>(5.0f);
auto t = gdut::make_translate(gdut::vector<float, 3>{1.0f, 2.0f, 3.0f});
auto r = gdut::make_rotate(gdut::vector<float, 3>{0.0f, 1.0f, 0.0f},
                           std::numbers::pi_v<float> / 2.0f);

gdut::vector<float, 4> v{1.0f, 1.0f, 1.0f, 1.0f};
auto result = t * r * s * v;
```

### 与 CMSIS-DSP 互操作

```cpp
auto m = gdut::matrix<float, 3, 3>::identity();

auto handle = m.get_handle();
float* data = m.get();
```

### 使用 double 精度

```cpp
auto dm = gdut::matrix<double, 4, 4>::identity();
auto dinv = dm.inverse();
```

## 设计细节

- **命名空间**：主实现位于 `gdut`
- **支持类型**：只对 `float` 和 `double` 提供完整实现
- **维度检查**：矩阵乘法会在编译期检查左矩阵列数和右矩阵行数是否匹配，并检查元素类型一致
- **内存布局**：行优先布局，与 CMSIS-DSP 兼容
- **性能语义**：默认构造清零；内部运算临时对象可使用“未清零构造”减少额外写内存
- **`operator[]`**：二维访问使用 C++23 的多参数下标语法；向量还支持单参数访问
- **1 维兼容性**：`vector<T,1>` 不启用 `operator*(vector, vector)` 点乘重载，避免与 `1x1` 矩阵乘法冲突
- **`make_rotate`**：会自动对轴向量做归一化，零向量属于未定义输入
- **`inverse()`**：底层直接调用 CMSIS-DSP 的求逆接口，浮点精度不足时建议使用 `double`
- **`det()`**：高维方阵使用带部分主元的 LU 分解，若主元接近 0 会提前返回 0

## 类型特征工具

```cpp
gdut::is_scalar_v<T>
gdut::is_matrix_v<Mat>
gdut::redefine_matrix_t<Mat, NewRows, NewCols>
```

- `is_scalar_v<T>`：判断是否为整型或浮点型
- `is_matrix_v<Mat>`：判断是否为当前矩阵体系中的类型
- `redefine_matrix_t<Mat, NewRows, NewCols>`：保留原值类型，只替换矩阵维度

相关源码：[DSP/matrix.hpp](../../DSP/matrix.hpp)
