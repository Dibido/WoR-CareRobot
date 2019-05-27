#include <cassert>
#include <cmath>
#include <iomanip>
#include <numeric>
#include <stdexcept>
#include <utility>

/**
 *
 */
template <class T, std::size_t M, std::size_t N>
Matrix<T, M, N>::Matrix(T value)
{
  for (std::size_t row = 0; row < M; ++row)
  {
    for (std::size_t column = 0; column < N; ++column)
    {
      matrix.at(row).at(column) = value;
    }
  }
}
/**
 *
 */
template <class T, std::size_t M, std::size_t N>
Matrix<T, M, N>::Matrix(const std::initializer_list<T>& aList)
{
  // Check the arguments
  assert(aList.size() == M * N);

  auto row_iter = aList.begin();
  for (std::size_t row = 0; row < M; ++row)
  {
    for (std::size_t column = 0; column < N; ++column, ++row_iter)
    {
      matrix.at(row).at(column) = *row_iter;
    }
  }
}
/**
 *
 */
template <class T, std::size_t M, std::size_t N>
Matrix<T, M, N>::Matrix(
    const std::initializer_list<std::initializer_list<T>>& aList)
{
  // Check the arguments, the static assert assures that there is at
  // least 1 M and 1 N!
  assert(aList.size() == M && (*aList.begin()).size() == N);

  auto row_iter = aList.begin();
  for (std::size_t row = 0; row < aList.size(); ++row, ++row_iter)
  {
    auto column_iter = (*row_iter).begin();
    for (std::size_t column = 0; column < (*row_iter).size();
         ++column, ++column_iter)
    {
      matrix.at(row).at(column) = *column_iter;
    }
  }
}
/**
 *
 */
template <class T, std::size_t M, std::size_t N>
Matrix<T, M, N>::Matrix(const Matrix<T, M, N>& aMatrix) : matrix(aMatrix.matrix)
{
}
/**
 *
 */
template <class T, std::size_t M, std::size_t N>
std::array<T, N>& Matrix<T, M, N>::at(std::size_t aRowIndex)
{
  return matrix.at(aRowIndex);
}
/**
 *
 */
template <class T, std::size_t M, std::size_t N>
const std::array<T, N>& Matrix<T, M, N>::at(std::size_t aRowIndex) const
{
  return matrix.at(aRowIndex);
}
/**
 *
 */
template <class T, std::size_t M, std::size_t N>
T& Matrix<T, M, N>::at(std::size_t aRowIndex, std::size_t aColumnIndex)
{
  return matrix.at(aRowIndex).at(aColumnIndex);
}
/**
 *
 */
template <class T, std::size_t M, std::size_t N>
const T& Matrix<T, M, N>::at(std::size_t aRowIndex,
                             std::size_t aColumnIndex) const
{
  return matrix.at(aRowIndex).at(aColumnIndex);
}
/**
 *
 */
template <class T, std::size_t M, std::size_t N>
std::array<T, N>& Matrix<T, M, N>::operator[](std::size_t aRowIndex)
{
  return matrix[aRowIndex];
}
/**
 *
 */
template <class T, std::size_t M, std::size_t N>
const std::array<T, N>& Matrix<T, M, N>::operator[](std::size_t aRowIndex) const
{
  return matrix[aRowIndex];
}
/**
 *
 */
template <class T, std::size_t M, std::size_t N>
Matrix<T, M, N>& Matrix<T, M, N>::operator=(const Matrix<T, M, N>& rhs)
{
  if (this != &rhs)
  {
    matrix = rhs.matrix;
  }
  return *this;
}
/**
 *
 */
template <class T, std::size_t M, std::size_t N>
bool Matrix<T, M, N>::operator==(const Matrix<T, M, N>& rhs) const
{
  return matrix == rhs.matrix;
}
/**
 *
 */
template <class T, std::size_t M, std::size_t N>
bool Matrix<T, M, N>::operator!=(const Matrix<T, M, N>& rhs) const
{
  return !operator==(rhs);
}
/**
 *
 */
template <class T, std::size_t M, std::size_t N>
template <class T2>
Matrix<T, M, N>& Matrix<T, M, N>::operator*=(const T2& scalar)
{
  static_assert(std::is_arithmetic<T2>::value,
                "Value T2 must be arithmetic, see "
                "http://en.cppreference.com/w/cpp/types/is_arithmetic");

  for (std::size_t row = 0; row < M; ++row)
  {
    for (std::size_t column = 0; column < N; ++column)
    {
      matrix.at(row).at(column) *= scalar;
    }
  }
  return *this;
}
/**
 *
 */
template <class T, std::size_t M, std::size_t N>
template <class T2>
Matrix<T, M, N> Matrix<T, M, N>::operator*(const T2& scalar) const
{
  static_assert(std::is_arithmetic<T2>::value,
                "Value T2 must be arithmetic, see "
                "http://en.cppreference.com/w/cpp/types/is_arithmetic");

  Matrix<T, M, N> result(*this);
  return result *= scalar;
}
/**
 *
 */
template <class T, std::size_t M, std::size_t N>
template <class T2>
Matrix<T, M, N>& Matrix<T, M, N>::operator/=(const T2& aScalar)
{
  static_assert(std::is_arithmetic<T2>::value,
                "Value T2 must be arithmetic, see "
                "http://en.cppreference.com/w/cpp/types/is_arithmetic");

  for (std::size_t row = 0; row < M; ++row)
  {
    for (std::size_t column = 0; column < N; ++column)
    {
      matrix.at(row).at(column) /= aScalar;
    }
  }
  return *this;
}
/**
 *
 */
template <class T, std::size_t M, std::size_t N>
template <class T2>
Matrix<T, M, N> Matrix<T, M, N>::operator/(const T2& aScalar) const
{
  static_assert(std::is_arithmetic<T2>::value,
                "Value T2 must be arithmetic, see "
                "http://en.cppreference.com/w/cpp/types/is_arithmetic");

  Matrix<T, M, N> result(*this);
  return result /= aScalar;
}
/**
 *
 */
template <class T, std::size_t M, std::size_t N>
Matrix<T, M, N>& Matrix<T, M, N>::operator+=(const Matrix<T, M, N>& rhs)
{
  for (std::size_t row = 0; row < M; ++row)
  {
    for (std::size_t column = 0; column < N; ++column)
    {
      matrix[row][column] += rhs.at(row, column);
    }
  }
  return *this;
}
/**
 *
 */
template <class T, std::size_t M, std::size_t N>
Matrix<T, M, N> Matrix<T, M, N>::operator+(const Matrix<T, M, N>& rhs) const
{
  Matrix<T, M, N> result(*this);
  return result += rhs;
}
/**
 *
 */
template <class T, std::size_t M, std::size_t N>
Matrix<T, M, N>& Matrix<T, M, N>::operator-=(const Matrix<T, M, N>& rhs)
{
  for (std::size_t row = 0; row < M; ++row)
  {
    for (std::size_t column = 0; column < N; ++column)
    {
      matrix[row][column] -= rhs.at(row, column);
    }
  }
  return *this;
}
/**
 *
 */
template <class T, std::size_t M, std::size_t N>
Matrix<T, M, N> Matrix<T, M, N>::operator-(const Matrix<T, M, N>& rhs) const
{
  Matrix<T, M, N> result(*this);
  return result -= rhs;
}
/**
 * (M, N) * (N, P) -> (M, P)
 */
template <typename T, std::size_t M, std::size_t N>
template <std::size_t columns>
Matrix<T, M, columns> Matrix<T, M, N>::
    operator*(const Matrix<T, N, columns>& rhs) const
{
  Matrix<T, M, columns> result;

  for (std::size_t row = 0; row < M; ++row)
  {
    for (std::size_t column = 0; column < columns; ++column)
    {
      for (std::size_t codependent = 0; codependent < N; ++codependent)
      {
        result[row][column] +=
            matrix[row][codependent] * rhs[codependent][column];
      }
    }
  }
  return result;
}
/**
 *
 */
template <class T, std::size_t M, std::size_t N>
Matrix<T, N, M> Matrix<T, M, N>::transpose() const
{
  Matrix<T, N, M> result;
  for (std::size_t n = 0; n < N; ++n)
  {
    for (std::size_t m = 0; m < M; ++m)
    {
      result.at(n).at(m) = matrix.at(m).at(n);
    }
  }
  return result;
}
/**
 *
 */
template <class T, std::size_t M, std::size_t N>
Matrix<T, N, N> Matrix<T, M, N>::identity() const
{
  Matrix<T, N, N> result;
  for (std::size_t i = 0; i < N; ++i)
  {
    result[i][i] = 1;
  }
  return result;
}
/**
 *
 */
template <class T, std::size_t M, std::size_t N>
Matrix<T, M, N> Matrix<T, M, N>::gauss() const
{
  Matrix<T, M, N> result(*this);

  for (std::size_t row = 0; row < M; ++row)
  {

    T max = 0;
    std::size_t maxRow = row;
    // Move maximum pivot value to top
    for (std::size_t subRow = maxRow + 1; subRow < M; ++subRow)
    {
      if (matrix[subRow][row] > max)
      {
        max = matrix[subRow][row];
        maxRow = subRow;
      }
    }
    std::swap(result[row], result[maxRow]);

    T factor = 0;
    for (std::size_t subRow = row; subRow < M; ++subRow)
    {
      if (row == subRow)
      {
        // calculate factor
        factor = result[row][row];
        for (std::size_t col = subRow; col < N; ++col)
        {
          result[subRow][col] /= factor;
        }
      }
      else
      {
        factor = result[subRow][row] / result[row][row];
        for (std::size_t subCol = 0; subCol < N; ++subCol)
        {
          result[subRow][subCol] -= result[row][subCol] * factor;
        }
      }
    }
  }
  return result;
}
/**
 *
 */
template <class T, std::size_t M, std::size_t N>
Matrix<T, M, N> Matrix<T, M, N>::gaussJordan() const
{
  Matrix<T, M, N> result(*this);

  for (std::size_t row = 0; row < M; ++row)
  {
    std::size_t maxRow = row;
    // Move maximum pivot value to top
    for (std::size_t subRow = maxRow + 1; subRow < M; ++subRow)
    {
      if (matrix[subRow][row] > matrix[maxRow][row])
      {
        maxRow = subRow;
      }
    }
    std::swap(result[row], result[maxRow]);
    T gaussFactor = result[row][row];
    for (std::size_t col = 0; col < N; ++col)
    {
      result[row][col] /= gaussFactor;
    }
    for (std::size_t subRow = row; subRow < M; ++subRow)
    {
      if (row != subRow)
      {
        gaussFactor = result[subRow][row] / result[row][row];
        for (std::size_t subCol = 0; subCol < N; ++subCol)
        {
          result[subRow][subCol] -= result[row][subCol] * gaussFactor;
        }
        T jordanFactor = result[row][subRow] / result[subRow][subRow];
        for (std::size_t subCol = 0; subCol < N; ++subCol)
        {
          result[row][subCol] -= result[subRow][subCol] * jordanFactor;
        }
      }
    }
  }
  return result;
}
/**
 *
 */
template <class T, std::size_t M, std::size_t N>
Matrix<T, M, 1> Matrix<T, M, N>::solve() const
{
  Matrix<T, M, 1> result;
  std::size_t n = N - 1;
  auto gj(this->gaussJordan());
  for (std::size_t m = 0; m < M; ++m)
  {
    result[m][0] = gj[m][n];
  }
  return result;
}
/**
 *
 */
template <class T, std::size_t M, std::size_t N>
Matrix<T, M, N> Matrix<T, M, N>::inverse() const
{
  static_assert(M == N);

  Matrix<T, M, N> result(*this);
  Matrix<T, M, N> inverse = this->identity();

  for (std::size_t row = 0; row < M; ++row)
  {
    std::size_t maxRow = row;
    // Move maximum pivot value to top
    for (std::size_t subRow = maxRow + 1; subRow < M; ++subRow)
    {
      if (matrix[subRow][row] > matrix[maxRow][row])
      {
        maxRow = subRow;
      }
    }
    std::swap(result[row], result[maxRow]);
    std::swap(inverse[maxRow], inverse[row]);

    if (result[row][row] == 0)
    {
      result[row][row] = 1;
    }
    T gaussFactor = result[row][row];

    for (std::size_t col = 0; col < N; ++col)
    {
      result[row][col] /= gaussFactor;
      inverse[row][col] /= gaussFactor;
    }
    for (std::size_t subRow = 0; subRow < M; ++subRow)
    {
      if (subRow != row)
      {

        gaussFactor = result[subRow][row] / result[row][row];
        for (std::size_t subCol = 0; subCol < N; ++subCol)
        {
          result[subRow][subCol] -= result[row][subCol] * gaussFactor;
          inverse[subRow][subCol] -= inverse[row][subCol] * gaussFactor;
        }

        if (result[subRow][subRow] == 0)
        {
          result[subRow][subRow] = 1;
        }
        T jordanFactor = result[row][subRow] / result[subRow][subRow];

        for (std::size_t subCol = 0; subCol < N; ++subCol)
        {
          result[row][subCol] -= result[subRow][subCol] * jordanFactor;
          inverse[row][subCol] -= inverse[subRow][subCol] * jordanFactor;
        }
      }
    }
  }
  return inverse;
}
/**
 *
 */
template <class T, std::size_t M, std::size_t N>
Matrix<T, N, M> Matrix<T, M, N>::pseudoInverse() const
{
  static_assert(M < N, "Only right pseudo inverse is implemented.");
  // Right inverse
  return this->transpose() * (*this * this->transpose()).inverse();
}
/**
 *
 */
template <class T, std::size_t M, std::size_t N>
std::string Matrix<T, M, N>::to_string() const
{
  std::string result =
      "Matrix<" + std::to_string(M) + "," + std::to_string(N) + ">\n{\n";
  for (std::size_t i = 0; i < M; ++i)
  {
    for (std::size_t j = 0; j < N; ++j)
    {
      result += std::to_string(matrix[i][j]) + ",";
    }
    result += "\n";
  }
  result += "}";
  return result;
}

/**
 *
 */
template <typename T, const std::size_t M, const std::size_t N>
bool equals(const Matrix<T, M, N>& lhs,
            const Matrix<T, M, N>& rhs,
            const T aPrecision /*= std::numeric_limits<T>::epsilon()*/,
            const unsigned long aFactor /*= 1*/)
{

  T actPrecision = aPrecision * ( T )aFactor;
  for (std::size_t m = 0; m < M; ++m)
  {
    for (std::size_t n = 0; n < N; ++n)
    {
      T lhsVal = lhs[m][n];
      T rhsVal = rhs[m][n];
      if (lhsVal != lhsVal || rhsVal != rhsVal ||
          (lhsVal < rhsVal - actPrecision) || (lhsVal > rhsVal + actPrecision))
      {
        return false;
      }
    }
  }
  return true;
}

template <typename T, std::size_t M, std::size_t N>
Matrix<T, M, N> Matrix<T, M, N>::dot(const Matrix<T, M, N>& rhs) const
{
  static_assert(M == 1 || N == 1,
                "DotProduct can only multiply two column- or row-vectors");

  Matrix<T, M, N> result(*this);

  for (std::size_t m = 0; m < M; ++m)
  {
    for (std::size_t n = 0; n < N; ++n)
    {
      result[m][n] = matrix[m][n] * rhs[m][n];
    }
  }
  return result;
}

template <typename T, std::size_t M, std::size_t N>
T Matrix<T, M, N>::dotSum() const
{

  static_assert(
      M == 1 || N == 1,
      "Sum of a DotProduct can only multiply two column- or row-vectors");

  T sum;

  for (std::size_t m = 0; m < M; ++m)
  {
    for (std::size_t n = 0; n < N; ++n)
    {
      sum += matrix[m][n];
    }
  }
  return sum;
}

template <typename T, std::size_t M, std::size_t N>
T cosineSimilarity(const Matrix<T, M, N>& lhs, const Matrix<T, M, N>& rhs)
{
  static_assert(
      M == 1 || N == 1,
      "Cosine Similarity can only be performed on two column- or row-vectors");

  Matrix<T, M, N> dotProduct = lhs.dot(rhs);
  double dotSum = dotProduct.dotSum();

  double sumLhs;
  double sumRhs;

  for (std::size_t m = 0; m < M; ++m)
  {
    for (std::size_t n = 0; n < N; ++n)
    {
      sumLhs += std::pow(lhs[m][n], 2);
      sumRhs += std::pow(rhs[m][n], 2);
    }
  }

  double test = std::sqrt(sumLhs) * std::sqrt(sumRhs);

  return dotSum / test;
}
