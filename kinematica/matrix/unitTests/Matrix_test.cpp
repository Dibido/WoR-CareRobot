#include "matrix/Matrix.hpp"
#include <limits>
#include <string>

#include "gtest/gtest.h"

TEST(MatrixConstructors, DefaultContructor)
{
  std::string m0_as_string(
      "Matrix<3,3>\n"
      "{\n"
      "0.000000,0.000000,0.000000,\n"
      "0.000000,0.000000,0.000000,\n"
      "0.000000,0.000000,0.000000,\n"
      "}");
  Matrix<double, 3, 3> m0;
  EXPECT_EQ(m0_as_string, m0.to_string());

  std::string m1_as_string(
      "Matrix<3,3>\n"
      "{\n"
      "1.000000,1.000000,1.000000,\n"
      "1.000000,1.000000,1.000000,\n"
      "1.000000,1.000000,1.000000,\n"
      "}");
  Matrix<double, 3, 3> m1(1);
  EXPECT_EQ(m1_as_string, m1.to_string());

  std::stringstream stream;
  stream << m1;
  EXPECT_EQ(stream.str(), m1.to_string());
}
TEST(MatrixConstructors, LinearConstructor)
{
  std::string m0_as_string(
      "Matrix<3,3>\n"
      "{\n"
      "1.000000,2.000000,3.000000,\n"
      "4.000000,5.000000,6.000000,\n"
      "7.000000,8.000000,9.000000,\n"
      "}");
  Matrix<double, 3, 3> m0{ 1, 2, 3, 4, 5, 6, 7, 8, 9 };
  EXPECT_EQ(m0_as_string, m0.to_string());
}
TEST(MatrixConstructors, ArrayListConstructor)
{
  std::string m0_as_string(
      "Matrix<3,3>\n"
      "{\n"
      "1.000000,2.000000,3.000000,\n"
      "4.000000,5.000000,6.000000,\n"
      "7.000000,8.000000,9.000000,\n"
      "}");
  Matrix<double, 3, 3> m0{ { 1, 2, 3 }, { 4, 5, 6 }, { 7, 8, 9 } };
  EXPECT_EQ(m0_as_string, m0.to_string());
}
TEST(MatrixConstructors, CopyConstructor)
{
  std::string m0_as_string(
      "Matrix<3,3>\n"
      "{\n"
      "1.000000,2.000000,3.000000,\n"
      "4.000000,5.000000,6.000000,\n"
      "7.000000,8.000000,9.000000,\n"
      "}");
  Matrix<double, 3, 3> m0{ { 1, 2, 3 }, { 4, 5, 6 }, { 7, 8, 9 } };

  Matrix<double, 3, 3> m1 = m0;

  EXPECT_EQ(m0.to_string(), m1.to_string());

  EXPECT_EQ(m0, m1);
}

TEST(MatrixElementAccess, At)
{
  Matrix<double, 3, 3> m0{ { 1, 2, 3 }, { 4, 5, 6 }, { 7, 8, 9 } };

  EXPECT_THROW(m0.at(m0.getRows() + 1), std::out_of_range);

  EXPECT_THROW(m0.at(m0.getRows(), m0.getColumns() + 1), std::out_of_range);

  EXPECT_NO_THROW(m0[m0.getRows() + 1]);
}

TEST(MatrixOperators, AssignmentOperator)
{
  Matrix<double, 3, 3> m0;
  Matrix<double, 3, 3> m1{ { 1, 2, 3 }, { 4, 5, 6 }, { 7, 8, 9 } };
  m0 = m1;
  EXPECT_EQ(m0, m1);
}
TEST(MatrixOperators, ComparisonOperator)
{
  Matrix<double, 3, 3> m0{ { 1, 2, 3 }, { 4, 5, 6 }, { 7, 8, 9 } };
  Matrix<double, 3, 3> m1{ { 1, 2, 3 }, { 4, 5, 6 }, { 7, 8, 9 } };
  EXPECT_EQ(m0, m1);

  Matrix<double, 3, 3> m2{ { 1, 2, 3 }, { 4, 5, 6 }, { 7, 8, 9 } };
  Matrix<double, 3, 3> m3{ { 9, 8, 7 }, { 6, 5, 4 }, { 3, 2, 1 } };
  EXPECT_NE(m2, m3);
}
TEST(MatrixOperators, ScalarMultiplication)
{
  Matrix<double, 3, 3> m0 = { { 1, 2, 3 }, { 4, 5, 6 }, { 7, 8, 9 } };
  Matrix<double, 3, 3> m1 = { { 1, 2, 3 }, { 4, 5, 6 }, { 7, 8, 9 } };
  Matrix<double, 3, 3> m2{ { 1 * 2, 2 * 2, 3 * 2 },
                           { 4 * 2, 5 * 2, 6 * 2 },
                           { 7 * 2, 8 * 2, 9 * 2 } };

  EXPECT_EQ(m2, m1 * 2);
  EXPECT_EQ(m0, m1);
  EXPECT_EQ(m2, m1 *= 2);
  EXPECT_EQ(m2, m1);
}
TEST(MatrixOperators, ScalarDivision)
{
  Matrix<double, 3, 3> m0{ { 1 * 2, 2 * 2, 3 * 2 },
                           { 4 * 2, 5 * 2, 6 * 2 },
                           { 7 * 2, 8 * 2, 9 * 2 } };
  Matrix<double, 3, 3> m1{ { 1 * 2, 2 * 2, 3 * 2 },
                           { 4 * 2, 5 * 2, 6 * 2 },
                           { 7 * 2, 8 * 2, 9 * 2 } };
  Matrix<double, 3, 3> m2 = { { 1, 2, 3 }, { 4, 5, 6 }, { 7, 8, 9 } };

  EXPECT_EQ(m2, m1 / 2);
  EXPECT_EQ(m0, m1);
  EXPECT_EQ(m2, m1 /= 2);
  EXPECT_EQ(m2, m1);
}
TEST(MatrixOperators, MatrixAddition)
{
  Matrix<double, 3, 3> m0 = { { 1, 2, 3 }, { 4, 5, 6 }, { 7, 8, 9 } };
  Matrix<double, 3, 3> m1 = { { 1, 2, 3 }, { 4, 5, 6 }, { 7, 8, 9 } };
  Matrix<double, 3, 3> m2{ { 1 * 2, 2 * 2, 3 * 2 },
                           { 4 * 2, 5 * 2, 6 * 2 },
                           { 7 * 2, 8 * 2, 9 * 2 } };

  EXPECT_EQ(m2, m0 + m1);
  EXPECT_EQ(m0, m1);
  EXPECT_EQ(m2, m1 += m0);
  EXPECT_EQ(m2, m1);
}
TEST(MatrixOperators, MatrixSubtraction)
{
  Matrix<double, 3, 3> m0{ { 1 * 2, 2 * 2, 3 * 2 },
                           { 4 * 2, 5 * 2, 6 * 2 },
                           { 7 * 2, 8 * 2, 9 * 2 } };
  Matrix<double, 3, 3> m1 = { { 1, 2, 3 }, { 4, 5, 6 }, { 7, 8, 9 } };
  Matrix<double, 3, 3> m2 = { { 1, 2, 3 }, { 4, 5, 6 }, { 7, 8, 9 } };

  EXPECT_EQ(m2, m0 - m1);
  EXPECT_EQ(m2, m0 -= m1);
}
TEST(MatrixOperators, MatrixMatrixMultiplication)
{
  Matrix<double, 3, 3> m0{ { 1, 2, 3 }, { 4, 5, 6 }, { 7, 8, 9 } };
  Matrix<double, 3, 3> m1{ { 30, 36, 42 }, { 66, 81, 96 }, { 102, 126, 150 } };

  EXPECT_EQ(m1, m0 * m0);
}
TEST(MatrixOperators, MatrixColumnVectorMultiplication)
{
  Matrix<double, 3, 3> m0{ { 1, 2, 3 }, { 4, 5, 6 }, { 7, 8, 9 } };
  Matrix<double, 3, 1> m1{ { { 1 } }, { { 2 } }, { { 3 } } };
  Matrix<double, 3, 1> m2{ { { 14 } }, { { 32 } }, { { 50 } } };

  EXPECT_EQ(m2, m0 * m1);
}
TEST(MatrixOperators, MatrixRowVectorMultiplication)
{
  Matrix<double, 1, 3> m0; //{{1},{2},{3}};
  m0.at(0, 0) = 1;
  m0.at(0, 1) = 2;
  m0.at(0, 2) = 3;
  Matrix<double, 3, 1> m1; //{{1},{2},{3}};
  m1.at(0, 0) = 1;
  m1.at(1, 0) = 2;
  m1.at(2, 0) = 3;
  Matrix<double, 1, 1> m2;
  m2.at(0, 0) = 14;

  EXPECT_EQ(m2, m0 * m1);
}

TEST(MatrixFunctions, MatrixTranspose)
{
  // See https://en.wikipedia.org/wiki/Transpose
  Matrix<double, 3, 3> m0{ { 1, 2, 3 }, { 4, 5, 6 }, { 7, 8, 9 } };
  Matrix<double, 3, 3> m1{ { 9, 8, 7 }, { 6, 5, 4 }, { 3, 2, 1 } };

  EXPECT_EQ(m0, m0.transpose().transpose());
  EXPECT_EQ((m0 + m1).transpose(), m0.transpose() + m1.transpose());
  EXPECT_EQ((m0 * 4.0).transpose(), m0.transpose() * 4.0);
}
TEST(MatrixFunctions, MatrixIdentity)
{
  Matrix<double, 3, 3> m0{ { 1, 0, 0 }, { 0, 1, 0 }, { 0, 0, 1 } };
  Matrix<double, 3, 3> m1{ { 1, 2, 3 }, { 4, 5, 6 }, { 7, 8, 9 } };

  EXPECT_EQ(m0, m1.identity());
  EXPECT_EQ(m1, m1 * m1.identity());
  EXPECT_EQ(m1, m1.identity() * m1);
}

TEST(MatrixFunctions, MatrixGauss)
{
  Matrix<double, 3, 4> m0{ { 0, 1, 1, 5 }, { 3, 2, 2, 13 }, { 1, -1, 3, 8 } };
  Matrix<double, 3, 4> m1(m0.gauss());

  double c = m1.at(2, 3);
  double b = m1.at(1, 3) - m1.at(1, 2) * c;
  double a = m1.at(0, 3) - m1.at(0, 1) * b - m1.at(0, 2) * c;

  EXPECT_NEAR(a, 1.0, 0.00001);
  EXPECT_NEAR(b, 2.0, 0.00001);
  EXPECT_NEAR(c, 3.0, 0.00001);
}
TEST(MatrixFunctions, MatrixGauss2)
{
  Matrix<double, 3, 4> m0{ { 1, 3, 1, 9 }, { 1, 1, -1, 1 }, { 3, 11, 5, 35 } };
  Matrix<double, 3, 4> m1(m0.gauss());

  double c = m1.at(2, 3);
  double b = m1.at(1, 3) - m1.at(1, 2) * c;
  double a = m1.at(0, 3) - m1.at(0, 1) * b - m1.at(0, 2) * c;

  EXPECT_NEAR(a, -3.0, 0.00001);
  EXPECT_NEAR(b, 4.0, 0.00001);
  EXPECT_NEAR(c, 0, 0.00001);
}
TEST(MatrixFunctions, MatrixGaussJordan)
{
  Matrix<double, 3, 4> m0{ { 0, 1, 1, 5 }, { 3, 2, 2, 13 }, { 1, -1, 3, 8 } };
  Matrix<double, 3, 4> m1{ { 1, 0, 0, 1 }, { 0, 1, 0, 2 }, { 0, 0, 1, 3 } };
  // EXPECT_EQ(m0.gaussJordan(),m1);
  EXPECT_EQ(true, equals(m0.gaussJordan(), m1,
                         std::numeric_limits<double>::epsilon(), 100));
}
TEST(MatrixFunctions, MatrixSolve)
{
  Matrix<double, 3, 4> m0{ { 0, 1, 1, 5 }, { 3, 2, 2, 13 }, { 1, -1, 3, 8 } };
  Matrix<double, 3, 1> m1{ { { 1 } }, { { 2 } }, { { 3 } } };

  // The following test fails because of rounding errors, use equals
  // instead. EXPECT_EQ(m0.solve(),m1);

  EXPECT_EQ(true, equals(m0.solve(), m1, std::numeric_limits<double>::epsilon(),
                         100));
}
TEST(MatrixFunctions, MatrixInverse)
{
  Matrix<double, 3, 3> m0{ { 1, 2, 0 }, { 1, 0, 1 }, { 2, 2, 2 } };

  EXPECT_EQ(m0.identity(), m0 * m0.inverse());
  EXPECT_EQ(m0.identity(), m0.inverse() * m0);

  Matrix<double, 3, 3> m1{ { 1, 2, 3 }, { 0, 1, 5 }, { 5, 6, 0 } };

  // The following tests fail because of rounding errors, use equals
  // instead. EXPECT_EQ( m1.identity(),m1*m1.inverse());
  // EXPECT_EQ( m1.identity(),m1.inverse()*m1);

  EXPECT_EQ(true, equals(m1.identity(), m1 * m1.inverse(),
                         std::numeric_limits<double>::epsilon(), 100));
  Matrix<double, 3, 3> m2{{0,0,0},{0,0,0},{0,0,0}};

  EXPECT_EQ(true, equals(m2.identity(), m2.inverse(),
                         std::numeric_limits<double>::epsilon(), 100));
}
TEST(MatrixFunctions, MatrixPseudoInverse)
{
  Matrix<double, 3, 5> m0{ { -1, 8, 2, 8, 7 },
                           { 5, 6, -5, 7, 2 },
                           { -9, 0, 1, 2, -3 } };

  auto test = m0 * m0.pseudoInverse();

  EXPECT_EQ(true, equals(test.identity(), test,
                         std::numeric_limits<double>::epsilon(), 100));
}
TEST(MatrixFunctions, MatrixColumnVectorEquality)
{
  Matrix<double, 3, 1> m0{ { { 1 } }, { { 2 } }, { { 3 } } };
  Matrix<double, 3, 1> m1{ { { 1 } }, { { 2 } }, { { 3 } } };

  EXPECT_EQ(true, equals(m0, m1 /*,std::numeric_limits<double>::epsilon()*/));
}
TEST(MatrixFunctions, MatrixRowVectorEquality)
{
  Matrix<double, 1, 3> m0{ { 1, 2, 3 } };
  Matrix<double, 1, 3> m1{ { 1, 2, 3 } };

  EXPECT_EQ(true, equals(m0, m1 /*,std::numeric_limits<double>::epsilon()*/));
}
TEST(MatrixFunctions, MatrixRowVectorInEquality)
{
  Matrix<double, 1, 3> m0{ { 1, 2, 3 } };
  Matrix<double, 1, 3> m1{ { 1, 2, 4 } };
  EXPECT_EQ(false, equals(m0, m1 /*,std::numeric_limits<double>::epsilon()*/));
}
