#ifndef KINEMATICS_JACOBIMATRIX_HPP
#define KINEMATICS_JACOBIMATRIX_HPP
#include "kinematics/Configuration.hpp"
#include "matrix/Matrix.hpp"
#include <cmath>
#include <vector>
namespace kinematics
{
  inline double CalcJ11(double Ct1,
                        double St1,
                        double Ct2,
                        double St2,
                        double Ct3,
                        double St3,
                        double Ct4,
                        double St4,
                        double Ct5,
                        double St5,
                        double Ct6,
                        double St6,
                        double Ct7,
                        double St7)
  {
    // clang-format off
	return 
(-Ct1*(Ct3*St5*(0.257*St6+0.088*Ct6)+St3*((-St4*(0.088*St6-0.257*Ct6+0.384))+Ct4*(Ct5*(0.257*St6+0.088*Ct6)-0.0825)+0.0825)))-St1*(Ct2*(Ct3*((-St4*(0.088*St6-0.257*Ct6+0.384))+Ct4*(Ct5*(0.257*St6+0.088*Ct6)-0.0825)+0.0825)-St3*St5*(0.257*St6+0.088*Ct6))-St2*((-Ct4*(0.088*St6-0.257*Ct6+0.384))-St4*(Ct5*(0.257*St6+0.088*Ct6)-0.0825)-0.316));

    // clang-format on
  }
  inline double CalcJ12(double Ct1,
                        double St1,
                        double Ct2,
                        double St2,
                        double Ct3,
                        double St3,
                        double Ct4,
                        double St4,
                        double Ct5,
                        double St5,
                        double Ct6,
                        double St6,
                        double Ct7,
                        double St7)
  {
    // clang-format off
	return 
Ct1*((-St2*(Ct3*((-St4*(0.088*St6-0.257*Ct6+0.384))+Ct4*(Ct5*(0.257*St6+0.088*Ct6)-0.0825)+0.0825)-St3*St5*(0.257*St6+0.088*Ct6)))-Ct2*((-Ct4*(0.088*St6-0.257*Ct6+0.384))-St4*(Ct5*(0.257*St6+0.088*Ct6)-0.0825)-0.316));

    // clang-format on
  }
  inline double CalcJ13(double Ct1,
                        double St1,
                        double Ct2,
                        double St2,
                        double Ct3,
                        double St3,
                        double Ct4,
                        double St4,
                        double Ct5,
                        double St5,
                        double Ct6,
                        double St6,
                        double Ct7,
                        double St7)
  {
    // clang-format off
	return 
Ct1*Ct2*((-Ct3*St5*(0.257*St6+0.088*Ct6))-St3*((-St4*(0.088*St6-0.257*Ct6+0.384))+Ct4*(Ct5*(0.257*St6+0.088*Ct6)-0.0825)+0.0825))-St1*(Ct3*((-St4*(0.088*St6-0.257*Ct6+0.384))+Ct4*(Ct5*(0.257*St6+0.088*Ct6)-0.0825)+0.0825)-St3*St5*(0.257*St6+0.088*Ct6));

    // clang-format on
  }
  inline double CalcJ14(double Ct1,
                        double St1,
                        double Ct2,
                        double St2,
                        double Ct3,
                        double St3,
                        double Ct4,
                        double St4,
                        double Ct5,
                        double St5,
                        double Ct6,
                        double St6,
                        double Ct7,
                        double St7)
  {
    // clang-format off
	return 
Ct1*(Ct2*Ct3*((-Ct4*(0.088*St6-0.257*Ct6+0.384))-St4*(Ct5*(0.257*St6+0.088*Ct6)-0.0825))-St2*(St4*(0.088*St6-0.257*Ct6+0.384)-Ct4*(Ct5*(0.257*St6+0.088*Ct6)-0.0825)))-St1*St3*((-Ct4*(0.088*St6-0.257*Ct6+0.384))-St4*(Ct5*(0.257*St6+0.088*Ct6)-0.0825));

    // clang-format on
  }
  inline double CalcJ15(double Ct1,
                        double St1,
                        double Ct2,
                        double St2,
                        double Ct3,
                        double St3,
                        double Ct4,
                        double St4,
                        double Ct5,
                        double St5,
                        double Ct6,
                        double St6,
                        double Ct7,
                        double St7)
  {
    // clang-format off
	return 
Ct1*(Ct2*((-Ct3*Ct4*St5*(0.257*St6+0.088*Ct6))-Ct5*St3*(0.257*St6+0.088*Ct6))-St2*St4*St5*(0.257*St6+0.088*Ct6))-St1*(Ct3*Ct5*(0.257*St6+0.088*Ct6)-Ct4*St3*St5*(0.257*St6+0.088*Ct6));

    // clang-format on
  }
  inline double CalcJ16(double Ct1,
                        double St1,
                        double Ct2,
                        double St2,
                        double Ct3,
                        double St3,
                        double Ct4,
                        double St4,
                        double Ct5,
                        double St5,
                        double Ct6,
                        double St6,
                        double Ct7,
                        double St7)
  {
    // clang-format off
	return 
Ct1*(Ct2*(Ct3*(Ct4*Ct5*(0.257*Ct6-0.088*St6)-St4*(0.257*St6+0.088*Ct6))-St3*St5*(0.257*Ct6-0.088*St6))-St2*((-Ct4*(0.257*St6+0.088*Ct6))-Ct5*St4*(0.257*Ct6-0.088*St6)))-St1*(Ct3*St5*(0.257*Ct6-0.088*St6)+St3*(Ct4*Ct5*(0.257*Ct6-0.088*St6)-St4*(0.257*St6+0.088*Ct6)));

    // clang-format on
  }
  inline double CalcJ17(double Ct1,
                        double St1,
                        double Ct2,
                        double St2,
                        double Ct3,
                        double St3,
                        double Ct4,
                        double St4,
                        double Ct5,
                        double St5,
                        double Ct6,
                        double St6,
                        double Ct7,
                        double St7)
  {
    // clang-format off
	return 
0;

    // clang-format on
  }
  inline double CalcJ21(double Ct1,
                        double St1,
                        double Ct2,
                        double St2,
                        double Ct3,
                        double St3,
                        double Ct4,
                        double St4,
                        double Ct5,
                        double St5,
                        double Ct6,
                        double St6,
                        double Ct7,
                        double St7)
  {
    // clang-format off
	return 
Ct1*(Ct2*(Ct3*((-St4*(0.088*St6-0.257*Ct6+0.384))+Ct4*(Ct5*(0.257*St6+0.088*Ct6)-0.0825)+0.0825)-St3*St5*(0.257*St6+0.088*Ct6))-St2*((-Ct4*(0.088*St6-0.257*Ct6+0.384))-St4*(Ct5*(0.257*St6+0.088*Ct6)-0.0825)-0.316))-St1*(Ct3*St5*(0.257*St6+0.088*Ct6)+St3*((-St4*(0.088*St6-0.257*Ct6+0.384))+Ct4*(Ct5*(0.257*St6+0.088*Ct6)-0.0825)+0.0825));

    // clang-format on
  }
  inline double CalcJ22(double Ct1,
                        double St1,
                        double Ct2,
                        double St2,
                        double Ct3,
                        double St3,
                        double Ct4,
                        double St4,
                        double Ct5,
                        double St5,
                        double Ct6,
                        double St6,
                        double Ct7,
                        double St7)
  {
    // clang-format off
	return 
St1*((-St2*(Ct3*((-St4*(0.088*St6-0.257*Ct6+0.384))+Ct4*(Ct5*(0.257*St6+0.088*Ct6)-0.0825)+0.0825)-St3*St5*(0.257*St6+0.088*Ct6)))-Ct2*((-Ct4*(0.088*St6-0.257*Ct6+0.384))-St4*(Ct5*(0.257*St6+0.088*Ct6)-0.0825)-0.316));

    // clang-format on
  }
  inline double CalcJ23(double Ct1,
                        double St1,
                        double Ct2,
                        double St2,
                        double Ct3,
                        double St3,
                        double Ct4,
                        double St4,
                        double Ct5,
                        double St5,
                        double Ct6,
                        double St6,
                        double Ct7,
                        double St7)
  {
    // clang-format off
	return 
Ct1*(Ct3*((-St4*(0.088*St6-0.257*Ct6+0.384))+Ct4*(Ct5*(0.257*St6+0.088*Ct6)-0.0825)+0.0825)-St3*St5*(0.257*St6+0.088*Ct6))+Ct2*St1*((-Ct3*St5*(0.257*St6+0.088*Ct6))-St3*((-St4*(0.088*St6-0.257*Ct6+0.384))+Ct4*(Ct5*(0.257*St6+0.088*Ct6)-0.0825)+0.0825));

    // clang-format on
  }
  inline double CalcJ24(double Ct1,
                        double St1,
                        double Ct2,
                        double St2,
                        double Ct3,
                        double St3,
                        double Ct4,
                        double St4,
                        double Ct5,
                        double St5,
                        double Ct6,
                        double St6,
                        double Ct7,
                        double St7)
  {
    // clang-format off
	return 
Ct1*St3*((-Ct4*(0.088*St6-0.257*Ct6+0.384))-St4*(Ct5*(0.257*St6+0.088*Ct6)-0.0825))+St1*(Ct2*Ct3*((-Ct4*(0.088*St6-0.257*Ct6+0.384))-St4*(Ct5*(0.257*St6+0.088*Ct6)-0.0825))-St2*(St4*(0.088*St6-0.257*Ct6+0.384)-Ct4*(Ct5*(0.257*St6+0.088*Ct6)-0.0825)));

    // clang-format on
  }
  inline double CalcJ25(double Ct1,
                        double St1,
                        double Ct2,
                        double St2,
                        double Ct3,
                        double St3,
                        double Ct4,
                        double St4,
                        double Ct5,
                        double St5,
                        double Ct6,
                        double St6,
                        double Ct7,
                        double St7)
  {
    // clang-format off
	return 
St1*(Ct2*((-Ct3*Ct4*St5*(0.257*St6+0.088*Ct6))-Ct5*St3*(0.257*St6+0.088*Ct6))-St2*St4*St5*(0.257*St6+0.088*Ct6))+Ct1*(Ct3*Ct5*(0.257*St6+0.088*Ct6)-Ct4*St3*St5*(0.257*St6+0.088*Ct6));

    // clang-format on
  }
  inline double CalcJ26(double Ct1,
                        double St1,
                        double Ct2,
                        double St2,
                        double Ct3,
                        double St3,
                        double Ct4,
                        double St4,
                        double Ct5,
                        double St5,
                        double Ct6,
                        double St6,
                        double Ct7,
                        double St7)
  {
    // clang-format off
	return 
Ct1*(Ct3*St5*(0.257*Ct6-0.088*St6)+St3*(Ct4*Ct5*(0.257*Ct6-0.088*St6)-St4*(0.257*St6+0.088*Ct6)))+St1*(Ct2*(Ct3*(Ct4*Ct5*(0.257*Ct6-0.088*St6)-St4*(0.257*St6+0.088*Ct6))-St3*St5*(0.257*Ct6-0.088*St6))-St2*((-Ct4*(0.257*St6+0.088*Ct6))-Ct5*St4*(0.257*Ct6-0.088*St6)));

    // clang-format on
  }
  inline double CalcJ27(double Ct1,
                        double St1,
                        double Ct2,
                        double St2,
                        double Ct3,
                        double St3,
                        double Ct4,
                        double St4,
                        double Ct5,
                        double St5,
                        double Ct6,
                        double St6,
                        double Ct7,
                        double St7)
  {
    // clang-format off
	return 
0;

    // clang-format on
  }
  inline double CalcJ31(double Ct1,
                        double St1,
                        double Ct2,
                        double St2,
                        double Ct3,
                        double St3,
                        double Ct4,
                        double St4,
                        double Ct5,
                        double St5,
                        double Ct6,
                        double St6,
                        double Ct7,
                        double St7)
  {
    // clang-format off
	return 
0;

    // clang-format on
  }
  inline double CalcJ32(double Ct1,
                        double St1,
                        double Ct2,
                        double St2,
                        double Ct3,
                        double St3,
                        double Ct4,
                        double St4,
                        double Ct5,
                        double St5,
                        double Ct6,
                        double St6,
                        double Ct7,
                        double St7)
  {
    // clang-format off
	return 
St2*((-Ct4*(0.088*St6-0.257*Ct6+0.384))-St4*(Ct5*(0.257*St6+0.088*Ct6)-0.0825)-0.316)-Ct2*(Ct3*((-St4*(0.088*St6-0.257*Ct6+0.384))+Ct4*(Ct5*(0.257*St6+0.088*Ct6)-0.0825)+0.0825)-St3*St5*(0.257*St6+0.088*Ct6));

    // clang-format on
  }
  inline double CalcJ33(double Ct1,
                        double St1,
                        double Ct2,
                        double St2,
                        double Ct3,
                        double St3,
                        double Ct4,
                        double St4,
                        double Ct5,
                        double St5,
                        double Ct6,
                        double St6,
                        double Ct7,
                        double St7)
  {
    // clang-format off
	return 
-St2*((-Ct3*St5*(0.257*St6+0.088*Ct6))-St3*((-St4*(0.088*St6-0.257*Ct6+0.384))+Ct4*(Ct5*(0.257*St6+0.088*Ct6)-0.0825)+0.0825));

    // clang-format on
  }
  inline double CalcJ34(double Ct1,
                        double St1,
                        double Ct2,
                        double St2,
                        double Ct3,
                        double St3,
                        double Ct4,
                        double St4,
                        double Ct5,
                        double St5,
                        double Ct6,
                        double St6,
                        double Ct7,
                        double St7)
  {
    // clang-format off
	return 
(-Ct2*(St4*(0.088*St6-0.257*Ct6+0.384)-Ct4*(Ct5*(0.257*St6+0.088*Ct6)-0.0825)))-Ct3*St2*((-Ct4*(0.088*St6-0.257*Ct6+0.384))-St4*(Ct5*(0.257*St6+0.088*Ct6)-0.0825));

    // clang-format on
  }
  inline double CalcJ35(double Ct1,
                        double St1,
                        double Ct2,
                        double St2,
                        double Ct3,
                        double St3,
                        double Ct4,
                        double St4,
                        double Ct5,
                        double St5,
                        double Ct6,
                        double St6,
                        double Ct7,
                        double St7)
  {
    // clang-format off
	return 
(-Ct2*St4*St5*(0.257*St6+0.088*Ct6))-St2*((-Ct3*Ct4*St5*(0.257*St6+0.088*Ct6))-Ct5*St3*(0.257*St6+0.088*Ct6));

    // clang-format on
  }
  inline double CalcJ36(double Ct1,
                        double St1,
                        double Ct2,
                        double St2,
                        double Ct3,
                        double St3,
                        double Ct4,
                        double St4,
                        double Ct5,
                        double St5,
                        double Ct6,
                        double St6,
                        double Ct7,
                        double St7)
  {
    // clang-format off
	return 
(-Ct2*((-Ct4*(0.257*St6+0.088*Ct6))-Ct5*St4*(0.257*Ct6-0.088*St6)))-St2*(Ct3*(Ct4*Ct5*(0.257*Ct6-0.088*St6)-St4*(0.257*St6+0.088*Ct6))-St3*St5*(0.257*Ct6-0.088*St6));

    // clang-format on
  }
  inline double CalcJ37(double Ct1,
                        double St1,
                        double Ct2,
                        double St2,
                        double Ct3,
                        double St3,
                        double Ct4,
                        double St4,
                        double Ct5,
                        double St5,
                        double Ct6,
                        double St6,
                        double Ct7,
                        double St7)
  {
    // clang-format off
	return 
0;

    // clang-format on
  }
  inline double CalcJ41(double Ct1,
                        double St1,
                        double Ct2,
                        double St2,
                        double Ct3,
                        double St3,
                        double Ct4,
                        double St4,
                        double Ct5,
                        double St5,
                        double Ct6,
                        double St6,
                        double Ct7,
                        double St7)
  {
    // clang-format off
	return 
std::pow((Ct1*(Ct2*(Ct3*(Ct4*(St5*St7+Ct5*Ct6*Ct7)-Ct7*St4*St6)-St3*(Ct6*Ct7*St5-Ct5*St7))-St2*((-St4*(St5*St7+Ct5*Ct6*Ct7))-Ct4*Ct7*St6))-St1*(St3*(Ct4*(St5*St7+Ct5*Ct6*Ct7)-Ct7*St4*St6)+Ct3*(Ct6*Ct7*St5-Ct5*St7))),2)/(std::pow((St1*(Ct2*(Ct3*(Ct4*(St5*St7+Ct5*Ct6*Ct7)-Ct7*St4*St6)-St3*(Ct6*Ct7*St5-Ct5*St7))-St2*((-St4*(St5*St7+Ct5*Ct6*Ct7))-Ct4*Ct7*St6))+Ct1*(St3*(Ct4*(St5*St7+Ct5*Ct6*Ct7)-Ct7*St4*St6)+Ct3*(Ct6*Ct7*St5-Ct5*St7))),2)+std::pow((Ct1*(Ct2*(Ct3*(Ct4*(St5*St7+Ct5*Ct6*Ct7)-Ct7*St4*St6)-St3*(Ct6*Ct7*St5-Ct5*St7))-St2*((-St4*(St5*St7+Ct5*Ct6*Ct7))-Ct4*Ct7*St6))-St1*(St3*(Ct4*(St5*St7+Ct5*Ct6*Ct7)-Ct7*St4*St6)+Ct3*(Ct6*Ct7*St5-Ct5*St7))),2))-(((-St1*(Ct2*(Ct3*(Ct4*(St5*St7+Ct5*Ct6*Ct7)-Ct7*St4*St6)-St3*(Ct6*Ct7*St5-Ct5*St7))-St2*((-St4*(St5*St7+Ct5*Ct6*Ct7))-Ct4*Ct7*St6)))-Ct1*(St3*(Ct4*(St5*St7+Ct5*Ct6*Ct7)-Ct7*St4*St6)+Ct3*(Ct6*Ct7*St5-Ct5*St7)))*(St1*(Ct2*(Ct3*(Ct4*(St5*St7+Ct5*Ct6*Ct7)-Ct7*St4*St6)-St3*(Ct6*Ct7*St5-Ct5*St7))-St2*((-St4*(St5*St7+Ct5*Ct6*Ct7))-Ct4*Ct7*St6))+Ct1*(St3*(Ct4*(St5*St7+Ct5*Ct6*Ct7)-Ct7*St4*St6)+Ct3*(Ct6*Ct7*St5-Ct5*St7))))/(std::pow((St1*(Ct2*(Ct3*(Ct4*(St5*St7+Ct5*Ct6*Ct7)-Ct7*St4*St6)-St3*(Ct6*Ct7*St5-Ct5*St7))-St2*((-St4*(St5*St7+Ct5*Ct6*Ct7))-Ct4*Ct7*St6))+Ct1*(St3*(Ct4*(St5*St7+Ct5*Ct6*Ct7)-Ct7*St4*St6)+Ct3*(Ct6*Ct7*St5-Ct5*St7))),2)+std::pow((Ct1*(Ct2*(Ct3*(Ct4*(St5*St7+Ct5*Ct6*Ct7)-Ct7*St4*St6)-St3*(Ct6*Ct7*St5-Ct5*St7))-St2*((-St4*(St5*St7+Ct5*Ct6*Ct7))-Ct4*Ct7*St6))-St1*(St3*(Ct4*(St5*St7+Ct5*Ct6*Ct7)-Ct7*St4*St6)+Ct3*(Ct6*Ct7*St5-Ct5*St7))),2));

    // clang-format on
  }
  inline double CalcJ42(double Ct1,
                        double St1,
                        double Ct2,
                        double St2,
                        double Ct3,
                        double St3,
                        double Ct4,
                        double St4,
                        double Ct5,
                        double St5,
                        double Ct6,
                        double St6,
                        double Ct7,
                        double St7)
  {
    // clang-format off
	return 
(St1*((-St2*(Ct3*(Ct4*(St5*St7+Ct5*Ct6*Ct7)-Ct7*St4*St6)-St3*(Ct6*Ct7*St5-Ct5*St7)))-Ct2*((-St4*(St5*St7+Ct5*Ct6*Ct7))-Ct4*Ct7*St6))*(Ct1*(Ct2*(Ct3*(Ct4*(St5*St7+Ct5*Ct6*Ct7)-Ct7*St4*St6)-St3*(Ct6*Ct7*St5-Ct5*St7))-St2*((-St4*(St5*St7+Ct5*Ct6*Ct7))-Ct4*Ct7*St6))-St1*(St3*(Ct4*(St5*St7+Ct5*Ct6*Ct7)-Ct7*St4*St6)+Ct3*(Ct6*Ct7*St5-Ct5*St7))))/(std::pow((St1*(Ct2*(Ct3*(Ct4*(St5*St7+Ct5*Ct6*Ct7)-Ct7*St4*St6)-St3*(Ct6*Ct7*St5-Ct5*St7))-St2*((-St4*(St5*St7+Ct5*Ct6*Ct7))-Ct4*Ct7*St6))+Ct1*(St3*(Ct4*(St5*St7+Ct5*Ct6*Ct7)-Ct7*St4*St6)+Ct3*(Ct6*Ct7*St5-Ct5*St7))),2)+std::pow((Ct1*(Ct2*(Ct3*(Ct4*(St5*St7+Ct5*Ct6*Ct7)-Ct7*St4*St6)-St3*(Ct6*Ct7*St5-Ct5*St7))-St2*((-St4*(St5*St7+Ct5*Ct6*Ct7))-Ct4*Ct7*St6))-St1*(St3*(Ct4*(St5*St7+Ct5*Ct6*Ct7)-Ct7*St4*St6)+Ct3*(Ct6*Ct7*St5-Ct5*St7))),2))-(Ct1*((-St2*(Ct3*(Ct4*(St5*St7+Ct5*Ct6*Ct7)-Ct7*St4*St6)-St3*(Ct6*Ct7*St5-Ct5*St7)))-Ct2*((-St4*(St5*St7+Ct5*Ct6*Ct7))-Ct4*Ct7*St6))*(St1*(Ct2*(Ct3*(Ct4*(St5*St7+Ct5*Ct6*Ct7)-Ct7*St4*St6)-St3*(Ct6*Ct7*St5-Ct5*St7))-St2*((-St4*(St5*St7+Ct5*Ct6*Ct7))-Ct4*Ct7*St6))+Ct1*(St3*(Ct4*(St5*St7+Ct5*Ct6*Ct7)-Ct7*St4*St6)+Ct3*(Ct6*Ct7*St5-Ct5*St7))))/(std::pow((St1*(Ct2*(Ct3*(Ct4*(St5*St7+Ct5*Ct6*Ct7)-Ct7*St4*St6)-St3*(Ct6*Ct7*St5-Ct5*St7))-St2*((-St4*(St5*St7+Ct5*Ct6*Ct7))-Ct4*Ct7*St6))+Ct1*(St3*(Ct4*(St5*St7+Ct5*Ct6*Ct7)-Ct7*St4*St6)+Ct3*(Ct6*Ct7*St5-Ct5*St7))),2)+std::pow((Ct1*(Ct2*(Ct3*(Ct4*(St5*St7+Ct5*Ct6*Ct7)-Ct7*St4*St6)-St3*(Ct6*Ct7*St5-Ct5*St7))-St2*((-St4*(St5*St7+Ct5*Ct6*Ct7))-Ct4*Ct7*St6))-St1*(St3*(Ct4*(St5*St7+Ct5*Ct6*Ct7)-Ct7*St4*St6)+Ct3*(Ct6*Ct7*St5-Ct5*St7))),2));

    // clang-format on
  }
  inline double CalcJ43(double Ct1,
                        double St1,
                        double Ct2,
                        double St2,
                        double Ct3,
                        double St3,
                        double Ct4,
                        double St4,
                        double Ct5,
                        double St5,
                        double Ct6,
                        double St6,
                        double Ct7,
                        double St7)
  {
    // clang-format off
	return 
((Ct2*St1*((-St3*(Ct4*(St5*St7+Ct5*Ct6*Ct7)-Ct7*St4*St6))-Ct3*(Ct6*Ct7*St5-Ct5*St7))+Ct1*(Ct3*(Ct4*(St5*St7+Ct5*Ct6*Ct7)-Ct7*St4*St6)-St3*(Ct6*Ct7*St5-Ct5*St7)))*(Ct1*(Ct2*(Ct3*(Ct4*(St5*St7+Ct5*Ct6*Ct7)-Ct7*St4*St6)-St3*(Ct6*Ct7*St5-Ct5*St7))-St2*((-St4*(St5*St7+Ct5*Ct6*Ct7))-Ct4*Ct7*St6))-St1*(St3*(Ct4*(St5*St7+Ct5*Ct6*Ct7)-Ct7*St4*St6)+Ct3*(Ct6*Ct7*St5-Ct5*St7))))/(std::pow((St1*(Ct2*(Ct3*(Ct4*(St5*St7+Ct5*Ct6*Ct7)-Ct7*St4*St6)-St3*(Ct6*Ct7*St5-Ct5*St7))-St2*((-St4*(St5*St7+Ct5*Ct6*Ct7))-Ct4*Ct7*St6))+Ct1*(St3*(Ct4*(St5*St7+Ct5*Ct6*Ct7)-Ct7*St4*St6)+Ct3*(Ct6*Ct7*St5-Ct5*St7))),2)+std::pow((Ct1*(Ct2*(Ct3*(Ct4*(St5*St7+Ct5*Ct6*Ct7)-Ct7*St4*St6)-St3*(Ct6*Ct7*St5-Ct5*St7))-St2*((-St4*(St5*St7+Ct5*Ct6*Ct7))-Ct4*Ct7*St6))-St1*(St3*(Ct4*(St5*St7+Ct5*Ct6*Ct7)-Ct7*St4*St6)+Ct3*(Ct6*Ct7*St5-Ct5*St7))),2))-((Ct1*Ct2*((-St3*(Ct4*(St5*St7+Ct5*Ct6*Ct7)-Ct7*St4*St6))-Ct3*(Ct6*Ct7*St5-Ct5*St7))-St1*(Ct3*(Ct4*(St5*St7+Ct5*Ct6*Ct7)-Ct7*St4*St6)-St3*(Ct6*Ct7*St5-Ct5*St7)))*(St1*(Ct2*(Ct3*(Ct4*(St5*St7+Ct5*Ct6*Ct7)-Ct7*St4*St6)-St3*(Ct6*Ct7*St5-Ct5*St7))-St2*((-St4*(St5*St7+Ct5*Ct6*Ct7))-Ct4*Ct7*St6))+Ct1*(St3*(Ct4*(St5*St7+Ct5*Ct6*Ct7)-Ct7*St4*St6)+Ct3*(Ct6*Ct7*St5-Ct5*St7))))/(std::pow((St1*(Ct2*(Ct3*(Ct4*(St5*St7+Ct5*Ct6*Ct7)-Ct7*St4*St6)-St3*(Ct6*Ct7*St5-Ct5*St7))-St2*((-St4*(St5*St7+Ct5*Ct6*Ct7))-Ct4*Ct7*St6))+Ct1*(St3*(Ct4*(St5*St7+Ct5*Ct6*Ct7)-Ct7*St4*St6)+Ct3*(Ct6*Ct7*St5-Ct5*St7))),2)+std::pow((Ct1*(Ct2*(Ct3*(Ct4*(St5*St7+Ct5*Ct6*Ct7)-Ct7*St4*St6)-St3*(Ct6*Ct7*St5-Ct5*St7))-St2*((-St4*(St5*St7+Ct5*Ct6*Ct7))-Ct4*Ct7*St6))-St1*(St3*(Ct4*(St5*St7+Ct5*Ct6*Ct7)-Ct7*St4*St6)+Ct3*(Ct6*Ct7*St5-Ct5*St7))),2));

    // clang-format on
  }
  inline double CalcJ44(double Ct1,
                        double St1,
                        double Ct2,
                        double St2,
                        double Ct3,
                        double St3,
                        double Ct4,
                        double St4,
                        double Ct5,
                        double St5,
                        double Ct6,
                        double St6,
                        double Ct7,
                        double St7)
  {
    // clang-format off
	return 
((St1*(Ct2*Ct3*((-St4*(St5*St7+Ct5*Ct6*Ct7))-Ct4*Ct7*St6)-St2*(Ct7*St4*St6-Ct4*(St5*St7+Ct5*Ct6*Ct7)))+Ct1*St3*((-St4*(St5*St7+Ct5*Ct6*Ct7))-Ct4*Ct7*St6))*(Ct1*(Ct2*(Ct3*(Ct4*(St5*St7+Ct5*Ct6*Ct7)-Ct7*St4*St6)-St3*(Ct6*Ct7*St5-Ct5*St7))-St2*((-St4*(St5*St7+Ct5*Ct6*Ct7))-Ct4*Ct7*St6))-St1*(St3*(Ct4*(St5*St7+Ct5*Ct6*Ct7)-Ct7*St4*St6)+Ct3*(Ct6*Ct7*St5-Ct5*St7))))/(std::pow((St1*(Ct2*(Ct3*(Ct4*(St5*St7+Ct5*Ct6*Ct7)-Ct7*St4*St6)-St3*(Ct6*Ct7*St5-Ct5*St7))-St2*((-St4*(St5*St7+Ct5*Ct6*Ct7))-Ct4*Ct7*St6))+Ct1*(St3*(Ct4*(St5*St7+Ct5*Ct6*Ct7)-Ct7*St4*St6)+Ct3*(Ct6*Ct7*St5-Ct5*St7))),2)+std::pow((Ct1*(Ct2*(Ct3*(Ct4*(St5*St7+Ct5*Ct6*Ct7)-Ct7*St4*St6)-St3*(Ct6*Ct7*St5-Ct5*St7))-St2*((-St4*(St5*St7+Ct5*Ct6*Ct7))-Ct4*Ct7*St6))-St1*(St3*(Ct4*(St5*St7+Ct5*Ct6*Ct7)-Ct7*St4*St6)+Ct3*(Ct6*Ct7*St5-Ct5*St7))),2))-((Ct1*(Ct2*Ct3*((-St4*(St5*St7+Ct5*Ct6*Ct7))-Ct4*Ct7*St6)-St2*(Ct7*St4*St6-Ct4*(St5*St7+Ct5*Ct6*Ct7)))-St1*St3*((-St4*(St5*St7+Ct5*Ct6*Ct7))-Ct4*Ct7*St6))*(St1*(Ct2*(Ct3*(Ct4*(St5*St7+Ct5*Ct6*Ct7)-Ct7*St4*St6)-St3*(Ct6*Ct7*St5-Ct5*St7))-St2*((-St4*(St5*St7+Ct5*Ct6*Ct7))-Ct4*Ct7*St6))+Ct1*(St3*(Ct4*(St5*St7+Ct5*Ct6*Ct7)-Ct7*St4*St6)+Ct3*(Ct6*Ct7*St5-Ct5*St7))))/(std::pow((St1*(Ct2*(Ct3*(Ct4*(St5*St7+Ct5*Ct6*Ct7)-Ct7*St4*St6)-St3*(Ct6*Ct7*St5-Ct5*St7))-St2*((-St4*(St5*St7+Ct5*Ct6*Ct7))-Ct4*Ct7*St6))+Ct1*(St3*(Ct4*(St5*St7+Ct5*Ct6*Ct7)-Ct7*St4*St6)+Ct3*(Ct6*Ct7*St5-Ct5*St7))),2)+std::pow((Ct1*(Ct2*(Ct3*(Ct4*(St5*St7+Ct5*Ct6*Ct7)-Ct7*St4*St6)-St3*(Ct6*Ct7*St5-Ct5*St7))-St2*((-St4*(St5*St7+Ct5*Ct6*Ct7))-Ct4*Ct7*St6))-St1*(St3*(Ct4*(St5*St7+Ct5*Ct6*Ct7)-Ct7*St4*St6)+Ct3*(Ct6*Ct7*St5-Ct5*St7))),2));

    // clang-format on
  }
  inline double CalcJ45(double Ct1,
                        double St1,
                        double Ct2,
                        double St2,
                        double Ct3,
                        double St3,
                        double Ct4,
                        double St4,
                        double Ct5,
                        double St5,
                        double Ct6,
                        double St6,
                        double Ct7,
                        double St7)
  {
    // clang-format off
	return 
((St1*(Ct2*(Ct3*Ct4*(Ct5*St7-Ct6*Ct7*St5)-St3*(St5*St7+Ct5*Ct6*Ct7))+St2*St4*(Ct5*St7-Ct6*Ct7*St5))+Ct1*(Ct3*(St5*St7+Ct5*Ct6*Ct7)+Ct4*St3*(Ct5*St7-Ct6*Ct7*St5)))*(Ct1*(Ct2*(Ct3*(Ct4*(St5*St7+Ct5*Ct6*Ct7)-Ct7*St4*St6)-St3*(Ct6*Ct7*St5-Ct5*St7))-St2*((-St4*(St5*St7+Ct5*Ct6*Ct7))-Ct4*Ct7*St6))-St1*(St3*(Ct4*(St5*St7+Ct5*Ct6*Ct7)-Ct7*St4*St6)+Ct3*(Ct6*Ct7*St5-Ct5*St7))))/(std::pow((St1*(Ct2*(Ct3*(Ct4*(St5*St7+Ct5*Ct6*Ct7)-Ct7*St4*St6)-St3*(Ct6*Ct7*St5-Ct5*St7))-St2*((-St4*(St5*St7+Ct5*Ct6*Ct7))-Ct4*Ct7*St6))+Ct1*(St3*(Ct4*(St5*St7+Ct5*Ct6*Ct7)-Ct7*St4*St6)+Ct3*(Ct6*Ct7*St5-Ct5*St7))),2)+std::pow((Ct1*(Ct2*(Ct3*(Ct4*(St5*St7+Ct5*Ct6*Ct7)-Ct7*St4*St6)-St3*(Ct6*Ct7*St5-Ct5*St7))-St2*((-St4*(St5*St7+Ct5*Ct6*Ct7))-Ct4*Ct7*St6))-St1*(St3*(Ct4*(St5*St7+Ct5*Ct6*Ct7)-Ct7*St4*St6)+Ct3*(Ct6*Ct7*St5-Ct5*St7))),2))-((Ct1*(Ct2*(Ct3*Ct4*(Ct5*St7-Ct6*Ct7*St5)-St3*(St5*St7+Ct5*Ct6*Ct7))+St2*St4*(Ct5*St7-Ct6*Ct7*St5))-St1*(Ct3*(St5*St7+Ct5*Ct6*Ct7)+Ct4*St3*(Ct5*St7-Ct6*Ct7*St5)))*(St1*(Ct2*(Ct3*(Ct4*(St5*St7+Ct5*Ct6*Ct7)-Ct7*St4*St6)-St3*(Ct6*Ct7*St5-Ct5*St7))-St2*((-St4*(St5*St7+Ct5*Ct6*Ct7))-Ct4*Ct7*St6))+Ct1*(St3*(Ct4*(St5*St7+Ct5*Ct6*Ct7)-Ct7*St4*St6)+Ct3*(Ct6*Ct7*St5-Ct5*St7))))/(std::pow((St1*(Ct2*(Ct3*(Ct4*(St5*St7+Ct5*Ct6*Ct7)-Ct7*St4*St6)-St3*(Ct6*Ct7*St5-Ct5*St7))-St2*((-St4*(St5*St7+Ct5*Ct6*Ct7))-Ct4*Ct7*St6))+Ct1*(St3*(Ct4*(St5*St7+Ct5*Ct6*Ct7)-Ct7*St4*St6)+Ct3*(Ct6*Ct7*St5-Ct5*St7))),2)+std::pow((Ct1*(Ct2*(Ct3*(Ct4*(St5*St7+Ct5*Ct6*Ct7)-Ct7*St4*St6)-St3*(Ct6*Ct7*St5-Ct5*St7))-St2*((-St4*(St5*St7+Ct5*Ct6*Ct7))-Ct4*Ct7*St6))-St1*(St3*(Ct4*(St5*St7+Ct5*Ct6*Ct7)-Ct7*St4*St6)+Ct3*(Ct6*Ct7*St5-Ct5*St7))),2));

    // clang-format on
  }
  inline double CalcJ46(double Ct1,
                        double St1,
                        double Ct2,
                        double St2,
                        double Ct3,
                        double St3,
                        double Ct4,
                        double St4,
                        double Ct5,
                        double St5,
                        double Ct6,
                        double St6,
                        double Ct7,
                        double St7)
  {
    // clang-format off
	return 
((St1*(Ct2*(Ct3*((-Ct4*Ct5*Ct7*St6)-Ct6*Ct7*St4)+Ct7*St3*St5*St6)-St2*(Ct5*Ct7*St4*St6-Ct4*Ct6*Ct7))+Ct1*(St3*((-Ct4*Ct5*Ct7*St6)-Ct6*Ct7*St4)-Ct3*Ct7*St5*St6))*(Ct1*(Ct2*(Ct3*(Ct4*(St5*St7+Ct5*Ct6*Ct7)-Ct7*St4*St6)-St3*(Ct6*Ct7*St5-Ct5*St7))-St2*((-St4*(St5*St7+Ct5*Ct6*Ct7))-Ct4*Ct7*St6))-St1*(St3*(Ct4*(St5*St7+Ct5*Ct6*Ct7)-Ct7*St4*St6)+Ct3*(Ct6*Ct7*St5-Ct5*St7))))/(std::pow((St1*(Ct2*(Ct3*(Ct4*(St5*St7+Ct5*Ct6*Ct7)-Ct7*St4*St6)-St3*(Ct6*Ct7*St5-Ct5*St7))-St2*((-St4*(St5*St7+Ct5*Ct6*Ct7))-Ct4*Ct7*St6))+Ct1*(St3*(Ct4*(St5*St7+Ct5*Ct6*Ct7)-Ct7*St4*St6)+Ct3*(Ct6*Ct7*St5-Ct5*St7))),2)+std::pow((Ct1*(Ct2*(Ct3*(Ct4*(St5*St7+Ct5*Ct6*Ct7)-Ct7*St4*St6)-St3*(Ct6*Ct7*St5-Ct5*St7))-St2*((-St4*(St5*St7+Ct5*Ct6*Ct7))-Ct4*Ct7*St6))-St1*(St3*(Ct4*(St5*St7+Ct5*Ct6*Ct7)-Ct7*St4*St6)+Ct3*(Ct6*Ct7*St5-Ct5*St7))),2))-((Ct1*(Ct2*(Ct3*((-Ct4*Ct5*Ct7*St6)-Ct6*Ct7*St4)+Ct7*St3*St5*St6)-St2*(Ct5*Ct7*St4*St6-Ct4*Ct6*Ct7))-St1*(St3*((-Ct4*Ct5*Ct7*St6)-Ct6*Ct7*St4)-Ct3*Ct7*St5*St6))*(St1*(Ct2*(Ct3*(Ct4*(St5*St7+Ct5*Ct6*Ct7)-Ct7*St4*St6)-St3*(Ct6*Ct7*St5-Ct5*St7))-St2*((-St4*(St5*St7+Ct5*Ct6*Ct7))-Ct4*Ct7*St6))+Ct1*(St3*(Ct4*(St5*St7+Ct5*Ct6*Ct7)-Ct7*St4*St6)+Ct3*(Ct6*Ct7*St5-Ct5*St7))))/(std::pow((St1*(Ct2*(Ct3*(Ct4*(St5*St7+Ct5*Ct6*Ct7)-Ct7*St4*St6)-St3*(Ct6*Ct7*St5-Ct5*St7))-St2*((-St4*(St5*St7+Ct5*Ct6*Ct7))-Ct4*Ct7*St6))+Ct1*(St3*(Ct4*(St5*St7+Ct5*Ct6*Ct7)-Ct7*St4*St6)+Ct3*(Ct6*Ct7*St5-Ct5*St7))),2)+std::pow((Ct1*(Ct2*(Ct3*(Ct4*(St5*St7+Ct5*Ct6*Ct7)-Ct7*St4*St6)-St3*(Ct6*Ct7*St5-Ct5*St7))-St2*((-St4*(St5*St7+Ct5*Ct6*Ct7))-Ct4*Ct7*St6))-St1*(St3*(Ct4*(St5*St7+Ct5*Ct6*Ct7)-Ct7*St4*St6)+Ct3*(Ct6*Ct7*St5-Ct5*St7))),2));

    // clang-format on
  }
  inline double CalcJ47(double Ct1,
                        double St1,
                        double Ct2,
                        double St2,
                        double Ct3,
                        double St3,
                        double Ct4,
                        double St4,
                        double Ct5,
                        double St5,
                        double Ct6,
                        double St6,
                        double Ct7,
                        double St7)
  {
    // clang-format off
	return 
((St1*(Ct2*(Ct3*(Ct4*(Ct7*St5-Ct5*Ct6*St7)+St4*St6*St7)-St3*((-Ct6*St5*St7)-Ct5*Ct7))-St2*(Ct4*St6*St7-St4*(Ct7*St5-Ct5*Ct6*St7)))+Ct1*(St3*(Ct4*(Ct7*St5-Ct5*Ct6*St7)+St4*St6*St7)+Ct3*((-Ct6*St5*St7)-Ct5*Ct7)))*(Ct1*(Ct2*(Ct3*(Ct4*(St5*St7+Ct5*Ct6*Ct7)-Ct7*St4*St6)-St3*(Ct6*Ct7*St5-Ct5*St7))-St2*((-St4*(St5*St7+Ct5*Ct6*Ct7))-Ct4*Ct7*St6))-St1*(St3*(Ct4*(St5*St7+Ct5*Ct6*Ct7)-Ct7*St4*St6)+Ct3*(Ct6*Ct7*St5-Ct5*St7))))/(std::pow((St1*(Ct2*(Ct3*(Ct4*(St5*St7+Ct5*Ct6*Ct7)-Ct7*St4*St6)-St3*(Ct6*Ct7*St5-Ct5*St7))-St2*((-St4*(St5*St7+Ct5*Ct6*Ct7))-Ct4*Ct7*St6))+Ct1*(St3*(Ct4*(St5*St7+Ct5*Ct6*Ct7)-Ct7*St4*St6)+Ct3*(Ct6*Ct7*St5-Ct5*St7))),2)+std::pow((Ct1*(Ct2*(Ct3*(Ct4*(St5*St7+Ct5*Ct6*Ct7)-Ct7*St4*St6)-St3*(Ct6*Ct7*St5-Ct5*St7))-St2*((-St4*(St5*St7+Ct5*Ct6*Ct7))-Ct4*Ct7*St6))-St1*(St3*(Ct4*(St5*St7+Ct5*Ct6*Ct7)-Ct7*St4*St6)+Ct3*(Ct6*Ct7*St5-Ct5*St7))),2))-((Ct1*(Ct2*(Ct3*(Ct4*(Ct7*St5-Ct5*Ct6*St7)+St4*St6*St7)-St3*((-Ct6*St5*St7)-Ct5*Ct7))-St2*(Ct4*St6*St7-St4*(Ct7*St5-Ct5*Ct6*St7)))-St1*(St3*(Ct4*(Ct7*St5-Ct5*Ct6*St7)+St4*St6*St7)+Ct3*((-Ct6*St5*St7)-Ct5*Ct7)))*(St1*(Ct2*(Ct3*(Ct4*(St5*St7+Ct5*Ct6*Ct7)-Ct7*St4*St6)-St3*(Ct6*Ct7*St5-Ct5*St7))-St2*((-St4*(St5*St7+Ct5*Ct6*Ct7))-Ct4*Ct7*St6))+Ct1*(St3*(Ct4*(St5*St7+Ct5*Ct6*Ct7)-Ct7*St4*St6)+Ct3*(Ct6*Ct7*St5-Ct5*St7))))/(std::pow((St1*(Ct2*(Ct3*(Ct4*(St5*St7+Ct5*Ct6*Ct7)-Ct7*St4*St6)-St3*(Ct6*Ct7*St5-Ct5*St7))-St2*((-St4*(St5*St7+Ct5*Ct6*Ct7))-Ct4*Ct7*St6))+Ct1*(St3*(Ct4*(St5*St7+Ct5*Ct6*Ct7)-Ct7*St4*St6)+Ct3*(Ct6*Ct7*St5-Ct5*St7))),2)+std::pow((Ct1*(Ct2*(Ct3*(Ct4*(St5*St7+Ct5*Ct6*Ct7)-Ct7*St4*St6)-St3*(Ct6*Ct7*St5-Ct5*St7))-St2*((-St4*(St5*St7+Ct5*Ct6*Ct7))-Ct4*Ct7*St6))-St1*(St3*(Ct4*(St5*St7+Ct5*Ct6*Ct7)-Ct7*St4*St6)+Ct3*(Ct6*Ct7*St5-Ct5*St7))),2));

    // clang-format on
  }
  inline double CalcJ51(double Ct1,
                        double St1,
                        double Ct2,
                        double St2,
                        double Ct3,
                        double St3,
                        double Ct4,
                        double St4,
                        double Ct5,
                        double St5,
                        double Ct6,
                        double St6,
                        double Ct7,
                        double St7)
  {
    // clang-format off
	return 
0;

    // clang-format on
  }
  inline double CalcJ52(double Ct1,
                        double St1,
                        double Ct2,
                        double St2,
                        double Ct3,
                        double St3,
                        double Ct4,
                        double St4,
                        double Ct5,
                        double St5,
                        double Ct6,
                        double St6,
                        double Ct7,
                        double St7)
  {
    // clang-format off
	return 
((Ct2*(Ct3*(Ct4*(St5*St7+Ct5*Ct6*Ct7)-Ct7*St4*St6)-St3*(Ct6*Ct7*St5-Ct5*St7))-St2*((-St4*(St5*St7+Ct5*Ct6*Ct7))-Ct4*Ct7*St6))*sqrt(std::pow(((-St2*(Ct3*(Ct4*(Ct7*St5-Ct5*Ct6*St7)+St4*St6*St7)-St3*((-Ct6*St5*St7)-Ct5*Ct7)))-Ct2*(Ct4*St6*St7-St4*(Ct7*St5-Ct5*Ct6*St7))),2)+std::pow(((-St2*(Ct3*(Ct4*Ct5*St6+Ct6*St4)-St3*St5*St6))-Ct2*(Ct4*Ct6-Ct5*St4*St6)),2)))/(std::pow((St2*(Ct3*(Ct4*(St5*St7+Ct5*Ct6*Ct7)-Ct7*St4*St6)-St3*(Ct6*Ct7*St5-Ct5*St7))+Ct2*((-St4*(St5*St7+Ct5*Ct6*Ct7))-Ct4*Ct7*St6)),2)+std::pow(((-St2*(Ct3*(Ct4*(Ct7*St5-Ct5*Ct6*St7)+St4*St6*St7)-St3*((-Ct6*St5*St7)-Ct5*Ct7)))-Ct2*(Ct4*St6*St7-St4*(Ct7*St5-Ct5*Ct6*St7))),2)+std::pow(((-St2*(Ct3*(Ct4*Ct5*St6+Ct6*St4)-St3*St5*St6))-Ct2*(Ct4*Ct6-Ct5*St4*St6)),2))-((St2*(Ct3*(Ct4*(St5*St7+Ct5*Ct6*Ct7)-Ct7*St4*St6)-St3*(Ct6*Ct7*St5-Ct5*St7))+Ct2*((-St4*(St5*St7+Ct5*Ct6*Ct7))-Ct4*Ct7*St6))*(2*(St2*(Ct4*St6*St7-St4*(Ct7*St5-Ct5*Ct6*St7))-Ct2*(Ct3*(Ct4*(Ct7*St5-Ct5*Ct6*St7)+St4*St6*St7)-St3*((-Ct6*St5*St7)-Ct5*Ct7)))*((-St2*(Ct3*(Ct4*(Ct7*St5-Ct5*Ct6*St7)+St4*St6*St7)-St3*((-Ct6*St5*St7)-Ct5*Ct7)))-Ct2*(Ct4*St6*St7-St4*(Ct7*St5-Ct5*Ct6*St7)))+2*(St2*(Ct4*Ct6-Ct5*St4*St6)-Ct2*(Ct3*(Ct4*Ct5*St6+Ct6*St4)-St3*St5*St6))*((-St2*(Ct3*(Ct4*Ct5*St6+Ct6*St4)-St3*St5*St6))-Ct2*(Ct4*Ct6-Ct5*St4*St6))))/(2*sqrt(std::pow(((-St2*(Ct3*(Ct4*(Ct7*St5-Ct5*Ct6*St7)+St4*St6*St7)-St3*((-Ct6*St5*St7)-Ct5*Ct7)))-Ct2*(Ct4*St6*St7-St4*(Ct7*St5-Ct5*Ct6*St7))),2)+std::pow(((-St2*(Ct3*(Ct4*Ct5*St6+Ct6*St4)-St3*St5*St6))-Ct2*(Ct4*Ct6-Ct5*St4*St6)),2))*(std::pow((St2*(Ct3*(Ct4*(St5*St7+Ct5*Ct6*Ct7)-Ct7*St4*St6)-St3*(Ct6*Ct7*St5-Ct5*St7))+Ct2*((-St4*(St5*St7+Ct5*Ct6*Ct7))-Ct4*Ct7*St6)),2)+std::pow(((-St2*(Ct3*(Ct4*(Ct7*St5-Ct5*Ct6*St7)+St4*St6*St7)-St3*((-Ct6*St5*St7)-Ct5*Ct7)))-Ct2*(Ct4*St6*St7-St4*(Ct7*St5-Ct5*Ct6*St7))),2)+std::pow(((-St2*(Ct3*(Ct4*Ct5*St6+Ct6*St4)-St3*St5*St6))-Ct2*(Ct4*Ct6-Ct5*St4*St6)),2)));

    // clang-format on
  }
  inline double CalcJ53(double Ct1,
                        double St1,
                        double Ct2,
                        double St2,
                        double Ct3,
                        double St3,
                        double Ct4,
                        double St4,
                        double Ct5,
                        double St5,
                        double Ct6,
                        double St6,
                        double Ct7,
                        double St7)
  {
    // clang-format off
	return 
(St2*((-St3*(Ct4*(St5*St7+Ct5*Ct6*Ct7)-Ct7*St4*St6))-Ct3*(Ct6*Ct7*St5-Ct5*St7))*sqrt(std::pow(((-St2*(Ct3*(Ct4*(Ct7*St5-Ct5*Ct6*St7)+St4*St6*St7)-St3*((-Ct6*St5*St7)-Ct5*Ct7)))-Ct2*(Ct4*St6*St7-St4*(Ct7*St5-Ct5*Ct6*St7))),2)+std::pow(((-St2*(Ct3*(Ct4*Ct5*St6+Ct6*St4)-St3*St5*St6))-Ct2*(Ct4*Ct6-Ct5*St4*St6)),2)))/(std::pow((St2*(Ct3*(Ct4*(St5*St7+Ct5*Ct6*Ct7)-Ct7*St4*St6)-St3*(Ct6*Ct7*St5-Ct5*St7))+Ct2*((-St4*(St5*St7+Ct5*Ct6*Ct7))-Ct4*Ct7*St6)),2)+std::pow(((-St2*(Ct3*(Ct4*(Ct7*St5-Ct5*Ct6*St7)+St4*St6*St7)-St3*((-Ct6*St5*St7)-Ct5*Ct7)))-Ct2*(Ct4*St6*St7-St4*(Ct7*St5-Ct5*Ct6*St7))),2)+std::pow(((-St2*(Ct3*(Ct4*Ct5*St6+Ct6*St4)-St3*St5*St6))-Ct2*(Ct4*Ct6-Ct5*St4*St6)),2))-((St2*(Ct3*(Ct4*(St5*St7+Ct5*Ct6*Ct7)-Ct7*St4*St6)-St3*(Ct6*Ct7*St5-Ct5*St7))+Ct2*((-St4*(St5*St7+Ct5*Ct6*Ct7))-Ct4*Ct7*St6))*((-2*St2*((-St3*(Ct4*(Ct7*St5-Ct5*Ct6*St7)+St4*St6*St7))-Ct3*((-Ct6*St5*St7)-Ct5*Ct7))*((-St2*(Ct3*(Ct4*(Ct7*St5-Ct5*Ct6*St7)+St4*St6*St7)-St3*((-Ct6*St5*St7)-Ct5*Ct7)))-Ct2*(Ct4*St6*St7-St4*(Ct7*St5-Ct5*Ct6*St7))))-2*St2*((-St3*(Ct4*Ct5*St6+Ct6*St4))-Ct3*St5*St6)*((-St2*(Ct3*(Ct4*Ct5*St6+Ct6*St4)-St3*St5*St6))-Ct2*(Ct4*Ct6-Ct5*St4*St6))))/(2*sqrt(std::pow(((-St2*(Ct3*(Ct4*(Ct7*St5-Ct5*Ct6*St7)+St4*St6*St7)-St3*((-Ct6*St5*St7)-Ct5*Ct7)))-Ct2*(Ct4*St6*St7-St4*(Ct7*St5-Ct5*Ct6*St7))),2)+std::pow(((-St2*(Ct3*(Ct4*Ct5*St6+Ct6*St4)-St3*St5*St6))-Ct2*(Ct4*Ct6-Ct5*St4*St6)),2))*(std::pow((St2*(Ct3*(Ct4*(St5*St7+Ct5*Ct6*Ct7)-Ct7*St4*St6)-St3*(Ct6*Ct7*St5-Ct5*St7))+Ct2*((-St4*(St5*St7+Ct5*Ct6*Ct7))-Ct4*Ct7*St6)),2)+std::pow(((-St2*(Ct3*(Ct4*(Ct7*St5-Ct5*Ct6*St7)+St4*St6*St7)-St3*((-Ct6*St5*St7)-Ct5*Ct7)))-Ct2*(Ct4*St6*St7-St4*(Ct7*St5-Ct5*Ct6*St7))),2)+std::pow(((-St2*(Ct3*(Ct4*Ct5*St6+Ct6*St4)-St3*St5*St6))-Ct2*(Ct4*Ct6-Ct5*St4*St6)),2)));

    // clang-format on
  }
  inline double CalcJ54(double Ct1,
                        double St1,
                        double Ct2,
                        double St2,
                        double Ct3,
                        double St3,
                        double Ct4,
                        double St4,
                        double Ct5,
                        double St5,
                        double Ct6,
                        double St6,
                        double Ct7,
                        double St7)
  {
    // clang-format off
	return 
((Ct3*St2*((-St4*(St5*St7+Ct5*Ct6*Ct7))-Ct4*Ct7*St6)+Ct2*(Ct7*St4*St6-Ct4*(St5*St7+Ct5*Ct6*Ct7)))*sqrt(std::pow(((-St2*(Ct3*(Ct4*(Ct7*St5-Ct5*Ct6*St7)+St4*St6*St7)-St3*((-Ct6*St5*St7)-Ct5*Ct7)))-Ct2*(Ct4*St6*St7-St4*(Ct7*St5-Ct5*Ct6*St7))),2)+std::pow(((-St2*(Ct3*(Ct4*Ct5*St6+Ct6*St4)-St3*St5*St6))-Ct2*(Ct4*Ct6-Ct5*St4*St6)),2)))/(std::pow((St2*(Ct3*(Ct4*(St5*St7+Ct5*Ct6*Ct7)-Ct7*St4*St6)-St3*(Ct6*Ct7*St5-Ct5*St7))+Ct2*((-St4*(St5*St7+Ct5*Ct6*Ct7))-Ct4*Ct7*St6)),2)+std::pow(((-St2*(Ct3*(Ct4*(Ct7*St5-Ct5*Ct6*St7)+St4*St6*St7)-St3*((-Ct6*St5*St7)-Ct5*Ct7)))-Ct2*(Ct4*St6*St7-St4*(Ct7*St5-Ct5*Ct6*St7))),2)+std::pow(((-St2*(Ct3*(Ct4*Ct5*St6+Ct6*St4)-St3*St5*St6))-Ct2*(Ct4*Ct6-Ct5*St4*St6)),2))-((St2*(Ct3*(Ct4*(St5*St7+Ct5*Ct6*Ct7)-Ct7*St4*St6)-St3*(Ct6*Ct7*St5-Ct5*St7))+Ct2*((-St4*(St5*St7+Ct5*Ct6*Ct7))-Ct4*Ct7*St6))*(2*((-Ct3*St2*(Ct4*St6*St7-St4*(Ct7*St5-Ct5*Ct6*St7)))-Ct2*((-Ct4*(Ct7*St5-Ct5*Ct6*St7))-St4*St6*St7))*((-St2*(Ct3*(Ct4*(Ct7*St5-Ct5*Ct6*St7)+St4*St6*St7)-St3*((-Ct6*St5*St7)-Ct5*Ct7)))-Ct2*(Ct4*St6*St7-St4*(Ct7*St5-Ct5*Ct6*St7)))+2*((-Ct3*St2*(Ct4*Ct6-Ct5*St4*St6))-Ct2*((-Ct4*Ct5*St6)-Ct6*St4))*((-St2*(Ct3*(Ct4*Ct5*St6+Ct6*St4)-St3*St5*St6))-Ct2*(Ct4*Ct6-Ct5*St4*St6))))/(2*sqrt(std::pow(((-St2*(Ct3*(Ct4*(Ct7*St5-Ct5*Ct6*St7)+St4*St6*St7)-St3*((-Ct6*St5*St7)-Ct5*Ct7)))-Ct2*(Ct4*St6*St7-St4*(Ct7*St5-Ct5*Ct6*St7))),2)+std::pow(((-St2*(Ct3*(Ct4*Ct5*St6+Ct6*St4)-St3*St5*St6))-Ct2*(Ct4*Ct6-Ct5*St4*St6)),2))*(std::pow((St2*(Ct3*(Ct4*(St5*St7+Ct5*Ct6*Ct7)-Ct7*St4*St6)-St3*(Ct6*Ct7*St5-Ct5*St7))+Ct2*((-St4*(St5*St7+Ct5*Ct6*Ct7))-Ct4*Ct7*St6)),2)+std::pow(((-St2*(Ct3*(Ct4*(Ct7*St5-Ct5*Ct6*St7)+St4*St6*St7)-St3*((-Ct6*St5*St7)-Ct5*Ct7)))-Ct2*(Ct4*St6*St7-St4*(Ct7*St5-Ct5*Ct6*St7))),2)+std::pow(((-St2*(Ct3*(Ct4*Ct5*St6+Ct6*St4)-St3*St5*St6))-Ct2*(Ct4*Ct6-Ct5*St4*St6)),2)));

    // clang-format on
  }
  inline double CalcJ55(double Ct1,
                        double St1,
                        double Ct2,
                        double St2,
                        double Ct3,
                        double St3,
                        double Ct4,
                        double St4,
                        double Ct5,
                        double St5,
                        double Ct6,
                        double St6,
                        double Ct7,
                        double St7)
  {
    // clang-format off
	return 
((St2*(Ct3*Ct4*(Ct5*St7-Ct6*Ct7*St5)-St3*(St5*St7+Ct5*Ct6*Ct7))-Ct2*St4*(Ct5*St7-Ct6*Ct7*St5))*sqrt(std::pow(((-St2*(Ct3*(Ct4*(Ct7*St5-Ct5*Ct6*St7)+St4*St6*St7)-St3*((-Ct6*St5*St7)-Ct5*Ct7)))-Ct2*(Ct4*St6*St7-St4*(Ct7*St5-Ct5*Ct6*St7))),2)+std::pow(((-St2*(Ct3*(Ct4*Ct5*St6+Ct6*St4)-St3*St5*St6))-Ct2*(Ct4*Ct6-Ct5*St4*St6)),2)))/(std::pow((St2*(Ct3*(Ct4*(St5*St7+Ct5*Ct6*Ct7)-Ct7*St4*St6)-St3*(Ct6*Ct7*St5-Ct5*St7))+Ct2*((-St4*(St5*St7+Ct5*Ct6*Ct7))-Ct4*Ct7*St6)),2)+std::pow(((-St2*(Ct3*(Ct4*(Ct7*St5-Ct5*Ct6*St7)+St4*St6*St7)-St3*((-Ct6*St5*St7)-Ct5*Ct7)))-Ct2*(Ct4*St6*St7-St4*(Ct7*St5-Ct5*Ct6*St7))),2)+std::pow(((-St2*(Ct3*(Ct4*Ct5*St6+Ct6*St4)-St3*St5*St6))-Ct2*(Ct4*Ct6-Ct5*St4*St6)),2))-((St2*(Ct3*(Ct4*(St5*St7+Ct5*Ct6*Ct7)-Ct7*St4*St6)-St3*(Ct6*Ct7*St5-Ct5*St7))+Ct2*((-St4*(St5*St7+Ct5*Ct6*Ct7))-Ct4*Ct7*St6))*(2*(Ct2*St4*(Ct6*St5*St7+Ct5*Ct7)-St2*(Ct3*Ct4*(Ct6*St5*St7+Ct5*Ct7)-St3*(Ct7*St5-Ct5*Ct6*St7)))*((-St2*(Ct3*(Ct4*(Ct7*St5-Ct5*Ct6*St7)+St4*St6*St7)-St3*((-Ct6*St5*St7)-Ct5*Ct7)))-Ct2*(Ct4*St6*St7-St4*(Ct7*St5-Ct5*Ct6*St7)))+2*((-St2*((-Ct3*Ct4*St5*St6)-Ct5*St3*St6))-Ct2*St4*St5*St6)*((-St2*(Ct3*(Ct4*Ct5*St6+Ct6*St4)-St3*St5*St6))-Ct2*(Ct4*Ct6-Ct5*St4*St6))))/(2*sqrt(std::pow(((-St2*(Ct3*(Ct4*(Ct7*St5-Ct5*Ct6*St7)+St4*St6*St7)-St3*((-Ct6*St5*St7)-Ct5*Ct7)))-Ct2*(Ct4*St6*St7-St4*(Ct7*St5-Ct5*Ct6*St7))),2)+std::pow(((-St2*(Ct3*(Ct4*Ct5*St6+Ct6*St4)-St3*St5*St6))-Ct2*(Ct4*Ct6-Ct5*St4*St6)),2))*(std::pow((St2*(Ct3*(Ct4*(St5*St7+Ct5*Ct6*Ct7)-Ct7*St4*St6)-St3*(Ct6*Ct7*St5-Ct5*St7))+Ct2*((-St4*(St5*St7+Ct5*Ct6*Ct7))-Ct4*Ct7*St6)),2)+std::pow(((-St2*(Ct3*(Ct4*(Ct7*St5-Ct5*Ct6*St7)+St4*St6*St7)-St3*((-Ct6*St5*St7)-Ct5*Ct7)))-Ct2*(Ct4*St6*St7-St4*(Ct7*St5-Ct5*Ct6*St7))),2)+std::pow(((-St2*(Ct3*(Ct4*Ct5*St6+Ct6*St4)-St3*St5*St6))-Ct2*(Ct4*Ct6-Ct5*St4*St6)),2)));

    // clang-format on
  }
  inline double CalcJ56(double Ct1,
                        double St1,
                        double Ct2,
                        double St2,
                        double Ct3,
                        double St3,
                        double Ct4,
                        double St4,
                        double Ct5,
                        double St5,
                        double Ct6,
                        double St6,
                        double Ct7,
                        double St7)
  {
    // clang-format off
	return 
((St2*(Ct3*((-Ct4*Ct5*Ct7*St6)-Ct6*Ct7*St4)+Ct7*St3*St5*St6)+Ct2*(Ct5*Ct7*St4*St6-Ct4*Ct6*Ct7))*sqrt(std::pow(((-St2*(Ct3*(Ct4*(Ct7*St5-Ct5*Ct6*St7)+St4*St6*St7)-St3*((-Ct6*St5*St7)-Ct5*Ct7)))-Ct2*(Ct4*St6*St7-St4*(Ct7*St5-Ct5*Ct6*St7))),2)+std::pow(((-St2*(Ct3*(Ct4*Ct5*St6+Ct6*St4)-St3*St5*St6))-Ct2*(Ct4*Ct6-Ct5*St4*St6)),2)))/(std::pow((St2*(Ct3*(Ct4*(St5*St7+Ct5*Ct6*Ct7)-Ct7*St4*St6)-St3*(Ct6*Ct7*St5-Ct5*St7))+Ct2*((-St4*(St5*St7+Ct5*Ct6*Ct7))-Ct4*Ct7*St6)),2)+std::pow(((-St2*(Ct3*(Ct4*(Ct7*St5-Ct5*Ct6*St7)+St4*St6*St7)-St3*((-Ct6*St5*St7)-Ct5*Ct7)))-Ct2*(Ct4*St6*St7-St4*(Ct7*St5-Ct5*Ct6*St7))),2)+std::pow(((-St2*(Ct3*(Ct4*Ct5*St6+Ct6*St4)-St3*St5*St6))-Ct2*(Ct4*Ct6-Ct5*St4*St6)),2))-((St2*(Ct3*(Ct4*(St5*St7+Ct5*Ct6*Ct7)-Ct7*St4*St6)-St3*(Ct6*Ct7*St5-Ct5*St7))+Ct2*((-St4*(St5*St7+Ct5*Ct6*Ct7))-Ct4*Ct7*St6))*(2*((-St2*(Ct3*(Ct4*Ct5*St6*St7+Ct6*St4*St7)-St3*St5*St6*St7))-Ct2*(Ct4*Ct6*St7-Ct5*St4*St6*St7))*((-St2*(Ct3*(Ct4*(Ct7*St5-Ct5*Ct6*St7)+St4*St6*St7)-St3*((-Ct6*St5*St7)-Ct5*Ct7)))-Ct2*(Ct4*St6*St7-St4*(Ct7*St5-Ct5*Ct6*St7)))+2*((-St2*(Ct3*(Ct4*Ct5*St6+Ct6*St4)-St3*St5*St6))-Ct2*(Ct4*Ct6-Ct5*St4*St6))*((-St2*(Ct3*(Ct4*Ct5*Ct6-St4*St6)-Ct6*St3*St5))-Ct2*((-Ct4*St6)-Ct5*Ct6*St4))))/(2*sqrt(std::pow(((-St2*(Ct3*(Ct4*(Ct7*St5-Ct5*Ct6*St7)+St4*St6*St7)-St3*((-Ct6*St5*St7)-Ct5*Ct7)))-Ct2*(Ct4*St6*St7-St4*(Ct7*St5-Ct5*Ct6*St7))),2)+std::pow(((-St2*(Ct3*(Ct4*Ct5*St6+Ct6*St4)-St3*St5*St6))-Ct2*(Ct4*Ct6-Ct5*St4*St6)),2))*(std::pow((St2*(Ct3*(Ct4*(St5*St7+Ct5*Ct6*Ct7)-Ct7*St4*St6)-St3*(Ct6*Ct7*St5-Ct5*St7))+Ct2*((-St4*(St5*St7+Ct5*Ct6*Ct7))-Ct4*Ct7*St6)),2)+std::pow(((-St2*(Ct3*(Ct4*(Ct7*St5-Ct5*Ct6*St7)+St4*St6*St7)-St3*((-Ct6*St5*St7)-Ct5*Ct7)))-Ct2*(Ct4*St6*St7-St4*(Ct7*St5-Ct5*Ct6*St7))),2)+std::pow(((-St2*(Ct3*(Ct4*Ct5*St6+Ct6*St4)-St3*St5*St6))-Ct2*(Ct4*Ct6-Ct5*St4*St6)),2)));

    // clang-format on
  }
  inline double CalcJ57(double Ct1,
                        double St1,
                        double Ct2,
                        double St2,
                        double Ct3,
                        double St3,
                        double Ct4,
                        double St4,
                        double Ct5,
                        double St5,
                        double Ct6,
                        double St6,
                        double Ct7,
                        double St7)
  {
    // clang-format off
	return 
((St2*(Ct3*(Ct4*(Ct7*St5-Ct5*Ct6*St7)+St4*St6*St7)-St3*((-Ct6*St5*St7)-Ct5*Ct7))+Ct2*(Ct4*St6*St7-St4*(Ct7*St5-Ct5*Ct6*St7)))*sqrt(std::pow(((-St2*(Ct3*(Ct4*(Ct7*St5-Ct5*Ct6*St7)+St4*St6*St7)-St3*((-Ct6*St5*St7)-Ct5*Ct7)))-Ct2*(Ct4*St6*St7-St4*(Ct7*St5-Ct5*Ct6*St7))),2)+std::pow(((-St2*(Ct3*(Ct4*Ct5*St6+Ct6*St4)-St3*St5*St6))-Ct2*(Ct4*Ct6-Ct5*St4*St6)),2)))/(std::pow((St2*(Ct3*(Ct4*(St5*St7+Ct5*Ct6*Ct7)-Ct7*St4*St6)-St3*(Ct6*Ct7*St5-Ct5*St7))+Ct2*((-St4*(St5*St7+Ct5*Ct6*Ct7))-Ct4*Ct7*St6)),2)+std::pow(((-St2*(Ct3*(Ct4*(Ct7*St5-Ct5*Ct6*St7)+St4*St6*St7)-St3*((-Ct6*St5*St7)-Ct5*Ct7)))-Ct2*(Ct4*St6*St7-St4*(Ct7*St5-Ct5*Ct6*St7))),2)+std::pow(((-St2*(Ct3*(Ct4*Ct5*St6+Ct6*St4)-St3*St5*St6))-Ct2*(Ct4*Ct6-Ct5*St4*St6)),2))-(((-St2*(Ct3*(Ct4*(Ct7*St5-Ct5*Ct6*St7)+St4*St6*St7)-St3*((-Ct6*St5*St7)-Ct5*Ct7)))-Ct2*(Ct4*St6*St7-St4*(Ct7*St5-Ct5*Ct6*St7)))*((-St2*(Ct3*(Ct4*((-St5*St7)-Ct5*Ct6*Ct7)+Ct7*St4*St6)-St3*(Ct5*St7-Ct6*Ct7*St5)))-Ct2*(Ct4*Ct7*St6-St4*((-St5*St7)-Ct5*Ct6*Ct7)))*(St2*(Ct3*(Ct4*(St5*St7+Ct5*Ct6*Ct7)-Ct7*St4*St6)-St3*(Ct6*Ct7*St5-Ct5*St7))+Ct2*((-St4*(St5*St7+Ct5*Ct6*Ct7))-Ct4*Ct7*St6)))/(sqrt(std::pow(((-St2*(Ct3*(Ct4*(Ct7*St5-Ct5*Ct6*St7)+St4*St6*St7)-St3*((-Ct6*St5*St7)-Ct5*Ct7)))-Ct2*(Ct4*St6*St7-St4*(Ct7*St5-Ct5*Ct6*St7))),2)+std::pow(((-St2*(Ct3*(Ct4*Ct5*St6+Ct6*St4)-St3*St5*St6))-Ct2*(Ct4*Ct6-Ct5*St4*St6)),2))*(std::pow((St2*(Ct3*(Ct4*(St5*St7+Ct5*Ct6*Ct7)-Ct7*St4*St6)-St3*(Ct6*Ct7*St5-Ct5*St7))+Ct2*((-St4*(St5*St7+Ct5*Ct6*Ct7))-Ct4*Ct7*St6)),2)+std::pow(((-St2*(Ct3*(Ct4*(Ct7*St5-Ct5*Ct6*St7)+St4*St6*St7)-St3*((-Ct6*St5*St7)-Ct5*Ct7)))-Ct2*(Ct4*St6*St7-St4*(Ct7*St5-Ct5*Ct6*St7))),2)+std::pow(((-St2*(Ct3*(Ct4*Ct5*St6+Ct6*St4)-St3*St5*St6))-Ct2*(Ct4*Ct6-Ct5*St4*St6)),2)));

    // clang-format on
  }
  inline double CalcJ61(double Ct1,
                        double St1,
                        double Ct2,
                        double St2,
                        double Ct3,
                        double St3,
                        double Ct4,
                        double St4,
                        double Ct5,
                        double St5,
                        double Ct6,
                        double St6,
                        double Ct7,
                        double St7)
  {
    // clang-format off
	return 
0;

    // clang-format on
  }
  inline double CalcJ62(double Ct1,
                        double St1,
                        double Ct2,
                        double St2,
                        double Ct3,
                        double St3,
                        double Ct4,
                        double St4,
                        double Ct5,
                        double St5,
                        double Ct6,
                        double St6,
                        double Ct7,
                        double St7)
  {
    // clang-format off
	return 
((St2*(Ct4*Ct6-Ct5*St4*St6)-Ct2*(Ct3*(Ct4*Ct5*St6+Ct6*St4)-St3*St5*St6))*(St2*(Ct3*(Ct4*(Ct7*St5-Ct5*Ct6*St7)+St4*St6*St7)-St3*((-Ct6*St5*St7)-Ct5*Ct7))+Ct2*(Ct4*St6*St7-St4*(Ct7*St5-Ct5*Ct6*St7))))/(std::pow((St2*(Ct3*(Ct4*(Ct7*St5-Ct5*Ct6*St7)+St4*St6*St7)-St3*((-Ct6*St5*St7)-Ct5*Ct7))+Ct2*(Ct4*St6*St7-St4*(Ct7*St5-Ct5*Ct6*St7))),2)+std::pow(((-St2*(Ct3*(Ct4*Ct5*St6+Ct6*St4)-St3*St5*St6))-Ct2*(Ct4*Ct6-Ct5*St4*St6)),2))-(((-St2*(Ct3*(Ct4*Ct5*St6+Ct6*St4)-St3*St5*St6))-Ct2*(Ct4*Ct6-Ct5*St4*St6))*(Ct2*(Ct3*(Ct4*(Ct7*St5-Ct5*Ct6*St7)+St4*St6*St7)-St3*((-Ct6*St5*St7)-Ct5*Ct7))-St2*(Ct4*St6*St7-St4*(Ct7*St5-Ct5*Ct6*St7))))/(std::pow((St2*(Ct3*(Ct4*(Ct7*St5-Ct5*Ct6*St7)+St4*St6*St7)-St3*((-Ct6*St5*St7)-Ct5*Ct7))+Ct2*(Ct4*St6*St7-St4*(Ct7*St5-Ct5*Ct6*St7))),2)+std::pow(((-St2*(Ct3*(Ct4*Ct5*St6+Ct6*St4)-St3*St5*St6))-Ct2*(Ct4*Ct6-Ct5*St4*St6)),2));

    // clang-format on
  }
  inline double CalcJ63(double Ct1,
                        double St1,
                        double Ct2,
                        double St2,
                        double Ct3,
                        double St3,
                        double Ct4,
                        double St4,
                        double Ct5,
                        double St5,
                        double Ct6,
                        double St6,
                        double Ct7,
                        double St7)
  {
    // clang-format off
	return 
(-(St2*((-St3*(Ct4*Ct5*St6+Ct6*St4))-Ct3*St5*St6)*(St2*(Ct3*(Ct4*(Ct7*St5-Ct5*Ct6*St7)+St4*St6*St7)-St3*((-Ct6*St5*St7)-Ct5*Ct7))+Ct2*(Ct4*St6*St7-St4*(Ct7*St5-Ct5*Ct6*St7))))/(std::pow((St2*(Ct3*(Ct4*(Ct7*St5-Ct5*Ct6*St7)+St4*St6*St7)-St3*((-Ct6*St5*St7)-Ct5*Ct7))+Ct2*(Ct4*St6*St7-St4*(Ct7*St5-Ct5*Ct6*St7))),2)+std::pow(((-St2*(Ct3*(Ct4*Ct5*St6+Ct6*St4)-St3*St5*St6))-Ct2*(Ct4*Ct6-Ct5*St4*St6)),2)))-(St2*((-St2*(Ct3*(Ct4*Ct5*St6+Ct6*St4)-St3*St5*St6))-Ct2*(Ct4*Ct6-Ct5*St4*St6))*((-St3*(Ct4*(Ct7*St5-Ct5*Ct6*St7)+St4*St6*St7))-Ct3*((-Ct6*St5*St7)-Ct5*Ct7)))/(std::pow((St2*(Ct3*(Ct4*(Ct7*St5-Ct5*Ct6*St7)+St4*St6*St7)-St3*((-Ct6*St5*St7)-Ct5*Ct7))+Ct2*(Ct4*St6*St7-St4*(Ct7*St5-Ct5*Ct6*St7))),2)+std::pow(((-St2*(Ct3*(Ct4*Ct5*St6+Ct6*St4)-St3*St5*St6))-Ct2*(Ct4*Ct6-Ct5*St4*St6)),2));

    // clang-format on
  }
  inline double CalcJ64(double Ct1,
                        double St1,
                        double Ct2,
                        double St2,
                        double Ct3,
                        double St3,
                        double Ct4,
                        double St4,
                        double Ct5,
                        double St5,
                        double Ct6,
                        double St6,
                        double Ct7,
                        double St7)
  {
    // clang-format off
	return 
(((-Ct3*St2*(Ct4*Ct6-Ct5*St4*St6))-Ct2*((-Ct4*Ct5*St6)-Ct6*St4))*(St2*(Ct3*(Ct4*(Ct7*St5-Ct5*Ct6*St7)+St4*St6*St7)-St3*((-Ct6*St5*St7)-Ct5*Ct7))+Ct2*(Ct4*St6*St7-St4*(Ct7*St5-Ct5*Ct6*St7))))/(std::pow((St2*(Ct3*(Ct4*(Ct7*St5-Ct5*Ct6*St7)+St4*St6*St7)-St3*((-Ct6*St5*St7)-Ct5*Ct7))+Ct2*(Ct4*St6*St7-St4*(Ct7*St5-Ct5*Ct6*St7))),2)+std::pow(((-St2*(Ct3*(Ct4*Ct5*St6+Ct6*St4)-St3*St5*St6))-Ct2*(Ct4*Ct6-Ct5*St4*St6)),2))-(((-St2*(Ct3*(Ct4*Ct5*St6+Ct6*St4)-St3*St5*St6))-Ct2*(Ct4*Ct6-Ct5*St4*St6))*(Ct3*St2*(Ct4*St6*St7-St4*(Ct7*St5-Ct5*Ct6*St7))+Ct2*((-Ct4*(Ct7*St5-Ct5*Ct6*St7))-St4*St6*St7)))/(std::pow((St2*(Ct3*(Ct4*(Ct7*St5-Ct5*Ct6*St7)+St4*St6*St7)-St3*((-Ct6*St5*St7)-Ct5*Ct7))+Ct2*(Ct4*St6*St7-St4*(Ct7*St5-Ct5*Ct6*St7))),2)+std::pow(((-St2*(Ct3*(Ct4*Ct5*St6+Ct6*St4)-St3*St5*St6))-Ct2*(Ct4*Ct6-Ct5*St4*St6)),2));

    // clang-format on
  }
  inline double CalcJ65(double Ct1,
                        double St1,
                        double Ct2,
                        double St2,
                        double Ct3,
                        double St3,
                        double Ct4,
                        double St4,
                        double Ct5,
                        double St5,
                        double Ct6,
                        double St6,
                        double Ct7,
                        double St7)
  {
    // clang-format off
	return 
(((-St2*((-Ct3*Ct4*St5*St6)-Ct5*St3*St6))-Ct2*St4*St5*St6)*(St2*(Ct3*(Ct4*(Ct7*St5-Ct5*Ct6*St7)+St4*St6*St7)-St3*((-Ct6*St5*St7)-Ct5*Ct7))+Ct2*(Ct4*St6*St7-St4*(Ct7*St5-Ct5*Ct6*St7))))/(std::pow((St2*(Ct3*(Ct4*(Ct7*St5-Ct5*Ct6*St7)+St4*St6*St7)-St3*((-Ct6*St5*St7)-Ct5*Ct7))+Ct2*(Ct4*St6*St7-St4*(Ct7*St5-Ct5*Ct6*St7))),2)+std::pow(((-St2*(Ct3*(Ct4*Ct5*St6+Ct6*St4)-St3*St5*St6))-Ct2*(Ct4*Ct6-Ct5*St4*St6)),2))-(((-St2*(Ct3*(Ct4*Ct5*St6+Ct6*St4)-St3*St5*St6))-Ct2*(Ct4*Ct6-Ct5*St4*St6))*(St2*(Ct3*Ct4*(Ct6*St5*St7+Ct5*Ct7)-St3*(Ct7*St5-Ct5*Ct6*St7))-Ct2*St4*(Ct6*St5*St7+Ct5*Ct7)))/(std::pow((St2*(Ct3*(Ct4*(Ct7*St5-Ct5*Ct6*St7)+St4*St6*St7)-St3*((-Ct6*St5*St7)-Ct5*Ct7))+Ct2*(Ct4*St6*St7-St4*(Ct7*St5-Ct5*Ct6*St7))),2)+std::pow(((-St2*(Ct3*(Ct4*Ct5*St6+Ct6*St4)-St3*St5*St6))-Ct2*(Ct4*Ct6-Ct5*St4*St6)),2));

    // clang-format on
  }
  inline double CalcJ66(double Ct1,
                        double St1,
                        double Ct2,
                        double St2,
                        double Ct3,
                        double St3,
                        double Ct4,
                        double St4,
                        double Ct5,
                        double St5,
                        double Ct6,
                        double St6,
                        double Ct7,
                        double St7)
  {
    // clang-format off
	return 
(((-St2*(Ct3*(Ct4*Ct5*Ct6-St4*St6)-Ct6*St3*St5))-Ct2*((-Ct4*St6)-Ct5*Ct6*St4))*(St2*(Ct3*(Ct4*(Ct7*St5-Ct5*Ct6*St7)+St4*St6*St7)-St3*((-Ct6*St5*St7)-Ct5*Ct7))+Ct2*(Ct4*St6*St7-St4*(Ct7*St5-Ct5*Ct6*St7))))/(std::pow((St2*(Ct3*(Ct4*(Ct7*St5-Ct5*Ct6*St7)+St4*St6*St7)-St3*((-Ct6*St5*St7)-Ct5*Ct7))+Ct2*(Ct4*St6*St7-St4*(Ct7*St5-Ct5*Ct6*St7))),2)+std::pow(((-St2*(Ct3*(Ct4*Ct5*St6+Ct6*St4)-St3*St5*St6))-Ct2*(Ct4*Ct6-Ct5*St4*St6)),2))-(((-St2*(Ct3*(Ct4*Ct5*St6+Ct6*St4)-St3*St5*St6))-Ct2*(Ct4*Ct6-Ct5*St4*St6))*(St2*(Ct3*(Ct4*Ct5*St6*St7+Ct6*St4*St7)-St3*St5*St6*St7)+Ct2*(Ct4*Ct6*St7-Ct5*St4*St6*St7)))/(std::pow((St2*(Ct3*(Ct4*(Ct7*St5-Ct5*Ct6*St7)+St4*St6*St7)-St3*((-Ct6*St5*St7)-Ct5*Ct7))+Ct2*(Ct4*St6*St7-St4*(Ct7*St5-Ct5*Ct6*St7))),2)+std::pow(((-St2*(Ct3*(Ct4*Ct5*St6+Ct6*St4)-St3*St5*St6))-Ct2*(Ct4*Ct6-Ct5*St4*St6)),2));

    // clang-format on
  }
  inline double CalcJ67(double Ct1,
                        double St1,
                        double Ct2,
                        double St2,
                        double Ct3,
                        double St3,
                        double Ct4,
                        double St4,
                        double Ct5,
                        double St5,
                        double Ct6,
                        double St6,
                        double Ct7,
                        double St7)
  {
    // clang-format off
	return 
-(((-St2*(Ct3*(Ct4*Ct5*St6+Ct6*St4)-St3*St5*St6))-Ct2*(Ct4*Ct6-Ct5*St4*St6))*(St2*(Ct3*(Ct4*((-St5*St7)-Ct5*Ct6*Ct7)+Ct7*St4*St6)-St3*(Ct5*St7-Ct6*Ct7*St5))+Ct2*(Ct4*Ct7*St6-St4*((-St5*St7)-Ct5*Ct6*Ct7))))/(std::pow((St2*(Ct3*(Ct4*(Ct7*St5-Ct5*Ct6*St7)+St4*St6*St7)-St3*((-Ct6*St5*St7)-Ct5*Ct7))+Ct2*(Ct4*St6*St7-St4*(Ct7*St5-Ct5*Ct6*St7))),2)+std::pow(((-St2*(Ct3*(Ct4*Ct5*St6+Ct6*St4)-St3*St5*St6))-Ct2*(Ct4*Ct6-Ct5*St4*St6)),2));

    // clang-format on
  }

  /**
   * @brief Calculates the jacobian for a 7-DoF robotarm
   *
   * @param aBigTheta
   * @return Matrix<double, 6, 7>
   */
  Matrix<double, 6, 7> calculateJacobiMatrix(const Configuration& aBigTheta)
  {
    const double lCosT1 = std::cos(aBigTheta[0]);
    const double lSinT1 = std::sin(aBigTheta[0]);
    const double lCosT2 = std::cos(aBigTheta[1]);
    const double lSinT2 = std::sin(aBigTheta[1]);
    const double lCosT3 = std::cos(aBigTheta[2]);
    const double lSinT3 = std::sin(aBigTheta[2]);
    const double lCosT4 = std::cos(aBigTheta[3]);
    const double lSinT4 = std::sin(aBigTheta[3]);
    const double lCosT5 = std::cos(aBigTheta[4]);
    const double lSinT5 = std::sin(aBigTheta[4]);
    const double lCosT6 = std::cos(aBigTheta[5]);
    const double lSinT6 = std::sin(aBigTheta[5]);
    const double lCosT7 = std::cos(aBigTheta[6]);
    const double lSinT7 = std::sin(aBigTheta[6]);
    // clang-format off
    Matrix<double, 6, 7> lJacobian{
      { CalcJ11(lCosT1, lSinT1, lCosT2, lSinT2, lCosT3, lSinT3, lCosT4, lSinT4, lCosT5, lSinT5, lCosT6, lSinT6, lCosT7, lSinT7),
        CalcJ12(lCosT1, lSinT1, lCosT2, lSinT2, lCosT3, lSinT3, lCosT4, lSinT4, lCosT5, lSinT5, lCosT6, lSinT6, lCosT7, lSinT7),
        CalcJ13(lCosT1, lSinT1, lCosT2, lSinT2, lCosT3, lSinT3, lCosT4, lSinT4, lCosT5, lSinT5, lCosT6, lSinT6, lCosT7, lSinT7),
        CalcJ14(lCosT1, lSinT1, lCosT2, lSinT2, lCosT3, lSinT3, lCosT4, lSinT4, lCosT5, lSinT5, lCosT6, lSinT6, lCosT7, lSinT7),
        CalcJ15(lCosT1, lSinT1, lCosT2, lSinT2, lCosT3, lSinT3, lCosT4, lSinT4, lCosT5, lSinT5, lCosT6, lSinT6, lCosT7, lSinT7),
        CalcJ16(lCosT1, lSinT1, lCosT2, lSinT2, lCosT3, lSinT3, lCosT4, lSinT4, lCosT5, lSinT5, lCosT6, lSinT6, lCosT7, lSinT7),
        CalcJ17(lCosT1, lSinT1, lCosT2, lSinT2, lCosT3, lSinT3, lCosT4, lSinT4, lCosT5, lSinT5, lCosT6, lSinT6, lCosT7, lSinT7) },

      { CalcJ21(lCosT1, lSinT1, lCosT2, lSinT2, lCosT3, lSinT3, lCosT4, lSinT4, lCosT5, lSinT5, lCosT6, lSinT6, lCosT7, lSinT7),
        CalcJ22(lCosT1, lSinT1, lCosT2, lSinT2, lCosT3, lSinT3, lCosT4, lSinT4, lCosT5, lSinT5, lCosT6, lSinT6, lCosT7, lSinT7),
        CalcJ23(lCosT1, lSinT1, lCosT2, lSinT2, lCosT3, lSinT3, lCosT4, lSinT4, lCosT5, lSinT5, lCosT6, lSinT6, lCosT7, lSinT7),
        CalcJ24(lCosT1, lSinT1, lCosT2, lSinT2, lCosT3, lSinT3, lCosT4, lSinT4, lCosT5, lSinT5, lCosT6, lSinT6, lCosT7, lSinT7),
        CalcJ25(lCosT1, lSinT1, lCosT2, lSinT2, lCosT3, lSinT3, lCosT4, lSinT4, lCosT5, lSinT5, lCosT6, lSinT6, lCosT7, lSinT7),
        CalcJ26(lCosT1, lSinT1, lCosT2, lSinT2, lCosT3, lSinT3, lCosT4, lSinT4, lCosT5, lSinT5, lCosT6, lSinT6, lCosT7, lSinT7),
        CalcJ27(lCosT1, lSinT1, lCosT2, lSinT2, lCosT3, lSinT3, lCosT4, lSinT4, lCosT5, lSinT5, lCosT6, lSinT6, lCosT7, lSinT7) },

      { CalcJ31(lCosT1, lSinT1, lCosT2, lSinT2, lCosT3, lSinT3, lCosT4, lSinT4, lCosT5, lSinT5, lCosT6, lSinT6, lCosT7, lSinT7),
        CalcJ32(lCosT1, lSinT1, lCosT2, lSinT2, lCosT3, lSinT3, lCosT4, lSinT4, lCosT5, lSinT5, lCosT6, lSinT6, lCosT7, lSinT7),
        CalcJ33(lCosT1, lSinT1, lCosT2, lSinT2, lCosT3, lSinT3, lCosT4, lSinT4, lCosT5, lSinT5, lCosT6, lSinT6, lCosT7, lSinT7),
        CalcJ34(lCosT1, lSinT1, lCosT2, lSinT2, lCosT3, lSinT3, lCosT4, lSinT4, lCosT5, lSinT5, lCosT6, lSinT6, lCosT7, lSinT7),
        CalcJ35(lCosT1, lSinT1, lCosT2, lSinT2, lCosT3, lSinT3, lCosT4, lSinT4, lCosT5, lSinT5, lCosT6, lSinT6, lCosT7, lSinT7),
        CalcJ36(lCosT1, lSinT1, lCosT2, lSinT2, lCosT3, lSinT3, lCosT4, lSinT4, lCosT5, lSinT5, lCosT6, lSinT6, lCosT7, lSinT7),
        CalcJ37(lCosT1, lSinT1, lCosT2, lSinT2, lCosT3, lSinT3, lCosT4, lSinT4, lCosT5, lSinT5, lCosT6, lSinT6, lCosT7, lSinT7) },

      { CalcJ41(lCosT1, lSinT1, lCosT2, lSinT2, lCosT3, lSinT3, lCosT4, lSinT4, lCosT5, lSinT5, lCosT6, lSinT6, lCosT7, lSinT7),
        CalcJ42(lCosT1, lSinT1, lCosT2, lSinT2, lCosT3, lSinT3, lCosT4, lSinT4, lCosT5, lSinT5, lCosT6, lSinT6, lCosT7, lSinT7),
        CalcJ43(lCosT1, lSinT1, lCosT2, lSinT2, lCosT3, lSinT3, lCosT4, lSinT4, lCosT5, lSinT5, lCosT6, lSinT6, lCosT7, lSinT7),
        CalcJ44(lCosT1, lSinT1, lCosT2, lSinT2, lCosT3, lSinT3, lCosT4, lSinT4, lCosT5, lSinT5, lCosT6, lSinT6, lCosT7, lSinT7),
        CalcJ45(lCosT1, lSinT1, lCosT2, lSinT2, lCosT3, lSinT3, lCosT4, lSinT4, lCosT5, lSinT5, lCosT6, lSinT6, lCosT7, lSinT7),
        CalcJ46(lCosT1, lSinT1, lCosT2, lSinT2, lCosT3, lSinT3, lCosT4, lSinT4, lCosT5, lSinT5, lCosT6, lSinT6, lCosT7, lSinT7),
        CalcJ47(lCosT1, lSinT1, lCosT2, lSinT2, lCosT3, lSinT3, lCosT4, lSinT4, lCosT5, lSinT5, lCosT6, lSinT6, lCosT7, lSinT7) },

      { CalcJ51(lCosT1, lSinT1, lCosT2, lSinT2, lCosT3, lSinT3, lCosT4, lSinT4, lCosT5, lSinT5, lCosT6, lSinT6, lCosT7, lSinT7),
        CalcJ52(lCosT1, lSinT1, lCosT2, lSinT2, lCosT3, lSinT3, lCosT4, lSinT4, lCosT5, lSinT5, lCosT6, lSinT6, lCosT7, lSinT7),
        CalcJ53(lCosT1, lSinT1, lCosT2, lSinT2, lCosT3, lSinT3, lCosT4, lSinT4, lCosT5, lSinT5, lCosT6, lSinT6, lCosT7, lSinT7),
        CalcJ54(lCosT1, lSinT1, lCosT2, lSinT2, lCosT3, lSinT3, lCosT4, lSinT4, lCosT5, lSinT5, lCosT6, lSinT6, lCosT7, lSinT7),
        CalcJ55(lCosT1, lSinT1, lCosT2, lSinT2, lCosT3, lSinT3, lCosT4, lSinT4, lCosT5, lSinT5, lCosT6, lSinT6, lCosT7, lSinT7),
        CalcJ56(lCosT1, lSinT1, lCosT2, lSinT2, lCosT3, lSinT3, lCosT4, lSinT4, lCosT5, lSinT5, lCosT6, lSinT6, lCosT7, lSinT7),
        CalcJ57(lCosT1, lSinT1, lCosT2, lSinT2, lCosT3, lSinT3, lCosT4, lSinT4, lCosT5, lSinT5, lCosT6, lSinT6, lCosT7, lSinT7) },

      { CalcJ61(lCosT1, lSinT1, lCosT2, lSinT2, lCosT3, lSinT3, lCosT4, lSinT4, lCosT5, lSinT5, lCosT6, lSinT6, lCosT7, lSinT7),
        CalcJ62(lCosT1, lSinT1, lCosT2, lSinT2, lCosT3, lSinT3, lCosT4, lSinT4, lCosT5, lSinT5, lCosT6, lSinT6, lCosT7, lSinT7),
        CalcJ63(lCosT1, lSinT1, lCosT2, lSinT2, lCosT3, lSinT3, lCosT4, lSinT4, lCosT5, lSinT5, lCosT6, lSinT6, lCosT7, lSinT7),
        CalcJ64(lCosT1, lSinT1, lCosT2, lSinT2, lCosT3, lSinT3, lCosT4, lSinT4, lCosT5, lSinT5, lCosT6, lSinT6, lCosT7, lSinT7),
        CalcJ65(lCosT1, lSinT1, lCosT2, lSinT2, lCosT3, lSinT3, lCosT4, lSinT4, lCosT5, lSinT5, lCosT6, lSinT6, lCosT7, lSinT7),
        CalcJ66(lCosT1, lSinT1, lCosT2, lSinT2, lCosT3, lSinT3, lCosT4, lSinT4, lCosT5, lSinT5, lCosT6, lSinT6, lCosT7, lSinT7),
        CalcJ67(lCosT1, lSinT1, lCosT2, lSinT2, lCosT3, lSinT3, lCosT4, lSinT4, lCosT5, lSinT5, lCosT6, lSinT6, lCosT7, lSinT7) }
    };
    //clang-format on
    return lJacobian;
  }
} // namespace kinematics
#endif // KINEMATICS_JACOBIMATRIX_HPP
