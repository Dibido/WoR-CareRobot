// Bring in gtest
#include <gtest/gtest.h>
// Bring in my package's API, which is what I'm testing
// Make everything public in to test code.
#define private public
#include <agv_parser/AgvParser.hpp>
#undef private

/**
 * @brief Test the Agv Data
 * Check that the Agv Data that is sent to the Parser is handled properly
 */
TEST(AgvParser, AgvDataTooShort)
{
  // Create with default port
  agv_parser::AgvParser lAgvParser("/dev/ttyUSB0");
  std::string lData = "#S#";
  try
  {
    lAgvParser.parseRecievedMessage(lData);
    FAIL() << "Expected std::invalid_argument";
  }
  catch (std::invalid_argument const& err)
  {
    EXPECT_EQ(err.what(), std::string("The message length is invalid"));
  }
  catch (...)
  {
    FAIL() << "Expected std::invalid_argument";
  }
}

TEST(AgvParser, AgvDataWrongFormat)
{
  // Create with default port
  agv_parser::AgvParser lAgvParser("/dev/ttyUSB0");
  std::string lData = "#a#0.23";
  try
  {
    lAgvParser.parseRecievedMessage(lData);
    FAIL() << "Expected std::invalid_argument";
  }
  catch (std::invalid_argument const& err)
  {
    EXPECT_EQ(err.what(), std::string("The message format is invalid"));
  }
  catch (...)
  {
    FAIL() << "Expected std::invalid_argument";
  }
}

TEST(AgvParser, AgvDataWrongValue)
{
  // Create with default port
  agv_parser::AgvParser lAgvParser("/dev/ttyUSB0");
  std::string lData = "#S#-2";
  try
  {
    lAgvParser.parseRecievedMessage(lData);
    FAIL() << "Expected std::invalid_argument";
  }
  catch (std::invalid_argument const& err)
  {
    EXPECT_EQ(err.what(), std::string("The speed is invalid"));
  }
  catch (...)
  {
    FAIL() << "Expected std::invalid_argument";
  }
}

TEST(AgvParser, AgvDataCorrect)
{
  // Create with default port
  const double lMaximumDeviation = 0.00005;
  agv_parser::AgvParser lAgvParser("/dev/ttyUSB0");
  std::string lData = "#S#0.232";
  float lAgvSpeed = lAgvParser.parseRecievedMessage(lData);
  EXPECT_NEAR(lAgvSpeed, 0.232, lMaximumDeviation);
}
