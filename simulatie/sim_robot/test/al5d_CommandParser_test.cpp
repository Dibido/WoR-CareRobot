// https://stackoverflow.com/questions/13547771/g-project-compilation-with-boost-unit-test
#define BOOST_TEST_DYN_LINK                   // this is optional
#define BOOST_TEST_MODULE CommandParser_test  // specify the name of your test module

#include <boost/test/included/unit_test.hpp>  // include this to get main()

#include <sim_robot/CommandParser.hpp>
#include <sim_robot/Command.hpp>

BOOST_AUTO_TEST_SUITE(CommandParser_test)

// test if the string splitting works
BOOST_AUTO_TEST_CASE(testStringSplit)
{
  commands::CommandParser parser;
  std::vector<std::string> testVec;
  std::vector<std::string> testVec2 = { "test1", "test2" };

  parser.splitString("test1\rtest2", testVec);

  BOOST_CHECK(testVec.size() == testVec2.size());
  BOOST_CHECK_EQUAL(testVec.at(0), testVec2.at(0));
  BOOST_CHECK_EQUAL(testVec.at(1), testVec2.at(1));
}

// test for positive cases of isMoveCommand
BOOST_AUTO_TEST_CASE(testStringIsMove_1)
{
  commands::CommandParser parser;
  BOOST_CHECK(parser.isMoveCommand("#100P200"));
}
BOOST_AUTO_TEST_CASE(testStringIsMove_2)
{
  commands::CommandParser parser;
  BOOST_CHECK(parser.isMoveCommand("#100P200#100P200"));
}
BOOST_AUTO_TEST_CASE(testStringIsMove_3)
{
  commands::CommandParser parser;
  BOOST_CHECK(parser.isMoveCommand("#100P200S300"));
}
BOOST_AUTO_TEST_CASE(testStringIsMove_4)
{
  commands::CommandParser parser;
  BOOST_CHECK(parser.isMoveCommand("#100P200S300#100P200S300"));
}
BOOST_AUTO_TEST_CASE(testStringIsMove_5)
{
  commands::CommandParser parser;
  BOOST_CHECK(parser.isMoveCommand("#100P200S300#100P200"));
}
BOOST_AUTO_TEST_CASE(testStringIsMove_6)
{
  commands::CommandParser parser;
  BOOST_CHECK(parser.isMoveCommand("#100P200#100P200S300"));
}
BOOST_AUTO_TEST_CASE(testStringIsMove_7)
{
  commands::CommandParser parser;
  BOOST_CHECK(parser.isMoveCommand("#100P-200#100P200S300"));
}
BOOST_AUTO_TEST_CASE(testStringIsMove_8)
{
  commands::CommandParser parser;
  BOOST_CHECK(parser.isMoveCommand("#100P200#100P-200S300"));
}
BOOST_AUTO_TEST_CASE(testStringIsMove_9)
{
  commands::CommandParser parser;
  BOOST_CHECK(parser.isMoveCommand("#100P-200#100P-200S300"));
}
BOOST_AUTO_TEST_CASE(testStringIsMove_10)
{
  commands::CommandParser parser;
  BOOST_CHECK(parser.isMoveCommand("#100P200.2#100P200S300"));
}
BOOST_AUTO_TEST_CASE(testStringIsMove_11)
{
  commands::CommandParser parser;
  BOOST_CHECK(parser.isMoveCommand("#100P200#100P200.2S300"));
}
BOOST_AUTO_TEST_CASE(testStringIsMove_12)
{
  commands::CommandParser parser;
  BOOST_CHECK(parser.isMoveCommand("#100P200.2#100P200.2S300"));
}

BOOST_AUTO_TEST_CASE(testStringIsMove_13)
{
  commands::CommandParser parser;
  BOOST_CHECK_EQUAL(parser.isMoveCommand("#100P--200#100P200S300"), false);
}
BOOST_AUTO_TEST_CASE(testStringIsMove_14)
{
  commands::CommandParser parser;
  BOOST_CHECK_EQUAL(parser.isMoveCommand("#100P200#100P--200S300"), false);
}
BOOST_AUTO_TEST_CASE(testStringIsMove_15)
{
  commands::CommandParser parser;
  BOOST_CHECK_EQUAL(parser.isMoveCommand("#100P--200#100P--200S300"), false);
}
BOOST_AUTO_TEST_CASE(testStringIsMove_16)
{
  commands::CommandParser parser;
  BOOST_CHECK_EQUAL(parser.isMoveCommand("#100P200.2.2#100P200S300"), false);
}
BOOST_AUTO_TEST_CASE(testStringIsMove_17)
{
  commands::CommandParser parser;
  BOOST_CHECK_EQUAL(parser.isMoveCommand("#100P200#100P200.2.2S300"), false);
}
BOOST_AUTO_TEST_CASE(testStringIsMove_18)
{
  commands::CommandParser parser;
  BOOST_CHECK_EQUAL(parser.isMoveCommand("#100P200.2.2#100P200.2.2S300"), false);
}
BOOST_AUTO_TEST_CASE(testStringIsMove_19)
{
  commands::CommandParser parser;
  BOOST_CHECK_EQUAL(parser.isMoveCommand("#100"), false);
}
BOOST_AUTO_TEST_CASE(testStringIsMove_20)
{
  commands::CommandParser parser;
  BOOST_CHECK_EQUAL(parser.isMoveCommand("P100"), false);
}
BOOST_AUTO_TEST_CASE(testStringIsMove_21)
{
  commands::CommandParser parser;
  BOOST_CHECK_EQUAL(parser.isMoveCommand("S100"), false);
}
BOOST_AUTO_TEST_CASE(testStringIsMove_22)
{
  commands::CommandParser parser;
  BOOST_CHECK_EQUAL(parser.isMoveCommand("geenCommando"), false);
}
BOOST_AUTO_TEST_CASE(testStringIsMove_23)
{
  commands::CommandParser parser;
  BOOST_CHECK(parser.isMoveCommand("#0P1600#1P1543#2P700#3P1500#4P500#5P1450T500"));
}
BOOST_AUTO_TEST_CASE(testStringIsMove_24)
{
  commands::CommandParser parser;
  BOOST_CHECK_EQUAL(parser.isMoveCommand("#0P100S-200"), false);
}
BOOST_AUTO_TEST_CASE(testStringIsMove_25)
{
  commands::CommandParser parser;
  BOOST_CHECK_EQUAL(parser.isMoveCommand("#0P100S200.2"), false);
}
BOOST_AUTO_TEST_CASE(testStringIsMove_26)
{
  commands::CommandParser parser;
  BOOST_CHECK_EQUAL(parser.isMoveCommand("#-0P100S200"), false);
}
BOOST_AUTO_TEST_CASE(testStringIsMove_27)
{
  commands::CommandParser parser;
  BOOST_CHECK_EQUAL(parser.isMoveCommand("#0.2P100S200"), false);
}

// test isStopCommand
BOOST_AUTO_TEST_CASE(testStringIsStop_1)
{
  commands::CommandParser parser;
  BOOST_CHECK(parser.isStopCommand("STOP1"));
}
BOOST_AUTO_TEST_CASE(testStringIsStop_2)
{
  commands::CommandParser parser;
  BOOST_CHECK(parser.isStopCommand("STOP1000"));
}
BOOST_AUTO_TEST_CASE(testStringIsStop_3)
{
  commands::CommandParser parser;
  BOOST_CHECK(parser.isStopCommand(" STOP1"));
}
BOOST_AUTO_TEST_CASE(testStringIsStop_4)
{
  commands::CommandParser parser;
  BOOST_CHECK_EQUAL(parser.isStopCommand("STO1"), false);
}
BOOST_AUTO_TEST_CASE(testStringIsStop_5)
{
  commands::CommandParser parser;
  BOOST_CHECK_EQUAL(parser.isStopCommand("geenSTOPCommando"), false);
}

BOOST_AUTO_TEST_CASE(parseCommand_1)
{
  commands::CommandParser parser;
  std::vector<commands::Command> testContainer;
  parser.parseCommand("#100P200", testContainer);
  BOOST_CHECK(testContainer.size() == 1);
  BOOST_CHECK(testContainer.at(0).getType() == commands::MOVE);
  BOOST_CHECK(testContainer.at(0).getChannel() == 100);
  BOOST_CHECK(testContainer.at(0).getPwm() == 200);
  BOOST_CHECK(testContainer.at(0).getSpeed() == 0);
  BOOST_CHECK(testContainer.at(0).getTime() == 0);
}

BOOST_AUTO_TEST_CASE(parseCommand_2)
{
  commands::CommandParser parser;
  std::vector<commands::Command> testContainer;
  parser.parseCommand("#100P200#100P200", testContainer);
  BOOST_CHECK(testContainer.size() == 2);
  BOOST_CHECK(testContainer.at(0).getType() == commands::MOVE);
  BOOST_CHECK(testContainer.at(0).getChannel() == 100);
  BOOST_CHECK(testContainer.at(0).getPwm() == 200);
  BOOST_CHECK(testContainer.at(0).getSpeed() == 0);
  BOOST_CHECK(testContainer.at(0).getTime() == 0);
  BOOST_CHECK(testContainer.at(1).getType() == commands::MOVE);
  BOOST_CHECK(testContainer.at(1).getChannel() == 100);
  BOOST_CHECK(testContainer.at(1).getPwm() == 200);
  BOOST_CHECK(testContainer.at(1).getSpeed() == 0);
  BOOST_CHECK(testContainer.at(1).getTime() == 0);
}

BOOST_AUTO_TEST_CASE(createCommandContainerMove_3)
{
  commands::CommandParser parser;
  std::vector<commands::Command> testContainer;
  parser.parseCommand("#100P200T500", testContainer);
  BOOST_CHECK(testContainer.size() == 1);
  BOOST_CHECK(testContainer.at(0).getType() == commands::MOVE);
  BOOST_CHECK(testContainer.at(0).getChannel() == 100);
  BOOST_CHECK(testContainer.at(0).getPwm() == 200);
  BOOST_CHECK(testContainer.at(0).getSpeed() == 0);
  BOOST_CHECK(testContainer.at(0).getTime() == 500);
}

BOOST_AUTO_TEST_CASE(createCommandContainerMove_4)
{
  commands::CommandParser parser;
  std::vector<commands::Command> testContainer;
  parser.parseCommand("#100P200#100P200T500", testContainer);
  BOOST_CHECK(testContainer.size() == 2);
  BOOST_CHECK(testContainer.at(0).getType() == commands::MOVE);
  BOOST_CHECK(testContainer.at(0).getChannel() == 100);
  BOOST_CHECK(testContainer.at(0).getPwm() == 200);
  BOOST_CHECK(testContainer.at(0).getSpeed() == 0);
  BOOST_CHECK(testContainer.at(0).getTime() == 500);
  BOOST_CHECK(testContainer.at(1).getType() == commands::MOVE);
  BOOST_CHECK(testContainer.at(1).getChannel() == 100);
  BOOST_CHECK(testContainer.at(1).getPwm() == 200);
  BOOST_CHECK(testContainer.at(1).getSpeed() == 0);
  BOOST_CHECK(testContainer.at(1).getTime() == 500);
}

BOOST_AUTO_TEST_CASE(createCommandContainerMove_5)
{
  commands::CommandParser parser;
  std::vector<commands::Command> testContainer;
  parser.parseCommand("geenCommando", testContainer);  // geen
  BOOST_CHECK(testContainer.size() == 0);
}

BOOST_AUTO_TEST_CASE(createCommandContainerMove_6)
{
  commands::CommandParser parser;
  std::vector<commands::Command> testContainer;
  parser.parseCommand("#100P200S300", testContainer);
  BOOST_CHECK(testContainer.size() == 1);
  BOOST_CHECK(testContainer.at(0).getType() == commands::MOVE);
  BOOST_CHECK(testContainer.at(0).getChannel() == 100);
  BOOST_CHECK(testContainer.at(0).getPwm() == 200);
  BOOST_CHECK(testContainer.at(0).getSpeed() == 300);
  BOOST_CHECK(testContainer.at(0).getTime() == 0);
}

BOOST_AUTO_TEST_CASE(createCommandContainerMove_7)
{
  commands::CommandParser parser;
  std::vector<commands::Command> testContainer;
  parser.parseCommand("#100P200S300#100P200", testContainer);
  BOOST_CHECK(testContainer.size() == 2);
  BOOST_CHECK(testContainer.at(0).getType() == commands::MOVE);
  BOOST_CHECK(testContainer.at(0).getChannel() == 100);
  BOOST_CHECK(testContainer.at(0).getPwm() == 200);
  BOOST_CHECK(testContainer.at(0).getSpeed() == 300);
  BOOST_CHECK(testContainer.at(0).getTime() == 0);
  BOOST_CHECK(testContainer.at(1).getType() == commands::MOVE);
  BOOST_CHECK(testContainer.at(1).getChannel() == 100);
  BOOST_CHECK(testContainer.at(1).getPwm() == 200);
  BOOST_CHECK(testContainer.at(1).getSpeed() == 0);
  BOOST_CHECK(testContainer.at(1).getTime() == 0);
}

BOOST_AUTO_TEST_CASE(createCommandContainerMove_8)
{
  commands::CommandParser parser;
  std::vector<commands::Command> testContainer;
  parser.parseCommand("#100P200#100P200S300", testContainer);
  BOOST_CHECK(testContainer.size() == 2);
  BOOST_CHECK(testContainer.at(0).getType() == commands::MOVE);
  BOOST_CHECK(testContainer.at(0).getChannel() == 100);
  BOOST_CHECK(testContainer.at(0).getPwm() == 200);
  BOOST_CHECK(testContainer.at(0).getSpeed() == 0);
  BOOST_CHECK(testContainer.at(0).getTime() == 0);
  BOOST_CHECK(testContainer.at(1).getType() == commands::MOVE);
  BOOST_CHECK(testContainer.at(1).getChannel() == 100);
  BOOST_CHECK(testContainer.at(1).getPwm() == 200);
  BOOST_CHECK(testContainer.at(1).getSpeed() == 300);
  BOOST_CHECK(testContainer.at(1).getTime() == 0);
}

BOOST_AUTO_TEST_CASE(createCommandContainerMove_9)
{
  commands::CommandParser parser;
  std::vector<commands::Command> testContainer;
  parser.parseCommand("#100P200#100P200S300T500", testContainer);
  BOOST_CHECK(testContainer.size() == 2);
  BOOST_CHECK(testContainer.at(0).getType() == commands::MOVE);
  BOOST_CHECK(testContainer.at(0).getChannel() == 100);
  BOOST_CHECK(testContainer.at(0).getPwm() == 200);
  BOOST_CHECK(testContainer.at(0).getSpeed() == 0);
  BOOST_CHECK(testContainer.at(0).getTime() == 500);
  BOOST_CHECK(testContainer.at(1).getType() == commands::MOVE);
  BOOST_CHECK(testContainer.at(1).getChannel() == 100);
  BOOST_CHECK(testContainer.at(1).getPwm() == 200);
  BOOST_CHECK(testContainer.at(1).getSpeed() == 300);
  BOOST_CHECK(testContainer.at(1).getTime() == 500);
}

BOOST_AUTO_TEST_CASE(createCommandContainerMove_10)
{
  commands::CommandParser parser;
  std::vector<commands::Command> testContainer;
  parser.parseCommand("#0P1600#1P1543#2P700#3P1500#4P500#5P1450T500", testContainer);
  BOOST_CHECK(testContainer.size() == 6);
  BOOST_CHECK(testContainer.at(0).getType() == commands::MOVE);
  BOOST_CHECK(testContainer.at(0).getChannel() == 0);
  BOOST_CHECK(testContainer.at(0).getPwm() == 1600);
  BOOST_CHECK(testContainer.at(0).getSpeed() == 0);
  BOOST_CHECK(testContainer.at(0).getTime() == 500);
  BOOST_CHECK(testContainer.at(1).getType() == commands::MOVE);
  BOOST_CHECK(testContainer.at(1).getChannel() == 1);
  BOOST_CHECK(testContainer.at(1).getPwm() == 1543);
  BOOST_CHECK(testContainer.at(1).getSpeed() == 0);
  BOOST_CHECK(testContainer.at(1).getTime() == 500);
  BOOST_CHECK(testContainer.at(2).getType() == commands::MOVE);
  BOOST_CHECK(testContainer.at(2).getChannel() == 2);
  BOOST_CHECK(testContainer.at(2).getPwm() == 700);
  BOOST_CHECK(testContainer.at(2).getSpeed() == 0);
  BOOST_CHECK(testContainer.at(2).getTime() == 500);
  BOOST_CHECK(testContainer.at(3).getType() == commands::MOVE);
  BOOST_CHECK(testContainer.at(3).getChannel() == 3);
  BOOST_CHECK(testContainer.at(3).getPwm() == 1500);
  BOOST_CHECK(testContainer.at(3).getSpeed() == 0);
  BOOST_CHECK(testContainer.at(3).getTime() == 500);
  BOOST_CHECK(testContainer.at(4).getType() == commands::MOVE);
  BOOST_CHECK(testContainer.at(4).getChannel() == 4);
  BOOST_CHECK(testContainer.at(4).getPwm() == 500);
  BOOST_CHECK(testContainer.at(4).getSpeed() == 0);
  BOOST_CHECK(testContainer.at(4).getTime() == 500);
  BOOST_CHECK(testContainer.at(5).getType() == commands::MOVE);
  BOOST_CHECK(testContainer.at(5).getChannel() == 5);
  BOOST_CHECK(testContainer.at(5).getPwm() == 1450);
  BOOST_CHECK(testContainer.at(5).getSpeed() == 0);
  BOOST_CHECK(testContainer.at(5).getTime() == 500);
}

BOOST_AUTO_TEST_CASE(createCommandContainerMove_11)
{
  commands::CommandParser parser;
  std::vector<commands::Command> testContainer;
  parser.parseCommand("#0P1000", testContainer);
  BOOST_CHECK(testContainer.size() == 1);
  BOOST_CHECK(testContainer.at(0).getType() == commands::MOVE);
  BOOST_CHECK(testContainer.at(0).getChannel() == 0);
  BOOST_CHECK(testContainer.at(0).getPwm() == 1000);
  BOOST_CHECK(testContainer.at(0).getSpeed() == 0);
  BOOST_CHECK(testContainer.at(0).getTime() == 0);
}

BOOST_AUTO_TEST_CASE(createCommandContainerMove_12)
{
  commands::CommandParser parser;
  std::vector<commands::Command> testContainer;
  parser.parseCommand("#0P-1000", testContainer);
  BOOST_CHECK(testContainer.size() == 1);
  BOOST_CHECK(testContainer.at(0).getType() == commands::MOVE);
  BOOST_CHECK(testContainer.at(0).getChannel() == 0);
  BOOST_CHECK(testContainer.at(0).getPwm() == -1000);
  BOOST_CHECK(testContainer.at(0).getSpeed() == 0);
  BOOST_CHECK(testContainer.at(0).getTime() == 0);
}

BOOST_AUTO_TEST_CASE(createCommandContainerMove_13)
{
  commands::CommandParser parser;
  std::vector<commands::Command> testContainer;
  parser.parseCommand("#0P--1000", testContainer);
  BOOST_CHECK(testContainer.size() == 0);
}

BOOST_AUTO_TEST_CASE(createCommandContainerMove_14)
{
  commands::CommandParser parser;
  std::vector<commands::Command> testContainer;
  parser.parseCommand("#0P1000.2", testContainer);
  BOOST_CHECK(testContainer.size() == 1);
  BOOST_CHECK(testContainer.at(0).getType() == commands::MOVE);
  BOOST_CHECK(testContainer.at(0).getChannel() == 0);
  BOOST_CHECK(testContainer.at(0).getPwm() == 1000.2);
  BOOST_CHECK(testContainer.at(0).getSpeed() == 0);
  BOOST_CHECK(testContainer.at(0).getTime() == 0);
}

BOOST_AUTO_TEST_CASE(createCommandContainerMove_15)
{
  commands::CommandParser parser;
  std::vector<commands::Command> testContainer;
  parser.parseCommand("#0P1000.2.2", testContainer);
  BOOST_CHECK(testContainer.size() == 0);
}

BOOST_AUTO_TEST_CASE(createCommandContainerMove_16)
{
  commands::CommandParser parser;
  std::vector<commands::Command> testContainer;
  parser.parseCommand("#0P-1000.2", testContainer);
  BOOST_CHECK(testContainer.size() == 1);
  BOOST_CHECK(testContainer.at(0).getType() == commands::MOVE);
  BOOST_CHECK(testContainer.at(0).getChannel() == 0);
  BOOST_CHECK(testContainer.at(0).getPwm() == -1000.2);
  BOOST_CHECK(testContainer.at(0).getSpeed() == 0);
  BOOST_CHECK(testContainer.at(0).getTime() == 0);
}

BOOST_AUTO_TEST_CASE(createCommandContainerMove_17)
{
  commands::CommandParser parser;
  std::vector<commands::Command> testContainer;
  parser.parseCommand("#0P--1000.2", testContainer);
  BOOST_CHECK(testContainer.size() == 0);
}

BOOST_AUTO_TEST_CASE(createCommandContainerMove_18)
{
  commands::CommandParser parser;
  std::vector<commands::Command> testContainer;
  parser.parseCommand("#0P-1000.2.2", testContainer);
  BOOST_CHECK(testContainer.size() == 0);
}
BOOST_AUTO_TEST_CASE(createCommandContainerMove_19)
{
  commands::CommandParser parser;
  std::vector<commands::Command> testContainer;
  parser.parseCommand("#0P1000S200", testContainer);
  BOOST_CHECK(testContainer.size() == 1);
}
BOOST_AUTO_TEST_CASE(createCommandContainerMove_20)
{
  commands::CommandParser parser;
  std::vector<commands::Command> testContainer;
  parser.parseCommand("#-0P1000S200", testContainer);
  BOOST_CHECK(testContainer.size() == 0);
}
BOOST_AUTO_TEST_CASE(createCommandContainerMove_21)
{
  commands::CommandParser parser;
  std::vector<commands::Command> testContainer;
  parser.parseCommand("#0.2P1000S200", testContainer);
  BOOST_CHECK(testContainer.size() == 0);
}
BOOST_AUTO_TEST_CASE(createCommandContainerMove_22)
{
  commands::CommandParser parser;
  std::vector<commands::Command> testContainer;
  parser.parseCommand("#0P1000S-200", testContainer);
  BOOST_CHECK(testContainer.size() == 0);
}
BOOST_AUTO_TEST_CASE(createCommandContainerMove_23)
{
  commands::CommandParser parser;
  std::vector<commands::Command> testContainer;
  parser.parseCommand("#0P1000S200.2", testContainer);
  BOOST_CHECK(testContainer.size() == 0);
}

BOOST_AUTO_TEST_CASE(createCommandContainerMove_containerSize)
{
  commands::CommandParser parser;
  std::vector<commands::Command> testContainer;
  parser.parseCommand("#100P200", testContainer);                                      // 0
  parser.parseCommand("#100P200#100P200", testContainer);                              // 1, 2
  parser.parseCommand("#100P200T500", testContainer);                                  // 3
  parser.parseCommand("#100P200#100P200T500", testContainer);                          // 4, 5
  parser.parseCommand("geenCommando", testContainer);                                  // geen
  parser.parseCommand("#100P200S300", testContainer);                                  // 6
  parser.parseCommand("#100P200S300#100P200", testContainer);                          // 7, 8
  parser.parseCommand("#100P200#100P200S300", testContainer);                          // 9, 10
  parser.parseCommand("#100P200#100P200S300T500", testContainer);                      // 11, 12
  parser.parseCommand("#0P1600#1P1543#2P700#3P1500#4P500#5P1450T500", testContainer);  // 13...18
  parser.parseCommand("#0P1000", testContainer);                                       // 19
  parser.parseCommand("#0P-1000", testContainer);                                      // 20
  parser.parseCommand("#0P--1000", testContainer);                                     // geen
  parser.parseCommand("#0P1000.2", testContainer);                                     // 21
  parser.parseCommand("#0P1000.2.2", testContainer);                                   // geen
  parser.parseCommand("#0P-1000.2", testContainer);                                    // 22
  parser.parseCommand("#0P--1000.2", testContainer);                                   // geen
  parser.parseCommand("#0P-1000.2.2", testContainer);                                  // geen

  BOOST_CHECK(testContainer.size() == 23);
}

BOOST_AUTO_TEST_CASE(createCommandContainerStop_1)
{
  commands::CommandParser parser;
  std::vector<commands::Command> testContainer;
  parser.parseCommand("STOP1", testContainer);
  BOOST_CHECK(testContainer.size() == 1);
  BOOST_CHECK(testContainer.at(0).getType() == commands::STOP);
  BOOST_CHECK(testContainer.at(0).getChannel() == 1);
  BOOST_CHECK(testContainer.at(0).getPwm() == 0);
  BOOST_CHECK(testContainer.at(0).getSpeed() == 0);
  BOOST_CHECK(testContainer.at(0).getTime() == 0);
}

BOOST_AUTO_TEST_CASE(createCommandContainerStop_2)
{
  commands::CommandParser parser;
  std::vector<commands::Command> testContainer;
  parser.parseCommand("STOP1000", testContainer);
  BOOST_CHECK(testContainer.size() == 1);
  BOOST_CHECK(testContainer.at(0).getType() == commands::STOP);
  BOOST_CHECK(testContainer.at(0).getChannel() == 1000);
  BOOST_CHECK(testContainer.at(0).getPwm() == 0);
  BOOST_CHECK(testContainer.at(0).getSpeed() == 0);
  BOOST_CHECK(testContainer.at(0).getTime() == 0);
}

BOOST_AUTO_TEST_CASE(createCommandContainerStop_3)
{
  commands::CommandParser parser;
  std::vector<commands::Command> testContainer;
  parser.parseCommand("STOP1000T500", testContainer);
  BOOST_CHECK(testContainer.size() == 0);
}

BOOST_AUTO_TEST_CASE(createCommandContainerStop_4)
{
  commands::CommandParser parser;
  std::vector<commands::Command> testContainer;
  parser.parseCommand(" STOP1a", testContainer);
  BOOST_CHECK(testContainer.size() == 0);
}

BOOST_AUTO_TEST_CASE(createCommandContainerStop_5)
{
  commands::CommandParser parser;
  std::vector<commands::Command> testContainer;
  parser.parseCommand("STO1", testContainer);
  BOOST_CHECK(testContainer.size() == 0);
}

BOOST_AUTO_TEST_CASE(createCommandContainerStop_6)
{
  commands::CommandParser parser;
  std::vector<commands::Command> testContainer;
  parser.parseCommand("geenSTOPCommando", testContainer);
  BOOST_CHECK(testContainer.size() == 0);
}

BOOST_AUTO_TEST_CASE(createCommandContainerStop_containerSize)
{
  commands::CommandParser parser;
  std::string testStringA = "STOP1";
  std::string testStringB = "STOP1000";
  std::string testStringB2 = "STOP1000T500";
  std::string testStringC = " STOP1a";
  std::string testStringD = "STO1";
  std::string testStringE = "geenSTOPCommando";
  std::vector<commands::Command> testContainer;

  parser.parseCommand(testStringA, testContainer);   // 0
  parser.parseCommand(testStringB, testContainer);   // 1
  parser.parseCommand(testStringB2, testContainer);  // geen
  parser.parseCommand(testStringC, testContainer);   // geen
  parser.parseCommand(testStringD, testContainer);   // geen
  parser.parseCommand(testStringE, testContainer);   // geen

  BOOST_CHECK(testContainer.size() == 2);
}

BOOST_AUTO_TEST_CASE(Command_test_copy_assign)
{
  commands::Command a(commands::MOVE, 100, 100, 100, 100);
  commands::Command b(commands::MOVE, 100, 100, 100, 100);
  commands::Command c = b;
  commands::Command d(b);
  // b
  BOOST_CHECK(a.getType() == b.getType());
  BOOST_CHECK(a.getChannel() == b.getChannel());
  BOOST_CHECK(a.getPwm() == b.getPwm());
  BOOST_CHECK(a.getSpeed() == b.getSpeed());
  BOOST_CHECK(a.getTime() == b.getTime());
  // c
  BOOST_CHECK(a.getType() == c.getType());
  BOOST_CHECK(a.getChannel() == c.getChannel());
  BOOST_CHECK(a.getPwm() == c.getPwm());
  BOOST_CHECK(a.getSpeed() == c.getSpeed());
  BOOST_CHECK(a.getTime() == c.getTime());
  // d
  BOOST_CHECK(a.getType() == d.getType());
  BOOST_CHECK(a.getChannel() == d.getChannel());
  BOOST_CHECK(a.getPwm() == d.getPwm());
  BOOST_CHECK(a.getSpeed() == d.getSpeed());
  BOOST_CHECK(a.getTime() == d.getTime());
}

BOOST_AUTO_TEST_CASE(parse_test)
{
  commands::CommandParser parser;
  std::vector<commands::Command> testContainer = {};
  parser.parseCommand("#2P-1500S10T2000\r#2P1500.5T2000", testContainer);
  BOOST_CHECK(testContainer.at(0).getType() == commands::MOVE);
  BOOST_CHECK(testContainer.at(0).getChannel() == 2);
  BOOST_CHECK(testContainer.at(0).getPwm() == -1500);
  BOOST_CHECK(testContainer.at(0).getSpeed() == 10);
  BOOST_CHECK(testContainer.at(0).getTime() == 2000);
  BOOST_CHECK(testContainer.at(1).getType() == commands::MOVE);
  BOOST_CHECK(testContainer.at(1).getChannel() == 2);
  BOOST_CHECK(testContainer.at(1).getPwm() == 1500.5);
  BOOST_CHECK(testContainer.at(1).getSpeed() == 0);
  BOOST_CHECK(testContainer.at(1).getTime() == 2000);
  BOOST_CHECK(testContainer.size() == 2);
}

BOOST_AUTO_TEST_CASE(parse_test_noBackslash)
{
  commands::CommandParser parser;
  std::vector<commands::Command> testContainer = {};
  parser.parseCommand("STOP1STOP2STOP3", testContainer);
  BOOST_CHECK(testContainer.size() == 0);
}

BOOST_AUTO_TEST_CASE(parse_test_withBackslash)
{
  commands::CommandParser parser;
  std::vector<commands::Command> testContainer = {};
  parser.parseCommand("STOP1\rSTOP2\rSTOP3\r", testContainer);
  BOOST_CHECK(testContainer.at(0).getType() == commands::STOP);
  BOOST_CHECK(testContainer.at(1).getType() == commands::STOP);
  BOOST_CHECK(testContainer.at(2).getType() == commands::STOP);
  BOOST_CHECK(testContainer.size() == 3);
}

BOOST_AUTO_TEST_CASE(parse_test_withSpace)
{
  commands::CommandParser parser;
  std::vector<commands::Command> testContainer = {};
  parser.parseCommand("STOP1\r STOP2\rSTOP3 \r", testContainer);
  BOOST_CHECK(testContainer.at(0).getType() == commands::STOP);
  BOOST_CHECK(testContainer.at(1).getType() == commands::STOP);
  BOOST_CHECK(testContainer.at(2).getType() == commands::STOP);
  BOOST_CHECK(testContainer.size() == 3);
}

BOOST_AUTO_TEST_CASE(parse_test_stopMoveStopMove)
{
  commands::CommandParser parser;
  std::vector<commands::Command> testContainer = {};
  parser.parseCommand("STOP1\r #100P200\r STOP2\r #200P300", testContainer);
  BOOST_CHECK(testContainer.at(0).getType() == commands::STOP);
  BOOST_CHECK(testContainer.at(1).getType() == commands::MOVE);
  BOOST_CHECK(testContainer.at(2).getType() == commands::STOP);
  BOOST_CHECK(testContainer.at(1).getType() == commands::MOVE);
  BOOST_CHECK(testContainer.size() == 4);
}

BOOST_AUTO_TEST_SUITE_END()
