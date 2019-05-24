// // https://stackoverflow.com/questions/13547771/g-project-compilation-with-boost-unit-test
// #define BOOST_TEST_DYN_LINK                        // this is optional
// #define BOOST_TEST_MODULE al5d_CommandParser_test  // specify the name of your test module

// #include <boost/test/included/unit_test.hpp>  // include this to get main()

// #include <sim_robot/JointController.hpp>

// #define TESTJOINT_MIN_PW 500
// #define TESTJOINT_MAX_PW 2500
// #define TESTJOINT_MIN_RAD -M_PI_2
// #define TESTJOINT_MAX_RAD M_PI_2
// #define TESTJOINT_MAX_VEL M_PI

// BOOST_AUTO_TEST_SUITE(JointController)

// BOOST_AUTO_TEST_CASE(test_inRange)
// {
//   gazebo::physics::JointPtr emptyJ;
//   gazebo::JointController testJoint(emptyJ, "testJoint", 0, TESTJOINT_MIN_PW, TESTJOINT_MAX_PW, TESTJOINT_MIN_RAD,
//                                     TESTJOINT_MAX_RAD, TESTJOINT_MAX_VEL);

//   BOOST_CHECK(testJoint.inRange(TESTJOINT_MIN_PW));
//   BOOST_CHECK(testJoint.inRange(TESTJOINT_MAX_PW));
//   BOOST_CHECK(testJoint.inRange(TESTJOINT_MIN_PW + ((TESTJOINT_MAX_PW - TESTJOINT_MIN_PW) / 2)));
//   BOOST_CHECK_EQUAL(false, testJoint.inRange(TESTJOINT_MIN_PW - 1));
//   BOOST_CHECK_EQUAL(false, testJoint.inRange(TESTJOINT_MAX_PW + 1));
// }

// BOOST_AUTO_TEST_CASE(test_move, *boost::unit_test::tolerance(0.00001))
// {
//   gazebo::physics::JointPtr emptyJ;
//   gazebo::JointController testJoint(emptyJ, "testJoint", 0, TESTJOINT_MIN_PW, TESTJOINT_MAX_PW, TESTJOINT_MIN_RAD,
//                                     TESTJOINT_MAX_RAD, TESTJOINT_MAX_VEL);

//   BOOST_CHECK(testJoint.move(TESTJOINT_MAX_PW, 1, 0, 1));
//   BOOST_CHECK_EQUAL(testJoint.getTargetPos(), TESTJOINT_MAX_RAD);

//   BOOST_CHECK(testJoint.move(TESTJOINT_MIN_PW, 1, 0, 1));
//   BOOST_CHECK_EQUAL(testJoint.getTargetPos(), TESTJOINT_MIN_RAD);

//   BOOST_CHECK_EQUAL(false, testJoint.move(TESTJOINT_MIN_PW - 1, 1, 0, 1));
// }

// BOOST_AUTO_TEST_CASE(test_update, *boost::unit_test::tolerance(0.00001))
// {
//   gazebo::physics::JointPtr emptyJ;
//   gazebo::JointController testJoint(emptyJ, "testJoint", 0, TESTJOINT_MIN_PW, TESTJOINT_MAX_PW, TESTJOINT_MIN_RAD,
//                                     TESTJOINT_MAX_RAD, TESTJOINT_MAX_VEL);

//   double rate = 50;

//   testJoint.setCurrentPos(0);
//   BOOST_CHECK_EQUAL(0, testJoint.getCurrentPos());

//   testJoint.move(TESTJOINT_MAX_PW, 1000, 0, rate);
//   BOOST_CHECK_NE(testJoint.getTargetPos(), testJoint.getCurrentPos());
//   BOOST_CHECK_EQUAL(testJoint.getCurrentVel(), M_PI_2);

//   for (size_t i = 0; i < static_cast<size_t>(rate) - 1; ++i)
//   {
//     testJoint.update();
//   }

//   BOOST_CHECK_NE(testJoint.getTargetPos(), testJoint.getCurrentPos());
//   testJoint.update();
//   BOOST_CHECK_EQUAL(testJoint.getTargetPos(), testJoint.getCurrentPos());

//   // Double speed
//   testJoint.setCurrentPos(0);
//   BOOST_CHECK_EQUAL(0, testJoint.getCurrentPos());

//   testJoint.move(TESTJOINT_MAX_PW, 2000, 0, rate);
//   BOOST_CHECK_NE(testJoint.getTargetPos(), testJoint.getCurrentPos());
//   BOOST_CHECK_EQUAL(testJoint.getCurrentVel(), M_PI);

//   for (size_t i = 0; i < static_cast<size_t>(rate / 2) - 1; ++i)
//   {
//     testJoint.update();
//   }

//   BOOST_CHECK_NE(testJoint.getTargetPos(), testJoint.getCurrentPos());
//   testJoint.update();
//   BOOST_CHECK_EQUAL(testJoint.getTargetPos(), testJoint.getCurrentPos());
// }

// BOOST_AUTO_TEST_CASE(test_move_max_velocity, *boost::unit_test::tolerance(0.00001))
// {
//   gazebo::physics::JointPtr emptyJ;
//   gazebo::JointController testJoint(emptyJ, "testJoint", 0, TESTJOINT_MIN_PW, TESTJOINT_MAX_PW, TESTJOINT_MIN_RAD,
//                                     TESTJOINT_MAX_RAD, TESTJOINT_MAX_VEL);

//   double rate = 50;

//   testJoint.setCurrentPos(0);
//   BOOST_CHECK_EQUAL(0, testJoint.getCurrentPos());
//   testJoint.move(TESTJOINT_MAX_PW, 1000, 0, rate);
//   BOOST_CHECK_EQUAL(testJoint.getCurrentVel(), M_PI_2);

//   testJoint.setCurrentPos(0);
//   BOOST_CHECK_EQUAL(0, testJoint.getCurrentPos());
//   testJoint.move(TESTJOINT_MAX_PW, 2000, 0, rate);
//   BOOST_CHECK_EQUAL(testJoint.getCurrentVel(), M_PI);

//   testJoint.setCurrentPos(0);
//   BOOST_CHECK_EQUAL(0, testJoint.getCurrentPos());
//   testJoint.move(TESTJOINT_MAX_PW, 2001, 0, rate);
//   BOOST_CHECK_EQUAL(testJoint.getCurrentVel(), TESTJOINT_MAX_VEL);
// }

// BOOST_AUTO_TEST_CASE(test_speed_time)
// {
//   gazebo::physics::JointPtr emptyJ;
//   gazebo::JointController testJoint(emptyJ, "testJoint", 0, TESTJOINT_MIN_PW, TESTJOINT_MAX_PW, TESTJOINT_MIN_RAD,
//                                     TESTJOINT_MAX_RAD, TESTJOINT_MAX_VEL);

//   double rate = 50;

//   // move in 1 second
//   testJoint.setCurrentPos(0);
//   BOOST_CHECK_EQUAL(0, testJoint.getCurrentPos());
//   testJoint.move(TESTJOINT_MAX_PW, 0, 1000, rate);
//   BOOST_CHECK_EQUAL(testJoint.getCurrentVel(), M_PI_2);

//   // move in 2 seconds
//   testJoint.setCurrentPos(0);
//   BOOST_CHECK_EQUAL(0, testJoint.getCurrentPos());
//   testJoint.move(TESTJOINT_MAX_PW, 0, 2000, rate);
//   BOOST_CHECK_EQUAL(testJoint.getCurrentVel(), M_PI_4);

//   // move in 0.5 second
//   testJoint.setCurrentPos(0);
//   BOOST_CHECK_EQUAL(0, testJoint.getCurrentPos());
//   testJoint.move(TESTJOINT_MAX_PW, 0, 500, rate);
//   BOOST_CHECK_EQUAL(testJoint.getCurrentVel(), M_PI);

//   // move in 1 second (time), but speed move in 2 seconds!
//   testJoint.setCurrentPos(0);
//   BOOST_CHECK_EQUAL(0, testJoint.getCurrentPos());
//   testJoint.move(TESTJOINT_MAX_PW, 500, 1000, rate);
//   BOOST_CHECK_EQUAL(testJoint.getCurrentVel(), M_PI_2);

//   // move in 0.5 second (time), but speed move in 1 second...
//   testJoint.setCurrentPos(0);
//   BOOST_CHECK_EQUAL(0, testJoint.getCurrentPos());
//   testJoint.move(TESTJOINT_MAX_PW, 1000, 500, rate);
//   BOOST_CHECK_EQUAL(testJoint.getCurrentVel(), M_PI);
// }

// BOOST_AUTO_TEST_CASE(test_operators)
// {
//   gazebo::physics::JointPtr emptyJ;
//   gazebo::JointController testJoint(emptyJ, "testJoint", 0, TESTJOINT_MIN_PW, TESTJOINT_MAX_PW, TESTJOINT_MIN_RAD,
//                                     TESTJOINT_MAX_RAD, TESTJOINT_MAX_VEL);

//   gazebo::JointController testJoint2(testJoint);
//   BOOST_CHECK(testJoint == testJoint2);
//   gazebo::JointController testJoint3 = testJoint2;
//   BOOST_CHECK(testJoint == testJoint3);
//   BOOST_CHECK(testJoint2 == testJoint3);

//   testJoint.setCurrentPos(M_PI_2);
//   BOOST_CHECK(testJoint != testJoint2);
// }

// BOOST_AUTO_TEST_SUITE_END()
