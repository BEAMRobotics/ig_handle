/* Copyright (c) 2019, BEAM Robotics,
 *
 * Refer to the accompanying LICENSE file for license information.
 *
 * ############################################################################
 ******************************************************************************
 |     _ _ __ _ _      _ _ _ _ _ _ _    _ _ _ _ _ _       _ _         _ _     |
 |   |     _ _   \    |     _ _ _ _|   /            \    /    \      /    \   |
 |   |    |   \   \   |    |           |     _ _     |   |     \_ _ /     |   |
 |   |    |   |   |   |    |           |    |   |    |   |                |   |
 |   |    |   /   /   |    |_ _ _ _    |    |_ _|    |   |                |   |
 |   |    |  /   /    |            |   |             |   |    /\ _ /\     |   |
 |   |    |  \   \    |     _ _ _ _|   |     _ _     |   |    |      |    |   |
 |   |    |   \   \   |    |           |    |   |    |   |    |      |    |   |
 |   |    |   |   |   |    |           |    |   |    |   |    |      |    |   |
 |   |    |_ _/   /   |    |_ _ _ _    |    |   |    |   |    |      |    |   |
 |   |_ _ _ _ _ _/    |_ _ _ _ _ _ |   | _ _|   |_ _ |   | _ _|      | _ _|   |
 |  _________________________________________________________________________ |
 | |____________________________________________________________________/___/ |
 |   |    _ _     _ _     _ _     _ _   _ _ _   _ _      _ _     _ _     ||   |
 |   |   |_ _|   /   \   |_ _|   /   \    |      |     /    `  /_ _ `    ||   |
 |  _|__ |  \ __ \_ _/ _ |_ _| _ \_ _/  _ |   _ _|_  _ \ _ _.  ._ _/ ____||_  |
 | |___________________________________________________________________/___/  |                                                                           |
 ******************************************************************************
 * ############################################################################
 *
 * File: example_package_node.cpp
 * Desc: Code that allows the example_package_nodelet to be run as a node for
 *       convenience.
 * Auth: Nick Charron
 *
 * ############################################################################
*/

#include <gtest/gtest.h>
#include <stdio.h>
#include "example_package/example_class/example_class.hpp"

// Test the ExampleClass::Bar(int input) function.
TEST(ExampleClassTest, BarTest) {
    ExampleClass foobar_test;

    // Bar() should return 0 for inputs of 1 or 2 and 1 otherwise
    EXPECT_EQ(foobar_test.Bar(0), 1);
    EXPECT_EQ(foobar_test.Bar(1), 0);
    EXPECT_EQ(foobar_test.Bar(2), 0);
    EXPECT_EQ(foobar_test.Bar(3), 1);

    EXPECT_EQ(foobar_test.Bar(-1000), 1);
    EXPECT_EQ(foobar_test.Bar(1000), 1);
}

// Test the ExampleClass::Foo() function.
TEST(ExampleClassTest, FooTest) {
    ExampleClass foobar_test;

    // Test the function a whole bunch of times to make sure it stays within
    // bounds.
    int NUM_TESTS = 2;  // <- Too low to discover bug. Set to 3 to find the bug.
    for (int i_trial = 0; i_trial < NUM_TESTS; i_trial++) {
        int result = foobar_test.Foo();
        EXPECT_TRUE((result == 1) || (result == 2));
    }
}
