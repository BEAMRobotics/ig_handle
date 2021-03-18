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

#pragma once

class ExampleClass {
 public:
    ExampleClass();

    // Example function that doesn't work properly for whatever reason.
    // Is supposed to only return either 1 or 2, but it actually
    // returns 1, 2, or 3!
    int Foo();

    // Example function that expects an input value of either 1 or 2
    //
    // Returns 0 for success and 1 for failure
    int Bar(int input);

 private:
    // A private cycling variable that should repeat 1, 2, 1, 2, ... in an
    // endless cycle
    int private_cycler;
};