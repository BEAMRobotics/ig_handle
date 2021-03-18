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

#include <example_package/example_class/example_class.h>

#include <iostream>
#include <string>

#include <boost/log/trivial.hpp>

#include <beam_utils/log.h>

// Constructor starts off the object with cycler set to 1.
ExampleClass::ExampleClass() {
    this->private_cycler = 1;
    BEAM_INFO("Constructed Example Class.");
}

// INTENDED FUNCTIONALITY
// Some function that  provides cycling output 1, 2, 1, 2, 1, 2, ...
// ACTUAL FUNCTIONALITY (BUG)
// Some function that  provides cycling output 1, 2, 3, 1, 2, 3, 1, 2, ...
int ExampleClass::Foo() {
    int return_value = this->private_cycler;
    this->private_cycler++;

    if (this->private_cycler > 2) {  
        this->private_cycler = 1;
    }

    // Demonstration of extreme log verbosity.
    // Just for demonstration to show log filtering.
    // Please avoid logging with this level granularity.
    BOOST_LOG_TRIVIAL(trace) << "ExampleClass::Foo() complete. Returning: "
                             << std::to_string(return_value);

    return return_value;
}

// Some function that only accepts an input of 1 or 2
//
// Returns: 0 - Success
//          1 - Failure
int ExampleClass::Bar(int input) {
    // Check that the input value satisfies the correct bounds (1 or 2)
    //
    // NOTE: If your function only accepts a subset of parameter values, CHECK
    // THEM. For this example, the compiler only knows that input has to be an
    // int. It doesn't know about the 1 or 2 restriction. You need to check
    // this yourself!
    if (!(input == 1) && !(input == 2)) {
        // Bounds not satisfied!

        // Create an error message
        std::string error_string =
          "ExampleClass::Bar() - input out of bounds. Received: " +
          std::to_string(input);

        // Output the error message
        BOOST_LOG_TRIVIAL(error) << error_string;
        return 1;  // Failure
    } else {
        // All is well

        // Do some work

        return 0;  // Success
    }
}
