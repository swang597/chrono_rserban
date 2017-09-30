// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================
//
// =============================================================================

#include <string>
#include <iostream>

#include "chrono_thirdparty/SimpleOpt/SimpleOpt.h"

// ID values to identify command line arguments
enum { OPT_HELP, OPT_FILE, OPT_LINE, OPT_THREADS, OPT_NO_RENDERING, OPT_NO_COPY };

// Table of CSimpleOpt::Soption structures. Each entry specifies:
// - the ID for the option (returned from OptionId() during processing)
// - the option as it should appear on the command line
// - type of the option
// The last entry must be SO_END_OF_OPTIONS
CSimpleOptA::SOption g_options[] = {{OPT_FILE, "-f", SO_REQ_CMB},
                                    {OPT_LINE, "-l", SO_REQ_CMB},
                                    {OPT_THREADS, "-t", SO_REQ_CMB},
                                    {OPT_NO_RENDERING, "--no-rendering", SO_NONE},
                                    {OPT_NO_COPY, "--no-copy", SO_NONE},
                                    {OPT_HELP, "-?", SO_NONE},
                                    {OPT_HELP, "-h", SO_NONE},
                                    {OPT_HELP, "--help", SO_NONE},
                                    SO_END_OF_OPTIONS};

void ShowUsage(const std::string& name) {
    std::cout << "Usage: " << name << " -f=FILE_NAME -l=LINE -t=THREADS [OPTIONS]" << std::endl;
    std::cout << " -f=FILE_NAME" << std::endl;
    std::cout << "        Name of input file" << std::endl;
    std::cout << "        Each line contains a point in parameter space:" << std::endl;
    std::cout << "        slope (deg), radius (mm), density (kg/m3), coef. friction, cohesion" << std::endl;
    std::cout << " -l=LINE" << std::endl;
    std::cout << "        Line in input file" << std::endl;
    std::cout << " -t=THREADS" << std::endl;
    std::cout << "        Number of OpenMP threads" << std::endl;
    std::cout << " --no-rendering" << std::endl;
    std::cout << "        Disable OpenGL rendering" << std::endl;
    std::cout << " --no-copy" << std::endl;
    std::cout << "        Disable copying of input file to output directory" << std::endl;
    std::cout << " -? -h --help" << std::endl;
    std::cout << "        Print this message and exit." << std::endl;
    std::cout << std::endl;
}

bool GetProblemSpecs(int argc, char** argv, std::string& file, int& line, int& threads, bool& render, bool& copy) {
    // Create the option parser and pass it the program arguments and the array of valid options.
    CSimpleOptA args(argc, argv, g_options);

    render = true;
    copy = true;

    // Then loop for as long as there are arguments to be processed.
    while (args.Next()) {
        // Exit immediately if we encounter an invalid argument.
        if (args.LastError() != SO_SUCCESS) {
            std::cout << "Invalid argument: " << args.OptionText() << std::endl;
            ShowUsage(argv[0]);
            return false;
        }

        // Process the current argument.
        switch (args.OptionId()) {
            case OPT_HELP:
                ShowUsage(argv[0]);
                return false;
            case OPT_FILE:
                file = args.OptionArg();
                break;
            case OPT_LINE:
                line = std::stoi(args.OptionArg());
                break;
            case OPT_THREADS:
                threads = std::stoi(args.OptionArg());
                break;
            case OPT_NO_RENDERING:
                render = false;
                break;
            case OPT_NO_COPY:
                copy = false;
                break;
        }
    }

    return true;
}
