#ifndef GONOGO_OPTIONS_H
#define GONOGO_OPTIONS_H

#include <iostream>
#include <string>

#include "chrono_thirdparty/SimpleOpt/SimpleOpt.h"

// ID values to identify command line arguments
enum { OPT_HELP, OPT_SLOPE, OPT_RADIUS, OPT_DENSITY, OPT_FRICTION, OPT_COHESION, OPT_NO_RENDERING };

// Table of CSimpleOpt::Soption structures. Each entry specifies:
// - the ID for the option (returned from OptionId() during processing)
// - the option as it should appear on the command line
// - type of the option
// The last entry must be SO_END_OF_OPTIONS
CSimpleOptA::SOption g_options[] = {{OPT_SLOPE, "-s", SO_REQ_CMB},    {OPT_RADIUS, "-r", SO_REQ_CMB},
                                    {OPT_DENSITY, "-d", SO_REQ_CMB},  {OPT_FRICTION, "-f", SO_REQ_CMB},
                                    {OPT_COHESION, "-c", SO_REQ_CMB}, {OPT_NO_RENDERING, "--no-rendering", SO_NONE},
                                    {OPT_HELP, "-?", SO_NONE},        {OPT_HELP, "-h", SO_NONE},
                                    {OPT_HELP, "--help", SO_NONE},    SO_END_OF_OPTIONS};

static inline void ShowUsage(const std::string& name) {
    std::cout << "Usage: " << name << " [OPTIONS]" << std::endl;
    std::cout << " -s=SLOPE_INDEX" << std::endl;
    std::cout << "        Specify index of slope [0, 10, 20, 30] (degrees)" << std::endl;
    std::cout << " -r=RADIUS_INDEX" << std::endl;
    std::cout << "        Specify index of radius [5, 10, 20, 30] (mm)" << std::endl;
    std::cout << " -d=DENSITY_INDEX" << std::endl;
    std::cout << "        Specify index of density [1000, 2000, 3000] (kg/m3)" << std::endl;
    std::cout << " -f=FRICTION_INDEX" << std::endl;
    std::cout << "        Specify index of coef. of friction [0.2, 0.4, 0.6, 0.8, 1.0]" << std::endl;
    std::cout << " -c=COHESION_INDEX" << std::endl;
    std::cout << "        Specify index of cohesion value [50, 100, 150, 200]" << std::endl;
    std::cout << " --no-rendering" << std::endl;
    std::cout << "        Disable OpenGL rendering" << std::endl;
    std::cout << " -? -h --help" << std::endl;
    std::cout << "        Print this message and exit." << std::endl;
    std::cout << std::endl;
}

static inline bool GetProblemSpecs(int argc,
                                   char** argv,
                                   int& slope,
                                   int& radius,
                                   int& density,
                                   int& friction,
                                   int& cohesion,
                                   bool& render) {
    // Create the option parser and pass it the program arguments and the array of valid options.
    CSimpleOptA args(argc, argv, g_options);

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
            case OPT_SLOPE:
                slope = std::stoi(args.OptionArg());
                break;
            case OPT_RADIUS:
                radius = std::stoi(args.OptionArg());
                break;
            case OPT_DENSITY:
                density = std::stoi(args.OptionArg());
                break;
            case OPT_FRICTION:
                friction = std::stoi(args.OptionArg());
                break;
            case OPT_COHESION:
                cohesion = std::stoi(args.OptionArg());
                break;
            case OPT_NO_RENDERING:
                render = false;
                break;
        }
    }

    return true;
}

#endif
