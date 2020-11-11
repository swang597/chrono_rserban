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

#include "chrono_thirdparty/cxxopts/ChCLI.h"

bool GetProblemSpecs(int argc, char** argv, std::string& file, int& line, int& threads, bool& render, bool& copy, bool&pov_output) {
    chrono::ChCLI cli(argv[0]);

    cli.AddOption<std::string>("Demo", "f,filename", "Name of input file");
    cli.AddOption<int>("Demo", "l,line", "Line in input file", "1");
    cli.AddOption<int>("Demo", "t,threads", "Number of OpenMP threads", "1");
    cli.AddOption<bool>("Demo", "render", "OpenGL rendering", "true");
    cli.AddOption<bool>("Demo", "copy", "Copy input file to output directory", "true");
    cli.AddOption<bool>("Demo", "pov", "Output for POV-Ray post-processing", "false");

    if (!cli.Parse(argc, argv)) {
        cli.Help();
        return false;
    }

    file = cli.GetAsType<std::string>("filename");
    line = cli.GetAsType<int>("line");
    threads = cli.GetAsType<int>("threads");
    render = cli.GetAsType<bool>("render");
    copy = cli.GetAsType<bool>("copy");
    pov_output = cli.GetAsType<bool>("pov");

    return true;
}
