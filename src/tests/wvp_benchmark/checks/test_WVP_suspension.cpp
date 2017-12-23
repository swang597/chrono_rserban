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
// Check for the WVP suspension forces
//
// The vehicle reference frame has Z up, X towards the front of the vehicle, and
// Y pointing to the left.
// All units SI.
//
// =============================================================================

#include "chrono/core/ChFileutils.h"
#include "chrono/utils/ChUtilsInputOutput.h"
#include "chrono_vehicle/ChConfigVehicle.h"
#include "chrono_vehicle/ChVehicleModelData.h"

#include "chrono_models/vehicle/wvp/WVP.h"
#include "chrono_models/vehicle/wvp/WVP_Vehicle.h"

#include <math.h>
#include <chrono>

using namespace chrono;
using namespace chrono::vehicle;
using namespace chrono::vehicle::wvp;

// =============================================================================

// output directory
const std::string out_dir = "../WVP_CHECKS";
bool data_output = true;

// =============================================================================

void CheckSuspension(WVP_Vehicle& vehicle, int axle, utils::CSV_writer& csv) {
    // Suspension subsystem
    auto susp = std::static_pointer_cast<WVP_DoubleWishboneFront>(vehicle.GetSuspension(axle));

    // Force elements (of type ChLinkSpringCB)
    auto spring_L = susp->GetSpring(VehicleSide::LEFT);
    auto spring_R = susp->GetSpring(VehicleSide::RIGHT);
    auto shock_L = susp->GetShock(VehicleSide::LEFT);
    auto shock_R = susp->GetShock(VehicleSide::RIGHT);

    // Rest length of the spring
    auto restLength = susp->getSpringRestLength();

    // Force calculation functor classes
    auto springForce = static_cast<WVP_SpringForce*>(susp->getSpringForceFunctor());
    auto shockForce = static_cast<WVP_ShockForce*>(susp->getShockForceFunctor());

    // Use the evaluate() methods of WVP_SpringForce and WVP_ShockForce
    // to obtain the forces generated at different configurations:
    //     WVP_SpringForce::evaluate(displ)
    //     WVP_ShockForce::evaluate(displ, vel, displ_other, vel_other)

    //// TODO
    //springForce->evaluate(displ_mine);
    //shockForce->evaluate(displ_mine, vel_mine, displ_other, vel_other);


	//single spring curve check
	csv << "Displacement" << "SpringForce"<<std::endl;
	for (int i = -1000; i < 1000; i++){
		double displ_mine = i*.001;
		csv << displ_mine;
		csv << springForce->evaluate(displ_mine) << std::endl;
	}


	//single wheel damping curve check
	double displ_other = 0;
	double vel_other = 0;
	csv << "displ_mine" << "vel_mine" << "displ_other" << "vel_other" << "shockForce" << std::endl;
	for (int i = -100; i < 100; i++) {
		for (int j = -300; j < 300; j++) {
			double displ_mine = i*0.005;
			double vel_mine = j*.01;
			csv << displ_mine << vel_mine << displ_other << vel_other;
			csv << shockForce->evaluate(displ_mine, vel_mine, displ_other, vel_other) << std::endl;
		}
	}






	//parallel wheel damping curve check
	/*double displ_other = 0;
	double vel_other = 0;
	csv << "displ_mine" << "vel_mine" << "displ_other" << "vel_other" << "shockForce" << std::endl;
	for (int i = -100; i < 100; i++) {
		for (int j = -300; j < 300; j++) {
			double displ_mine = i*0.005;
			double vel_mine = j*.01;
			double displ_other = displ_mine;
			double vel_other = vel_mine;
			csv << displ_mine << vel_mine << displ_other << vel_other;
			csv << shockForce->evaluate(displ_mine, vel_mine, displ_other, vel_other) << std::endl;
		}
	}*/



	//opposite wheel curve check (includes roll stabilization)







}

// =============================================================================

int main(int argc, char* argv[]) {
    WVP_Vehicle vehicle;

    if (data_output) {
        if (ChFileutils::MakeDirectory(out_dir.c_str()) < 0) {
            std::cout << "Error creating directory " << out_dir << std::endl;
            return 1;
        }
    }

    utils::CSV_writer csv("\t");
    csv.stream().setf(std::ios::scientific | std::ios::showpos);
    csv.stream().precision(6);

    // Checks for front suspension
    CheckSuspension(vehicle, 0, csv);

    csv.write_to_file(out_dir + "/suspension_check.dat");
	std::cout << "Written data to file\n.";

    return 0;
}
