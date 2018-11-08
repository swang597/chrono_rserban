// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Asher Elmquist
// =============================================================================
//
//
// =============================================================================


#ifndef CHSENSOR_H
#define CHSENSOR_H

#include "chrono/physics/ChBody.h"



class ChSensor
{
public: ChSensor(std::shared_ptr<chrono::ChBody> parent, double updateRate, bool visualize);
		~ChSensor();
		enum noiseType {NONE, GAUSSIAN};
		void SetNoiseType(noiseType n_type);


protected:
		noiseType m_noiseType = NONE;	//noise model
		double m_mean = 0;
		double m_stdev = 0;

		double m_updateRate = 0;
		double m_timeLastUpdated = 0;
		std::shared_ptr<chrono::ChBody> m_parent;
		bool m_visualize;

};

#endif
