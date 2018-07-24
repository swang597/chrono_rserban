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


#include "ChSensor.h"


ChSensor::ChSensor(std::shared_ptr<chrono::ChBody> parent, double updateRate, bool visualize){
	m_updateRate = updateRate;
	m_parent = parent;
	m_visualize = visualize;
}
ChSensor::~ChSensor(){
}
void ChSensor::SetNoiseType(noiseType n_type){
	m_noiseType = n_type;
}
