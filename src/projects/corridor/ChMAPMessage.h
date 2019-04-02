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
// Authors: Dylan Hatch
// =============================================================================
//
// Base class for MAP messages.
//
// =============================================================================

#ifndef CH_MAPMESSAGE_H
#define CH_MAPMESSAGE_H

#include <string>
#include <list>
#include <vector>

#include "chrono_thirdparty/rapidjson/document.h"
#include "chrono_thirdparty/rapidjson/istreamwrapper.h"

class ChMAPMessage {
public:
    ChMAPMessage();
    bool ParseFromFile(std::string filename);

    class intersectionGeometry {
    public:
        class referencePoint {
        public:
            std::string descriptiveIntersctionName;
            std::string intersectionID;
            std::string msgCount;
            std::string masterLaneWidth;
            double referenceLat;
            double referenceLon;
            std::string referenceElevation;

            bool ParseFromObject(rapidjson::Value& object);
        };

        class verifiedPoint {
        public:
            double verifiedMapLat;
            double verifiedMapLon;
            std::string verifiedMapElevation;
            std::string verifiedSurveyedLat;
            std::string verifiedSurveyedLon;
            std::string verifiedSurveyedElevation;

            bool ParseFromObject(rapidjson::Value& object);
        };

        class approach {
        public:
            class lane {
            public:
                class connection {
                public:
                    std::string remoteID;
                    std::string fromLane;
                    std::string toLane;
                    std::string signal_id;
                    std::list<std::string> maneuvers;

                    bool ParseFromObject(rapidjson::Value& object);
                };

                class laneNode {
                public:
                    int nodeNumber;
                    double nodeLat;
                    double nodeLong;
                    double nodeElev;
                    double laneWidthDelta;

                    bool ParseFromObject(rapidjson::Value& object);
                };

                std::string laneID;
                std::string descriptiveName;
                std::string laneType;
                std::string typeAttributes; // Perhaps these two fields should be
                std::string sharedWith; //     Something other than string.
                std::vector<connection> connections;
                std::vector<int> laneManeuvers;
                std::vector<laneNode> laneNodes;

                bool ParseFromObject(rapidjson::Value& object);
            };

            bool ParseFromObject(rapidjson::Value& object);

        private:
            std::string approachType;
            std::string approachID;
            std::vector<lane> lanes;
        };

        bool ParseFromObject(rapidjson::Value& object);

    private:
        referencePoint refPoint;
        verifiedPoint verPoint;
        std::vector<approach> laneList;
    };

private:
    size_t minuteOfTheYear;
    std::string layerType;
    intersectionGeometry geometry;
};

#endif
