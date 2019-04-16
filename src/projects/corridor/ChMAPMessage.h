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
#include "framework.h"

class ChMAPMessage {
public:
    ChMAPMessage();
    std::string ParseFromFile(std::string filename);
    bool MakeLanePoints(std::vector<std::vector<av::GPScoord>>& lanePoints);

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

            std::string ParseFromObject(rapidjson::Value& object, std::string path);
        };

        class verifiedPoint {
        public:
            double verifiedMapLat;
            double verifiedMapLon;
            std::string verifiedMapElevation;
            std::string verifiedSurveyedLat;
            std::string verifiedSurveyedLon;
            std::string verifiedSurveyedElevation;

            std::string ParseFromObject(rapidjson::Value& object, std::string path);
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

                    std::string ParseFromObject(rapidjson::Value& object, std::string path, int k);
                };

                class laneNode {
                public:
                    int nodeNumber;
                    double nodeLat;
                    double nodeLong;
                    double nodeElev;
                    double laneWidthDelta;

                    std::string ParseFromObject(rapidjson::Value& object, std::string path, int k);
                };

                std::string laneID;
                std::string descriptiveName;
                std::string laneType;
                std::string typeAttributes; // Perhaps these two fields should be
                std::string sharedWith; //     Something other than string.
                std::vector<connection> connections;
                std::vector<int> laneManeuvers;
                std::vector<laneNode> laneNodes;

                std::string ParseFromObject(rapidjson::Value& object, std::string path, int j);
            };

            std::string ParseFromObject(rapidjson::Value& object, std::string path, int i);
            const std::vector<lane>& GetLanes();
            
        private:
            std::string approachType;
            std::string approachID;
            std::vector<lane> lanes;
        };

        std::string ParseFromObject(rapidjson::Value& object, std::string path);
        const std::vector<approach>& GetLaneList();

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
