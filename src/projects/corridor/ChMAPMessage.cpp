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

#include <iostream>
#include <fstream>

#include "ChMAPMessage.h"

ChMAPMessage::ChMAPMessage() {

}

bool ChMAPMessage::ParseFromFile(std::string filename) {
    rapidjson::Document document;

    std::ifstream mapinfo(filename);
    rapidjson::IStreamWrapper wrapinfo(mapinfo);
    document.ParseStream(wrapinfo);

    assert(document.IsObject());

    auto mapDataIt = document.FindMember("mapData");
    auto& mapData  = mapDataIt->value;

    if (mapDataIt == document.MemberEnd())
        return false;

    auto minuteIt  = mapData.FindMember("minuteOfTheYear");
    auto typeIt    = mapData.FindMember("layerType");

    if (minuteIt  == mapData.MemberEnd()
    ||  typeIt    == mapData.MemberEnd())
        return false;

    minuteOfTheYear = minuteIt->value.GetInt();
    layerType       = typeIt->value.GetString();

    auto geometryIt = mapData.FindMember("intersectionGeometry");
    if (geometryIt == mapData.MemberEnd())
        return false;

    if (!geometry.ParseFromObject(geometryIt->value))
        return false;

    return true;
}

bool ChMAPMessage::intersectionGeometry::ParseFromObject(rapidjson::Value& object) {
    auto refIt = object.FindMember("referencePoint");
    auto verIt = object.FindMember("verifiedPoint");
    auto listIt = object.FindMember("laneList");

    if (refIt  == object.MemberEnd()
    ||  verIt  == object.MemberEnd()
    ||  listIt == object.MemberEnd())
        return false;

    if (!refPoint.ParseFromObject(refIt->value))
        return false;
    if (!verPoint.ParseFromObject(verIt->value))
        return false;

    auto& list = listIt->value;
    auto approachIt = list.FindMember("approach");

    if (approachIt == object.MemberEnd())
        return false;

    auto& approaches = approachIt->value;
    assert(approaches.IsArray());

    size_t s = approaches.Size();
    laneList.resize(s);
    auto it = approaches.Begin();
    for (approach& ap : laneList)
        ap.ParseFromObject(*(it++));

    return true;
}

bool ChMAPMessage::intersectionGeometry::referencePoint::ParseFromObject(rapidjson::Value& object) {
    auto nameIt  = object.FindMember("descriptiveIntersctionName");
    auto IDIt    = object.FindMember("intersectionID");
    auto countIt = object.FindMember("msgCount");
    auto widthIt = object.FindMember("masterLaneWidth");
    auto latIt   = object.FindMember("referenceLat");
    auto lonIt   = object.FindMember("referenceLon");
    auto elevIt  = object.FindMember("referenceElevation");

    if (nameIt  == object.MemberEnd()
    ||  IDIt    == object.MemberEnd()
    ||  countIt == object.MemberEnd()
    ||  widthIt == object.MemberEnd()
    ||  latIt   == object.MemberEnd()
    ||  lonIt   == object.MemberEnd()
    ||  elevIt  == object.MemberEnd())
        return false;

    descriptiveIntersctionName = nameIt->value.GetString();
    intersectionID             = IDIt->value.GetString();
    msgCount                   = countIt->value.GetString();
    masterLaneWidth            = widthIt->value.GetString();
    referenceLat               = latIt->value.GetDouble();
    referenceLon               = lonIt->value.GetDouble();
    referenceElevation         = elevIt->value.GetString();

    return true;
}

bool ChMAPMessage::intersectionGeometry::verifiedPoint::ParseFromObject(rapidjson::Value& object) {
    auto latIt   = object.FindMember("verifiedMapLat");
    auto lonIt   = object.FindMember("verifiedMapLon");
    auto elevIt  = object.FindMember("verifiedMapElevation");
    auto sLatIt  = object.FindMember("verifiedSurveyedLat");
    auto sLonIt  = object.FindMember("verifiedSurveyedLon");
    auto sElevIt = object.FindMember("verifiedSurveyedElevation");

    if (latIt   == object.MemberEnd()
    ||  lonIt   == object.MemberEnd()
    ||  elevIt  == object.MemberEnd()
    ||  sLatIt  == object.MemberEnd()
    ||  sLonIt  == object.MemberEnd()
    ||  sElevIt == object.MemberEnd())
        return false;

    verifiedMapLat            = latIt->value.GetDouble();
    verifiedMapLon            = lonIt->value.GetDouble();
    verifiedMapElevation      = elevIt->value.GetString();
    verifiedSurveyedLat       = sLatIt->value.GetString();
    verifiedSurveyedLon       = sLonIt->value.GetString();
    verifiedSurveyedElevation = sElevIt->value.GetString();

    return true;
}

bool ChMAPMessage::intersectionGeometry::approach::ParseFromObject(rapidjson::Value& object) {
     auto typeIt = object.FindMember("approachType");
     auto IDIt   = object.FindMember("approachID");

     if (typeIt == object.MemberEnd()
     ||  IDIt   == object.MemberEnd())
         return false;

    approachType = typeIt->value.GetString();
    approachID   = IDIt->value.GetString();

    std::string laneField;
    if (approachType == "Ingress" || approachType == "Egress")
        laneField = "drivingLanes";
    else if (approachType == "Crosswalk")
        laneField = "crosswalkLanes";

    auto laneIt = object.FindMember(laneField.c_str());

    if (laneIt == object.MemberEnd())
        return false;

    auto& jLanes = laneIt->value;
    assert(jLanes.IsArray());

    lanes.resize(jLanes.Size());
    auto it = jLanes.Begin();
    for (lane& l : lanes)
        l.ParseFromObject(*(it++));

    return true;
}

bool ChMAPMessage::intersectionGeometry::approach::lane::ParseFromObject(rapidjson::Value& object) {
    auto IDIt     = object.FindMember("laneID");
    auto nameIt   = object.FindMember("descriptiveName");
    auto typeIt   = object.FindMember("laneType");
    auto attIt    = object.FindMember("typeAttributes");
    auto sharedIt = object.FindMember("sharedWith");
    auto connIt   = object.FindMember("connections");
    auto manIt    = object.FindMember("laneManeuvers");
    auto nodeIt   = object.FindMember("laneNodes");

    if (IDIt       == object.MemberEnd()
    ||  nameIt     == object.MemberEnd()
    ||  typeIt     == object.MemberEnd()
    ||  attIt      == object.MemberEnd()
    ||  sharedIt   == object.MemberEnd()
    ||  connIt     == object.MemberEnd()
    ||  manIt      == object.MemberEnd()
    ||  nodeIt     == object.MemberEnd())
        return false;

    laneID = IDIt->value.GetString();
    descriptiveName = nameIt->value.GetString();
    laneType = typeIt->value.GetString();
    // typeAttributes = attIt->value.GetString();
    // sharedWith = sharedIt->value.GetString();

    auto& conns = connIt->value;
    auto& mans  = manIt->value;
    auto& nodes = nodeIt->value;
    assert(conns.IsArray() && mans.IsArray() && nodes.IsArray());

    connections.resize(conns.Size());
    laneManeuvers.resize(mans.Size());
    laneNodes.resize(nodes.Size());

    auto cIt = conns.Begin();
    for (connection& c : connections)
        c.ParseFromObject(*(cIt++));

    auto mIt = mans.Begin();
    for (int& m : laneManeuvers)
        m = (mIt++)->GetInt();

    auto nIt = nodes.Begin();
    for (laneNode& n : laneNodes)
        n.ParseFromObject(*(nIt++));

    return true;
}

bool ChMAPMessage::intersectionGeometry::approach::lane::connection::ParseFromObject(rapidjson::Value& object) {
    auto IDIt   = object.FindMember("remoteID");
    auto fromIt = object.FindMember("fromLane");
    auto toIt   = object.FindMember("toLane");
    auto sigIt  = object.FindMember("signal_id");
    auto manIt  = object.FindMember("maneuvers");

    if (IDIt   == object.MemberEnd()
    ||  fromIt == object.MemberEnd()
    ||  toIt   == object.MemberEnd()
    ||  sigIt  == object.MemberEnd()
    ||  manIt  == object.MemberEnd())
        return false;

    remoteID = IDIt->value.GetString();
    fromLane = fromIt->value.GetString();
    toLane = toIt->value.GetString();
    signal_id = sigIt->value.GetString();
    auto& mans = manIt->value;

    assert(mans.IsArray());
    maneuvers.resize(mans.Size());

    auto it = mans.Begin();
    for (std::string& m : maneuvers)
        m = (it++)->GetString();

    return true;
}

bool ChMAPMessage::intersectionGeometry::approach::lane::laneNode::ParseFromObject(rapidjson::Value& object) {
    auto numIt = object.FindMember("nodeNumber");
    auto latIt = object.FindMember("nodeLat");
    auto lonIt = object.FindMember("nodeLong");
    auto eleIt = object.FindMember("nodeElev");
    auto widIt = object.FindMember("laneWidthDelta");

    if (numIt == object.MemberEnd()
    ||  latIt == object.MemberEnd()
    ||  lonIt == object.MemberEnd()
    ||  eleIt == object.MemberEnd()
    ||  widIt == object.MemberEnd())
        return false;

    nodeNumber     = numIt->value.GetInt();
    nodeLat        = latIt->value.GetDouble();
    nodeLong       = lonIt->value.GetDouble();

    if (eleIt->value.IsString()) {
        nodeElev       = atoi(eleIt->value.GetString());
        laneWidthDelta = atoi(widIt->value.GetString());
        return true;
    }

    nodeElev       = eleIt->value.GetDouble();
    laneWidthDelta = widIt->value.GetDouble();

    return true;
}
