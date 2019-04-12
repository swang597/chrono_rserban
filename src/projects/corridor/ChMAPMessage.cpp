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

std::string ChMAPMessage::ParseFromFile(std::string filename) {
    rapidjson::Document document;
    std::string path = "mapData";

    std::ifstream mapinfo(filename);
    rapidjson::IStreamWrapper wrapinfo(mapinfo);
    document.ParseStream(wrapinfo);

    assert(document.IsObject());

    auto mapDataIt = document.FindMember("mapData");
    auto& mapData  = mapDataIt->value;

    if (mapDataIt == document.MemberEnd())
        return path;

    auto minuteIt  = mapData.FindMember("minuteOfTheYear");
    auto typeIt    = mapData.FindMember("layerType");

    if (minuteIt  == mapData.MemberEnd())
        return path + "->minuteOfTheYear -- Field not found";
    if (typeIt    == mapData.MemberEnd())
        return path + "->layerType -- Field not found";

    if (!minuteIt->value.IsInt())
        return path + "->minuteOfTheYear -- Field not int";
    minuteOfTheYear = minuteIt->value.GetInt();
    if (!typeIt->value.IsString())
        return path + "->layerType -- Field not string";
    layerType       = typeIt->value.GetString();

    auto geometryIt = mapData.FindMember("intersectionGeometry");
    if (geometryIt == mapData.MemberEnd())
        return path + "->intersectionGeometry";

    return geometry.ParseFromObject(geometryIt->value, path);
}

std::string ChMAPMessage::intersectionGeometry::ParseFromObject(rapidjson::Value& object, std::string path) {
    path += "->intersectionGeometry";

    auto refIt = object.FindMember("referencePoint");
    auto verIt = object.FindMember("verifiedPoint");
    auto listIt = object.FindMember("laneList");

    if (refIt  == object.MemberEnd())
        return path + "->referencePoint -- Field not found";
    if (verIt  == object.MemberEnd())
        return path + "->verifiedPoint -- Field not found";
    if (listIt == object.MemberEnd())
        return path + "->laneList -- Field not found";

    std::string p;
    if ((p = refPoint.ParseFromObject(refIt->value, path)) != "")
        return p;
    if ((p = verPoint.ParseFromObject(verIt->value, path)) != "")
        return p;

    auto& list = listIt->value;
    auto approachIt = list.FindMember("approach");

    if (approachIt == object.MemberEnd())
        return path + "->approach -- Field not found";

    auto& approaches = approachIt->value;
    assert(approaches.IsArray());

    size_t s = approaches.Size();
    laneList.resize(s);
    auto it = approaches.Begin();
    p = "";
    int i = 0;
    for (approach& ap : laneList)
        if ((p = ap.ParseFromObject(*(it++), path, i++)) != "")
            return p;

    return "";
}

std::string ChMAPMessage::intersectionGeometry::referencePoint::ParseFromObject(rapidjson::Value& object, std::string path) {
    path += "->referencePoint";

    auto nameIt  = object.FindMember("descriptiveIntersctionName");
    auto IDIt    = object.FindMember("intersectionID");
    auto countIt = object.FindMember("msgCount");
    auto widthIt = object.FindMember("masterLaneWidth");
    auto latIt   = object.FindMember("referenceLat");
    auto lonIt   = object.FindMember("referenceLon");
    auto elevIt  = object.FindMember("referenceElevation");

    if (nameIt  == object.MemberEnd())
        return path + "->descriptiveIntersctionName -- Field not found";
    if (IDIt    == object.MemberEnd())
        return path + "->intersectionID -- Field not found";
    if (countIt == object.MemberEnd())
        return path + "->msgCount -- Field not found";
    if (widthIt == object.MemberEnd())
        return path + "->masterLaneWidth -- Field not found";
    if (latIt   == object.MemberEnd())
        return path + "->referenceLat -- Field not found";
    if (lonIt   == object.MemberEnd())
        return path + "->referenceLon -- Field not found";
    if (elevIt  == object.MemberEnd())
        return path + "->referenceElevation -- Field not found";

    if (!nameIt->value.IsString())
        return path + "->descriptiveIntersctionName -- Field not string";
    descriptiveIntersctionName = nameIt->value.GetString();
    if (!IDIt->value.IsString())
        return path + "->intersectionID -- Field not string";
    intersectionID             = IDIt->value.GetString();
    if (!countIt->value.IsString())
        return path + "->msgCount -- Field not string";
    msgCount                   = countIt->value.GetString();
    if (!widthIt->value.IsString())
        return path + "->masterLaneWidth -- Field not string";
    masterLaneWidth            = widthIt->value.GetString();
    if (!latIt->value.IsDouble())
        return path + "->referenceLat -- Field not double";
    referenceLat               = latIt->value.GetDouble();
    if (!lonIt->value.IsDouble())
        return path + "->referenceLon -- Field not double";
    referenceLon               = lonIt->value.GetDouble();
    if (!elevIt->value.IsString())
        return path + "->referenceElevation -- Field not string";
    referenceElevation         = elevIt->value.GetString();

    return "";
}

std::string ChMAPMessage::intersectionGeometry::verifiedPoint::ParseFromObject(rapidjson::Value& object, std::string path) {
    path += "->verifiedPoint";

    auto latIt   = object.FindMember("verifiedMapLat");
    auto lonIt   = object.FindMember("verifiedMapLon");
    auto elevIt  = object.FindMember("verifiedMapElevation");
    auto sLatIt  = object.FindMember("verifiedSurveyedLat");
    auto sLonIt  = object.FindMember("verifiedSurveyedLon");
    auto sElevIt = object.FindMember("verifiedSurveyedElevation");

    if (latIt   == object.MemberEnd())
        return path + "->verifiedMapLat -- Field not found";
    if (lonIt   == object.MemberEnd())
        return path + "->verifiedMapLon -- Field not found";
    if (elevIt  == object.MemberEnd())
        return path + "->verifiedMapElevation -- Field not found";
    if (sLatIt  == object.MemberEnd())
        return path + "->verifiedSurveyedLat -- Field not found";
    if (sLonIt  == object.MemberEnd())
        return path + "->verifiedSurveyedLon -- Field not found";
    if (sElevIt == object.MemberEnd())
        return path + "->verifiedSurveyedElevation -- Field not found";

    if (!latIt->value.IsDouble())
        return path + "->verifiedMapLat -- Field not double";
    verifiedMapLat            = latIt->value.GetDouble();
    if (!lonIt->value.IsDouble())
        return path + "->verifiedMapLon -- Field not double";
    verifiedMapLon            = lonIt->value.GetDouble();
    if (!elevIt->value.IsString())
        return path + "->verifiedMapElevation -- Field not string";
    verifiedMapElevation      = elevIt->value.GetString();
    if (!sLatIt->value.IsString())
        return path + "->verifiedSurveyedLat -- Field not string";
    verifiedSurveyedLat       = sLatIt->value.GetString();
    if (!sLonIt->value.IsString())
        return path + "->verifiedSurveyedLon -- Field not string";
    verifiedSurveyedLon       = sLonIt->value.GetString();
    if (!sElevIt->value.IsString())
        return path + "->verifiedSurveyedElevation -- Field not string";
    verifiedSurveyedElevation = sElevIt->value.GetString();

    return "";
}

std::string ChMAPMessage::intersectionGeometry::approach::ParseFromObject(rapidjson::Value& object, std::string path, int i) {
    path += "->approach[" + std::to_string(i) + "]";

    auto typeIt = object.FindMember("approachType");
    auto IDIt   = object.FindMember("approachID");

    if (typeIt == object.MemberEnd())
        return path + "->approachType -- Field not found";
    if (IDIt   == object.MemberEnd())
        return path + "->approachID -- Field not found";

    if (!typeIt->value.IsString())
        return path + "->approachType -- Field not string";
    approachType = typeIt->value.GetString();
    if (!IDIt->value.IsString())
        return path + "->approachID -- Field not string";
    approachID   = IDIt->value.GetString();

    std::string laneField;
    if (approachType == "Ingress" || approachType == "Egress")
        laneField = "drivingLanes";
    else if (approachType == "Crosswalk")
        laneField = "crosswalkLanes";

    auto laneIt = object.FindMember(laneField.c_str());

    if (laneIt == object.MemberEnd())
        return path + "->" + laneField + " -- Field not found.";

    auto& jLanes = laneIt->value;
    assert(jLanes.IsArray());

    lanes.resize(jLanes.Size());
    auto it = jLanes.Begin();
    std::string p = "";
    int j = 0;
    for (lane& l : lanes)
        if ((p = l.ParseFromObject(*(it++), path, j++)) != "")
            return p;

    return "";
}

std::string ChMAPMessage::intersectionGeometry::approach::lane::ParseFromObject(rapidjson::Value& object, std::string path, int j) {
    path += "->lanes[" + std::to_string(j) + "]";

    auto IDIt     = object.FindMember("laneID");
    auto nameIt   = object.FindMember("descriptiveName");
    auto typeIt   = object.FindMember("laneType");
    auto attIt    = object.FindMember("typeAttributes");
    auto sharedIt = object.FindMember("sharedWith");
    auto connIt   = object.FindMember("connections");
    auto manIt    = object.FindMember("laneManeuvers");
    auto nodeIt   = object.FindMember("laneNodes");

    if (IDIt       == object.MemberEnd())
        return path + "->laneID" + " -- Field not found.";
    if (nameIt     == object.MemberEnd())
        return path + "->descriptiveName" + " -- Field not found.";
    if (typeIt     == object.MemberEnd())
        return path + "->laneType" + " -- Field not found.";
    if (attIt      == object.MemberEnd())
        return path + "->typeAttributes" + " -- Field not found.";
    if (sharedIt   == object.MemberEnd())
        return path + "->sharedWith" + " -- Field not found.";
    if (connIt     == object.MemberEnd())
        return path + "->connections" + " -- Field not found.";
    if (manIt      == object.MemberEnd())
        return path + "->laneManeuvers" + " -- Field not found.";
    if (nodeIt     == object.MemberEnd())
        return path + "->laneNodes" + " -- Field not found.";

    if (!IDIt->value.IsString())
        return path + "->laneID -- Field not string.";
    laneID = IDIt->value.GetString();
    if (!nameIt->value.IsString())
        return path + "->descriptiveName -- Field not string.";
    descriptiveName = nameIt->value.GetString();
    if (!typeIt->value.IsString())
        return path + "->laneType -- Field not string.";
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
    std::string p = "";
    int k = 0;
    for (connection& c : connections)
        if ((p = c.ParseFromObject(*(cIt++), path, k++)) != "")
            return p + " ";

    auto mIt = mans.Begin();
    for (int& m : laneManeuvers)
        if (!mIt->IsInt())
            return path + "->laneManeuvers -- Field not int.";
        else
            m = (mIt++)->GetInt();

    auto nIt = nodes.Begin();
    k = 0;
    for (laneNode& n : laneNodes)
        if ((p = n.ParseFromObject(*(nIt++), path, k++)) != "")
            return p;

    return "";
}

// FINISH THIS THING
std::string ChMAPMessage::intersectionGeometry::approach::lane::connection::ParseFromObject(rapidjson::Value& object, std::string path, int k) {
    path += "->connections[" + std::to_string(k) + "]";

    auto IDIt   = object.FindMember("remoteID");
    auto fromIt = object.FindMember("fromLane");
    auto toIt   = object.FindMember("toLane");
    auto sigIt  = object.FindMember("signal_id");
    auto manIt  = object.FindMember("maneuvers");

    if (IDIt   == object.MemberEnd())
        return path + "->remoteID -- Field not found.";

    if (!IDIt->value.IsString())
        return path + "->remoteID -- Field not string.";
    remoteID = IDIt->value.GetString();
    if (remoteID == "")
        return "";

    if (fromIt == object.MemberEnd())
        return path + "->fromLane -- Field not found. ";
    if (toIt   == object.MemberEnd())
        return path + "->toLane -- Field not found.";
    if (sigIt  == object.MemberEnd())
        return path + "->signal_id -- Field not found.";
    if (manIt  == object.MemberEnd())
        return path + "->maneuvers -- Field not found.";

    if (!fromIt->value.IsString())
        return path + "->fromLane -- Field not string.";
    fromLane = fromIt->value.GetString();
    if (!toIt->value.IsString())
        return path + "->toLane -- Field not string.";
    toLane = toIt->value.GetString();
    if (!sigIt->value.IsString())
        return path + "->signal_id -- Field not string.";
    signal_id = sigIt->value.GetString();
    auto& mans = manIt->value;

    assert(mans.IsArray());
    maneuvers.resize(mans.Size());

    auto it = mans.Begin();
    for (std::string& m : maneuvers)
        if (!it->IsString())
            return path + "->maneuvers -- Field not string.";
        else
            m = (it++)->GetString();

    return "";
}

std::string ChMAPMessage::intersectionGeometry::approach::lane::laneNode::ParseFromObject(rapidjson::Value& object, std::string path, int k) {
    path += "->laneNodes[" + std::to_string(k) + "]";

    auto numIt = object.FindMember("nodeNumber");
    auto latIt = object.FindMember("nodeLat");
    auto lonIt = object.FindMember("nodeLong");
    auto eleIt = object.FindMember("nodeElev");
    auto widIt = object.FindMember("laneWidthDelta");

    if (numIt == object.MemberEnd())
        return path + "->nodeNumber -- Field not found.";
    if (latIt == object.MemberEnd())
        return path + "->nodeLat -- Field not found.";
    if (lonIt == object.MemberEnd())
        return path + "->nodeLong -- Field not found.";
    if (eleIt == object.MemberEnd())
        return path + "->nodeElev -- Field not found.";
    if (widIt == object.MemberEnd())
        return path + "->laneWidthDelta -- Field not found.";

    if (!numIt->value.IsInt())
        return path + "->nodeNumber -- Field not int.";
    nodeNumber     = numIt->value.GetInt();
    if (!latIt->value.IsDouble())
        return path + "->nodeLat -- Field not double.";
    nodeLat        = latIt->value.GetDouble();
    if (!lonIt->value.IsDouble())
        return path + "->nodeLong -- Field not double.";
    nodeLong       = lonIt->value.GetDouble();

    if (eleIt->value.IsString()) {
        nodeElev       = atoi(eleIt->value.GetString());
        return "";
    }

    if (widIt->value.IsString()) {
        laneWidthDelta = atoi(widIt->value.GetString());
        return "";
    }

    if (!eleIt->value.IsInt())
        return path + "->nodeElev -- Field not int or string.";
    nodeElev       = eleIt->value.GetInt();
    if (!widIt->value.IsInt())
        return path + "->laneWidthDelta -- Field not int or string.";
    laneWidthDelta = widIt->value.GetInt();

    return "";
}
