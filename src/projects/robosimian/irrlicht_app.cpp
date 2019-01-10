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
// Author: Radu Serban
// =============================================================================
//
// =============================================================================

#include "irrlicht_app.h"

using namespace chrono;

namespace robosimian {

RobotIrrApp::RobotIrrApp(robosimian::RoboSimian* robot,
                         robosimian::Driver* driver,
                         const wchar_t* title,
                         irr::core::dimension2d<irr::u32> dims)
    : ChIrrApp(robot->GetSystem(), title, dims, false, true, true, irr::video::EDT_OPENGL),
      m_robot(robot),
      m_driver(driver),
      m_HUD_x(650),
      m_HUD_y(20),
      m_grid(false) {
    m_erecv = new RobotGUIEventReceiver(this);
    SetUserEventReceiver(m_erecv);
}

RobotIrrApp::~RobotIrrApp() {
    delete m_erecv;
}

void RobotIrrApp::EnableGrid(const ChCoordsys<>& csys, int nu, int nv) {
    m_gridCsys = csys;
    m_gridNu = nu;
    m_gridNv = nv;
    m_grid = true;
}

void RobotIrrApp::renderTextBox(const std::string& msg,
                                int xpos,
                                int ypos,
                                int length,
                                int height,
                                irr::video::SColor color) {
    irr::core::rect<irr::s32> mclip(xpos, ypos, xpos + length, ypos + height);
    GetVideoDriver()->draw2DRectangle(irr::video::SColor(90, 60, 60, 60),
                                      irr::core::rect<irr::s32>(xpos, ypos, xpos + length, ypos + height), &mclip);
    irr::gui::IGUIFont* font = GetIGUIEnvironment()->getBuiltInFont();
    font->draw(msg.c_str(), irr::core::rect<irr::s32>(xpos + 3, ypos + 3, xpos + length, ypos + height), color);
}

void RobotIrrApp::DrawAll() {
    if (m_grid) {
        irrlicht::ChIrrTools::drawGrid(GetVideoDriver(), 0.1, 0.1, m_gridNu, m_gridNv, m_gridCsys,
                                       irr::video::SColor(255, 255, 130, 80), true);
    }

    ChIrrAppInterface::DrawAll();

    char msg[100];

    sprintf(msg, "Time %.2f", m_robot->GetSystem()->GetChTime());
    renderTextBox(msg, m_HUD_x, m_HUD_y, 120, 15, irr::video::SColor(255, 250, 200, 00));

    auto type = std::string("Contact method: ") +
                (m_robot->GetSystem()->GetContactMethod() == ChMaterialSurface::NSC ? "NSC" : "SMC");
    renderTextBox(type.c_str(), m_HUD_x, m_HUD_y + 15, 120, 15, irr::video::SColor(255, 250, 200, 00));

    sprintf(msg, "Driver phase: %s", m_driver->GetCurrentPhase().c_str());
    renderTextBox(msg, m_HUD_x, m_HUD_y + 30, 120, 15, irr::video::SColor(255, 250, 200, 00));

    sprintf(msg, "omega FR: %.2f", std::abs(m_robot->GetWheelOmega(robosimian::FR)));
    renderTextBox(msg, m_HUD_x, m_HUD_y + 60, 120, 15, irr::video::SColor(255, 250, 200, 00));

    sprintf(msg, "omega RR: %.2f", std::abs(m_robot->GetWheelOmega(robosimian::RR)));
    renderTextBox(msg, m_HUD_x, m_HUD_y + 75, 120, 15, irr::video::SColor(255, 250, 200, 00));

    sprintf(msg, "omega FL: %.2f", std::abs(m_robot->GetWheelOmega(robosimian::FL)));
    renderTextBox(msg, m_HUD_x, m_HUD_y + 90, 120, 15, irr::video::SColor(255, 250, 200, 00));

    sprintf(msg, "omega RL: %.2f", std::abs(m_robot->GetWheelOmega(robosimian::RL)));
    renderTextBox(msg, m_HUD_x, m_HUD_y + 105, 120, 15, irr::video::SColor(255, 250, 200, 00));
}

bool RobotGUIEventReceiver::OnEvent(const irr::SEvent& event) {
    if (event.EventType != irr::EET_KEY_INPUT_EVENT)
        return false;

    if (!event.KeyInput.PressedDown) {
        switch (event.KeyInput.Key) {
            case irr::KEY_KEY_C:
                m_vis = (m_vis == robosimian::VisualizationType::MESH ? robosimian::VisualizationType::COLLISION
                                                                      : robosimian::VisualizationType::MESH);

                m_app->m_robot->SetVisualizationTypeChassis(m_vis);
                m_app->m_robot->SetVisualizationTypeSled(m_vis);
                m_app->m_robot->SetVisualizationTypeLimbs(m_vis);
                m_app->m_robot->SetVisualizationTypeWheels(m_vis);

                m_app->AssetBindAll();
                m_app->AssetUpdateAll();

                return true;
        }
    }
    return false;
}

}  // end namespace robosimian
