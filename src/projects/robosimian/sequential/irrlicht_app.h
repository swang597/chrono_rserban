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

#ifndef ROBO_IRRLICHT_APP_H
#define ROBO_IRRLICHT_APP_H

#include "chrono_irrlicht/ChIrrApp.h"
#include "chrono_irrlicht/ChIrrTools.h"

#include "chrono_models/robosimian/robosimian.h"

class RobotGUIEventReceiver;

class RobotIrrApp : public chrono::irrlicht::ChIrrApp {
  public:
    RobotIrrApp(chrono::robosimian::RoboSimian* robot,
                chrono::robosimian::Driver* driver,
                const wchar_t* title = 0,
                irr::core::dimension2d<irr::u32> dims = irr::core::dimension2d<irr::u32>(1000, 800));

    ~RobotIrrApp();

    void EnableGrid(const chrono::ChCoordsys<>& csys, int nu, int nv);

    virtual void DrawAll() override;

  private:
    void renderTextBox(const std::string& msg,
                       int xpos,
                       int ypos,
                       int length = 120,
                       int height = 15,
                       irr::video::SColor color = irr::video::SColor(255, 20, 20, 20));

  private:
    chrono::robosimian::RoboSimian* m_robot;
    chrono::robosimian::Driver* m_driver;

    RobotGUIEventReceiver* m_erecv;

    int m_HUD_x;  ///< x-coordinate of upper-left corner of HUD elements
    int m_HUD_y;  ///< y-coordinate of upper-left corner of HUD elements

    bool m_grid;
    chrono::ChCoordsys<> m_gridCsys;
    int m_gridNu;
    int m_gridNv;

    friend class RobotGUIEventReceiver;
};

class RobotGUIEventReceiver : public irr::IEventReceiver {
  public:
    RobotGUIEventReceiver(RobotIrrApp* app) : m_app(app), m_vis(chrono::robosimian::VisualizationType::COLLISION) {}

    virtual bool OnEvent(const irr::SEvent& event) override;

  private:
    chrono::robosimian::VisualizationType m_vis;
    RobotIrrApp* m_app;
};

#endif
