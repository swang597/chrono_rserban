// ============================================================================================
// Render simulation data
// 
// DATA ASSUMED TO BE SPECIFIED IN A RIGHT-HAND-FRAME WITH Z UP
//
// The following variables are assumed to be specified (externally, in an included header):
// datafile         - name of input data file
// cam_loc          - camera location point
// cam_lookat       - camera "look at" point
// cam_perspective  - true/false
// cam_angle        - camera FOV angle
// ============================================================================================

#version 3.7;
global_settings { assumed_gamma 1 }
global_settings { ambient_light rgb<1, 1, 1> }
      
// ============================================================================================
//
// OPTIONS
//
// ============================================================================================

//#declare datafile = "DoubleWishbone.dat"
//#declare cam_perspective = true;
//#declare cam_lookat = <-1.25, 0, 0>;
//#declare cam_loc = <-3.25, -1.75, 1>;
//#declare cam_angle = 50;

// -------------------------------------------------------
// Render objects?
#declare render_objects = true;

// Render static objects?
#declare render_static = false;

// Render links?
#declare render_links = true;  

// Render TSDAs?
#declare render_tsda = true;


// -------------------------------------------------------
// Dimensions for rendering links
#declare revolute_radius = 0.015;
#declare revolute_halflen = 0.04;

#declare cyl_radius = 0.04;
#declare cyl_halflen = 0.05;
 
#declare spherical_radius = 0.035;
 
#declare spring_radius = 0.05;
#declare damper_radius = 0.016;
  
#declare distance_cnstr_radius = 0.007;

#declare prismatic_halflen = 1;  
#declare prismatic_radius = 0.015;

// -------------------------------------------------------
// Draw global frame?
#declare draw_global_frame = true;
#declare global_frame_radius = 0.004;
#declare global_frame_len = 0.15;

// Draw body frames?
#declare draw_body_frame = true;
#declare body_frame_radius = 0.002;
#declare body_frame_len = 0.25;

// Draw shadows?       
#declare draw_shadows = false;
       
// -------------------------------------------------------------
// CAMERA SETTINGS (perspective/ortographic and location/lookat)
//
// RIGHT-HAND FRAME WITH Z UP

// Convert camera location and look_at to LEFT-HAND-FRAME with Y up
#declare cloc = <cam_loc.x, cam_loc.z, cam_loc.y>;
#declare clookat = <cam_lookat.x, cam_lookat.z,  cam_lookat.y>;

// Create the camera
camera { 
    #if (cam_perspective)
        perspective   
    #else
        orthographic
    #end
    location cloc
    look_at  clookat  
    angle cam_angle
}


// ============================================================================================  
// 
// INCLUDES
//
// ============================================================================================   

#include "shapes.inc"
#include "strings.inc"
#include "textures.inc"
#include "colors.inc"        
#include "functions.inc"  
#include "transforms.inc"

// ============================================================================================ 
//
// MACRO DEFINITIONS
//
// ============================================================================================

// --------------------------------------------------------------------------------------------
// Convert a LEFT-HAND-FRAME quaternion into a (4x3) Povray transformation matrix
//
#macro QToMatrix(Q)
  // Use: matrix <M[0].x,M[0].y,M[0].z,M[1].x,M[1].y,M[1].z,M[2].x,M[2].y,M[2].z,M[3].x,M[3].y,M[3].z>
  #local X2 = Q.x + Q.x;
  #local Y2 = Q.y + Q.y;
  #local Z2 = Q.z + Q.z;
  #local XX = Q.x * X2;
  #local XY = Q.x * Y2;
  #local XZ = Q.x * Z2;
  #local YY = Q.y * Y2;
  #local YZ = Q.y * Z2;
  #local ZZ = Q.z * Z2;
  #local TX = Q.t * X2;
  #local TY = Q.t * Y2;
  #local TZ = Q.t * Z2;
  array[4] {<1.0 - (YY + ZZ),XY + TZ,XZ - TY>,<XY - TZ,1.0 - (XX + ZZ),YZ + TX>,<XZ + TY,YZ - TX,1.0 - (XX + YY)>,<0,0,0>}
#end

#macro QMatrix(Q)
  // Use quaternion directly as an object modifier
  #local M = QToMatrix(Q)
  transform { matrix <M[0].x,M[0].y,M[0].z,M[1].x,M[1].y,M[1].z,M[2].x,M[2].y,M[2].z,M[3].x,M[3].y,M[3].z> }
#end
           
// --------------------------------------------------------------------------------------------
// Set RIGHT-HAND-FRAME location and orientation as an object modifier. 
//
// Input:
//    L  -  location vector <x,y,z>
//    Q  -  orientation quaternion <e0, e1, e2, e3> 
//
// NOTE: L and Q are assumed to be expressed in a right-hand frame with Z up. 
// 
// The conversion from right-hand frames with Z up (input) to left-hand frames with Y up (povray)
// requires:
//    (1) swapping y and z
//    (2) changing the sign of the rotation angle. 
//
#macro position(L, Q)
    #local pL = <L.x, L.z, L.y>;  
    #local pQ = <Q.y,Q.t,Q.z,-Q.x>;  // i.e., <e1,e3,e2,-e0>   
    QMatrix(pQ) 
    translate pL
#end     
     
// --------------------------------------------------------------------------------------------
// Create a RIGHT-HAND-FRAME quaternion from an angle and an axis
//
#macro Q_from_AngAxis(a, axis)
  #local a2 = a/2;
  #local sina2 = sin(a2);
  #local e0 = cos(a2); 
  #local e1 = sina2 * axis.x;
  #local e2 = sina2 * axis.y;
  #local e3 = sina2 * axis.z;  
  #local Q = <e0,e1,e2,e3>;
  Q
#end

// --------------------------------------------------------------------------------------------
// Render a RIGHT-HAND-FRAME with Z up
//    
#macro XYZframe(len, rad)  
   #local White_texture = texture {pigment{color rgb<1,1,1>}   finish{phong 1}}
   #local Red_texture   = texture {pigment{color rgb<0.8,0,0>} finish{phong 1}}
   #local Green_texture = texture {pigment{color rgb<0,0.8,0>} finish{phong 1}}
   #local Blue_texture  = texture {pigment{color rgb<0,0,0.8>} finish{phong 1}}  
   
   union {  
        // X-axis
        cylinder{
          <0,0,0>, <len,  0,  0>, rad
          texture{checker texture{Red_texture} texture{White_texture} scale len/10 translate<0,rad,rad>}
          no_shadow
        }  
        cone{
          <len, 0, 0>, 4*rad, <len + 8*rad, 0, 0>, 0
          texture{Red_texture}
          no_shadow
        } 
              
        // Y-axis
        cylinder{
          <0,0,0>, < 0,  0, len>, rad
          texture{checker texture{Green_texture} texture{White_texture} scale len/10 translate<rad,rad,0>}
          no_shadow
        }   
        cone{
          <0, 0, len>, 4*rad, <0, 0, len + 8*rad>, 0
          texture{Green_texture}
          no_shadow
        }
            
        // Z-axis
        cylinder{
          <0,0,0>, < 0, len,  0>, rad
          texture{checker texture{Blue_texture} texture{White_texture} scale len/10 translate<rad,0,rad>}
          no_shadow
        } 
        cone{
          <0, len, 0>, 4*rad, <0, len + 8*rad, 0>, 0
          texture{Blue_texture}
          no_shadow
        }
   }
#end


// --------------------------------------------------------------------------------------------
// Render a mesh at specified location and with specified orientation
//
// NOTES:
//   (1) It is assumed that a file [mesh_name].inc exists in the search path
//   (2) This file must define a macro "mesh_name()" which returns a mesh object
//   (3) The mesh vertices are specified in a RIGHT-HAND-FRAME with Z up.
//
// Use as:
//    object {
//       MyMesh(mesh_name, pos, rot)
//       [object_modifiers]
//    }
//
#macro MyMesh(mesh_name, pos, rot)
    Parse_String(concat("#include \"", mesh_name,"\""))
    Parse_String(concat(mesh_name, " position(pos,rot)"))
#end   


// --------------------------------------------------------------------------------------------          
// Render a curve at specified location and with specified orientation
//
// NOTES:
//    (1) It is assumed that a file [curve_name].inc exists in the search path
//    (2) This file muct define a macro "curve_name()" which return an object
//
// Use as:
//    object {
//       MyCurve(curve_name, pos, rot)
//       [object_modifiers]
//    }
//
#macro MyCurve(curve_name, pos, rot)
    Parse_String(concat("#include \"", curve_name,"\""))
    Parse_String(concat(curve_name, " position(pos,rot)"))
#end

             
// --------------------------------------------------------------------------------------------          
// Render a coil spring of specified (radius) between the two specified points          
#macro CoilSpring(p1, p2, rad)
  #local IsoTexture = texture { pigment { rgb <1.0, 0.6, 0.6> } finish {phong 0.6 phong_size 250 } }
                         
  #local center = 0.5 * (p1 + p2);
  #local dir = p2-p1;
  #local len = vlength(dir);                     
  #local LCorner = 1.1 * <-rad, -len/2, -rad>;
  #local RCorner = 1.1 * <rad, len/2,  rad>;
  isosurface {
     function { f_helix1(x,y,z, 1, 100, 0.004, rad, 1, 1, 0) }
         //P0= number of helixes
         //P1= frequency
         //P2= minor radius
         //P3= major radius
         //P4= Y scale cross section
         //P5= cross section
         //P6= cross section rotation (¡)
     contained_by { box { LCorner, RCorner } }
     max_gradient 1.5
     texture { IsoTexture }
     no_shadow

     // Apply transform (translation + rotation) directly in POV-Ray frame
     Point_At_Trans(<dir.x, dir.z, dir.y>)
     translate <center.x, center.z, center.y>
  }
#end

// ============================================================================================
//
// READ DATA AND RENDER SCENE
//
// ============================================================================================

// Open data file
#warning concat("LOADING DATA FILE : ",  datafile, "\n")
#fopen MyDataFile datafile read 

// Read first line: number of bodies, visualization objects, and links
#read (MyDataFile, numBodies, numObjects, numLinks, numTSDA)

        // ---------------------------------------------
        // RENDER BODY FRAMES
        // ---------------------------------------------

#for (i, 1, numBodies)
    #read (MyDataFile, id, active, ax, ay, az, e0, e1, e2, e3)
    #if (draw_body_frame & (active | render_static))  
       #if (draw_global_frame & (id = -1))
         object {
            XYZframe(0.15, body_frame_radius * 1.5) 
            position(<ax,ay,az>,<e0,e1,e2,e3>)  
         }        
       #else             
         object {
            XYZframe(body_frame_len, body_frame_radius) 
            position(<ax,ay,az>,<e0,e1,e2,e3>)  
         }    
       #end
    #end
#end

        // ---------------------------------------------
        //    RENDER OBJECTS (VISUAL ASSETS)
        // ---------------------------------------------

#for (i, 1, numObjects)

    #read (MyDataFile, id, active, ax, ay, az, e0, e1, e2, e3, cR, cG, cB, shape)

    #switch (shape)

        // sphere -------------  
        #case (0)
            # read (MyDataFile, ar)  
            #if (render_objects & (active | render_static))
               sphere {
                 <0,0,0>, ar
                 position(<ax,ay,az>,<e0,e1,e2,e3>)
                 pigment {color rgbt <cR, cG, cB, 0> }
                 finish {diffuse 1 ambient 0.0 specular .05 } 
               }
            #end
        #break

        // box ----------------
        #case (2)
           #read (MyDataFile, hx, hy, hz)
           #if (render_objects & (active | render_static))  
              box {   
                <-hx, -hz, -hy>, 
                <hx, hz, hy>     
                position(<ax,ay,az>,<e0,e1,e2,e3>)
                pigment {color rgbt <cR, cG, cB, 0>}
                finish {diffuse 1 ambient 0.0 specular .05 } 
              }
           #end
        #break

        // cylinder --------------
        #case (3)
            #read (MyDataFile, ar, p1x, p1y, p1z, p2x, p2y, p2z)
            #if (p1x = p2x & p1y = p2y & p1z = p2z) 
                 #warning concat("DEGENERATE CYLINDER : ",  str(id,-3,0), "\n")
            #end
            #if (render_objects & (active | render_static))
               cylinder {
                 <p1x,p1z,p1y>, <p2x,p2z,p2y>, ar
                 pigment {color rgbt <cR, cG, cB, 0> transmit 0}
                 position(<ax,ay,az>,<e0,e1,e2,e3>)
                 finish {diffuse 1 ambient 0.0 specular .05 }
               }
            #end
        #break

        // rounded cylinder --------------
        #case (10)
            #read (MyDataFile, ar, hl, sr)
            #if (render_objects & (active | render_static))
                 object {
                 Round_Cylinder(<0,0,hl + sr>, <0,0,-hl - sr>, ar+sr, sr, 0)
                 pigment {color rgbt <cR, cG, cB, 0> }
                 position(<ax,ay,az>,<e0,e1,e2,e3>)
                 finish {diffuse 1 ambient 0.0 specular .05 }
               }
            #end
        #break

        // capsule ------------
        #case (7)
            #read (MyDataFile, ar, hl)
            #if (render_objects & (active | render_static))
              sphere_sweep {
                linear_spline
                2
                <0,0,-hl>,ar,<0,0,hl>,ar
                pigment {color rgbt <cR, cG, cB, 0> }
                position(<ax,ay,az>,<e0,e1,e2,e3>)     
                finish {diffuse 1 ambient 0.0 specular .05 }
              }
            #end
        #break

        // mesh ----------------
        #case (5)
            #read (MyDataFile, mesh_name)
            #if (render_objects & (active | render_static))
               #warning concat("Mesh name: ", mesh_name, "\n")
               object { MyMesh(mesh_name, <ax,ay,az>,<e0,e1,e2,e3>) }
            #end
        #break

        // Bezier curve ----------
        #case (12)
             #read (MyDataFile, curve_name) 
             #if (render_objects)
                #warning concat("Curve name: ", curve_name, "\n")
                object { MyCurve(curve_name, <ax,ay,az>,<e0,e1,e2,e3>) }
             #end
        #break

    #end  // switch (shape)

#end  // for objects

        // ---------------------------------------------
        //    RENDER LINKS
        // ---------------------------------------------   

#if (render_links)
  #for (i, 1, numLinks)
  
    #read (MyDataFile, link)
     
    #switch (link)
  
        // Spherical ------
        #case (1)
            #read (MyDataFile, px, py, pz)
            sphere {
            <px,pz,py>, spherical_radius
            pigment{color Bronze2 transmit 0.3}
        }
        #break
  
        // Revolute -------
        #case (0)
            #read (MyDataFile, px, py, pz, dx, dy, dz)
            cylinder {
                  <px-revolute_halflen*dx,  pz-revolute_halflen*dz, py-revolute_halflen*dy>, 
                  <px+revolute_halflen*dx,  pz+revolute_halflen*dz, py+revolute_halflen*dy>, 
                  revolute_radius   
                  pigment{color Bronze2 transmit 0.3}
            }
        #break

        // Cylindrical -------
        #case (8)
            #read (MyDataFile, px, py, pz, dx, dy, dz)
            cylinder {
                  <px-cyl_halflen*dx,  pz-cyl_halflen*dz, py-cyl_halflen*dy>, 
                  <px+cyl_halflen*dx,  pz+cyl_halflen*dz, py+cyl_halflen*dy>, 
                  cyl_radius   
                  pigment{color Bronze2 transmit 0.3}
            }
        #break
  
        // Prismatic -------
        #case (2)
            #read (MyDataFile, px, py, pz, dx, dy, dz)
            cylinder {
                  <px-prismatic_halflen*dx,  pz-prismatic_halflen*dz, py-prismatic_halflen*dy>, 
                  <px+prismatic_halflen*dx,  pz+prismatic_halflen*dz, py+prismatic_halflen*dy>, 
                  prismatic_radius
                  pigment {color Bronze2 transmit 0.3}
              }
        #break
  
        // Universal ----
        #case (3)
            #read (MyDataFile, px, py, pz, ux, uy, uz, vx, vy, vz)
            cylinder {
                  <px-revolute_halflen*ux,  pz-revolute_halflen*uz, py-revolute_halflen*uy>, 
                  <px+revolute_halflen*ux,  pz+revolute_halflen*uz, py+revolute_halflen*uy>, 
                  revolute_radius   
                  pigment{color Bronze2 transmit 0.3}
            }  
            cylinder {
                  <px-revolute_halflen*vx,  pz-revolute_halflen*vz, py-revolute_halflen*vy>, 
                  <px+revolute_halflen*vx,  pz+revolute_halflen*vz, py+revolute_halflen*vy>, 
                  revolute_radius
                  pigment{color Bronze2 transmit 0.3}
            }
        #break
  
        // LinkSpring ------
        #case (6)
            #read (MyDataFile, p1x, p1y, p1z, p2x, p2y, p2z)
            //cylinder {
            //  <p1x,p1z,p1y>, <p2x,p2z,p2y>, spring_radius
            //  texture{Peel scale revolute_halflen/2}
            //}  
        #break
    
        // TSDA ------
        #case (7)
            #read (MyDataFile, p1x, p1y, p1z, p2x, p2y, p2z)
            //CoilSpring(<p1x,p1y,p1z>, <p2x,p2y,p2z>, spring_radius)
        #break
  
        // LinkEngine ------
        #case (5)
            #read (MyDataFile, px, py, pz, dx, dy, dz)
            cylinder {
                  <px-revolute_halflen*dx,  pz-revolute_halflen*dz, py-revolute_halflen*dy>, 
                  <px+revolute_halflen*dx,  pz+revolute_halflen*dz, py+revolute_halflen*dy>, 
                  spring_radius   
                  pigment{color Scarlet transmit 0.5}
            }
        #break
  
        // Distance constraint -------
        #case (4)
            #read (MyDataFile, p1x, p1y, p1z, p2x, p2y, p2z)
            cylinder {
              <p1x,p1z,p1y>, <p2x,p2z,p2y>, distance_cnstr_radius
              pigment {color DarkSlateGray }
            }
        #break
  
    #end  // switch (link)
      
  #end  // for links
#end  // if (render_links)     


        // ---------------------------------------------
        //    RENDER TSDA
        // ---------------------------------------------   

#if (render_tsda)
  #for (i, 1, numTSDA)

    #read (MyDataFile, tsda, p1x, p1y, p1z, p2x, p2y, p2z)

    #switch (tsda)
      // segment
      #case (0)
         cylinder {
           <p1x,p1z,p1y>, <p2x,p2z,p2y>, damper_radius 
           pigment{color rgb <1.0, 0.6, 0.6> transmit 0.3}
         }
      #break

      // coil
      #case (1)
        CoilSpring(<p1x,p1y,p1z>, <p2x,p2y,p2z>, spring_radius)
      #break

    #end // switch (tsda)
  #end // for tsda
#end // if (render_tsda)

// Done processing data file.
#fclose MyDataFile

        
// ============================================================================================     
//
// ENVIRONMENT
//
// ============================================================================================     


  // Set a color of the background (sky)
  background { color rgb <1, 1, 1> }
   
   // Create a regular point light source behind the camera
  #if (draw_shadows)
    light_source { 
      1500*(cloc - clookat)
      color rgb <1,1,1>
      translate <0, 1500, 0>
    }         
  #else
    light_source { 
      1500*(cloc - clookat)
      color rgb <1,1,1>
      translate <0, 1500, 0> 
      shadowless
    }
  #end

