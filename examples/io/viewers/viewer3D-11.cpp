/**
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU Lesser General Public License as
 *  published by the Free Software Foundation, either version 3 of the
 *  License, or  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 **/

/**
 * @file viewer3D-10-interaction.cpp
 * @ingroup examples/3dViewer
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2014/10/12
 *
 * Simple example of class Viewer3D.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include <QtGui/qapplication.h>
#include "DGtal/base/Common.h"
#include "DGtal/io/viewers/Viewer3D.h"
#include "DGtal/helpers/StdDefs.h"

///////////////////////////////////////////////////////////////////////////////
//

using namespace std;
using namespace DGtal;
using namespace Z3i;


class Particle
{
public :
  Particle();

  void init();
  void draw();
  void animate();

private :
  qglviewer::Vec speed_, pos_;
  int age_, ageMax_;
};


class Viewer : public QGLViewer
{
protected :
  virtual void draw();
  virtual void init();
  virtual void animate();
  virtual QString helpString() const;

private:
  int nbPart_;
  Particle* particle_;
};

//////////////////////   V i e w e r  ///////////////////////
void Viewer::init()
{
  restoreStateFromFile();
  glDisable(GL_LIGHTING);
  nbPart_ = 1000000;
  particle_ = new Particle[nbPart_];
  glPointSize(3.0);
  setGridIsDrawn();
  help();
  startAnimation();
}

void Viewer::draw()
{
  glBegin(GL_POINTS);
  for (int i=0; i<nbPart_; i++)
    particle_[i].draw();
  glEnd();
}

void Viewer::animate()
{
  for (int i=0; i<nbPart_; i++)
    particle_[i].animate();
}

QString Viewer::helpString() const
{
  QString text("<h2>A n i m a t i o n</h2>");
  text += "Use the <i>animate()</i> function to implement the animation part of your ";
  text += "application. Once the animation is started, <i>animate()</i> and <i>draw()</i> ";
  text += "are called in an infinite loop, at a frequency that can be fixed.<br><br>";
  text += "Press <b>Return</b> to start/stop the animation.";
  return text;
}

///////////////////////   P a r t i c l e   ///////////////////////////////

Particle::Particle()
{
  init();
}

void Particle::animate()
{
  //speed_.z -= 0.05f;
  //pos_ += 0.1f * speed_;

  //if (pos_.z < 0.0)
  //  {
  //    speed_.z = -0.8*speed_.z;
  //    pos_.z = 0.0;
  //  }
  //
  pos_ += 0.1f * speed_;
  pos_[2] = 0.4f*(600-age_)/600.0f*cos(age_/30.0f);
   

  if (++age_ == ageMax_)
    init();
}

void Particle::draw()
{
  glColor3f(age_/(float)ageMax_, age_/(float)ageMax_, 1.0);
  glVertex3fv(pos_);
}


void Particle::init()
{
  pos_ = Vec(0.0, 0.0, 1.0);
  float angle = 2.0 * M_PI * rand() / RAND_MAX;
  //float norm  = 0.04 * rand() / RAND_MAX;
  float norm  = 0.04;
  speed_ = Vec(norm*cos(angle), norm*sin(angle), 0.0);
  age_ = 0;
  ageMax_ = 200 + static_cast<int>(400.0 * rand() / RAND_MAX);
}



int main(int argc, char** argv)
{
  QApplication application(argc,argv);

  Viewer viewer;

  viewer.setWindowTitle("animation");

  viewer.show();

  return application.exec();
}
//
//


typedef Viewer3D<Space,KSpace> MyViewer;
typedef MyViewer::SelectCallbackFct SelectCallbackFct;
typedef KSpace::SCell SCell;

struct BigData
{
  KSpace K;
  std::map< int32_t, SCell > cells;
};

int reaction1( void* viewer, int32_t name, void* data )
{
  BigData* bg = (BigData*) data;
  trace.info() << "Reaction1 with name " << name 
               << " cell " << bg->K.sKCoords( bg->cells[ name ] ) << std::endl;

  MyViewer* vw = (MyViewer*) viewer;
  (*vw) << CustomColors3D( Color::Yellow, Color::Yellow ) << bg->cells[ 10004 ];
  (*vw) << MyViewer::updateDisplay;
  return 0;
}
int reaction23( void* viewer, int32_t name, void* data )
{
  BigData* bg = (BigData*) data;
  trace.info() << "Reaction23 with name " << name
               << " cell " << bg->K.sKCoords( bg->cells[ name ] ) << std::endl;
  return 0;
}
///////////////////////////////////////////////////////////////////////////////
// Standard services - public :

//int main( int argc, char** argv )
//{
//  QApplication application(argc,argv);
//  BigData data;
//  Point p1( 0, 0, 0 );
//  Point p2( 5, 5 ,5 );
//  Point p3( 2, 3, 4 );
//  KSpace & K = data.K; 
//  K.init( p1, p2, true );
//  
//  MyViewer viewer( K );
//  viewer.show();
//  SCell surfel1 = K.sCell( Point( 1, 1, 2 ), KSpace::POS ); 
//  SCell surfel2 = K.sCell( Point( 3, 3, 4 ), KSpace::NEG ); 
//  SCell surfel3 = K.sCell( Point( 5, 6, 5 ), KSpace::POS ); 
//  SCell surfel4 = K.sCell( Point( 0, 0, 0 ), KSpace::POS ); 
//  data.cells[ 10001 ] = surfel1;
//  data.cells[ 10002 ] = surfel2;
//  data.cells[ 10003 ] = surfel3;
//  data.cells[ 10004 ] = surfel3;
//  viewer << SetMode3D( surfel1.className(), "Basic" );
//  viewer << SetName3D( 10001 ) << CustomColors3D( Color::Red, Color::Red ) << surfel1;
//  viewer<< MyViewer::updateDisplay;
//  sleep(5);
//  viewer << SetName3D( 10002 ) << CustomColors3D( Color::Green, Color::Green ) << surfel2;
//  viewer<< MyViewer::updateDisplay;
//  sleep(5);
//  viewer << SetName3D( 10003 ) << CustomColors3D( Color::Blue, Color::Blue ) << surfel3;
//  viewer<< MyViewer::updateDisplay;
//  sleep(5);
//  viewer << SetSelectCallback3D( reaction1,  &data, 10001, 10001 );
//  viewer << SetSelectCallback3D( reaction23, &data, 10002, 10003 );
//  viewer<< MyViewer::updateDisplay;
//  return application.exec();
//}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
