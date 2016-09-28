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
 * @file tutorial-examples/shapeGridCurveEstimator.cpp
 * @ingroup tutorial-examples
 * @author Tristan Roussillon (tristan.roussillon@liris.cnrs.fr)
 *
 *
 * @date 2010/10/17
 * 
 * @brief An example of generating a grid curve from a parametric shape
 * and estimating its length. 
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include <stdlib.h>
///////////////////////////////////////////////////////////////////////////////

#include "DGtal/base/Common.h"
#include "DGtal/helpers/StdDefs.h"
#include "DGtal/geometry/curves/Polyline.h"
#include "DGtal/io/boards/Board2D.h"

///////////////////////////////////////////////////////////////////////////////

using namespace DGtal;
using namespace Z3i;

template<typename TPoint>
void DisplaySurfels( Board2D& board, TPoint startPoint )
{
  typedef TPoint Point;
  typedef Polyline<Point> Poly;

  const Dimension dim = Point::size();

  for ( int i=0; i<dim; ++ i )
    {
      Poly p;
      Point pt = startPoint;
      p.push_back( pt );
      pt[i] += dim+1;
      p.push_back( pt );
      board << p;
    }
  for ( int i=0; i<dim-1; ++i )
    {
      for ( int j=i+1; j<dim; ++j )
        {
          Poly p;
          Point p0,p1,p2,p3;
          p0 = p1 = p2 = p3 = startPoint;
          p1[i] += dim;
          p2[i] += dim; p2[j] += dim;
          p3[j] += dim;
          p.push_back( p0 );
          p.push_back( p1 );
          p.push_back( p2 );
          p.push_back( p3 );
          p.push_back( p0 );
          board << p;
        }
    }
}

int main( int argc, const char** argv )
{
  typedef PointVector<2,int> Point2D;
  typedef PointVector<3,int> Point3D;
  typedef PointVector<4,int> Point4D;
  typedef PointVector<5,int> Point5D;
  typedef PointVector<6,int> Point6D;
  typedef PointVector<7,int> Point7D;
  typedef Polyline<Point2D> Polyline2D;
  typedef Polyline<Point3D> Polyline3D;
  typedef Polyline<Point4D> Polyline4D;
  typedef Polyline<Point5D> Polyline5D;
  typedef Polyline<Point6D> Polyline6D;
  typedef Polyline<Point7D> Polyline7D;

  DGtal::Board2D aBoard( Color::White );

  Polyline2D poly2D;
  Polyline3D poly3D;
  Polyline4D poly4D;
  Polyline5D poly5D;
  Polyline6D poly6D;
  Polyline7D poly7D;

  // Each dimension has its own display mode.
  aBoard << SetMode( poly2D.className(), "" );
  aBoard << SetMode( poly3D.className(), "Both" );
  aBoard << SetMode( poly4D.className(), "Fill" );
  aBoard << SetMode( poly5D.className(), "Both" );
  aBoard << SetMode( poly6D.className(), "Edges" );
  //aBoard << SetMode( poly7D.className(), "Both" );

  CustomStyle myCS6D( poly6D.className(), new CustomPen( Color(255,0,0), Color(0,0,255,100), 4 ) );
  CustomStyle myCS7D( poly7D.className(), new CustomPen( Color(255,0,0), Color(0,0,255,100), 4 ) );


  Point2D p2d;
  Point3D p3d( {0,10,5} );
  Point4D p4d( {25,0,0,0} );
  Point5D p5d( {0,10,10,0,0} );
  Point6D p6d( {25,0,0,0,10,10} );
  Point7D p7d( {0,0,20,20,0,0,0} );

  DisplaySurfels( aBoard, p2d );
  DisplaySurfels( aBoard, p3d );
  DisplaySurfels( aBoard, p4d );
  DisplaySurfels( aBoard, p5d );
  aBoard << myCS6D; // has no effect because a MODE was already defined for 6D
  DisplaySurfels( aBoard, p6d );
  aBoard << myCS7D; // ok 
  DisplaySurfels( aBoard, p7d );

  aBoard.saveCairo( "dgtalBoard2D-6-polyline.png", Board2D::CairoPNG);
  aBoard.saveSVG( "dgtalBoard2D-6-polyline.svg" );
  return 0;

}

///////////////////////////////////////////////////////////////////////////////

