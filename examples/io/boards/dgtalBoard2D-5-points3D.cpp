
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
 * @file io/boards/dgtalBoard2D-5-points3D.cpp
 * @ingroup Examples
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5807), University of Savoie, France
 *
 * @date 2010/11/26
 *
 * An example file named dgtalBoard2D-5-points3D.
 * 
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include "DGtal/base/Common.h"
#include "DGtal/io/boards/Board2D.h"
#include "DGtal/io/Color.h"
#include "DGtal/helpers/StdDefs.h"
///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;
using namespace DGtal::Z3i;

///////////////////////////////////////////////////////////////////////////////

int main()
{
  trace.beginBlock ( "Example dgtalBoard2D-5-points3D" );

  Point p1( -2, -2, -2 );
  Point p2( 1, 0, 0 );
  Point p3( 0, 1, 0 );
  Point p4( 0, 0, 1 );
  Point p5( 1, 1, 0 );
  Point p6( 1, 0, 1 );
  Point p7( 0, 1, 1 );
  //Point p8( 1, 1, 1 );
  //Point p9( 2, 2, 2 );
  Z2i::Domain domain( Z2i::Point(-3,-3), Z2i::Point(3,3) );
  //Domain domain( p1, p3 );
  
  Board2D board( Color::White );
  board << SetMode( p1.className(), "Grid" );
  board << domain << p2 << p3 << p4 << p5 << p6 << p7;
  board << SetProjection( {-3.0,-2.0, 2.0, -2.0, 0.0, 3.0} );
  board << p2 << p3 << p4 << p5 << p6 << p7;

#ifdef WITH_CAIRO
  board.saveCairo("dgtalBoard2D-5-points3D.png", Board2D::CairoPNG);
#endif
  
  trace.endBlock();
  return 0;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
