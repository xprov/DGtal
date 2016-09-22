
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
#include "DGtal/shapes/Shapes.h"
///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;

///////////////////////////////////////////////////////////////////////////////
//
const Color red   ( 255,   0,   0 );
const Color dred  ( 192,   0,   0 );
const Color green ( 0,   255,   0 );
const Color dgreen( 0,   192,   0 );
const Color blue  ( 0,     0, 255 );
const Color dblue ( 0,     0, 192 );

struct DigitalPlane
{
  typedef Z3i::Point Point;
  typedef Z3i::Vector Vector;
  typedef Z3i::Point::Component Integer;
  DigitalPlane( Vector v, Integer mu, Integer omega ) : 
    v(v), mu(mu), omega(omega)
  {}
  bool operator()( const Point& p ) const
    {
      int height = p.dot(v);
      return mu <= height && height < mu + omega;
    }
  Vector v;
  Integer mu;
  Integer omega;
};

int main()
{
  trace.beginBlock ( "Example dgtalBoard2D-5-points3D" );

  Z2i::Domain domain2D( Z2i::Point(-5,-5),    Z2i::Point(5,5) );
  Z3i::Domain domain3D( Z3i::Point(-5,-5,-5), Z3i::Point(5,5,5) );


  DigitalSetBySTLVector<Z3i::Domain> S( domain3D );
  std::vector<Z3i::Point> v( { Z3i::Point( 1,0,0 ),
                               Z3i::Point( 0,1,0 ),
                               Z3i::Point( 0,0,1 ),
                               Z3i::Point( 1,1,0 ),
                               Z3i::Point( 1,0,1 ),
                               Z3i::Point( 0,1,1 ) } );
  S.insert( v.begin(), v.end() );

  Board2D board( Color::White );
  board << SetMode( S.className(), "Grid" );
  //board << domain2D;
  //board << S;

  // Change projection on the fly
  // board << SetProjection( {-2.0,-2.0, 2.0, -2.0, 0.0, 3.0} );
  // board << CustomStyle( S.begin()->className(), new CustomColors( red, dred ) );
  // board << S;

  // Change projection on the fly
  // typedef LibBoard::Point PrjPoint;
  // board << CustomStyle( S.begin()->className(), new CustomColors( green, dred ) );
  // board << SetProjection( { PrjPoint(-4.0,-4.0), PrjPoint(4.0, -4.0), PrjPoint(0.0, 6.0) } );
  // board << S;

  Z3i::DigitalSet shape_set( domain3D );
  //Shapes<Z3i::Domain>::addNorm1Ball( shape_set, Z3i::Point( 5, 5, 5 ), 2 );
  //Shapes<Z3i::Domain>::addNorm2Ball( shape_set, Z3i::Point( 3, 3, 3 ), 2 );
  DigitalPlane p( Z3i::Vector( 3,5,8 ), 0, 16 );
  Shapes<Z3i::Domain>::makeSetFromPointPredicate( shape_set, p );

  Z3i::Object6_18 shape( Z3i::dt6_18, shape_set );
  board << SetMode( shape.className(), "DrawAdjacenciesNoArrow" );
  // set projection back to default for dimension 3.
  // board << SetProjection( 3, "default" );
  board << shape;

  board.saveCairo("out.png", Board2D::CairoPNG);
  
  trace.endBlock();




  return 0;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
