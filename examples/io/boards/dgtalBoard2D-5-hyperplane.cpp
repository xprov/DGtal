
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
#include <stdlib.h>
#include <time.h>
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

template<int dim, typename Component>
struct DigitalPlane
{
  typedef PointVector<dim,Component> Point;
  typedef PointVector<dim,Component> Vector;
  typedef Component Integer;
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

template<int dim, typename Component>
struct PlaneIntersection {

    typedef PointVector<dim,Component> Point;
    typedef PointVector<dim,Component> Vector;
    typedef DigitalPlane<dim,Component> DP;

    PlaneIntersection( const std::vector<Vector>& v ) 
      {
        for ( unsigned int i=0; i<v.size(); ++i )
          {
            myPlanes.push_back( DP( v[i], 0, v[i].norm1() ) );
          }
      }

    bool operator()( const Point& p ) const
      {
        for ( unsigned int i=0; i<myPlanes.size(); ++i )
          {
            if ( ! myPlanes[i](p) )
              return false;
          }
        return true;
      }

    std::vector< DP > myPlanes;
};



template<int dim>
void dgtalPlaneToBoard( Board2D & board, const std::vector<int> v, int omega )
{
  typedef DGtal::int32_t Integer;
  typedef SpaceND<dim,Integer> Space;
  typedef MetricAdjacency<Space,1> Adj1;
  typedef MetricAdjacency<Space,2> Adj2;
  typedef DigitalTopology<Adj1,Adj2> Topology;
  typedef typename Space::Point Point;
  typedef typename Space::Vector Vector;

  typedef HyperRectDomain<Space> Domain;
  typedef typename DigitalSetSelector< Domain, SMALL_DS+HIGH_BEL_DS >::Type DigitalSet;
  typedef Object< Topology, DigitalSet> Object_1_2;

  static const Adj1 adj1;
  static const Adj2 adj2;
  const Topology t( adj1, adj2 );


  Point p0,p1;
  for ( int i=0; i<dim; ++i )
    {
      p0[i] = -2;
      p1[i] = 2;
    }


  Vector n;
  for ( int i=0; i<dim; ++i )
    n[i] = v[i];

  Domain d(p0,p1);
  DigitalSet shape_set( d );
  DigitalPlane<dim,Integer> p( n, 0, omega );
  Shapes<Domain>::makeSetFromPointPredicate( shape_set, p );

  Object_1_2 shape( t, shape_set );
  board << SetMode( shape.className(), "DrawAdjacenciesNoArrow" );
  board << shape;
}



int main(int argc, const char ** argv )
{
  (void) argc; (void) argv;
  std::vector<int> v;
  int omega;
  if ( argc < 3 )
    {
      v.push_back( 3 );
      v.push_back( 5 );
      omega = 8;
    }
  else 
    {
      for ( int i=1; i<argc-1; ++i )
        {
          v.push_back( atoi( argv[i] ) );
        }
      omega = atoi( argv[argc-1] );
    }
  int n = v.size();

  trace.beginBlock ( "Example dgtalBoard2D-5-points3D" );
  stringstream ss;
  ss << "# Digital plane { x in Z^" << n << " | 0 <= x.(" << v[0];
  for ( int i=1; i<n; ++i )
    {
      ss << "," << v[i];
    }
  ss << ") < " << omega << " }\n";
  trace.info() << ss.str();

  Board2D board( Color::White );
  if ( n == 2 ) dgtalPlaneToBoard<2>( board, v, omega );
  if ( n == 3 ) dgtalPlaneToBoard<3>( board, v, omega );
  if ( n == 4 ) dgtalPlaneToBoard<4>( board, v, omega );
  if ( n == 5 ) dgtalPlaneToBoard<5>( board, v, omega );
  if ( n == 6 ) dgtalPlaneToBoard<6>( board, v, omega );
  if ( n == 7 ) dgtalPlaneToBoard<7>( board, v, omega );
  board.saveSVG( "dgtalBoard2D-5-hyperplane.svg" );
  board.saveCairo( "dgtalBoard2D-5-hyperplane.png", 
                  Board2D::CairoPNG ); // but png doesn't manage transparency...

  trace.endBlock();
  return 0;

  
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
