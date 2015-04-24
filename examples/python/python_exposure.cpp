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
 * @file python_exposure.cpp
 * @ingroup Examples
 * @author Xavier Provençal (\c xavier.provencal@univ-smb.fr )
 * Laboratoire de Mathematiques (CNRS, UMR 5127), Université de Savoie, France
 *
 * @date 2015/04/24
 *
 * Attempt to expose DGtal::LatticePolytop2D to Python.
 *
 */

///////////////////////////////////////////////////////////////////////////////
#include <cstdlib>
#include <iostream>

#include "DGtal/base/Common.h"
#include "DGtal/arithmetic/LatticePolytope2D.h"
#include "DGtal/helpers/StdDefs.h"

#include <boost/python.hpp>
///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;
using namespace boost::python;

///////////////////////////////////////////////////////////////////////////////
// Standard services - public :
//

using namespace Z2i;
typedef LatticePolytope2D<Space> LatticePolytop;

BOOST_PYTHON_MODULE(python_exposure)
{
  class_<LatticePolytop>( "LatticePolytop" )
    ;
}


/**
   Main.
*/
//int main( int argc, char** argv )
//{
//  if ( argc < 4 )
//    {
//      usage( argc, argv );
//      return 0;
//    }
//
//  //! [lower-integer-convex-hull-types]
//  //! [lower-integer-convex-hull-types]
//
//  //! [lower-integer-convex-hull-instantiation]
//  CIP cip;
//  cip.push_front( Point( -10, -10 ) );
//  cip.push_front( Point( -10, 10 ) );
//  cip.push_front( Point( 10, 10 ) );
//  cip.push_front( Point( 10, -10 ) );
//  Domain domain = cip.boundingBoxDomain();
//  Board2D board;
//  board << domain 
//        << CustomStyle( cip.className(), 
//                        new CustomColors( Color::Red, Color::None ) )
//        << cip;
//  board.saveEPS( "lower-integer-convex-hull.eps" );
//  board.clear();
//  //! [lower-integer-convex-hull-instantiation]
//
//  int a = atoi( argv[ 1 ] );
//  int b = atoi( argv[ 2 ] );
//  int c = atoi( argv[ 3 ] );
//
//  //! [lower-integer-convex-hull-process]
//  typedef LatticePolytope2D<Z2>::HalfSpace HalfSpace;
//  HalfSpace hs( Vector( a, b ), c );
//  cip.cut( hs );
//  DigitalSet aSet( domain );
//  Shapes<Domain>::makeSetFromPointPredicate( aSet, hs );
//  board << domain 
//        << CustomStyle( aSet.className(), 
//                        new CustomColors( Color::Green, Color::Green ) )
//        << SetMode( Point().className(), "Grid" )
//        << aSet
//        << CustomStyle( cip.className(), 
//                        new CustomColors( Color::Red, Color::None ) )
//        << cip;
//  board.saveEPS( "lower-integer-convex-hull-cut.eps" );
//  //! [lower-integer-convex-hull-process]
//
//  //! [lower-integer-convex-hull-stats]
//  std::cout << "Number of vertices        = " << cip.size() << std::endl;
//  std::cout << "Area                      = " << (((double)cip.twiceArea())/2.0) << std::endl;
//  std::cout << "Number of interior points = " << cip.numberInteriorPoints() << std::endl;
//  std::cout << "Number of boundary points = " << cip.numberBoundaryPoints() << std::endl;
//  //! [lower-integer-convex-hull-stats]
//  return 0;
//}
//
//
//
////                                                                           //
/////////////////////////////////////////////////////////////////////////////////
