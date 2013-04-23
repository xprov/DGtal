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
 * @file testDigitalSetByNeighborTree.cpp
 * @ingroup Tests
 * @author Xavier Provençal (\c xavier.provencal@univ-savoie.fr )
 * Laboratoire de Mathematiques (CNRS, UMR 5807), Université de Savoie, France

 * @date 21/12/2012
 *
 * This file is part of the DGtal library
 */

/**
 * Description of test_DigitalSet <p>
 * Aim: simple tests of models of \ref CDigitalSet
 */

#include <cstdio>
#include <cmath>
#include <iostream>
#include <fstream>
#include <algorithm>
#include <string>

#include "DGtal/base/Common.h"
#include "DGtal/kernel/SpaceND.h"
#include "DGtal/kernel/domains/HyperRectDomain.h"
#include "DGtal/kernel/sets/CDigitalSet.h"
#include "DGtal/kernel/sets/CDigitalSetArchetype.h"
#include "DGtal/kernel/domains/CDomain.h"
#include "DGtal/kernel/domains/CDomainArchetype.h"
#include "DGtal/kernel/sets/DigitalSetByNeighborTree.h"

// #include "DGtal/helpers/StdDefs.h"

// #include "DGtal/io/boards/Board2D.h"


using namespace DGtal;
using namespace std;



int main( int argc, const char ** argv )
{
  typedef SpaceND<3> SpaceType;
  typedef HyperRectDomain<SpaceType> Domain;
  typedef SpaceType::Point Point;
  typedef DigitalSetByNeighborTree< Domain > DS;
  typedef DS::Tree Tree;
  typedef DS::Node Node;
  typedef DS::Direction Direction;

  bool res;
  res = true;
  DGtal::int32_t t[] =  { 0, 0, 0};
  Point a ( t );
  DGtal::int32_t t2[] = { 5, 5, 5};
  Point b ( t2);
  trace.beginBlock ( "Wouhou !" );
  Domain dom;
  DS ds( dom );

  Node *n, *m;
  // for ( int i=0; i<4; ++i )
  //   {
  //     n  = &ds.myTree.roots[i];
  //     cout << "i=" << i << ".\n";
  //     n->selfDisplay( cout );
  //     for ( int j=1; j<=2; ++j )
  //       {
  //         cout << "  d = " << j << "\n";
  //         m = ds.myTree.findNeighbor( n, Direction(j) ); 
  //         m->selfDisplay( cout, 2 );

  //         m = ds.myTree.findNeighbor( n, Direction(-j) ); 
  //         cout << "  d = " << -j << "\n";
  //         m->selfDisplay( cout, 2);
  //         cout << endl;
  //       }
  //   }

  // cout << ds << endl;

  n = & ds.myTree.roots[ 0 ];
  if ( argc < 4 )
    {
      cerr << "Erreur, manque les coords d'un points" << endl;
      exit(1);
    }
  Point p( atoi( argv[1]), atoi( argv[2] ), atoi( argv[3] ));

  Node * other_1 = ds.myTree.findNode( p, false );
  Node * other_2 = ds.myTree.findNode( p, true );
  Node * other_3 = ds.myTree.findNode( p, false );

  for ( int i=p[0]; i<0; ++i )
    {
      n = ds.myTree.findNeighbor( n, Direction( -1 ) );
    }
  for ( int i=0; i<p[0]; ++i )
    {
      n = ds.myTree.findNeighbor( n, Direction( 1 ) );
    }

  for ( int i=p[1]; i<0; ++i )
    {
      n = ds.myTree.findNeighbor( n, Direction( -2 ) );
    }
  for ( int i=0; i < p[1]; ++i )
    {
      n = ds.myTree.findNeighbor( n, Direction( 2 ) );
    }

  for ( int i=p[2]; i<0; ++i )
    {
      n = ds.myTree.findNeighbor( n, Direction( -3 ) );
    }
  for ( int i=0; i < p[2]; ++i )
    {
      n = ds.myTree.findNeighbor( n, Direction( 3 ) );
    }

  n->selfDisplay( cout );
  cout << n << " " << other_1 << " " << other_2 << " " << other_3 << endl;
  res = ( other_1 == NULL ) && ( n == other_2 ) && ( n == other_3);


  //for ( int i=-33; i<=33; ++i )
  //  {
  //    cout << i << " -> " ;
  //    //for ( int k=DS::Tree::depth( i )-1; k >=0; --k )
  //    for ( int k=5; k >=0; --k )
  //      {
  //        unsigned int mask = ( 1 << k );
  //        if ( i & mask )
  //          cout << "1" ;
  //        else
  //          cout << "0" ;
  //      }
  //    cout << endl;
  //  }
  
  ds.myTree.addPoint( p );

    {
      cout << "In the tree : \n";
      DS::Tree::Iterator it = ds.myTree.begin();
      DS::Tree::Iterator itEnd = ds.myTree.end();
      for ( ; it != itEnd ; ++it )
        {
          cout << (*it)->getPosition() << ", ";
        }
      cout << endl;
    }
    {
      cout << "In the set : \n";
      DS::ConstIterator it = ds.begin();
      DS::ConstIterator itEnd = ds.end();
      for ( ; it != itEnd; ++it )
        {
          cout << *it << ", ";
        }
      cout << endl;
    }

  trace.endBlock();
  trace.emphase() << ( res ? "Passed." : "Error." ) << endl;
  return res ? 0 : 1;
}

/** @ingroup Tests **/
