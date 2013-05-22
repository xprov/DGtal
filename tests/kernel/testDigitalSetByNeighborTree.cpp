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
#include <vector>

#include "DGtal/base/Common.h"
#include "DGtal/kernel/SpaceND.h"
#include "DGtal/kernel/NumberTraits.h"
#include "DGtal/kernel/domains/HyperRectDomain.h"
#include "DGtal/kernel/sets/CDigitalSet.h"
#include "DGtal/kernel/sets/CDigitalSetArchetype.h"
#include "DGtal/kernel/domains/CDomain.h"
#include "DGtal/kernel/domains/CDomainArchetype.h"
#include "DGtal/kernel/sets/DigitalSetByNeighborTree.h"
#include "DGtal/kernel/sets/DigitalSetBySTLSet.h"


using namespace DGtal;
using namespace std;

typedef SpaceND<3> SpaceType;
typedef HyperRectDomain<SpaceType> Domain;
typedef SpaceType::Point Point;
typedef SpaceType::Integer Integer;
typedef DigitalSetByNeighborTree< Domain > DS;
typedef DigitalSetBySTLSet< Domain > DS_OTHER;
typedef DS::Tree Tree;
typedef DS::Node Node;
typedef DS::Direction Direction;

#define MAX NumberTraits< Integer >::max()
#define MIN NumberTraits< Integer >::min()

DS_OTHER * ds_other;


void genRandomPoints( vector<Point> & vec, int n, int max_coord )
{
  Point p;
  vec.clear();
  for ( int i=0; i<n; ++i )
    {
      for ( int d=0; d<DS::D; ++d )
        {
          p[d] = ( ( rand() % 2 ) ? 1 : -1 ) * ( rand() % max_coord );
        }
      vec.push_back( p );
    }
}

bool test_one_point( DS & ds, const Point & p )
{
  Node *n, *m;

  ds.insert( p );
  ds_other->insert( p );

  // find/create node for p
  m = ds.myTree->findNode( p, false );

  // go to 'm' stating from the origin and using only 'findNeighbor'
  Point origin( 0, 0, 0 );
  n = ds.myTree->findNode( origin, false );
  for ( int d=0; d<DS::D; ++d )
    {
      for ( int i=p[d]; i<0; ++i )
        {
          n = ds.myTree->findNeighbor( n, Direction( -(d+1) ) );
        }
      for ( int i=0; i<p[d]; ++i )
        {
          n = ds.myTree->findNeighbor( n, Direction( d+1 ) );
        }
    }
  return ( m != NULL) && ( m == n );
} 

int test_small_points( DS & ds, int n, int max_coord )
{
  int nb_ok = 0;
  vector<Point> v;
  genRandomPoints( v, n, max_coord );
  vector<Point>::const_iterator itEnd = v.end();
  for ( vector<Point>::const_iterator it = v.begin();
       it != itEnd; ++it )
    {
      nb_ok += ( test_one_point( ds, *it ) ) ? 1 : 0;
    }
  cout << "Set size : " << ds.size() << ", tree size : " << ds.myTree->size() << endl;
  return nb_ok;
}

int test_big_points( DS & ds, int n ) 
{
  int nb_ok = 0;
  vector< Point > v;
  genRandomPoints( v, n, MAX );
  vector<Point>::const_iterator itEnd = v.end();
  for ( vector<Point>::const_iterator it = v.begin();
       it != itEnd; ++it )
    {
      Point p = *it;
      ds.insert( p );
      ds_other->insert( p );
    }
  for ( vector<Point>::const_iterator it = v.begin();
       it != itEnd; ++it )
    {
      //DS::ConstIterator it_ds = ds.find( *it );
      //nb_ok += ( ( it_ds != ds.end() ) && ( *it == *it_ds ) ) ? 1 : 0;
      Node * n = ds.myTree->findNode( *it, false );
      nb_ok += ( ( n != NULL ) && ( n->getPosition() == *it ) && ( n->inside() ) );
    }
  cout << "Set size : " << ds.size() << ", tree size : " << ds.myTree->size() << endl;
  return nb_ok;
}

bool test_set_content( const DS & ds )
{
  bool ok = true;

  ok = ( ds.size() == ds_other->size() );

  if ( ok )
    {
      DS_OTHER::ConstIterator itEnd = ds_other->end();
      for( DS_OTHER::ConstIterator it = ds_other->begin() ;
          it != itEnd; ++it )
        {
          DS::ConstIterator it_ds = ds.find( *it );
          if ( ( it_ds == ds.end() ) || ( *it != *it_ds ) )
            {
              ok = false;
              break;
            }
        }
    } 
  cout << "All points in the set : " << ( ( ok ) ? "yes" : "no" ) << endl;

  if ( ok )
    {
      DS::ConstIterator itEnd = ds.end();
      for( DS::ConstIterator it = ds.begin() ;
          it != itEnd; ++it )
        {
          DS_OTHER::ConstIterator it_set = ds_other->find( *it );
          if ( ( it_set == ds_other->end() ) || ( *it != *it_set ) )
            {
              ok = false;
              break;
            }
        }
    }
  cout << "Only points in the set : " << ( ( ok ) ? "yes" : "no" ) << endl;
  return ok;
}

bool test_node_retrival( const DS & ds )
{
  int nb_ok = 0;
  int nb_points = ds_other->size();
  DS_OTHER::ConstIterator itEnd = ds_other->end();
  for( DS_OTHER::ConstIterator it = ds_other->begin() ;
      it != itEnd; ++it )
    {
      Node * n = ds.myTree->findNode( *it, false );
      if ( ( n != NULL ) && ( *it == n->getPosition() ) && ( n->inside() ) )
        {
          ++nb_ok;
        }
    } 
  cout << "Points retreived : " << nb_ok << " / " << nb_points << endl;
  return ( nb_ok == nb_points );
}

void usage( const char * name )
{
  cerr << "Usage : " << name << "  [nb_tests [max_coord]]" << endl;
}

int main( int argc, const char ** argv )
{
  srand( 42 );

  bool res = true;
  bool all_ok = true;


  DGtal::int32_t t[] =  { MIN, MIN, MIN };
  Point a ( t );
  DGtal::int32_t t2[] = { MAX, MAX, MAX };
  Point b ( t2 );

  Domain dom( a, b );
  DS ds( dom );
  ds_other = new DS_OTHER( dom );
  
  if ( ( argc > 1 ) && ( strcmp( argv[1], "-h" ) == 0 ) )
    {
      usage( argv[0] );
      exit( EXIT_SUCCESS );
    }

  int nb_tests = ( argc >= 2 ) ? atoi( argv[1] ) : 100;
  int max_coord = ( argc >=3 ) ?  atoi( argv[2] ) : 10000;
  int nb_ok;

  trace.beginBlock ( "Test small coordinates" );
  nb_ok = test_small_points( ds, nb_tests, max_coord );
  res = ( nb_ok == nb_tests );
  trace.endBlock();
  if ( res )
    {
      trace.emphase() << "Passed." << endl;
    }
  else
    {
      trace.emphase() << "Failed. ( " << nb_ok << " / " << nb_tests << " )"<< endl;
      all_ok = false;
    }


  trace.beginBlock ( "Test big coordinates" );
  nb_ok = test_big_points( ds, nb_tests );
  res = ( nb_ok == nb_tests );
  trace.endBlock();
  if ( res )
    {
      trace.emphase() << "Passed." << endl;
    }
  else
    {
      trace.emphase() << "Failed. ( " << nb_ok << " / " << nb_tests << " )"<< endl;
      all_ok = false;
    }

  trace.beginBlock ( "Test set content" );
  res = test_set_content( ds );
  trace.endBlock();
  if ( res )
    {
      trace.emphase() << "Passed." << endl;
    }
  else
    {
      trace.emphase() << "Failed." << endl;
      all_ok = false;
    }


  trace.beginBlock ( "Test node retrival" );
  res = test_node_retrival( ds );
  trace.endBlock();
  if ( res )
    {
      trace.emphase() << "Passed." << endl;
    }
  else
    {
      trace.emphase() << "Failed." << endl;
      all_ok = false;
    }


  trace.beginBlock ( "Test bounding box" );
  Point myLower, myUpper, otherLower, otherUpper;
  ds.computeBoundingBox( myLower, myUpper );
  ds_other->computeBoundingBox( otherLower, otherUpper );
  res = (myLower == otherLower) && (myUpper == otherUpper);
  trace.endBlock();
  if ( res )
    {
      trace.emphase() << "Passed." << endl;
    }
  else
    {
      trace.emphase() << "Failed." << endl;
      cout << "moi : " << myLower << " " << myUpper << endl;
      cout << "eux : " << otherLower << " " << otherUpper << endl;
      all_ok = false;
    }



  return all_ok ? 0 : 1;

}

/** @ingroup Tests **/
