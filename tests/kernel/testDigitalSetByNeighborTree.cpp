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

#include "DGtal/helpers/StdDefs.h"

#include "DGtal/io/boards/Board2D.h"


using namespace DGtal;
using namespace std;


int main()
{
  typedef SpaceND<4> Space4Type;
  typedef HyperRectDomain<Space4Type> Domain;
  typedef Space4Type::Point Point;

  bool res;
  res = true;
  DGtal::int32_t t[] =  { 1, 2, 3 , 4};
  Point a ( t );
  DGtal::int32_t t2[] = { 5, 5, 3 , 5};
  Point b ( t2);
  trace.beginBlock ( "Wouhou !" );
  Domain dom;
  DigitalSetByNeighborTree< Domain > ds( dom );

  trace.endBlock();
  trace.emphase() << ( res ? "Passed." : "Error." ) << endl;
  return res ? 0 : 1;
}

/** @ingroup Tests **/
