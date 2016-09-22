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
#include <fstream>
#include <algorithm>
///////////////////////////////////////////////////////////////////////////////

//! [shapeGridCurveEstimator-basicIncludes]
#include "DGtal/base/Common.h"
#include "DGtal/helpers/StdDefs.h"
#include "ConfigExamples.h"
//! [shapeGridCurveEstimator-basicIncludes]

//! [shapeGridCurveEstimator-shapeIncludes]
//shape and digitizer
#include "DGtal/shapes/Shapes.h"
#include "DGtal/shapes/ShapeFactory.h"
#include "DGtal/shapes/GaussDigitizer.h"
//! [shapeGridCurveEstimator-shapeIncludes]

//! [shapeGridCurveEstimator-trackingIncludes]
//tracking grid curve
#include "DGtal/topology/helpers/Surfaces.h"
#include "DGtal/geometry/curves/GridCurve.h"
//! [shapeGridCurveEstimator-trackingIncludes]

//! [shapeGridCurveEstimator-estimationIncludes]
//length estimation knowing the shape
#include "DGtal/geometry/curves/estimation/TrueGlobalEstimatorOnPoints.h"
#include "DGtal/geometry/curves/estimation/ParametricShapeArcLengthFunctor.h"
//length estimation based on a DSS segmentation
#include "DGtal/geometry/curves/estimation/DSSLengthEstimator.h"
//! [shapeGridCurveEstimator-estimationIncludes]

#include "DGtal/io/boards/Board2D.h"

///////////////////////////////////////////////////////////////////////////////

using namespace DGtal;

int main()
{
  //shape
  typedef Flower2D<Z2i::Space> Flower; 
  Flower2D<Z2i::Space> flower(Z2i::Point(0,0), 20, 5, 5, 0);
  
  //! [shapeGridCurveEstimator-dig]
  //implicit digitization of a shape of type Flower 
  //into a digital space of type Space
  double h = 1; 
  GaussDigitizer<Z2i::Space,Flower> dig;  
  dig.attach( flower );
  dig.init( flower.getLowerBound()+Z2i::Vector(-1,-1),
            flower.getUpperBound()+Z2i::Vector(1,1), h ); 
  //! [shapeGridCurveEstimator-dig]
  
  //! [shapeGridCurveEstimator-prepareTracking]
  //Khalimsky space
  Z2i::KSpace ks;
  ks.init( dig.getLowerBound(), dig.getUpperBound(), true );
  //adjacency (4-connectivity)
  SurfelAdjacency<2> sAdj( true );
  //! [shapeGridCurveEstimator-prepareTracking]

  //! [shapeGridCurveEstimator-tracking]
  //searching for one boundary element
  Z2i::SCell bel = Surfaces<Z2i::KSpace>::findABel( ks, dig, 1000 );
  //tracking
  std::vector<Z2i::Point> boundaryPoints;
  Surfaces<Z2i::KSpace>
    ::track2DBoundaryPoints( boundaryPoints, ks, sAdj, dig, bel );
  //! [shapeGridCurveEstimator-tracking]

  //! [shapeGridCurveEstimator-instantiation]
  Z2i::Curve c;
  c.initFromVector( boundaryPoints );  
  //! [shapeGridCurveEstimator-instantiation]
  
  DGtal::Board2D aBoard( Color::White );
  aBoard << c; 

  Z2i::Curve c2;
  std::vector<Z2i::Point> v( { Z2i::Point(5,5) + Z2i::Point( 0,0 ), 
                               Z2i::Point(5,5) + Z2i::Point( 1,0 ),
                               Z2i::Point(5,5) + Z2i::Point( 2,0 ),
                               Z2i::Point(5,5) + Z2i::Point( 3,0 ),
                               Z2i::Point(5,5) + Z2i::Point( 3,1 ),
                               Z2i::Point(5,5) + Z2i::Point( 3,2 ),
                               Z2i::Point(5,5) + Z2i::Point( 3,3 ),
                               Z2i::Point(5,5) + Z2i::Point( 2,3 ),
                               Z2i::Point(5,5) + Z2i::Point( 1,3 ),
                               Z2i::Point(5,5) + Z2i::Point( 0,3 ),
                               Z2i::Point(5,5) + Z2i::Point( 0,2 ),
                               Z2i::Point(5,5) + Z2i::Point( 0,1 ),
                               Z2i::Point(5,5) + Z2i::Point( 0,0 ) } );
  c2.initFromPointsVector( v );
  aBoard << SetMode( c2.className(), "Fill" );
  aBoard << c2;

  DGtal::GridCurve<Z3i::K3> c3d;
  std::vector<Z3i::Point> v3d( { Z3i::Point( 1,0,0 ),
                               Z3i::Point( 1,1,0 ),
                               Z3i::Point( 0,1,0 ),
                               Z3i::Point( 0,1,1 ),
                               Z3i::Point( 0,0,1 ),
                               Z3i::Point( 1,0,1 )});

  c3d.initFromPointsVector( v3d );
  aBoard << SetMode( c2.className(), "" );
  aBoard << c3d;
  aBoard.saveCairo("out.png", Board2D::CairoPNG);
  
  return 0;

}

///////////////////////////////////////////////////////////////////////////////

