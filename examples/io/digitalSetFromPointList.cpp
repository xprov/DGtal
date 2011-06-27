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
 * @file digitalSetFromVol.cpp
 * @ingroup Examples
 * @author Bertrand Kerautret (\c kerautre@loria.fr )
 * LORIA (CNRS, UMR 7503), University of Nancy, France
 *
 * @date 2011/04/01
 *
 * An example file named digitalSetFromPointList.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <QtGui/qapplication.h>
#include "DGtal/base/Common.h"
#include "DGtal/io/readers/PointListReader.h"
#include "DGtal/io/3dViewers/DGtalQGLViewer.h"
#include "DGtal/helpers/StdDefs.h"
#include "ConfigExamples.h"

///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;

///////////////////////////////////////////////////////////////////////////////

int main( int argc, char** argv )
{
  std::string inputFilename = examplesPath + "samples/pointList3d.pl"; 
  QApplication application(argc,argv);
  DGtalQGLViewer viewer;
  viewer.show();     
  // Importing the 3d set of points  contained with the default index (0, 1, 2);
  vector<Z3i::Point> vectPoints=  PointListReader<Z3i::Point>::getPointsFromFile(inputFilename); 
  for(unsigned int i=0; i<vectPoints.size();i++){
    viewer << vectPoints.at(i);
  }
  
  // Importing the 3d set of points with another index definition  (0, 2, 1);
  vector<unsigned int> vPos;
  vPos.push_back(0);
  vPos.push_back(2);
  vPos.push_back(1);
  vectPoints=  PointListReader<Z3i::Point>::getPointsFromFile(inputFilename, vPos); 
  viewer<< CustomColors3D(QColor(255,0,0), QColor(255,0,0)); 
  for(unsigned int i=0; i<vectPoints.size();i++){
    viewer << vectPoints.at(i);
  }
  
  viewer   << DGtalQGLViewer::updateDisplay;
  return application.exec();
}
///////////////////////////////////////////////////////////////////////////////
