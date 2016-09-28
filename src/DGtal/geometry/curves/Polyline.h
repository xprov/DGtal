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

#pragma once

/**
 * @file Polyline.h
 * 
 * @author Tristan Roussillon (\c
 * tristan.roussillon@liris.cnrs.fr ) Laboratoire d'InfoRmatique en
 * Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS,
 * France
 *
 * @author Xavier Provençal (\c
 * xavier.provencal@univ-smb.fr ) Laboratoire de Mathématiques
 * LAMA (CNRS, UMR 5127), CNRS, France
 *
 *
 * @date 2016/09/27
 *
 * @brief Header file for module Polyline.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(Polyline_RECURSES)
#error Recursive header files inclusion detected in Polyline.h
#else // defined(Polyline_RECURSES)
/** Prevents recursive inclusion of headers. */
#define Polyline_RECURSES

#if !defined Polyline_h
/** Prevents repeated inclusion of headers. */
#define Polyline_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include <vector>
#include <iterator>
#include <cstddef>

#include "DGtal/base/Common.h"
#include "DGtal/helpers/StdDefs.h"
#include "DGtal/io/readers/PointListReader.h"

#include "DGtal/base/BasicFunctors.h"
#include "DGtal/base/Circulator.h"
#include "DGtal/base/ConstRangeAdapter.h"
#include "DGtal/base/ConstIteratorAdapter.h"

#include "DGtal/topology/CCellularGridSpaceND.h"
#include "DGtal/topology/KhalimskySpaceND.h"

//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{



  /////////////////////////////////////////////////////////////////////////////
  // class GridCurve
  /////////////////////////////////////////////////////////////////////////////
    /**
    * @brief Aim: describes a polygonal chain in a space of dimension n.

    @tparam TPoint a model of point.

    @snippet geometry/io/board/dgtalBoard2D-6-polyline.cpp
    
    */

  template <typename TPoint = DGtal::Z2i::Point>
  class Polyline
  {

  public: 
    typedef TPoint Point; 
  
    typedef typename std::vector<Point> Storage; 

    // ----------------------- Standard services ------------------------------
  public:

    /**
     * Destructor.
     */
    ~Polyline();

    /**
     * Default Constructor.
     * (the underlying Khalimsky space is default constructed). 
     */
    Polyline();

    /**
     * Copy constructor.
     * @param other the object to clone.
     */
    Polyline( const Polyline & other );

    /**
     * Assignment.
     * @param other the object to copy.
     * @return a reference on 'this'.
     */
    Polyline & operator=( const Polyline & other );

    // ----------------------- common ------------------------------

    /**
     * @return the style name used for drawing this object.
     */
    std::string className() const;

    /**
     * Writes/Displays the object on an output stream.
     * @param out the output stream where the object is written.
     */
    void selfDisplay ( std::ostream & out ) const;

    /**
     * Checks the validity/consistency of the object.
     * @return 'true' if the object is valid, 'false' otherwise.
     */
    bool isValid() const;

    // ----------------------- streams ------------------------------

    /**
     * Init.
     * @param in any input stream,
     */
    void initFromVectorStream(std::istream & in );

    /**
     * Outputs the grid curve to the stream @a out.
     * @param out any output stream,
     */
    void writeVectorToStream( std::ostream & out ) const;

    // ----------------------- Initializations ------------------------------

    /**
     * Init from a STL vector of points.
     * @param aVectorOfPoints the vector containing a sequence of grid points (digital coordinates).
     * @see initFromPointsRange
     */
    void initFromPointsVector( const std::vector<Point>& aVectorOfPoints ); 



    // ----------------------- open/closed ------------------------------


    /**
     * Checks whether the grid curve is open or closed. By definition, a
     * Polyline is closed iff its last point is equal to its first.
     *
     * @return 'true' if grid curve is closed, 'false' otherwise
     */
    bool isClosed() const;

    /**
     * @return 'true' if the grid curve is not closed, 'false' otherwise
     * @see isClosed
     */
    bool isOpen() const;

    // ----------------------- container interface ------------------------------

    typedef typename Storage::const_iterator const_iterator; 
    typedef typename Storage::const_iterator ConstIterator; 
    typedef typename Storage::const_reverse_iterator const_reverse_iterator; 
    typedef typename Storage::const_reverse_iterator ConstReverseIterator; 

    /**
     * @return begin iterator on points
     */
    ConstIterator begin() const; 

    /**
     * @return end iterator on points
     */
    ConstIterator end() const; 
     
    /**
     * @return reverse begin iterator on points
     */
    ConstReverseIterator rbegin() const; 

    /**
     * @return reverse end iterator on points
     */
    ConstReverseIterator rend() const; 

    /**
     * @return the list of points.
     */
    const Storage& getPoints() const;

    /**
     * @return last points
     */
    Point back() const; 

    /**
     * Back insertion of @e aPoint
     * @param aPoint any point
     * @see pushBack
     * NB: this alias is kept for STL compliance
     */
    void push_back(const Point& aPoint); 

    /**
     * Back insertion of @e aPoint
     * @param aPoint any signed cell
     */
    void pushBack(const Point& aPoint); 

    /**
     * @return number of points
     */
    typename Storage::size_type size() const; 

    // ------------------------- private Datas --------------------------------
  private:
    /**
     * list of points
     */
    Storage myPoints;


    // ------------------------- Public Datas --------------------------------
  public:



    // ------------------------- Internal --------------------------------
  private:

    
    // ------------------------- inner classes --------------------------------

  public: 


  }; // end of class Polyline



  /**
   * Overloads 'operator<<' for displaying objects of class 'Polyline'.
   * @param out the output stream where the object is written.
   * @param object the object of class 'Polyline' to write.
   * @return the output stream after the writing.
   */
  template<typename TPoint>
  std::ostream&
  operator<< ( std::ostream & out, const Polyline<TPoint> & object );


} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions/methods.
#include "DGtal/geometry/curves/Polyline.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined Polyline_h

#undef Polyline_RECURSES
#endif // else defined(Polyline_RECURSES)

