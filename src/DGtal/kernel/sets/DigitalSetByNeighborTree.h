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
 * @file DigitalSetByNeighborTree.h
 * @author Xavier Provençal (\c xavier.provencal@univ-savoie.fr )
 * Laboratoire de Mathematiques (CNRS, UMR 5807), Universitée de Savoie, France
 *
 * @date 21/12/2012
 *
 * This file is part of the DGtal library.
 */

#if defined(DigitalSetByNeighborTree_RECURSES)
#error Recursive header files inclusion detected in DigitalSetByNeighborTree.h
#else // defined(DigitalSetByNeighborTree_RECURSES)
/** Prevents recursive inclusion of headers. */
#define DigitalSetByNeighborTree_RECURSES

#if !defined DigitalSetByNeighborTree_h
/** Prevents repeated inclusion of headers. */
#define DigitalSetByNeighborTree_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include "DGtal/base/Common.h"
//////////////////////////////////////////////////////////////////////////////


#define NB_SONS POWER<2, Space::dimension >::RET
#define NB_NEIGHBORS 2 * Space::dimension

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // template class DigitalSetByNeighborTree
  /**
    Description of template class 'DigitalSetByNeighborTree' <p>

    \brief Aim: A container class for storing sets of digital points
    within some given domain.

    This container designed in order to optimize the insertion or test the
    inclusions of points which are neighbors of a given point. This data
    structure has been originally designed in order to test is a path is
    self-intersecting in a time linear to its length. See the follownig citation
    for more details :

    S. Brlek, M. Koskas, X. Provençal, A linear time and space algorithm for
    detecting path intersection in image Zd, Theoretical Computer Science (TCS)
    412, 2011, p. 4841-4850. 

    Note : this set structure is designed for sparse set representation. Indeed,
    let \c d be the dimension of domain, each point is represented by a note
    that includes (2d)(2^d)+1 pointers.

    Model of CDigitalSet.
   */
  template <typename TDomain>
  class DigitalSetByNeighborTree
  {
  public:
    typedef TDomain Domain;
    typedef DigitalSetByNeighborTree<Domain> Self;
    typedef typename Domain::Space Space;
    typedef typename Domain::Dimension Dimension;
    typedef typename Domain::Point Point;
    typedef typename Domain::Vector Vector;
    typedef typename Domain::Size Size;

    struct SConstIterator;
    typedef struct SConstIterator Iterator;
    typedef struct SIterator ConstIterator;

    // ---------------------- New types definition ---------------------------

  private:



    // Little template trick in order to compute integer's exponential at
    // compile time.
    template < int N, int K >
      struct POWER
        {
          enum { RET = N * POWER<N,K-1>::RET };
        };

    template < int N >
      struct POWER<N,0>
        {
          enum { RET = 1 };
        };

    struct SNode
      {
        struct SNode * father;
        struct SNode * sons[ NB_SONS ];
        struct SNode * neighbors[ NB_NEIGHBORS ];
      };
    typedef struct SNode Node;

    class Tree 
    {
    public :
      Tree();
      Node roots[ NB_SONS ];
    };

    // ----------------------- Standard services ------------------------------
  public:

    /**
     * Destructor.
     */
    ~DigitalSetByNeighborTree();

    /**
     * Constructor.
     * Creates the empty set in the domain [d].
     *
     * @param d any domain.
     */
    DigitalSetByNeighborTree( const Domain & d );

    /**
     * Copy constructor.
     * @param other the object to clone.
     */
    DigitalSetByNeighborTree ( const DigitalSetByNeighborTree & other );

    /**
     * Assignment.
     * @param other the object to copy.
     * @return a reference on 'this'.
     */
    DigitalSetByNeighborTree & operator= ( const DigitalSetByNeighborTree & other );

    /**
     * @return the embedding domain.
     */
    const Domain & domain() const;

    // ----------------------- Standard Set services --------------------------
  public:

    /**
     * @return the number of elements in the set.
     */
    Size size() const;

    /**
     * @return 'true' iff the set is empty (no element).
     */
    bool empty() const;
     
    /**
     * Adds point [p] to this set.
     *
     * @param p any digital point.
     * @pre p should belong to the associated domain.
     */
    void insert( const Point & p );

    /**
     * Adds the collection of points specified by the two iterators to
     * this set.
     *
     * @param first the start point in the collection of Point.
     * @param last the last point in the collection of Point.
     * @pre all points should belong to the associated domain.
     */
    template <typename PointInputIterator>
    void insert( PointInputIterator first, PointInputIterator last );

    /**
     * Adds point [p] to this set if the point is not already in the
     * set.
     *
     * @param p any digital point.
     *
     * @pre p should belong to the associated domain.
     * @pre p should not belong to this.
     */
    void insertNew( const Point & p );

    /**
     * Adds the collection of points specified by the two iterators to
     * this set.
     *
     * @param first the start point in the collection of Point.
     * @param last the last point in the collection of Point.
     *
     * @pre all points should belong to the associated domain.
     * @pre each point should not belong to this.
     */
    template <typename PointInputIterator>
    void insertNew( PointInputIterator first, PointInputIterator last );

    /**
     * Removes point [p] from the set.
     * 
     * @param p the point to remove.
     * @return the number of removed elements (0 or 1).
     */
    Size erase( const Point & p );

    /**
     * Removes the point pointed by [it] from the set.
     * 
     * @param it an iterator on this set.
     * Note: generally faster than giving just the point.
     */
    void erase( Iterator it );

    /**
     * Removes the collection of points specified by the two iterators from
     * this set.
     *
     * @param first the start point in this set.
     * @param last the last point in this set.
     */
    void erase( Iterator first, Iterator last );

    /**
     * Clears the set.
     * @post this set is empty.
     */
    void clear();

    /**
     * @param p any digital point.
     */
    ConstIterator find( const Point & p ) const;

    /**
     * @param p any digital point.
     * @return an iterator pointing on [p] if found, otherwise end().
     */
    Iterator find( const Point & p );

    /**
     * @return a const iterator on the first element in this set.
     */
    ConstIterator begin() const;

    /**
     * @return a const iterator on the element after the last in this set.
     */
    ConstIterator end() const;

    /**
     * @return an iterator on the first element in this set.
     */
    Iterator begin();

    /**
     * @return a iterator on the element after the last in this set.
     */
    Iterator end();

    /**
     * set union to left.
     * @param aSet any other set.
     */
    DigitalSetByNeighborTree<Domain> & operator+=
    ( const DigitalSetByNeighborTree<Domain> & aSet );

    // ----------------------- Other Set services -----------------------------
  public:
    
    /**
     * Computes the bounding box of this set.
     *
     * @param lower the first point of the bounding box (lowest in all
     * directions).
     * @param upper the last point of the bounding box (highest in all
     * directions).
     */
    void computeBoundingBox( Point & lower, Point & upper ) const;


    // ----------------------- Interface --------------------------------------
  public:

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

    // ------------------------- Protected Datas ------------------------------
  protected:

    /**
     * The associated domain;
     */
    const Domain & myDomain;

    /**
     * The container storing the points of the set.
     */
    Tree * myTree;


  public:
    


    // --------------- CDrawableWithBoard2D realization ---------------------
  public:

    /**
     * Default drawing style object.
     * @return the dyn. alloc. default style for this object. 
     */
    //DrawableWithBoard2D* defaultStyle( std::string mode = "" ) const;

    /**
     * @return the style name used for drawing this object.
     */
    std::string className() const;


    // ------------------------- Hidden services ------------------------------
  protected:

    /**
     * Default Constructor.
     * Forbidden since a Domain is necessary for defining a set.
     */
    DigitalSetByNeighborTree();

  private:


    // ------------------------- Internals ------------------------------------
  private:


  }; // end of class DigitalSetByNeighborTree


  /**
   * Overloads 'operator<<' for displaying objects of class
   * 'DigitalSetByNeighborTree'.
   * @param out the output stream where the object is written.
   * @param object the object of class 'DigitalSetByNeighborTree' to write.
   * @return the output stream after the writing.
   */
  template <typename Domain>
  std::ostream&
  operator<< ( std::ostream & out, const DigitalSetByNeighborTree<Domain> & object );

} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/kernel/sets/DigitalSetByNeighborTree.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined DigitalSetByNeighborTree_h

#undef DigitalSetByNeighborTree_RECURSES
#endif // else defined(DigitalSetBySTLSet_RECURSES)
