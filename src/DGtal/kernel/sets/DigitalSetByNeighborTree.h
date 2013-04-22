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
#include <iostream>
#include <vector>
#include "DGtal/base/Common.h"
#include "DGtal/base/ExpressionTemplates.h"
//////////////////////////////////////////////////////////////////////////////


// #define D Space::dimension
// #define NB_SONS DigitalSetByNeighborTree<Domain>::POWER< 2, Space::dimension >::RET
// #define NB_NEIGHBORS ( 2 * Space::dimension )

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
    typedef typename Domain::Integer TInteger;
    typedef typename NumberTraits< TInteger >::SignedVersion Integer;
    typedef typename NumberTraits< TInteger >::UnsignedVersion UnsignedInteger;

    struct SConstIterator;
    typedef struct SConstIterator Iterator;
    typedef struct SIterator ConstIterator;

    // ---------------------- New types definition ---------------------------

  public:



    class Node;
    class Root;
    class Direction;
    class TypeOfSon;

    class Tree 
    {

      // ---------------------- New types definition ---------------------------
    public :
      Tree();

      /**
       * Returns the adress if the neighbor of 'n' in direction 'd'. If no such
       * node exist in the tree then it is created.
       *
       * @param n the adress of the node.
       * @param d the direction of the neighbor relatively to 'n'.
       * @returns the adress of the neighbor.
       */
      Node * findNeighbor( Node * n, Direction d );

      /**
       * Returns the adress of a given son of the node. In the case where such a
       * node does not exist yet, a new node is created and added to the sons.
       *
       * @param father the father node.
       * @param type the tyde of the requested son.
       */
      Node * getSon( Node * father, TypeOfSon type );

      /**
       * Located the node representing a point p. If no such node exists in the
       * tree, then NULL is returned.
       *
       * @param p a point of the domain.
       * @returns the adress of the node at position p or NULL if there are
       * no such node in the tree.
       */
      Node * findNode( const Point & p );

      void selfDisplay( std::ostream & out, int indent = 0 ) const;

    public : // TODO : this should be protected.

      /**
       * In order to represent a d dimensional space, a tree with 2^d roots is
       * built. More precisely, we build one tree per hyper-octant but we link
       * all the roots together in order to have a single tree.
       */
      Root roots[ DigitalSetByNeighborTree::nb_sons ];

    protected :

      // Nodes are allocated by the tree and stored in recondary structure.
      std::vector<Node * > myNodes;

      // ------------------- static arithmetic services -----------------
      
    public :

      /**
       * Return the depth of a node whose deepest coordinate is 'n'.
       * 
       * Note that the roots of the tree encode all nodes with coordinates in
       * {0,-1} so that depth is 0 for both 0 and -1. For other negative
       * numbers, the depth is log_2(-n+1) while the depth of a positive
       * number is simply log_2(n).
       *
       * @param n a signed Integer 
       * @returns the minimal depth to encode n.
       */
      static UnsignedInteger depth( Integer n );


      /**
       * Returns the depth the node that encodes a point p.
       *
       * @param p a point.
       * @returns the depth of p.
       */
      static UnsignedInteger depth( Point p );

      // ------------------------ Private servies -------------------
    protected :

      /**
       * Return the adress of the root of the subtree containing p.
       *
       * @param p a point.
       * @returns the adress of the root.
       */
      Root * getRoot( const Point & p );

    };


    /**
     * A node may have up do 2^d sons. In order to identify the sons a node,
     * each of them is designated by a list of bits. These bits allows to compute
     * de coordinates of the son from the coordinates of its father.
     *
     * Let 'S' be a son the son of type 'b = b_1b_2...b_d' of the node 'F'. Let '(x_1, x_2,
     * ..., x_d )' be the coordinates of 'F', then the i-th coordinate of 'S' is
     * '2*x_i + b_i'.
     *
     * These lists of bits are stored using a single instance of 'unsigned int'
     * which limits the dimension of the Domain to 32.
     */
    class TypeOfSon
      {
    public :
        TypeOfSon() : type(0) {}
        TypeOfSon( unsigned int aType ) : type(aType) {}
        TypeOfSon( const TypeOfSon &other ) : type( other.type ) {}
        ~TypeOfSon(){}
        TypeOfSon & operator=( const TypeOfSon & other ) 
          {
            type = other.type;
            return *this;
          }
        bool operator==( const TypeOfSon & other ) const
          {
            return type == other.type;
          }
        bool operator!=( const TypeOfSon & other ) const
          {
            return type != other.type;
          }

        unsigned intValue() const
          {
            return type;
          }

        // Returns the value of the n-th bit
        bool getBit( int n ) const
          {
            //return ( type & ( 1 << ( D - n - 1 ) ) );
            return ( type & ( 1 << n ) );
          }

        // Flips the value of the n-th bit
        void flipBit( int n )
          {
            //type ^= ( 1 << ( D - n - 1 ) );
            type ^= ( 1 << n );
          }

          
        void selfDisplay( std::ostream & out, int indent = 0 ) const;

    public :
        unsigned int type;
      };

    /**
     * A Direction is an elementary step in the digital space. In other words, a
     * vector of the form +/- e_i. These steps are conviniently described unsing
     * a signed int.
     *
     * Class Direction defines a bijection between directions and {0, 1, ...,
     * 2^d-1} that is used to index the ``neighbors`` array in the node
     * structure.
     *
     * This bijection sums un to order the neighbors in the following way :
     * e_1, -e_1, e_2, -e_2, ..., e_d, -e_d.
     */
    class Direction 
      {
    public :
        Direction( int dir ) : d( dir ) {}
        ~Direction() {}
        int intValue() const
          {
            return (d>0) ? (d-1)*2 : (-2*d)-1;
          }
        int axis( ) const 
          {
            return intValue()/2;
          }
        bool positive() const 
          {
            return d>0;
          }
        bool negative() const 
          {
            return d<0;
          }
        int d;
      };

    /**
     * The tree structure that is built here is a radix-tree that encodes the
     * coordinates of the points in the digital set, bit per bit. This tree is
     * enhanced with neighboring links that allow to go from a point X to it's
     * neighbor X' = X+e_i in constant time.
    
     * In order to represent a digital set, a tree is built dynamically. The
     * digital set is represented the following way : 
     *  - If there is no node coding a point X, then X is not in the set.
     *  - If there is a node coding a point X, then the boolean 'inTheSet'
     *    indicates if the point is in the set.
    
     * In dimension d, a node may have up to '2^d' sons since the points have
     * 'd' coordinages. There is also up to '2*d' neighbornigs links since a
     * piont 'X' has '2*d' neighbors of the form 'X +/- e_i'.
     */
    class Node
      {
    public :

      friend class Tree;

      // ----------------------- Standard services ------------------------------
      Node() {}
      void initNode( Node * father, TypeOfSon type );


      /**
       * Acces and maybe create a neighbor.
       *
       * @param d the direction of the neighbor
       */
      Node * findNeighbor( Direction d );

      /**
       * Sets a neighbor.
       *
       * @param id the number identifying the neighbor
       * @param n  the adress of the neighbor
       */
      void setNeighbor( int id, Node * n );

      /**
       * Gets a neighbor.
       *
       * @param id the number identifying the neighbor.
       * @return the adress the neighbor.
       */
      Node * getNeighbor( int id ) const;


      // ------------------------- Protected Datas ------------------------------
      
    protected :

      Node * myFather;
      Node * sons[ DigitalSetByNeighborTree::nb_sons ];
      Node * neighbors[ DigitalSetByNeighborTree::nb_neighbors ];
      bool inTheSet;
      TypeOfSon myType;

      // ----------------------- Interface --------------------------------------

    public :

      Point getPosition() const;

      void displayPos( std::ostream & out ) const;


      /**
       * Writes/Displays the object on an output stream.
       * @param out the output stream where the object is written.
       */
      void selfDisplay( std::ostream & out, int indent = 0 ) const;

      };

    class Root : public Node
      {
    public :
        Root( ) {}
        void initRoot( unsigned int aTypeOfRoot );
        unsigned int typeOfRoot;
        void selfDisplay( std::ostream & out, int indent = 0 ) const;
        Point getPosition() const;
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

    static const int D = Space::dimension;
    static const int nb_sons = DGtal::POW< 2, D >::VALUE;
    static const int nb_neighbors = 2*D;

    // ----------------------- Static arithmetic services -----------------------------
  public:


    // ----------------------- Interface --------------------------------------
  public:

    /**
     * Writes/Displays the object on an output stream.
     * @param out the output stream where the object is written.
     */
    void selfDisplay ( std::ostream & out ) const;

    void displayAllNodes( std::ostream & out ) const;

    /**
     * Checks the validity/consistency of the object.
     * @return 'true' if the object is valid, 'false' otherwise.
     */
    bool isValid() const;

    // ------------------------- Protected Datas ------------------------------
  public:

    /**
     * The associated domain;
     */
    const Domain & myDomain;

    /**
     * The container storing the points of the set.
     */
    Tree myTree;


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


    template <typename Domain>
      friend std::ostream&
      operator<< ( std::ostream & out, const DigitalSetByNeighborTree<Domain> & object );


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
  operator<< ( std::ostream & out, const DigitalSetByNeighborTree<Domain> & object )
    {
      object.selfDisplay( out );
      return out;
    }

} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/kernel/sets/DigitalSetByNeighborTree.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined DigitalSetByNeighborTree_h

#undef DigitalSetByNeighborTree_RECURSES
#endif // else defined(DigitalSetBySTLSet_RECURSES)
