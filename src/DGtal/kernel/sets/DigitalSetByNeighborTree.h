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
#include "DGtal/kernel/NumberTraits.h"
//////////////////////////////////////////////////////////////////////////////



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
public :
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


    // ---------------------- New classes definition ---------------------------

  public:



    class Tree;
    class Node;
    class Root;
    class Direction;
    class TypeOfSon;
    class Iterator;
    class ConstIterator;

    class Iterator
      {
    public :
      typedef Iterator Self;
      typedef Point Value;
      typedef Value* Pointer;
      typedef Value& Reference;

      // --------------------------- std types ------------------------
      typedef Value value_type;
      typedef Pointer pointer;
      typedef Reference reference;
      typedef std::forward_iterator_tag iterator_category;

    public :

      // Copy constructor
      // @param other the object to clone.
      Iterator ( const Iterator & other ) : it( other.it ), myTree( other.myTree ) {}

      // Default constructor
      Iterator () {}

      // Constructor from Tree::Iterator 
      // @param other an iterator on a Tree.
      Iterator ( const typename Tree::Iterator & other, Tree * t ) : it( other ), myTree(t) {}

      // The point is constructed on request because the node do not remember
      // their coordinates. Thus this call has time complexity in 
      // O( log( max( coordinates ) ) )
      Point operator* () const
        {
          return (*it)->getPosition();
        }

      Self & operator= ( const Self & other )
        {
          it = other.it;
          myTree = other.myTree;
          return *this;
        }

      bool operator== ( const Iterator & other ) const
        {
          return ( ( myTree == other.myTree) && ( this->it == other.it ) );
        }

      bool operator!= ( const Iterator & other ) const
        {
          return ( ( myTree != other.myTree) || ( this->it != other.it ) );
        }

      Iterator & operator++ () //préfix
        {
          // iterates on the nodes of the tree but consider only those that are
          // inside the set.
          ++it;
          typename Tree::Iterator itEnd = myTree->end();
          while ( ( it != itEnd ) && ( ! (*it)->inside() ) )
            {
              ++it;
            }
          return *this;
        }

      Iterator operator++ ( int ) //postfix
        {
          Iterator i( *this );
          // iterates on the nodes of the tree but consider only those that are
          // inside the set.
          ++it;
          typename Tree::Iterator itEnd = myTree->end();
          while ( ( it != itEnd ) && ( ! (*it)->inside() ) )
            {
              ++it;
            }
          return i;
        }

      friend class ConstIterator;
      friend class DigitalSetByNeighborTree;

    protected :
      Tree * myTree;
      typename Tree::Iterator it;

      };

    class ConstIterator
      {
    public :

      // default constructor, does nothing.
      ConstIterator() {}

      // Constructor from Tree::ConstIterator
      ConstIterator ( const typename Tree::ConstIterator & other, Tree * t ) 
        : it( other ), myTree(t) {}

      // Constructor from Tree::Iterator
      ConstIterator ( const typename Tree::Iterator & other, Tree * t ) 
        : it( other ), myTree( t ) {}

      // copy constructor
      ConstIterator ( const ConstIterator & other ) : 
        it( other.it ), myTree( other.myTree )  {}

      // copy constructor from Iterator
      ConstIterator ( const Iterator & other ) 
        : it( other.it ), myTree( other.myTree ) {}

      // O( log( max( coordinates ) ) )
      Point operator* () const
        {
          return (*it)->getPosition();
        }

      ConstIterator & operator= ( ConstIterator & other )
        {
          it = other.it;
          myTree = other.myTree;
          return *this;
        }

      ConstIterator & operator= ( Iterator & other )
        {
          it = other.it;
          myTree = other.myTree;
          return *this;
        }

      bool operator== ( const ConstIterator & other ) const
        {
          return ( ( myTree == other.myTree ) && ( this->it == other.it ) );
        }

      bool operator!= ( const ConstIterator & other ) const
        {
          return ( ( myTree != other.myTree ) || ( this->it != other.it ) );
        }

      ConstIterator & operator++ () //préfix
        {
          ++it;
          typename Tree::Iterator itEnd = myTree->end();
          while ( ( it != itEnd ) && ( ! (*it)->inside() ) )
            {
              ++it;
            }
          return *this;
        }

      ConstIterator operator++ ( int ) //postfix
        {
          ConstIterator i( this );
          ++it;
          typename Tree::Iterator itEnd = myTree->end();
          while ( ( it != itEnd ) && ( ! (*it)->inside() ) )
            {
              ++it;
            }
          return i;
        }

      ConstIterator & operator-- () //préfix
        {
          --it;
          typename Tree::Iterator itBegin = myTree->begin();
          while ( ( it != itBegin ) && ( ! (*it)->inside() ) )
            {
              --it;
            }
          return *this;
        }

      ConstIterator operator-- ( int ) //postfix
        {
          ConstIterator i( this );
          --it;
          typename Tree::Iterator itBegin = myTree->begin();
          while ( ( it != itBegin ) && ( ! (*it)->inside() ) )
            {
              --it;
            }
          return i;
        }

      friend class DigitalSetByNeighborTree;

    protected :
      typename Tree::ConstIterator it;
      Tree * myTree;
      };

    class Tree 
    {

      // ---------------------- New types definition ---------------------------

  public :
      typedef std::vector<Node*> Container;

      class Iterator
        {

      public :

        typedef std::forward_iterator_tag iterator_category;

        Iterator()
          {
            n = NULL;
          } 

        Iterator( Node * aNode ) 
          {
            n = aNode;
            if ( n != NULL )
              {
                pos = n->getPosition();
              }
          } 
        Iterator( const Iterator & other )
          : n( other.n ), pos( other.pos )
          { } 

        Node * operator* () const
          {
            return n;
          }

        Self & operator= ( const Self & other )
          {
            n = other.n;
            pos = other.pos;
            return *this;
          }

        bool operator== ( const Iterator & other ) const
          {
            return ( n == other.n );
          }

        bool operator!= ( const Iterator & other ) const
          {
            return ( n != other.n );
          }


        Iterator & operator++ () //préfix
          {
            n = n->next( pos );
            return *this;
          }

        Iterator operator++ ( int ) //postfix
          {
            Iterator i( *this );
            n = next( pos );
            return i;
          }

        friend class Tree::ConstIterator;

      protected :

        Node * n;
        Point pos;
        };

      class ConstIterator
        {

      public :

        typedef std::forward_iterator_tag iterator_category;

        ConstIterator()
          {
            n = NULL;
          }

        ConstIterator( Node * aNode ) 
          {
            n = aNode;
            if ( n != NULL )
              {
                pos = n->getPosition();
              }
          } 

        ConstIterator( const ConstIterator & other )
          : n( other.n ), pos( other.pos )
          { } 

        ConstIterator( const Tree::Iterator & other )
          : n( other.n ), pos( other.pos )
          { } 

        const Node * operator* () const
          {
            return n;
          }

        Self & operator= ( const Self & other )
          {
            n = other.n;
            pos = other.pos;
            return *this;
          }

        bool operator== ( const ConstIterator & other ) const
          {
            return ( n == other.n );
          }

        bool operator!= ( const ConstIterator & other ) const
          {
            return ( n != other.n );
          }

        ConstIterator & operator++ () //préfix
          {
            n = n->next( pos );
            return *this;
          }

        ConstIterator operator++ ( int ) //postfix
          {
            ConstIterator i( *this );
            n = n->next( pos );
            return i;
          }

      protected :

        Node * n;
        Point pos;
        };

      // ---------------------- Standard services ------------------------------
    public :
      Tree();
      ~Tree();

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
       * tree then, depending on the boolean 'add' either new nodes are created
       * in order to add the specified point to the structure, or NULL is
       * returned.
       *
       * @param p a point of the domain.
       * @param add indicate if new nodes should be created.
       * @returns the adress of the node at position p or NULL if add is false
       * and there are no such node in the tree.
       */
      Node * findNode( const Point & p, bool add );

      /**
       * Add a point p to the tree.
       *
       * @param p a point of the domain.
       * @returns true if a new point was added, false if it was already in.
       */
      bool addPoint( const Point & p );

      /**
       * Remove a point p to the tree.
       *
       * Note : Does not remove the node, no memory is freed when removing a
       * point this way.
       *
       * @param p a point of the domain.
       * @returns true if a point was removed.
       */
      bool removePoint( const Point & p );

      /**
       * Given the adress of a node in the Tree, returns a ConstIterator on that
       * Node.
       *
       * @param n a pointer to a Node.
       * @returns a ConstIterator on n, this->end() if n is NULL.
       */
      ConstIterator find( const Node * n ) const;

      /**
       * Given the adress of a node in the Tree, returns a Iterator on that
       * Node.
       *
       * @param n a pointer to a Node.
       * @returns an Iterator on n or this->end() if n is NULL.
       */
      Iterator find( Node * n );

      /**
       * Given a Point, returns a ConstIterator on the corresponding Node.
       *
       * @param p a point.
       * @returns a ConstIterator on n or this->end() if p is not in the Tree.
       */
      ConstIterator find( const Point & p ) const;

      /**
       * Given a Point, returns a Iterator on the corresponding Node.
       *
       * @param p a point.
       * @returns a ConstIterator on n or this->end() if p is not in the Tree.
       */
      Iterator find( const Point & p );

      void selfDisplay( std::ostream & out, int indent = 0 ) const;

      /**
       * Returns the number of nodes in the tree. Which is NOT the number of
       * points in the set represented by this tree. Note also that the 2^D
       * roots are not considered since they are statically created by the
       * constructor.
       *
       * @returns the number of nodes in the tree.
       */
      Size size() const;

      /**
       * Computes the bounding box of this set.
       *
       * @param lower (returns) the first point of the bounding box (lowest in
       * all directions).
       * @param upper (returns) the last point of the bounding box (highest in
       * all directions).
       */
      void computeBoundingBox( Point & lower, Point & upper ) const;

    protected :

      /**
       * Recursively computes the bounding box of this set.
       *
       * @param lower (returns) the first point of the bounding box (lowest in
       * all directions).
       * @param upper (returns) the last point of the bounding box (highest in
       * all directions).
       * @param n the adress of the actual node.
       * @param position the position of the node n
       */ 
      void computeBoundingBoxRec( Point & lower, Point & upper, const Node * n, 
                                  Point & position ) const;

    protected :

      /**
       * In order to represent a d dimensional space, a tree with 2^d roots is
       * built. More precisely, we build one tree per hyper-octant but we link
       * all the roots together in order to have a single tree.
       */
      Root myRoots[ DigitalSetByNeighborTree::nb_sons ];

      // Nodes are allocated by the tree and stored in recondary structure.
      Container myNodes;

      // The number of nodes created... for debugging purpose
      Size nb_nodes;

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
      static UnsignedInteger depth( const Point & p );

      /**
       * Modify a Point in order to get to position of one of it's sons in the
       * Tree structure.
       *
       * @param pos the potision of the node
       * @type the type of the son
       */
      static void goDown( Point & pos, TypeOfSon type );

      /**
       * Modify a Point in order to get to position of it's father in the Tree
       * structure
       *
       * @param pos the potision of the node
       */
      static void goUp( Point & pos );

      // ------------------------ Private servies -------------------
    protected :

      /**
       * Return the adress of the root of the subtree containing p.
       *
       * @param p a point.
       * @returns the adress of the root.
       */
      Root * getRoot( const Point & p );

      // ------------------------ Iterator services -------------------
      
    public :

      /**
       * Tree begin() iterator. Always point on the node at ( 0, 0, ..., 0 .)
       *
       * @return an Iterator on the first node of a tree.
       **/
      Iterator begin();

      /**
       * Tree end() iterator.
       *
       * @return an Iterator after the last node of a tree.
       **/
      Iterator end();

      /**
       * iTree begin() const iterator.
       *
       * @return an ConstIterator on the first node of a tree.
       **/
      ConstIterator begin() const;

      /**
       * Tree end() const iterator.
       *
       * @return a ConstIterator after the last element of a tree.
       **/
      ConstIterator end() const;

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
            return ( type & ( 1 << n ) );
          }

        // Flips the value of the n-th bit
        void flipBit( int n )
          {
            type ^= ( 1 << n );
          }

        // display function, for debugging purpose...
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

      /**
       * Determines if the node represent a point that is in the set.
       *
       * @return true if in the set, false otherwise.
       */
      bool inside( ) const;

      /**
       * Add the point to the set.
       */
      void addToSet( );

      /**
       * Remove the point from the set.
       */
      void removeFromSet( );

      /**
       * Test the node is a root, no dynamic_cast involved.
       * @returns true if it is a node, false otherwise.
       */
      bool isRoot() const;

      /**
       * Perform a postfix course of the tree.
       *
       * @returns a pointer to the next node.
       */
      Node * next( Point & pos ) const;



      // ------------------------- Protected Datas ------------------------------
      
    protected :

      Node * myFather;
      Node * mySons[ DigitalSetByNeighborTree::nb_sons ];
      Node * myNeighbors[ DigitalSetByNeighborTree::nb_neighbors ];
      bool inTheSet;
      TypeOfSon myType;

      // ----------------------- Interface --------------------------------------

    public :

      Point getPosition() const;


      /**
       * Writes/Displays the object on an output stream.
       * @param out the output stream where the object is written.
       */
      void selfDisplay( std::ostream & out, int indent = 0 ) const;

      friend class Tree::Iterator;
      friend class Tree::ConstIterator;

      };

    class Root : public Node
      {
    public :
        Root( ) {}
        void initRoot( unsigned int aTypeOfRoot );
        void selfDisplay( std::ostream & out, int indent = 0 ) const;
        Point getPosition() const;

    public :
        Point myPosition;
        Root * nextRoot;
        friend class Tree;
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


    // ----------------------- Specific Set services ---------------------------

    /**
     * Check if a given neighbor of a point is in the set.
     *
     * @param it and iterator pointing on a point of the set.
     * @param d a direction.
     * @returns true if the designated neighbor is in the set, false otherwise.
     */
    bool isNeighborInside( Iterator it, Direction d );

    /**
     * Check if a given neighbor of a point is in the set.
     *
     * @param n the adress of a node of the tree.
     * @param d a direction.
     * @returns true if the designated neighbor is in the set, false otherwise.
     */
    bool isNeighborInside( Node * n, Direction d );

    /**
     * Adds the neighbor of *it, in direction d, to the set.
     *
     * @param it and iterator pointing on a point of the set.
     * @returns a pair, with its member pair::first set to an iterator pointing
     * on the designated neighbor. The pair::second element in the pair is set
     * to true if a new point was added to the set or false if it was already
     * in.
     */
    std::pair< Iterator, bool> insertNeighbor( Iterator it, Direction d );




    // ----------------------- Static data ------------------------------------
  public:

    static const int D = Space::dimension;
    static const int nb_sons = DGtal::POW< 2, D >::VALUE;
    static const int nb_neighbors = 2*D;


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

    // The associated domain;
    const Domain & myDomain;

    // The container storing the points of the set.
    Tree * myTree;

    // The number of elements in the set.
    Size mySize;


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
