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
 * @file IIGeometricFunctors.h
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2014/05/14
 *
 * Header file for module IIGeometricFunctors.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(IIGeometricFunctors_RECURSES)
#error Recursive header files inclusion detected in IIGeometricFunctors.h
#else // defined(IIGeometricFunctors_RECURSES)
/** Prevents recursive inclusion of headers. */
#define IIGeometricFunctors_RECURSES

#if !defined IIGeometricFunctors_h
/** Prevents repeated inclusion of headers. */
#define IIGeometricFunctors_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "DGtal/base/Common.h"
#include "DGtal/math/EigenDecomposition.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /**
  * This namespace gathers functors transforming the integral
  * invariant covariance matrix into some geometric quantity: normal
  * vector, mean curvature, first and second principal curvatures,
  * principal directions, etc.
  */
  namespace IIGeometricFunctors 
  {

    /////////////////////////////////////////////////////////////////////////////
    // template class IINormalDirectionFunctor
    /**
    * Description of template class 'IINormalDirectionFunctor' <p>
    * \brief Aim: A functor Matrix -> RealVector that returns the normal
    * direction by diagonalizing the given covariance matrix.
    *
    * @tparam TSpace a model of CSpace, for instance SpaceND.
    *
    * @see IntegralInvariantCovarianceEstimator
    */
    template  <typename TSpace>
    class IINormalDirectionFunctor
    {
      // ----------------------- Standard services ------------------------------
    public:
      typedef IINormalDirectionFunctor<TSpace> Self;
      typedef TSpace Space;
      typedef typename Space::RealVector RealVector;
      typedef typename RealVector::Component Component;
      typedef SimpleMatrix<Component,Space::dimension,Space::dimension> Matrix;
      typedef Matrix Argument;
      typedef RealVector Quantity;
      typedef Quantity Value;

      /// Default constructor.
      IINormalDirectionFunctor() {}
      /// Copy constructor. Nothing to do.
      IINormalDirectionFunctor( const Self& /* other */ ) {}
      /// Assignment. Nothing to do.
      /// @return itself
      Self& operator=( const Self& /* other */ ) { return *this; }
      /**
      * Apply operator.
      * @param arg any symmetric positive matrix (covariance matrix
      *
      * @return the normal direction for the II covariance matrix,
      * which is the eigenvector associated with the smallest
      * eigenvalue.
      */
      Value operator()( const Argument& arg ) const
      {
        EigenDecomposition<Space::dimension, Component>
          ::getEigenDecomposition( arg, eigenVectors, eigenValues );
        return eigenVectors.column( 0 ); // normal vector is associated to smallest eigenvalue.      
      }

      /**
      * Initializes the functor with the gridstep and the ball
      * Euclidean radius. Not used for this estimator.
      */
      void init( Component /* h */, Component /* r */ ) {}

      /// A data member only used for temporary calculations.
      mutable Matrix eigenVectors;
      /// A data member only used for temporary calculations.
      mutable RealVector eigenValues;
    }; // end of class IINormalDirectionFunctor


    /////////////////////////////////////////////////////////////////////////////
    // template class IITangentDirectionFunctor
    /**
    * Description of template class 'IITangentDirectionFunctor' <p>
    * \brief Aim: A functor Matrix -> RealVector that returns the tangent
    * direction by diagonalizing the given covariance matrix. This functor is valid only in 2D space.
    *
    * @tparam TSpace a model of CSpace, for instance SpaceND.
    *
    * @see IntegralInvariantCovarianceEstimator
    */
    template  <typename TSpace>
    class IITangentDirectionFunctor
    {
      // ----------------------- Standard services ------------------------------
    public:
      typedef IITangentDirectionFunctor<TSpace> Self;
      typedef TSpace Space;
      typedef typename Space::RealVector RealVector;
      typedef typename RealVector::Component Component;
      typedef SimpleMatrix<Component,Space::dimension,Space::dimension> Matrix;
      typedef Matrix Argument;
      typedef RealVector Quantity;
      typedef Quantity Value;

      BOOST_STATIC_ASSERT(( Space::dimension == 2 ));

      /// Default constructor.
      IITangentDirectionFunctor() {}
      /// Copy constructor. Nothing to do.
      IITangentDirectionFunctor( const Self& /* other */ ) {}
      /// Assignment. Nothing to do.
      /// @return itself
      Self& operator=( const Self& /* other */ ) { return *this; }
      /**
      * Apply operator.
      * @param arg any symmetric positive matrix (covariance matrix
      *
      * @return the tangent direction for the II covariance matrix,
      * which is the eigenvector associated with the highest
      * eigenvalue.
      */
      Value operator()( const Argument& arg ) const
      {
        EigenDecomposition<Space::dimension, Component>
          ::getEigenDecomposition( arg, eigenVectors, eigenValues );
        return eigenVectors.column( 1 ); // tangent vector is associated to greatest eigenvalue.      
      }
      /// A data member only used for temporary calculations.
      mutable Matrix eigenVectors;
      /// A data member only used for temporary calculations.
      mutable RealVector eigenValues;
    }; // end of class IITangentDirectionFunctor


    /////////////////////////////////////////////////////////////////////////////
    // template class IIFirstPrincipalDirectionFunctor
    /**
    * Description of template class 'IIFirstPrincipalDirectionFunctor'
    * <p> \brief Aim: A functor Matrix -> RealVector that returns the
    * first principal curvature direction by diagonalizing the given
    * covariance matrix. This functor is valid starting from 2D space
    * and is equivalent to IITangentDirectionFunctor in 2D. Note that
    * by first we mean the direction with greatest curvature in absolute
    * value.
    *
    * @tparam TSpace a model of CSpace, for instance SpaceND.
    *
    * @see IntegralInvariantCovarianceEstimator
    */
    template  <typename TSpace>
    class IIFirstPrincipalDirectionFunctor
    {
      // ----------------------- Standard services ------------------------------
    public:
      typedef IIFirstPrincipalDirectionFunctor<TSpace> Self;
      typedef TSpace Space;
      typedef typename Space::RealVector RealVector;
      typedef typename RealVector::Component Component;
      typedef SimpleMatrix<Component,Space::dimension,Space::dimension> Matrix;
      typedef Matrix Argument;
      typedef RealVector Quantity;
      typedef Quantity Value;

      BOOST_STATIC_ASSERT(( Space::dimension >= 2 ));

      /// Default constructor.
      IIFirstPrincipalDirectionFunctor() {}
      /// Copy constructor. Nothing to do.
      IIFirstPrincipalDirectionFunctor( const Self& /* other */ ) {}
      /// Assignment. Nothing to do.
      /// @return itself
      Self& operator=( const Self& /* other */ ) { return *this; }
      /**
      * Apply operator.
      * @param arg any symmetric positive matrix (covariance matrix
      *
      * @return the first principal curvature direction for the II
      * covariance matrix, which is the eigenvector associated with
      * the highest eigenvalue.
      */
      Value operator()( const Argument& arg ) const
      {
        EigenDecomposition<Space::dimension, Component>
          ::getEigenDecomposition( arg, eigenVectors, eigenValues );
        return eigenVectors.column( Space::dimension - 1 ); // first principal curvature direction is associated to greatest eigenvalue.      
      }

      /**
      * Initializes the functor with the gridstep and the ball
      * Euclidean radius. Not used for this estimator.
      */
      void init( Component /* h */, Component /* r */ ) {}

      /// A data member only used for temporary calculations.
      mutable Matrix eigenVectors;
      /// A data member only used for temporary calculations.
      mutable RealVector eigenValues;
    }; // end of class IIFirstPrincipalDirectionFunctor


    /////////////////////////////////////////////////////////////////////////////
    // template class IISecondPrincipalDirectionFunctor
    /**
    * Description of template class
    * 'IISecondPrincipalDirectionFunctor' <p> \brief Aim: A functor
    * Matrix -> RealVector that returns the second principal curvature
    * direction by diagonalizing the given covariance matrix. This
    * functor is valid starting from 3D space. Note that by second we
    * mean the direction with second greatest curvature in absolute
    * value.
    *
    * @tparam TSpace a model of CSpace, for instance SpaceND.
    *
    * @see IntegralInvariantCovarianceEstimator
    */
    template  <typename TSpace>
    class IISecondPrincipalDirectionFunctor
    {
      // ----------------------- Standard services ------------------------------
    public:
      typedef IISecondPrincipalDirectionFunctor<TSpace> Self;
      typedef TSpace Space;
      typedef typename Space::RealVector RealVector;
      typedef typename RealVector::Component Component;
      typedef SimpleMatrix<Component,Space::dimension,Space::dimension> Matrix;
      typedef Matrix Argument;
      typedef RealVector Quantity;
      typedef Quantity Value;

      BOOST_STATIC_ASSERT(( Space::dimension >= 3 ));

      /// Default constructor.
      IISecondPrincipalDirectionFunctor() {}
      /// Copy constructor. Nothing to do.
      IISecondPrincipalDirectionFunctor( const Self& /* other */ ) {}
      /// Assignment. Nothing to do.
      /// @return itself
      Self& operator=( const Self& /* other */ ) { return *this; }
      /**
      * Apply operator.
      * @param arg any symmetric positive matrix (covariance matrix
      *
      * @return the second principal curvature direction for the II
      * covariance matrix, which is the eigenvector associated with
      * the highest eigenvalue.
      */
      Value operator()( const Argument& arg ) const
      {
        EigenDecomposition<Space::dimension, Component>
          ::getEigenDecomposition( arg, eigenVectors, eigenValues );
        return eigenVectors.column( Space::dimension - 2 ); // second principal curvature direction is associated to greatest eigenvalue.      
      }

      /**
      * Initializes the functor with the gridstep and the ball
      * Euclidean radius. Not used for this estimator.
      */
      void init( Component /* h */, Component /* r */ ) {}

      /// A data member only used for temporary calculations.
      mutable Matrix eigenVectors;
      /// A data member only used for temporary calculations.
      mutable RealVector eigenValues;
    }; // end of class IISecondPrincipalDirectionFunctor


    /////////////////////////////////////////////////////////////////////////////
    // template class IICurvatureFunctor
    /**
    * Description of template class 'IICurvatureFunctor' <p> \brief
    * Aim: A functor Real -> Real that returns the 2d curvature by
    * transforming the given volume. This functor is valid only in 2D
    * space.
    *
    * @tparam TSpace a model of CSpace, for instance SpaceND.
    *
    * @see IntegralInvariantVolumeEstimator
    */
    template  <typename TSpace>
    struct IICurvatureFunctor
    {
      // ----------------------- Standard services ------------------------------
    public:
      typedef IICurvatureFunctor<TSpace> Self;
      typedef TSpace Space;
      typedef typename Space::RealVector RealVector;
      typedef typename RealVector::Component Component;
      typedef Component Argument;
      typedef Component Quantity;
      typedef Quantity Value;

      BOOST_STATIC_ASSERT(( Space::dimension == 2 ));

      /**
      * Apply operator.
      * @param arg any symmetric positive matrix (covariance matrix
      *
      * @return the curvature associated with the II area measurement.
      */
      Value operator()( const Argument& arg ) const
      {
        Quantity cp_quantity = arg;
        cp_quantity *= dh2;
        return d3_r * ( dPI_2 - d1_r2 * cp_quantity );
      }

      /**
      * Initializes the functor with the gridstep and the ball Euclidean radius.
      *
      * @param h the gridstep
      * @param r the ball radius
      */
      void init( Component h, Component r )
      {
        d1_r2 = 1.0 / ( r * r );
        dPI_2 = M_PI / 2.0;
        d3_r = 3.0 / r;
        dh2 = h * h;
      }

    private:
      Quantity dh2;
      Quantity d3_r;
      Quantity dPI_2;
      Quantity d1_r2;
    }; // end of class IICurvatureFunctor


    /////////////////////////////////////////////////////////////////////////////
    // template class IIMeanCurvature3DFunctor
    /**
    * Description of template class 'IIMeanCurvature3DFunctor' <p> \brief
    * Aim: A functor Real -> Real that returns the 3d mean curvature by
    * transforming the given volume. This functor is valid only in 3D
    * space.
    *
    * @tparam TSpace a model of CSpace, for instance SpaceND.
    *
    * @see IntegralInvariantVolumeEstimator
    */
    template  <typename TSpace>
    struct IIMeanCurvature3DFunctor
    {
      // ----------------------- Standard services ------------------------------
    public:
      typedef IIMeanCurvature3DFunctor<TSpace> Self;
      typedef TSpace Space;
      typedef typename Space::RealVector RealVector;
      typedef typename RealVector::Component Component;
      typedef Component Argument;
      typedef Component Quantity;
      typedef Quantity Value;

      BOOST_STATIC_ASSERT(( Space::dimension == 3 ));

      /**
      * Apply operator.
      * @param arg any symmetric positive matrix (covariance matrix
      *
      * @return the curvature associated with the II area measurement.
      */
      Value operator()( const Argument& arg ) const
      {
        Quantity cp_quantity = arg;
        cp_quantity *= dh3;
        return d8_3r - d_4_PIr4 * cp_quantity;
      }

      /**
      * Initializes the functor with the gridstep and the ball Euclidean radius.
      *
      * @param h the gridstep
      * @param r the ball radius
      */
      void init( Component h, Component r )
      {
        d8_3r = 8.0 / ( 3.0 * r );
        double r2 = r * r;
        d_4_PIr4 = 4.0 / ( M_PI * r2 * r2 );
        dh3 = h * h * h;
      }

    private:
      Quantity dh3;
      Quantity d8_3r;
      Quantity d_4_PIr4;
    }; // end of class IIMeanCurvature3DFunctor



  } // namespace IIGeometricFunctors 

} // namespace DGtal


//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined IIGeometricFunctors_h

#undef IIGeometricFunctors_RECURSES
#endif // else defined(IIGeometricFunctors_RECURSES)
