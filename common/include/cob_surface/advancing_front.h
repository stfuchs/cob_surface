/*!
 *****************************************************************
 * \file
 *
 * \note
 *   Copyright (c) 2014 \n
 *   Fraunhofer Institute for Manufacturing Engineering
 *   and Automation (IPA) \n\n
 *
 *****************************************************************
 *
 * \note
 *  Project name: care-o-bot
 * \note
 *  ROS stack name: cob_environment_perception
 * \note
 *  ROS package name: cob_surface
 *
 * \author
 *  Author: Steffen Fuchs, email:richard.bormann@ipa.fhg.de
 * \author
 *  Supervised by: Richard Bormann, email:richard.bormann@ipa.fhg.de
 *
 * \date Date of creation: 11/2014
 *
 * \brief
 * Description:
 *
 * ToDo:
 *
 *
 *****************************************************************
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     - Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer. \n
 *     - Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution. \n
 *     - Neither the name of the Fraunhofer Institute for Manufacturing
 *       Engineering and Automation (IPA) nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission. \n
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License LGPL as
 * published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License LGPL for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License LGPL along with this program.
 * If not, see <http://www.gnu.org/licenses/>.
 *
 ****************************************************************/

#ifndef COB_SURFACE_ADVANCING_FRONT_HPP
#define COB_SURFACE_ADVANCING_FRONT_HPP

#include <set>
#include <cob_surface/sweepline.h>
#include <cob_surface/geometry.h>

namespace SweepLine
{
  template<typename SurfaceT>
  struct AdvancingFront
  {
    typedef typename SurfaceT::VertexHandle VertexHandle;
    typedef typename SurfaceT::HalfedgeHandle HalfedgeHandle;
    typedef typename SurfaceT::FaceHandle FaceHandle;
    typedef typename SurfaceT::Point PointT;
    typedef typename SurfaceT::ProjPoint ProjPoint;
    typedef typename SurfaceT::Scalar ScalarT;

    struct Compare
    {
      Compare(SurfaceT* sf) : sf_(sf) {}
      bool operator() (const VertexHandle& a, const VertexHandle& b) {
        if (projSpace(sf_,a)[0] == projSpace(sf_,b)[0])
          return projSpace(sf_,a)[1] > projSpace(sf_,b)[1];
        return projSpace(sf_,a)[0] < projSpace(sf_,b)[0];
      }
      SurfaceT* sf_;
    };

    typedef typename std::set<VertexHandle, Compare>::iterator NodeIter;
    std::set<VertexHandle,Compare> bst_;
    SurfaceT* sf_;

    AdvancingFront(SurfaceT* sf) : bst_(Compare(sf)), sf_(sf) {}

    /** 
     * define first triangle
     * 
     * @param a left most vertex (temporary)
     * @param b middle vertex
     * @param c right most vertex (temporary)
     */
    inline void initialize(const VertexHandle& a,
                           const VertexHandle& b,
                           const VertexHandle& c)
    {
      sf_->add_face(a,b,c);
      std::cout<<"init: "<< projSpace(sf_,a).transpose() << " "
               << projSpace(sf_,b).transpose() << " "
               << projSpace(sf_,c).transpose() << std::endl;
      bst_.insert(c);
      bst_.insert(b);
      bst_.insert(a);
    }

    inline void insert(const VertexHandle& v)
    {
      NodeIter r = bst_.upper_bound(v);
      NodeIter l = leftNeighbor(r);
      NodeIter x = bst_.insert(r,v);

      ProjPoint p = projSpace(sf_,v);
      ProjPoint pr = projSpace(sf_,*r);
      ProjPoint pl = projSpace(sf_,*l);
      //std::cout << "insert: " << p.transpose() << std::endl;
      FaceHandle fh = sf_->add_face(v,*r,*l);
      if(!fh.is_valid())
      {
        std::cout << "M: " << p.transpose() << " "
                  << pr.transpose() << " "
                  << pl.transpose() << std::endl;
        return;
      }
      HalfedgeHandle heh_outer = sf_->halfedge_handle(v);
      assert( sf_->is_boundary(heh_outer) );
      HalfedgeHandle heh_l = sf_->opposite_halfedge_handle(heh_outer);
      HalfedgeHandle heh_r = sf_->next_halfedge_handle(heh_l);
      delaunayFix(sf_->next_halfedge_handle(heh_r));

      NodeIter rr = rightNeighbor(r);
      if(rr!=bst_.end())
      {

        ProjPoint prr = projSpace(sf_,*rr);
        if( (pr-p)[0] < .005 || (p-pr).dot(prr-pr) > 0 )
        {
          fh = sf_->add_face(v,*rr,*r);
          if(!fh.is_valid())
          {
            std::cout << "R: " << p.transpose() << " "
                      << prr.transpose() << " "
                      << pr.transpose() << std::endl;
            return;
          }
          bst_.erase(r);
        }
      }

      if(l != bst_.begin())
      {
        NodeIter ll = leftNeighbor(l);
        ProjPoint pll = projSpace(sf_,*ll);

        if( (p-pl)[0] < .005 || (p-pl).dot(pll-pl) > 0 )
        {
          fh = sf_->add_face(v,*l,*ll);
          if(!fh.is_valid())
          {
            std::cout << "L: " << p.transpose() << " "
                      << pl.transpose() << " "
                      << pll.transpose() << std::endl;
            return;
          }
          bst_.erase(l);
        }
      }
    }

    bool delaunayFix(const HalfedgeHandle& heh)
    {
      VertexHandle a = sf_->to_vertex_handle(heh);
      VertexHandle b = sf_->to_vertex_handle(sf_->next_halfedge_handle(heh));
      HalfedgeHandle oheh = sf_->opposite_halfedge_handle(heh);
      VertexHandle c = sf_->to_vertex_handle(oheh);
      VertexHandle d = sf_->to_vertex_handle(sf_->next_halfedge_handle(oheh));
      /*std::cout << projSpace(sf_,a).transpose() << std::endl;
      std::cout << projSpace(sf_,b).transpose() << std::endl;
      std::cout << projSpace(sf_,c).transpose() << std::endl;
      std::cout << projSpace(sf_,d).transpose() << std::endl;*/

      if (cob_surface::Geometry::inCircumcircle<ScalarT>(
            projSpace(sf_,a),projSpace(sf_,b),projSpace(sf_,c),projSpace(sf_,d)))
      {
        //std::cout << "Flip: " << sf_->is_flip_ok(sf_->edge_handle(heh)) << std::endl;
        //return true;
        //assert( sf_->is_flip_ok(sf_->edge_handle(heh)) );
        if(!sf_->is_flip_ok(sf_->edge_handle(heh)))
        {
          std::cout << "flip not ok" << std::endl;
          std::cout << projSpace(sf_,a).transpose() << std::endl;
          std::cout << projSpace(sf_,b).transpose() << std::endl;
          std::cout << projSpace(sf_,c).transpose() << std::endl;
          std::cout << projSpace(sf_,d).transpose() << std::endl;
          return false;
        }
        sf_->flip(sf_->edge_handle(heh));
        return true;
      }
      return false;
    }
  };
}

#endif
