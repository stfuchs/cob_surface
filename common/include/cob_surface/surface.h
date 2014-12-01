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
 * \date Date of creation: 09/2014
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

#ifndef COB_SURFACE_SURFACE_H
#define COB_SURFACE_SURFACE_H

#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>

namespace OpenMesh
{
  template<>
  struct vector_traits<Eigen::Vector3f>
  {
    typedef float value_type;
    typedef Eigen::Matrix<value_type,3,1> vector_type;
    static size_t size() { return 3; }
    static const size_t size_ = 3;
  };
}

namespace cob_surface
{
  // defines surface data structure
  struct DataStructure : public OpenMesh::DefaultTraits
  {
    typedef Eigen::Vector3f Point; // point in map space
    //typedef OpenMesh::Vec3f  Normal;
    //typedef OpenMesh::Vec2f  TexCoord;
    //typedef OpenMesh::Vec3uc Color;

    VertexTraits
    {
    public:
      Eigen::Vector2f proj_point; // point in projection space
      Eigen::Quaternion<float> q; // rotation of covariance
      Eigen::Vector3f s; // eigenvalues -> Scale: s_.asDiagonal()
    };
    //HalfedgeTraits  {};
    //EdgeTraits      {};
    //FaceTraits      {};

    //VertexAttributes(0);
    HalfedgeAttributes(OpenMesh::Attributes::PrevHalfedge);
    //EdgeAttributes(0);
    FaceAttributes(OpenMesh::Attributes::Status);
  };

  // define surface data type
  typedef OpenMesh::TriMesh_ArrayKernelT<DataStructure> Surface;

  /** 
   * returns the point of VertexHandle vh in projection space
   * 
   * @param sf - pointer to surface
   * @param vh - vertex handle
   * 
   * @return point in projection space
   */
  template<typename SurfaceT>
  inline Eigen::Vector2f& projSpace(
    SurfaceT* sf, const typename SurfaceT::VertexHandle& vh) {
    return sf->data(vh).proj_point;
  }

  template<typename SurfaceT>
  inline const Eigen::Vector2f& projSpace(
    const SurfaceT* sf, const typename SurfaceT::VertexHandle& vh) {
    return sf->data(vh).proj_point;
  }

  /** 
   * returns the point of VertexHandle vh in map space
   * 
   * @param sf - pointer to surface
   * @param vh - vertex handle
   * 
   * @return point in map space
   */
  template<typename SurfaceT>
  inline Eigen::Vector3f& mapSpace(
    SurfaceT* sf, const typename SurfaceT::VertexHandle& vh) {
    return sf->point(vh);
  }

  template<typename SurfaceT>
  inline const Eigen::Vector3f& mapSpace(
    const SurfaceT* sf, const typename SurfaceT::VertexHandle& vh) {
    return sf->point(vh);
  }
  
    

  /** 
   * Returns first or second vertex handle of an edge handle
   * 
   * @param sf - the reference surface
   * @param eh - the edge handle
   * @param i - 0 or 1
   * 
   * @return vertex handle associated with the edge handle
   */
  template<typename SurfaceT>
  inline typename SurfaceT::VertexHandle getVertexHandle(
    const SurfaceT& sf,
    const typename SurfaceT::EdgeHandle& eh,
    unsigned int i)
  {
    return sf.to_vertex_handle(sf.halfedge_handle(eh, i));
  }

  /** 
   * helper to access the three vertex handles of a triangle face
   * 
   * @param sf - pointer to surface
   * @param fh - face handle of triangle
   * @param vh1 - out vertex handle 1 (CCW)
   * @param vh2 - out vertex handle 2 (CCW)
   * @param vh3 - out vertex handle 3 (CCW)
   */
  template<typename SurfaceT>
  inline void getFaceVertexHandles(
    const SurfaceT* sf, const typename SurfaceT::FaceHandle& fh,
    typename SurfaceT::VertexHandle& vh1,
    typename SurfaceT::VertexHandle& vh2,
    typename SurfaceT::VertexHandle& vh3)
  {
    typename SurfaceT::HalfedgeHandle heh = sf->halfedge_handle(fh);
    vh1 = sf->to_vertex(heh); heh = sf->next_halfedge_handle(heh);
    vh2 = sf->to_vertex(heh); heh = sf->next_halfedge_handle(heh);
    vh3 = sf->to_vertex(heh);
  }

  /** 
   * computes the orientation of the face formed by vertices v1,v2,v3
   * as (v3-v1)x(v2-v1)*v1 with respect to (0,0,0)
   * 
   * @param sf - surface that holdes handles of v1,v2,v3
   * @param v1 - vertex handle that forms desired face
   * @param v2 - vertex handle that forms desired face
   * @param v3 - vertex handle that forms desired face
   * 
   * @return <0 if oriented towards origin, else >0
   */
  template<typename SurfaceT>
  inline typename float calcFaceOrientation(
    const SurfaceT& sf,
    const typename SurfaceT::VertexHandle& v1,
    const typename SurfaceT::VertexHandle& v2,
    const typename SurfaceT::VertexHandle& v3)
  {
    const typename SurfaceT::Point& p1 = sf.point(v1);
    const typename SurfaceT::Point& p2 = sf.point(v2);
    const typename SurfaceT::Point& p3 = sf.point(v3);
    return (p3-p1).cross(p2-p1).dot(p1);
  }
}


#endif
