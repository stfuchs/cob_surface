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
 * \date Date of creation: 10/2014
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

#ifndef COB_SURFACE_CONVERSION_H
#define COB_SURFACE_CONVERSION_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace cob_surface
{
  template<typename MeshT>
  struct DefaultOpenMeshConversionPolicy
  {
    typedef typename MeshT::VertexHandle VHandle;
    typedef typename MeshT::FaceHandle FHandle;

    template<typename PointT>
    static inline VHandle addVertex(
      const PointT& point, unsigned int idx, MeshT& mesh)
    {
      return mesh.add_vertex(typename MeshT::Point(point.x, point.y, point.z));
    }

    static inline FHandle addFace(
      const VHandle& v1, const VHandle& v2, const VHandle& v3,
      MeshT& mesh)
    {
      return mesh.add_face(v1,v2,v3);
    }
  };


  template<typename SensorT>
  class Conversion
  {
  public:
    /** 
     * 
     * 
     * @param a 
     * @param b 
     * 
     * @return 
     */
    static inline bool isNeighbor(const Eigen::Vector3f& a, const Eigen::Vector3f& b)
    {
      return SensorT::areNeighbors(a,b);
    }

    /** 
     * converts an organized PointCloud to surface mesh
     * 
     * @param pc - Input point cloud
     * @param mesh - Output mesh
     */
    template<typename PointT, typename MeshT,
             typename Policy = DefaultOpenMeshConversionPolicy<MeshT> >
    static void pointCloud2Mesh(
      const typename pcl::PointCloud<PointT>::ConstPtr& pc, MeshT& mesh);
  };
}

#include "cob_surface/impl/conversion.hpp"

#endif
