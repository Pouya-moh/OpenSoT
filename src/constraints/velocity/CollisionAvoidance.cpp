/*
 * Copyright (C) 2014 Walkman
 * Author: Yangwei YOU
 * email:  yangwei.you@foxmail.com
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU Lesser General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
*/

#include <OpenSoT/constraints/velocity/CollisionAvoidance.h>
#include <boost/filesystem.hpp>
#include <OpenSoT/utils/collision_utils.h>
#include <kdl_parser/kdl_parser.hpp>
#include <fcl/config.h>
#include <fcl/BV/OBBRSS.h>
#include <fcl/BVH/BVH_model.h>
#include <fcl/distance.h>
#include <fcl/shape/geometric_shapes.h>
#include <geometric_shapes/shapes.h>
#include <geometric_shapes/shape_operations.h>
#include <boost/make_shared.hpp>
#include <fcl/config.h>

// local version of vectorKDLToEigen since oldest versions are bogous.
// To use instead of:
// #include <eigen_conversions/eigen_kdl.h>
// tf::vectorKDLToEigen
// void vectorKDLToEigen ( const KDL::Vector &k, Eigen::Matrix<double, 3, 1> &e )
// {
//     for ( int i = 0; i < 3; ++i ) {
//         e[i] = k[i];
//     }
// }

using namespace OpenSoT::constraints::velocity;

using namespace Eigen;

CollisionAvoidance::CollisionAvoidance ( const Eigen::VectorXd& x,
        XBot::ModelInterface &robot,
        std::string& base_link,
        const std::vector<std::string> &interested_links,
        const std::vector<std::string> &environment_links,
        const double &detection_threshold,
        const double &linkPair_threshold,
        const double &boundScaling ) :
    Constraint ( "self_collision_avoidance", x.size() ),
    _detection_threshold ( detection_threshold ),
    _linkPair_threshold ( linkPair_threshold ),
//     computeLinksDistance ( robot ),
    robot_col ( robot ),
    _x_cache ( x ),
    _boundScaling ( boundScaling ),
    base_name ( base_link )
{
    _J_transform.setZero ( 3,6 );

    parseCollisionObjects();

    update ( x );
}

bool CollisionAvoidance::parseCollisionObjects()
{
    boost::shared_ptr<urdf::Model> urdf_model_ptr =
        boost::shared_ptr<urdf::Model> ( new urdf::Model() );
    urdf_model_ptr->initString ( robot_col.getUrdfString() );

    boost::shared_ptr<srdf::Model> srdf_model_ptr =
        boost::shared_ptr<srdf::Model> ( new srdf::Model() );
    srdf_model_ptr->initString ( *urdf_model_ptr, robot_col.getSrdfString() );

//     moveit_robot_model.reset(new robot_model::RobotModel(urdf_model_ptr, srdf_model_ptr));

    boost::filesystem::path original_urdf ( robot_col.getUrdfPath() );
    std::string capsule_model_urdf_filename = std::string ( original_urdf.stem().c_str() ) + std::string ( "_capsules.urdf" );
    boost::filesystem::path capsule_urdf ( original_urdf.parent_path() /
                                           capsule_model_urdf_filename );

    boost::filesystem::path original_srdf ( robot_col.getSrdfPath() );
    std::string capsule_model_srdf_filename = std::string ( original_srdf.stem().c_str() ) + std::string ( "_capsules.srdf" );
    boost::filesystem::path capsule_srdf ( original_srdf.parent_path() /
                                           capsule_model_srdf_filename );

    std::string urdf_to_load, srdf_to_load;

    if ( boost::filesystem::exists ( capsule_urdf ) ) {
        urdf_to_load = capsule_urdf.c_str();
    } else {
        urdf_to_load = original_urdf.c_str();
    }

    if ( boost::filesystem::exists ( capsule_srdf ) ) {
        srdf_to_load = capsule_srdf.c_str();
    } else {
        srdf_to_load = original_srdf.c_str();
    }


    std::cout<<"srdf_to_load: "<<srdf_to_load<<std::endl;
    std::cout<<"urdf_to_load: "<<urdf_to_load<<std::endl;

    urdf::Model robot_urdf;
    srdf::Model robot_srdf;
    robot_urdf.initFile ( urdf_to_load );
    robot_srdf.initFile ( robot_urdf, srdf_to_load );

    linksToUpdate.clear();
    std::vector<boost::shared_ptr<urdf::Link> > links;
    for ( auto &it:_interested_link_pairs ) {
        boost::shared_ptr<urdf::Link> link;
        linksToUpdate.insert ( it.first );
        robot_urdf.getLink ( it.first, link );
        links.push_back ( link );
    }
    for ( auto &link:links ) {
        if ( link->collision ) {
            if ( link->collision->geometry->type == urdf::Geometry::CYLINDER ||
                    link->collision->geometry->type == urdf::Geometry::SPHERE   ||
                    link->collision->geometry->type == urdf::Geometry::BOX      ||
                    link->collision->geometry->type == urdf::Geometry::MESH ) {

                shared_ptr<fcl::CollisionGeometry> shape;
                KDL::Frame shape_origin;

                if ( link->collision->geometry->type == urdf::Geometry::CYLINDER ) {
                    std::cout << "adding capsule for " << link->name << std::endl;

                    boost::shared_ptr<urdf::Cylinder> collisionGeometry =
                        boost::dynamic_pointer_cast<urdf::Cylinder> (
                            link->collision->geometry );

                    shape.reset ( new fcl::Capsule ( collisionGeometry->radius,
                                                     collisionGeometry->length ) );

                    shape_origin = toKdl ( link->collision->origin );
                    shape_origin.p -= collisionGeometry->length/2.0 * shape_origin.M.UnitZ();

                } else if ( link->collision->geometry->type == urdf::Geometry::SPHERE ) {
                    std::cout << "adding sphere for " << link->name << std::endl;

                    boost::shared_ptr<urdf::Sphere> collisionGeometry =
                        boost::dynamic_pointer_cast<urdf::Sphere> (
                            link->collision->geometry );

                    shape.reset ( new fcl::Sphere ( collisionGeometry->radius ) );
                    shape_origin = toKdl ( link->collision->origin );
                } else if ( link->collision->geometry->type == urdf::Geometry::BOX ) {
                    std::cout << "adding box for " << link->name << std::endl;

                    boost::shared_ptr<urdf::Box> collisionGeometry =
                        boost::dynamic_pointer_cast<urdf::Box> (
                            link->collision->geometry );

                    shape.reset ( new fcl::Box ( collisionGeometry->dim.x,
                                                 collisionGeometry->dim.y,
                                                 collisionGeometry->dim.z ) );
                    shape_origin = toKdl ( link->collision->origin );
                    std::cout << "Box has size " << collisionGeometry->dim.x <<
                              ", " << collisionGeometry->dim.y <<
                              ", " << collisionGeometry->dim.z << std::endl;
                } else if ( link->collision->geometry->type == urdf::Geometry::MESH ) {
                    std::cout << "adding mesh for " << link->name << std::endl;

                    boost::shared_ptr< ::urdf::Mesh> collisionGeometry = boost::dynamic_pointer_cast< ::urdf::Mesh> ( link->collision->geometry );

                    shapes::Mesh *mesh = shapes::createMeshFromResource ( collisionGeometry->filename );
                    if ( mesh == NULL ) {
                        std::cout << "Error loading mesh for link " << link->name << std::endl;
                        continue;
                    }

                    std::vector<fcl::Vec3f> vertices;
                    std::vector<fcl::Triangle> triangles;

                    for ( unsigned int i=0; i < mesh->vertex_count; ++i ) {
                        fcl::Vec3f v ( mesh->vertices[3*i]*collisionGeometry->scale.x,
                                       mesh->vertices[3*i + 1]*collisionGeometry->scale.y,
                                       mesh->vertices[3*i + 2]*collisionGeometry->scale.z );

                        vertices.push_back ( v );
                    }

                    for ( unsigned int i=0; i< mesh->triangle_count; ++i ) {
                        fcl::Triangle t ( mesh->triangles[3*i],
                                          mesh->triangles[3*i + 1],
                                          mesh->triangles[3*i + 2] );
                        triangles.push_back ( t );
                    }

                    // add the mesh data into the BVHModel structure
                    shape.reset ( new fcl::BVHModel<fcl::OBBRSS> );
                    fcl::BVHModel<fcl::OBBRSS>* bvhModel = ( fcl::BVHModel<fcl::OBBRSS>* ) shape.get();
                    bvhModel->beginModel();
                    bvhModel->addSubModel ( vertices, triangles );
                    bvhModel->endModel();

                    shape_origin = toKdl ( link->collision->origin );
                }

                boost::shared_ptr<fcl::CollisionObject> collision_object (
                    new fcl::CollisionObject ( shape ) );


                collision_objects_[link->name] = new fcl::CollisionObject ( shape ); // collision_object;
                shapes_[link->name] = shape;

                /* Store the transformation of the CollisionShape from URDF
                 * that is, we store link_T_shape for the actual link */
                link_T_shape[link->name] = shape_origin;
            } else {
                std::cout << "Collision type unknown for link " << link->name << std::endl;
            }
        } else {
            std::cout << "Collision not defined for link " << link->name << std::endl;
        }
    }
    return true;
}

bool CollisionAvoidance::updateCollisionObjects()
{
    for ( auto &it:linksToUpdate ) {
        std::string link_name = it;
        KDL::Frame w_T_link, w_T_shape;
        robot_col.getPose ( link_name, w_T_link );
        w_T_shape = w_T_link * link_T_shape[link_name];

        fcl::Transform3f fcl_w_T_shape = KDL2fcl ( w_T_shape );
        fcl::CollisionObject* collObj_shape = collision_objects_[link_name].get();
        collObj_shape->setTransform ( fcl_w_T_shape );
    }
    return true;
}

double CollisionAvoidance::getLinkPairThreshold()
{
    return _linkPair_threshold;
}

double CollisionAvoidance::getDetectionThreshold()
{
    return _detection_threshold;
}


void CollisionAvoidance::setLinkPairThreshold ( const double &linkPair_threshold )
{
    _linkPair_threshold = std::fabs ( linkPair_threshold );
    //this->update();
}

void CollisionAvoidance::setDetectionThreshold ( const double &detection_threshold )
{
    _detection_threshold = std::fabs ( detection_threshold );
    //this->update();
}


void CollisionAvoidance::update ( const Eigen::VectorXd &x )
{
    // we update _Aineq and _bupperBound only if x has changed
    //if(!(x == _x_cache)) {
    _x_cache = x;
    calculate_Aineq_bUpperB ( _Aineq, _bUpperBound );
    _bLowerBound = -1.0e20*_bLowerBound.setOnes ( _bUpperBound.size() );

    //}
//    std::cout << "_Aineq" << _Aineq.toString() << std::endl << std::endl;
    //    std::cout << "_bUpperBound" << _bUpperBound.toString() << std::endl << std::endl;
}

bool OpenSoT::constraints::velocity::CollisionAvoidance::setCollisionInterestList ( const std::list<LinkPairDistance::LinksPair> &interestList )
{
    bool ok = computeLinksDistance.setCollisionWhiteList ( whiteList );
    this->calculate_Aineq_bUpperB ( _Aineq, _bUpperBound );
    _bLowerBound = -1.0e20*_bLowerBound.setOnes ( _bUpperBound.size() );
    return ok;
}

void CollisionAvoidance::skewSymmetricOperator ( const Eigen::Vector3d & r_cp, Eigen::MatrixXd& J_transform )
{
    if ( J_transform.rows() != 3 || J_transform.cols() != 6 ) {
        J_transform.setZero ( 3,6 );
    }

    J_transform.block ( 0,0,3,3 ) = Eigen::Matrix3d::Identity();
    J_transform.block ( 0,3,3,3 ) <<       0,  r_cp ( 2 ), -r_cp ( 1 ),
                      -r_cp ( 2 ),        0,  r_cp ( 0 ),
                      r_cp ( 1 ), -r_cp ( 0 ),        0;
}

std::list<LinkPairDistance> CollisionAvoidance::getLinkDistances ( const double &detectionThreshold )
{
    std::list<LinkPairDistance> results;

    updateCollisionObjects();


    for ( auto &it:_interested_link_pairs ) {
        std::string linkA = it.first;
        std::string linkB = it.second;

        fcl::CollisionObject* collObj_shapeA = collision_objects_[linkA];
        fcl::CollisionObject* collObj_shapeB = collision_objects_[linkB];

        fcl::DistanceRequest request;
#if FCL_MINOR_VERSION > 2
        request.gjk_solver_type = fcl::GST_INDEP;
#endif
        request.enable_nearest_points = true;

        // result will be returned via the collision result structure
        fcl::DistanceResult result;

        // perform distance test
        fcl::distance ( collObj_shapeA, collObj_shapeB, request, result );

        // p1Homo, p2Homo newly computed points by FCL
        // absolutely computed w.r.t. base-frame
        KDL::Frame linkA_pA, linkB_pB;

        if ( collObj_shapeA->getNodeType() == fcl::GEOM_CAPSULE &&
                collObj_shapeB->getNodeType() == fcl::GEOM_CAPSULE ) {
            globalToLinkCoordinates ( linkA, result.nearest_points[0], linkA_pA );
            globalToLinkCoordinates ( linkB, result.nearest_points[1], linkB_pB );
        } else {
            shapeToLinkCoordinates ( linkA, result.nearest_points[0], linkA_pA );
            shapeToLinkCoordinates ( linkB, result.nearest_points[1], linkB_pB );
        }

        if ( result.min_distance < detectionThreshold )
            results.push_back ( LinkPairDistance ( linkA, linkB,
                                                   linkA_pA, linkB_pB,
                                                   result.min_distance ) );
    }

    results.sort();

    return results;
}

void CollisionAvoidance::calculate_Aineq_bUpperB ( Eigen::MatrixXd & Aineq_fc,
        Eigen::VectorXd & bUpperB_fc )
{

//    robot_col.updateiDyn3Model(x, false);

    std::list<LinkPairDistance> interested_LinkPairs;
    std::list<LinkPairDistance>::iterator j;
    interested_LinkPairs = getLinkDistances ( _detection_threshold );

    /*//////////////////////////////////////////////////////////*/

    MatrixXd Aineq_fc_Eigen ( interested_LinkPairs.size(), robot_col.getJointNum() );
    VectorXd bUpperB_fc_Eigen ( interested_LinkPairs.size() );

    double Dm_LinkPair;
    KDL::Frame Link1_T_CP,Link2_T_CP;
    std::string Link1_name, Link2_name;

    int Link1_index, Link2_index;

    KDL::Frame Waist_T_Link1, Waist_T_Link2, Waist_T_Link1_CP, Waist_T_Link2_CP;
    KDL::Vector Link1_origin_kdl, Link2_origin_kdl, Link1_CP_kdl, Link2_CP_kdl;
    Eigen::Matrix<double, 3, 1> Link1_origin, Link2_origin, Link1_CP, Link2_CP;

    Vector3d closepoint_dir;

    MatrixXd Link1_CP_Jaco, Link2_CP_Jaco;

    Affine3d Waist_frame_world_Eigen;
    robot_col.getPose ( base_name, Waist_frame_world_Eigen );
    Waist_frame_world_Eigen.inverse();

    Matrix3d Waist_frame_world_Eigen_Ro = Waist_frame_world_Eigen.matrix().block ( 0,0,3,3 );
    MatrixXd temp_trans_matrix ( 6,6 );
    temp_trans_matrix.setZero ( 6,6 );
    temp_trans_matrix.block ( 0,0,3,3 ) = Waist_frame_world_Eigen_Ro;
    temp_trans_matrix.block ( 3,3,3,3 ) = Waist_frame_world_Eigen_Ro;

    int linkPairIndex = 0;
    for ( j = interested_LinkPairs.begin(); j != interested_LinkPairs.end(); ++j ) {

        LinkPairDistance& linkPair ( *j );

        Dm_LinkPair = linkPair.getDistance();
        Link1_T_CP = linkPair.getLink_T_closestPoint().first;
        Link2_T_CP = linkPair.getLink_T_closestPoint().second;
        Link1_name = linkPair.getLinkNames().first;
        Link2_name = linkPair.getLinkNames().second;


        robot_col.getPose ( Link1_name, base_name, Waist_T_Link1 );
        robot_col.getPose ( Link2_name, base_name, Waist_T_Link2 );

        Waist_T_Link1_CP = Waist_T_Link1 * Link1_T_CP;
        Waist_T_Link2_CP = Waist_T_Link2 * Link2_T_CP;
        Link1_origin_kdl = Waist_T_Link1.p;
        Link2_origin_kdl = Waist_T_Link2.p;
        Link1_CP_kdl = Waist_T_Link1_CP.p;
        Link2_CP_kdl = Waist_T_Link2_CP.p;

        tf::vectorKDLToEigen ( Link1_origin_kdl, Link1_origin );
        tf::vectorKDLToEigen ( Link2_origin_kdl, Link2_origin );
        tf::vectorKDLToEigen ( Link1_CP_kdl, Link1_CP );
        tf::vectorKDLToEigen ( Link2_CP_kdl, Link2_CP );


        closepoint_dir = Link2_CP - Link1_CP;
        closepoint_dir = closepoint_dir / Dm_LinkPair;

        robot_col.getRelativeJacobian ( Link1_name, base_name,Link1_CP_Jaco );

        Link1_CP_Jaco = temp_trans_matrix * Link1_CP_Jaco;
        skewSymmetricOperator ( Link1_CP - Link1_origin,_J_transform );
        Link1_CP_Jaco = _J_transform * Link1_CP_Jaco;

        robot_col.getRelativeJacobian ( Link2_name, base_name, Link2_CP_Jaco );

        Link2_CP_Jaco = temp_trans_matrix * Link2_CP_Jaco;
        skewSymmetricOperator ( Link2_CP - Link2_origin,_J_transform );
        Link2_CP_Jaco = _J_transform * Link2_CP_Jaco;


        Aineq_fc_Eigen.row ( linkPairIndex ) = closepoint_dir.transpose() * ( Link1_CP_Jaco - Link2_CP_Jaco );
        bUpperB_fc_Eigen ( linkPairIndex ) = ( Dm_LinkPair - _linkPair_threshold ) * _boundScaling;

        ++linkPairIndex;

    }

    Aineq_fc = Aineq_fc_Eigen;
    bUpperB_fc = bUpperB_fc_Eigen;

}

void CollisionAvoidance::setBoundScaling ( const double &boundScaling )
{
    _boundScaling = boundScaling;
}
