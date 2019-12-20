#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_types.h>
#include <iostream>

#include <pcl/common/common.h>
#include <pcl/common/eigen.h>
#include <pcl/io/obj_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/visualization/pcl_visualizer.h>

//various world points 

//GOAL FRAME
//bottom left 
const pcl::PointXYZ bottom_left(-366,0,0);
//top left
const pcl::PointXYZ top_left(-366,244,0);
//top right 
const pcl::PointXYZ top_right(366,244,0);
//bottom right
const pcl::PointXYZ bottom_right(366,0,0);
//bottom left direction and distance 
const pcl::PointXYZ upwards(0,244,0);
//top bar distance and direction 
const pcl::PointXYZ across(732,0,0);

//Goal Dimensions  
//Post radius 
const float post_radius = 6.35;
//Ball Radius 
const float ball_radius = 11;

//BALL 
//for sphere creation 
const pcl::PointXYZ init_ball_point(-200,11,-100);

// Test function that instantiates a mimic of the inoutted vector or ball positions -> here represented as a stright line on the Z axis 
// To be commented out or deleted 
std::vector<pcl::PointXYZ> instantiateStraightBallTrack ( pcl::PointXYZ init_position, int frames, int pixel_per_frame )
{
    std::vector<pcl::PointXYZ> result;
    pcl::PointXYZ temp = init_position;

    for ( int i = 0; i < frames; i++ ){
        temp.z = -(pixel_per_frame * i + init_position.z); 
        temp.x = pixel_per_frame * i + init_position.x;
        temp.y = pixel_per_frame * i + init_position.y; 
        result.push_back(temp);
    }

    return result;
}

// Instatiating ball track from inputted vector 
void ballTracing ( std::vector<pcl::PointXYZ> ball_points, pcl::visualization::PCLVisualizer::Ptr target )
{
    int array_len = ball_points.size();

    for (int i =0; i <array_len; i++)
    {
        std::string name = "ball" + std::to_string(i);
        target->addSphere( ball_points[i], ball_radius, 0.2, 0.2, 0.8, name );
    }
}

// Instantiate posts for goal frame 
void drawOneCylinder (pcl::PointXYZ startingPoint, pcl::PointXYZ direction, float radius, std::string id,  
    pcl::visualization::PCLVisualizer::Ptr target)
{

    pcl::ModelCoefficients line_coeff;
    line_coeff.values.resize (7);
    line_coeff.values[0] = startingPoint.x;
    line_coeff.values[1] = startingPoint.y;
    line_coeff.values[2] = startingPoint.z;

    line_coeff.values[3] = direction.x;
    line_coeff.values[4] = direction.y;
    line_coeff.values[5] = direction.z;

    line_coeff.values[6] = radius;

    target->addCylinder(line_coeff,id);

}
// Tranlate copa coordinates into visualizer coordinates 
// Use RealSense Manager instead
pcl::PointXYZ coordCorrection ( pcl::PointXYZ RS_ball_point )
{
    pcl::PointXYZ corrected_coord; 
    corrected_coord.x = RS_ball_point.y;
    corrected_coord.y = RS_ball_point.z; 
    corrected_coord.z = RS_ball_point.x;

    return corrected_coord;
}

int main()
{
    // viewer definition
    pcl::visualization::PCLVisualizer::Ptr vis(new pcl::visualization::PCLVisualizer("3D View"));

    // point container for ground plane 
    pcl::PointCloud<pcl::PointXYZ>::Ptr goal (new pcl::PointCloud<pcl::PointXYZ>);
    
    // color space
    // visualization statements
    //pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red(goal,1,0,0);
    //pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> blue(ball_trace,0,0,255);

    //Cylinder radius 
    float radius = 2;

    //draw ground plane 
    pcl::ModelCoefficients coeffs;
    coeffs.values.push_back (0.0);
    coeffs.values.push_back (1.0);
    coeffs.values.push_back (0.0);
    coeffs.values.push_back (0.0);

   
    //flat rectengle that serves as our ground plane 
    vis->addCube(-400,400,0,0.5,-400,400,0.2,0.8,0.2, "cube");

    //draw goal frame 
    drawOneCylinder(bottom_left, upwards, post_radius, "left", vis);
    drawOneCylinder(top_left, across, post_radius, "top", vis);
    drawOneCylinder(bottom_right, upwards, post_radius, "right", vis);

    //instantiate sphere
    //vis->addSphere(init_ball_point,1.5, 0, 0, 255, "ball");
   
    //add gyro 
    vis->addCoordinateSystem(10);

    std::vector<pcl::PointXYZ> ball_trace = instantiateStraightBallTrack(init_ball_point, ball_radius, ball_radius*2);
    ballTracing(ball_trace, vis);

    vis->spin();
  
    // while(!vis.wasStopped())
    // {
    //     // clean rendering space points
    //     //vis.removeAllPointClouds();

    //     vis.spinOnce();
        
    // }

    
    return 0;
}
