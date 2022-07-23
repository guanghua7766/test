#include <pluginlib/class_list_macros.h>

#include <virtual_wall/virtual_wall.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

PLUGINLIB_EXPORT_CLASS(virtual_wall::VirtualWall, costmap_2d::Layer)

using namespace std;
using costmap_2d::NO_INFORMATION;
using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::FREE_SPACE;
bool isUpdated=false;
namespace virtual_wall {

    VirtualWall::VirtualWall()
    {
        cout << "VirtualWall()" << endl;
        // ros::NodeHandle n;
        wallmax_x = 0.0;
        wallmax_y = 0.0;
        wallmin_x = 0.0;
        wallmin_y = 0.0;
    }

    VirtualWall::~VirtualWall()
    {
      cout << "~VirtualWall()" << endl;
    }

    void VirtualWall::onInitialize()
    {
        boost::unique_lock < boost::recursive_mutex > lock(data_access_);
        ros::NodeHandle g_nh;
        nh = ros::NodeHandle("~/" + name_);
        matchSize();
        current_ = true;
        enabled_ = true;
        add_wall_sub_ = g_nh.subscribe("/virtual_wall", 5000, &VirtualWall::AddWallCallback, this);
        // wall_list_pub = g_nh.advertise<std_msgs::Int32>("/virtual_wall_list", 1);
        delete_wall_sub = g_nh.subscribe("/delete_wall", 1, &VirtualWall::DeleteWallCallback, this);
        //发布虚拟墙标记
        // wall_maker_pub = g_nh.advertise<visualization_msgs::Marker>("virtual_wall_vis", 1);
    }

    void VirtualWall::matchSize()
    {
        boost::unique_lock < boost::recursive_mutex > lock(data_access_);
        costmap_2d::Costmap2D* master = layered_costmap_->getCostmap();
        resolution = master->getResolution();
        // resizeMap(master->getSizeInCellsX(), master->getSizeInCellsY(), master->getResolution(),
        //           master->getOriginX(), master->getOriginY());

        global_frame_ = layered_costmap_->getGlobalFrameID();
        cout << "global_frame_ : " << global_frame_ << endl;
        map_frame_ = "map";
    }

    /* *********************************************************************
    * updateBounds
    *
    * update obstacles and bounds
    */
    void VirtualWall::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y,
                                    double* max_x, double* max_y)
    {
         //useExtraBounds(min_x, min_y, max_x, max_y);
        *min_x = std::min(wallmin_x, *min_x);
        *min_y = std::min(wallmin_y, *min_y);
        *max_x = std::max(wallmax_x, *max_x);
        *max_y = std::max(wallmax_y, *max_y);
        /*if (isUpdated) {
           *min_x=-3.6;
           *min_y=-4.1;
           *max_x=-3.9;
           *max_y=3.89;
        }*/
      //    *min_x=robot_x-5.0;
      //    *min_y=robot_y-5.0;
      //    *max_x=robot_x+5.0;;
      //    *max_y=robot_y+5.0;
      //    cout << "min_x："<<*min_x << endl;
      //  cout << "min_y："<<*min_y << endl;
      // cout << "max_x："<<*max_x << endl;
      //  cout << "max_y："<<*max_y << endl;
    }

    /* *********************************************************************
    * updateCosts
    *
    * updates the master grid in the area defined in updateBounds
    */
    void VirtualWall::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
    {
            //updateWithMax(master_grid, min_i, min_j, max_i, max_j);
            
            boost::unique_lock < boost::recursive_mutex > lock(data_access_);
            //判断frame id 是否是map
            if(global_frame_ == map_frame_){    
                  geometry_msgs::Point pt;
                  pt.z = 0.15;
                  for (size_t i = 0; i < wallPoint.size(); i++)
                  {
                        // cout << "(" << wallPoint[i].polygon.points[j].x << ", " << wallPoint[i].polygon.points[j].y << ")";
                        unsigned int pixle_x;
                        unsigned int pixle_y;
                        
                        bool ret = master_grid.worldToMap(wallPoint[i].x, wallPoint[i].y, pixle_x, pixle_y);                        
                        if (ret)
                        {
                          // cout << " (" << pixle_x << ", " << pixle_y << ") ";
                          //if (wallPoint[i].x<min_i||wallPoint[i].x>max_i||wallPoint[i].y<min_j||wallPoint[i].y>max_j) continue;
                          master_grid.setCost(pixle_x, pixle_y, costmap_2d::LETHAL_OBSTACLE);
                        }
                }
            } else {
                  geometry_msgs::TransformStamped transform;
                  try
                  {
                    transform = tf_->lookupTransform(global_frame_, map_frame_, ros::Time(0));      
                  } catch (tf2::TransformException ex){
                    ROS_ERROR("%s", ex.what());
                    return;
                  }
                  // Copy map data given proper transformations
                  tf2::Transform tf2_transform;
                  tf2::convert(transform.transform, tf2_transform);
                  for (size_t i = 0; i < wallPoint.size(); i++)
                  {            
                      double wx, wy;    
                      unsigned int pixle_x;
                      unsigned int pixle_y;
                      wx = wallPoint[i].x;
                      wy = wallPoint[i].y;
                      tf2::Vector3 p(wx, wy, 0);
                      p = tf2_transform*p;
                      bool ret = master_grid.worldToMap(p.x(), p.y(), pixle_x, pixle_y);
                      if (ret)
                      {
                        master_grid.setCost(pixle_x, pixle_y, costmap_2d::LETHAL_OBSTACLE);
                      }
                  }
            }

    }


    //添加虚拟墙
    void VirtualWall::AddWallCallback(const geometry_msgs::PolygonConstPtr& msg)
    {
      
      int size=msg->points.size();
      for (int i=0;i<size;i++)
      {
         geometry_msgs::Point point;
         point.x = msg->points[i].x;
         point.y = msg->points[i].y;
         point.z = msg->points[i].z;
//cout << "point.x" <<point.x<< endl;
//cout << "point.y" <<point.y<< endl;
         wallmax_x = std::max(wallmax_x, point.x);
         wallmax_y = std::max(wallmax_y, point.y);
         wallmin_x = std::min(wallmin_x, point.x);
         wallmin_y = std::min(wallmin_y, point.y);
         wallPoint.push_back(point);
         
      }
    }

    //删除虚拟墙
    void VirtualWall::DeleteWallCallback(const std_msgs::String& msg)
    {
        wallPoint.clear();
               isUpdated=true;
    }
}
