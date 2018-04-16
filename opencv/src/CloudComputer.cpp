#include "CloudComputer.hpp"
#include <cmath>
#include <iostream>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
//BROKEN
//#include <pcl/visualization/pcl_visualizer.h>


CloudComputer::CloudComputer(std::vector<IConfigurable::Param>&& params)
  : IConfigurable{ {"pipe"}, std::move(params)} {


//   myfifo = "myfifo";

//   /* create the FIFO (named pipe) */
//   mkfifo(myfifo, 0666);

  //![create_images]
  /// Windows names

  w = 500;
  
  /// Create black empty images
  hallway_image = cv::Mat::zeros( w, w, CV_8UC3 );

}

void CloudComputer::addCircle( cv::Mat img, cv::Point center , int color)
{
 int thickness = -1;
 int lineType = 8;

 

 circle( img,
         center,
         1.0,
         cv::Scalar( color, 0, 255 ),
         thickness,
         lineType );
}


void CloudComputer::display2D(ORB_SLAM2::Map* total_map) {

  char hallway_window[] = "Hallway Projection";
  hallway_image = cv::Mat::zeros( w, w, CV_8UC3 );

  const vector<ORB_SLAM2::MapPoint*> &map_pts = total_map->GetAllMapPoints();
  const vector<ORB_SLAM2::MapPoint*> &ref_pts = total_map->GetReferenceMapPoints();

  set<ORB_SLAM2::MapPoint*> set_ref_pts(ref_pts.begin(), ref_pts.end());

  for(size_t i=0, iend=map_pts.size(); i<iend;i++)
  {
      if(map_pts[i]->isBad() || set_ref_pts.count(map_pts[i]))
          continue;
      cv::Mat pos = map_pts[i]->GetWorldPos();
      pt new_pt = {pos.at<float>(0), pos.at<float>(1), pos.at<float>(2)};
      //use only x and z
      cv::Point p = cv::Point(static_cast<int>(w/2+floor(new_pt.y*20)),
                          static_cast<int>(w/2+floor(new_pt.z*20)));
      addCircle(hallway_image, p, 0);
    //   hallway_image.at<uchar>(static_cast<int>(w/2+floor(new_pt.x*25)),static_cast<int>(w/2+floor(new_pt.z*25))) = 1;
  }

  for(set<ORB_SLAM2::MapPoint*>::iterator sit=set_ref_pts.begin(), send=set_ref_pts.end(); sit!=send; sit++)
  {
      if((*sit)->isBad())
          continue;
      cv::Mat pos = (*sit)->GetWorldPos();
      pt new_pt = {pos.at<float>(0), pos.at<float>(1), pos.at<float>(2)};
      //use only x and z
      cv::Point p = cv::Point(static_cast<int>(w/2+floor(new_pt.y*20)),
                          static_cast<int>(w/2+floor(new_pt.z*20)));
      addCircle(hallway_image, p, 0);
    //   hallway_image.at<uchar>(static_cast<int>(w+floor(new_pt.x*25)),static_cast<int>(w+floor(new_pt.z*25))) = 1;
  }

  // Display
  imshow( hallway_window, hallway_image );
  //cv::moveWindow( hallway_window, 0, 200 );

}

void CloudComputer::getPointRanges(ORB_SLAM2::Map* total_map) {

  const vector<ORB_SLAM2::MapPoint*> &map_pts = total_map->GetAllMapPoints();
  const vector<ORB_SLAM2::MapPoint*> &ref_pts = total_map->GetReferenceMapPoints();

  set<ORB_SLAM2::MapPoint*> set_ref_pts(ref_pts.begin(), ref_pts.end());

  pt max_pt_vals = {0,0,0};
  pt min_pt_vals = {0,0,0};

  for(size_t i=0, iend=map_pts.size(); i<iend;i++)
  {
      if(map_pts[i]->isBad() || set_ref_pts.count(map_pts[i]))
          continue;
      cv::Mat pos = map_pts[i]->GetWorldPos();
      pt new_pt = {pos.at<float>(0), pos.at<float>(1), pos.at<float>(2)};
      if(new_pt.x > max_pt_vals.x) max_pt_vals.x = new_pt.x;
      if(new_pt.y > max_pt_vals.y) max_pt_vals.y = new_pt.y;
      if(new_pt.z > max_pt_vals.z) max_pt_vals.z = new_pt.z;
      if(new_pt.x < min_pt_vals.x) min_pt_vals.x = new_pt.x;
      if(new_pt.y < min_pt_vals.y) min_pt_vals.y = new_pt.y;
      if(new_pt.z < min_pt_vals.z) min_pt_vals.z = new_pt.z;

  }

  for(set<ORB_SLAM2::MapPoint*>::iterator sit=set_ref_pts.begin(), send=set_ref_pts.end(); sit!=send; sit++)
  {
      if((*sit)->isBad())
          continue;
      cv::Mat pos = (*sit)->GetWorldPos();
      pt new_pt = {pos.at<float>(0), pos.at<float>(1), pos.at<float>(2)};
      if(new_pt.x > max_pt_vals.x) max_pt_vals.x = new_pt.x;
      if(new_pt.y > max_pt_vals.y) max_pt_vals.y = new_pt.y;
      if(new_pt.z > max_pt_vals.z) max_pt_vals.z = new_pt.z;
      if(new_pt.x < min_pt_vals.x) min_pt_vals.x = new_pt.x;
      if(new_pt.y < min_pt_vals.y) min_pt_vals.y = new_pt.y;
      if(new_pt.z < min_pt_vals.z) min_pt_vals.z = new_pt.z;
  }

  std::cout << "min pt" << min_pt_vals.x << ", "<< min_pt_vals.y << ", "<< min_pt_vals.z << "\n" << std::endl;
  std::cout << "max pt" << max_pt_vals.x << ", "<< max_pt_vals.y << ", "<< max_pt_vals.z << "\n" << std::endl;

}

void CloudComputer::displayCloud(ORB_SLAM2::Map* total_map){


  int fd;
  
  /* write "Hi" to the FIFO */
  fd = open(myfifo, O_WRONLY);
  // write(fd, "Hi", sizeof("Hi"));
  // close(fd);
  //
  // /* remove the FIFO */
  // unlink(myfifo);




  const vector<ORB_SLAM2::MapPoint*> &map_pts = total_map->GetAllMapPoints();
  const vector<ORB_SLAM2::MapPoint*> &ref_pts = total_map->GetReferenceMapPoints();

  set<ORB_SLAM2::MapPoint*> set_ref_pts(ref_pts.begin(), ref_pts.end());

  for(size_t i=0, iend=map_pts.size(); i<iend;i++)
  {
      if(map_pts[i]->isBad() || set_ref_pts.count(map_pts[i]))
          continue;
      cv::Mat pos = map_pts[i]->GetWorldPos();


      // TODO
      pt new_pt = {pos.at<float>(0), pos.at<float>(1), pos.at<float>(2)};
      write(fd, &new_pt, sizeof(pt));
      // pcl_file << pos.at<float>(0) << " "
      //        << pos.at<float>(1) << " "
      //        << pos.at<float>(2) << "\n";
  }

  for(set<ORB_SLAM2::MapPoint*>::iterator sit=set_ref_pts.begin(), send=set_ref_pts.end(); sit!=send; sit++)
  {
      if((*sit)->isBad())
          continue;
      cv::Mat pos = (*sit)->GetWorldPos();

      //TODO
      pt new_pt = {pos.at<float>(0), pos.at<float>(1), pos.at<float>(2)};
      write(fd, &new_pt, sizeof(pt));
      // pcl_file << pos.at<float>(0) << " "
      //        << pos.at<float>(1) << " "
      //        << pos.at<float>(2) << "\n";

  }

  close(fd);

  /* remove the FIFO */
  //unlink(myfifo);

}
