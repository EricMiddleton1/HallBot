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
}

void CloudComputer::displayCloud(ORB_SLAM2::Map* total_map){


  int fd;
  char * myfifo = "myfifo";

  /* create the FIFO (named pipe) */
  mkfifo(myfifo, 0666);

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
  unlink(myfifo);

}
