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

CloudComputer::CloudComputer(std::vector<IConfigurable::Param> &&params)
    : IConfigurable{{"regression_type"}, std::move(params)}, regression_type{std::stof(getParam("regression_type"))}, how_recent{std::stof(getParam("how_recent"))}
{

  //       OLD PCL
  //   myfifo = "myfifo";
  //   /* create the FIFO (named pipe) */
  //   mkfifo(myfifo, 0666);

  w = 500;

  /// Create black empty images
  hallway_image = cv::Mat::zeros(w, w, CV_8UC3);
  wall_alert = false;
}

void CloudComputer::addCircle(cv::Mat img, cv::Point center, int color)
{
  int thickness = -1;
  int lineType = 8;

  circle(img,
         center,
         0.1,
         cv::Scalar(color, 0, 255),
         thickness,
         lineType);
}

void CloudComputer::drawLine(cv::Mat img, cv::Vec4f line, int thickness, cv::Scalar color)
{

  // calculate start point
  cv::Point startPoint;
  startPoint.x = line[2] - w * line[0]; // x0
  startPoint.y = line[3] - w * line[1]; // y0
  // calculate end point
  cv::Point endPoint;
  endPoint.x = line[2] + w * line[0]; //x[1]
  endPoint.y = line[3] + w * line[1]; //y[1]

  // draw overlay of bottom lines on image
  cv::clipLine(cv::Size(w, w), startPoint, endPoint);
  cv::line(img, startPoint, endPoint, color, thickness, 8, 0);
}

void CloudComputer::display2D(ORB_SLAM2::Map *total_map)
{

  char hallway_window[] = "Hallway Projection";
  if (wall_alert)
  {
    hallway_image = cv::Mat::ones(w, w, CV_8UC3);
    hallway_image = cv::Scalar(255, 255, 255);
  }
  else
  {
    hallway_image = cv::Mat::zeros(w, w, CV_8UC3);
  }

  const vector<ORB_SLAM2::MapPoint *> &map_pts = total_map->GetAllMapPoints();
  const vector<ORB_SLAM2::MapPoint *> &ref_pts = total_map->GetReferenceMapPoints();

  set<ORB_SLAM2::MapPoint *> set_ref_pts(ref_pts.begin(), ref_pts.end());

  for (size_t i = 0, iend = map_pts.size(); i < iend; i++)
  {
    if (map_pts[i]->isBad() || set_ref_pts.count(map_pts[i]))
      continue;
    cv::Mat pos = map_pts[i]->GetWorldPos();
    pt new_pt = {pos.at<float>(0), pos.at<float>(1), pos.at<float>(2)};
    //use only x and z
    cv::Point p = cv::Point(static_cast<int>(w / 2 + floor(new_pt.x * 20)),
                            static_cast<int>(w / 2 - floor(new_pt.z * 20)));
    pts_vector.push_back(p);
    addCircle(hallway_image, p, 0);
    //   hallway_image.at<uchar>(static_cast<int>(w/2+floor(new_pt.x*25)),static_cast<int>(w/2+floor(new_pt.z*25))) = 1;
  }

  for (set<ORB_SLAM2::MapPoint *>::iterator sit = set_ref_pts.begin(), send = set_ref_pts.end(); sit != send; sit++)
  {
    if ((*sit)->isBad())
      continue;
    cv::Mat pos = (*sit)->GetWorldPos();
    pt new_pt = {pos.at<float>(0), pos.at<float>(1), pos.at<float>(2)};
    //use only x and z
    cv::Point p = cv::Point(static_cast<int>(w / 2 + floor(new_pt.x * 20)),
                            static_cast<int>(w / 2 - floor(new_pt.z * 20)));
    pts_vector.push_back(p);
    addCircle(hallway_image, p, 0);
    //   hallway_image.at<uchar>(static_cast<int>(w+floor(new_pt.x*25)),static_cast<int>(w+floor(new_pt.z*25))) = 1;
  }

  // long term memory
  if (pts_vector.size() > (how_recent * how_recent))
  {
    // older -- n^2
    cv::Vec4f long_term_line;
    vector<cv::Point> longterm_pts_vector(pts_vector.end() - (how_recent * how_recent), pts_vector.end());
    cv::fitLine(longterm_pts_vector, long_term_line, regression_type, 0, 0.01, 0.01);
    drawLine(hallway_image, long_term_line, 1, cv::Scalar(0, 255, 0));
    // newer -- n
    cv::Vec4f short_term_line;
    vector<cv::Point> recent_pts_vector(pts_vector.end() - how_recent, pts_vector.end());
    cv::fitLine(recent_pts_vector, short_term_line, regression_type, 0, 0.01, 0.01);
    drawLine(hallway_image, short_term_line, 1, cv::Scalar(255, 0, 0));
    // TODO
    // theta between lines
    float top = (long_term_line[0] * short_term_line[0]) + (long_term_line[1] * short_term_line[1]);
    float bottom = sqrt(pow(long_term_line[0], 2) + pow(long_term_line[1], 2)) *
                   sqrt(pow(short_term_line[0], 2) + pow(short_term_line[1], 2));
    float theta = cos(top);
    if (theta > 0.75)
    {
      std::cout << "[THETA]: " << theta << std::endl;
      wall_alert = true;
    }
    else
    {
      wall_alert = false;
    }
    // std::cout << "[Long  T]: " << long_term_line << std::endl;
    // std::cout << "[Short T]: " << short_term_line << std::endl;
  }
  // short term memory
  else if (pts_vector.size() > how_recent)
  {
    cv::Vec4f short_term_line;
    vector<cv::Point> recent_pts_vector(pts_vector.end() - how_recent, pts_vector.end());
    cv::fitLine(recent_pts_vector, short_term_line, regression_type, 0, 0.01, 0.01);
    drawLine(hallway_image, short_term_line, 1, cv::Scalar(255, 0, 0));
  }
  else
  {
    cv::Vec4f total_line;
    cv::fitLine(pts_vector, total_line, regression_type, 0, 0.01, 0.01);
    drawLine(hallway_image, total_line, 1, cv::Scalar(255, 0, 0));
  }

  // cv::line(hallway_image, cv::Point(myLine[2] - myLine[0] * 15, myLine[3] - myLine[1] * 15),
  //          cv::Point(myLine[2] + myLine[0] * 30,
  //                    myLine[3] + myLine[1] * 30),
  //          cv::Scalar(255, 0, 0), 1, 8, 0);

  //std::cout << "[LINE VECTOR]: " << myLine << std::endl;

  // Display
  imshow(hallway_window, hallway_image);
  //cv::moveWindow( hallway_window, 0, 200 );
}

void CloudComputer::getPointRanges(ORB_SLAM2::Map *total_map)
{

  const vector<ORB_SLAM2::MapPoint *> &map_pts = total_map->GetAllMapPoints();
  const vector<ORB_SLAM2::MapPoint *> &ref_pts = total_map->GetReferenceMapPoints();

  set<ORB_SLAM2::MapPoint *> set_ref_pts(ref_pts.begin(), ref_pts.end());

  pt max_pt_vals = {0, 0, 0};
  pt min_pt_vals = {0, 0, 0};

  for (size_t i = 0, iend = map_pts.size(); i < iend; i++)
  {
    if (map_pts[i]->isBad() || set_ref_pts.count(map_pts[i]))
      continue;
    cv::Mat pos = map_pts[i]->GetWorldPos();
    pt new_pt = {pos.at<float>(0), pos.at<float>(1), pos.at<float>(2)};
    if (new_pt.x > max_pt_vals.x)
      max_pt_vals.x = new_pt.x;
    if (new_pt.y > max_pt_vals.y)
      max_pt_vals.y = new_pt.y;
    if (new_pt.z > max_pt_vals.z)
      max_pt_vals.z = new_pt.z;
    if (new_pt.x < min_pt_vals.x)
      min_pt_vals.x = new_pt.x;
    if (new_pt.y < min_pt_vals.y)
      min_pt_vals.y = new_pt.y;
    if (new_pt.z < min_pt_vals.z)
      min_pt_vals.z = new_pt.z;
  }

  for (set<ORB_SLAM2::MapPoint *>::iterator sit = set_ref_pts.begin(), send = set_ref_pts.end(); sit != send; sit++)
  {
    if ((*sit)->isBad())
      continue;
    cv::Mat pos = (*sit)->GetWorldPos();
    pt new_pt = {pos.at<float>(0), pos.at<float>(1), pos.at<float>(2)};
    if (new_pt.x > max_pt_vals.x)
      max_pt_vals.x = new_pt.x;
    if (new_pt.y > max_pt_vals.y)
      max_pt_vals.y = new_pt.y;
    if (new_pt.z > max_pt_vals.z)
      max_pt_vals.z = new_pt.z;
    if (new_pt.x < min_pt_vals.x)
      min_pt_vals.x = new_pt.x;
    if (new_pt.y < min_pt_vals.y)
      min_pt_vals.y = new_pt.y;
    if (new_pt.z < min_pt_vals.z)
      min_pt_vals.z = new_pt.z;
  }

  std::cout << "min pt" << min_pt_vals.x << ", " << min_pt_vals.y << ", " << min_pt_vals.z << "\n"
            << std::endl;
  std::cout << "max pt" << max_pt_vals.x << ", " << max_pt_vals.y << ", " << max_pt_vals.z << "\n"
            << std::endl;
}

void CloudComputer::displayCloud(ORB_SLAM2::Map *total_map)
{

  int fd;

  /* write "Hi" to the FIFO */
  fd = open(myfifo, O_WRONLY);
  // write(fd, "Hi", sizeof("Hi"));
  // close(fd);
  //
  // /* remove the FIFO */
  // unlink(myfifo);

  const vector<ORB_SLAM2::MapPoint *> &map_pts = total_map->GetAllMapPoints();
  const vector<ORB_SLAM2::MapPoint *> &ref_pts = total_map->GetReferenceMapPoints();

  set<ORB_SLAM2::MapPoint *> set_ref_pts(ref_pts.begin(), ref_pts.end());

  for (size_t i = 0, iend = map_pts.size(); i < iend; i++)
  {
    if (map_pts[i]->isBad() || set_ref_pts.count(map_pts[i]))
      continue;
    cv::Mat pos = map_pts[i]->GetWorldPos();

    // TODO
    pt new_pt = {pos.at<float>(0), pos.at<float>(1), pos.at<float>(2)};
    write(fd, &new_pt, sizeof(pt));
    // pcl_file << pos.at<float>(0) << " "
    //        << pos.at<float>(1) << " "
    //        << pos.at<float>(2) << "\n";
  }

  for (set<ORB_SLAM2::MapPoint *>::iterator sit = set_ref_pts.begin(), send = set_ref_pts.end(); sit != send; sit++)
  {
    if ((*sit)->isBad())
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
