#include "CloudComputer.hpp"
#include <cmath>
#include <iostream>
// #include <fcntl.h>
// #include <sys/stat.h>
// #include <sys/types.h>
// #include <unistd.h>
#include <algorithm>
#include <numeric>

// #include <pcl/io/pcd_io.h>
// #include <pcl/io/ply_io.h>
// #include <pcl/point_cloud.h>
// #include <pcl/console/parse.h>
// #include <pcl/common/transforms.h>
//BROKEN
//#include <pcl/visualization/pcl_visualizer.h>

CloudComputer::CloudComputer(std::vector<IConfigurable::Param> &&params)
    : IConfigurable{{"regression_type"}, std::move(params)}, regression_type{std::stof(getParam("regression_type"))}, how_recent{std::stof(getParam("how_recent"))}, theta{std::stof(getParam("theta"))}, auto_adjust_angle{std::stof(getParam("auto_adjust_angle"))}
{

  //       OLD PCL
  //   myfifo = "myfifo";
  //   /* create the FIFO (named pipe) */
  //   mkfifo(myfifo, 0666);

  theta = (theta * PI) / 180.0;

  w = 500;

  /// Create black empty images
  hallway_image = cv::Mat::zeros(w, w, CV_8UC3);
  wall_alert = false;
  enough_pts_already = false;
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

cv::Vec4f CloudComputer::convertLine2D(cv::Vec6f a3dline)
{
  cv::Vec4f a2dline = cv::Vec4f(a3dline[0], a3dline[2], a3dline[3], a3dline[5]);
  return a2dline;
}

cv::Point CloudComputer::convertPoint2D(cv::Point3f a3dpoint)
{
  // convention is to keep X and Z dimensions
  cv::Point a2dpoint = cv::Point(a3dpoint.x, a3dpoint.z);
  return a2dpoint;
}

cv::Point CloudComputer::adjustPtForDisp(cv::Point p)
{
  return cv::Point(static_cast<int>(w / 2 + floor(p.x * 20)),
                   static_cast<int>(w / 2 - floor(p.y * 20)));
}

void CloudComputer::display2D(ORB_SLAM2::Map *total_map)
{

  char hallway_window[] = "Hallway Projection";
  // set background to white if short term and long term angle is above a certain threshhold
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
    cv::Mat pos;
    if (auto_adjust_angle)
    {
      pos = autoRotate(map_pts[i]->GetWorldPos());
    }
    else
    {
      pos = rotateWithTheta(map_pts[i]->GetWorldPos());
    }

    pt new_pt = {pos.at<float>(0), pos.at<float>(1), pos.at<float>(2)};
    //use only x and z
    cv::Point p_raw = cv::Point(new_pt.x, new_pt.z);
    raw_pts_vector.push_back(p_raw);
    cv::Point p = cv::Point(static_cast<int>(w / 2 + floor(new_pt.x * 20)),
                            static_cast<int>(w / 2 - floor(new_pt.z * 20)));
    // make 3D point
    if (!enough_pts_already)
    {
      cv::Point3f p3 = cv::Point3f(static_cast<int>(w / 2 + floor(new_pt.x * 20)),
                                   static_cast<int>(w / 2 + floor(new_pt.y * 20)),
                                   static_cast<int>(w / 2 - floor(new_pt.z * 20)));
      pts_vector_3d.push_back(p3);
    }
    pts_vector.push_back(p);
    addCircle(hallway_image, p, 0);
  }

  for (set<ORB_SLAM2::MapPoint *>::iterator sit = set_ref_pts.begin(), send = set_ref_pts.end(); sit != send; sit++)
  {
    if ((*sit)->isBad())
      continue;
    cv::Mat pos;
    if (auto_adjust_angle)
    {
      pos = autoRotate((*sit)->GetWorldPos());
    }
    else
    {
      pos = rotateWithTheta((*sit)->GetWorldPos());
    }
    pt new_pt = {pos.at<float>(0), pos.at<float>(1), pos.at<float>(2)};
    //use only x and z
    cv::Point p_raw = cv::Point(new_pt.x, new_pt.z);
    raw_pts_vector.push_back(p_raw);
    cv::Point p = cv::Point(static_cast<int>(w / 2 + floor(new_pt.x * 20)),
                            static_cast<int>(w / 2 - floor(new_pt.z * 20)));
    // make 3D point
    if (!enough_pts_already)
    {
      cv::Point3f p3 = cv::Point3f(static_cast<int>(w / 2 + floor(new_pt.x * 20)),
                                   static_cast<int>(w / 2 + floor(new_pt.y * 20)),
                                   static_cast<int>(w / 2 - floor(new_pt.z * 20)));
      pts_vector_3d.push_back(p3);
    }
    pts_vector.push_back(p);
    addCircle(hallway_image, p, 0);
  }

  // long term memory
  if (pts_vector.size() > (how_recent * how_recent))
  {

    //REVIEW:
    if (!enough_pts_already)
    {
      enough_pts_already = true;
    }
    //std::cout << "[COMPASS]: " << compass_line << endl;
    //REVIEW:

    // older -- n^2
    // cv::Vec4f long_term_line;  // NOW PRIVATE VAR
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
    // normalize denominator, might not be needed
    // float bottom = sqrt(pow(long_term_line[0], 2) + pow(long_term_line[1], 2)) *
    //                sqrt(pow(short_term_line[0], 2) + pow(short_term_line[1], 2));
    float dir_theta = acos(top) * 180 / PI;
    if (dir_theta > 90)
    {
      dir_theta = 180 - dir_theta;
    }
    if (dir_theta > 55)
    {
      wall_alert = true;
    }
    else
    {
      wall_alert = false;
    }
  }
  // short term memory
  else if (pts_vector.size() > how_recent)
  {
    cv::Vec4f short_term_line;
    vector<cv::Point> recent_pts_vector(pts_vector.end() - how_recent, pts_vector.end());
    cv::fitLine(recent_pts_vector, short_term_line, regression_type, 0, 0.01, 0.01);
    drawLine(hallway_image, short_term_line, 1, cv::Scalar(0, 255, 0));
  }
  else
  {
    cv::Vec4f total_line;
    cv::fitLine(pts_vector, total_line, regression_type, 0, 0.01, 0.01);
    drawLine(hallway_image, total_line, 1, cv::Scalar(0, 255, 0));
  }
  // Display
  imshow(hallway_window, hallway_image);
}

void CloudComputer::calcHistogram()
{
  // number of bins k
  int k = w / 10;
  // int k = 10;
  // init hist
  vector<int> buckets(k);
  // histogram of X axis
  // find max and min values
  int min = pts_vector.at(0).x;
  int max = pts_vector.at(0).x;
  for (int i = 0; i < pts_vector.size(); i++)
  {
    if (pts_vector.at(i).x < min)
    {
      min = pts_vector.at(i).x;
    }
    if (pts_vector.at(i).x > max)
    {
      max = pts_vector.at(i).x;
    }
  }
  int range = max - min;
  int b_width = range / k;
  // start and end of current bucket being populated
  int start, end;
  for (int j = 0; j < buckets.size(); j++)
  {
    start = min + (b_width * j);
    end = start + b_width;
    buckets[j] = std::count_if(pts_vector.begin(), pts_vector.end(), [start, end](const auto &cur_pt) {
      if (cur_pt.x >= start && cur_pt.x < end)
      {
        return true;
      }
      else
      {
        return false;
      }
    });
  }
  // for (int m = 0; m < buckets.size(); m++)
  // {
  //   std::cout << "[" << m << "]:" << buckets.at(m) << " ";
  // }
  // std::cout << endl;

  vector<int> diffs(k);
  std::adjacent_difference(buckets.begin(), buckets.end(), diffs.begin());
  int max_diff = std::max_element(diffs.begin(), diffs.end()) - diffs.begin();
  int left_hall = min + (b_width * (max_diff));
  int min_diff = std::min_element(diffs.begin(), diffs.end()) - diffs.begin();
  int right_hall = min + (b_width * (min_diff));
  // for (int i = 0; i < diffs.size(); i++)
  // {
  //   std::cout << diffs[i] << std::endl;
  // }
  // std::cout << "Left: " << left_hall << " Right: " << right_hall << " max diff: " << max_diff << " min diff: " << min_diff << std::endl;

  // print lines for hall boundaries
  cv::Point startPoint;
  startPoint.x = left_hall; // x0
  startPoint.y = 1;         // y0
  cv::Point endPoint;
  endPoint.x = left_hall; // x0
  endPoint.y = w - 5;     // y0
  // cv::clipLine(cv::Size(w, w), startPoint, endPoint);
  cv::line(hallway_image, startPoint, endPoint, cv::Scalar(0, 255, 255), 1, 8, 0);
  startPoint.x = right_hall; // x0
  startPoint.y = 1;          // y0
  endPoint.x = right_hall;   // x0
  endPoint.y = w - 5;        // y0
  // cv::clipLine(cv::Size(w, w), startPoint, endPoint);
  cv::line(hallway_image, startPoint, endPoint, cv::Scalar(0, 255, 255), 1, 8, 0);
}

// ONLY RUN AFTER how_recent^2 pts have been found
// void CloudComputer::detectFacingWall()
// {
//   // number of bins k
//   int k = sqrt(how_recent);
//   // int k = 10;
//   // init hist
//   vector<int> buckets(k);
//   // histogram of Z axis
//   // find max and min values
//   int min = pts_vector.at(0).z;
//   int max = pts_vector.at(0).z;
//   for (int i = 0; i < (pts_vector.size() - pow(how_recent, 2)); i++)
//   {
//     if (pts_vector.at(i).z < min)
//     {
//       min = pts_vector.at(i).z;
//     }
//     if (pts_vector.at(i).z > max)
//     {
//       max = pts_vector.at(i).z;
//     }
//   }
//   int range = max - min;
//   int b_width = range / k;
//   // start and end of current bucket being populated
//   int start, end;
//   for (int j = 0; j < buckets.size(); j++)
//   {
//     start = min + (b_width * j);
//     end = start + b_width;
//     buckets[j] = std::count_if((pts_vector.end() - pow(how_recent, 2)), pts_vector.end(), [start, end](const auto &cur_pt) {
//       if (cur_pt.z >= start && cur_pt.z < end)
//       {
//         return true;
//       }
//       else
//       {
//         return false;
//       }
//     });
//   }
//   for (int m = 0; m < buckets.size(); m++)
//   {
//     std::cout << "[" << m << "]:\t" << buckets.at(m) << " ";
//   }
//   std::cout << endl;

//   // vector<int> diffs(k);
//   // std::adjacent_difference(buckets.begin(), buckets.end(), diffs.begin());
//   // int max_diff = std::max_element(diffs.begin(), diffs.end()) - diffs.begin();
//   // int left_hall = min + (b_width * (max_diff));
//   // int min_diff = std::min_element(diffs.begin(), diffs.end()) - diffs.begin();
//   // int right_hall = min + (b_width * (min_diff));
//   // for (int i = 0; i < diffs.size(); i++)
//   // {
//   //   std::cout << diffs[i] << std::endl;
//   // }
//   // std::cout << "Left: " << left_hall << " Right: " << right_hall << " max diff: " << max_diff << " min diff: " << min_diff << std::endl;

//   // // print lines for hall boundaries
//   // cv::Point startPoint;
//   // startPoint.x = left_hall; // x0
//   // startPoint.y = 1;         // y0
//   // cv::Point endPoint;
//   // endPoint.x = left_hall; // x0
//   // endPoint.y = w - 5;     // y0
//   // // cv::clipLine(cv::Size(w, w), startPoint, endPoint);
//   // cv::line(hallway_image, startPoint, endPoint, cv::Scalar(0, 255, 255), 1, 8, 0);
//   // startPoint.x = right_hall; // x0
//   // startPoint.y = 1;          // y0
//   // endPoint.x = right_hall;   // x0
//   // endPoint.y = w - 5;        // y0
//   // // cv::clipLine(cv::Size(w, w), startPoint, endPoint);
//   // cv::line(hallway_image, startPoint, endPoint, cv::Scalar(0, 255, 255), 1, 8, 0);
// }

cv::Vec4f CloudComputer::getGreenLine()
{
  cv::Vec4f greenLine{0.f, 0.f, 0.f, 0.f};
  
  if(enough_pts_already) {
    cv::fitLine(raw_pts_vector, greenLine, regression_type, 0, 0.01, 0.01);
  }
  return greenLine;
}

cv::Mat CloudComputer::rotateWithTheta(cv::Mat pos)
{
  cv::Mat rot_matrix = (cv::Mat_<float>(3, 3) << 1, 0, 0,
                        0, cos(theta), -sin(theta),
                        0, sin(theta), cos(theta));
  return rot_matrix * pos;
}

cv::Mat CloudComputer::autoRotate(cv::Mat pos)
{
  // if not enough pts yet, do nothing
  if (!enough_pts_already)
  {
    return pos;
  }

  // if enough pts found, check if line has been calculated
  // if line not calculated, do that and update old pts...
  if (!compass_line[0])
  {
    // regresion line on 3d pts found before enough_pts_already flag thrown
    cv::fitLine(pts_vector_3d, compass_line, regression_type, 0, 0.01, 0.01);
    // create vector on X Z plane
    cv::Mat xz_vector = (cv::Mat_<float>(1, 3) << compass_line[0], 0, compass_line[2]);
    // theta between lines
    auto_theta = acos(((xz_vector.at<float>(0) * compass_line[0]) + (xz_vector.at<float>(1) * compass_line[1]) + (xz_vector.at<float>(2) * compass_line[2])) / (sqrt(pow(compass_line[0], 2) + pow(compass_line[1], 2) + pow(compass_line[2], 2)) * sqrt(pow(xz_vector.at<float>(0), 2) + pow(xz_vector.at<float>(1), 2) + pow(xz_vector.at<float>(2), 2))));
    cv::Mat rot_matrix_init = (cv::Mat_<float>(3, 3) << 1, 0, 0,
                               0, cos(auto_theta), -sin(auto_theta),
                               0, sin(auto_theta), cos(auto_theta));
    pts_vector.clear();
    std::cout << "[BEGIN UPDATE OF OLD PTS]" << std::endl;
    std::cout << (auto_theta * 180 / PI) << std::endl;

    for (int i = 0; i < pts_vector_3d.size(); i++)
    {
      cv::Mat pt_i = (cv::Mat_<float>(3, 1) << pts_vector_3d.at(i).x, pts_vector_3d.at(i).y, pts_vector_3d.at(i).z);
      // std::cout << pt_i << std::endl;
      pt_i = rot_matrix_init * pt_i;
      // std::cout << "[a Matrix multiplied successfully]" << std::endl;

      pts_vector_3d.at(i).x = static_cast<int>(pt_i.at<float>(0));
      pts_vector_3d.at(i).y = static_cast<int>(pt_i.at<float>(1));
      pts_vector_3d.at(i).z = static_cast<int>(pt_i.at<float>(2));

      // update 2d point vector
      cv::Point p = cv::Point(static_cast<int>(pts_vector_3d.at(i).x),
                              static_cast<int>(pts_vector_3d.at(i).z));
      pts_vector.push_back(p);
    }
    std::cout << "[FINISHED UPDATE OF OLD PTS]" << std::endl;
  }
  //return current point after rotation made
  cv::Mat rot_matrix = (cv::Mat_<float>(3, 3) << 1, 0, 0,
                        0, cos(auto_theta), -sin(auto_theta),
                        0, sin(auto_theta), cos(auto_theta));
  return rot_matrix * pos;
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
