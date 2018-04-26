#include "CloudComputer.hpp"
#include <cmath>
#include <iostream>
#include <algorithm>
#include <numeric>

CloudComputer::CloudComputer(std::vector<IConfigurable::Param> &&params)
    : IConfigurable{{"regression_type"}, std::move(params)}, regression_type{std::stof(getParam("regression_type"))}, how_recent{std::stof(getParam("how_recent"))}, theta{std::stof(getParam("theta"))}, auto_adjust_angle{std::stof(getParam("auto_adjust_angle"))}
{
  theta = (theta * PI) / 180.0;
  w = 500;
  // Create black empty images
  hallway_image = cv::Mat::zeros(w, w, CV_8UC3);
  bw_hallway_image = cv::Mat::zeros(w, w, CV_8UC1);
  wall_alert = false;
  enough_pts_already = false;
  adjusted_3d = false;
  stored_green_theta = -1000;
  green_theta_count = 50;
}

void CloudComputer::addCircle(cv::Mat img, cv::Point center, int color)
{
  int thickness = -1;
  int lineType = 8;
  circle(img,
         center,
         0.1,
         cv::Scalar(color, 255, 255),
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
  // preserve only X and Z
  cv::Vec4f a2dline = cv::Vec4f(a3dline[0], a3dline[2], a3dline[3], a3dline[5]);
  return a2dline;
}

int CloudComputer::convertPoint2D(auto num, int axis)
{
  if (!axis)
  {
    auto output = static_cast<int>(w / 2 + (num * 100));
    return output;
  }
  else
  {
    auto output = static_cast<int>(9.5 * w / 10 - (num * 100));
    return output;
  }
}

float CloudComputer::undoConvertPoint2D(int num, int axis)
{
  if (!axis)
  {
    auto output = ((num - (w / 2)) / 100.f);
    return output;
  }
  else
  {
    auto output = ((num - (9.5 * w / 10)) / 100.f);
    return output;
  }
}

void CloudComputer::displayPoints()
{
  bw_hallway_image = cv::Mat::zeros(w, w, CV_8UC1); // reset image
  float green_theta = getGreenTheta();
  // rot matrix based on green theta
  cv::Mat rot_matrix = (cv::Mat_<float>(3, 3) << cos(green_theta), 0, sin(green_theta),
                        0, 1, 0,
                        -sin(green_theta), 0, cos(green_theta));
  for (int i = 0; i < raw_mat_vector.size(); i++)
  {
    cv::Mat pt_i = (cv::Mat_<float>(3, 1) << raw_mat_vector.at(i).at<float>(0), raw_mat_vector.at(i).at<float>(1), raw_mat_vector.at(i).at<float>(2));
    pt_i = rot_matrix * pt_i;
    cv::Point p = cv::Point(convertPoint2D(pt_i.at<float>(0), 0), convertPoint2D(pt_i.at<float>(2), 1));
    addCircle(hallway_image, p, 255);
    circle(bw_hallway_image, p, 0.1, cv::Scalar(255), -1, 8);
  }
}

void CloudComputer::updatePointVectors(cv::Mat pos)
{
  cv::Mat clone_pos = pos.clone();
  //All point vectors are stored RAW
  raw_mat_vector.push_back(clone_pos);
  //2d point vector
  cv::Point p = cv::Point(static_cast<int>(clone_pos.at<float>(0)), static_cast<int>(clone_pos.at<float>(2)));
  pts_vector.push_back(p);
  //3d point vector
  cv::Point3f p3 = cv::Point3f(static_cast<int>(clone_pos.at<float>(0)), static_cast<int>(clone_pos.at<float>(1)), static_cast<int>(clone_pos.at<float>(2)));
  pts_vector_3d.push_back(p3);
}

void CloudComputer::clearPointVectors()
{
  raw_mat_vector.clear();
  pts_vector.clear();
  pts_vector_3d.clear();
}

void CloudComputer::makeGreenLine()
{
  //HACK need to add toggle for how many pts to consider...
  if (pts_vector.size() > 0)
  {
    cv::fitLine(pts_vector, long_term_line, regression_type, 0, 0.01, 0.01);
  }
}

void CloudComputer::steerPoints()
{
  float green_theta = getGreenTheta();
  // rot matrix based on green theta
  cv::Mat rot_matrix = (cv::Mat_<float>(3, 3) << cos(green_theta), 0, sin(green_theta),
                        0, 1, 0,
                        -sin(green_theta), 0, cos(green_theta));
  vector<cv::Mat> raw_clone(raw_mat_vector);
  clearPointVectors();
  // std::cout << "[BEGIN STEER]" << std::endl;
  for (int i = 0; i < raw_clone.size(); i++)
  {
    cv::Mat pt_i = (cv::Mat_<float>(3, 1) << raw_clone.at(i).at<float>(0), raw_clone.at(i).at<float>(1), raw_clone.at(i).at<float>(2));
    pt_i = rot_matrix * pt_i;
    updatePointVectors(pt_i);
  }
  // std::cout << "[FINISHED STEER]" << std::endl;
}

void CloudComputer::displayCamera()
{
  float green_theta = getGreenTheta();
  // rot matrix based on green theta
  cv::Mat rot_matrix = (cv::Mat_<float>(3, 3) << cos(green_theta), 0, sin(green_theta),
                        0, 1, 0,
                        -sin(green_theta), 0, cos(green_theta));
  cv::Mat pt_i = (cv::Mat_<float>(3, 1) << cam_pos[0], 0, cam_pos[1]);
  pt_i = rot_matrix * pt_i;
  cv::Point p = cv::Point(convertPoint2D(pt_i.at<float>(0), 0),
                          convertPoint2D(pt_i.at<float>(2), 1));
  circle(hallway_image,
         p,
         2,
         cv::Scalar(0, 128, 255),
         -1,
         8);
}

//TODO
cv::Vec2f CloudComputer::getHallPosition()
{
  cv::Vec2f output = cv::Vec2f(0, 0);
  output[0] = cam_pos[0] - undoConvertPoint2D(left_wall, 0);
  output[1] = undoConvertPoint2D(right_wall, 0) - cam_pos[0];

  return output;
}

float CloudComputer::getWidth()
{
  auto output = undoConvertPoint2D(right_wall, 0) - undoConvertPoint2D(left_wall, 0);
  return output;
}

void CloudComputer::display2D(ORB_SLAM2::Map *total_map)
{
  // diplay window
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
  clearPointVectors();
  // get map points and make iterator for reference points.
  const vector<ORB_SLAM2::MapPoint *> &map_pts = total_map->GetAllMapPoints();
  const vector<ORB_SLAM2::MapPoint *> &ref_pts = total_map->GetReferenceMapPoints();
  set<ORB_SLAM2::MapPoint *> set_ref_pts(ref_pts.begin(), ref_pts.end());
  // Map points loop
  for (size_t i = 0, iend = map_pts.size(); i < iend; i++)
  {
    if (map_pts[i]->isBad() || set_ref_pts.count(map_pts[i]))
      continue;
    cv::Mat pos;
    if (auto_adjust_angle)
    {
      pos = map_pts[i]->GetWorldPos().clone();
      pos = autoRotate(pos);
    }
    else
    {
      pos = map_pts[i]->GetWorldPos().clone();
      pos = rotateWithTheta(pos);
    }
    updatePointVectors(pos);
  }
  // // Refernece points loop
  for (set<ORB_SLAM2::MapPoint *>::iterator sit = set_ref_pts.begin(), send = set_ref_pts.end(); sit != send; sit++)
  {
    if ((*sit)->isBad())
      continue;
    cv::Mat pos2;
    if (auto_adjust_angle)
    {
      pos2 = (*sit)->GetWorldPos().clone();
      pos2 = autoRotate(pos2);
    }
    else
    {
      pos2 = (*sit)->GetWorldPos().clone();
      pos2 = rotateWithTheta(pos2);
    }
    updatePointVectors(pos2);
  }
  //regression line
  makeGreenLine();
  // width histogram
  curHallwayHistogram();
  // display all 2D pts
  displayPoints();
  // new style green reference line
  cv::Vec4f greenline = cv::Vec4f(0, 1, w / 2, w / 2);
  drawLine(hallway_image, greenline, 1, cv::Scalar(0, 255, 0));
  // display camera
  displayCamera();
  distToFacingWall();
  // hallwayDetector(); // IMPORTANT: must go after dist to facing wall
  // std::cout << "[dist to facing wall]: " << distToFacingWall() << std::endl;
  // Display
  imshow(hallway_window, hallway_image);
}

void CloudComputer::hallwayDetector()
{
  cv::Mat structuring_elem = (cv::Mat_<int>(3, 3) << 0, 1, 0,
                              1, 1, 1,
                              0, 1, 0);
  dilate(bw_hallway_image, bw_hallway_image, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3)), cv::Point(-1, -1), 1, 1, 1);
  erode(bw_hallway_image, bw_hallway_image, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3)), cv::Point(-1, -1), 2, 1, 1);
  dilate(bw_hallway_image, bw_hallway_image, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3)), cv::Point(-1, -1), 1, 1, 1);
  char hallway_window[] = "HALLWAY DETECTOR";
  imshow(hallway_window, bw_hallway_image);
  bw_hallway_image = cv::Mat::zeros(w, w, CV_8UC1); // reset image
}

void CloudComputer::curHallwayHistogram()
{
  std::vector<int> proj = getProjection(bw_hallway_image, 0);
  int sum = std::accumulate(proj.begin(), proj.end(), 0);
  int total = std::count_if(proj.begin(), proj.end(), [](int value) {
    if (value)
    {
      return true;
    }
    else
    {
      return false;
    }
  });
  if (!total)
  {
    return;
  }
  int average = sum / total;
  // char hist_window[] = "WIDTH HISTOGRAM YO";
  // imshow(hist_window, bw_hallway_image);
  int cutoff = average; //toggle if necessary
  // left wall
  left_wall = 0;
  int i = proj.size() - 1;
  while (!left_wall && i >= 0)
  {
    if (proj[i] > cutoff)
    {
      left_wall = i;
    }
    i--;
  }
  // right wall
  right_wall = 0;
  i = 0;
  while (!right_wall && i <= proj.size() - 1)
  {
    if (proj[i] > cutoff)
    {
      // std::cout << proj[i] << " > " << average << " at: " << i << std::endl;
      right_wall = i;
    }
    i++;
  }
  //REVIEW
  // correct wall locations...
  // left_wall = w - left_wall;
  // right_wall = w - right_wall;
  // display walls
  cv::Vec4f left_line = cv::Vec4f(0, 1, left_wall, 5);
  drawLine(hallway_image, left_line, 1, cv::Vec3b(10, 255, 255));
  cv::Vec4f right_line = cv::Vec4f(0, 1, right_wall, 5);
  drawLine(hallway_image, right_line, 1, cv::Vec3b(10, 255, 255));
}

std::vector<int> CloudComputer::getProjection(const cv::Mat &image, int vertical)
{

  if (vertical)
  {
    std::vector<int> projection(image.rows);
    for (unsigned int i = 0; i < projection.size(); ++i)
    {
      projection[i] = cv::countNonZero(image(cv::Rect(0, i, image.cols, 1)));
    }
    return projection;
  }
  else // horizontal
  {
    std::vector<int> projection(image.cols);
    for (unsigned int i = 0; i < projection.size(); ++i)
    {
      projection[i] = cv::countNonZero(image(cv::Rect(i, 0, 1, image.rows)));
    }
    return projection;
  }
}

float CloudComputer::distToFacingWall()
{
  std::vector<int> proj = getProjection(bw_hallway_image, 1);
  int sum = std::accumulate(proj.begin(), proj.end(), 0);
  int total = std::count_if(proj.begin(), proj.end(), [](int value) {
    if (value)
    {
      return true;
    }
    else
    {
      return false;
    }
  });
  if (!total)
  {
    return -1;
  }
  int average = sum / total;
  // loop from the back ;)
  bool found_wall = false;
  int k = 0;
  while (!found_wall && k < proj.size())
  {
    if (proj[k] > average)
    {
      // convert to raw coord
      // std::cout << k << " " << left_wall << " " << right_wall << std::endl;
      // calculate start point
      cv::Point startPoint;
      startPoint.x = left_wall;
      startPoint.y = k;
      // calculate end point
      cv::Point endPoint;
      endPoint.x = right_wall;
      endPoint.y = k;
      cv::line(hallway_image, startPoint, endPoint, cv::Scalar(255, 0, 0), 1, 8, 0);
      // cv::line(bw_hallway_image, cv::Point(static_cast<int>(left_wall), static_cast<int>(k)),
      //      cv::Point(static_cast<int>(right_wall), static_cast<int>(k)), cv::Scalar(255), 1, 8, 0);
      // float raw_y = (k - (3.f * w / 4.f)) / 50.f;
      //HACK PLS DOUBLE CHECK
      float raw_y = undoConvertPoint2D(k, 1);
      // rotate camera
      float green_theta = getGreenTheta();
      // rot matrix based on green theta
      cv::Mat rot_matrix = (cv::Mat_<float>(2, 2) << cos(green_theta), -sin(green_theta),
                            sin(green_theta), cos(green_theta));
      cv::Mat cam = (cv::Mat_<float>(2, 1) << cam_pos[0], cam_pos[1]);
      cv::Mat rot_cam = rot_matrix * cam;
      float cam_y = cam.at<float>(1);
      //TODO check for negative output??
      // std::cout << "wall: " << raw_y << " cam: " << cam_pos << endl;
      return (-raw_y - cam_y);
    }
    k++;
  }
  return -1;
}

cv::Vec4f CloudComputer::getGreenLine()
{
  return long_term_line;
}

float CloudComputer::getGreenTheta()
{
  // angle between green line and 2D y axis
  // float green_theta = atan2(-long_term_line[0], long_term_line[1]);
  // if (green_theta > PI / 2)
  // {
  //   green_theta -= PI;
  // }
  // else if (green_theta < PI / 2)
  // {
  //   green_theta += PI;
  // }

  // return green_theta;
  float green_theta = (atan2(long_term_line[1], long_term_line[0]) * 180 / PI);
  if (green_theta < 0)
  {
    green_theta += 90;
  }
  else if (green_theta > 0)
  {
    green_theta = 90 - green_theta;
    if (green_theta != 0)
    {
      green_theta *= -1;
    }
  }
  else
  {
    green_theta = 0;
  }
  //Turn into radians
  green_theta = green_theta * PI / 180;
  // if first green theta calculated:
  if (stored_green_theta == -1000)
  {
    stored_green_theta = green_theta;
  }
  else
  {
    // filter
    stored_green_theta = (stored_green_theta * (1 - (green_theta_count / 100.f))) + (green_theta * (green_theta_count / 100.f));
    if (green_theta_count > 5)
    {
      green_theta_count -= 0.5f;
    }
  }

  return stored_green_theta;
}

void CloudComputer::setCameraPos(cv::Vec2f pos)
{
  cam_pos = pos;
}

cv::Mat CloudComputer::rotateWithTheta(cv::Mat pos)
{
  cv::Mat pos_clone = pos.clone();
  cv::Mat rot_matrix = (cv::Mat_<float>(3, 3) << 1, 0, 0,
                        0, cos(theta), -sin(theta),
                        0, sin(theta), cos(theta));
  return rot_matrix * pos_clone;
}

cv::Mat CloudComputer::autoRotate(cv::Mat pos)
{
  // if not enough pts yet, do nothing
  if (pts_vector_3d.size() < how_recent)
  {
    return pos;
  }
  // if enough pts found, check if line has been calculated
  // if line not calculated, do that and update old pts...
  if (!adjusted_3d)
  {
    std::cout << "[ANGLE AUTO ADJUSTING...]" << std::endl;
    // regresion line on 3d pts found before enough_pts_already flag thrown
    cv::fitLine(pts_vector_3d, compass_line, regression_type, 0, 0.01, 0.01);
    // create vector on X Z plane
    cv::Mat xz_vector = (cv::Mat_<float>(1, 3) << compass_line[0], 0, compass_line[2]);
    // theta between lines
    auto_theta = acos(((xz_vector.at<float>(0) * compass_line[0]) + (xz_vector.at<float>(1) * compass_line[1]) + (xz_vector.at<float>(2) * compass_line[2])) / (sqrt(pow(compass_line[0], 2) + pow(compass_line[1], 2) + pow(compass_line[2], 2)) * sqrt(pow(xz_vector.at<float>(0), 2) + pow(xz_vector.at<float>(1), 2) + pow(xz_vector.at<float>(2), 2))));
    // rot matrix based on auto_theta
    cv::Mat rot_matrix_init = (cv::Mat_<float>(3, 3) << 1, 0, 0,
                               0, cos(auto_theta), -sin(auto_theta),
                               0, sin(auto_theta), cos(auto_theta));
    std::cout << "[3d THETA]: " << (auto_theta * 180 / PI) << " degrees" << std::endl;
    // clone raw pts and clear all vectors
    vector<cv::Mat> raw_clone(raw_mat_vector);
    clearPointVectors();
    std::cout << "[BEGIN UPDATE OF " << raw_clone.size() << " OLD PTS]" << std::endl;
    for (int i = 0; i < raw_clone.size(); i++)
    {
      cv::Mat pt_i = (cv::Mat_<float>(3, 1) << raw_clone.at(i).at<float>(0), raw_clone.at(i).at<float>(1), raw_clone.at(i).at<float>(2));
      pt_i = rot_matrix_init * pt_i;
      updatePointVectors(pt_i);
    }
    std::cout << "[FINISHED UPDATE OF OLD PTS]" << std::endl;
    adjusted_3d = true;
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
