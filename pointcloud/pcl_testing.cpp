#include <iostream>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <boost/unordered_map.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>

// #define MAX_BUF 1024


int
  main (int argc, char** argv)
{

  struct pt {
    float x;
    float y;
    float z;
  };

  pt new_pt;

  int fd;
  char * myfifo = "../../opencv/myfifo";

  boost::unordered::unordered_map<std::string, pt> pt_map;
  int i = 1;
  while(1){
      /* open, read, and display the message from the FIFO */
    fd = open(myfifo, O_RDONLY);
    read(fd, &new_pt, sizeof(pt));
      /* if new pt, at to hash map */
    std::string s = "";
    s.append(boost::lexical_cast<std::string>(new_pt.x));
    s.append(" ");
    s.append(boost::lexical_cast<std::string>(new_pt.y));
    s.append(" ");
    s.append(boost::lexical_cast<std::string>(new_pt.z));

    if(!pt_map.count(s)) {
      pt_map[s] = new_pt;
      i++;
      std::cout << "POINT #" << i << " " << new_pt.x << " " << new_pt.y << " " << new_pt.z << std::endl;
    }
  }


  // printf("Received: %s\n", buf);
  close(fd);

  

  return (0);
}
