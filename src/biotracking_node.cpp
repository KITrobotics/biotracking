#include <biotracking/biotracking.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv,"biotracking");
  ros::NodeHandle nh("~");
  std::shared_ptr<Biotracking> tracker;
  tracker.reset(new Biotracking(nh));
  ros::spin();
  return 0;
}
