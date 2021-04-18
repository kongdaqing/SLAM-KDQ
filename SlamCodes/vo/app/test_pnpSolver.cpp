#include "Simulator.hpp"
using namespace ov;
int main(int argc,char **argv) {
  if (argc < 2) {
    std::cerr << "Please input config file!" << std::endl;
    return -1;
  }
  Config *cfg = new Config(argv[1]);
  Camera *cam = new Camera(cfg);
  Simulator sim(cfg,cam);
  sim.createLandMarkers();
  

}
