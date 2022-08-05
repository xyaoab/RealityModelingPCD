#include "../lib/param.hpp"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string>

int main() {
  std::string filename = "/home/abby/project/RealityModelingPCD/test/input.cfg";
  param::parameter param(filename);
  if (!param) {
    std::cerr << "Could not find file " << filename << std::endl;
    std::abort();
  }

  std::cout << "Read existing values" << std::endl;
  std::cout << "Reading dataOutDir: " << param.get<std::string>("dataOutDir") << std::endl;
  std::cout << "Reading dataInDir: " << param.get<std::string>("dataInDir") << std::endl;
  std::cout << "Reading submapBoundaryOut: " << param.get<std::string>("submapBoundaryOut") << std::endl;
  std::cout << "Reading pointcloudIn: " << param.get<std::string>("pointcloudIn") << std::endl;

  std::cout << "Reading submapSizeX: " << param.get<double>("submapSizeX") << std::endl;
  std::cout << "Reading submapSizeY: " << param.get<double>("submapSizeY") << std::endl;
  std::cout << "Reading submapSizeZ: " << param.get<double>("submapSizeZ") << std::endl;
  std::cout << "Reading submapMargin: " << param.get<double>("submapMargin") << std::endl;
  std::cout << "Reading boundaryMinX: " << param.get<double>("boundaryMinX") << std::endl;
  std::cout << "Reading boundaryMinY: " << param.get<double>("boundaryMinY") << std::endl;
  std::cout << "Reading boundaryMinZ: " << param.get<double>("boundaryMinZ") << std::endl;
  std::cout << "Reading boundaryMaxX: " << param.get<double>("boundaryMaxX") << std::endl;
  std::cout << "Reading boundaryMaxY: " << param.get<double>("boundaryMaxY") << std::endl;
  std::cout << "Reading boundaryMaxZ: " << param.get<double>("boundaryMaxZ") << std::endl;
  
  std::cout << std::endl;

}