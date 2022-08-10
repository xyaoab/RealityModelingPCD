#include <math.h>
#include <time.h>
#include <stdio.h>

#include <string>
#include <vector>

#include "../lib/param.hpp"

using namespace std;

string dataInDir = ".";
string dataOutDir = ".";
string pointcloudIn = "pointcloud.txt";
string submapBoundaryOut = "submap_boundary.txt";
double submapSizeX = 5.0;
double submapSizeY = 5.0;
double submapSizeZ = 5.0;
double submapMargin = 0.5;
double boundaryMinX = -5000.0;
double boundaryMaxX = 5000.0;
double boundaryMinY = -5000.0;
double boundaryMaxY = 5000.0;
double boundaryMinZ = -500.0;
double boundaryMaxZ = 500.0;
bool increProc = false;

int main(int argc, char** argv)
{
  printf("\nRead parameters...\n");
  string filename = "../config/input.cfg";
  param::parameter param(filename);

  if (!param) {
    cerr << "Could not find file " << filename << endl;
    abort();
  }

  dataInDir = param.get<string>("dataInDir");
  dataOutDir = param.get<string>("dataOutDir");
  pointcloudIn = param.get<string>("pointcloudIn");
  submapBoundaryOut = param.get<string>("submapBoundaryOut");

  submapSizeX = param.get<double>("submapSizeX");
  submapSizeY = param.get<double>("submapSizeY");
  submapSizeZ = param.get<double>("submapSizeZ");
  submapMargin = param.get<double>("submapMargin");

  boundaryMinX = param.get<double>("boundaryMinX");
  boundaryMaxX = param.get<double>("boundaryMaxX");
  boundaryMinY = param.get<double>("boundaryMinY");
  boundaryMaxY = param.get<double>("boundaryMaxY");
  boundaryMinZ = param.get<double>("boundaryMinZ");
  boundaryMaxZ = param.get<double>("boundaryMaxZ");

  increProc = param.get<bool>("increProc");
  printf("\nRead parameters Done\n");  
  
  int indMinX = int(boundaryMinX / submapSizeX);
  if (boundaryMinX < 0) indMinX--;
  int indMinY = int(boundaryMinY / submapSizeY);
  if (boundaryMinY < 0) indMinY--;
  int indMinZ = int(boundaryMinZ / submapSizeZ);
  if (boundaryMinZ < 0) indMinZ--;

  int indMaxX = int(boundaryMaxX / submapSizeX);
  if (boundaryMaxX < 0) indMaxX--;
  int indMaxY = int(boundaryMaxY / submapSizeY);
  if (boundaryMaxY < 0) indMaxY--;
  int indMaxZ = int(boundaryMaxZ / submapSizeZ);
  if (boundaryMaxZ < 0) indMaxZ--;

  int submapNumX = indMaxX - indMinX + 1;
  int submapNumY = indMaxY - indMinY + 1;
  int submapNumZ = indMaxZ - indMinZ + 1;

  int submapNum = submapNumZ * submapNumY * submapNumX;
  int* submapAll = new int[submapNum];
  for (int ind = 0; ind < submapNum; ind++) submapAll[ind] = -1;

  vector<vector<int>> ptInSubmapAll;
  vector<int> submapIDAll;
  vector<int> nonMarginPtNumAll;
  int maxSubmapID = -1;

  printf ("\nReading data...\n");

  if (increProc) {
    string submapBoundaryInDir = dataOutDir + "/" + submapBoundaryOut;
    FILE *submapBoundaryInFile = fopen(submapBoundaryInDir.c_str(), "r");
    
    if (submapBoundaryInFile != NULL) {
      int submapID;
      float lowerboundX, upperboundX, lowerboundY, upperboundY, lowerboundZ, upperboundZ, submapMargin;
      int val1, val2, val3, val4, val5, val6, val7, val8;
      while (1) {
        val1 = fscanf(submapBoundaryInFile, "%d", &submapID);
        val2 = fscanf(submapBoundaryInFile, "%f", &lowerboundX);
        val3 = fscanf(submapBoundaryInFile, "%f", &upperboundX);
        val4 = fscanf(submapBoundaryInFile, "%f", &lowerboundY);
        val5 = fscanf(submapBoundaryInFile, "%f", &upperboundY);
        val6 = fscanf(submapBoundaryInFile, "%f", &lowerboundZ);
        val7 = fscanf(submapBoundaryInFile, "%f", &upperboundZ);
        val8 = fscanf(submapBoundaryInFile, "%f", &submapMargin);

        if (val1 != 1 || val2 != 1 || val3 != 1 || val4 != 1 || val5 != 1 || val6 != 1 || val7 != 1 || val8 != 1) break;

        float centerX = (lowerboundX + upperboundX) / 2.0;
        float centerY = (lowerboundY + upperboundY) / 2.0;
        float centerZ = (lowerboundZ + upperboundZ) / 2.0;

        int indX = int(centerX / submapSizeX) - indMinX;
        if (centerX < 0) indX--;
        int indY = int(centerY / submapSizeY) - indMinY;
        if (centerY < 0) indY--;
        int indZ = int(centerZ / submapSizeZ) - indMinZ;
        if (centerZ < 0) indZ--;
      
        int ind = submapNumX * submapNumY * indZ + submapNumX * indY + indX;
      
        if (ind >= 0 && ind < submapNum) {
          vector<int> ptInSubmap;
          ptInSubmapAll.push_back(ptInSubmap);
          submapIDAll.push_back(submapID);
          nonMarginPtNumAll.push_back(0);
          submapAll[ind] = ptInSubmapAll.size() - 1;
        }
      
        if (maxSubmapID < submapID) maxSubmapID = submapID;
      }

      fclose(submapBoundaryInFile);

      printf ("\nRead %d existing submaps, continue reading data...\n", int(ptInSubmapAll.size()));
    } else {
      increProc = false;
      printf ("\nCannot read submap boundary file, skip existing submaps.\n");
    }
  }

  string pointcloudInDir = dataInDir + "/" + pointcloudIn;
  FILE *pointcloudInFile = fopen(pointcloudInDir.c_str(), "r");
  if (pointcloudInFile == NULL) {
    printf ("\nCannot read point cloud file, exit.\n\n");
    exit(1);
  }

  vector<float> ptXAll, ptYAll, ptZAll, poseXAll, poseYAll, poseZAll, accuRatingAll;
  
  float ptX, ptY, ptZ, poseX, poseY, poseZ, accuRating;
  int val1, val2, val3, val4, val5, val6, val7;
  while (1) {
    val1 = fscanf(pointcloudInFile, "%f", &ptX);
    val2 = fscanf(pointcloudInFile, "%f", &ptY);
    val3 = fscanf(pointcloudInFile, "%f", &ptZ);
    val4 = fscanf(pointcloudInFile, "%f", &poseX);
    val5 = fscanf(pointcloudInFile, "%f", &poseY);
    val6 = fscanf(pointcloudInFile, "%f", &poseZ);
    val7 = fscanf(pointcloudInFile, "%f", &accuRating);

    if (val1 != 1 || val2 != 1 || val3 != 1 || val4 != 1 || val5 != 1 || val6 != 1 || val7 != 1) break;
    
    ptXAll.push_back(ptX);
    ptYAll.push_back(ptY);
    ptZAll.push_back(ptZ);
    poseXAll.push_back(poseX);
    poseYAll.push_back(poseY);
    poseZAll.push_back(poseZ);
    accuRatingAll.push_back(accuRating);
  }
  
  fclose(pointcloudInFile);

  int ptNum = ptXAll.size();
  printf ("\nRead %d points, parsing data...\n", ptNum);
  
  int nonMarginSubmapCount = 0;
  for (int i = 0; i < ptNum; i++) {
    if (ptXAll[i] < boundaryMinX || ptXAll[i] > boundaryMaxX || ptYAll[i] < boundaryMinY || ptYAll[i] > boundaryMaxY || 
        ptZAll[i] < boundaryMinZ || ptZAll[i] > boundaryMaxZ) break;

    int indX = int(ptXAll[i] / submapSizeX) - indMinX;
    if (ptXAll[i] < 0) indX--;
    int indY = int(ptYAll[i] / submapSizeY) - indMinY;
    if (ptYAll[i] < 0) indY--;
    int indZ = int(ptZAll[i] / submapSizeZ) - indMinZ;
    if (ptZAll[i] < 0) indZ--;

    int ind = submapNumX * submapNumY * indZ + submapNumX * indY + indX;

    int marginMinX = 0, marginMaxX = 0;
    if (ptXAll[i] - submapSizeX * (indX + indMinX) < submapMargin) marginMinX = -1;
    else if (submapSizeX * (indX + indMinX + 1) - ptXAll[i] < submapMargin) marginMaxX = 1;
    int marginMinY = 0, marginMaxY = 0;
    if (ptYAll[i] - submapSizeY * (indY + indMinY) < submapMargin) marginMinY = -1;
    else if (submapSizeY * (indY + indMinY + 1) - ptYAll[i] < submapMargin) marginMaxY = 1;
    int marginMinZ = 0, marginMaxZ = 0;
    if (ptZAll[i] - submapSizeZ * (indZ + indMinZ) < submapMargin) marginMinZ = -1;
    else if (submapSizeZ * (indZ + indMinZ + 1) - ptZAll[i] < submapMargin) marginMaxZ = 1;

    for (int indX2 = indX + marginMinX; indX2 <= indX + marginMaxX; indX2++) {
      for (int indY2 = indY + marginMinY; indY2 <= indY + marginMaxY; indY2++) {
        for (int indZ2 = indZ + marginMinZ; indZ2 <= indZ + marginMaxZ; indZ2++) {
          if (indX2 >= 0 && indX2 < submapNumX && indY2 >= 0 && indY2 < submapNumY && indY2 >= 0 && indY2 < submapNumY) {
            int ind2 = submapNumX * submapNumY * indZ2 + submapNumX * indY2 + indX2;
            if (ind2 >= 0 && ind2 < submapNum) {
              if (submapAll[ind2] >= 0) {
                ptInSubmapAll[submapAll[ind2]].push_back(i);
              } else {
                vector<int> ptInSubmap;
                ptInSubmap.push_back(i);
                ptInSubmapAll.push_back(ptInSubmap);
                submapIDAll.push_back(-1);
                nonMarginPtNumAll.push_back(0);
                submapAll[ind2] = ptInSubmapAll.size() - 1;
              }
            }
          }
        }
      }
    }

    if (nonMarginPtNumAll[submapAll[ind]] == 0) nonMarginSubmapCount++;
    nonMarginPtNumAll[submapAll[ind]]++;
  }

  printf ("\nParsed into %d submaps, saveing data...\n", nonMarginSubmapCount);

  string writeMode = "w";
  if (increProc) writeMode = "a";

  string submapBoundaryOutDir = dataOutDir + "/" + submapBoundaryOut;
  FILE *submapBoundaryOutFile = fopen(submapBoundaryOutDir.c_str(), writeMode.c_str());
  if (submapBoundaryOutFile == NULL) {
    printf ("\nCannot write submap boundary file, exit.\n\n");
    exit(1);
  }

  int fileCount = maxSubmapID + 1;
  for (int ind = 0; ind < submapNum; ind++) {
    if (submapAll[ind] >= 0) {
      if (submapIDAll[submapAll[ind]] >= 0 || nonMarginPtNumAll[submapAll[ind]] > 0) {
        int indX = ind % submapNumX;
        int indY = ((ind - indX) / submapNumX) % submapNumY;
        int indZ = (((ind - indX) / submapNumX - indY) / submapNumY) % submapNumZ;

        int fileID = fileCount;
        if (submapIDAll[submapAll[ind]] >= 0) {
          fileID = submapIDAll[submapAll[ind]];
          writeMode = "a";
          
          printf ("Submap %d - appended.\n", fileID);
        } else {
          fprintf(submapBoundaryOutFile, "%d %f %f %f %f %f %f %f\n", fileCount, submapSizeX * (indX + indMinX), 
                  submapSizeX * (indX + indMinX + 1), submapSizeY * (indY + indMinY), submapSizeY * (indY + indMinY + 1), 
                  submapSizeZ * (indZ + indMinZ), submapSizeZ * (indZ + indMinZ + 1), submapMargin);

          writeMode = "w";
          fileCount++;
          
          printf ("Submap %d - new.\n", fileID);
        }

        string submapOutDir = dataOutDir + "/" + to_string(fileID) + ".txt";
        FILE *submapOutFile = fopen(submapOutDir.c_str(), writeMode.c_str());
        if (submapOutFile == NULL) {
          printf ("\nCannot write submap files, exit.\n\n");
          exit(1);
        }
        
        vector<int> ptInSubmap = ptInSubmapAll[submapAll[ind]];
        int ptInSubmapNum = ptInSubmap.size();
        for (int i = 0; i < ptInSubmapNum; i++) {
          fprintf(submapOutFile, "%f %f %f %f %f %f %f\n", ptXAll[ptInSubmap[i]], ptYAll[ptInSubmap[i]], ptZAll[ptInSubmap[i]], 
                  poseXAll[ptInSubmap[i]], poseYAll[ptInSubmap[i]], poseZAll[ptInSubmap[i]], accuRatingAll[ptInSubmap[i]]);
        }
        
        fclose(submapOutFile);
      }
    }
  }

  delete[] submapAll;
  fclose(submapBoundaryOutFile);

  printf ("\nData saved in '%s', complete.\n\n", dataOutDir.c_str());

  return 0;
}
