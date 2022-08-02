#include <math.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <vector>

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

int main(int argc, char** argv)
{

  // nhPrivate.getParam("dataInDir", dataInDir);
  // nhPrivate.getParam("dataOutDir", dataOutDir);
  // nhPrivate.getParam("pointcloudIn", pointcloudIn);
  // nhPrivate.getParam("submapBoundaryOut", submapBoundaryOut);
  // nhPrivate.getParam("submapSizeX", submapSizeX);
  // nhPrivate.getParam("submapSizeY", submapSizeY);
  // nhPrivate.getParam("submapSizeZ", submapSizeZ);
  // nhPrivate.getParam("submapMargin", submapMargin);
  // nhPrivate.getParam("boundaryMinX", boundaryMinX);
  // nhPrivate.getParam("boundaryMaxX", boundaryMaxX);
  // nhPrivate.getParam("boundaryMinY", boundaryMinY);
  // nhPrivate.getParam("boundaryMaxY", boundaryMaxY);
  // nhPrivate.getParam("boundaryMinZ", boundaryMinZ);
  // nhPrivate.getParam("boundaryMaxZ", boundaryMaxZ);

  printf ("\nRead data...\n");

  string pointcloudInDir = dataInDir + "/" + pointcloudIn;
  FILE *pointcloudInFile = fopen(pointcloudInDir.c_str(), "r");
  if (pointcloudInFile == NULL) {
    printf ("\nCannot read input files, exit.\n\n");
    exit(1);
  }

  vector<float> ptXAll, ptYAll, ptZAll,
                poseXAll, poseYAll, poseZAll,
                poseRollAll, posePitchAll, poseYawAll, accuRatingAll;
  
  float ptX, ptY, ptZ, poseX, poseY, poseZ, poseRoll, posePitch, poseYaw, accuRating;
  int val1, val2, val3, val4, val5, val6, val7, val8, val9, val10;
  while (1) {
    val1 = fscanf(pointcloudInFile, "%f", &ptX);
    val2 = fscanf(pointcloudInFile, "%f", &ptY);
    val3 = fscanf(pointcloudInFile, "%f", &ptZ);
    val4 = fscanf(pointcloudInFile, "%f", &poseX);
    val5 = fscanf(pointcloudInFile, "%f", &poseY);
    val6 = fscanf(pointcloudInFile, "%f", &poseZ);
    val7 = fscanf(pointcloudInFile, "%f", &poseRoll);
    val8 = fscanf(pointcloudInFile, "%f", &posePitch);
    val9 = fscanf(pointcloudInFile, "%f", &poseYaw);
    val10 = fscanf(pointcloudInFile, "%f", &accuRating);

    if (val1 != 1 || val2 != 1 || val3 != 1 || val4 != 1 || val5 != 1 || val6 != 1 || val7 != 1
        || val8 != 1 || val9 != 1 || val10 != 1) break;
    
    ptXAll.push_back(ptX);
    ptYAll.push_back(ptY);
    ptZAll.push_back(ptZ);
    poseXAll.push_back(poseX);
    poseYAll.push_back(poseY);
    poseZAll.push_back(poseZ);
    poseRollAll.push_back(poseRoll);
    posePitchAll.push_back(posePitch);
    poseYawAll.push_back(poseYaw);
    accuRatingAll.push_back(accuRating);
  }
  
  fclose(pointcloudInFile);

  int ptNum = ptXAll.size();
  printf ("\nRead %d points, parsing data...\n", ptNum);
  
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
  vector<int> nonMarginPtNumAll;

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

  string submapBoundaryOutDir = dataOutDir + "/" + submapBoundaryOut;
  FILE *submapBoundaryOutFile = fopen(submapBoundaryOutDir.c_str(), "w");

  int fileCount = 0;
  for (int ind = 0; ind < submapNum; ind++) {
    if (submapAll[ind] >= 0) {
      if (nonMarginPtNumAll[submapAll[ind]] > 0) {
        int indX = ind % submapNumX;
        int indY = ((ind - indX) / submapNumX) % submapNumY;
        int indZ = (((ind - indX) / submapNumX - indY) / submapNumY) % submapNumZ;

        fprintf(submapBoundaryOutFile, "%d %f %f %f %f %f %f %f\n", fileCount, submapSizeX * (indX + indMinX), 
                submapSizeX * (indX + indMinX + 1), submapSizeY * (indY + indMinY), submapSizeY * (indY + indMinY + 1), 
                submapSizeZ * (indZ + indMinZ), submapSizeZ * (indZ + indMinZ + 1), submapMargin);

        string submapOutDir = dataOutDir + "/" + to_string(fileCount) + ".txt";
        FILE *submapOutFile = fopen(submapOutDir.c_str(), "w");
        if (submapOutFile == NULL) {
          printf ("\nCannot write output files, exit.\n\n");
          exit(1);
        }
        
        vector<int> ptInSubmap = ptInSubmapAll[submapAll[ind]];
        int ptInSubmapNum = ptInSubmap.size();
        for (int i = 0; i < ptInSubmapNum; i++) {
          fprintf(submapOutFile, "%f %f %f %f %f %f %f %f %f %f\n", ptXAll[ptInSubmap[i]], ptYAll[ptInSubmap[i]], ptZAll[ptInSubmap[i]], 
                  poseXAll[ptInSubmap[i]], poseYAll[ptInSubmap[i]], poseZAll[ptInSubmap[i]],
                  poseRollAll[ptInSubmap[i]], posePitchAll[ptInSubmap[i]], poseYawAll[ptInSubmap[i]],
                  accuRatingAll[ptInSubmap[i]]);
        }
        
        fclose(submapOutFile);
        fileCount++;
      }
    }
  }

  delete[] submapAll;
  fclose(submapBoundaryOutFile);

  printf ("\nData saved in '%s', complete.\n\n", dataOutDir.c_str());

  return 0;
}
