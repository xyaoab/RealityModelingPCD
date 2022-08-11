#include <math.h>
#include <time.h>
#include <stdio.h>

#include <string>
#include <vector>

#include "../lib/param.hpp"
using namespace std;

string dataInDir = ".";
string dataOutDir = ".";
string submapBoundaryIn = "submap_boundary.txt";
string meshOut = "mesh.obj";

int main(int argc, char** argv)
{
 
  printf("\n[MeshMerger]Read parameters...\n");
  string filename = "../config/mesh.cfg";
  param::parameter param(filename);

  dataInDir = param.get<string>("dataInDir");
  dataOutDir = param.get<string>("dataOutDir");
  submapBoundaryIn = param.get<string>("submapBoundaryIn");
  meshOut = param.get<string>("meshOut");

  string submapBoundaryInDir = dataInDir + "/" + submapBoundaryIn;
  FILE *submapBoundaryInFile = fopen(submapBoundaryInDir.c_str(), "r");
  if (submapBoundaryInFile == NULL) {
    printf ("\nCannot read submap boundary file, exit.\n\n");
    exit(1);
  }

  vector<int> submapIDAll;
  vector<float> lowerboundXAll, upperboundXAll, lowerboundYAll, upperboundYAll, lowerboundZAll, upperboundZAll;

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

    submapIDAll.push_back(submapID);
    lowerboundXAll.push_back(lowerboundX);
    upperboundXAll.push_back(upperboundX);
    lowerboundYAll.push_back(lowerboundY);
    upperboundYAll.push_back(upperboundY);
    lowerboundZAll.push_back(lowerboundZ);
    upperboundZAll.push_back(upperboundZ);
  }

  fclose(submapBoundaryInFile);

  int submapNum = submapIDAll.size();
  printf ("\nMerging %d meshes...\n", submapNum);

  vector<float> vertexX, vertexY, vertexZ, vertexXAll, vertexYAll, vertexZAll;
  vector<float> normX, normY, normZ, normXAll, normYAll, normZAll;
  vector<int> vertexMap, faceInd1, faceInd2, faceInd3, faceInd1All, faceInd2All, faceInd3All;

  bool withNorm = true;
  int mergedVertexCount = 0;
  for (submapID = 0; submapID < submapNum; submapID++) {
    vertexX.clear();
    vertexY.clear();
    vertexZ.clear();
    vertexMap.clear();
    normX.clear();
    normY.clear();
    normZ.clear();
    faceInd1.clear();
    faceInd2.clear();
    faceInd3.clear();

    lowerboundX = lowerboundXAll[submapID];
    upperboundX = upperboundXAll[submapID];
    lowerboundY = lowerboundYAll[submapID];
    upperboundY = upperboundYAll[submapID];
    lowerboundZ = lowerboundZAll[submapID];
    upperboundZ = upperboundZAll[submapID];

    string meshInDir = dataInDir + "/" + to_string(submapID) + ".obj";
    FILE *meshInFile = fopen(meshInDir.c_str(), "r");
    if (meshInFile == NULL) {
      printf ("Cannot read %s, skip.\n", meshInDir.c_str());
      continue;
    }

    char str[1000];
    float x, y, z, nx, ny, nz;
    int ind1, ind2, ind3;
    int val1, val2, val3, val4;
    while (1) {
      val1 = fscanf(meshInFile, "%s", str);
      if (val1 != 1) break;

      string type(str);
      if (type == "#") {
        val2 = fscanf(meshInFile, "%[^\n] ", str);
        if (val2 != 1) break;
      } else if (type == "v") {
        val2 = fscanf(meshInFile, "%f", &x);
        val3 = fscanf(meshInFile, "%f", &y);
        val4 = fscanf(meshInFile, "%f", &z);

        if (val2 != 1 || val3 != 1 || val4 != 1) break;

        vertexX.push_back(x);
        vertexY.push_back(y);
        vertexZ.push_back(z);
        vertexMap.push_back(-1);
      } else if (type == "vn") {
        val2 = fscanf(meshInFile, "%f", &nx);
        val3 = fscanf(meshInFile, "%f", &ny);
        val4 = fscanf(meshInFile, "%f", &nz);

        if (val2 != 1 || val3 != 1 || val4 != 1) break;

        normX.push_back(nx);
        normY.push_back(ny);
        normZ.push_back(nz);
      } else if (type == "f") {
        val2 = fscanf(meshInFile, "%s", str);
        string number1(str);
        ind1 = stoi(number1);
        val3 = fscanf(meshInFile, "%s", str);
        string number2(str);
        ind2 = stoi(number2);
        val4 = fscanf(meshInFile, "%s", str);
        string number3(str);
        ind3 = stoi(number3);

        if (val2 != 1 || val3 != 1 || val4 != 1) break;

        faceInd1.push_back(ind1);
        faceInd2.push_back(ind2);
        faceInd3.push_back(ind3);
      }
    }

    int vertexNum = vertexX.size();
    int normNum = normX.size();
    if (vertexNum != normNum) withNorm = false;

    int faceNum = faceInd1.size();
    for (int i = 0; i < faceNum; i++) {
      float x1 = vertexX[faceInd1[i] - 1];
      float y1 = vertexY[faceInd1[i] - 1];
      float z1 = vertexZ[faceInd1[i] - 1];
      
      float x2 = vertexX[faceInd2[i] - 1];
      float y2 = vertexY[faceInd2[i] - 1];
      float z2 = vertexZ[faceInd2[i] - 1];
      
      float x3 = vertexX[faceInd3[i] - 1];
      float y3 = vertexY[faceInd3[i] - 1];
      float z3 = vertexZ[faceInd3[i] - 1]; 

      if ((x1 >= lowerboundX && x1 <= upperboundX && y1 >= lowerboundY && y1 <= upperboundY && z1 >= lowerboundZ && z1 <= upperboundZ) ||
          (x2 >= lowerboundX && x2 <= upperboundX && y2 >= lowerboundY && y2 <= upperboundY && z2 >= lowerboundZ && z2 <= upperboundZ) ||
          (x3 >= lowerboundX && x3 <= upperboundX && y3 >= lowerboundY && y3 <= upperboundY && z3 >= lowerboundZ && z3 <= upperboundZ)) {

        vertexMap[faceInd1[i] - 1] = 0;
        vertexMap[faceInd2[i] - 1] = 0;
        vertexMap[faceInd3[i] - 1] = 0;
      } else {
        faceInd1[i] = -1;
        faceInd2[i] = -1;
        faceInd3[i] = -1;
      }
    }

    for (int i = 0; i < vertexNum; i++) {
      if (vertexMap[i] != -1) {
        vertexXAll.push_back(vertexX[i]);
        vertexYAll.push_back(vertexY[i]);
        vertexZAll.push_back(vertexZ[i]);
        vertexMap[i] = mergedVertexCount;
        
        if (withNorm) {
          normXAll.push_back(normX[i]);
          normYAll.push_back(normY[i]);
          normZAll.push_back(normZ[i]);
        }
        
        mergedVertexCount++;
      }
    }

    for (int i = 0; i < faceNum; i++) {
      if (faceInd1[i] != -1) {
        faceInd1All.push_back(vertexMap[faceInd1[i] - 1] + 1);
        faceInd2All.push_back(vertexMap[faceInd2[i] - 1] + 1);
        faceInd3All.push_back(vertexMap[faceInd3[i] - 1] + 1);
      }
    }
  
    fclose(meshInFile);
  }

  int vertexAllNum = vertexXAll.size();
  int faceAllNum = faceInd1All.size();
  if (vertexAllNum == 0 || faceAllNum == 0) {
    printf ("\nEmpty mesh, no mesh file saved, complete.\n\n");
    return 0;
  }
  
  if (withNorm) {
    printf ("\nSaving mesh file with norm...\n");
  } else {
    printf ("\nSaving mesh file without norm...\n");
  }

  string meshOutDir = dataOutDir + "/" + meshOut;
  FILE *meshOutFile = fopen(meshOutDir.c_str(), "w");
  if (meshOutFile == NULL) {
    printf ("\nCannot read mesh file, exit.\n\n");
    exit(1);
  }

  for (int i = 0; i < vertexAllNum; i++) {
    fprintf(meshOutFile, "v %f %f %f\n", vertexXAll[i], vertexYAll[i], vertexZAll[i]);
    if (withNorm) {
      fprintf(meshOutFile, "vn %f %f %f\n", normXAll[i], normYAll[i], normZAll[i]);
    }
  }


  for (int i = 0; i < faceAllNum; i++) {
    if (withNorm) {
      fprintf(meshOutFile, "f %d//%d %d//%d %d//%d\n", faceInd1All[i], faceInd1All[i], faceInd2All[i], faceInd2All[i], faceInd3All[i], faceInd3All[i]);
    } else {
      fprintf(meshOutFile, "f %d %d %d\n", faceInd1All[i], faceInd2All[i], faceInd3All[i]);
    }
  }

  fclose(meshOutFile);

  printf ("\nSaved file '%s', complete.\n\n", meshOutDir.c_str());

  return 0;
}