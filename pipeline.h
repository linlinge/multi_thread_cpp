#pragma once
#include <iostream>
#include <fstream>
#include <string>
#include "PCLExtend.h"
#include "FileExtend.h"
#include <random>
#include <algorithm>
#include <stdlib.h>
#include <time.h> 
#include <Eigen/Dense>
#include "TopLayer.h"
#include "Processing.h"

using namespace std;
struct Position
{
    float x,y,z;    
};
struct PositionNormal{
    float x,y,z;
    float nx,ny,nz;
};
struct PositionNormalColor{
    float x,y,z;
    float nx,ny,nz;
    unsigned char r,g,b;
    PositionNormalColor(){}
    PositionNormalColor(PointType pts){
        
    }
};
struct PositionNormalColorA{
    float x,y,z;
    float nx,ny,nz;
    unsigned char r,g,b,alpha;
};
struct PositionColorNormal{
    float x,y,z;
    unsigned char r,g,b;
    int nx,ny,nz;
};
struct PositionColorScalar{
    float x,y,z;
    unsigned char r,g,b;
    float scalar;
};
struct PositionColorAlpha{
    float x,y,z;
    unsigned char r,g,b,alpha;
};


enum MODE{POSITION,POSITION_NORMAL,POSITION_NORMAL_COLOR,POSITION_NORMAL_COLORA,POSITION_COLOR_NORMAL,POSITION_COLOR_SCALAR,POSITION_COLOR_ALPHA};

class PlyParsing
{
    private:
        FILE* fp_;
        /* header parsing paramters */
        int length_of_header_;
        char char_header[1000];
        int mode;
        
        string load_path_;   

    public:  
        pcl::PointCloud<PointType>::Ptr cloud_;          
        long vertex_num_;

        void Init(string load_path);
        void ReadLine(pcl::PointCloud<PointType>::Ptr cloud, int itmp, string filepath,long linenum);
        void ReadLine(PointType& pts, string filepath,long linenum);
        void PrintHeader(string filepath); // Do not require to call Init firstly
        void LoadPly(double num_or_ratio, string save_path="");
};

class Pipeline
{
    private:
        pcl::PointCloud<PointType>::Ptr LPC_;
        string RPC_path_;
        vector<vector<int>> hash_table_;    
        PlyParsing pps_;
        void RegionGrowth();        

    public:
        void Downsampling(string load_path, double num_or_ratio=0.5, string save_path="");
        void OR1(string save_path="");
        void Indexing(pcl::PointCloud<PointType>::Ptr cloud, string filepath);
        void Merging(string read_files_path, string write_file_path);
        void Run(string LPC_filepath,string RPC_filepath);
};