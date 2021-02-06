#pragma once
#include "PCLExtend.h"
// #include "FileExtend.h"
#include "StringExtend.h"
#include "VectorExtend.h"
// #include "Patch.h"
#include <iostream>
using namespace std;

class Processing
{
    public:
        pcl::PointCloud<PointType>::Ptr cloud_out_regular_;
        pcl::PointCloud<PointType>::Ptr cloud_out_irregular_;
        pcl::PointCloud<PointType>::Ptr cloud_;
        pcl::search::KdTree<PointType>::Ptr kdtree_;
        int flag_count_;
        vector<int> status_;
        int cnt_;
        
        Processing(pcl::PointCloud<PointType>::Ptr cloud){
            flag_count_=0;
            cnt_=0;
            cloud_=cloud;
            status_.resize(cloud_->points.size());
            cloud_out_regular_=pcl::PointCloud<PointType>::Ptr(new pcl::PointCloud<PointType>);
            cloud_out_irregular_=pcl::PointCloud<PointType>::Ptr(new pcl::PointCloud<PointType>);
            kdtree_=pcl::search::KdTree<PointType>::Ptr(new pcl::search::KdTree<PointType>);
            kdtree_->setInputCloud(cloud_);
        }


        // input: patch
        // void OR2(string domain="0", double thresh_kIQR=3.0);
        void GenerateNoise();
        void RegionGrowth(string domain="0", double level=8, double thresh_kIQR=3.0);
        void LoOP(string domain="0", int K=40, double thresh=0.8);
        void ExtractResult(string str);
    
    private:
        void GetScopeIndices(string str,vector<int>& cIdx);
};