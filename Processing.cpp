#include "Processing.h"
#include "Statistics.h"
#define MAX2(D1,D2) ((D1)>(D2) ? (D1):(D2))
#define MAX3(D1,D2,D3) MAX2(MAX2(D1,D2),(D3))

bool customRegionGrowing(const PointType& point_a, const PointType& point_b, float squared_distance)
{
    return true;
}

// void Processing::OR2(string domain, double thresh_kIQR)
// {
//     // Step 01: init scope
//     vector<int> scope;
//     GetScopeIndices(domain, scope);    
//     cnt_++;
//     vector<double> dist;
//     dist.resize(cloud_->points.size());

//     /* Step 02: calculate distance between points and plane */
//     for(int j=0;j<cloud_->points.size();j++)
//         dist[j]=pch_->GetDistance(cloud_->points[j]);

//     /* Step 03: determine the threshold */
//     double thresh=TukeyFence(dist,2);                        
//     // pcl::PointCloud<PointType>::Ptr cloud_part_0(new pcl::PointCloud<PointType>);
//     // pcl::PointCloud<PointType>::Ptr cloud_part_1(new pcl::PointCloud<PointType>);
//     for(int j=0;j<dist.size();j++){
//         if(dist[j]>thresh){
//             // cloud_part_0->points.push_back(cloud_->points[j]);
//             cloud_->points[j].r=255;
//             cloud_->points[j].g=0;
//             cloud_->points[j].b=0;
//             status_[j]=-cnt_;
//         }
//         else{
//             // cloud_part_1->points.push_back(cloud_->points[j]);             
//             status_[j]=cnt_;
//         }
//     }
// }

void Processing::RegionGrowth(string domain, double level, double thresh_kIQR)
{
    /* Step 01: Init */
    cnt_++;
    vector<int> scope;
    GetScopeIndices(domain, scope);    

    /* Step 02: copy point cloud */
    pcl::PointCloud<PointType>::Ptr cloud_active(new pcl::PointCloud<PointType>);
    pcl::copyPointCloud(*cloud_,scope,*cloud_active);

    /* Step 03: */
    pcl::IndicesClustersPtr clusters (new pcl::IndicesClusters);
    pcl::IndicesClustersPtr small_clusters (new pcl::IndicesClusters);
    pcl::IndicesClustersPtr large_clusters (new pcl::IndicesClusters);
    pcl::search::KdTree<PointType>::Ptr kdtree_tmp(new pcl::search::KdTree<PointType>);

    PointType vmin,vmax;
    pcl::getMinMax3D(*cloud_active, vmin, vmax);
    double box_max=MAX3(vmax.x-vmin.x,vmax.y-vmin.y,vmax.z-vmin.z);
    double tolerance=box_max/pow(2,level);

    // Set up a Conditional Euclidean Clustering class
    pcl::ConditionalEuclideanClustering<PointType> cec(true);
    cec.setInputCloud(cloud_active);
    cec.setConditionFunction(&customRegionGrowing);
    cec.setClusterTolerance(tolerance);
    cec.segment(*clusters);
    vector<int> cluster_size_set;
    for(int i=0;i<clusters->size();i++){
       cluster_size_set.push_back((*clusters)[i].indices.size());
    }

    /* Step 04: Threshold */
    double IQR=Quantile(cluster_size_set,0.75)-Quantile(cluster_size_set,0.25);
    double thresh=Quantile(cluster_size_set,0.75)+IQR*thresh_kIQR;
    cout<<"quantity of cluster:"<<cluster_size_set.size()<<endl;
    cout<<"cluster thresh:"<<thresh<<endl;

    for(int i=0;i<clusters->size();i++){
        int current_cluster_size=(*clusters)[i].indices.size();
        if(current_cluster_size<=thresh){
            for(int j=0;j<(*clusters)[i].indices.size();j++){
                int itmp=(*clusters)[i].indices[j];
                vector<int> idx;
                vector<float> dist;
                kdtree_->nearestKSearch(cloud_active->points[itmp],2,idx,dist);
                int itmp2=idx[0];
                for(int k=0;k<idx.size();k++){
                    if(itmp==idx[k])
                        itmp2=itmp;
                }
                status_[itmp2]=-cnt_;
            }
        }
        else{
            for(int j=0;j<(*clusters)[i].indices.size();j++){
                int itmp=(*clusters)[i].indices[j];
                vector<int> idx;
                vector<float> dist;
                kdtree_->nearestKSearch(cloud_active->points[itmp],2,idx,dist);
                int itmp2=idx[0];
                for(int k=0;k<idx.size();k++){
                    if(itmp==idx[k])
                        itmp2=itmp;
                }
                status_[itmp2]=cnt_;
                cloud_->points[itmp2].r=0;
                cloud_->points[itmp2].g=255;
                cloud_->points[itmp2].b=0; 
            }
        }
    }
}

void Processing::ExtractResult(string strtmp)
{
    for(int i=0;i<status_.size();i++){
        if(status_[i]>0){
            cloud_out_regular_->points.push_back(cloud_->points[i]);
        }
        else if(status_[i]<0){
            cloud_out_irregular_->points.push_back(cloud_->points[i]);
        }
        else
        {
            cout<<"error ! "<<status_[i]<<endl;
        }
        
    }

    // save file
    if(strtmp.find(".ply")!=string::npos){
        pcl::io::savePLYFileBinary(strtmp.substr(0,strtmp.size()-4)+"_regular.ply",*cloud_out_regular_);
        pcl::io::savePLYFileBinary(strtmp.substr(0,strtmp.size()-4)+"_irregular.ply",*cloud_out_irregular_);
    }
    else if(strtmp[strtmp.size()-1]=='/' && strtmp.find(".")!=string::npos){
        pcl::io::savePLYFileBinary(strtmp+"regular.ply",*cloud_out_regular_);
        pcl::io::savePLYFileBinary(strtmp+"irregular.ply",*cloud_out_irregular_);
    }
    else if(strtmp.find(".")!=string::npos){
        pcl::io::savePLYFileBinary(strtmp+"_regular.ply",*cloud_out_regular_);
        pcl::io::savePLYFileBinary(strtmp+"_irregular.ply",*cloud_out_irregular_);
    }
    else{
        cout<<"Error: output path string error!"<<endl;
    }
}

void Processing::LoOP(string domain,int K, double thresh)
{
    /* Step 01: Init */
    cnt_++;
    vector<int> scope;
    GetScopeIndices(domain, scope);   

    /* Step 02: copy point cloud */
    pcl::PointCloud<PointType>::Ptr cloud_active(new pcl::PointCloud<PointType>);
    pcl::copyPointCloud(*cloud_,scope,*cloud_active);


	pcl::search::KdTree<PointType>::Ptr kdtree(new pcl::search::KdTree<PointType>);
	kdtree->setInputCloud(cloud_active);
	vector<double> sigma;
	vector<double> plof;
	vector<double> rst_LoOP_;
    double nplof=0;
    // Resize Scores	
	sigma.resize(cloud_active->points.size());
	plof.resize(cloud_active->points.size());
	rst_LoOP_.resize(cloud_active->points.size());

	// Step 01: Calculate sigma
	#pragma omp parallel for
	for(int i=0;i<cloud_active->points.size();i++){
		// find k-nearest neighours
		vector<int> idx(K+1);
		vector<float> dist(K+1);
		kdtree->nearestKSearch (i, K+1, idx, dist);
		// cout<<cloud->points[i]<<endl;
		double sum=0;
		for(int j=1;j<K+1;j++){
			sum+=dist[j];
		}
		sum=sum/K;
		sigma[i]=sqrt(sum);
	}
	
	// Step 02: calculate mean
	double mean=0;
	#pragma omp parallel for
	for (int i = 0; i < cloud_active->points.size(); i++){        
        vector<int> idx(K+1);
		vector<float> dist(K+1);
		kdtree->nearestKSearch (cloud_active->points[i], K+1, idx, dist);
        double sum = 0;
        for (int j = 1; j < K+1; j++)
          sum += sigma[idx[j]];
        sum /= K;
        plof[i] = sigma[i] / sum  - 1.0f;				
        mean += plof[i] * plof[i];
    }
	nplof=sqrt(mean/cloud_active->points.size());	

	// Step 03: caculate score
	#pragma omp parallel for
	for(int i=0;i<cloud_active->points.size();i++){
		double value = plof[i] / (nplof * sqrt(2.0f));
		// rst_.records_[i].item1_=value;

        double dem = 1.0 + 0.278393 * value;
        dem += 0.230389 * value * value;
        dem += 0.000972 * value * value * value;
        dem += 0.078108 * value * value * value * value;
        double op = std::max(0.0, 1.0 - 1.0 / dem);
        rst_LoOP_[i] = op;
	}

	vector<int> idx;
    for(int i=0;i<rst_LoOP_.size();i++){
        vector<int> idx;
        vector<float> dist;
        kdtree_->nearestKSearch(cloud_active->points[i],1,idx,dist);

        if(rst_LoOP_[i]<thresh){
			status_[idx[0]]=cnt_;
		}
        else
        {
            status_[idx[0]]=-cnt_;
        }        
	}
}

void Processing::GetScopeIndices(string str, vector<int>& cIdx)
{
    vector<int> emnt; // elements
    Str2Vec(str,",",emnt);
    VecFindPos(status_,emnt,cIdx);
}

void Processing::GenerateNoise()
{
    std::cout << "读取文件" << std::endl;

    pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>); // 创建点云（指针）

    pcl::io::loadPLYFile<PointType>("E:\\data\\12.ply", *cloud);
    std::cout << "正确查找文件" << std::endl;


    //添加高斯噪声
    pcl::PointCloud<PointType>::Ptr cloudfiltered(new pcl::PointCloud<PointType>());
    cloudfiltered->points.resize(cloud->points.size());//将点云的cloud的size赋值给噪声
    cloudfiltered->header = cloud->header;
    cloudfiltered->width = cloud->width;
    cloudfiltered->height = cloud->height;

    boost::mt19937 rng;
    rng.seed(static_cast<unsigned int>(time(0)));
    boost::normal_distribution<> nd(0, 2);
    boost::variate_generator<boost::mt19937&, boost::normal_distribution<>> var_nor(rng, nd);

    //添加噪声
    for (size_t point_i = 0; point_i < cloud->points.size(); ++point_i)
    {
        cloudfiltered->points[point_i].x = cloud->points[point_i].x + static_cast<float> (var_nor());
        cloudfiltered->points[point_i].y = cloud->points[point_i].y + static_cast<float> (var_nor());
        cloudfiltered->points[point_i].z = cloud->points[point_i].z + static_cast<float> (var_nor());
    }

    pcl::io::savePLYFileBinary("/home/llg/dataset/",*cloudfiltered);

    // pcl::visualization::PCLVisualizer viewer("Cloud Viewer");
    // //设置颜色为绿色
    // pcl::visualization::PointCloudColorHandlerCustom<PointType> singleColor(cloud, 0, 255, 0);
    // //往窗口添加点云并设置颜色，原图为cloud 添加后为cloudfiltered，记得改
    // viewer.addPointCloud(cloud, singleColor, "cloud");
    // //添加点云后，通过点云ID来设置显示大小
    // viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");
    // //重置相机，将点云显示到窗口
    // viewer.resetCamera();
    // while (!viewer.wasStopped()){
    //     viewer.spinOnce();
    // }
}