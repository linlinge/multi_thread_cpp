#include "pipeline.h"
void PlyParsing::Init(string load_path)
{
    load_path_=load_path;
    fp_=fopen(load_path_.c_str(),"rb");
    length_of_header_=0;
    fread((void*)(&char_header), sizeof(char_header), 1, fp_);
    string str_tmp=char_header;    
    length_of_header_=str_tmp.find("end_header")+10;

    /* which type of file? */
    int pos_xyz=str_tmp.find("property float x");
    int pos_rgb=str_tmp.find("property uchar red");
    int pos_nxyz=str_tmp.find("property float nx");
    int pos_scalar=str_tmp.find("property float scalar_Scalar_field");
    int pos_alpha=str_tmp.find("property uchar alpha");
    vertex_num_= atoi(str_tmp.substr(str_tmp.find("element vertex")+15, str_tmp.find("property float")-str_tmp.find("element vertex")-15).c_str());

    if((pos_xyz !=-1) && (pos_nxyz==-1) && (pos_rgb==-1) && (pos_scalar==-1)) mode=POSITION;
    else if((pos_xyz+51==pos_rgb) && (pos_rgb + 60 == pos_scalar)) mode=POSITION_COLOR_SCALAR;
    else if((pos_xyz+51==pos_rgb) && (pos_rgb + 60 == pos_alpha)) mode=POSITION_COLOR_ALPHA;
    else if((pos_xyz+51==pos_nxyz) && (pos_rgb==-1) && (pos_scalar==-1))   mode=POSITION_NORMAL;
    else if((pos_xyz + 51 == pos_rgb) && (pos_rgb + 60 == pos_nxyz) && (pos_scalar==-1) ) mode=POSITION_COLOR_NORMAL;
    else if((pos_xyz + 51 == pos_nxyz) && (pos_nxyz+54 == pos_rgb) && (pos_scalar==-1)) mode=POSITION_NORMAL_COLORA;
    fclose(fp_);
}

void PlyParsing::PrintHeader(string filepath)
{
    fp_=fopen(filepath.c_str(),"rb");
    fseek(fp_,0,0);
    fread((void*)(&char_header), sizeof(char_header), 1, fp_);
    string str_tmp=char_header;
    length_of_header_=str_tmp.find("end_header")+10;
    str_tmp=str_tmp.substr(0,length_of_header_);
    cout<<str_tmp<<endl;
    fclose(fp_);
}

/*
    itmp: the index of point that should be added in the point cloud
*/
void PlyParsing::ReadLine(pcl::PointCloud<PointType>::Ptr cloud,int itmp, string filepath, long linenum)
{ 
    FILE* fp=fopen(filepath.c_str(), "rb");
    if(POSITION==mode){
        Position ptmp;          
        long int start_cursor=length_of_header_+sizeof(ptmp)*linenum+1;
        fseek(fp,start_cursor,0);
        fread((void*)(&ptmp), sizeof(ptmp), 1, fp);      

	   
        cloud->points[itmp].x=ptmp.x;
        cloud->points[itmp].y=ptmp.y;
        cloud->points[itmp].z=ptmp.z;        
    }
    else if(POSITION_COLOR_SCALAR==mode){
        PositionColorScalar ptmp;          
        long int start_cursor=length_of_header_+sizeof(ptmp)*linenum+1;
        fseek(fp,start_cursor,0);
        fread((void*)(&ptmp), sizeof(ptmp), 1, fp);      
	    
        cloud->points[itmp].x=ptmp.x;
        cloud->points[itmp].y=ptmp.y;
        cloud->points[itmp].z=ptmp.z;
        cloud->points[itmp].r=ptmp.r;
        cloud->points[itmp].g=ptmp.g;
        cloud->points[itmp].b=ptmp.b;        
    }
    else if(POSITION_COLOR_ALPHA==mode){
        PositionColorAlpha ptmp;          
        long int start_cursor=length_of_header_+sizeof(ptmp)*linenum+1;
        int flag=fseek(fp,start_cursor,0);
        if(flag!=0) cout<<"error!"<<endl;
        fread((void*)(&ptmp), sizeof(ptmp), 1, fp); 
        cloud->points[itmp].x=ptmp.x;
        cloud->points[itmp].y=ptmp.y;
        cloud->points[itmp].z=ptmp.z;
        cloud->points[itmp].r=ptmp.r;
        cloud->points[itmp].g=ptmp.g;
        cloud->points[itmp].b=ptmp.b;
    }
    else if(POSITION_NORMAL==mode){ // position normal        
        PositionNormal ptmp;          
        long int start_cursor=length_of_header_+sizeof(ptmp)*linenum+1;
        fseek(fp,start_cursor,0);
        fread((void*)(&ptmp), sizeof(ptmp), 1, fp);      
        cloud->points[itmp].x=ptmp.x;
        cloud->points[itmp].y=ptmp.y;
        cloud->points[itmp].z=ptmp.z;
        cloud->points[itmp].normal_x=ptmp.nx;
        cloud->points[itmp].normal_y=ptmp.ny;
        cloud->points[itmp].normal_z=ptmp.nz;
    }
    else if(POSITION_COLOR_NORMAL==mode) // Position Color Normal
    {
        PositionColorNormal ptmp;         
        long int start_cursor=length_of_header_+sizeof(ptmp)*linenum+1;
        fseek(fp,start_cursor,0);
        fread((void*)(&ptmp), sizeof(ptmp), 1, fp); 
        cloud->points[itmp].x=ptmp.x;
        cloud->points[itmp].y=ptmp.y;
        cloud->points[itmp].z=ptmp.z;
        cloud->points[itmp].r=ptmp.r;
        cloud->points[itmp].g=ptmp.g;
        cloud->points[itmp].b=ptmp.b;
        cloud->points[itmp].normal_x=ptmp.nx;
        cloud->points[itmp].normal_y=ptmp.ny;
        cloud->points[itmp].normal_z=ptmp.nz;
    }
    else if(POSITION_NORMAL_COLORA==mode){
        PositionNormalColorA ptmp;         
        long int start_cursor=length_of_header_+sizeof(ptmp)*linenum+1;
        fseek(fp,start_cursor,0);
        fread((void*)(&ptmp), sizeof(ptmp), 1, fp);         
        cloud->points[itmp].x=ptmp.x;
        cloud->points[itmp].y=ptmp.y;
        cloud->points[itmp].z=ptmp.z;
        cloud->points[itmp].r=ptmp.r;
        cloud->points[itmp].g=ptmp.g;
        cloud->points[itmp].b=ptmp.b;
        cloud->points[itmp].normal_x=ptmp.nx;
        cloud->points[itmp].normal_y=ptmp.ny;
        cloud->points[itmp].normal_z=ptmp.nz;        
    }
    fclose(fp);
}

void PlyParsing::ReadLine(PointType& pts, string filepath,long linenum)
{
    FILE* fp=fopen(filepath.c_str(), "rb");
    if(POSITION==mode){
        Position ptmp;          
        long start_cursor=length_of_header_+sizeof(ptmp)*linenum+1;
        fseek(fp,start_cursor,0);
        fread((void*)(&ptmp), sizeof(ptmp), 1, fp);      	   
        pts.x=ptmp.x;
        pts.y=ptmp.y;
        pts.z=ptmp.z;        
    }
    else if(POSITION_COLOR_SCALAR==mode){
        PositionColorScalar ptmp;          
        long start_cursor=length_of_header_+sizeof(ptmp)*linenum+1;
        fseek(fp,start_cursor,0);
        fread((void*)(&ptmp), sizeof(ptmp), 1, fp);      
	    
        pts.x=ptmp.x;
        pts.y=ptmp.y;
        pts.z=ptmp.z;
        pts.r=ptmp.r;
        pts.g=ptmp.g;
        pts.b=ptmp.b;        
    }
    else if(POSITION_COLOR_ALPHA==mode){
        PositionColorAlpha ptmp;          
        long start_cursor=length_of_header_+sizeof(ptmp)*linenum+1;
        int flag=fseek(fp,start_cursor,0);
        if(flag!=0) cout<<"error!"<<endl;
        fread((void*)(&ptmp), sizeof(ptmp), 1, fp); 
        pts.x=ptmp.x;
        pts.y=ptmp.y;
        pts.z=ptmp.z;
        pts.r=ptmp.r;
        pts.g=ptmp.g;
        pts.b=ptmp.b;
    }
    else if(POSITION_NORMAL==mode){ // position normal        
        PositionNormal ptmp;          
        long start_cursor=length_of_header_+sizeof(ptmp)*linenum+1;
        fseek(fp,start_cursor,0);
        fread((void*)(&ptmp), sizeof(ptmp), 1, fp);      
        pts.x=ptmp.x;
        pts.y=ptmp.y;
        pts.z=ptmp.z;
        pts.normal_x=ptmp.nx;
        pts.normal_y=ptmp.ny;
        pts.normal_z=ptmp.nz;
    }
    else if(POSITION_COLOR_NORMAL==mode) // Position Color Normal
    {
        PositionColorNormal ptmp;         
        long start_cursor=length_of_header_+sizeof(ptmp)*linenum+1;
        fseek(fp,start_cursor,0);
        fread((void*)(&ptmp), sizeof(ptmp), 1, fp); 
        pts.x=ptmp.x;
        pts.y=ptmp.y;
        pts.z=ptmp.z;
        pts.r=ptmp.r;
        pts.g=ptmp.g;
        pts.b=ptmp.b;
        pts.normal_x=ptmp.nx;
        pts.normal_y=ptmp.ny;
        pts.normal_z=ptmp.nz;
    }
    else if(POSITION_NORMAL_COLORA==mode){
        PositionNormalColorA ptmp;         
        long start_cursor=length_of_header_+sizeof(ptmp)*linenum+1;
        fseek(fp,start_cursor,0);
        fread((void*)(&ptmp), sizeof(ptmp), 1, fp);
        pts.x=ptmp.x;
        pts.y=ptmp.y;
        pts.z=ptmp.z;
        pts.r=ptmp.r;
        pts.g=ptmp.g;
        pts.b=ptmp.b;
        pts.normal_x=ptmp.nx;
        pts.normal_y=ptmp.ny;
        pts.normal_z=ptmp.nz;        
    }
    fclose(fp);
}

void PlyParsing::LoadPly(double num_or_ratio, string save_path)
{
    /* random sampling */
    vector<int> v;

    cout<<"number of vertex:"<<vertex_num_<<endl;
    #if TEST_RUNTIME_DOWNSAMPLING
    double itime, ftime, exec_time;
    itime = omp_get_wtime();
    #endif

    // determine number of ratio
    int LEN;
    if(num_or_ratio>=1){
        LEN=(int)num_or_ratio;
    }
    else if(num_or_ratio<1 && num_or_ratio>0){
        LEN=(int)vertex_num_*num_or_ratio;
    }
    else{
        cout<<"LoadPly parameter error!"<<endl;
    }

    v.resize(LEN);
    #pragma omp parallel for
    for (int i = 0; i < LEN; ++i)        
        v[i]=rand() % vertex_num_;

    // WriteVector("/home/llg/dataset_paper/1.txt",v);
    // sort(v.begin(),v.end());
    // vector<int>::iterator it=(v.begin(),v.end());
    // v.erase(it,v.end());
    
    #if TEST_RUNTIME_DOWNSAMPLING
    ftime = omp_get_wtime();
    exec_time = ftime - itime;
    cout<<"Time taken is "<< exec_time<<endl;
    #endif


    // pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>);
    cloud_->points.resize(v.size());

    #pragma omp parallel for
    for(int i=0;i<v.size();i++){        
        ReadLine(cloud_,i,load_path_,v[i]);        
    }    

     

    if(""!=save_path){
        #if EXTRACT_V_NEGTIVE
            vector<int> v_neg;
            vector<int> v_all;
            v_all.resize(vertex_num_);
            #pragma omp parallel for
            for(int i=0;i<vertex_num_;i++)
                v_all[i]=i;
            std::set_difference(v_all.begin(),v_all.end(),v.begin(),v.end(),std::inserter(v_neg,v_neg.end()));        
            pcl::PointCloud<PointType>::Ptr cloud_v_neg(new pcl::PointCloud<PointType>);
            cloud_v_neg->points.resize(v_neg.size());
            for(int i=0;i<v_neg.size();i++) ReadLine(cloud_v_neg,i,load_path,v_neg[i]);
            pcl::io::savePLYFileBinary(save_path+"/no_select.ply",*cloud_v_neg);
        #endif      
        cout<<cloud_->points.size()<<endl;

        // pcl::io::savePLYFileBinary(save_path,*cloud);        
        pcl::io::savePLYFileASCII(save_path,*cloud_);        
    }
}


void Pipeline::Run(string LPC_filepath,string RPC_filepath)
{
    /* Init */
    pps_.Init(RPC_filepath);

    /* step 01: Down-sampling */
    // PlyParsing pps;	
	// pps.PrintHeader(filepath);
	// pps.LoadPly(filepath);

    
    // /* step 02: outlier removal I */
    // cloud_=pcl::PointCloud<PointType>::Ptr(new pcl::PointCloud<PointType>);
    // pcl::io::loadPLYFile(filepath,*cloud_);
    // RegionGrowth();

    /* step 03: Segmentation */
    /* step 04: Indexing */   
    pcl::PointCloud<PointType>::Ptr LPC_cloud(new pcl::PointCloud<PointType>);
	pcl::io::loadPLYFile(LPC_filepath,*LPC_cloud);
    Indexing(LPC_cloud,RPC_filepath);
}

void Pipeline::Downsampling(string load_path, double num_or_ratio, string save_path)
{
	PlyParsing pps;	
    pps.Init(load_path);
	pps.LoadPly(num_or_ratio,save_path);
    LPC_=pps.cloud_;
}
void Pipeline::OR1(string save_path)
{
    Processing pcs(LPC_);
    pcs.LoOP();
    pcs.RegionGrowth("1",9,4.0);
    if(""!=save_path)
        pcs.ExtractResult(save_path);
}
void Pipeline::RegionGrowth()
{
    pcl::search::Search<PointType>::Ptr tree (new pcl::search::KdTree<PointType>);
    pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
    // pcl::NormalEstimation<PointType, pcl::Normal> normal_estimator;
    // normal_estimator.setSearchMethod (tree);
    // normal_estimator.setInputCloud (cloud_);
    // normal_estimator.setKSearch (100);
    // normal_estimator.compute (*normals);

    normals->points.resize(LPC_->points.size());
    for(int i=0;i<normals->points.size();i++){
        normals->points[i].normal_x=LPC_->points[i].normal_x;
        normals->points[i].normal_y=LPC_->points[i].normal_y;
        normals->points[i].normal_z=LPC_->points[i].normal_z;
    }

    pcl::RegionGrowing<PointType, pcl::Normal> reg;
    reg.setMinClusterSize(50);
    reg.setMaxClusterSize(1000000);
    reg.setSearchMethod(tree);
    reg.setNumberOfNeighbours(50);
    reg.setInputCloud(LPC_);
    //reg.setIndices (indices);
    reg.setInputNormals(normals);
    reg.setSmoothnessThreshold(3.0 / 180.0 * M_PI);
    reg.setCurvatureThreshold(100.0);

    std::vector <pcl::PointIndices> clusters;
    reg.extract (clusters);

    srand((int)time(0));
    for(int i=0;i<clusters.size();i++){
        int rtmp=rand()%255;
        int gtmp=rand()%255;
        int btmp=rand()%255;
        for(int j=0;j<clusters[i].indices.size();j++){
            LPC_->points[clusters[i].indices[j]].r=rtmp;
            LPC_->points[clusters[i].indices[j]].g=gtmp;
            LPC_->points[clusters[i].indices[j]].b=btmp;

            LPC_->points[clusters[i].indices[j]].normal_x=normals->points[clusters[i].indices[j]].normal_x;
            LPC_->points[clusters[i].indices[j]].normal_y=normals->points[clusters[i].indices[j]].normal_y;
            LPC_->points[clusters[i].indices[j]].normal_z=normals->points[clusters[i].indices[j]].normal_z;
        }
    }
    pcl::io::savePLYFileBinary("/home/llg/dataset_paper/out_of_core_version.ply",*LPC_);
}

void Pipeline::Indexing(pcl::PointCloud<PointType>::Ptr cloud,string filepath)
{
    hash_table_.resize(cloud->points.size());
    pcl::search::KdTree<PointType>::Ptr kdtree(new pcl::search::KdTree<PointType>);
    kdtree->setInputCloud(cloud);

    #pragma omp parallel for
    for(long i=0;i<pps_.vertex_num_;i++){
        PointType pts;
        pps_.ReadLine(pts,filepath,i);        
        vector<int> idx;
        vector<float> dist;
        kdtree->nearestKSearch(pts,1,idx,dist);
        hash_table_[idx[0]].push_back(i);
    }
    cout<<hash_table_.size()<<endl;
}

void Pipeline::Merging(string read_files_path, string write_file_path)
{
    vector<string> filepath,filename;
    list_all_files(read_files_path,filepath,filename);
    for(int i=0;i<filepath.size();i++){
        pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>);
        pcl::io::loadPLYFile(filepath[i],*cloud);
        
    }
}