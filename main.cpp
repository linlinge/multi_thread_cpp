#include <iostream>	
#include "PCLExtend.h"
#include <Eigen/Dense>
#include "V3.hpp"
#include <vector>
#include "pipeline.h"
#include "TopLayer.h"
#include "Processing.h"
using namespace std;

int main(int argc,char** argv)
{
	Pipeline ppl;
	string mode=argv[1];
	if("00_downsampling"==mode){ // mproc test_sampling <load_path> <1000> [save_path]
		string load_path=argv[2];
		double num_or_ratio=strtod(argv[3],NULL);
		if(argc==4){			
			ppl.Downsampling(load_path,num_or_ratio);
		}
		else if(argc==5){						
			string save_path=argv[4];
			ppl.Downsampling(load_path,num_or_ratio,save_path);
		}
	}
	else if("01_OR1"==mode){
		pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>);
		pcl::io::loadPLYFile(argv[2],*cloud);
		Processing pcs(cloud);
		pcs.RegionGrowth("0",9.0,40.0);
		pcs.ExtractResult(argv[3]);
	}
	else if("02_Segmentation"==mode){

	}
	else if("03_Indexing_and_Patch"==mode){

	}
	else if("04_OR2"==mode){

	}
	else if("05_merging"==mode){
	
	}
	else if("print_header"==mode){
		PlyParsing pps;
		pps.PrintHeader(argv[2]);
	}
 
	return 0;
}