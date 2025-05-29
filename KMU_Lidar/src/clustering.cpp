#include "clustering.h"


/**
 ** 다목적 클러스터링
*/


void Clustering::clusterObject(const pcl::PointCloud<PointType>::Ptr& input_pointCloud, const std::shared_ptr<std::vector<pcl::PointCloud<PointType>::Ptr>>& bigObject_Cloud_vector)
{
// 다운샘플링
    pcl::PointCloud<PointType>::Ptr downsample_pointCloud(new pcl::PointCloud<PointType>);
    pcl::VoxelGrid<PointType> voxelGrid;
    voxelGrid.setInputCloud(input_pointCloud);
    voxelGrid.setLeafSize(0.1f, 0.1f, 0.1f);
    voxelGrid.filter(*downsample_pointCloud);

// 벽면제거
    float z_threshold = 1.7 + LI_TO_GND_Z;  // 차량의 높이보다 좀 높아야됨
    float xy_tolerance = 0.3;  // 벽면 제거를 위한 x, y 좌표의 제거 범위
    pcl::PassThrough<PointType> pass;
    pass.setInputCloud(downsample_pointCloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(z_threshold, std::numeric_limits<float>::max());
    pcl::PointIndices::Ptr wall_indices(new pcl::PointIndices());
    pass.filter(wall_indices->indices);

    // 벽 포인트의 x, y 범위 내 포인트 제거
    std::set<int> indices_to_remove;
    for (int index : wall_indices->indices) {
        PointType wall_point = downsample_pointCloud->points[index];

        for (size_t i = 0; i < downsample_pointCloud->points.size(); ++i) {
            if (std::abs(downsample_pointCloud->points[i].x - wall_point.x) <= xy_tolerance &&
                std::abs(downsample_pointCloud->points[i].y - wall_point.y) <= xy_tolerance &&
                downsample_pointCloud->points[i].z < wall_point.z) {
                indices_to_remove.insert(i);
            }
        }
    }

    // 중복을 제거한 인덱스를 사용하여 포인트 제거
    pcl::PointIndices::Ptr final_indices(new pcl::PointIndices());
    final_indices->indices.assign(indices_to_remove.begin(), indices_to_remove.end());

    pcl::PointCloud<PointType>::Ptr noWall_PointCloud(new pcl::PointCloud<PointType>);
    pcl::ExtractIndices<PointType> extract;
    if (!downsample_pointCloud->points.empty() && !final_indices->indices.empty()) {
        extract.setInputCloud(downsample_pointCloud);
        extract.setIndices(final_indices);
        extract.setNegative(true);
        extract.filter(*noWall_PointCloud);
    } else {
        // 에러 처리 또는 빈 클라우드 처리
        std::cout << "No points to process after filtering." << std::endl;
    }
    pcl::PointCloud<PointType>::Ptr preprocessed_pointCloud(new pcl::PointCloud<PointType>);
    *preprocessed_pointCloud = *noWall_PointCloud;
    
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<PointType>::Ptr tree(new pcl::search::KdTree<PointType>());

   // 노멀 계산을 위한 Normal Estimation 객체 생성
    pcl::NormalEstimation<PointType, pcl::Normal> normal_estimator;
    normal_estimator.setSearchMethod(tree);
    normal_estimator.setInputCloud(preprocessed_pointCloud);
    normal_estimator.setKSearch(30);  // 인접 포인트 수 설정
    normal_estimator.compute(*normals);

    // Region Growing 알고리즘 설정
    pcl::RegionGrowing<PointType, pcl::Normal> reg;
    reg.setMinClusterSize(50);
    reg.setMaxClusterSize(500);
    reg.setSearchMethod(tree);
    reg.setNumberOfNeighbours(30);  // 인접한 포인트 수
    reg.setInputCloud(preprocessed_pointCloud);
    reg.setInputNormals(normals);
    reg.setSmoothnessThreshold(30.0 / 180.0 * M_PI);  // 곡률 임계값
    reg.setCurvatureThreshold(3.0);  // 곡률 임계값

    std::vector<pcl::PointIndices> clusterIndices;
    reg.extract(clusterIndices);
    
    
// 각 클러스터의 포인트 클라우드를 컨테이너에 저장
    PointType clusterPoint;
    float diagonal = -1;
    int markerId = 0;
    if (preprocessed_pointCloud->points.size() > 0){

        // 각 클러스터를 output_pointCloud_vector에 저장
        for(pcl::PointIndices& clusterIdx : clusterIndices)
        {
            // x, y의 최대값 최소값 저장 포인트
            PointType minXPoint, maxXPoint, minYPoint, maxYPoint, minZPoint, maxZPoint;
            minXPoint.x = minXPoint.y = minXPoint.z = std::numeric_limits<float>::max();
            maxXPoint.x = maxXPoint.y = maxXPoint.z = std::numeric_limits<float>::lowest();
            minYPoint.x = minYPoint.y = minYPoint.z = std::numeric_limits<float>::max();
            maxYPoint.x = maxYPoint.y = maxYPoint.z = std::numeric_limits<float>::lowest();
            minZPoint.x = minZPoint.y = minZPoint.z = std::numeric_limits<float>::max();
            maxZPoint.x = maxZPoint.y = maxZPoint.z = std::numeric_limits<float>::lowest();

            clusterPoint.x = 0.0; clusterPoint.y = 0.0; clusterPoint.z = 0.0;
            for(int& index : clusterIdx.indices)
            { 
                PointType point = preprocessed_pointCloud->points[index];
                point.z = 0;
                if (point.x > maxXPoint.x) maxXPoint = point;
                if (point.x < minXPoint.x) minXPoint = point;
                if (point.y > maxYPoint.y) maxYPoint = point;
                if (point.y < minYPoint.y) minYPoint = point;
                if (point.z > maxZPoint.z) maxZPoint = point;
                if (point.z < minZPoint.z) minZPoint = point;

                clusterPoint.x += point.x;
                clusterPoint.y += point.y;
            }

            clusterPoint.x /= clusterIdx.indices.size();
            clusterPoint.y /= clusterIdx.indices.size();

            float scaleX = maxXPoint.x - minXPoint.x;
            float scaleY = maxYPoint.y - minYPoint.y;

            // 벽면 및 바닥일 경우 클러스터링 대상 제외
            // if (((maxZPoint.z + LI_TO_GND_Z) > 2.3) || ((maxZPoint.z - minZPoint.z) < 0.3))
            // {
            //     continue;
            // }

            // 큰 장애물
            if (((0 < scaleX && scaleX < 0.1) && (0 < scaleY && scaleY < 0.1)) || 
                ((0 < scaleY && scaleY < 0.1) && (0 < scaleX && scaleX < 0.1)))
            {
                pcl::PointCloud<PointType>::Ptr ClusterCloud (new pcl::PointCloud<PointType>);

                for(int& index : clusterIdx.indices)
                { 
                    PointType pt = preprocessed_pointCloud->points[index];
                    ClusterCloud->points.push_back(pt);
                }

                ClusterCloud->width = ClusterCloud->points.size();
                ClusterCloud->height = 1;
                ClusterCloud->is_dense = true;

                bigObject_Cloud_vector->push_back(ClusterCloud);
            } 

            // 작은 장애물
                // 조건에 따른 추가 바람
        }
    }
}


//clustercone + processObject --> scale car
void Clustering::processObject(const std::shared_ptr<std::vector<pcl::PointCloud<PointType>::Ptr>>& input_pointClouds, const std::shared_ptr<visualization_msgs::MarkerArray>& markerarray, const std::shared_ptr<visualization_msgs::MarkerArray>& markerarray_vis)
{
    int Id = 0;
    visualization_msgs::Marker marker;
    for (pcl::PointCloud<PointType>::Ptr cluster : *input_pointClouds){
        PointType minPoint, maxPoint;
        pcl::getMinMax3D(*cluster, minPoint, maxPoint);
        ++Id;
        double ori_x, ori_y, ori_z, ori_w,r, theta_avg, mean_y, mean_x,mid_x, mid_y, thetas, theta, pos_x, pos_y, Lshape_fitting_cost;
        std::tie(theta, mean_x, mean_y, Lshape_fitting_cost) = get_best_theta(*cluster);
        cout << "ID - " << Id << std::endl;
        cout << "    L-shaep-fitting cost: " << Lshape_fitting_cost << endl; 
        cout << "    Heading: " << theta << endl; 

        visualization_msgs::Marker bbox;
        bbox = bbox_3d(*cluster, ego_heading_deg, ego_x, ego_y, Id);
        marker.header.frame_id = "base_link";
        marker.header.stamp = ros::Time::now();
        marker.type = visualization_msgs::Marker::CUBE;
        marker.action = visualization_msgs::Marker::ADD;

        marker.pose.position.x = mean_x;
        marker.pose.position.y = mean_y;
        marker.pose.position.z = 0.25;  //(maxPoint.z + minPoint.z) / 2; // 11/7 이부분을 수정해야하는 것인가?

        tf::Quaternion quaternion = tf::createQuaternionFromYaw(theta*DEG_TO_RAD);
        marker.pose.orientation.x = quaternion.x();
        marker.pose.orientation.y = quaternion.y();
        marker.pose.orientation.z = quaternion.z();
        marker.pose.orientation.w = quaternion.w();
        
        marker.scale.x = fabs(maxPoint.x - minPoint.x);
        marker.scale.y = fabs(maxPoint.y - minPoint.y);
        marker.scale.z = 0.5; //fabs(maxPoint.z - minPoint.z); // 11/7 이 부분을 수정해야하는가?

        marker.color.r = 0.8;
        marker.color.g = 0.8;
        marker.color.b = 0.8;
        marker.color.a = 1.0;
        marker.id = Id;
        marker.lifetime = ros::Duration(0.1);

        markerarray->markers.push_back(marker);
        markerarray_vis->markers.push_back(bbox);
        
    }
}


/*
void Clustering::clusterObject(const pcl::PointCloud<PointType>::Ptr& input_pointCloud, const std::shared_ptr<visualization_msgs::MarkerArray>& smallObject_MarkerArray, const std::shared_ptr<visualization_msgs::MarkerArray>& bigObject_MarkerArray, const pcl::PointCloud<PointType>::Ptr& debug_pointCloud)
{
// 다운샘플링
    pcl::PointCloud<PointType>::Ptr downsample_pointCloud(new pcl::PointCloud<PointType>);
    pcl::VoxelGrid<PointType> voxelGrid;
    voxelGrid.setInputCloud(input_pointCloud);
    voxelGrid.setLeafSize(0.1f, 0.1f, 0.1f);
    voxelGrid.filter(*downsample_pointCloud);

// 벽면제거
    float z_threshold = 1.7 + LI_TO_GND_Z;  // 차량의 높이보다 좀 높아야됨
    float xy_tolerance = 0.3;  // 벽면 제거를 위한 x, y 좌표의 제거 범위
    pcl::PassThrough<PointType> pass;
    pass.setInputCloud(downsample_pointCloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(z_threshold, std::numeric_limits<float>::max());
    pcl::PointIndices::Ptr wall_indices(new pcl::PointIndices());
    pass.filter(wall_indices->indices);

    // 벽 포인트의 x, y 범위 내 포인트 제거
    std::set<int> indices_to_remove;
    for (int index : wall_indices->indices) {
        PointType wall_point = downsample_pointCloud->points[index];

        for (size_t i = 0; i < downsample_pointCloud->points.size(); ++i) {
            if (std::abs(downsample_pointCloud->points[i].x - wall_point.x) <= xy_tolerance &&
                std::abs(downsample_pointCloud->points[i].y - wall_point.y) <= xy_tolerance &&
                downsample_pointCloud->points[i].z < wall_point.z) {
                indices_to_remove.insert(i);
            }
        }
    }

    // 중복을 제거한 인덱스를 사용하여 포인트 제거
    pcl::PointIndices::Ptr final_indices(new pcl::PointIndices());
    final_indices->indices.assign(indices_to_remove.begin(), indices_to_remove.end());

    pcl::PointCloud<PointType>::Ptr noWall_PointCloud(new pcl::PointCloud<PointType>);
    pcl::ExtractIndices<PointType> extract;
    if (!downsample_pointCloud->points.empty() && !final_indices->indices.empty()) {
        extract.setInputCloud(downsample_pointCloud);
        extract.setIndices(final_indices);
        extract.setNegative(true);
        extract.filter(*noWall_PointCloud);
    } else {
        // 에러 처리 또는 빈 클라우드 처리
        std::cout << "No points to process after filtering." << std::endl;
    }
    pcl::PointCloud<PointType>::Ptr preprocessed_pointCloud(new pcl::PointCloud<PointType>);
    *preprocessed_pointCloud = *noWall_PointCloud;
    *debug_pointCloud = *preprocessed_pointCloud;
    
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<PointType>::Ptr tree(new pcl::search::KdTree<PointType>());

   // 노멀 계산을 위한 Normal Estimation 객체 생성
    pcl::NormalEstimation<PointType, pcl::Normal> normal_estimator;
    normal_estimator.setSearchMethod(tree);
    normal_estimator.setInputCloud(preprocessed_pointCloud);
    normal_estimator.setKSearch(30);  // 인접 포인트 수 설정
    normal_estimator.compute(*normals);

    // Region Growing 알고리즘 설정
    pcl::RegionGrowing<PointType, pcl::Normal> reg;
    reg.setMinClusterSize(50);
    reg.setMaxClusterSize(500);
    reg.setSearchMethod(tree);
    reg.setNumberOfNeighbours(30);  // 인접한 포인트 수
    reg.setInputCloud(preprocessed_pointCloud);
    reg.setInputNormals(normals);
    reg.setSmoothnessThreshold(30.0 / 180.0 * M_PI);  // 곡률 임계값
    reg.setCurvatureThreshold(1.0);  // 곡률 임계값

    std::vector<pcl::PointIndices> clusterIndices;
    reg.extract(clusterIndices);
    
    
// 각 클러스터의 포인트 클라우드를 컨테이너에 저장
    PointType clusterPoint;
    float diagonal = -1;
    int markerId = 0;
    //std::shared_ptr<std::vector<float>> cornerMtx (new std::vector<float>(4));
    // 각 클러스터를 output_pointCloud_vector에 저장
    std::cout << "marker_size: " << bigObject_MarkerArray->markers.size() << std::endl;
    smallObject_MarkerArray->markers.clear();
    bigObject_MarkerArray->markers.clear();
    for(pcl::PointIndices& clusterIdx : clusterIndices)
    {
        // x, y의 최대값 최소값 저장 포인트
        PointType minXPoint, maxXPoint, minYPoint, maxYPoint, maxZPoint;
        minXPoint.x = minXPoint.y = minXPoint.z = std::numeric_limits<float>::max();
        maxXPoint.x = maxXPoint.y = maxXPoint.z = std::numeric_limits<float>::lowest();
        minYPoint.x = minYPoint.y = minYPoint.z = std::numeric_limits<float>::max();
        maxYPoint.x = maxYPoint.y = maxYPoint.z = std::numeric_limits<float>::lowest();
        maxZPoint.x = maxZPoint.y = maxZPoint.z = std::numeric_limits<float>::lowest();

        // 받은 인덱스의 포인트의 평균
        clusterPoint.x = 0.0; clusterPoint.y = 0.0; clusterPoint.z = 0.0;
        for(int& index : clusterIdx.indices)
        { 
            PointType point = preprocessed_pointCloud->points[index];
            point.z = 0;
            if (point.x > maxXPoint.x) maxXPoint = point;
            if (point.x < minXPoint.x) minXPoint = point;
            if (point.y > maxYPoint.y) maxYPoint = point;
            if (point.y < minYPoint.y) minYPoint = point;
            if (point.z > maxZPoint.z) maxZPoint = point;

            clusterPoint.x += point.x;
            clusterPoint.y += point.y;
        }
        std::cout << "maxX: " << maxXPoint.x << " minX: " << minXPoint.x << std::endl;
        std::cout << "maxY: " << maxYPoint.y << " minY: " << minYPoint.y << std::endl;

        clusterPoint.x /= clusterIdx.indices.size();
        clusterPoint.y /= clusterIdx.indices.size();

        float scaleX = maxXPoint.x - minXPoint.x;
        float scaleY = maxYPoint.y - minYPoint.y;

        // 큰 장애물
        if (((1.5 < scaleX && scaleX < 4.5) && (0.5 < scaleY && scaleY < 4.5)) || 
            ((1.5 < scaleY && scaleY < 4.5) && (0.5 < scaleX && scaleX < 4.5)))
        {
            if (maxZPoint.z + LI_TO_GND_Z > 2.3) {
                continue;
            }
            visualization_msgs::Marker carMarker;
            carMarker.header.frame_id = "map"; // 모든 마커에 대해 동일한 frame_id 사용
            carMarker.header.stamp = rosTime;
            carMarker.ns = "car_markers";
            carMarker.id = markerId++; // 각 마커에 고유 ID 부여
            carMarker.type = visualization_msgs::Marker::CUBE;
            carMarker.action = visualization_msgs::Marker::ADD;

            carMarker.scale.x = 4;      // 차량(장애물) x 방향 길이
            carMarker.scale.y = 2;      // 차량(장애물) y 방향 길이
            carMarker.scale.z = 1.5;                            // 차량(장애물) z 방향 길이

            carMarker.pose.position.x = minXPoint.x + carMarker.scale.x / 2;        // 중심 x 위치
            carMarker.pose.position.y = (minYPoint.y + maxYPoint.y) / 2;            // 중심 y 위치
            carMarker.pose.position.z = LI_TO_GND_Z / 2 + carMarker.scale.z / 2;    // 중심 z 위치

            // 마커의 색상 및 기타 설정
            carMarker.color.r = 0.0;
            carMarker.color.g = 1.0;
            carMarker.color.b = 0.0;
            carMarker.color.a = 0.2;

            // 큰 장애물 MarkerArray에 추가
            bigObject_MarkerArray->markers.push_back(carMarker);
        } 
        // 작은 장애물

        

    }
}
*/
/** 
 ** 콘 전용 클러스터링
*/
//파라미터만 수정
void Clustering::clusterCone(const pcl::PointCloud<PointType>::Ptr& input_pointCloud, const pcl::PointCloud<PointType>::Ptr& output_pointCloud)
{
// 클러스터링 설정 및 실행
    pcl::EuclideanClusterExtraction<PointType> coneClusterExtractor;
    pcl::search::KdTree<PointType>::Ptr tree(new pcl::search::KdTree<PointType>());

    // 클러스터링 대상 지정
    tree->setInputCloud(input_pointCloud);
    
    // 클러스터 크기 설정(콘 크기에 맞춘게 아니라 자동차 크기에 맞추서 조정)
    coneClusterExtractor.setClusterTolerance(0.15);
    coneClusterExtractor.setMinClusterSize(2);
    coneClusterExtractor.setMaxClusterSize(80);

    // 클러스터 검색 알고리즘 지정
    coneClusterExtractor.setSearchMethod(tree);

    // 각 클러스터에 위치하는 포인트를 가르키는 인덱스
    coneClusterExtractor.setInputCloud(input_pointCloud);
    std::vector<pcl::PointIndices> clusterIndices;
    coneClusterExtractor.extract(clusterIndices);

// 각 클러스터의 포인트 클라우드를 컨테이너에 저장
    PointType clusterPoint;
    // 각 클러스터를 output_pointCloud_vector에 저장
    for(pcl::PointIndices& clusterIdx : clusterIndices)
    {
        // x, y의 최대값 최소값 저장 변수
        float maxX = std::numeric_limits<float>::min(); 
        float minX = std::numeric_limits<float>::max();
        float maxY = std::numeric_limits<float>::min();
        float minY = std::numeric_limits<float>::max();

        // 받은 인덱스의 포인트의 평균
        clusterPoint.x = 0.0; clusterPoint.y = 0.0; clusterPoint.z = 0.0;
        for(int& index : clusterIdx.indices)
        { 
            PointType point = input_pointCloud->points[index];
            if (point.x > maxX) maxX = point.x;
            if (point.x < minX) minX = point.x;
            if (point.y > maxY) maxY = point.y;
            if (point.y < minY) minY = point.y;

            clusterPoint.x += point.x;
            clusterPoint.y += point.y;
        }

        clusterPoint.x /= clusterIdx.indices.size();
        clusterPoint.y /= clusterIdx.indices.size();

        output_pointCloud->points.push_back(clusterPoint);
    }
}


void Clustering::clusterCar(const pcl::PointCloud<PointType>::Ptr& input_pointCloud, const std::shared_ptr<std::vector<pcl::PointCloud<PointType>::Ptr>>& bigObject_Cloud_vector)
{
// 클러스터링 설정 및 실행
    pcl::EuclideanClusterExtraction<PointType> coneClusterExtractor;
    pcl::search::KdTree<PointType>::Ptr tree(new pcl::search::KdTree<PointType>());

    // 클러스터링 대상 지정
    tree->setInputCloud(input_pointCloud);
    
    // 클러스터 크기 설정 -->edit!!
    coneClusterExtractor.setClusterTolerance(0.3); //distance
    coneClusterExtractor.setMinClusterSize(10);  //min point
    coneClusterExtractor.setMaxClusterSize(100); //max point

    // 클러스터 검색 알고리즘 지정
    coneClusterExtractor.setSearchMethod(tree);

    // 각 클러스터에 위치하는 포인트를 가르키는 인덱스
    coneClusterExtractor.setInputCloud(input_pointCloud);
    std::vector<pcl::PointIndices> clusterIndices;
    coneClusterExtractor.extract(clusterIndices);

// 각 클러스터의 포인트 클라우드를 컨테이너에 저장
    PointType clusterPoint;
    // 각 클러스터를 output_pointCloud_vector에 저장
    for(pcl::PointIndices& clusterIdx : clusterIndices)
    {
        // x, y의 최대값 최소값 저장 변수
        float maxX = std::numeric_limits<float>::min(); 
        float minX = std::numeric_limits<float>::max();
        float maxY = std::numeric_limits<float>::min();
        float minY = std::numeric_limits<float>::max();

        // 받은 인덱스의 포인트의 평균
        clusterPoint.x = 0.0; clusterPoint.y = 0.0; clusterPoint.z = 0.0;
        for(int& index : clusterIdx.indices)
        { 
            PointType point = input_pointCloud->points[index];
            if (point.x > maxX) maxX = point.x;
            if (point.x < minX) minX = point.x;
            if (point.y > maxY) maxY = point.y;
            if (point.y < minY) minY = point.y;

            clusterPoint.x += point.x;
            clusterPoint.y += point.y;
        }

        clusterPoint.x /= clusterIdx.indices.size();
        clusterPoint.y /= clusterIdx.indices.size();

        pcl::PointCloud<PointType>::Ptr ClusterCloud (new pcl::PointCloud<PointType>);

        for(int& index : clusterIdx.indices)
        { 
            PointType pt = input_pointCloud->points[index];
            ClusterCloud->points.push_back(pt);
        }
        ClusterCloud->width = ClusterCloud->points.size();
        ClusterCloud->height = 1;
        ClusterCloud->is_dense = true;

        bigObject_Cloud_vector->push_back(ClusterCloud);
    }
}


/**
 ** 중심점을 선정한 뒤 해당 중심점을 기준으로 ROI 설정
*/
void Clustering::setConeROI(const pcl::PointCloud<PointType>::Ptr& input_pointCloud, const pcl::PointCloud<PointType>::Ptr& output_pointCloud, const std::shared_ptr<visualization_msgs::MarkerArray>& ROI_MarkerArray)
{
    if (input_pointCloud->points.size() < 4)
    {
        return;
    }

    auto start = std::chrono::high_resolution_clock::now();
    //std::vector<PointType> area_points;
    pcl::PointCloud<PointType>::Ptr temp_pointCloud(new pcl::PointCloud<PointType>);
    std::shared_ptr<std::vector<float>> x_values = std::make_shared<std::vector<float>>();
    std::shared_ptr<std::vector<float>> y_values = std::make_shared<std::vector<float>>();
    pcl::KdTreeFLANN<PointType> kdtree;

    PointType midPoint;
    midPoint.x = 1.0;
    midPoint.y = 0;

    float prev_coeffs = 0;

    for (int i = 0; i < 15; ++i)
    {
        PointType pastMidPoint;
        pastMidPoint.x = midPoint.x;
        pastMidPoint.y = midPoint.y;

        x_values->clear();
        y_values->clear();

    // 초기화 및 영역 재선언
        float startX = midPoint.x - 1.5;
        float endX = midPoint.x + 1.5;
        float startY = midPoint.y - 2.5;
        float endY = midPoint.y + 2.5;

    // 선형회귀 알고리즘
        for (PointType& point : input_pointCloud->points)
        {
            float fdis = pow(point.x - pastMidPoint.x, 2) + pow(point.y - pastMidPoint.y, 2);
            if (fdis < 2.25) // || (startY < point.y && point.y < endY) && (startX < point.x && point.x < endX)) 
            {
                x_values->push_back(point.x);
                y_values->push_back(point.y);
                temp_pointCloud->points.push_back(point);
            }
        }

        if (x_values->size() <= 2)
        {
            continue;
        }

        const int n = x_values->size();
        Eigen::MatrixXd X(n, 2);
        Eigen::VectorXd Y(n);

        for (int i = 0; i < n; ++i)
        {
            X(i, 0) = (*x_values)[i];
            X(i, 1) = 1;
            Y(i)    = (*y_values)[i];
        }
        
        // 선형회귀 수치 업데이트 지점 (위는 t-1번째, 아래는 t번쨰)
        Eigen::VectorXd coeffs = X.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(Y);
    
        // 최대 기울기 변화량
        float w = 15;

        // 기울기 변화량 최대 설정
        if (coeffs[0] < prev_coeffs)
        {
            coeffs[0] = std::min(tan(atan(prev_coeffs) - w), static_cast<float>(coeffs[0]));
        }
        else
        {
            coeffs[0] = std::max(tan(atan(prev_coeffs) + w), static_cast<float>(coeffs[0]));
        }

        // 디버깅용
        float start_x = *std::min_element(x_values->begin(), x_values->end());
        float end_x = *std::max_element(x_values->begin(), x_values->end());
        
        int num = 0;
        PointType linear_point;

        if (midPoint.x > 1.8)
        {
            coeffs[0] *= 1.3;
        }

        linear_point.x = midPoint.x;
        linear_point.y = coeffs[0] * midPoint.x + coeffs[1];
        output_pointCloud->push_back(linear_point);

        // 한 영역 넘어갈때마다의 거리
        float dis = 0.2;

        // midPoint 업데이트
        midPoint.x += dis / std::sqrt(std::pow(coeffs[0], 2) + 1);
        midPoint.y = coeffs[0] * midPoint.x + coeffs[1];

        // 기울기 저장
        prev_coeffs = coeffs[0];

        kdtree.setInputCloud(temp_pointCloud);
        std::vector<int> pointIdxNKNSearch(1);
        std::vector<float> pointNKNSquaredDistance(1);

        // 콘과 너무 가까울 시 수정 거리
        float feedbackDis = 0.05;
        
        if (kdtree.nearestKSearch(midPoint, 1, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
        {
            // 가장 가까운 포인트의 인덱스는 pointIdxNKNSearch[0]에, 거리의 제곱은 pointNKNSquaredDistance[0]에 저장됩니다.
            if (pointNKNSquaredDistance[0] < 0.25)
            {
                // 만약 가장 가까운 포인트가 좌측이라면
                if ((midPoint.y-temp_pointCloud->points[pointIdxNKNSearch[0]].y) > coeffs[0] * (midPoint.x-temp_pointCloud->points[pointIdxNKNSearch[0]].x))
                {
                    midPoint.x += feedbackDis * 1.0       / sqrt(1 + coeffs[0]*coeffs[0]);
                    midPoint.y += feedbackDis * coeffs[0] / sqrt(1 + coeffs[0]*coeffs[0]);
                }
                else
                {
                    midPoint.x -= feedbackDis * 1.0       / sqrt(1 + coeffs[0]*coeffs[0]);
                    midPoint.y -= feedbackDis * coeffs[0] / sqrt(1 + coeffs[0]*coeffs[0]);
                }
            }
        }

        // 디버깅용 마커 생성
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";  // 혹은 사용자의 특정 frame
        marker.header.stamp = ros::Time::now();

        marker.ns = "circles";
        marker.id = i;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;

        marker.pose.position.x = midPoint.x;
        marker.pose.position.y = midPoint.y;
        marker.pose.position.z = -0.7;

        marker.scale.x = 1;  // 원의 지름으로 설정
        marker.scale.y = 1;  // 원의 지름으로 설정
        marker.scale.z = 0.1;

        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.color.a = 0.05;
        
        ROI_MarkerArray->markers.push_back(marker);
    }

    for (visualization_msgs::Marker& marker : ROI_MarkerArray->markers)
    {
        x_values->push_back(marker.pose.position.x);
        y_values->push_back(marker.pose.position.y);
    }

    const int n = x_values->size();
    Eigen::MatrixXd X(n, 4); // 3차 방정식에 맞게 열의 수를 4로 변경
    Eigen::VectorXd Y(n);

    for (int j = 0; j < n; ++j)
    {
        X(j, 0) = std::pow((*x_values)[j], 3); // x^3 항 추가
        X(j, 1) = std::pow((*x_values)[j], 2); // x^2 항
        X(j, 2) = (*x_values)[j];              // x 항
        X(j, 3) = 1;                           // 상수 항
        Y(j)    = (*y_values)[j];
    }

    // 디버깅용 라인 생성
    Eigen::VectorXd coeffs = X.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(Y);
    std::cout << coeffs[0] << "x^3 + " << coeffs[1] << "x^2 + " << coeffs[2] << "x + " << coeffs[3] << std::endl;

    visualization_msgs::Marker Linemarker;
    Linemarker.header.frame_id = "/map";
    Linemarker.header.stamp = rosTime;
    Linemarker.id = 1001;
    Linemarker.type = visualization_msgs::Marker::LINE_STRIP;
    Linemarker.action = visualization_msgs::Marker::ADD;
    Linemarker.scale.x = 0.2;
    Linemarker.color.a = 0.8;

    Linemarker.ns = "midLine";
    Linemarker.color.g = 1.0;
    
    for (float x = 0; x < 15.0; x += 0.5)
    {
        float y = coeffs[0] * x*x + coeffs[1] * x + coeffs[2];
        geometry_msgs::Point p;
        p.x = x;
        p.y = y;
        p.z = -0.5;
        Linemarker.points.push_back(p);
    }
    ROI_MarkerArray->markers.push_back(Linemarker);

    // ROI 설정
    pcl::KdTreeFLANN<PointType> kdtreeCluster;
    kdtreeCluster.setInputCloud(input_pointCloud);
    *temp_pointCloud = *output_pointCloud;
    output_pointCloud->clear();
    
    std::unordered_set<int> all_indices;

    for(PointType& point : temp_pointCloud->points){
        std::vector<int> idxes;
        std::vector<float> _;
        idxes.clear();
        _.clear();

        kdtreeCluster.radiusSearch(point, 1, idxes, _);
        
        for (const auto& idx : idxes) {
            all_indices.insert(idx);
        }
    } 

    for (const auto& idx : all_indices){
        output_pointCloud->points.push_back(input_pointCloud->points[idx]);
    }

    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

    // 경과 시간 출력
    std::cout << "소요 시간: " << duration.count() << "ms" << std::endl;
}

/**
 ** 콘 좌우 구별
*/
//!this jinsk
void Clustering::identifyLRcone_v2(const pcl::PointCloud<PointType>::Ptr& input_pointCloud, const pcl::PointCloud<PointType>::Ptr& output_Rcone_pointCloud, const pcl::PointCloud<PointType>::Ptr& output_Lcone_pointCloud, const std::shared_ptr<visualization_msgs::MarkerArray>&  debug_Rcone_pointCloud = nullptr, const std::shared_ptr<visualization_msgs::MarkerArray>&  debug_Lcone_pointCloud = nullptr, const std::shared_ptr<visualization_msgs::MarkerArray>&  mid_Line = nullptr)
{
    pcl::PointCloud<PointType>::Ptr temp_pointCloud(new pcl::PointCloud<PointType>);
    *temp_pointCloud = *input_pointCloud;

    //x:front, y:left, z:up

    PointType LPoint;
    LPoint.x = 0;
    LPoint.y = 2;
    LPoint.z = 0;

    PointType RPoint;
    RPoint.x = 0;
    RPoint.y = -2;
    RPoint.z = 0;

    PointType backstep_LPoint;
    backstep_LPoint.x = 0;
    backstep_LPoint.y = 0;
    backstep_LPoint.z = 0;

    PointType backstep_RPoint;
    backstep_RPoint.x = 0;
    backstep_RPoint.y = 0;
    backstep_RPoint.z = 0;

    HungarianAlgorithm HungAlgo;
    
    if (temp_pointCloud->points.size() < 4)
    {
        return;
    }

    if (temp_pointCloud->points[temp_pointCloud->points.size() - 1].y > 2)
    {
        backstep_LPoint.x = -0.5;
        backstep_LPoint.y = 0;

        backstep_RPoint.x = -0.1;
        backstep_RPoint.y = -1;
    }
    // 만약 우회전이라면
    else if (temp_pointCloud->points[temp_pointCloud->points.size() - 1].y < -2)
    {
        backstep_LPoint.x = -0.1;
        backstep_LPoint.y = 1;

        backstep_RPoint.x = -0.5;
        backstep_RPoint.y = 0;
    }
    // 직진이라면
    else
    {
        backstep_LPoint.x = -0.5;
        backstep_LPoint.y = 0.25;

        backstep_RPoint.x = -0.5;
        backstep_RPoint.y = -0.25;
    }

    //here to start

    // 헝가리안 알고리즘으로 포인트 1대1 할당
    for (unsigned int k = 0; k < (temp_pointCloud->points.size()-3) ; k++)
    {
        vector<PointType> LRPoint = {LPoint, RPoint};
        std::vector<std::vector<double>> costMatrix(2, std::vector<double>(temp_pointCloud->points.size(), 0));
        vector<int> assignment;
        for(unsigned int i = 0; i < 2; i++)
        {
            for(unsigned int j = 0; j < temp_pointCloud->points.size(); j++)
            {
                costMatrix[i][j] = pow(LRPoint[i].x - temp_pointCloud->points[j].x, 2) + pow(LRPoint[i].y - temp_pointCloud->points[j].y, 2);
            }
        }

        HungAlgo.Solve(costMatrix, assignment);

        LPoint = temp_pointCloud->points[assignment[0]];
        RPoint = temp_pointCloud->points[assignment[1]];

        output_Lcone_pointCloud->points.push_back(LPoint);
        output_Rcone_pointCloud->points.push_back(RPoint);

        // 지목된 포인트는 계산에서 제외
        temp_pointCloud->points[assignment[0]].x = 999999;
        temp_pointCloud->points[assignment[0]].y = 999999;

        temp_pointCloud->points[assignment[1]].x = 999999;
        temp_pointCloud->points[assignment[1]].y = 999999;

        // 다음 좌표 보정
        LPoint.x += backstep_LPoint.x;
        LPoint.y += backstep_LPoint.y;

        RPoint.x += backstep_RPoint.x;
        RPoint.y += backstep_RPoint.y;
    }
    

    visualization_msgs::Marker line_strip_L;
    line_strip_L.header.frame_id = "map";  // 좌표계 설정
    line_strip_L.header.stamp = rosTime;
    line_strip_L.type = visualization_msgs::Marker::LINE_STRIP;
    line_strip_L.action = visualization_msgs::Marker::ADD;

    line_strip_L.scale.x = 0.05; line_strip_L.color.r = 1.0; line_strip_L.color.g = 0.0; line_strip_L.color.b = 0.0; line_strip_L.color.a = 1.0;

    visualization_msgs::Marker line_strip_R;
    line_strip_R.header.frame_id = "map";  // 좌표계 설정
    line_strip_R.header.stamp = rosTime;
    line_strip_R.type = visualization_msgs::Marker::LINE_STRIP;
    line_strip_R.action = visualization_msgs::Marker::ADD;

    line_strip_R.scale.x = 0.05; line_strip_R.color.r = 0.0; line_strip_R.color.g = 0.0; line_strip_R.color.b = 1.0; line_strip_R.color.a = 1.0;
    
    for (const PointType& point : output_Lcone_pointCloud->points)
    {
        if (point.x < 3) 
        {
            geometry_msgs::Point p;
            p.x = point.x; p.y = point.y; p.z = point.z;
            line_strip_L.points.push_back(p);
        }
    }
    debug_Lcone_pointCloud->markers.push_back(line_strip_L);

    for (const PointType& point : output_Rcone_pointCloud->points)
    {
        if (point.x < 3)
        {
            geometry_msgs::Point p;
            p.x = point.x; p.y = point.y; p.z = point.z;
            line_strip_R.points.push_back(p);
        }
    }
    debug_Rcone_pointCloud->markers.push_back(line_strip_R);

    visualization_msgs::Marker center_line;
    center_line.header.frame_id = "map";
    center_line.header.stamp = rosTime;
    center_line.ns = "center_line";
    center_line.id = 0;
    center_line.type = visualization_msgs::Marker::LINE_STRIP;
    center_line.action = visualization_msgs::Marker::ADD;
    center_line.scale.x = 0.1; center_line.color.r = 0.0; center_line.color.g = 1.0; center_line.color.b = 0.0; center_line.color.a = 1.0;
    
    int R_marker_len = debug_Rcone_pointCloud->markers[0].points.size();
    int L_marker_len = debug_Lcone_pointCloud->markers[0].points.size();
    int min_len = (R_marker_len < L_marker_len) ? R_marker_len : L_marker_len;
    int max_len = (R_marker_len > L_marker_len) ? R_marker_len : L_marker_len;

    for (int i = 0; i < ((min_len < 2) ? min_len : 2); i++)
    {
        geometry_msgs::Point p;
        p.x = (debug_Lcone_pointCloud->markers[0].points[i].x + debug_Rcone_pointCloud->markers[0].points[i].x) / 2.0;
        p.y = (debug_Lcone_pointCloud->markers[0].points[i].y + debug_Rcone_pointCloud->markers[0].points[i].y) / 2.0;
        p.z = 0.0;
        center_line.points.push_back(p);
    }
    
    mid_Line->markers.push_back(center_line);
    
    // 불안정한 포인트 제거
    // pcl::PointCloud<PointType>::Ptr temp_Lcone_pointCloud(new pcl::PointCloud<PointType>);
    // pcl::PointCloud<PointType>::Ptr temp_Rcone_pointCloud(new pcl::PointCloud<PointType>);
    // *temp_Lcone_pointCloud = *output_Lcone_pointCloud;
    // *temp_Rcone_pointCloud = *output_Rcone_pointCloud;
    // output_Lcone_pointCloud->points.clear();
    // output_Rcone_pointCloud->points.clear();

    // if (output_Lcone_pointCloud->points.size() > 2) 
    // {
    //     output_Lcone_pointCloud->points.resize(output_Lcone_pointCloud->points.size() - 1);
    // }
    // if (output_Rcone_pointCloud->points.size() > 2) 
    // {
    //     output_Rcone_pointCloud->points.resize(output_Rcone_pointCloud->points.size() - 1);
    // }

    // for (PointType& point : temp_Lcone_pointCloud->points)
    // {
    //     if (point.x * point.x + point.y * point.y < 4)
    //     {
    //         output_Lcone_pointCloud->points.push_back(point);
    //     }
    // }
    // for (PointType& point : temp_Rcone_pointCloud->points)
    // {
    //     if (point.x * point.x + point.y * point.y < 4)
    //     {
    //         output_Rcone_pointCloud->points.push_back(point);
    //     }
    // }

}

float x_old = 0;
float y_old = 0;

void Clustering::trackingCone(const pcl::PointCloud<PointType>::Ptr& input_L_pointCloud, const pcl::PointCloud<PointType>::Ptr& input_R_pointCloud, const pcl::PointCloud<PointType>::Ptr& output_pointCloud)
{   
    // 한쪽 라인만 보이는경우 보정값
    float h = 0.3;

    pcl::PointCloud<PointType>::Ptr temp_pointCloud(new pcl::PointCloud<PointType>());

    // 좌측, 우측 포인트 클라우드 모두 있다면,
    if (!input_L_pointCloud->points.empty() && !input_R_pointCloud->points.empty())
    {
        // x가 작은 것부터 큰 순서대로 정렬
        pcl::PointCloud<PointType> sorted_L = *input_L_pointCloud;
        pcl::PointCloud<PointType> sorted_R = *input_R_pointCloud;

        // 먼곳 중앙만 검출하면 됨
        std::sort(sorted_L.points.begin(), sorted_L.points.end(), [](const PointType& a, const PointType& b) { return a.x < b.x; });
        std::sort(sorted_R.points.begin(), sorted_R.points.end(), [](const PointType& a, const PointType& b) { return a.x < b.x; });

        // 두 포인트 클라우드 모두 있는 경우 중앙점을 output_pointCloud에 저장
        for (const auto& l_point : sorted_L.points)
        {
            for (const auto& r_point : sorted_R.points)
            {
                PointType center_point;
                center_point.x = (l_point.x + r_point.x) / 2.0;
                center_point.y = (l_point.y + r_point.y) / 2.0;
                center_point.z = (l_point.z + r_point.z) / 2.0;
                center_point.x = (center_point.x + x_old) / 2.0;
                center_point.y= (center_point.y + y_old) / 2.0;

                temp_pointCloud->points.push_back(center_point);
            }
        }
    }
    // 좌측만 있다면
    else if (!input_L_pointCloud->points.empty())
    {
        // input_L_pointCloud만 있는 경우, y값을 낮춰서 output_pointCloud에 저장
        pcl::PointCloud<PointType> sorted_L = *input_L_pointCloud;
        // 가장 가까이 있는거 검출
        std::sort(sorted_L.points.begin(), sorted_L.points.end(), [](const PointType& a, const PointType& b) { return a.x > b.x; });

        for (const auto& l_point : sorted_L.points)
        {
            PointType new_point = l_point;
            new_point.x += 0.1;
            new_point.y -= h; // y값을 h만큼 낮춤
            new_point.x = (new_point.x + x_old) / 2.0;
            new_point.y = (new_point.y + y_old) / 2.0;

            temp_pointCloud->points.push_back(new_point);
        }
    }
    // 우측만 있다면
    else if (!input_R_pointCloud->points.empty())
    {
        // input_R_pointCloud만 있는 경우, y값을 높여서 output_pointCloud에 저장
        pcl::PointCloud<PointType> sorted_R = *input_R_pointCloud;
        // 가장 가까이 있는거 검출
        std::sort(sorted_R.points.begin(), sorted_R.points.end(), [](const PointType& a, const PointType& b) { return a.x > b.x; });

        for (const auto& r_point : sorted_R.points)
        {
            PointType new_point = r_point;
            new_point.x += 0.1;
            new_point.y += h; // y값을 h만큼 높임
            new_point.x = (new_point.x + x_old) / 2.0;
            new_point.y = (new_point.y + y_old) / 2.0;

            temp_pointCloud->points.push_back(new_point);
        }
    }
    // 아무것도 안찍힌다면
    else
    {
        PointType pt;
        pt.x = 0.5;
        pt.y = 0;
        pt.z = 0;
        pt.x = (pt.x + x_old) / 2.0;
        pt.y = (pt.y + y_old) / 2.0;

        temp_pointCloud->points.push_back(pt);
    }

    // 경로생성
    PointType current_point1;
    current_point1.x = -0.30;
    current_point1.y = 0;
    current_point1.z = 0;

    PointType current_point2;
    current_point2.x = 0;
    current_point2.y = 0;
    current_point2.z = 0;

    // 세 점을 이용해 2차 방정식 생성
    double x1 = current_point1.x,                       y1 = current_point1.y;
    double x2 = current_point2.x,                       y2 = current_point2.y;
    double x3 = temp_pointCloud->points.back().x,       y3 = temp_pointCloud->points.back().y;

    x_old = x3;
    y_old = y3;

    double a = ((y3 - (x3 * (y2 - y1) / (x2 - x1)) - y1) / (x3 * (x3 - x1 - x2 + (x2 * x1) / x1)));
    double b = ((y2 - y1) / (x2 - x1)) - a * (x1 + x2);
    double c = y1 - (a * x1 * x1) - (b * x1);

    double steeringAngleLimit = 0.2618 ; // 최대 조향각 15도정도로 설정
    double maxSlopeChange = std::tan(steeringAngleLimit);  // 최대 기울기 변화 제한 계산

    // 곡선을 생성하여 포인트 클라우드에 추가
    float step = 0.05;
    double prevSlope = 0.0;  // 초기 기울기를 0으로 설정

    for (double x = x1; x <= x3; x += step) {
        double y = a * x * x + b * x + c;

        double currentSlope = 2 * a * x + b;

        // 기울기 변화에 제한
        if (std::abs(currentSlope - prevSlope) > maxSlopeChange) {
            currentSlope = prevSlope + maxSlopeChange * ((currentSlope > prevSlope) ? 1 : -1);
            y = currentSlope * x + c;
        }

        PointType point;
        point.x = x;
        point.y = y;
        point.z = 0.0;

        output_pointCloud->points.push_back(point);
        prevSlope = currentSlope;  // 기울기 갱신
    }

    output_pointCloud->width = output_pointCloud->points.size();
    output_pointCloud->height = 1;
    output_pointCloud->is_dense = true;
}


//----------------------------클러스터링 전용 함수--------------------------------
double Clustering::distance(const PointType& p1, const PointType& p2) {
    return sqrt((p2.x - p1.x) * (p2.x - p1.x) + (p2.y - p1.y) * (p2.y - p1.y));
}

int Clustering::orientation(const PointType& p, const PointType& q, const PointType& r) {
    double val = (q.y - p.y) * (r.x - q.x) - (q.x - p.x) * (r.y - q.y);
    return val == 0.0 ? 0 : (val > 0.0 ? 1 : 2);
}

bool Clustering::doIntersect(PointType p1, PointType q1, PointType p2, PointType q2, double L) {
    int o1 = orientation(p1, q1, p2);
    int o2 = orientation(p1, q1, q2);
    int o3 = orientation(p2, q2, p1);
    int o4 = orientation(p2, q2, q1);
    if (o1 != o2 && o3 != o4) {
        double A1 = q1.y - p1.y;
        double B1 = p1.x - q1.x;
        double C1 = A1 * p1.x + B1 * p1.y;
        double A2 = q2.y - p2.y;
        double B2 = p2.x - q2.x;
        double C2 = A2 * p2.x + B2 * p2.y;
        double det = A1 * B2 - A2 * B1;
        if (det != 0) {
            double x = (B2 * C1 - B1 * C2) / det;
            double y = (A1 * C2 - A2 * C1) / det;
            PointType intersection;
            intersection.x = x; intersection.y = y; intersection.z = 0;
            if (distance(intersection, p1) >= L && distance(intersection, q1) >= L &&
                distance(intersection, p2) >= L && distance(intersection, q2) >= L) {
                return true;
            }
        }
    }
    return false;
}


// --------------------------------------<현재 사용되지 않는 함수>---------------------------------------------
void Clustering::saveCluster(const pcl::PointCloud<PointType>::Ptr& input_pointCloud, const pcl::PointCloud<PointType>::Ptr& output_pointCloud)
{    
    auto start = std::chrono::high_resolution_clock::now();
    pcl::PointCloud<PointType>::Ptr temp_input_pointCloud (new pcl::PointCloud<PointType>());

    // 이전 데이터가 없을 시 진행하되 절대좌표를 기준으로 알고리즘을 적용할 거라 절대좌표로 전환
    if (output_pointCloud->points.size() == 0)
    {
        *output_pointCloud = *input_pointCloud;
        for (PointType& point : output_pointCloud->points)
        {
            transformRelToAbs(point);
        }
        return;
    }

    pcl::PointCloud<PointType>::Ptr unmatchedPoints (new pcl::PointCloud<PointType>());

    // input_pointCloud가 망가지지 않게 복사 및 절대좌표로 변환
    *temp_input_pointCloud = *input_pointCloud;
    for (PointType& point : temp_input_pointCloud->points)
    {
        transformRelToAbs(point);
    }

    // 헝가리안 알고리즘용 거리 표 생성
    std::vector<std::vector<double>> costMatrix(temp_input_pointCloud->points.size(), std::vector<double>(output_pointCloud->points.size()));
    vector<int> assignment;

    for (unsigned int i = 0; i < temp_input_pointCloud->points.size(); ++i) {
        for (unsigned int j = 0; j < output_pointCloud->points.size(); ++j) {
            double distance = pow(temp_input_pointCloud->points[i].x - output_pointCloud->points[j].x, 2) + pow(temp_input_pointCloud->points[i].y - output_pointCloud->points[j].y, 2);
            costMatrix[i][j] = distance;  // 유클리드 거리 ** 2 사용
        }
    }

    // Hungarian 알고리즘 사용  매칭 ->  temp_input_pointCloud->points[x] <-> output_pointCloud->points[assignment[x]]
    double cost = HungAlgo.Solve(costMatrix, assignment);
    std::cout << "-------------------------------------" << std::endl;
    for (unsigned int n = 0; n < costMatrix.size(); n++)
    {
        std::cout << n << "," << assignment[n] << "\t";
    }

	std::cout << "\ncost: " << cost << std::endl;
    std::cout << temp_input_pointCloud->points.size() << " / "<< output_pointCloud->points.size() << std::endl;

    // 매칭
    for (unsigned int n = 0; n < costMatrix.size(); n++) 
    {
        // 서로 매칭이 된다면, 칼만 필터로 좌표 업데이트
        if (assignment[n] >= 0)
        {
            Eigen::Vector2f initialState(output_pointCloud->points[assignment[n]].x, output_pointCloud->points[assignment[n]].y);
            kalman.init(initialState);
            kalman.predict();

            Eigen::Vector2f measurement(temp_input_pointCloud->points[n].x, temp_input_pointCloud->points[n].y);
            kalman.update(measurement);

            output_pointCloud->points[assignment[n]].x = kalman.state(0);
            output_pointCloud->points[assignment[n]].y = kalman.state(1);
        }
        else  // temp_input_pointCloud 안에 있는 포인트 중에서 매칭이 안된 포인트가 있다면
        {
            temp_input_pointCloud->points[n].intensity = 0;
            unmatchedPoints->points.push_back(temp_input_pointCloud->points[n]);
        }
    }

    // output_pointCloud 중에서 매칭이 안된 포인트가 있다면
    for (unsigned int n = 0; n < output_pointCloud->points.size(); ++n) {
        if (std::find(assignment.begin(), assignment.end(), n) == assignment.end()) {
            // assignment 배열에 n 인덱스가 없으면 매칭되지 않은 것
            if (output_pointCloud->points[n].x < 1)
            {

            }
            output_pointCloud->points[n].intensity -= 1;
        }
    }

    // 만약 예비 클러스터가 아무것도 없다면 채우고 리턴
    if (clusterStopOver->points.size() == 0)
    {
        *clusterStopOver = *unmatchedPoints;
        return;
    }

    // 헝가리안 알고리즘용 거리 표 리사이즈
    costMatrix.resize(clusterStopOver->points.size());
    for(auto& row : costMatrix) 
    {
        row.resize(unmatchedPoints->points.size());
    }

    // 헝가리안 알고리즘용 거리 표 생성
    for (unsigned int i = 0; i < clusterStopOver->points.size(); ++i) {
        for (unsigned int j = 0; j < unmatchedPoints->points.size(); ++j) {
            double distance = pow(clusterStopOver->points[i].x - unmatchedPoints->points[j].x, 2) + 
                              pow(clusterStopOver->points[i].y - unmatchedPoints->points[j].y, 2);
            costMatrix[i][j] = distance;  // 유클리드 거리 ** 2 사용
        }
    }
    assignment = {};
    cost = HungAlgo.Solve(costMatrix, assignment);

    for (unsigned int n = 0; n < costMatrix.size(); n++) 
    {
        // 서로 매칭이 된다면, 예비 클러스터와 발견한 클러스터와 일치하다는 의미
        if (assignment[n] >= 0)
        {
            double distance = pow(clusterStopOver->points[n].x - unmatchedPoints->points[assignment[n]].x, 2) + 
                              pow(clusterStopOver->points[n].y - unmatchedPoints->points[assignment[n]].y, 2);
            Eigen::Vector2f initialState(clusterStopOver->points[n].x, clusterStopOver->points[n].y);
            kalman.init(initialState);
            kalman.predict();

            Eigen::Vector2f measurement(unmatchedPoints->points[assignment[n]].x, unmatchedPoints->points[assignment[n]].y);
            kalman.update(measurement);

            if (clusterStopOver->points[n].intensity >= 2)
            {
                output_pointCloud->points.push_back(clusterStopOver->points[n]);
                continue;
            }
            clusterStopOver->points[n].intensity += 1;  // 생성 카운트
            clusterStopOver->points[n].x = kalman.state(0);
            clusterStopOver->points[n].y = kalman.state(1);
        }
        else  // clusterStopOver 안에 있는 포인트 중에서 매칭이 안된 포인트가 있다면 제거 진행
        {
            clusterStopOver->points[n].intensity = 0;  // 생성 카운트
        }
    }

    // unmatchedPoints 중에서 매칭이 안된 포인트가 있다면
    for (unsigned int n = 0; n < unmatchedPoints->points.size(); ++n) {
        if (std::find(assignment.begin(), assignment.end(), n) == assignment.end()) {
            clusterStopOver->points.push_back(unmatchedPoints->points[assignment[n]]);
            clusterStopOver->points[n].intensity = 0;
        }
    }

    output_pointCloud->width = output_pointCloud->points.size();
    output_pointCloud->height = 1;
    
    // --------------------------------------------------------------------------------------------------------
    
    auto end = std::chrono::high_resolution_clock::now();

    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

    // 경과 시간 출력
    std::cout << "소요 시간: " << duration.count() << "ms" << std::endl;
}

/**
 * 1. x값을 기준으로 범위를 3개 내지 4개로 소분화한다. (범위는 임의로 직접 설정한다.)
 * 2. 이때 범위는 서로 어느정도 겹치게 설정한다. 
 * 3. 각 범위별로 클러스터를 나눌 중앙 점을 구해준다.
 *    중앙점을 구하는 방법은 다음과 같다.
     *  1. 왜도를 측정해서 기울린 정도를 분석한 후 평균에서 +-를 진행할려고 했으나 연산과정이 매우 길어서 패스
     *  2. 그냥 평균으로 구하겠다.
 * 4. 각 중앙점을 연결해서 해당 중앙점을 기준으로 좌측과 우측을 구분한다.
 * 5. 차량의 위치 (0, 0, 0)을 첫 중앙점으로 두겠다.
*/
void Clustering::identifyLRcone(const pcl::PointCloud<PointType>::Ptr& input_pointCloud, pcl::PointCloud<PointType>::Ptr& output_Lcone_pointCloud, pcl::PointCloud<PointType>::Ptr& output_Rcone_pointCloud, pcl::PointCloud<PointType>::Ptr& debug_pointCloud)
{
    std::unique_ptr<std::vector<std::pair<float, float>>> midPoints (new std::vector<std::pair<float, float>>());
    midPoints->push_back(std::make_pair(0, 0));

    // 각 범위 설정
    std::unique_ptr<std::vector<std::pair<float, float>>> SplitedY = std::make_unique<std::vector<std::pair<float, float>>>
    (
        std::initializer_list<std::pair<float, float>>{{0, 4}, {2, 6}, {4, 8}}
    );

    // 각 범위별 중앙값 설정
    pcl::PointCloud<PointType>::Ptr tempPointCloud(new pcl::PointCloud<PointType>());
    float y_avg_past = 0;
    float y_avg_past_past = 0;
    for (std::pair<float, float> x_threshold : *SplitedY)
    {
        // 모든 포인트 클라우드를 순회하며 범위 안에 속하는 포인트를 찾음
        tempPointCloud->points.clear();
        for (PointType& point : input_pointCloud->points)
        {
            if (x_threshold.first <= point.x && point.x <= x_threshold.second)
            {
                PointType tempPoint;
                tempPoint.x = (x_threshold.first+x_threshold.second) / 2;
                tempPoint.y = point.y;
                tempPointCloud->points.push_back(tempPoint);
            }
        }

        // 범위 내에 있는 포인트 클라우드의 중앙점을 구함
        if (tempPointCloud->points.size() > 1)                       // 포인트가 1개 있으면 너무 편향되므로 2개 이상부터 받기로함
        {
            // 중앙점 계산
            float x_avg = 0;
            float y_avg = 0;
            float y_avg_temp;
            float y_plus_minus_count = 0;
            for (PointType& point : tempPointCloud->points)
            {
                if (std::abs(y_avg_past - point.y) < CONE_BETWEEN + 5)
                {
                    x_avg = point.x;
                    y_avg += point.y;
                }
            }
            y_avg = y_avg / tempPointCloud->points.size();

            for (PointType& point : tempPointCloud->points)
            {
                if (y_avg < point.y)
                {
                    y_plus_minus_count += 1;
                }
                if (y_avg > point.y)
                {
                    y_plus_minus_count -= 1;
                }
            }
            // 중앙점에 대한 보정
            if (y_plus_minus_count > 0) 
            {
                y_avg -= 1/tempPointCloud->points.size();
            }
            if (y_plus_minus_count < 0)
            {
                y_avg += 1/tempPointCloud->points.size();
            }

            // 너무 민감하게 바뀌지 않도록 이전 값과 비교해서 덜 이동하게 바꿈
            y_avg_temp = y_avg;
            y_avg = (7*y_avg+y_avg_past) / 8;
            y_avg_past_past = y_avg_past;
            y_avg_past = y_avg_temp;


            midPoints->push_back(std::make_pair(x_avg, y_avg));
        }
    }

    // 각 범위별로 구한 중앙점을 서로 연결해서 선을 만들고 외적을 구해 해당 점이 어디에 있는지 판단
    for(auto midPoint = midPoints->begin(); midPoint != midPoints->end() - 1; ++midPoint)
    {
        std::pair<float, float> firstMidPoint = *midPoint;
        std::pair<float, float> secondMidPoint = *(midPoint + 1);

        for (PointType& point : input_pointCloud->points)
        {
            if (firstMidPoint.first < point.x && point.x < secondMidPoint.first)
            {
                if (preprocessor.crossLine(firstMidPoint, secondMidPoint, point) > 0)
                {
                    output_Lcone_pointCloud->points.push_back(point);
                } else
                {
                    output_Rcone_pointCloud->points.push_back(point);
                }
            }
        }
    }

    // 가운데 라인을 확인하는 용도의 pointCloud
    for (const auto& point : *midPoints) 
    {
        PointType pcl_point;
        pcl_point.x = point.first;
        pcl_point.y = point.second;
        pcl_point.z = 0;

        debug_pointCloud->points.push_back(pcl_point);
    }

    debug_pointCloud->width = debug_pointCloud->points.size();
    debug_pointCloud->height = 1;
    debug_pointCloud->is_dense = true;
}


/**
 * 다음 점은 양측의 점으로부터 특정 계산을 통해 선정된다.
     * 그 계산법으로 생각나는 것은 전체 포인트에서 n차방정식 선형회귀로 구하는 것이다. (선형회귀 모델은 좀 여러가지라 많이 시도해봐야겠다.)
     * 그리고 해당 범위를 선형회귀해서 나온 라인과 찍힌 점을 비교해서 특정 거리 내에 있는 포인트를 콘으로 인식하고, 좌측 콘과 우측콘을 구분한다.
*/
/*
void Clustering::identifyLRcone(const pcl::PointCloud<PointType>::Ptr& input_pointCloud, pcl::PointCloud<PointType>::Ptr& output_Lcone_pointCloud, pcl::PointCloud<PointType>::Ptr& output_Rcone_pointCloud, pcl::PointCloud<PointType>::Ptr& debug_pointCloud)
{
    std::unique_ptr<std::vector<std::pair<float, float>>> midPoints (new std::vector<std::pair<float, float>>());
    midPoints->push_back(std::make_pair(0, 0));

    std::pair<float, float> splitedLine = {4, 0, 0};  // {x, y, 각도}

    pcl::PointCloud<PointType>::Ptr tempPointCloud(new pcl::PointCloud<PointType>());
    tempPointCloud->points.clear();
        for (PointType& point : input_pointCloud->points)
        {
            if (splitedLine[0] <= point.x && point.x <= splitedLine[1])
            {
                PointType tempPoint;
                tempPoint.x = (x_threshold.first+x_threshold.second) / 2;
                tempPoint.y = point.y;
                tempPointCloud->points.push_back(tempPoint);
            }
        }


}



/*
void Clustering::identifyLRcone(const pcl::PointCloud<PointType>::Ptr& input_pointCloud, pcl::PointCloud<PointType>::Ptr& output_Lcone_pointCloud, pcl::PointCloud<PointType>::Ptr& output_Rcone_pointCloud, pcl::PointCloud<PointType>::Ptr& debug_pointCloud)
{
    std::unique_ptr<std::vector<std::pair<float, float>>> midPoints (new std::vector<std::pair<float, float>>());
    midPoints->push_back(std::make_pair(0, 0));

    // 각 범위 설정
    std::unique_ptr<std::vector<std::pair<float, float>>> SplitedY = std::make_unique<std::vector<std::pair<float, float>>>
    (
        std::initializer_list<std::pair<float, float>>{{0, 4}, {2, 6}, {4, 8}, {6, 10}, {8, 12}, {10, 14}}
    );

    // 각 범위별 중앙값 설정
    pcl::PointCloud<PointType>::Ptr tempPointCloud(new pcl::PointCloud<PointType>());
    float y_avg_past = 0;
    float y_avg_past_past = 0;
    for (std::pair<float, float> x_threshold : *SplitedY)
    {
        // 모든 포인트 클라우드를 순회하며 범위 안에 속하는 포인트를 찾음
        tempPointCloud->points.clear();
        for (PointType& point : input_pointCloud->points)
        {
            if (x_threshold.first <= point.x && point.x <= x_threshold.second)
            {
                PointType tempPoint;
                tempPoint.x = (x_threshold.first+x_threshold.second) / 2;
                tempPoint.y = point.y;
                tempPointCloud->points.push_back(tempPoint);
            }
        }

        // 범위 내에 있는 포인트 클라우드의 중앙점을 구함
        if (tempPointCloud->points.size() > 1)                       // 포인트가 1개 있으면 너무 편향되므로 2개 이상부터 받기로함
        {
            // 중앙점 계산
            float x_avg = 0;
            float y_avg = 0;
            float y_avg_temp;
            float y_plus_minus_count = 0;
            float y_accel = 0;
            for (PointType& point : tempPointCloud->points)
            {
                y_accel = 2*y_avg_past - y_avg_past_past; 
                if (std::abs(y_avg_past - point.y) < CONE_BETWEEN + 5)
                {
                    x_avg = point.x;
                    y_avg += point.y;
                }
            }
            y_avg = y_avg / tempPointCloud->points.size();

            for (PointType& point : tempPointCloud->points)
            {
                if (y_avg < point.y)
                {
                    y_plus_minus_count += 1;
                }
                if (y_avg > point.y)
                {
                    y_plus_minus_count -= 1;
                }
            }
            // 중앙점에 대한 보정
            if (y_plus_minus_count > 0) 
            {
                y_avg -= 1;
            }
            if (y_plus_minus_count < 0)
            {
                y_avg += 1;
            }

            // 너무 민감하게 바뀌지 않도록 이전 값과 비교해서 덜 이동하게 바꿈
            y_avg_temp = y_avg;
            y_avg = (7*y_avg+y_avg_past) / 8;
            y_avg_past_past = y_avg_past;
            y_avg_past = y_avg_temp;


            midPoints->push_back(std::make_pair(x_avg, y_avg));
        }
    }

    // 각 범위별로 구한 중앙점을 서로 연결해서 선을 만들고 외적을 구해 해당 점이 어디에 있는지 판단
    for(auto midPoint = midPoints->begin(); midPoint != midPoints->end() - 1; ++midPoint)
    {
        std::pair<float, float> firstMidPoint = *midPoint;
        std::pair<float, float> secondMidPoint = *(midPoint + 1);

        for (PointType& point : input_pointCloud->points)
        {
            if (firstMidPoint.first < point.x && point.x < secondMidPoint.first)
            {
                if (preprocessor.crossLine(firstMidPoint, secondMidPoint, point) > 0)
                {
                    output_Lcone_pointCloud->points.push_back(point);
                } else
                {
                    output_Rcone_pointCloud->points.push_back(point);
                }
            }
        }
    }

    // 가운데 라인을 확인하는 용도의 pointCloud
    for (const auto& point : *midPoints) 
    {
        PointType pcl_point;
        pcl_point.x = point.first;
        pcl_point.y = point.second;
        pcl_point.z = 0;

        debug_pointCloud->points.push_back(pcl_point);
    }

    debug_pointCloud->width = debug_pointCloud->points.size();
    debug_pointCloud->height = 1;
    debug_pointCloud->is_dense = true;
}
*/






/**
 *? 알고리즘:
    * 콘 객체는 한 포인트로 표시된다는 전제
    * *<기본>
        * 1. 우선 모든 포인트를 차량과 가까운 순으로 정렬
        * 2. 가장 가까운 2개의 포인트는 y값을 이용해 좌측콘과 우축콘 구분 (좌측콘을 a,, 우측콘을 b라고 칭하고, 숫자는 쌍의 순서를 의미)
        * 3. 3번째로 가까운 포인트는 2a 혹은 2b가 될것인데 이는 1a와 1b와의 거리를 비교해 판단, 4번쨰로 가까운 포인트도 이와 같이 판단
        * 4. 5번쨰와 6번쨰 포인트도 2a와 2b를 통해 판단함
    * 
    ** <곡선>
        * 1. 곡선으로 콘이 놓일시 바깥쪽 콘이 안쪽 콘보다 많을 수 있음
        * 2. 우선 <기본>알고리즘에 의하면 한쌍씩 짝을 이루게 되는데 
        * ...
    ** <결과>
        * 이상치에 너무 민감함
*/
/*
void Clustering::identifyLRcone(const pcl::PointCloud<PointType>::Ptr& input_pointCloud, pcl::PointCloud<PointType>::Ptr& output_Lcone_pointCloud, pcl::PointCloud<PointType>::Ptr& output_Rcone_pointCloud)
{
    std::unique_ptr<std::vector<PointType>> sortedPoints(new std::vector<PointType>);
    pcl::PointCloud<PointType>::Ptr L_cone(new pcl::PointCloud<PointType>);
    pcl::PointCloud<PointType>::Ptr R_cone(new pcl::PointCloud<PointType>);
    int LIdx, RIdx;
    bool L_start;

    // 포인트 클라우드를 컨테이너에 넣고 x값이 작은것부터 큰 순서대로 정렬
    preprocessor.convertPCLtoVector(input_pointCloud, sortedPoints);
    
    // 우선 유클리디안으로 해보겠음
    std::vector<std::vector<float>> distance_table(sortedPoints->size(), std::vector<float>(sortedPoints->size(), INF));
    for (int i = 0; i < sortedPoints->size(); ++i)
    {
        for (int j = i + 1; j < sortedPoints->size(); ++j)
        {
            double dx = (*sortedPoints)[i].x - (*sortedPoints)[j].x;
            double dy = (*sortedPoints)[i].y - (*sortedPoints)[j].y;
            double distance = dx*dx + dy*dy;
            distance_table[i][j] = distance;
            distance_table[j][i] = distance;
        }
    }

    // 가장 가까운 2개의 포인트 좌우 구분 (y가 클수록 좌측)
    if ((*sortedPoints)[0].y > (*sortedPoints)[1].y) 
    {
        LIdx = 0;
        RIdx = 1;
        output_Lcone_pointCloud->push_back((*sortedPoints)[0]);
        output_Rcone_pointCloud->push_back((*sortedPoints)[1]);
    } else
    {
        LIdx = 1;
        RIdx = 0;
        output_Lcone_pointCloud->push_back((*sortedPoints)[1]);
        output_Rcone_pointCloud->push_back((*sortedPoints)[0]);
    }

    preprocessor.initCol(distance_table, RIdx, INF);
    preprocessor.initCol(distance_table, LIdx, INF);


    // 2개의 포인트로 나머지 포인트의 좌우 구분 진행
    for (int i = 0; i < 5; i++)
    {
        // 예외처리
        if (i >= sortedPoints->size()) {
            break;
        }

        // 첫번쨰 타겟은 좌측 콘으로부터 가장 가까운 대상
        auto L_min_iter = std::min_element(distance_table[LIdx].begin(), distance_table[LIdx].end());
        int L_target_index = std::distance(distance_table[LIdx].begin(), L_min_iter);
        
        float L_target_dis = distance_table[L_target_index][LIdx] - distance_table[L_target_index][RIdx];
        preprocessor.initCol(distance_table, LIdx, INF);

        // 두번째 타겟은 우측 콘으로부터 가장 가까운 대상
        auto R_min_iter = std::min_element(distance_table[RIdx].begin(), distance_table[RIdx].end());
        int R_target_index = std::distance(distance_table[RIdx].begin(), R_min_iter);
        
        float R_target_dis = distance_table[R_target_index][LIdx] - distance_table[R_target_index][RIdx];
        preprocessor.initCol(distance_table, RIdx, INF);

        // 즉, 첫번째 타겟이 좌측에 있고, 두번째 타겟이 우측에 잘 있다면
        if (L_target_dis < R_target_dis)
        {
            LIdx = L_target_index;
            RIdx = R_target_index;
            output_Lcone_pointCloud->push_back((*sortedPoints)[LIdx]);
            output_Rcone_pointCloud->push_back((*sortedPoints)[RIdx]);
        } else 
        {
            LIdx = R_target_index;
            RIdx = L_target_index;
            output_Lcone_pointCloud->push_back((*sortedPoints)[RIdx]);
            output_Rcone_pointCloud->push_back((*sortedPoints)[LIdx]);
        }
    }
}
*/
