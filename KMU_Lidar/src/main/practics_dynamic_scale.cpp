#include "pointcloud_generator.h"
#include "utility.h"
#include <laser_geometry/laser_geometry.h>
#include <pcl/common/centroid.h>
// 클래스 외부에서 static 멤버 변수 정의

using namespace std;
/*
! rosCloud를 private에 선언해주고, public에서 allocateMemory에 메모리 할당해주고 clearMemory에서 메모리 삭제해줌
sync 없애줌 : lidar 데이터와 gps 정보의 sync를 없애줌.
#include "laser_geometry/laser_geometry.h" 다운로드가 필요할 수 있음
 */

/**
 * 좌표계
 * x: front of vehicle
 * y: left side of vehicle  
 * z: upper side of vehicle
 * Forward view: negative for left, positive for right from x-axis
 * 회전: 축을 기준으로 오른손의 법칙을 따름
 * 라이다의 row: 아래서부터 위로 숫자가 증가
 * 라이다의 column: 라이다 후면부터 반시계 방향으로 증가
 */

void calculateTunnelCenterPointCloud(const pcl::PointCloud<PointType>::Ptr& leftTCloud, const pcl::PointCloud<PointType>::Ptr& rightTCloud, pcl::PointCloud<PointType>::Ptr& centerPointCloud) {
    // 결과를 저장할 포인트 클라우드를 초기화
    centerPointCloud->clear();

    bool leftAvailable = !leftTCloud->points.empty();
    bool rightAvailable = !rightTCloud->points.empty();

    PointType centerPoint;

    if (leftAvailable && rightAvailable) {
        // 좌측 및 우측 클라우드의 평균점을 계산
        Eigen::Vector4f leftCentroid, rightCentroid;
        pcl::compute3DCentroid(*leftTCloud, leftCentroid);
        pcl::compute3DCentroid(*rightTCloud, rightCentroid);

        // 좌측과 우측 중심점의 중간점을 계산
        centerPoint.x = (leftCentroid.x() + rightCentroid.x()) / 2.0;
        centerPoint.y = (leftCentroid.y() + rightCentroid.y()) / 2.0;
        centerPoint.z = (leftCentroid.z() + rightCentroid.z()) / 2.0;
    }
    else if (rightAvailable) {
        // 우측 벽만 감지된 경우, y 좌표를 좌측으로 조정
        Eigen::Vector4f rightCentroid;
        pcl::compute3DCentroid(*rightTCloud, rightCentroid);
        centerPoint.x = rightCentroid.x();
        centerPoint.y = rightCentroid.y() + 0.25;
        centerPoint.z = rightCentroid.z();
    }
    else if (leftAvailable) {
        // 좌측 벽만 감지된 경우, y 좌표를 우측으로 조정
        Eigen::Vector4f leftCentroid;
        pcl::compute3DCentroid(*leftTCloud, leftCentroid);
        centerPoint.x = leftCentroid.x();
        centerPoint.y = leftCentroid.y() - 0.25;
        centerPoint.z = leftCentroid.z();
    }
    else {
        // 벽이 모두 감지되지 않은 경우, 기본 중심점 설정
        centerPoint.x = 0.4;
        centerPoint.y = 0.0;
        centerPoint.z = 0.0;
    }

    // 결과 클라우드에 중심점을 추가
    centerPointCloud->points.push_back(centerPoint);
    centerPointCloud->width = 1;
    centerPointCloud->height = 1;
    centerPointCloud->is_dense = true;
}

class ImageProjection{
private:
// ros handle
    ros::NodeHandle nh;

// 차량 좌표 및 시간
    geometry_msgs::PoseStamped local;

// ros::subscriber & sync
    ros::Subscriber subLidar;

// ros::publisher
    ros::Publisher pubLaserCloudIn; // prototype pointCloud

    //
    ros::Publisher pubEmergencySign;
    std_msgs::Bool emergency;
    ros::Publisher pubEmergencyCloud;

    //콘주행
    ros::Publisher pubInterestCloud;
    ros::Publisher pubConeClusterCloud;
    ros::Publisher pubLconeCloud;
    ros::Publisher pubRconeCloud;
    ros::Publisher pubConeROICloud;
    ros::Publisher pubMidLineCloud;

    ros::Publisher pubObsSmall;

    ros::Publisher pubLConeLineMarkerArray;
    ros::Publisher pubRConeLineMarkerArray;
    ros::Publisher pubROIMarkerArray;
    ros::Publisher pubMidLineMarkerArray;

    //차량
    ros::Publisher pubStaticCloud;
    ros::Publisher pubStaticMarkerArray;
    ros::Publisher pubStaticBboxArray;

    ros::Publisher pubLeftCarCloud;
    ros::Publisher pubRightCarCloud;
    ros::Publisher pubLeftCarMarkerArray;
    ros::Publisher pubRightCarMarkerArray;


// rosMsg
    std::shared_ptr<sensor_msgs::PointCloud2> rosCloud;

// pcl pointCloud
    // pointCLoud 원본
    pcl::PointCloud<PointType>::Ptr laserCloudIn;

    //Emergency
    pcl::PointCloud<PointType>::Ptr emergencyCloud;

    //콘주행
    pcl::PointCloud<PointType>::Ptr interestCloud_1;
    pcl::PointCloud<PointType>::Ptr interestCloud_2;
    pcl::PointCloud<PointType>::Ptr coneClusterCloud;
    pcl::PointCloud<PointType>::Ptr LconeCloud;
    pcl::PointCloud<PointType>::Ptr RconeCloud;
    pcl::PointCloud<PointType>::Ptr coneROICloud;
    pcl::PointCloud<PointType>::Ptr midLineCloud;

    //차량
    pcl::PointCloud<PointType>::Ptr staticCloud;

    //콘주행
    std::shared_ptr<visualization_msgs::MarkerArray> LConeLineMarkerArray = std::make_shared<visualization_msgs::MarkerArray>();
    std::shared_ptr<visualization_msgs::MarkerArray> RConeLineMarkerArray = std::make_shared<visualization_msgs::MarkerArray>();
    std::shared_ptr<visualization_msgs::MarkerArray> ROIMarkerArray = std::make_shared<visualization_msgs::MarkerArray>();
    std::shared_ptr<visualization_msgs::MarkerArray> MidLineMarkerArray = std::make_shared<visualization_msgs::MarkerArray>();

    //차량
    std::shared_ptr<visualization_msgs::MarkerArray> staticMarkerArray = std::make_shared<visualization_msgs::MarkerArray>();
    std::shared_ptr<visualization_msgs::MarkerArray> staticBboxArray = std::make_shared<visualization_msgs::MarkerArray>();

    std::shared_ptr<std::vector<pcl::PointCloud<PointType>::Ptr>> static_Cloud_vector = std::make_shared<std::vector<pcl::PointCloud<PointType>::Ptr>>();

    pcl::PointCloud<PointType>::Ptr leftCarCloud;
    pcl::PointCloud<PointType>::Ptr rightCarCloud;
    std::shared_ptr<visualization_msgs::MarkerArray> leftCarMarkerArray = std::make_shared<visualization_msgs::MarkerArray>();
    std::shared_ptr<visualization_msgs::MarkerArray> rightCarMarkerArray = std::make_shared<visualization_msgs::MarkerArray>();
    
    static laser_geometry::LaserProjection projector;

    Preprocessor preprocessor;
    PointCloudGenerator pointCloudGenerator;

public:
    // 전처리 클래스 선언
    ImageProjection(): nh("~"){
        // Publish할 토픽 설정
        pubLaserCloudIn = nh.advertise<sensor_msgs::PointCloud2>("/laserCloudIn", 1);

        //Emergency
        pubEmergencySign = nh.advertise<std_msgs::Bool>("/emergency_sign", 1);
        pubEmergencyCloud = nh.advertise<sensor_msgs::PointCloud2>("/emergency_cloud", 1);

        //콘주행
        pubInterestCloud = nh.advertise<sensor_msgs::PointCloud2>("/interest_cloud", 1);
        pubConeClusterCloud = nh.advertise<sensor_msgs::PointCloud2>("/cone_cluster_point",1);
        pubLconeCloud = nh.advertise<sensor_msgs::PointCloud2>("/Lcone_cloud", 1);
        pubRconeCloud = nh.advertise<sensor_msgs::PointCloud2>("/Rcone_cloud", 1);
        pubConeROICloud = nh.advertise<sensor_msgs::PointCloud2>("/cone_ROI_cloud", 1);
        pubMidLineCloud = nh.advertise<sensor_msgs::PointCloud2>("/mid_line_cloud", 1);
        pubMidLineMarkerArray = nh.advertise<visualization_msgs::MarkerArray>("/mid_line_marker_array", 1);

        pubObsSmall = nh.advertise<visualization_msgs::MarkerArray>("/obs_small", 1);
        
        pubLConeLineMarkerArray = nh.advertise<visualization_msgs::MarkerArray>("/Lcone_line_marker_array", 1);
        pubRConeLineMarkerArray = nh.advertise<visualization_msgs::MarkerArray>("/Rcone_line_marker_array", 1);
        pubROIMarkerArray = nh.advertise<visualization_msgs::MarkerArray>("/tunnel_mid_marker_array", 1);

        //차량
        pubStaticCloud = nh.advertise<sensor_msgs::PointCloud2>("/static_cloud", 1);
        pubStaticMarkerArray = nh.advertise<visualization_msgs::MarkerArray>("/static_markers", 1);
        pubStaticBboxArray = nh.advertise<visualization_msgs::MarkerArray>("/static_markers_vis", 1);

        pubLeftCarCloud = nh.advertise<sensor_msgs::PointCloud2>("/left_car_cloud", 1);
        pubRightCarCloud = nh.advertise<sensor_msgs::PointCloud2>("/right_car_cloud", 1);
        pubLeftCarMarkerArray = nh.advertise<visualization_msgs::MarkerArray>("/left_car_marker_array", 1);
        pubRightCarMarkerArray = nh.advertise<visualization_msgs::MarkerArray>("/right_car_marker_array", 1);


        // Subscribe할 토픽 설정
        subLidar = nh.subscribe("/scan", 10, &ImageProjection::cloudHandler, this);

        // callback 정의
        allocateMemory();
        clearMemory();
    }

    ~ImageProjection() {}

    void allocateMemory(){
        laserCloudIn.reset(new pcl::PointCloud<PointType>());

        //Emergency
        emergencyCloud.reset(new pcl::PointCloud<PointType>());

        //콘주행
        coneClusterCloud.reset(new pcl::PointCloud<PointType>());
        coneROICloud.reset(new pcl::PointCloud<PointType>());
        interestCloud_1.reset(new pcl::PointCloud<PointType>());
        interestCloud_2.reset(new pcl::PointCloud<PointType>());
        LconeCloud.reset(new pcl::PointCloud<PointType>());
        RconeCloud.reset(new pcl::PointCloud<PointType>());
        midLineCloud.reset(new pcl::PointCloud<PointType>());

        //차량
        staticCloud.reset(new pcl::PointCloud<PointType>());
        leftCarCloud.reset(new pcl::PointCloud<PointType>());
        rightCarCloud.reset(new pcl::PointCloud<PointType>());


        rosCloud = std::make_shared<sensor_msgs::PointCloud2>();
    }

    void clearMemory(){
        laserCloudIn->clear();
        rosCloud->data.clear();

        //Emergency
        emergencyCloud->clear();
        
        //차량
        staticCloud->clear();
        deleteAllMarkers(staticMarkerArray);
        static_Cloud_vector->clear();

        //콘주행
        coneClusterCloud->clear();
        interestCloud_1->clear();
        interestCloud_2->clear();
        LconeCloud->clear();
        RconeCloud->clear();
        coneROICloud->clear();
        midLineCloud->clear();
        leftCarCloud->clear();
        rightCarCloud->clear();

        deleteAllMarkers(LConeLineMarkerArray);
        deleteAllMarkers(RConeLineMarkerArray);
        deleteAllMarkers(ROIMarkerArray);
        deleteAllMarkers(MidLineMarkerArray);
        deleteAllMarkers(leftCarMarkerArray);
        deleteAllMarkers(rightCarMarkerArray);
    }

    // 라이다와 GPS만 작동시 진행
    void cloudHandler(const sensor_msgs::LaserScan::ConstPtr& rosLaser) // ROS에서 lidar scan msg 받아서 pcl point cloud로 변환
    {
        cout << string(30, '\n');
        cout << "LiDAR is Working" << endl;
        projector.projectLaser(*rosLaser, *rosCloud,-1,laser_geometry::channel_option::Intensity | laser_geometry::channel_option::Distance);
        pcl::fromROSMsg(*rosCloud, *laserCloudIn); //ROS msg(PointCloud2) -> PCL(PointCloud)

        // 포인트 클라우드 가공 및 생성
        getPointCloud();

        // 포인트 클라우드 publish
        publishPointCloud();
        // 데이터 초기화
        clearMemory();
    }

    // 부채꼴 모양 ROI
    void getSectorROI(const pcl::PointCloud<PointType>::Ptr& inputCloud,
                      pcl::PointCloud<PointType>::Ptr& outputCloud,
                      float r_min, float r_max, float angle_limit_rad)
    {
        outputCloud->clear();
        for (const auto& point : inputCloud->points) {
            float distance = std::hypot(point.x, point.y);
            float angle = std::atan2(point.y, point.x);

            if (distance >= r_min && distance <= r_max &&
                std::abs(angle) <= angle_limit_rad)
            {
                outputCloud->points.push_back(point);
            }
        }
        outputCloud->width = outputCloud->points.size();
        outputCloud->height = 1;
        outputCloud->is_dense = true;
    }

    void getPointCloud(){
        // Emergency / 차단기
        pointCloudGenerator.getSectorROICloud(laserCloudIn, emergencyCloud, 0.3f, 5.0f, M_PI / 6);
        if (emergencyCloud->points.size() > 4)  emergency.data = true; //Emergency
        else                                    emergency.data = false; 

        // 트레픽 콘 검출
        pointCloudGenerator.getAngleCloud(laserCloudIn, coneROICloud, {0, 25.0}, {-6.0,6.0}, {-0.1, 0.1}, {-100, 100});
        pointCloudGenerator.getConeClusterCloud(coneROICloud, coneClusterCloud);

        pointCloudGenerator.getBox(coneClusterCloud, LconeCloud, {1.5, 2.5}, 3.0, 5.0);
        pointCloudGenerator.getBox(coneClusterCloud, RconeCloud, {1.5, -2.5}, 3.0, 5.0);
        
        if (!LconeCloud->empty() && !RconeCloud->empty()) {
            // 둘 다 있으면 정상적인 midLine 생성
            pointCloudGenerator.getConeTrackerCloud(LconeCloud, RconeCloud, midLineCloud);
        } else {
            // 둘 다 없으면 고정 offset midLine
            PointType offsetPoint;
            offsetPoint.x = 2.0;
            offsetPoint.y = 0.0;
            offsetPoint.z = 0.0;
            midLineCloud->points.push_back(offsetPoint);
            midLineCloud->width = 1;
            midLineCloud->height = 1;
            midLineCloud->is_dense = true;
        }

        // marker 변환 및 publish
        PC2_to_markerArray(midLineCloud, MidLineMarkerArray);

        PC2_to_markerArray(LconeCloud, LConeLineMarkerArray);
        PC2_to_markerArray(RconeCloud, RConeLineMarkerArray);

        // 차량
        pointCloudGenerator.getInterestCloud(laserCloudIn, staticCloud, {0, 9.0}, {-6, 6}, {-0.1, 0.1}); // 전방 5m, 좌우 3m
        pointCloudGenerator.getCarCloud(staticCloud, static_Cloud_vector); //static_Cloud_vector 생성하기
        
        // 좌/우 차량 클라우드 초기화
        leftCarCloud.reset(new pcl::PointCloud<PointType>());
        rightCarCloud.reset(new pcl::PointCloud<PointType>());

        // 좌/우로 차량 클러스터 구분
        std::vector<std::pair<float, pcl::PointCloud<PointType>::Ptr>> clusters_with_y;
        for (const auto& cluster : *static_Cloud_vector) {
            if (cluster->empty()) continue;

            Eigen::Vector4f centroid;
            pcl::compute3DCentroid(*cluster, centroid);
            clusters_with_y.emplace_back(centroid[1], cluster);
        }

        // y기준으로 오름차순 정렬 (왼쪽 → 오른쪽)
        std::sort(clusters_with_y.begin(), clusters_with_y.end(),
                [](const auto& a, const auto& b) {
                    return a.first < b.first;
                });

        // 정렬된 결과를 반으로 나눠서 왼쪽/오른쪽 구분
        size_t half = clusters_with_y.size() / 2;
        size_t cluster_num = clusters_with_y.size();
        for (size_t i = 0; i < cluster_num; ++i) {
            const auto& cluster = clusters_with_y[i].second;
            float y_val = clusters_with_y[i].first;

            //차량 기준 장애물 차량이 오른쪽에 있으면 흰색, 왼쪽에 있으면 파란색으로 구분
            // [예외 처리] 클러스터가 1개이고 y < 0이면 오른쪽으로 분류
            if (cluster_num == 1 && y_val < 0.0f) {
                *rightCarCloud += *cluster;
                addMarkerToArray(rightCarMarkerArray, cluster, "map", 1.0f, 1.0f, 1.0f); // 흰색
                continue;
            }

            // 기본 분할 로직
            if (i < half) {
                *leftCarCloud += *cluster;
                addMarkerToArray(leftCarMarkerArray, cluster, "map", 1.0f, 1.0f, 1.0f); // 흰색
            } else {
                *rightCarCloud += *cluster;
                addMarkerToArray(rightCarMarkerArray, cluster, "map", 0.0f, 0.0f, 1.0f); // 파란색
            }
        }

        pointCloudGenerator.getObjectMarkers(static_Cloud_vector, staticMarkerArray, staticBboxArray);

    }

    void publishPointCloud(){
        publisher(laserCloudIn, pubLaserCloudIn, "map");

        //Emergency
        pubEmergencySign.publish(emergency);
        publisher(emergencyCloud, pubEmergencyCloud, "map");

        // 콘 주행
        publisher(interestCloud_1, pubInterestCloud, "map");
        publisher(coneClusterCloud, pubConeClusterCloud, "map");
        publisher(LconeCloud, pubLconeCloud, "map");
        publisher(RconeCloud, pubRconeCloud, "map");
        publisher(coneROICloud, pubConeROICloud, "map");
        publisher(midLineCloud, pubMidLineCloud, "map");

        publisherMarkerArray(LConeLineMarkerArray, pubLConeLineMarkerArray, "map");
        publisherMarkerArray(RConeLineMarkerArray, pubRConeLineMarkerArray, "map");
        publisherMarkerArray(ROIMarkerArray, pubROIMarkerArray, "map");
        publisherMarkerArray(MidLineMarkerArray, pubMidLineMarkerArray, "map");

        //차량
        publisher(staticCloud, pubStaticCloud, "map");
        publisherMarkerArray(staticMarkerArray, pubStaticMarkerArray, "map");
        publisherMarkerArray(staticBboxArray, pubStaticBboxArray, "map");
        publisher(leftCarCloud, pubLeftCarCloud, "map");
        publisher(rightCarCloud, pubRightCarCloud, "map");
        publisherMarkerArray(leftCarMarkerArray, pubLeftCarMarkerArray, "map");
        publisherMarkerArray(rightCarMarkerArray, pubRightCarMarkerArray, "map");

        // 작은 장애물 (콘) 마커들 병합
        visualization_msgs::MarkerArray coneMarkers;
        coneMarkers.markers = LConeLineMarkerArray->markers;
        coneMarkers.markers.insert(
            coneMarkers.markers.end(),
            RConeLineMarkerArray->markers.begin(),
            RConeLineMarkerArray->markers.end()
        );
        //ID 중복 방지용 재지정
        int id =0;
        for(auto&marker : coneMarkers.markers){
            marker.id = id++;
            // Marker color 주황색
            marker.color.r = 1.0f;   // 주황색의 빨강
            marker.color.g = 0.5f;   // 주황색의 초록 (약간만)
            marker.color.b = 0.0f;   // 주황색의 파랑 없음
            marker.color.a = 1.0f;   // 불투명도
        }
        // 퍼블리시
        pubObsSmall.publish(coneMarkers);

        int car_id = 0;
        for (auto& marker : leftCarMarkerArray->markers) {
            marker.id = car_id++;
        }
        car_id = 0;
        for (auto& marker : rightCarMarkerArray->markers) {
            marker.id = car_id++;
        }
    }
};

laser_geometry::LaserProjection ImageProjection::projector;

int main (int argc, char** argv)
{
    ros::init(argc, argv, "practics_dynamic_scale");
    ImageProjection imageProjection;
    ros::spin();
}