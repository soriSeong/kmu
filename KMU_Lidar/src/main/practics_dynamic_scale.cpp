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
 ** 좌표
        * x: 차량의 정면
        * y: 차량의 좌측
        * z: 차량의 상측
		* 전방시야: x축으로부터 좌측이 음수, 우측이 양수 
        * 회전: 축을 기준으로 오른손의 법칙을 따름
        * 라이다의 row: 아래서부터 위로 숫자가 증가
        * 라이다의 column: 라이다 후면부터 반시계 방향으로 증가
 */
//

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

	//동적장애물
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

	ros::Publisher pubLConeLineMarkerArray;
	ros::Publisher pubRConeLineMarkerArray;
	ros::Publisher pubROIMarkerArray;
	ros::Publisher pubMidLineMarkerArray;

	//정적장애물
	ros::Publisher pubStaticCloud;
	ros::Publisher pubStaticMarkerArray;
	ros::Publisher pubStaticBboxArray;

	//터널
	// ros::Publisher pubLeftTCloud;
	// ros::Publisher pubRightTCloud;
	// ros::Publisher pubCenterPoint;

	//교차로
	// ros::Publisher pubCrossCloud;
	// ros::Publisher pubAllowSign;
	// ros::Publisher pubDirecSign;
	// ros::Publisher pubLen;
	// std_msgs::Bool allow;
	// float loc;
	// std_msgs::Bool direc;
	// std_msgs::Float32 len;
	// ros::Publisher pubLenCloud;

// rosMsg
	std::shared_ptr<sensor_msgs::PointCloud2> rosCloud;

// pcl pointCloud
	// pointCLoud 원본
	pcl::PointCloud<PointType>::Ptr laserCloudIn;

    //동적장애물
	pcl::PointCloud<PointType>::Ptr emergencyCloud;

	//콘주행
	pcl::PointCloud<PointType>::Ptr interestCloud_1;
	pcl::PointCloud<PointType>::Ptr interestCloud_2;
	pcl::PointCloud<PointType>::Ptr coneClusterCloud;
	pcl::PointCloud<PointType>::Ptr LconeCloud;
	pcl::PointCloud<PointType>::Ptr RconeCloud;
	pcl::PointCloud<PointType>::Ptr coneROICloud;
	pcl::PointCloud<PointType>::Ptr midLineCloud;

	//정적장애물
	pcl::PointCloud<PointType>::Ptr staticCloud;

	//터널
	// pcl::PointCloud<PointType>::Ptr leftTCloud;
	// pcl::PointCloud<PointType>::Ptr rightTCloud;
	// pcl::PointCloud<PointType>::Ptr centerPointCloud;

	//교차로
	// pcl::PointCloud<PointType>::Ptr crossCloud;
	// pcl::PointCloud<PointType>::Ptr lenCloud;

	//콘주행
	std::shared_ptr<visualization_msgs::MarkerArray> LConeLineMarkerArray = std::make_shared<visualization_msgs::MarkerArray>();
	std::shared_ptr<visualization_msgs::MarkerArray> RConeLineMarkerArray = std::make_shared<visualization_msgs::MarkerArray>();
	std::shared_ptr<visualization_msgs::MarkerArray> ROIMarkerArray = std::make_shared<visualization_msgs::MarkerArray>();
	std::shared_ptr<visualization_msgs::MarkerArray> MidLineMarkerArray = std::make_shared<visualization_msgs::MarkerArray>();

	//정적장애물
	std::shared_ptr<visualization_msgs::MarkerArray> staticMarkerArray = std::make_shared<visualization_msgs::MarkerArray>();
	std::shared_ptr<visualization_msgs::MarkerArray> staticBboxArray = std::make_shared<visualization_msgs::MarkerArray>();

	std::shared_ptr<std::vector<pcl::PointCloud<PointType>::Ptr>> static_Cloud_vector = std::make_shared<std::vector<pcl::PointCloud<PointType>::Ptr>>();



	static laser_geometry::LaserProjection projector;

	Preprocessor preprocessor;
	PointCloudGenerator pointCloudGenerator;

	
public:
	// 전처리 클래스 선언
	ImageProjection(): nh("~"){
		// Publish할 토픽 설정
		pubLaserCloudIn = nh.advertise<sensor_msgs::PointCloud2>("/laserCloudIn", 1);

		//동적장애물
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
		
		pubLConeLineMarkerArray = nh.advertise<visualization_msgs::MarkerArray>("/Lcone_line_marker_array", 1);
		pubRConeLineMarkerArray = nh.advertise<visualization_msgs::MarkerArray>("/Rcone_line_marker_array", 1);
		pubROIMarkerArray = nh.advertise<visualization_msgs::MarkerArray>("/tunnel_mid_marker_array", 1);

		//정적장애물
		pubStaticCloud = nh.advertise<sensor_msgs::PointCloud2>("/static_cloud", 1);
		pubStaticMarkerArray = nh.advertise<visualization_msgs::MarkerArray>("/static_markers", 1);
		pubStaticBboxArray = nh.advertise<visualization_msgs::MarkerArray>("/static_markers_vis", 1);

		//터널
		// pubLeftTCloud = nh.advertise<sensor_msgs::PointCloud2>("/left_T_cloud", 1);
		// pubRightTCloud = nh.advertise<sensor_msgs::PointCloud2>("/right_T_cloud", 1);
		
		// pubCenterPoint = nh.advertise<sensor_msgs::PointCloud2>("/center_point_cloud", 1);

		//��차로
		// pubCrossCloud = nh.advertise<sensor_msgs::PointCloud2>("/cross_cloud", 1);
		// pubAllowSign = nh.advertise<std_msgs::Bool>("/allow_sign", 1);
		// allow.data = false;
		// pubDirecSign = nh.advertise<std_msgs::Bool>("/direc_sign", 1);
		// pubLen = nh.advertise<std_msgs::Float32>("/len_value", 10);
		// pubLenCloud = nh.advertise<sensor_msgs::PointCloud2>("/len_cloud", 1);

		// Subscribe할 토픽 설정
        subLidar = nh.subscribe("/scan", 10, &ImageProjection::cloudHandler, this);


		// callback 정의

		allocateMemory();
		clearMemory();
	}

	~ImageProjection() {}


    void allocateMemory(){
		laserCloudIn.reset(new pcl::PointCloud<PointType>());

		//동적장애물
		emergencyCloud.reset(new pcl::PointCloud<PointType>());

        //콘주행
		coneClusterCloud.reset(new pcl::PointCloud<PointType>());
		coneROICloud.reset(new pcl::PointCloud<PointType>());
		interestCloud_1.reset(new pcl::PointCloud<PointType>());
		interestCloud_2.reset(new pcl::PointCloud<PointType>());
		LconeCloud.reset(new pcl::PointCloud<PointType>());
		RconeCloud.reset(new pcl::PointCloud<PointType>());
		midLineCloud.reset(new pcl::PointCloud<PointType>());

        //정적장애물
		staticCloud.reset(new pcl::PointCloud<PointType>());

		//터널
		// leftTCloud.reset(new pcl::PointCloud<PointType>());
		// rightTCloud.reset(new pcl::PointCloud<PointType>());
		// centerPointCloud.reset(new pcl::PointCloud<PointType>());

		//교차로
		// crossCloud.reset(new pcl::PointCloud<PointType>());
		// lenCloud.reset(new pcl::PointCloud<PointType>());

		 rosCloud = std::make_shared<sensor_msgs::PointCloud2>();
	}


	void clearMemory(){
		laserCloudIn->clear();
		rosCloud->data.clear();

		//동적장애물
		emergencyCloud->clear();
		

		//정적장애물
		staticCloud->clear();
		deleteAllMarkers(staticMarkerArray);
		static_Cloud_vector->clear();

        //터널
		// leftTCloud->clear();
		// rightTCloud->clear();
		// centerPointCloud->clear();

		//콘주행
		coneClusterCloud->clear();
		interestCloud_1->clear();
		interestCloud_2->clear();
		LconeCloud->clear();
		RconeCloud->clear();
		coneROICloud->clear();
		midLineCloud->clear();

		//교차로
		// crossCloud->clear();
		// lenCloud->clear();

		deleteAllMarkers(LConeLineMarkerArray);
		deleteAllMarkers(RConeLineMarkerArray);
		deleteAllMarkers(ROIMarkerArray);
		deleteAllMarkers(MidLineMarkerArray);

		

	}


// 라이다와 GPS만 작동시 진행
	void cloudHandler(const sensor_msgs::LaserScan::ConstPtr& rosLaser) // ROS에서 lidar scan msg 받아서 pcl point cloud로 변환
	{
		cout << string(30, '\n');
		cout << "LiDAR is Working" << endl;
      	projector.projectLaser(*rosLaser, *rosCloud,-1,laser_geometry::channel_option::Intensity | laser_geometry::channel_option::Distance);
		pcl::fromROSMsg(*rosCloud, *laserCloudIn); //ROS msg(PointCloud2) -> PCL(PointCloud)
		// Eigen::Affine3f transform = Eigen::Affine3f::Identity();
		// transform.rotate(Eigen::AngleAxisf(M_PI, Eigen::Vector3f::UnitZ()));
		// pcl::transformPointCloud(*laserCloudIn, *laserCloudIn, transform);

		//saveCurrentTime(rosCloud);
	// 포인트 클라우드 가공 및 생성
        getPointCloud();

		//cout << "Center Point: (" << centerPoint.x << ", " << centerPoint.y << ", " << centerPoint.z << ")" << endl;
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
	// 동적 장애물 / 차단기
		pointCloudGenerator.getSectorROICloud(laserCloudIn, emergencyCloud, 0.3f, 3.0f, M_PI / 6);
		if (emergencyCloud->points.size() > 4)  emergency.data = true; //Emergency
		else 									emergency.data = false; 

	// 트레픽 콘 검출
	    pointCloudGenerator.getAngleCloud(laserCloudIn, coneROICloud, {0, 25.0}, {-6.0,6.0}, {-0.1, 0.1}, {-100, 100});
		pointCloudGenerator.getConeClusterCloud(coneROICloud, coneClusterCloud);

		pointCloudGenerator.getBox(coneClusterCloud, LconeCloud, {1.5, 2.5}, 3.0, 5.0);
		pointCloudGenerator.getBox(coneClusterCloud, RconeCloud, {1.5, -2.5}, 3.0, 5.0);
		//pointCloudGenerator.getLRconeCloud(coneClusterCloud, RconeCloud, LconeCloud, nullptr, nullptr, nullptr);


	    pointCloudGenerator.getConeTrackerCloud(LconeCloud, RconeCloud, midLineCloud);
		PC2_to_markerArray(midLineCloud, MidLineMarkerArray);
		

	// 정적 장애물
		pointCloudGenerator.getInterestCloud(laserCloudIn, staticCloud, {0, 1.5}, {-1, 1}, {-0.1, 0.1});
		pointCloudGenerator.getCarCloud(staticCloud, static_Cloud_vector); //static_Cloud_vector 생성하기
	    pointCloudGenerator.getObjectMarkers(static_Cloud_vector, staticMarkerArray, staticBboxArray);

	// 터널
	    // pointCloudGenerator.getInterestCloud(laserCloudIn, leftTCloud, {0.3, 0.4}, {0, 0.7}, {-0.1, 0.1});
		// pointCloudGenerator.getInterestCloud(laserCloudIn, rightTCloud, {0.3, 0.4}, {-0.7, 0}, {-0.1, 0.1});
		// calculateTunnelCenterPointCloud(leftTCloud, rightTCloud, centerPointCloud);
		// PC2_to_markerArray(centerPointCloud, ROIMarkerArray);

	// 회전 교차로
		// pointCloudGenerator.getInterestCloud(laserCloudIn, crossCloud, {0.2, 0.8}, {-0.2, 0.2}, {-0.1, 0.1});
		// if (crossCloud->points.size() > 0) { //검출될 시
		// 	loc = crossCloud->points[0].y;
		// 	allow.data = false;
		// }
		// else { //검출 안될 시
		// 	allow.data = true;
		// 	if (loc < 0) {
		// 		direc.data = true; // Right
		// 	}
		// 	else {
		// 		direc.data = false; // Left
		// 	}
		// }

		// cout << ((allow.data) ? "Pass now!" : "Waiting") << endl;
		// cout << ((direc.data) ? " - Right" : " - Left") << endl;
		
		// pointCloudGenerator.getAngleCloud(laserCloudIn, lenCloud, {0.2, 1}, {-0.5, 0.5}, {-0.1, 0.1}, {-60, 60});
		// if (lenCloud->points.size() > 0) {
		// 	PointType P; P.x=0; P.y=0; P.z=0;
		// 	for (PointType point:lenCloud->points){
		// 		P.x += point.x;
		// 		P.y += point.y;
		// 	}
		// 	P.x /= lenCloud->points.size();
		// 	P.y /= lenCloud->points.size();
		// 	len.data = std::sqrt(std::pow(P.x, 2) + std::pow(P.y, 2));
		// }
		// else {
		// 	len.data = -1;
		// }
		
	}


	void publishPointCloud(){
		publisher(laserCloudIn, pubLaserCloudIn, "map");

		//동적장애물
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

        //정적장애물
		publisher(staticCloud, pubStaticCloud, "map");
		publisherMarkerArray(staticMarkerArray, pubStaticMarkerArray, "map");
		publisherMarkerArray(staticBboxArray, pubStaticBboxArray, "map");

		//터널
		// publisher(leftTCloud, pubLeftTCloud, "map");
		// publisher(rightTCloud, pubRightTCloud, "map");
		// publisher(centerPointCloud, pubCenterPoint, "map");

		// 회전 교차로
		// publisher(crossCloud, pubCrossCloud, "map");
		// pubAllowSign.publish(allow);
		// pubDirecSign.publish(direc);
		// pubLen.publish(len);
		// publisher(lenCloud, pubLenCloud, "map");
	}
};

laser_geometry::LaserProjection ImageProjection::projector;

int main (int argc, char** argv)
{
	ros::init(argc, argv, "practics_dynamic_scale");
	ImageProjection imageProjection;
	ros::spin();
}