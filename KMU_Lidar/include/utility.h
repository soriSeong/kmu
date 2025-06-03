#ifndef UTILITY_H
#define UTILITY_H

#include "include.h"

/**
 ** 목적
    ** 1. 상수 및 구조체 저장
    ** 2. preprocessor, clustering ... 등 해당 카테고리에 해당되지 않는 함수
*/
using namespace std;

// 상수 선언
    // ring 사용 여부
    extern const bool useCloudRing;                         // ring 사용 여부

    // PI 상수
    extern const double PI;                                 // PI
    extern const int INF;                                   // 포인트의 범위를 발산으로 하고 싶을 때 넣을 것

    // 라이다 센서 상수
    extern const int N_SCAN;                                // 라이다의 row 이자 ring
    extern const int Horizon_SCAN;                          // 라이다의 colmun
    extern const float ang_res_x;                           // col -> 각도
    extern const float ang_res_y;                           // row -> 각도
    extern const float ang_bottom;                          // 라이다 시야각 / 2
    extern const int groundScanInd;                         // 라이다 시야각 상하 분기각

    // 마운트 상수
    extern const double LI_TO_GPS_X;                        // 라이다와 GPS 사이 거리
    extern const double LI_TO_GND_Z;                        // 라이다와 지변 사이 거리
    extern const double LI_ROT_Y;                           // y축 회전
    extern const double CAR_TO_GPS_X;                       // GPS와 차량의 중심 사이 거리

    // 차량 크기
    extern const float CAR_SIZE_X;                         // 차량의 x축 길이 / 2 
    extern const float CAR_SIZE_Y;                         // 차량의 y축 길이 / 2 

    // 계산 상수
    extern const double DEG_TO_RAD;                         // 각도 -> 라이안
    extern const double RAD_TO_DEG;                         // 라이안 -> 각도

    // 클러스터링 상수
    extern const double Cone_Tolerance;                     // 콘 사이 거리
    extern const int CONE_min_SIZE;                         // 콘에 찍히는 포인트 최소
    extern const int CONE_MAX_SIZE;                         // 콘에 찍히는 포인트 최대

    // 기본 클러스터링 상수
    extern const double OBJECT_Tolerance;                          // 객체 사이 거리
    extern const int OBJECT_min_SIZE;                              // 객체에 찍히는 포인트 최소
    extern const int OBJECT_MAX_SIZE;                              // 객체에 찍히는 포인트 최대

    // 콘 주행 상수
    extern const double CONE_BETWEEN;                        // 차량과 양측에 위치한 콘 사이 y방향 거리


// 구조체 및 타입 선언
    // 기본 포인트 타입
    typedef pcl::PointXYZI PointType;

    // ring이 포함된 포인트 타입
    struct OusterPointXYZIRT {
        PCL_ADD_POINT4D;
        float intensity;
        uint32_t t;
        uint16_t reflectivity;
        uint8_t ring;
        uint16_t noise;
        uint32_t range;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    } EIGEN_ALIGN16;
    POINT_CLOUD_REGISTER_POINT_STRUCT(OusterPointXYZIRT,
        (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity)
        (uint32_t, t, t) (uint16_t, reflectivity, reflectivity)
        (uint8_t, ring, ring) (uint16_t, noise, noise) (uint32_t, range, range)
    )
    using PointXYZIRT = OusterPointXYZIRT;

    void PC2_to_markerArray(const pcl::PointCloud<PointType>::Ptr& input_pointCloud, const std::shared_ptr<visualization_msgs::MarkerArray>& markerarray);

    void addMarkerToArray(std::shared_ptr<visualization_msgs::MarkerArray>& markerArray,const pcl::PointCloud<PointType>::Ptr& cluster,const std::string& frame_id,float r, float g, float b);

    // 카메라 파라미터 구조체
    struct ParamsLidar {
        double X = 0.0;
        double Y = 0.0;
        double Z = 0.0;
        double YAW = 0.0;
        double PITCH = 0.0;
        double ROLL = 0.0;
    };

    struct CameraParams {
        int WIDTH = 640;
        int HEIGHT = 480;
        double FOV = 78;
        double X = 0.0;
        double Y = 0.0;
        double Z = 0.0;
        double YAW = 0.0;
        double PITCH = 0.0;
        double ROLL = 0.0;
    };


    struct G_CMD {
        std::vector<double> rvec = {0.0 * DEG_TO_RAD, 0 * DEG_TO_RAD, 0 * DEG_TO_RAD};
        std::vector<double> t_mat = {0, 0, 0};
        std::vector<double> focal_length = {636.4573730956954, 667.7077677609984};
        std::vector<int> cam = {640, 480};
    };

// 서브 프로세스
    pid_t subProcess_obj(int camera_idx, G_CMD g_cmd , std::vector<string> camera_pair);
    pid_t subProcess_line(int camera_idx, G_CMD g_cmd , std::vector<string> camera_pair);

// 객체 저장 포인터 클라우드 선언
    extern pcl::PointCloud<PointType>::Ptr clusterStopOver;


// 함수 선언
    // 현 시각 저장
    extern ros::Time rosTime;
    void saveCurrentTime(const sensor_msgs::PointCloud2::ConstPtr& rosCloud);

    // map을 읽고 global_path에 저장
    extern std::vector<std::pair<float, float>> global_path;
    extern int curr_index;

    // global_path 내에 현 위치 검색
    void findCurrIndex();

    // 현 자동차의 GPS 및 해딩을 전역 상수로 저장
    extern double ego_x;
    extern double ego_y;
    extern double ego_heading_deg;  // 각도
    extern double ego_heading_rad;  // 라디안
    void setEgo(const geometry_msgs::PoseStampedConstPtr& poseMsg, geometry_msgs::PoseStamped& local);

    // 터널에서 해딩값을 구해 전역 상수로 저장
    std::tuple<double, int> setHeadingInTunnel(const pcl::PointCloud<PointType>::Ptr& input_pointCloud, const std::pair<double, double> x_threshold, const std::pair<double, double> y_threshold, const std::pair<double, double> z_threshold);

    // 한 포인트의 좌표를 상대에서 절대로 변환
    void transformRelToAbs(PointType& point);

    // 한 포인트의 좌표를 절대에서 상대로 변환
    void transformAbsToRel(PointType& point);

    // markerArray 초기화
    void deleteAllMarkers(const std::shared_ptr<visualization_msgs::MarkerArray>& markerArray);

    // publish 실행
    void publisher(const pcl::PointCloud<PointType>::Ptr& pclCloud, const ros::Publisher& publisher, const std::string frame_id);

    // markerarray용 publish 실행
    void publisherMarkerArray(const std::shared_ptr<visualization_msgs::MarkerArray>& markerArray, const ros::Publisher& publisher, const std::string frame_id);

    // 칼만필터 클래스 선언
    class KalmanFilter {
    public:
        Eigen::Vector2f state; // 상태 벡터 (x, y, z 위치)
        Eigen::Matrix2f errorCovariance; // 오차 공분산 행렬
        Eigen::Matrix2f measurementNoise; // 측정 노이즈 공분산 행렬
        Eigen::Matrix2f processNoise; // 프로세스 노이즈 공분산 행렬
        Eigen::Matrix2f identity;

        KalmanFilter();
        void init(Eigen::Vector2f initialState);
        void predict();
        void update(Eigen::Vector2f measurement);
    };  


    
    std::tuple<double, double, double, double> euler_to_quaternion(const double &theta);

    double endpoint_distance(const PointType &point, const std::pair<double, double> minpoint, const std::pair<double, double> maxpoint);

    std::tuple<double, double> find_position(const pcl::PointCloud<PointType> &cloud, double threshold, double min_y, double max_y, double theta);

    bool compare(std::pair<float, float> a, std::pair<float, float> b);

    double variance_criterion(const cv::Mat& c1, const cv::Mat& c2);

    std::tuple<double, double, double, double> get_best_theta(const pcl::PointCloud<PointType> &cloud);

    visualization_msgs::Marker bbox_3d(const pcl::PointCloud<PointType> &cloud, const double &heading, const double &enu_x,const double &enu_y, const int &Id);

#endif // UTILITY_H