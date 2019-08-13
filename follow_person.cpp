//--------------------------------------------------
//
//
//author: Yutaro ISHIDA
//--------------------------------------------------


#include <hma_common_pkg/common.h>
#include <hma_common_pkg/follow_person.h>
#include "mrcohog.h"
#include "realadaboost.h"
#include "pf.h"
#include "util.h"


//--------------------------------------------------
//Constant
//--------------------------------------------------
const unsigned int IMG_HEIGHT = 240;
const unsigned int IMG_WIDTH = 320;

const double AOI_DEPTH_INIT = 1.0; //注目領域の初期距離[m]
const double AOI_DEPTH_MIN = 0.6; //注目領域の最小距離[m]
const double AOI_DEPTH_MAX = 1.4; //注目領域の最小距離[m]
const double AOI_DEPTH_WIDTH = 0.4; //注目領域の幅[m]


//--------------------------------------------------
//Global
//--------------------------------------------------


//--------------------------------------------------
//Device
//--------------------------------------------------


//--------------------------------------------------
//
//--------------------------------------------------
class FollowPersonDepthMRCoHOGRealAdaBoost{
private:
    //--------------------------------------------------
    //
    //--------------------------------------------------
    ros::NodeHandle nh_;

    MRCoHOG* mrcohog_;
    RealAdaBoost* realadaboost_;
    PF* pf_;
    Util* util_;

    image_transport::ImageTransport img_t_;
    image_transport::Subscriber sub_depth_;
    image_transport::Publisher pub_follow_;

    ros::Publisher pub_float32_multi_array_;

    double aoi_depth_;
    int aoi_width_;

    int *mrcohog_feature_;

    stringstream path_;
    ofstream ofs_;


public:
    //--------------------------------------------------
    //
    //--------------------------------------------------
    FollowPersonDepthMRCoHOGRealAdaBoost():img_t_(nh_){
        mrcohog_ = new MRCoHOG();
        realadaboost_ = new RealAdaBoost();
        pf_ = new PF(IMG_HEIGHT, IMG_WIDTH);
        util_ = new Util();

        pf_->init();

        sub_depth_ = img_t_.subscribe("/camera/depth_registered/image_raw", 1, &FollowPersonDepthMRCoHOGRealAdaBoost::subf_depth, this);
        pub_follow_ = img_t_.advertise("/result/follow_person_depth_mrcohog_realadaboost/img/follow", 1);

        pub_float32_multi_array_ = nh_.advertise<std_msgs::Float32MultiArray>("/result/follow_person_depth_mrcohog_realadaboost/person/xyz", 1);

        aoi_depth_ = AOI_DEPTH_INIT;
        aoi_width_ = 200 + (1.0 - AOI_DEPTH_INIT) * 80;

        mrcohog_feature_ = new int[NUM_MRCOHOG_DIM];

        ROS_INFO("[%s]: Load RealAdaBoost", ros::this_node::getName().c_str());
        path_ << getenv("HOME") << "/athome_cfg_file/" << PKG_NAME << "/realadaboost_param.txt";
        realadaboost_->load(path_.str());
        path_.str("");

        path_ << getenv("HOME") << "/athome_cfg_file/" << PKG_NAME << "/exp/follow_person_depth_mrcohog_realadaboost.csv";
        util_->ls_and_mkdir(path_.str());
        ofs_.open(path_.str().c_str());
        path_.str("");
    }


    //--------------------------------------------------
    //
    //--------------------------------------------------
    ~FollowPersonDepthMRCoHOGRealAdaBoost(){
    }


    //--------------------------------------------------
    //
    //--------------------------------------------------
    void subf_depth(const sensor_msgs::ImageConstPtr& smicp_depth){
        vector<int> potential_person(IMG_WIDTH, 0);
        int potential_person_max;

        cv::Mat mat_depth(IMG_HEIGHT, IMG_WIDTH, CV_8UC1);

        cv::Mat mrcohog_feature(1, NUM_MRCOHOG_DIM, CV_32FC1);

        int particle_ave[4];

        double t = 1 / cv::getTickFrequency();
        long clock_start_1, clock_end_1;


        clock_start_1 = cv::getTickCount();


        //openni2.launchでパブリッシュされる/camera/depth_registered/image_rawは32FC1
        cv_bridge::CvImagePtr cv_img_ptr;
        try{
            cv_img_ptr = cv_bridge::toCvCopy(smicp_depth, sensor_msgs::image_encodings::TYPE_32FC1);
        }
        catch(cv_bridge::Exception& e){
            ROS_ERROR("[%s]: cv_brige ERROR", ros::this_node::getName().c_str());
            return;
        }


        //cv_img_ptr->imageは[m]単位の深度画像
        for(int y = 0; y < IMG_HEIGHT; y++){
            for(int x = 0; x < IMG_WIDTH; x++){
                if(cv_img_ptr->image.at<float>(y * 2, x * 2) > (aoi_depth_ - AOI_DEPTH_WIDTH / 2) && cv_img_ptr->image.at<float>(y * 2, x * 2) < (aoi_depth_ + AOI_DEPTH_WIDTH / 2)){
                    mat_depth.at<unsigned char>(y, x) = 255;
                }
                else{
                    mat_depth.at<unsigned char>(y, x) = 0;
                }
            }
        }

        
        aoi_width_ = 200 + (1.0 - aoi_depth_) * 80;

        for(int x = 0; x < IMG_WIDTH - aoi_width_; x += 8){
            cv::Mat mat_depth_roi = mat_depth(cv::Rect(x, 0, aoi_width_, IMG_HEIGHT));
            cv::Mat mat_depth_roi_resize(64, 32, mat_depth_roi.type());
            cv::resize(mat_depth_roi, mat_depth_roi_resize, mat_depth_roi_resize.size(), cv::INTER_LINEAR);
            
            mrcohog_->get_mrcohog_feature(mat_depth_roi_resize, mrcohog_feature_);

            for(int i = 0; i < NUM_MRCOHOG_DIM; i++){
                mrcohog_feature.at<float>(0, i) = mrcohog_feature_[i];
            }

            if(realadaboost_->predict(mrcohog_feature, 0.0) == 1){
                for(int i = -10; i < aoi_width_ + 10; i++){
                    if((x + i) >= 0 && (x + i) <= IMG_WIDTH - 1){
                        if(potential_person[x + i] != 255){
                            potential_person[x + i]++;
                        }
                    }
                }
            }
        }

        
        potential_person_max = (*max_element(potential_person.begin(), potential_person.end()) + 0.001);
        if(potential_person_max > 0){
            for(int y = 0; y < IMG_HEIGHT; y++){
                for(int x = 0; x < IMG_WIDTH; x++){
                    mat_depth.at<unsigned char>(y, x) = (unsigned char)mat_depth.at<unsigned char>(y, x) * ((double)potential_person[x] / potential_person_max);
                }
            }
        }
        

        pf_->resample();
        pf_->predict();
        pf_->weight(mat_depth);
        pf_->get_particle_ave(particle_ave);


        if(cv_img_ptr->image.at<float>(particle_ave[0] * 2, particle_ave[0] * 2) > (aoi_depth_ - AOI_DEPTH_WIDTH / 2) && 
           cv_img_ptr->image.at<float>(particle_ave[0] * 2, particle_ave[0] * 2) < (aoi_depth_ + AOI_DEPTH_WIDTH / 2)){
            aoi_depth_ = cv_img_ptr->image.at<float>(particle_ave[0] * 2, particle_ave[1] * 2);
            if(aoi_depth_ < AOI_DEPTH_MIN){
                aoi_depth_ = AOI_DEPTH_MIN;
            }
            else if(aoi_depth_ > AOI_DEPTH_MAX){
                aoi_depth_ = AOI_DEPTH_MAX;
            }
            else if(isnan(aoi_depth_)){
                aoi_depth_ = AOI_DEPTH_MIN;
            }
        }

        
        clock_end_1 = cv::getTickCount();
        ofs_ << (double)(clock_end_1 - clock_start_1) * t << endl;

        
        cv::circle(mat_depth, cv::Point(particle_ave[1], particle_ave[0]), 8, cv::Scalar(128), 4, 4);

        cv_bridge::CvImage cv_img;
        std_msgs::Header header;
        cv_img = cv_bridge::CvImage(header, sensor_msgs::image_encodings::MONO8, mat_depth);

        pub_follow_.publish(cv_img.toImageMsg());

        //cout << "x: " << aoi_depth_ << endl;
        //cout << "y: " << ((tan(0.50) * 2 * aoi_depth_) / IMG_WIDTH) * (particle_ave[1] - (IMG_WIDTH / 2)) << endl;
        //cout << "z: " << ((tan(0.39) * 2 * aoi_depth_) / IMG_HEIGHT) * (particle_ave[0] - (IMG_HEIGHT / 2)) << endl;

        std_msgs::Float32MultiArray float32_multi_array;
        float32_multi_array.data.clear();

        float32_multi_array.data.push_back(aoi_depth_);
        float32_multi_array.data.push_back(((tan(0.50) * 2 * aoi_depth_) / IMG_WIDTH) * (particle_ave[1] - (IMG_WIDTH / 2)));
        float32_multi_array.data.push_back(((tan(0.39) * 2 * aoi_depth_) / IMG_HEIGHT) * (particle_ave[0] - (IMG_HEIGHT / 2)));

        pub_float32_multi_array_.publish(float32_multi_array);
    }
};


//--------------------------------------------------
//メイン関数
//--------------------------------------------------
int main(int argc, char** argv){
    ros::init(argc, argv, "follow_person_depth_mrcohog_realadaboost");

    FollowPersonDepthMRCoHOGRealAdaBoost* follow_person_depth_mrcohog_realadaboost;
    follow_person_depth_mrcohog_realadaboost = new FollowPersonDepthMRCoHOGRealAdaBoost();

    ros::Rate main_rate(30);
    while(ros::ok()){
        ros::spinOnce();
        main_rate.sleep();
    }

    return 0;
}
