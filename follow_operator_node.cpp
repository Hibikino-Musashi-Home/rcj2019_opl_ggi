//==================================================
/**
* @file
* @author Yutaro ISHIDA
* @brief 人物追跡ROSノード
*/
//==================================================




#include "hma_common_pkg/common/common_include.h"
#include "hma_common_pkg/common/common_global.h"
#include "hma_common_pkg/common/common_param.h"


#include "hma_lib_pkg/lib/libutil.h"
#include "hma_lib_pkg/lib/libopencv.h"
#include "hma_lib_pkg/lib/libpf.h"


#if GP_ROBOT == _HSR
    #include "hma_hsr_pkg/robot/robot_include.h"
    #include "hma_hsr_pkg/robot/robot_global.h"
    #include "hma_hsr_pkg/robot/robot_param.h"
#endif


#if GP_ROBOT == _EXIA
    #include "hma_exia_pkg/robot/robot_include.h"
    #include "hma_exia_pkg/robot/robot_global.h"
    #include "hma_exia_pkg/robot/robot_param.h"
#endif




//==================================================
/*
* グローバル
*/
//==================================================
const bool GP_DYNAMIC_LOOP_RATE = false;
const double GP_LOOP_RATE = 30.0;

const int GP_IMG_D_W = 640; //距離画像の横幅
const int GP_IMG_D_H = 480; //距離画像の縦幅

const double GP_AOI_CENTER_INIT = 0.9; //注目領域(中心)の初期距離[m]
const double GP_AOI_FRONT_WIDTH = 0.15; //注目領域(手前)までの幅[m]
const double GP_AOI_BACK_WIDTH = 0.15; //注目領域(奥)までの幅[m]
const double GP_AOI_CENTER_LIM_MAX = 3.0; //注目領域(中心)の最大距離[m]
const double GP_AOI_CENTER_LIM_MIN = 0.6; //注目領域(中心)の最小距離[m]

const int GP_TH_TOP = 0; //深度画像の追跡対象外範囲（上）
const int GP_TH_BOTTOM = 100; //深度画像の追跡対象外範囲（下）




//==================================================
/**
* @class FollowOperator
* @brief 人物追跡クラス
*/
//==================================================
class FollowOperator{
private:
    //==================================================
    /*
    * メンバ変数
    */
    //==================================================
    ros::NodeHandle nh_;

    ros::Time init_ros_time_;
    ros::Time update_ros_time_;

    LibUtil* libutil_;
    LibOpenCV* libopencv_;
    LibPF* libpf_;

    map<string, string> descriptor_;

    double aoi_front_;
    double aoi_back_;

    int cnt_lost_;

    sensor_msgs::ImageConstPtr smi_ptr_rgbd_depth_;
    ros::Time smi_ptr_rgbd_depth_update_ros_time_;
 

    //==================================================
    /*
    * ROSインタフェース
    */
    //==================================================
    ros::Publisher pub_point_;

    image_transport::ImageTransport img_t_;
    image_transport::Publisher img_t_pub_smi_dbg_;
    image_transport::Subscriber img_t_sub_smi_ptr_rgbd_depth_;




public:
    //==================================================
    /**
    * @fn コンストラクタ
    * @brief
    * @param
    * @return
    */
    //==================================================
    FollowOperator(
        ros::NodeHandle nh
    ):img_t_(nh_){
        //==================================================
        /*
        * メンバ変数
        */
        //==================================================
        nh_ = nh;

        init_ros_time_ = ros::Time::now();
        update_ros_time_ = init_ros_time_;

        libutil_ = new LibUtil(nh_);
        libopencv_ = new LibOpenCV(nh_);

        int sv_upper[4] = {GP_IMG_D_H, GP_IMG_D_W, 10, 10};
        int sv_lower[4] = {0, 0, -10, -10};
        int noise_lim[4] = {30, 30, 10, 10};
        libpf_ = new LibPF(nh_, sv_upper, sv_lower, noise_lim);

        descriptor_ = libutil_->getDescriptor();

        cnt_lost_ = 0;

        smi_ptr_rgbd_depth_update_ros_time_ = init_ros_time_;


        //==================================================
        /*
        * ROSインタフェース
        */
        //==================================================
        pub_point_ = nh_.advertise<geometry_msgs::Point>(
            "/follow_target_node/point",
            1
        );

        img_t_pub_smi_dbg_ = img_t_.advertise(
            ros::this_node::getName() + "/dbg",
            1
        );

        img_t_sub_smi_ptr_rgbd_depth_ = img_t_.subscribe(
            descriptor_["TOPIC_RGBD_DEPTH"],
            1,
            &FollowOperator::subfSMIPtrRGBDDepth,
            this
        );

        //==================================================
        /*
        * イニシャライズ
        */
        //==================================================
        libpf_->init();


        ros::Duration(0.1).sleep();


        return;
    }




    //==================================================
    /**
    * @fn デストラクタ
    * @brief
    * @param
    * @return
    */
    //==================================================
    ~FollowOperator(
        void
    ){
        //==================================================
        /*
        * ファイナライズ
        */
        //==================================================


        return;
    }





    //==================================================
    /**
    * @fn RGB-DカメラのDepth画像サブスクライブ関数
    * @brief
    * @param
    * @return
    */
    //==================================================
    void subfSMIPtrRGBDDepth(
        const sensor_msgs::ImageConstPtr& subt_smi_ptr_rgbd_depth
    ){
        smi_ptr_rgbd_depth_ = subt_smi_ptr_rgbd_depth;
        smi_ptr_rgbd_depth_update_ros_time_ = ros::Time::now();


        return;
    }




    //==================================================
    /**
    * @fn クラスメイン関数
    * @brief
    * @param
    * @return
    */
    //==================================================
    void main(
        void
    ){
        //==================================================
        /*
        * サブスクライブ開始の検証
        */
        //==================================================
        if(smi_ptr_rgbd_depth_update_ros_time_ <= init_ros_time_){
            return;
        }


        unsigned char depth_8UC1[GP_IMG_D_W * GP_IMG_D_H];
        int aoi_area = 0;
        int pf_ave[4];
        double coe = G_MM2M; //TODO 自動スケールチェック


        cv_bridge::CvImagePtr cv_img_ptr;
        
        cv_img_ptr = cv_bridge::toCvCopy(smi_ptr_rgbd_depth_, sensor_msgs::image_encodings::TYPE_16UC1); //TODO 型チェック
        cv::Mat cv_mat_depth(GP_IMG_D_H, GP_IMG_D_W, cv_img_ptr->image.type());
        cv::resize(cv_img_ptr->image, cv_mat_depth, cv_mat_depth.size(), cv::INTER_LINEAR);

        cv::Mat cv_mat_dbg = cv_mat_depth.clone();


        for(int y = 0; y < cv_mat_depth.rows; y++){
            for(int x = 0; x < cv_mat_depth.cols; x++){
                //画像上側のパススルー
                if(y < GP_TH_TOP){
                    depth_8UC1[cv_mat_depth.cols * y + x] = 0;
                }
                //画像下側のパススルー
                if(y > cv_mat_depth.rows - GP_TH_BOTTOM){
                    depth_8UC1[cv_mat_depth.cols * y + x] = 0;
                }
                else{
                    if(cv_mat_depth.at<unsigned short>(y, x) * coe > aoi_front_ && cv_mat_depth.at<unsigned short>(y, x) * coe < aoi_back_){
                        depth_8UC1[cv_mat_depth.cols * y + x] = 255;
                        cv_mat_dbg.at<unsigned short>(y, x) = 65535;
                        aoi_area++;
                    }
                    else{
                        depth_8UC1[cv_mat_depth.cols * y + x] = 0;
                        cv_mat_dbg.at<unsigned short>(y, x) = 0;
                    }
                }
            }
        }


        libpf_->resample();
        libpf_->predict();
        libpf_->weight(depth_8UC1);
        libpf_->getParticleAve(pf_ave);


        cv::circle(cv_mat_dbg, cv::Point(pf_ave[1], pf_ave[0]), 10, cv::Scalar(30000), 8, 8);
        for(int i = 0; i < sizeof(libpf_->particle_) / sizeof(libpf_->particle_[0]); i++){
            cv::circle(cv_mat_dbg, cv::Point(libpf_->particle_[i].sv[1], libpf_->particle_[i].sv[0]), 1, cv::Scalar(30000), -1, 8);
        }


        cv_img_ptr->image = cv_mat_dbg;
        img_t_pub_smi_dbg_.publish(cv_img_ptr->toImageMsg());


        if(
            cv_mat_depth.at<unsigned short>(pf_ave[0], pf_ave[1]) * coe > GP_AOI_CENTER_LIM_MIN &&
            cv_mat_depth.at<unsigned short>(pf_ave[0], pf_ave[1]) * coe < GP_AOI_CENTER_LIM_MAX
        ){
            aoi_front_ = cv_mat_depth.at<unsigned short>(pf_ave[0], pf_ave[1]) * coe - GP_AOI_FRONT_WIDTH;
            aoi_back_ = cv_mat_depth.at<unsigned short>(pf_ave[0], pf_ave[1]) * coe + GP_AOI_BACK_WIDTH;
        }


        //追跡中
        if(aoi_area > 1000){
            cnt_lost_ = 0;

            //TODO descriptorの使用
            double xyz[3];
            libopencv_->getXYZFromXYD(
                pf_ave[1],
                pf_ave[0],
                cv_mat_depth.at<unsigned short>(pf_ave[0], pf_ave[1]) * coe,
                cv_mat_depth.cols,
                cv_mat_depth.rows,
                58.0 * G_DEG2RAD,
                55.0 * G_DEG2RAD,
                xyz
            );

            geometry_msgs::Point pubt_point;
            pubt_point.x = xyz[0];
            pubt_point.y = xyz[1];
            pubt_point.z = xyz[2];
            pub_point_.publish(pubt_point);
        }
        //ロスト中
        else{
            if(cnt_lost_ == 30){
                libpf_->init();
                aoi_front_ = GP_AOI_CENTER_INIT - GP_AOI_FRONT_WIDTH;
                aoi_back_ = GP_AOI_CENTER_INIT + GP_AOI_BACK_WIDTH;
            }
            else{
                cnt_lost_++;
            }
        }


        return;
    }
};




//==================================================
/*
* メイン関数
*/
//==================================================
int main(
    int argc, char** argv
){
    ros::init(argc, argv, "follow_operator_node");
    ros::NodeHandle nh;

    bool p_dynamic_loop_rate;
    double p_loop_rate;
    double buf_p_loop_rate;

    nh.param<bool>(ros::this_node::getName() + "/dynamic_loop_rate", p_dynamic_loop_rate, GP_DYNAMIC_LOOP_RATE);
    nh.param<double>(ros::this_node::getName() + "/loop_rate", p_loop_rate, GP_LOOP_RATE);
    
    buf_p_loop_rate = p_loop_rate;

    FollowOperator* follow_operator;

    try{    
        follow_operator = new FollowOperator(nh);

        ros::Rate loop_wait = ros::Rate(p_loop_rate);
        while(ros::ok()){
            follow_operator->main();

            if(p_dynamic_loop_rate == true){
                nh.param<double>(ros::this_node::getName() + "/loop_rate", p_loop_rate, GP_LOOP_RATE);
                if(buf_p_loop_rate != p_loop_rate){
                    buf_p_loop_rate = p_loop_rate;
                    loop_wait = ros::Rate(p_loop_rate);
                }
            }

            ros::spinOnce();
            loop_wait.sleep();
        }
    }
    catch(HMAExErr& ex){
        ROS_ERROR("[%s]: FAILURE", ros::this_node::getName().c_str());
    }

    delete follow_operator;


    return 0;
}
