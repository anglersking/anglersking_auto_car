#include "Track_Detection.h"

// height: 120
// width: 160


Track_Detection::Track_Detection()
{
    ros::NodeHandle nh_private("~");
    nh_private.param<int>("GyraThreshold", this->GyraThreshold, 132); 
    nh_private.param<bool>("StartMove", this->StartMove, false); 
    nh_private.param<double>("MixKP", this->MixKP, 0.025f);
    nh_private.param<double>("MaxKP", this->MaxKP, 0.04f);
    nh_private.param<double>("HiBotSpeed", this->HiBotSpeed, 0.2);

    image_transport::ImageTransport it(n);
    this->Image_pub = it.advertise("camera/image", 1);
    this->RgbImage_pub = it.advertise("camera/rgb", 1);
    this->cmd_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel",1);

    memset(MiddleArray, 0, sizeof(MiddleArray));

    capture.open(0);
    if(!capture.isOpened())
    {
        ROS_INFO("Open video0 is error!");
        ros::shutdown();
    }

    capture.set(CV_CAP_PROP_FRAME_HEIGHT, 240);
    capture.set(CV_CAP_PROP_FRAME_WIDTH, 320);
    capture.set(CV_CAP_PROP_FPS, 30.0);
    if (!capture.set(CV_CAP_PROP_FOURCC, CV_FOURCC('M', 'J', 'P', 'G')))
    {
        ROS_INFO("set format failed \n");
    }

    frame = boost::make_shared< cv_bridge::CvImage >();
    frame->encoding = sensor_msgs::image_encodings::BGR8;
}

Track_Detection::~Track_Detection()
{
    capture.release();
}

void Track_Detection::Open_Imread()
{
    capture >> frame->image; //流的转换   

    if(frame->image.empty())
    {
        ROS_INFO( "Failed to capture frame!" );
        ros::shutdown();
    }

    frame->header.stamp = ros::Time::now();
    this->RgbImage_pub.publish(frame->toImageMsg());

    this->SrcImageRead = frame->image;
}

void Track_Detection::Image_Filter_Process()
{
    Mat GyraImage, GaussianImage, autoImage;

    resize(this->SrcImageRead, this->SrcImageRead, Size(), 0.5, 0.5);
    cvtColor(this->SrcImageRead, GyraImage, COLOR_RGB2GRAY);

    ros::param::get("/Track_Detection_node/GyraThreshold", GyraThreshold);
  
    //adaptiveThreshold(GyraImage, DestImageOut, 255, CV_ADAPTIVE_THRESH_MEAN_C, CV_THRESH_BINARY_INV, 45, 3);  
    threshold(GyraImage, DestImageOut, 0, 255, THRESH_OTSU | THRESH_BINARY);
}

void Track_Detection::Show_Image(String WindowsName)
{
    imshow(WindowsName, DestImageOut);
}

void Track_Detection::ImageMiddleDetection()
{
    unsigned char *PtrRows = NULL;

    memset(MiddleArray, 0, sizeof(MiddleArray));
    //ROS_INFO("%d, %d,",DestImageOut.rows, DestImageOut.cols);

     for (int i = DestImageOut.rows-1; i > 0; i--)
     {
        PtrRows = DestImageOut.ptr<uchar>(i);       //获取每行首地址

        for (int j = MIDDLE_VALUE; j > 1; j--)       // left line found
        { 
            if((PtrRows[j] - PtrRows[j-1]) != 0)    
            {
                this->MiddleArray[i][0] = j;
                break;
            }
            this->MiddleArray[i][0] = 0;
        }

        for (int j = MIDDLE_VALUE; j < DestImageOut.cols; j++)       // left line found
        {
            if((PtrRows[j] - PtrRows[j-1]) != 0)
            {
                this->MiddleArray[i][1] = j;
                break;
            }
            this->MiddleArray[i][1] = DestImageOut.cols;
        }
        this->MiddleArray[i][2] = (this->MiddleArray[i][0] + this->MiddleArray[i][1]) / 2;  //middle value
        DestImageOut.at<uchar>(i, this->MiddleArray[i][2]) = 0;
     }

     this->HiBotMove_Control();

    sensor_msgs::ImagePtr imageMsg = cv_bridge::CvImage(std_msgs::Header(), "mono8", DestImageOut).toImageMsg(); //mono8
    this->Image_pub.publish(imageMsg);
}

void Track_Detection::HiBotMove_Control()       // pid
{
    unsigned char EffectiveValue = 0;
    float MiddleAverageVal = 0;
    float OutPutVth = 0;

    for (int i = 1; i < PROSPECT_VALUE; i++)
    {
        if(fabs(MiddleArray[DestImageOut.rows-i][2] - MiddleArray[DestImageOut.rows-i-1][2]) > 10)  //寻找有效的前瞻值
        {
            EffectiveValue = i;
            break;
        }
        if(i == PROSPECT_VALUE-1)
        {
            EffectiveValue = PROSPECT_VALUE-1;
        }
    }

    float SlopeValue = (this->MiddleArray[DestImageOut.rows-1][2] - this->MiddleArray[DestImageOut.rows - EffectiveValue][2]) / ((EffectiveValue) *1.0f);

    for (int i = 0; i < EffectiveValue; i++)        //中值高斯滤波，去除不可信的中值
    {
        MiddleAverageVal += this->MiddleArray[DestImageOut.rows-i-1][2];
    }
    MiddleAverageVal = MiddleAverageVal/EffectiveValue;

    float Error = MIDDLE_VALUE - MiddleAverageVal;      //P

    ros::param::get("/Track_Detection_node/MixKP", this->MixKP);
    ros::param::get("/Track_Detection_node/MaxKP", this->MaxKP);

    if((EffectiveValue < 20) && (fabs(SlopeValue) > SLOPE_VALUE))   //斜率大于定值或者前瞻小于定值时，使用较大的PID参数
    {
         OutPutVth = Error * this->MaxKP;
        ROS_INFO("Max = %d  %d  %f  %d  %f",    this->MiddleArray[DestImageOut.rows-1][2],\
                                            this->MiddleArray[DestImageOut.rows - EffectiveValue][2]\
                                            , SlopeValue, EffectiveValue, OutPutVth);
    }
    else
    {
        OutPutVth = Error * this->MixKP;
        ROS_INFO("Mix = %d  %d  %f  %d  %f",    this->MiddleArray[DestImageOut.rows-1][2],\
                                            this->MiddleArray[DestImageOut.rows - EffectiveValue][2]\
                                            , SlopeValue, EffectiveValue, OutPutVth);
    }

    ros::param::get("/Track_Detection_node/StartMove", this->StartMove);
    ros::param::get("/Track_Detection_node/HiBotSpeed", this->HiBotSpeed);
    (this->StartMove)?(this->HiBotCmd_Vel(this->HiBotSpeed, OutPutVth)):(this->HiBotCmd_Vel(0.0, 0.0));
    
}

void Track_Detection::HiBotCmd_Vel(float vx, float vz)
{
    geometry_msgs::Twist  twistMsg;

    twistMsg.linear.x = vx;
    twistMsg.angular.z = vz;

    this->cmd_pub.publish(twistMsg);
}


int main(int argc, char** argv)
{
 
    ros::init(argc, argv, "HiBot_track_detection");
    ROS_INFO("[ZHOUXUEWEI] Track_Detection_node start!");

    Track_Detection HiBotTrackDetection;

    ros::Rate loop_rate(20);
    while(ros::ok())
    {
        HiBotTrackDetection.Open_Imread();
        HiBotTrackDetection.Image_Filter_Process();
        HiBotTrackDetection.ImageMiddleDetection();

        ros::spinOnce();
        loop_rate.sleep();
    }

}