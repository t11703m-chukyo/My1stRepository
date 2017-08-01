#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/superres/optical_flow.hpp> 


static const std::string OPENCV_WINDOW = "Image window";

cv::Mat capture, current, previous,flow,visual_flow, opening;

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;

public:
  ImageConverter()
    : it_(nh_)
  {

    image_sub_ = it_.subscribe("/usb_cam/image_raw", 1,
      &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/optical_flow/output_video", 1);

    cv::namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    
    //カメラ画像
    capture = cv_ptr->image;
    
     //グレイスケールへ変換
    cvtColor(capture, current, CV_BGR2GRAY);
   
    //前のフレームがあれば、オプティカルフローを計算し、表示する
    if(!previous.empty()) {
        //オプティカルフロー計算
      cv::calcOpticalFlowFarneback(previous, current, flow, 0.5, 1, 4, 1, 5, 1.1,3);
            
        //オプティカルフローを可視化
      opening = current.clone();
      
      int flow_ch = flow.channels();
      for(int y = 0; y < opening.rows; ++y){
      
          float* psrc = (float*)(flow.data + flow.step * y);        
        
				for(int x = 0; x < opening.cols; ++x){
				
				  float dx = psrc[0];
          float dy = psrc[1];
				  float r = (dx + dy);
						
					if( 4 < r ){ 
					  opening.data[ y * opening.step + x  ] = 0;
					}else{
					  opening.data[ y * opening.step + x  ] = 255;
					}
					psrc += flow_ch;
				}
			}
			
			//クロージング
			cv::morphologyEx(
         opening,
         opening,
         cv::MORPH_CLOSE,        // モルフォロジー演算の種類
         cv::Mat(),
         cv::Point( -1, -1 ),
         3,                     // 収縮と膨張が適用される回数．
         cv::BORDER_CONSTANT,
         cv::morphologyDefaultBorderValue()
         );
			//オープニング
      cv::morphologyEx(
         opening,
         opening,
         cv::MORPH_OPEN,        // モルフォロジー演算の種類
         cv::Mat(),
         cv::Point( -1, -1 ),
         50,                     // 収縮と膨張が適用される回数．
         cv::BORDER_CONSTANT,
         cv::morphologyDefaultBorderValue()
         );
         
      //動く物体を囲むための座標
      int x,y;
      int s_x,s_y,g_x,g_y;
      bool bEnd = false;
      //上
      for(y = 0; y < opening.rows; ++y){      
				for(x = 0; x < opening.cols; ++x){
				   if( opening.data[ y * opening.step + x ] == 0 ){
				     s_y = y;
				     bEnd = true;
				   }
				   if( bEnd ){ break; }
				}
				if( bEnd ){ break; }
			}
			bEnd = false;    
			//右
			for(x = 0; x < opening.cols; ++x){
			  for(y = 0; y < opening.rows; ++y){ 
				   if( opening.data[ y * opening.step + x ] == 0 ){
				     s_x = x;
				     bEnd = true;
				   }
				   if( bEnd ){ break; }
				}
				if( bEnd ){ break; }
			}
			bEnd = false; 
			//下
			for( y = opening.rows-1; -1 < y; --y){      
				for( x = opening.cols-1; -1 < x; --x){
				   if( opening.data[ y * opening.step + x ] == 0 ){
				     g_y = y;
				     bEnd = true;
				   }
				   if( bEnd ){ break; }
				}
				if( bEnd ){ break; }
			}   
			bEnd = false; 
			//左
			for( x = opening.cols-1; -1 < x; --x){
				for( y = opening.rows-1; -1 < y; --y){ 
				   if( opening.data[ y * opening.step + x ] == 0 ){
				     g_x = x;
				     bEnd = true;
				   }
				   if( bEnd ){ break; }
				}
				if( bEnd ){ break; }
			}
      bEnd = false; 
      
      
      //表示画面
      visual_flow = capture.clone();
      
      //動く物体を四角で囲む
			cv::rectangle(visual_flow, cv::Point(s_x,s_y), cv::Point(g_x, g_y), cv::Scalar(0,0,255), 3, 4);

      //表示
      cv::imshow(OPENCV_WINDOW, visual_flow);
      //printf("OK¥n");
      
    }
    //前のフレームを保存
    previous = current.clone();
    
    
    cv::waitKey(3);

    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg()); 


    }
};


int main(int argc, char** argv)
{
  
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
  
}
