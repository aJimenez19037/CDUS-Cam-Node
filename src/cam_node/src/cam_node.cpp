// Cleaned Up Camera Node Version
//
// This Camera Node will serve as the recepter of actual sensor data from
// the L515. It will then process the data from RealSense to OpenCV.
// The CV_Bridge will then be used to convert the cv::Mat type to a ROS 
// message. This node will then publish the ROS image message on a topic
// [A Receiver Node will receive this data by 
// subscribing to the aforementioned topic and switching it back over to 
// OpenCV by using cv_bridge again].

// Include ROS, ROS CV_Bridge package, and message types
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/MultiArrayDimension.h>

// Include Eigen Library for Matrix Manipulations
#include <eigen3/Eigen/Dense>

// Include all necessary OpenCV header files
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/objdetect.hpp>
#include <opencv2/core/eigen.hpp>

// Include RealSense Library
#include <librealsense2/rs.hpp>
#include <librealsense2/hpp/rs_processing.hpp>

// Other
#include <cmath>
#include <typeinfo>
#include <stdbool.h>

double median_finder(Eigen::VectorXd& vec) {
	
	double median_val;

	// Sort the vector in ascending order
	std::sort(vec.begin(), vec.end());

	// Determine the length and half length of the vector
	double len = vec.size();
	double half_len = len/2;
	
	// Calculate the mean according to whether there are an odd or
	// even number of values in the vector
	if ( int(len) % 2 == 0 ) {
		median_val = ( vec(int(half_len) -1) + vec(int(half_len)) ) / 2;
	}
	else {
		median_val = vec( int(round(half_len)-1) );
	}
	return median_val;
}

double iqr_finder(Eigen::VectorXd& vec) {

	double iqr_val;

	// Sort the vector in ascending order
	std::sort(vec.begin(), vec.end());

	// Determine the length and half length of the vector
	double len = vec.size();
	double half_len = len/2;

	// Calculate the IQR according to whether there are an even or 
	// odd number of values in the vector
	if ( int(len) % 2 ==0 ) {
		Eigen::VectorXd head = vec.head( int(half_len));
		Eigen::VectorXd tail = vec.tail( int(half_len));
		double q1 = median_finder( head );
		double q3 = median_finder( tail );
		iqr_val = q3 - q1;
	}
	else {
		// Calc the first version of the IQR
		int grab1 = int(round(half_len)-1);
		Eigen::VectorXd head1 = vec.head( grab1 );
		Eigen::VectorXd tail1 = vec.tail( grab1 );
		double q1 = median_finder( head1 );
		double q3 = median_finder( tail1 );
		double iqr1 = q3 - q1;
		// Calc the second version of the IQR
		int grab2 = int(round(half_len));
		Eigen::VectorXd head2 = vec.head( grab2 );
		Eigen::VectorXd tail2 = vec.tail( grab2 );
		q1 = median_finder( head2 );
		q3 = median_finder( tail2 );
		double iqr2 = q3 - q1;
		// Average the two for the official IQR
		iqr_val = (iqr1 + iqr2) /2;
	}
	return iqr_val;
}

int main(int argc, char **argv) try {

    // ----------------------------------------------------------------------------- //
	// ----------------------------------------------------------------------------- //
    // Set up files and parameters for the NN
    // ----------------------------------------------------------------------------- //
	// ----------------------------------------------------------------------------- //
    std::vector<std::string> class_names;
    std::string line;
    // The relative path is FROM the " ~/.ros " directory !!!!
    std::ifstream ifs(std::string("../catkin_ws/src/rs2opencv/src/NNet/object_detection_classes_coco.txt").c_str());
    while (getline(ifs, line))
    {
        class_names.push_back(line);
    }  
    // load the neural network model
    cv::dnn::Net nnet;
    nnet = cv::dnn::readNet("/home/nvidia/catkin_ws/src/cam_node/src/NNet/frozen_inference_graph.pb", 
                        "../catkin_ws/src/rs2opencv/src/NNet/ssd_mobilenet_v2_coco_2018_03_29.pbtxt.txt", 
                        "TensorFlow");

    // ----------------------------------------------------------------------------- //
	// ----------------------------------------------------------------------------- //
    // ----------------------------------------------------------------------------- //
	// ----------------------------------------------------------------------------- //




	// ----------------------------------------------------------------------------- //
	// ----------------------------------------------------------------------------- //
	// ROS Publisher Node Initialization work
	// ----------------------------------------------------------------------------- //
	// ----------------------------------------------------------------------------- //
	ros::init(argc, argv, "camera_node2");
	ros::NodeHandle nh_;
	ros::Publisher rgb_pub_ = nh_.advertise<sensor_msgs::Image>("RGB_data", 10);
	ros::Publisher depth_pub_ = nh_.advertise<sensor_msgs::Image>("depth_data", 10);
	ros::Publisher target_pub_ = nh_.advertise<std_msgs::Bool>("target_flag", 10);
    ros::Publisher obstacle_pub_ = nh_.advertise<std_msgs::Bool>("obstacle_flag", 10);
	ros::Publisher tar_corners_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("tar_corners_data", 10);
	ros::Publisher obs_corners_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("obs_corners_data", 10);
	ros::Publisher pose_flag_pub_ = nh_.advertise<std_msgs::Bool>("pose_flag", 10);
	bool target_flag = false;
    bool obstacle_flag = false;
    std_msgs::Bool target_flag_ros;
    std_msgs::Bool obstacle_flag_ros;
	std_msgs::Float32MultiArray tar_corn_msg;
	std_msgs::Float32MultiArray obs_corn_msg;
	// bool pose_flag = false;
	std_msgs::Bool pose_flag_ros;
	// ros::Rate loop_rate(0.25);
	ros::Rate loop_rate(20);

	// Initialize Eigen Matrices
	Eigen::MatrixXd dMap;
	Eigen::MatrixXd xMat;
	Eigen::MatrixXd yMat;
	Eigen::MatrixXd zMat;
	// RealSense2 Pipeline AND Pointcloud Initialization
	// rs2::colorizer color_map;
	rs2::pipeline pipe;
	rs2::pointcloud ptcloud;
	// Set config param to be able to stream (and ultimately align)
	rs2::config cfg;
	cfg.enable_stream(RS2_STREAM_DEPTH);
	cfg.enable_stream(RS2_STREAM_COLOR);
	auto profile = pipe.start(cfg);

	// Generate the aligned frameset
	rs2::align align_to_color(RS2_STREAM_COLOR);

	// ----------------------------------------------------------------------------- //
	// ----------------------------------------------------------------------------- //
	// Begin LOOPING for ROS and waiting for RealSense frames from L515
	// ----------------------------------------------------------------------------- //
	// ----------------------------------------------------------------------------- //
    

	while(ros::ok()) {
		
        // Get starting timepoint
        auto start = std::chrono::high_resolution_clock::now();

		// ----------------------------------------------------------------------------- //
		// ----------------------------------------------------------------------------- //
		// ----------------------------------------------------------------------------- //
		// ----------------------------------------------------------------------------- //
		// Get current RealSense frame
		ROS_INFO("____START____");
		rs2::frameset fs = pipe.wait_for_frames();

		// Publish pose flag!!!!
		pose_flag_ros.data = true;
		pose_flag_pub_.publish(pose_flag_ros);
		pose_flag_ros.data = false;
		pose_flag_pub_.publish(pose_flag_ros);

		// Process current frame to generate aligned frameset
		rs2::frameset fs_aligned = align_to_color.process(fs);

		// Pull depth data from aligned frameset
		// Pull frame properties in order to convert to OpenCV depth Map matrix
		rs2::depth_frame depth = fs_aligned.get_depth_frame();
		// const double depthScale = 
		const int w1 = depth.as<rs2::video_frame>().get_width();	// number of columns
		const int h1 = depth.as<rs2::video_frame>().get_height();	// number of rows

		// Pull BGR data from frame and convert
		// Pull frame properties in order to convert to OpenCV RGB matrix
		// rs2::frame bgr_fs = fs_aligned.get_color_frame();
		rs2::frame bgr_fs = fs.get_color_frame();
		const int w2 = bgr_fs.as<rs2::video_frame>().get_width();
		const int h2 = bgr_fs.as<rs2::video_frame>().get_height();
		cv::Mat bgr_img(cv::Size(w2,h2), CV_8UC3, (void*)bgr_fs.get_data(), cv::Mat::AUTO_STEP);
		cv::Mat rgb_image = bgr_img;
		// cv::cvtColor(bgr_img, rgb_image, cv::COLOR_BGR2RGB);

		// ----------------------------------------------------------------------------- //
		// ----------------------------------------------------------------------------- //
		// ----------------------------------------------------------------------------- //
		// ----------------------------------------------------------------------------- //
		// POINTCLOUD work....
		rs2::points pts = ptcloud.calculate(depth);
		const rs2::vertex* vtx_ptr = pts.get_vertices();
		size_t sz = pts.size();
		// Loop through and fill all the Point Cloud matrices
		xMat = Eigen::MatrixXd(h2,w2);
		yMat = Eigen::MatrixXd(h2,w2);
		zMat = Eigen::MatrixXd(h2,w2);
		dMap = Eigen::MatrixXd(h2,w2);
		for (int i = 0; i < h2; i++ ) {
			for (int j = 0; j < w2; j++ ) {
				xMat(i,j) = vtx_ptr->x;
				yMat(i,j) = vtx_ptr->y;
				zMat(i,j) = vtx_ptr->z;
				dMap(i,j) = sqrt(pow(vtx_ptr->x,2) + pow(vtx_ptr->y,2) + pow(vtx_ptr->z,2) );
				vtx_ptr ++;
			}
		}		

		// ----------------------------------------------------------------------------- //
		// ----------------------------------------------------------------------------- //
		// ----------------------------------------------------------------------------- //
		// ----------------------------------------------------------------------------- //

        std::vector<cv::Rect> bboxes;
        std::vector<std::string> labels;


        // ----------------------------------------------------------------------------- //
	    // ----------------------------------------------------------------------------- //
        // NOW WITH THE SINGLE FRAME, PERFORM THE nn PASS:
        // ----------------------------------------------------------------------------- //
	    // ----------------------------------------------------------------------------- //
        int img_height = rgb_image.cols;
        int img_width = rgb_image.rows;

        // Create the necessary "BLOB" type from the image
        cv::Mat blob = cv::dnn::blobFromImage(
                                rgb_image, 
                                1.0, 
                                cv::Size(300,300),
                                cv::Scalar(127.5, 127.5, 127.5),
                                true,
                                false );
        
        // Forward propagate the input through the model
        // set the input blob for the NN
        nnet.setInput(blob);
        // forward pass the image through the NN model
        // cv::Mat output = nnet.forward();
        cv::Mat output;
        output = nnet.forward();

        cv::Mat detectionMat(
                    output.size[2],
                    output.size[3],
                    CV_32F,
                    output.ptr<float>() );

        

        // Loop over the Detections and Draw Bounding Boxes
        for (int i = 0; i < detectionMat.rows; i++ ) {
            int class_id = detectionMat.at<float>(i,1);
            float confidence = detectionMat.at<float>(i,2);

            // Verify that the detection is above threshold
            if (confidence > 0.4) {
                int box_x = static_cast<int>(detectionMat.at<float>(i,3) * rgb_image.cols);
                int box_y = static_cast<int>(detectionMat.at<float>(i,4) * rgb_image.rows);
                int box_width = static_cast<int>( (detectionMat.at<float>(i,5) * rgb_image.cols) - box_x );
                int box_height = static_cast<int>( (detectionMat.at<float>(i,6) * rgb_image.rows) - box_y );

                cv::rectangle(rgb_image,
                                cv::Point(box_x, box_y),
                                cv::Point(box_x + box_width, box_y + box_height),
                                cv::Scalar(255,255,255),
                                2 );
                
                cv::Rect another_bb;
                another_bb.x = box_x;
                another_bb.y = box_y;
                another_bb.width = box_width;
                another_bb.height = box_height;
		        bboxes.push_back(another_bb);  

                std::string out_text = cv::format("%s ", (class_names[class_id -1].c_str()) );
                labels.push_back(out_text);

                cv::putText(rgb_image,
                            out_text,   // class_names[class_id - 1].c_str(),
                            cv::Point(box_x, box_y - 5),
                            cv::FONT_HERSHEY_SIMPLEX,
                            0.5,
                            cv::Scalar(0,0,255), // 0,255,255 for yellow, 0,0,255 for red
                            1 );
            }
        }

		int xpos, wpos, ypos, hpos;
		
		// ----------------------------------------------------------------------------- //
		// ----------------------------------------------------------------------------- //
		// LOOP Through BBOXES to use Heuristics to trigger flags
		// ----------------------------------------------------------------------------- //
		// ----------------------------------------------------------------------------- //
        // Objects of Interest:
        std::string a_str = "tv ";
        std::string b_str = "laptop ";
        std::string m_str = "microwave ";
        std::string d_str = "person ";
        std::string e_str = "cell phone ";
        
		int bbsize = bboxes.size();
        int label_counter = 0;
        // std_msgs::Bool debug_flag_ros;

		if (bbsize == 0) {
			ROS_INFO_STREAM("Nothing found... no trigger flag") ;
			target_flag = false;
			obstacle_flag = false;

			target_flag_ros.data = target_flag;
			std::cout << "Publishing Target: " << target_flag_ros << std::endl;
			target_pub_.publish(target_flag_ros);

			obstacle_flag_ros.data = obstacle_flag;
			std::cout << "Publishing Obstacle: " << obstacle_flag_ros << std::endl;
			obstacle_pub_.publish(obstacle_flag_ros);
		}

        for ( cv::Rect& bbox : bboxes) {
            // The current label is:
            std::string current_label = labels[label_counter];
            std::cout << "---------- Current label is ___________" << current_label << std::endl;

			if ((current_label.compare(b_str) == 1) || 
                (current_label.compare(m_str) == 1) || 
                (current_label.compare(d_str) == 1) || 
                (current_label.compare(e_str) == 1) ) {
                
				// "Re-Blow-up" the bb back to proper size that corresponds to original image
				// Pay close attention to data types when rounding and comparing max and mins...
				

                xpos = std::max(1.0, double(bbox.x));
                wpos = std::min( (double(w2) - xpos - 1.0), double(bbox.width));
                ypos = std::max(1.0, double(bbox.y));
                hpos = std::min( (double(h2) - ypos - 1.0), double(bbox.height));

				// A little bit of logic to ensure we don't go out of the image bounds
				if ((xpos + wpos) > w2){
					std::cout << "The current width is: " << wpos << std::endl;
					wpos = wpos - 1;
					std::cout << "The width has been adjusted to: " << wpos << std::endl;
				}
				if ((ypos + hpos) > h2){
					std::cout << "The current height is: " << hpos << std::endl;
					hpos = hpos - 1;
					std::cout << "The height has been adjusted to: " << hpos << std::endl;
				}

				// bbox3D function portion
				// --------------------------------------------------------------------- //
				// --------------------------------------------------------------------- //
				Eigen::MatrixXd Dm = dMap.block(ypos, xpos, hpos, wpos);
				Eigen::VectorXd DmVec(Eigen::Map<Eigen::VectorXd>(Dm.data(), Dm.cols()*Dm.rows() ));

				// --------- Perform the reference frame transformation!!!! ------------ //		
				Eigen::MatrixXd Xm = zMat.block(ypos, xpos, hpos, wpos);
				Eigen::MatrixXd Ym = (-1) * xMat.block(ypos, xpos, hpos, wpos);
				Eigen::MatrixXd Zm = (-1) * yMat.block(ypos, xpos, hpos, wpos);


				// This is now stuff insdie the < bbox3D.m > function file
				// -------------------------------------------------------------------------- //
				// -------------------------------------------------------------------------- //
				// -------------------------------------------------------------------------- //
				// -------------------------------------------------------------------------- //
				double med_depth = median_finder(DmVec);
				if (med_depth == 0.0) {
					break;
				}
				double iqr_depth = iqr_finder(DmVec);
				std::cout << "________The Median of the Depth data is: " << med_depth << std::endl;
				std::cout << "________The IQR of the Depth data is: " << iqr_depth << std::endl;

				// Initialize a reduced set of points corresponding to our object according to 
				// the heuristic
				double rmin = std::max( 0.25, (med_depth - (0.5 * iqr_depth) ) );
				double rmax = med_depth + (0.5 * iqr_depth);

				constexpr int diluter = 5;
				const int redRowDim = round( double(Dm.rows()) / double(diluter) );
				const int redColDim = round( double(Dm.cols()) / double(diluter) );
				Eigen::MatrixXd points((redRowDim * redColDim), 3);

				// Loop through the reduced set of Dm values to fill the points matrix
				int startRow = 0;

				// h's -- rows -- smaller
				// w's -- cols -- higher
				for ( int i = 0; i < Dm.rows(); i = i + diluter ) {
					for ( int j = 0; j < Dm.cols(); j = j + diluter ) {
						if ( (Dm(i,j) < rmax) && (Dm(i,j) > rmin) && (Xm(i,j) > 0.25) ) {
							Eigen::MatrixXd this_row(1,3);
							this_row << Xm(i,j), Ym(i,j), Zm(i,j);
                            // .block( startRow, startCol, #ofRow, #ofCol )
							points.block( startRow, 0, 1, 3 ) << this_row;
							startRow ++;
						}
					}
				}
				
				Eigen::MatrixXd points2 = points.block( 0,0, startRow-1 ,3);

				// Specify dimension values by pulling them out of points
				Eigen::VectorXd depth_vals = points2.block(0,0,startRow-1 ,1);
				Eigen::VectorXd width_vals = points2.block(0,1,startRow-1 ,1);
				Eigen::VectorXd height_vals = points2.block(0,2,startRow-1 ,1);

				// Calculate min values in each dimension (and max for y and z)
				double xmin = depth_vals.minCoeff();

				double ymin = width_vals.minCoeff();
				double zmin = height_vals.minCoeff();

				double ymax = width_vals.maxCoeff();
				double zmax = height_vals.maxCoeff();

				// Calculate the new dimensions/ parameters of the 3D Bounding Box
				double d_iqr = iqr_finder(depth_vals);
				double d = 3 * (d_iqr);
				double w = ymax - ymin;
				double h = zmax - zmin;
				double x = xmin + (0.5 * d);
				double y = ymin + (0.5 * w);
				double z = zmin + (0.5 * h);

				std::cout << "3D BB params: " << d << "\t" << w << "\t" << h << "\t" << x << "\t" << y << "\t" << z << std::endl;

				// -------------------------------------------------------------------------- //
				// -------------------------------------------------------------------------- //
				// -------------------------------------------------------------------------- //
				// -------------------------------------------------------------------------- //

				// This is now stuff insdie the < corners.m > function file
				// -------------------------------------------------------------------------- //
				// -------------------------------------------------------------------------- //
				// -------------------------------------------------------------------------- //
				// -------------------------------------------------------------------------- //

				// Initialize the Corner Matrices
				// Eigen::MatrixXd cX(8,1);
				// Eigen::MatrixXd cY(8,1);
				// Eigen::MatrixXd cZ(8,1);
				std::vector<double> cX;
				std::vector<double> cY;
				std::vector<double> cZ;

				int cornerRow = 0;

				// ----------------------------------------------------------------------------- //
				// ----------------------------------------------------------------------------- //				
				// Loop through to create all 8 corners of the 3D Bounding Box
				// ----------------------------------------------------------------------------- //
				// ----------------------------------------------------------------------------- //
				for (int i = 0; i < 2; i ++) {
					for (int j = 0; j < 2; j ++) {
						for (int k = 0; k < 2; k ++) {
							if (i == 0) {
								cX.push_back(x - (0.5 * d));
								// cX.block(cornerRow,0,1,1) << (x - (0.5 * d));
							}
							else {
								cX.push_back(x + (0.5 * d));
								// cX.block(cornerRow,0,1,1) << (x + (0.5 * d));
							}
							if (j == 0) {
								cY.push_back(y - (0.5 * w));
								// cY.block(cornerRow,0,1,1) << (y - (0.5 * w));
							}
							else {
								cY.push_back(y + (0.5 * w));
								// cY.block(cornerRow,0,1,1) << (y + (0.5 * w));
							}
							if (k == 0) {
								cZ.push_back(z - (0.5 * h));
								// cZ.block(cornerRow,0,1,1) << (z - (0.5 * h));
							}
							else {
								cZ.push_back(z + (0.5 * h));
								// cZ.block(cornerRow,0,1,1) << (z + (0.5 * h));
							}
							cornerRow ++;
						}
					}
				}

				// Concatenate the Corner Vectors:
				// vector1.insert( vector1.end(), vector2.begin(), vector2.end() );
				std::vector<double> cXYZ = cX;
				cXYZ.insert( cXYZ.end(), cY.begin(), cY.end() );
				cXYZ.insert( cXYZ.end(), cZ.begin(), cZ.end() );

				// std::cout << "Corners X: " << cX << std::endl;
				// std::cout << "Corners Y: " << cY << std::endl;
				// std::cout << "Corners Z: " << cZ << std::endl;
				std::cout << "Min X: " << *min_element(cX.begin(), cX.end()) << std::endl;
                // std::cout << "Min X: " << cX.minCoeff() << std::endl;
                // std::cout << "abs of y: " << abs(y) << std::endl;
				





				// -------------------------------------------------------------------------- //
				// -------------------------------------------------------------------------- //
				// -------------------------------------------------------------------------- //
				// -------------------------------------------------------------------------- //
                // Here's where we do the FLAGGING!

                // if ( (current_label == a_str) && (min(cX) < 0.75) && (abs(y) < 0.5) )
                //     obstacle_flag = int16(1)
                // elseif ( (labels(j) == 'Target') && (min(cX) < 0.75) && (abs(y) < 0.5) )
                //     target_flag = int16(1)
                // end

                
                // Check range to a and b strings:
				// (current_label.compare(a_str) == 1)

				std::cout << "---------- BEFORE " << std::endl;
				std::cout << current_label.compare(b_str) << std::endl;
				std::cout << current_label.compare(m_str) << std::endl;

                if ( ( (current_label.compare(a_str) == 0) || (current_label.compare(b_str) == 0) || (current_label.compare(m_str) == 0) ) 
                        && ( ((*min_element(cX.begin(), cX.end())) < 1) && (abs(y) < 0.5) ) ) {
					std::cout << current_label << std::endl;
					std::cout << b_str << std::endl;
					std::cout << m_str << std::endl;
                    ROS_INFO_STREAM("WE FOUND A TV, LAPTOP, or MICROWAVE closeby") ;
                    obstacle_flag = true;

					// std::vector<double> vec1 = { 1.1, 2., 3.1};
					// std_msgs::Float32MultiArray corn_msg;

					// set up dimensions
					obs_corn_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
					obs_corn_msg.layout.dim[0].size = cXYZ.size();
					obs_corn_msg.layout.dim[0].stride = 1;
					obs_corn_msg.layout.dim[0].label = "x"; // or whatever name you typically use to index vec1

					// copy in the data
					obs_corn_msg.data.clear();
					obs_corn_msg.data.insert(obs_corn_msg.data.end(), cXYZ.begin(), cXYZ.end());

					obstacle_flag_ros.data = obstacle_flag;
					std::cout << "Publishing Obstacle: " << obstacle_flag_ros << std::endl;
					obstacle_pub_.publish(obstacle_flag_ros);
					obs_corners_pub_.publish(obs_corn_msg);

                }
                if ( ( (current_label.compare(a_str) == 0) || (current_label.compare(b_str) == 0) || (current_label.compare(m_str) == 0)) 
                        && ((*min_element(cX.begin(), cX.end())) >= 1) ) {
					std::cout << current_label << std::endl;
					std::cout << b_str << std::endl;
					std::cout << m_str << std::endl;
                    ROS_INFO_STREAM("TV, LAPTOP, or MICROWAVE but too far away") ;
                    obstacle_flag = false;
                }

				std::cout << "---------- IN between" << std::endl;
				std::cout << current_label.compare(d_str) << std::endl;
				std::cout << current_label.compare(e_str) << std::endl;
				std::cout << "target_flag" << target_flag << std::endl;

                // Check range to c and d strings:
                if ( ((current_label.compare(d_str) == 0) || (current_label.compare(e_str) == 0) ) 
                        && ( ((*min_element(cX.begin(), cX.end())) < 1) && (abs(y) < 0.5) ) ) {
					std::cout << current_label << std::endl;
					std::cout << d_str << std::endl;
					std::cout << e_str << std::endl;
					std::cout << "WE FOUND A PERSON or CELLPHONE closeby" << std::endl;
                    // ROS_INFO_STREAM("WE FOUND A PERSON or CELLPHONE closeby") ;
                    target_flag = true;


					// set up dimensions
					tar_corn_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
					tar_corn_msg.layout.dim[0].size = cXYZ.size();
					tar_corn_msg.layout.dim[0].stride = 1;
					tar_corn_msg.layout.dim[0].label = "x"; // or whatever name you typically use to index vec1

					// copy in the data
					tar_corn_msg.data.clear();
					tar_corn_msg.data.insert(tar_corn_msg.data.end(), cXYZ.begin(), cXYZ.end());
					
					target_flag_ros.data = target_flag;
					std::cout << "Publishing Target: " << target_flag_ros << std::endl;
					target_pub_.publish(target_flag_ros);
					tar_corners_pub_.publish(tar_corn_msg);


                }
                if ( ((current_label.compare(d_str) == 0) || (current_label.compare(e_str) == 0) ) 
                        &&  ( (*min_element(cX.begin(), cX.end())) >= 1) ) {
					std::cout << current_label << std::endl;
					std::cout << d_str << std::endl;
					std::cout << e_str << std::endl;
					std::cout << "PERSON or CELLPHONE but too far away" << std::endl;
                    // ROS_INFO_STREAM("PERSON or CELLPHONE but too far away") ;
                    target_flag = false;
                }

			}
            else{
                ROS_INFO_STREAM("No Applicable objects found... no trigger flag") ;
                target_flag = false;
                obstacle_flag = false;

				target_flag_ros.data = target_flag;
				std::cout << "Publishing Target: " << target_flag_ros << std::endl;
				target_pub_.publish(target_flag_ros);

				obstacle_flag_ros.data = obstacle_flag;
				std::cout << "Publishing Obstacle: " << obstacle_flag_ros << std::endl;
				obstacle_pub_.publish(obstacle_flag_ros);
            }
            
        // Be sure to augment the label counter
        label_counter ++;
		}

        // // PUBLISH !!!!
        // target_flag_ros.data = target_flag;
		// std::cout << "Publishing Target: " << target_flag_ros << std::endl;
        // obstacle_flag_ros.data = obstacle_flag;
		// std::cout << "Publishing Obstacle: " << obstacle_flag_ros << std::endl;
        // target_pub_.publish(target_flag_ros);
        // obstacle_pub_.publish(obstacle_flag_ros);

		// tar_corners_pub_.publish(tar_corn_msg);
		// corners_pub_.publish(obs_corn_msg);

		// ROS_INFO("Conclusion of CV conversion. Convert to ROS and publish now.");

		// Create a CV_Bridge instance and an empty ROS Image message
		// in order to convert the cv::Mat depth_map to that ROS message

		// cv_bridge::CvImage depth_img_bridge;
		// sensor_msgs::Image depth_ros_msg;
		// std_msgs::Header depth_header; // empty header

		//header.seq = counter //user defined counter???

		// depth_header.stamp = ros::Time::now();

		////////////////////////////// what to put for image_encodings for depth????
		// depth_img_bridge = cv_bridge::CvImage(depth_header, sensor_msgs::image_encodings::RGB8, depth_grey);
		// depth_img_bridge.toImageMsg(depth_ros_msg); // from cv_bridge to sensor_msgs::Image

		// Create another CV_Bridge instance and an empty ROS Image message
		// in order to convert the cv::Mat rgb_img to that ROS message

		cv_bridge::CvImage rgb_img_bridge;
		sensor_msgs::Image rgb_ros_msg;
		std_msgs::Header rgb_header;
		rgb_header.stamp = ros::Time::now();
		rgb_img_bridge = cv_bridge::CvImage(rgb_header, sensor_msgs::image_encodings::RGB8, rgb_image);
		rgb_img_bridge.toImageMsg(rgb_ros_msg);


		// Publish the ROS Image message
		rgb_pub_.publish(rgb_ros_msg);
		// depth_pub_.publish(depth_ros_msg);

		// PUBLISH !!!!
        // target_flag_ros.data = target_flag;
		// std::cout << "Publishing Target: " << target_flag_ros << std::endl;
		// target_pub_.publish(target_flag_ros);
        // obstacle_flag_ros.data = obstacle_flag;
		// std::cout << "Publishing Obstacle: " << obstacle_flag_ros << std::endl;
        // obstacle_pub_.publish(obstacle_flag_ros);

		// Debug topic
		// std_msgs::Bool debug_flag_ros;
		// if (debug_flag == true) {
		// 	debug_flag = false;
		// 	debug_flag_ros.data = debug_flag;
		// 	debug_pub_.publish(debug_flag_ros);
		// }
		// else {
		// 	debug_flag = true;
		// 	debug_flag_ros.data = debug_flag;
		// 	debug_pub_.publish(debug_flag_ros);
		// }


		// Spin and conclude ROS loop
		ros::spinOnce();
		loop_rate.sleep();

        // Get ending timepoint
        auto stop = std::chrono::high_resolution_clock::now();
        // Get duration. Substart timepoints to get duration. To cast it to proper unit
        // use duration cast method
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
        std::cout << "Time taken by function: " << duration.count() << " milliseconds" << std::endl;

	}
	return EXIT_SUCCESS;

}

catch (const rs2::error & e) {
	std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n   " << e.what() << std::endl;
	return EXIT_FAILURE;
}
catch (const std::exception& e) {
	std::cerr << e.what() << std::endl;
	return EXIT_FAILURE;
}











