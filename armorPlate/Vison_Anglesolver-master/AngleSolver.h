#include "opencv2/core/core.hpp"

/**
* @brief The RectPnPSolver class
* solve the pnp problem of rectangle target.
*/
class RectPnPSolver {
public:
	RectPnPSolver(const cv::Mat & camera_matrix, const cv::Mat & dist_coeff, double target_width , double target_height ){
		camera_matrix.copyTo(cam_matrix);
		dist_coeff.copyTo(distortion_coeff);
		width_target = target_width;
		height_target = target_height;
	}

	void setTargetSize(double width, double height){
		width_target = width;
		height_target = height;
	}

	void setCameraParam(const cv::Mat & camera_matrix, const cv::Mat & dist_coeff){
		camera_matrix.copyTo(cam_matrix);
		dist_coeff.copyTo(distortion_coeff);
	}

	/**
	* @brief solvePnP
	* @param points2d image points set with following order : left_up, right_up, left_down, right_down
	* @param rot rotation between camera and target center
	* @param trans tanslation between camera and target center
	*/
	void solvePnP4Points(const std::vector<cv::Point2f> & points2d, cv::Mat & rot, cv::Mat & trans);

public:
	cv::Mat cam_matrix;
	cv::Mat distortion_coeff;
	double width_target;
	double height_target;
};

class AngleSolver : public RectPnPSolver {
public:
	AngleSolver(const cv::Mat & camera_matrix, const cv::Mat & dist_coeff,
		double target_width = 10, double target_height = 10, double z_scale = 1.0,
		double min_dist = 0.0, double max_dist = 100.0)
		: RectPnPSolver(camera_matrix, dist_coeff, target_width, target_height){
		setTargetSize(target_width, target_height);
		min_distance = min_dist;
		max_distance = max_dist;
		//width_target = 10;
		//height_target = 10;

		rot_camera2ptz = cv::Mat::eye(3, 3, CV_64FC1);
		trans_camera2ptz = cv::Mat::zeros(3, 1, CV_64FC1);
		offset_y_barrel_ptz = 2;//!!!!!!!!!!!!!!!!!!!!!!!!
		scale_z = z_scale;
	}

	void setScaleZ(double scale) { scale_z = scale; }
	void setRelationPoseCameraPTZ(const cv::Mat & rot_camera_ptz, const cv::Mat & trans_camera_ptz, double y_offset_barrel_ptz);

	void getTarget2dPoinstion(const cv::RotatedRect & rect, std::vector<cv::Point2f> & target2d, const cv::Point2f & offset);

	/**
	* @brief getAngle
	* @param rect input target rectangle

	* @param angle_x output angle on x axis
	* @param angle_y output angle on y axis
	* @param offset input rectangle offset, default (0,0)
	* @return
	*/
	bool getAngle(const cv::RotatedRect & rect, double & angle_x, double & angle_y, double bullet_speed = 0, double current_ptz_angle = 0.0, const cv::Point2f & offset = cv::Point2f());

	void tranformationCamera2PTZ(const cv::Mat & pos, cv::Mat & transed_pos);

	/**
	* @brief adjustPTZ2Barrel Calculate the angle the barrel should rotate to reach the point in PTZ coordinate
	* @param pos_in_ptz point in PTZ coordinate
	* @param angle_x angle of x axis the PTZ should rotate
	* @param angle_y angle of y axis the PTZ should rotate
	*/
	void adjustPTZ2Barrel(const cv::Mat & pos_in_ptz, double & angle_x, double & angle_y, double bullet_speed = 0.0, double current_ptz_angle = 0.0);

public:
	cv::Mat position_in_camera;
	cv::Mat position_in_ptz;
private:
	cv::Mat trans_camera2ptz;
	cv::Mat rot_camera2ptz;

	// offset between barrel and ptz on y axis (cm)
	double offset_y_barrel_ptz;

	// scope of detection distance (cm)
	double min_distance;
	double max_distance;
	double scale_z;
};


class AngleSolverFactory {
public:
	AngleSolverFactory(AngleSolver * angle_solver = NULL) : slover(angle_solver){
	}

	typedef enum { TARGET_RUNE, TARGET_ARMOR, TARGET_SAMLL_ATMOR } TargetType;

	void setSolver(AngleSolver * angle_slover){
		slover = angle_slover;
	}
	AngleSolver & getSolver(){
		return *slover;
	}
	void setTargetSize(double width, double height, TargetType type);

	void adjustRect2FixedRatio(cv::RotatedRect & rect, double wh_ratio){
		rect.size.height = rect.size.width / wh_ratio;
	}

	bool getAngle(const cv::RotatedRect & rect, TargetType type, double & angle_x, double & angle_y, double bullet_speed, double current_ptz_angle, const cv::Point2f & offset = cv::Point2f());

private:
	double armor_width;
	double armor_height;
	double small_armor_width;
	double small_armor_height;
	double rune_width;
	double rune_height;
	AngleSolver * slover;
};