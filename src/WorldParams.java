
public class WorldParams {


	public static int lidarLimit = 300; // in cm
	public static int lidarNoise = 1; // in cm 3
	public static double CMPerPixel = 5;// was int 5
	public static double accelerate_per_second = 1; // 1 meter per second 1^2?
	public static double real_max_speed=1.5;//3 meters per secondes
	public static double max_speed = 1; // our max speed 2 meter per second . (was 0.5, 1 meter per second)
	public static double rotation_per_second =100 ; //was 60
	public static int min_motion_accuracy = 0; // 2
	public static int max_motion_accuracy = 1; // 5
	public static int min_rotation_accuracy = 0; // 2
	public static int max_rotation_accuracy = 1; // 5

	//we added
	public static int right_safe_distance = 270;//was 270
	public static int left_safe_distance = 270;

	public static float error_in_lidar=0.02f;//was 0.05f
	public static long time_of_battery = 8;// 8 mints = 480,000 mils
	public static long safety_battery_time = (long)((0.1*time_of_battery*60000)); ;// safety time in mils
	public static double error_range = 0.02; //2% error range
	public static int size_of_drone = 10 ; //in cm
	public static double safety_distance = (size_of_drone + (error_range *lidarLimit))*CMPerPixel;   //80
	public static double extra_range = safety_distance*0.5;










}
