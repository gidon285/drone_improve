
public class WorldParams {
	public static int lidarLimit = 300; // in cm
	public static int lidarNoise = 1; // in cm 3
	public static int CMPerPixel = 5;
	public static double accelerate_per_second = 1; // 1 meter per second
	public static double max_speed = 0.5; // 1 meter per second
	public static double rotation_per_second = 60; // whole round per second
	public static int min_motion_accuracy = 0; // 2
	public static int max_motion_accuracy = 1; // 5
	public static int min_rotation_accuracy = 0; // 2
	public static int max_rotation_accuracy = 1; // 5


	//we added
	public static int right_safe_distance = 270;

	public static int left_safe_distance = 270;
	public static long time_of_battery = 1;// 8 mints = 480,000 mils
	public static long safety_battery_time = (time_of_battery/10)*1000; ;// safety time in mils


	//still need to use
	public static double error_range = 0.02; //2% error range
	public static int size_of_drone = 10 ; //in cm
	public static double safety_distance = (size_of_drone + (error_range *lidarLimit))*CMPerPixel;   //80
	public static double extra_range = safety_distance*1.2;








	/*
		public static int lidarLimit = 300; // in cm
		public static int lidarNoise = 1; // in cm 3
		public static double CMPerPixel = 2.5;//was 5
		public static double accelerate_per_second = 1; // 1 meter per second
		public static double max_speed = 0.5; // 1 meter per second
		public static double rotation_per_second = 60; // whole round per second
		public static int min_motion_accuracy = 0; // 2
		public static int max_motion_accuracy = 1; // 5
		public static int min_rotation_accuracy = 0; // 2
		public static int max_rotation_accuracy = 1; // 5

		////////////////// we added
		public static int right_safe_distance = 100;
		public static int left_safe_distance = 100;
		public static long time_of_battery = 1;// 8 mints = 480,000 mils
		public static long safety_battery_time = (time_of_battery/10)*1000; ;// safety time in mils
		public static double error_range = 0.02; //2% error range
		public static int size_of_drone = 10 ; //in cm
		public static double safety_distance = (size_of_drone + (error_range *lidarLimit))*CMPerPixel;   //80
		public static double extra_range = safety_distance*1.2;

	*/


}
