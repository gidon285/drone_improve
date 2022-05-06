import java.awt.*;
import java.util.ArrayList;
import java.util.Collections;

public class AutoAlgo1 {
    int points_to_check_distanse_from = 15;
    int spin_by = 0;
    boolean is_init = true;
    boolean try_to_return = false;
    int leftOrRight = 1;
    double max_rotation_to_direction = 20;
    boolean is_finish = true;
    boolean is_risky = false;
    boolean try_to_escape = false;
    double risky_dis = 0;
    int max_angle_risky = 10;
    boolean is_lidars_max = false;
    double max_distance_between_points = 50;//was 35
    Point init_point;
    int map_size = 3000;

    enum PixelState {blocked, explored, unexplored, visited}

    ;
    PixelState map[][];
    Drone drone;
    Point droneStartingPoint;
    ArrayList<Point> points;
    int isRotating;
    ArrayList<Double> degrees_left;
    ArrayList<Func> degrees_left_func;
    boolean isSpeedUp = false;
    Graph mGraph = new Graph();
    CPU ai_cpu;

    public AutoAlgo1(Map realMap) {
        degrees_left = new ArrayList<>();
        degrees_left_func = new ArrayList<>();
        points = new ArrayList<Point>();
        drone = new Drone(realMap);
        drone.addLidar(0); // lidar number 0 is forwards
        drone.addLidar(90); // lidar number 1 is right
        drone.addLidar(-90); //lidar number 2 is left
        initMap();
        isRotating = 0;
        ai_cpu = new CPU(200, "Auto_AI");
        ai_cpu.addFunction(this::update);
    }

    public void initMap() {
        map = new PixelState[map_size][map_size];
        for (int i = 0; i < map_size; i++) {
            for (int j = 0; j < map_size; j++) {
                map[i][j] = PixelState.unexplored;
            }
        }
        droneStartingPoint = new Point(map_size / 2, map_size / 2);
    }

    public void play() {
        drone.play();
        ai_cpu.play();
    }

    /**
     * Added timer logic to method.
     * @param deltaTime
     */
    public void update(int deltaTime) {
        updateVisited();
        updateMapByLidars();
        if (SimulationWindow.start_time > 0) {
            long time_passed = System.currentTimeMillis() - SimulationWindow.start_time;
            long time_for_return = (long) (((double) WorldParams.time_of_battery / 2) * 60000 - WorldParams.safety_battery_time);
            if (time_passed >= time_for_return) {
                SimulationWindow.return_home = true;
            }
            if (time_passed >= WorldParams.time_of_battery * 60000) {
                speedDown();
            }
        }
        ai(deltaTime);
        if (isRotating != 0) {
            updateRotating(deltaTime);
        }
        if (isSpeedUp) {
            drone.speedUp(deltaTime);
        } else {
            drone.slowDown(deltaTime);
        }
    }

    public void speedUp() {
        isSpeedUp = true;
    }

    public void speedDown() {
        isSpeedUp = false;
    }

    public void updateMapByLidars() {
        Point dronePoint = drone.getOpticalSensorLocation();
        Point fromPoint = new Point(dronePoint.x + droneStartingPoint.x, dronePoint.y + droneStartingPoint.y);
        for (int i = 0; i < drone.lidars.size(); i++) {
            Lidar lidar = drone.lidars.get(i);
            double rotation = drone.getGyroRotation() + lidar.degrees;
            for (int distanceInCM = 0; distanceInCM < lidar.current_distance; distanceInCM++) {
                Point p = Tools.getPointByDistance(fromPoint, rotation, distanceInCM);
                setPixel(p.x, p.y, PixelState.explored);
            }
            if (lidar.current_distance > 0 && lidar.current_distance < WorldParams.lidarLimit - WorldParams.lidarNoise) {
                Point p = Tools.getPointByDistance(fromPoint, rotation, lidar.current_distance);
                setPixel(p.x, p.y, PixelState.blocked);
            }
        }
    }

    public void updateVisited() {
        Point dronePoint = drone.getOpticalSensorLocation();
        Point fromPoint = new Point(dronePoint.x + droneStartingPoint.x, dronePoint.y + droneStartingPoint.y);
        setPixel(fromPoint.x, fromPoint.y, PixelState.visited);
    }

    public void setPixel(double x, double y, PixelState state) {
        int xi = (int) x;
        int yi = (int) y;
        if (state == PixelState.visited) {
            map[xi][yi] = state;
            return;
        }
        if (map[xi][yi] == PixelState.unexplored) {
            map[xi][yi] = state;
        }
    }

    public void paintBlindMap(Graphics g) {
        Color c = g.getColor();
        int i = (int) droneStartingPoint.y - (int) drone.startPoint.x;
        int startY = i;
        for (; i < map_size; i++) {
            int j = (int) droneStartingPoint.x - (int) drone.startPoint.y;
            int startX = j;
            for (; j < map_size; j++) {
                if (map[i][j] != PixelState.unexplored) {
                    if (map[i][j] == PixelState.blocked) {
                        g.setColor(Color.RED);
                    } else if (map[i][j] == PixelState.explored) {
                        g.setColor(Color.YELLOW);
                    } else if (map[i][j] == PixelState.visited) {
                        g.setColor(Color.BLUE);
                    }
                    g.drawLine(i - startY, j - startX, i - startY, j - startX);
                }
            }
        }
        g.setColor(c);
    }

    public void paintPoints(Graphics g) {
        for (int i = 0; i < points.size(); i++) {
            Point p = points.get(i);
            g.drawOval((int) p.x + (int) drone.startPoint.x - 10, (int) p.y + (int) drone.startPoint.y - 10, 20, 20);
        }
    }

    public void paint(Graphics g) {
        if (SimulationWindow.toogleRealMap) {
            drone.realMap.paint(g);
        }
        paintBlindMap(g);
        paintPoints(g);
        drone.paint(g);
    }

    /**
     * this is the main method of the AI.
     * lidar params angels(in regard to screen orientation):
     * right -> 0
     * down -> 90
     * left -> 180
     * up -> 270
     *
     * algorithm:
     *      - check if drone is on the way home:
     *          - check if not trying to escape, so the drone wont collide with wall
     *          - if not then check close_points are near, if so check which of them you can remove (to save)
     *          - check if not trying to return, so drone won't get interrupted, and change angel to last point on the way home
     *      - if not is_risky:
     *          collect data from lidars, turn on flag if needed
     *      - if is_risky:
     *          Alert!
     *          use given algorithm of movement, basically move forward and turn if needed.
     *   repeat until at starting position/ timer end.
     * @param deltaTime
     */
    public void ai(int deltaTime) {
        if (!SimulationWindow.toogleAI) {
            return;
        }
        if (is_init) {
            speedUp();
            Point dronePoint = drone.getOpticalSensorLocation();
            init_point = new Point(dronePoint);
            points.add(dronePoint);
            mGraph.addVertex(dronePoint);
            is_init = false;
        }
        Point dronePoint = drone.getOpticalSensorLocation();
        if (SimulationWindow.return_home) {
            if (!try_to_escape) {
                ArrayList<Point> close_points = get_x_LastPoint(points_to_check_distanse_from);//get 4 last points
                Point close_point = null;
                for (Point p : close_points) {
                    if (Tools.getDistanceBetweenPoints(p, dronePoint) < max_distance_between_points / 3) {
                        close_point = p;
                        break;
                    }
                }
                if (close_point != null)//we are close to some point
                {
                    while (getLastPoint() != close_point) {
                        removeLastPoint();
                    }
                    removeLastPoint();
                    if (points.isEmpty()) {
                        // stop
                        speedDown();
                        return;
                    }
                }
                if (!try_to_return) {
                    spin_by = (int) (Tools.getRotationBetweenPoints(getLastPoint(), dronePoint));
                    try_to_return = true;
                    spinBy(spin_by, true, new Func() {
                        @Override
                        public void method() {
                            try_to_return = false;
                        }
                    });
                }
            }
        } else {
            if (Tools.getDistanceBetweenPoints(getLastPoint(), dronePoint) >= max_distance_between_points) {
                points.add(dronePoint);
                mGraph.addVertex(dronePoint);
            }
        }
        // update if one direction is dangerous
        if (!is_risky) {
            Lidar lidar = drone.lidars.get(0);
            if (lidar.current_distance <= WorldParams.safety_distance + WorldParams.extra_range) {//was max_risky_distance
                is_risky = true;
                risky_dis = lidar.current_distance;
            }
            Lidar lidar1 = drone.lidars.get(1);
            if (lidar1.current_distance <= (WorldParams.safety_distance + WorldParams.extra_range)) {//was  max_risky_distance / 3
                is_risky = true;
            }
            Lidar lidar2 = drone.lidars.get(2);
            if (lidar2.current_distance <= (WorldParams.safety_distance + WorldParams.extra_range)) {//was  max_risky_distance / 3
                is_risky = true;
            }
        }
        //  if one direction is dangerous
        if (is_risky) {
            // we need to change direction
            if (!try_to_escape) {
                try_to_escape = true;
                Lidar right_lidar = drone.lidars.get(1);
                double right_lidar_distance = right_lidar.current_distance;
                Lidar left_lidar = drone.lidars.get(2);
                double left_lidar_distance = left_lidar.current_distance;
                spin_by = max_angle_risky;
                // checking if left and right are safe
                if (right_lidar_distance > WorldParams.right_safe_distance && left_lidar_distance > WorldParams.left_safe_distance) {
                    is_lidars_max = true;
                    Point l1 = Tools.getPointByDistance(dronePoint, right_lidar.degrees + drone.getGyroRotation(), right_lidar.current_distance);
                    Point l2 = Tools.getPointByDistance(dronePoint, left_lidar.degrees + drone.getGyroRotation(), left_lidar.current_distance);
                    Point last_point = getAvgLastPoint();
                    double dis_to_lidar1 = Tools.getDistanceBetweenPoints(last_point, l1);
                    double dis_to_lidar2 = Tools.getDistanceBetweenPoints(last_point, l2);

                    if (Tools.getDistanceBetweenPoints(getLastPoint(), dronePoint) >= max_distance_between_points && !SimulationWindow.return_home) {
                        points.add(dronePoint);
                        mGraph.addVertex(dronePoint);
                    }
                    spin_by = 90;
                    if (SimulationWindow.return_home) {
                        spin_by *= -1;
                    }
                    if (dis_to_lidar1 < dis_to_lidar2) {
                        spin_by *= (-1);
                    }

                } else {
                    // spin by 10
                    if (right_lidar_distance < left_lidar_distance) {
                        // if here spin by -10
                        spin_by *= (-1);
                    }
                }
                spinBy(spin_by, true, new Func() {
                    @Override
                    public void method() {
                        try_to_escape = false;
                        is_risky = false;
                        try_to_return = false;
                    }
                });
            }
        }
    }

    int counter = 0;

    public void doLeftRight() {
        if (is_finish) {
            leftOrRight *= -1;
            counter++;
            is_finish = false;
            spinBy(max_rotation_to_direction * leftOrRight, false, new Func() {
                @Override
                public void method() {
                    is_finish = true;
                }
            });
        }
    }

    double lastGyroRotation = 0;

    public void updateRotating(int deltaTime) {
        if (degrees_left.size() == 0) {
            return;
        }
        double degrees_left_to_rotate = degrees_left.get(0);
        boolean isLeft = true;
        if (degrees_left_to_rotate > 0) {
            isLeft = false;
        }
        double curr = drone.getGyroRotation();
        double just_rotated = 0;
        if (isLeft) {
            just_rotated = curr - lastGyroRotation;
            if (just_rotated > 0) {
                just_rotated = -(360 - just_rotated);
            }
        } else {
            just_rotated = curr - lastGyroRotation;
            if (just_rotated < 0) {
                just_rotated = 360 + just_rotated;
            }
        }
        lastGyroRotation = curr;
        degrees_left_to_rotate -= just_rotated;
        degrees_left.remove(0);
        degrees_left.add(0, degrees_left_to_rotate);
        if ((isLeft && degrees_left_to_rotate >= 0) || (!isLeft && degrees_left_to_rotate <= 0)) {
            degrees_left.remove(0);
            Func func = degrees_left_func.get(0);
            if (func != null) {
                func.method();
            }
            degrees_left_func.remove(0);
            if (degrees_left.size() == 0) {
                isRotating = 0;
            }
            return;
        }
        int direction = (int) (degrees_left_to_rotate / Math.abs(degrees_left_to_rotate));
        drone.rotateLeft(deltaTime * direction);
    }

    public void spinBy(double degrees, boolean isFirst, Func func) {
        lastGyroRotation = drone.getGyroRotation();
        if (isFirst) {
            degrees_left.add(0, degrees);
            degrees_left_func.add(0, func);
        } else {
            degrees_left.add(degrees);
            degrees_left_func.add(func);
        }
        isRotating = 1;
    }

    public void spinBy(double degrees, boolean isFirst) {
        lastGyroRotation = drone.getGyroRotation();
        if (isFirst) {
            degrees_left.add(0, degrees);
            degrees_left_func.add(0, null);
        } else {
            degrees_left.add(degrees);
            degrees_left_func.add(null);
        }
        isRotating = 1;
    }

    public void spinBy(double degrees) {
        lastGyroRotation = drone.getGyroRotation();
        degrees_left.add(degrees);
        degrees_left_func.add(null);
        isRotating = 1;
    }

    public Point getLastPoint() {
        if (points.size() == 0) {
            return init_point;
        }
        Point p1 = points.get(points.size() - 1);
        return p1;
    }

    public ArrayList<Point> get_x_LastPoint(int x) {
        ArrayList<Point> point_list = new ArrayList<Point>();
        if (points.size() == 0) {
            point_list.add(init_point);
            return point_list;
        }
        if (points.size() < x) {
            point_list.addAll(points);
            Collections.reverse(point_list);
            return point_list;
        }
        for (int i = 1; i <= x; i++) {
            point_list.add(points.get(points.size() - i));
        }
        return point_list;
    }

    public Point removeLastPoint() {
        if (points.isEmpty()) {
            return init_point;
        }
        return points.remove(points.size() - 1);
    }

    public Point getAvgLastPoint() {
        if (points.size() < 2) {
            return init_point;
        }
        Point p1 = points.get(points.size() - 1);
        Point p2 = points.get(points.size() - 2);
        return new Point((p1.x + p2.x) / 2, (p1.y + p2.y) / 2);
    }
}
