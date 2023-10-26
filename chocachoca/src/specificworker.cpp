/*
 *    Copyright (C) 2023 by YOUR NAME HERE
 *
 *    This file is part of RoboComp
 *
 *    RoboComp is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    RoboComp is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
 */
#include "specificworker.h"


/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(TuplePrx tprx, bool startup_check) : GenericWorker(tprx) {
    this->startup_check_flag = startup_check;
    // Uncomment if there's too many debug messages
    // but it removes the possibility to see the messages
    // shown in the console with qDebug()
//	QLoggingCategory::setFilterRules("*.debug=false\n");
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker() {
    std::cout << "Destroying SpecificWorker" << std::endl;
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params) {
//	THE FOLLOWING IS JUST AN EXAMPLE
//	To use innerModelPath parameter you should uncomment specificmonitor.cpp readConfig method content
//	try
//	{
//		RoboCompCommonBehavior::Parameter par = params.at("InnerModelPath");
//		std::string innermodel_path = par.value;
//		innerModel = std::make_shared(innermodel_path);
//	}
//	catch(const std::exception &e) { qFatal("Error reading config params"); }






    return true;
}

void SpecificWorker::initialize(int period) {
    std::cout << "Initialize worker" << std::endl;
    std::srand(std::time(nullptr));
    this->Period = period;
    if (this->startup_check_flag) {
        this->startup_check();
    } else {
        viewer = new AbstractGraphicViewer(this, QRectF(-5000, -5000, 10000, 10000));
        viewer->add_robot(460, 480, 0, 100, QColor("Blue"));
        viewer->show();
        viewer->activateWindow();
        timer.start(Period);

    }

}

std::vector<RoboCompLidar3D::TPoint> SpecificWorker::filterLidarPoints(const RoboCompLidar3D::TPoints& points) {
    const float Z_MAXHEIGHT = 2000;
    const float Z_MINHEIGHT = -1000;
    std::vector<RoboCompLidar3D::TPoint> vectorPoints;

    std::copy_if(
        points.begin(),
        points.end(),
        std::back_inserter(vectorPoints),
        [=](const auto& a) { return a.z < Z_MAXHEIGHT && a.z > Z_MINHEIGHT; });

    return vectorPoints;
}

std::vector<RoboCompLidar3D::TPoint> SpecificWorker::filterObstacles(const RoboCompLidar3D::TPoints& points, const RoboCompLidar3D::TPoints& wall) {
    std::vector<RoboCompLidar3D::TPoint> vectorPoints;

    std::copy_if(
        points.begin(),
        points.end(),
        std::back_inserter(vectorPoints),
        [=](const auto& a) {
            return wall.end() == std::find_if(wall.begin(), wall.end(), [&](const auto& b){
                return std::abs(a.x-b.x)<100 && std::abs(a.y-b.y)<100;
            });
        });

    return vectorPoints;
}

std::vector<RoboCompLidar3D::TPoint> SpecificWorker::filterForwardPoints(const std::vector<RoboCompLidar3D::TPoint>& points, double ref_angle= M_PI / 2, double threshold = 6 * (M_PI / 180)) {
    std::vector<RoboCompLidar3D::TPoint> vectorPoints;

    std::copy_if(
        points.begin(),
        points.end(),
        std::back_inserter(vectorPoints),
        [=](const auto& a) {
            if (a.y < 0) return false;
            auto angle = std::atan2(a.y, a.x);
            return std::abs(angle-ref_angle) < threshold;
        });
    return vectorPoints;
}

std::vector<RoboCompLidar3D::TPoint> SpecificWorker::filterClosePoints(const std::vector<RoboCompLidar3D::TPoint>& points) {
    const float MAX_DISTANCE = 1500;
    std::vector<RoboCompLidar3D::TPoint> vectorPoints;

    std::copy_if(
            points.begin(),
            points.end(),
            std::back_inserter(vectorPoints),
            [this](const auto& p) {
                return std::hypot(p.x,p.y,p.z) < MIN_DISTANCE + 600;
            });
    return vectorPoints;
}

RoboCompLidar3D::TPoint SpecificWorker::closestElement( const std::vector<RoboCompLidar3D::TPoint>::iterator& begin,
                                                        const std::vector<RoboCompLidar3D::TPoint>::iterator& end) {
    return *std::min_element(begin,end,
                            [](const auto &a, const auto &b) {return std::hypot(a.x, a.y, a.z) < std::hypot(b.x, b.y, b.z);});
}

void SpecificWorker::compute() {
    RoboCompLidar3D::TPoints ldata;
    try {
        ldata = lidar3d_proxy->getLidarData("bpearl", 0, 2 * M_PI, 1).points;
    }
    catch (const Ice::Exception &e) {
        std::cout << "Error reading from Camera" << e << std::endl;
        return;
    }

    auto vectorPoints = filterLidarPoints(ldata);
//    auto obstaclePoints = filterObstacles(ldata, vectorPoints);
    vector<RoboCompLidar3D::TPoint> obstaclePoints{};
    auto closePoints = filterClosePoints(vectorPoints);
    if (vectorPoints.empty()) return;

    auto forwardPoints = filterForwardPoints(vectorPoints);
    draw_lidar(vectorPoints, viewer, forwardPoints, closePoints, obstaclePoints);

    if (number_turns == 4) {
        number_turns = 0;
        MIN_DISTANCE += MIN_DISTANCE_STEP;
        this->loops++;
    }

    switch (estado){
    case Estado::TURN:
        if (closePoints.size() < CENTRAL_POINTS_DIFF*2) {
            estado = Estado::STRAIGHT_LINE;
            qInfo() << "ERROR\n";
            break;
        }
        estado = turn(closePoints);
        break;
    case Estado::STRAIGHT_LINE: {
        auto closest_point = closestElement(vectorPoints.begin(),vectorPoints.end());
        estado = straight_line(closest_point);
        break;
    }
    case Estado::FOLLOW_WALL: {
        auto wallPoints = filterForwardPoints(vectorPoints, 0, 5 * (M_PI / 180));
        if (!forwardPoints.empty() && !wallPoints.empty()) {
            auto closest_wall_point = closestElement(wallPoints.begin(), wallPoints.end());
            auto closest_forward_point = closestElement(forwardPoints.begin(), forwardPoints.end());
            estado = follow_wall(closest_wall_point, closest_forward_point);
            break;
        }
    }
    case Estado::IDLE:
        break;
    case Estado::SPIRAL: {
        auto closest_forward_point = closestElement(forwardPoints.begin(), forwardPoints.end());
        estado = spiral(closest_forward_point);
        }
    }
}

SpecificWorker::Estado SpecificWorker::turn(RoboCompLidar3D::TPoints points){
    static enum {RESET, LEFT, RIGHT} dir = RESET;

    auto first_point_ptr = std::min_element(points.begin(),points.end(),
    [](const auto& a, const auto& b){return std::hypot(a.x, a.y) < std::hypot(b.x, b.y);});

    auto first_point = points[points.size()/2];
    auto last_point = points[points.size()/2 + CENTRAL_POINTS_DIFF-(loops)*25];

    if(dir == RESET){
        dir = first_point.x > 0 ? LEFT : RIGHT;
        number_turns++;
    }

    int y_diff = last_point.y-first_point.y;
    int x_diff = last_point.x-first_point.x;

    double angle = std::abs(std::atan2(y_diff, x_diff));
    auto abs_angle = angle < M_PI/2 ? angle : M_PI-angle;
    float rotation_speed = static_cast<float>(calculateRotationSpeed(abs_angle));

    qInfo() << "TURN\n"
        "\t Angle: " << angle*180/M_PI<<"\n"
        "\t Abs Angle: " << abs_angle*180/M_PI<<"\n"
        "\t X diff: " << x_diff<<"\n"
        "\t Turns: " << number_turns << "\n"
        "\t Distance: " << MIN_DISTANCE<<"\n"
        "\t Speed: " << rotation_speed <<"\n"
        "\t Loops:" <<loops<<"\n";
    try {
        if (/*dir == LEFT &&*/ angle < M_PI/2) {
            omnirobot_proxy->setSpeedBase(0, 0, rotation_speed);
            return Estado::TURN;
//        }if (dir == RIGHT && angle < M_PI/2) {
//            omnirobot_proxy->setSpeedBase(0, 0, -rotation_speed);
//            return Estado::TURN;
        }
        else {
            omnirobot_proxy->setSpeedBase(0, 0, 0);
            dir = RESET;
            return loops < 4 || std::rand()%5!=0 ? Estado::FOLLOW_WALL : Estado::SPIRAL;
//            return Estado::FOLLOW_WALL;
        }
    } catch (const Ice::Exception &e) {
        std::cout << "Error controlling robot" << e << std::endl;
        return Estado::IDLE;
    }
}

SpecificWorker::Estado SpecificWorker::straight_line(const auto& closest_element) {
    static enum { RESET, FORWARD=1, BACKWARD=2 } dir = RESET;
    static RoboCompLidar3D::TPoint first_point = closest_element;

    // Min distance is bigger than the room
    if (closest_element.x * first_point.x < 0 && closest_element.y * first_point.y < 0)
        return Estado::SPIRAL;

    auto distance = std::hypot(closest_element.x, closest_element.y);
    if (number_turns == 3) distance -= MIN_DISTANCE_STEP;

    auto raw_distance = distance;
    if (dir == RESET) {
        dir = distance > MIN_DISTANCE ? FORWARD : BACKWARD;
        first_point=closest_element;
    }
    if (dir == BACKWARD)
        distance = 2*MIN_DISTANCE - distance;   // distance relative to MIN_DIST
    const float speed = calculateSpeed(distance);
//    const float speed = 0.2;

    float speed_x = speed * closest_element.x/raw_distance;
    float speed_y = speed * closest_element.y/raw_distance;

    if (dir==BACKWARD){
        speed_x = -speed_x;
        speed_y = -speed_y;
    }

    qInfo() << "STRAIGHT LINE\n"
               "\t State:" << dir << "\n"
               "\t Closest element:" << distance<<"\n"
               "\t x:" << closest_element.x<<"\n"
               "\t y:" << closest_element.y<<"\n"
               "\t Distance:" << MIN_DISTANCE<<"\n"
               "\t Speed: " << speed<<"\n"
               "\t Speed x: " << speed_x<<"\n"
               "\t Speed y: " << speed_y<<"\n"
               "\t Loops:" <<loops<<"\n";
    try {
        if (distance < MIN_DISTANCE) {
            omnirobot_proxy->setSpeedBase(0, 0, 0);
            dir = RESET;
            return Estado::TURN;
        }
        else {
            omnirobot_proxy->setSpeedBase(speed_y, -speed_x, 0);
            return  Estado::STRAIGHT_LINE;
        }
    }
    catch (const Ice::Exception &e) {
        std::cout << "Error controlling robot" << e << std::endl;
        return Estado::IDLE;
    }
}

double SpecificWorker::calculateSpeed(double distance) const {
    constexpr auto slope = (MAX_FORWARD_SPEED - MIN_FORWARD_SPEED) / SLOW_DISTANCE;
    return distance > MIN_DISTANCE + SLOW_DISTANCE ? MAX_FORWARD_SPEED :
           slope * distance + MIN_FORWARD_SPEED - slope*MIN_DISTANCE;
}

double SpecificWorker::calculateRotationSpeed(double angle) const {
    constexpr auto slope = (MIN_ROTATION_SPEED - ROTATION_SPEED) / (M_PI / 2 - SLOW_ANGLE);
    return angle < SLOW_ANGLE ? ROTATION_SPEED :
        slope * angle + ROTATION_SPEED - slope * SLOW_ANGLE;
}

SpecificWorker::Estado SpecificWorker::follow_wall(auto closest_wall_point, auto closest_forward_point) {
    const float THRESHOLD = 120;
    auto distance = std::hypot(closest_forward_point.x, closest_forward_point.y);
    const auto wall_distance = std::hypot(closest_wall_point.x, closest_wall_point.y);
    if (number_turns == 3) distance -= MIN_DISTANCE_STEP;

    const float forward_speed = calculateSpeed(distance);
    const auto rel_distance = wall_distance - MIN_DISTANCE;
    const float lateral_speed = std::abs(rel_distance)>THRESHOLD ? LATERAL_SPEED :  
            LATERAL_SPEED/(THRESHOLD*THRESHOLD)*rel_distance*rel_distance;

    qInfo() << "FOLLOW WALL\n"
            "\t Wall point: " << std::hypot(closest_wall_point.x, closest_wall_point.y) <<"\n"
            "\t Forward point: " << std::hypot(closest_forward_point.x, closest_forward_point.y) <<"\n"
            "\t Distance: " << distance <<"\n"
            "\t Turns: " << number_turns << "\n"
            "\t Distance: " << MIN_DISTANCE <<"\n"
            "\t Speed: " << forward_speed << "\n"
            "\t Lat Speed: " << lateral_speed << "\n"
            "\t Loops:" <<loops<<"\n";

    try {
        if (distance < MIN_DISTANCE) {
            omnirobot_proxy->setSpeedBase(0, 0, 0);
//            return loops < 1 ? Estado::TURN : Estado::SPIRAL;
            return Estado::TURN;
        }
        omnirobot_proxy->setSpeedBase(forward_speed,
                                      rel_distance < 0 ? +lateral_speed : -lateral_speed,
                                       0);
        return  Estado::FOLLOW_WALL;
    }
    catch (const Ice::Exception &e) {
        std::cout << "Error controlling robot" << e << std::endl;
        return Estado::IDLE;
    }
}

SpecificWorker::Estado SpecificWorker::spiral(const RoboCompLidar3D::TPoint& closest_point){
    static float _v_adv;
    static float _v_rot;
    static bool reset = true;

    if (reset) {
        _v_adv = 1.7;
//        _v_rot = 4;
        _v_rot = 1.0/3.0;
    }
    reset = false;

    qInfo() << "SPIRAL\n"
               "\t Speed: " << _v_adv <<"\n"
               "\t Rot Speed: " << _v_rot <<"\n"
               "\t Turns: " << number_turns << "\n"
               "\t Loops:" <<loops<<"\n";

//    if (std::hypot(closest_point.x, closest_point.y) > 600 && _v_rot < 5) {
    if (std::hypot(closest_point.x, closest_point.y) > 600 && std::rand()%1000 != 0) {
//        _v_adv -= 0.001;
        _v_rot += 0.1;
        omnirobot_proxy->setSpeedBase( _v_adv , 0, 1/_v_rot );
        return Estado::SPIRAL;
    } else {
        omnirobot_proxy->setSpeedBase(0, 0, 0);
        reset = true;
        return Estado::STRAIGHT_LINE;
    }
}


int SpecificWorker::startup_check() {
    std::cout << "Startup check" << std::endl;
    QTimer::singleShot(200, qApp, SLOT(quit()));
    return 0;
}

void SpecificWorker::draw_lidar(RoboCompLidar3D::TPoints& points, AbstractGraphicViewer *scene,
                                std::vector<RoboCompLidar3D::TPoint>& forward_points,
                                std::vector<RoboCompLidar3D::TPoint>& close_points,
                                std::vector<RoboCompLidar3D::TPoint>& obstacle_points) {
    static std::vector<QGraphicsItem *> borrar;
//    std::for_each(borrar.begin(),borrar.end(),[this](auto a){viewer->scene.removeItem(a);});


    for (auto &p: borrar) {
        viewer->scene.removeItem(p);
        delete p;
    }


    borrar.clear();
    float i = 0;
    for (const auto &p: points) {
        QColor color;

        color.setNamedColor(i/points.size() > 0.4 && i/points.size() < 0.6 ?"Green":"Red");
        auto point = viewer->scene.addRect(-25, -25, 50, 50,
                                           QPen(color),
                                           QBrush(color));
        point->setPos(p.x, p.y);
        borrar.push_back(point);

        i++;
    }

    for (const auto &p: forward_points) {
        QColor color("Orange");
        auto point = viewer->scene.addRect(-25, -25, 50, 50,
                                           QPen(color),
                                           QBrush(color));
        point->setPos(p.x, p.y);
        borrar.push_back(point);

        i++;
    }
    for (const auto &p: close_points) {
        QColor color("Cyan");
        auto point = viewer->scene.addRect(-25, -25, 50, 50,
                                           QPen(color),
                                           QBrush(color));
        point->setPos(p.x, p.y);
        borrar.push_back(point);

        i++;
    }
    for (const auto &p: obstacle_points) {
            QColor color("Brown");
        auto point = viewer->scene.addRect(-25, -25, 50, 50,
                                           QPen(color),
                                           QBrush(color));
        point->setPos(p.x, p.y);
        borrar.push_back(point);

        i++;
    }

    auto p = points[close_points.size()/2];
    auto point = viewer->scene.addRect(-50, -50, 100, 100,
                                       QPen(QColor("Magenta")),
                                       QBrush(QColor("Magenta")));
    point->setPos(p.x, p.y);
    borrar.push_back(point);

    try {
        p = close_points.at(close_points.size() / 2 + CENTRAL_POINTS_DIFF);
        point = viewer->scene.addRect(-50, -50, 100, 100,
                                      QPen(QColor("Yellow")),
                                      QBrush(QColor("Yellow")));
        point->setPos(p.x, p.y);
        borrar.push_back(point);
    } catch (const std::out_of_range &e) {
        
    }
}




/**************************************/
// From the RoboCompLidar3D you can call this methods:
// this->lidar3d_proxy->getLidarData(...)

/**************************************/
// From the RoboCompLidar3D you can use this types:
// RoboCompLidar3D::TPoint
// RoboCompLidar3D::TData
