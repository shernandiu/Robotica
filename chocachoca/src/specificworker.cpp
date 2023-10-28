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

#define DEGREE_TO_RADIAN(x) x * (M_PI / 180)

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

RoboCompLidar3D::TPoints SpecificWorker::filterLidarPoints(const RoboCompLidar3D::TPoints& points) {
    const float Z_MAXHEIGHT = 2000;
    RoboCompLidar3D::TPoints vectorPoints;
    std::ranges::copy_if( points, std::back_inserter(vectorPoints),
        [=](const auto& a) {return a.z < Z_MAXHEIGHT;});
    return vectorPoints;
}

RoboCompLidar3D::TPoints SpecificWorker::filterForwardPoints(const RoboCompLidar3D::TPoints& points,
        double ref_angle = M_PI / 2, double threshold = DEGREE_TO_RADIAN(6)) {
    RoboCompLidar3D::TPoints vectorPoints;
    std::ranges::copy_if(points, std::back_inserter(vectorPoints),
        [=](const auto& a) {
            if (ref_angle ==  M_PI / 2 && a.y < 0) return false;
            auto angle = std::atan2(a.y, a.x);
            return std::abs(angle-ref_angle) < threshold;});
    return vectorPoints;
}

RoboCompLidar3D::TPoints SpecificWorker::filterClosePoints(const RoboCompLidar3D::TPoints& points) {
    RoboCompLidar3D::TPoints vectorPoints;
    std::ranges::copy_if( points, std::back_inserter(vectorPoints),
        [this](const auto& p) {return std::hypot(p.x,p.y,p.z) < MIN_DISTANCE + 600;});
    return vectorPoints;
}

RoboCompLidar3D::TPoint SpecificWorker::closestElement(const RoboCompLidar3D::TPoints& points) {
    return *std::ranges::min_element(points,
        [](const auto& a, const auto& b) {return std::hypot(a.x, a.y, a.z) < std::hypot(b.x, b.y, b.z);});
}

void SpecificWorker::calculateMaxDistance(const std::vector<RoboCompLidar3D::TPoint>& points) {
    max_distance.fill(0);
    std::ranges::for_each(points, [&](const auto& p) {
        auto angle = std::atan2(p.y, p.x) + M_PI;
        auto index = std::static_cast<int>(angle/(2*M_PI)*max_distance.size());
        auto mod = p.x * p.x + p.y * p.y;
        max_distance[index] = MAX(max_distance[index], mod);
        });
}

bool SpecificWorker::isAnObstacle(const RoboCompLidar3D::TPoint& point) {
    auto angle = std::atan2(point.y, point.x) + M_PI;
    auto index = std::static_cast<int>(angle/(2*M_PI)*max_distance.size());
    auto mod = p.x * p.x + p.y * p.y;
    return mod < max_distance[index] - IS_OBSTACLE_THRESHOLD;
}

std::vector<RoboCompLidar3D::TPoint> SpecificWorker::filterObstacles(const std::vector<RoboCompLidar3D::TPoint>& points) {
    return points | std::views::filter(isAnObstacle);
}

SpecificWorker::Estado SpecificWorker::avoidObstacle(const RoboCompLidar3D::TPoint& closest_fw_point
        const RoboCompLidar3D::TPoint& closest_obstacle) {
    try {
        if (!isAnObstacle(closest_fw_point)) {
            omnirobot_proxy->setSpeedBase(0, 0, 0);
            return Estado::TURN; 
        } if (std::hypot(closest_fw_point.x, closest_fw_point.y) <= OBSTACLE_DISTANCE) {
            omnirobot_proxy->setSpeedBase(0, 1.0, 0);
            return Estado::AVOID_OBSTACLE;
        } if (closest_obstacle.y > -100) {
            omnirobot_proxy->setSpeedBase(1.0, 0,);
            return Estado::AVOID_OBSTACLE;
        } else {
           omnirobot_proxy->setSpeedBase(1.0, 0,);
            return Estado::FOLLOW_WALL;
        }
    } catch (const Ice::Exception &e) {
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

void SpecificWorker::compute() {
    RoboCompLidar3D::TPoints ldata;
    try {
        ldata = lidar3d_proxy->getLidarData("bpearl", 0, 2 * M_PI, 1).points;
    }
    catch (const Ice::Exception& e) {
        std::cout << "Error reading from Camera" << e << std::endl;
        return;
    }

    auto vectorPoints = filterLidarPoints(ldata);
    if (vectorPoints.empty()) return;

    auto closePoints = filterClosePoints(vectorPoints);
    auto forwardPoints = filterForwardPoints(vectorPoints);
    draw_lidar(vectorPoints, viewer, forwardPoints, closePoints);

    if (number_turns == 4) {
        number_turns = 0;
        MIN_DISTANCE += MIN_DISTANCE_STEP;
        this->loops++;
    }
    switch (estado) {
        case Estado::TURN:
            estado = turn(closePoints);
            break;
        case Estado::STRAIGHT_LINE: {
            auto closest_point = closestElement(vectorPoints);
            estado = straight_line(closest_point);
            break;
        }
        case Estado::FOLLOW_WALL: {
            auto wallPoints = filterForwardPoints(vectorPoints, 0, DEGREE_TO_RADIAN(5));
            if (!forwardPoints.empty() && !wallPoints.empty()) {
                auto closest_wall_point = closestElement(wallPoints);
                auto closest_forward_point = closestElement(forwardPoints);
                estado = follow_wall(closest_wall_point, closest_forward_point);
                break;
            }
        }
        case Estado::IDLE:
            break;
        case Estado::SPIRAL: {
            auto closest_forward_point = closestElement(forwardPoints);
            estado = spiral(closest_forward_point);
        }
    }
}

SpecificWorker::Estado SpecificWorker::straight_line(const RoboCompLidar3D::TPoint& closest_element) {
    static enum { RESET, FORWARD, BACKWARD} dir = RESET;
    static RoboCompLidar3D::TPoint last_point = closest_element;

    // Min distance is bigger than the room
    if (closest_element.x * last_point.x < 0 && closest_element.y * last_point.y < 0)
        return Estado::SPIRAL;
    last_point=closest_element;

    auto distance = std::hypot(closest_element.x, closest_element.y);
    if (number_turns == 3) distance -= MIN_DISTANCE_STEP;

    const auto raw_distance = distance;
    if (dir == RESET)
        dir = distance > MIN_DISTANCE ? FORWARD : BACKWARD;
    if (dir == BACKWARD)
        distance = 2*MIN_DISTANCE - distance;   // distance relative to MIN_DIST

    const float speed = calculateSpeed(distance);
    float speed_x = speed * closest_element.x/raw_distance;
    float speed_y = speed * closest_element.y/raw_distance;

    if (dir==BACKWARD) {
        speed_x = -speed_x;
        speed_y = -speed_y;
    }
    qInfo() << "STRAIGHT LINE\n" <<
        "\t State:" << dir << "\n" <<
        "\t Closest element:" << distance<<"\n" <<
        "\t x:" << closest_element.x<<"\n" <<
        "\t y:" << closest_element.y<<"\n" <<
        "\t Distance:" << MIN_DISTANCE<<"\n" <<
        "\t Speed: " << speed<<"\n" <<
        "\t Speed x: " << speed_x<<"\n" <<
        "\t Speed y: " << speed_y<<"\n" <<
        "\t Loops:" <<loops<<"\n";

    try {
        if (distance < MIN_DISTANCE) {
            dir = RESET;
            omnirobot_proxy->setSpeedBase(0, 0, 0);
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

SpecificWorker::Estado SpecificWorker::turn(const RoboCompLidar3D::TPoints& points) {
    static enum {RESET, FORWARD, BACKWARD} dir = RESET;

    auto first_point = points[points.size()/2 - CENTRAL_POINTS_DIFF+ loops*10];
    auto last_point = points[points.size()/2];

    if (dir == RESET)
        dir = first_point.y > 0 ? FORWARD : BACKWARD;
    
    const auto y_diff = last_point.y - first_point.y;
    const auto x_diff = last_point.x-first_point.x;
    const auto angle = std::abs(std::atan2(y_diff, x_diff));
    const auto abs_angle = angle < M_PI/2 ? angle : M_PI-angle;
    const auto rotation_speed = static_cast<float>(calculateRotationSpeed(abs_angle));

    qInfo() << "TURN\n" <<
        "\t Angle: " << angle*180/M_PI<<"\n" <<
        "\t Abs Angle: " << abs_angle*180/M_PI<<"\n" <<
        "\t X diff: " << x_diff<<"\n" <<
        "\t Turns: " << number_turns << "\n" <<
        "\t Distance: " << MIN_DISTANCE<<"\n" <<
        "\t Speed: " << rotation_speed <<"\n" <<
        "\t Loops:" <<loops<<"\n";
    try {
        if (dir==FORWARD && angle < M_PI/2) {
            omnirobot_proxy->setSpeedBase(0, 0, rotation_speed);
            return Estado::TURN;
        } if (dir == BACKWARD && angle > M_PI/2) {
            omnirobot_proxy->setSpeedBase(0, 0, -rotation_speed);
            return Estado::TURN;
        } else {
            omnirobot_proxy->setSpeedBase(0, 0, 0);
            dir = RESET;
            return (loops < MIN_LOOPS_SPIRAL || std::rand()%SPIRAL_PROBABILITY !=0) ?
                Estado::FOLLOW_WALL : Estado::SPIRAL;
        }
    } catch (const Ice::Exception &e) {
        std::cout << "Error controlling robot" << e << std::endl;
        return Estado::IDLE;
    }
}

SpecificWorker::Estado SpecificWorker::follow_wall(const RoboCompLidar3D::TPoint& closest_wall_point, const RoboCompLidar3D::TPoint& closest_forward_point) {
    constexpr float THRESHOLD = 100;
    static RoboCompLidar3D::TPoint last_fw_point = closest_forward_point;
    static RoboCompLidar3D::TPoint last_wl_point = closest_wall_point;

    // Min distance is bigger than the room
    if ((last_fw_point.x * closest_forward_point.x < 0 && last_fw_point.y * closest_forward_point.y < 0) ||
        (last_wl_point.x * closest_wall_point.x < 0 && last_wl_point.y * closest_wall_point.y < 0))
        return Estado::SPIRAL;

    last_fw_point = closest_forward_point;
    last_wl_point = closest_wall_point;

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
        if (isAnObstacle(closest_forward_point) && distance < OBSTACLE_DISTANCE) {
            omnirobot_proxy->setSpeedBase(0, 1, 0);
            return Estado::AVOID_OBSTACLE;
        }
        if (!isAnObstacle(closest_forward_point) && distance < MIN_DISTANCE) {
            number_turns++;
            omnirobot_proxy->setSpeedBase(0, 0, ROTATION_SPEED);
            return Estado::TURN;
        }
        omnirobot_proxy->setSpeedBase(forward_speed,
                                      rel_distance < 0 ? lateral_speed : -lateral_speed,
                                      0);
        return  Estado::FOLLOW_WALL;
    }
    catch (const Ice::Exception &e) {
        std::cout << "Error controlling robot" << e << std::endl;
        return Estado::IDLE;
    }
}

SpecificWorker::Estado SpecificWorker::spiral(const RoboCompLidar3D::TPoint& closest_point) {
    static enum Dir { RESET=-1, CLOCKWISE=0, ANTICLOCKWISE=1 } dir = RESET;
    static float rotation_speed_inverse = 1.0 / SPIRAL_ROTATION_SPEED;
    if (dir == RESET) {
        rotation_speed_inverse = 1.0 / SPIRAL_ROTATION_SPEED;
        dir = static_cast<Dir>(rand() % 2);
    }
    const auto rotation_speed = (dir == ANTICLOCKWISE ? 1.0 : -1.0) / rotation_speed_inverse;
    rotation_speed_inverse += SPIRAL_ROTATION_INC;
    qInfo() << "SPIRAL\n"
               "\t Speed: " << SPIRAL_FORWARD_SPEED <<"\n"
               "\t Rot Speed: " << rotation_speed_inverse << "\n"
               "\t V: " << rotation_speed << "\n"
               "\t Turns: " << number_turns << "\n"
               "\t Loops:" <<loops<<"\n"; 
    if (std::hypot(closest_point.x, closest_point.y) > 1000) {
        omnirobot_proxy->setSpeedBase(SPIRAL_FORWARD_SPEED , 0, rotation_speed);
        return Estado::SPIRAL;
    } else {
        omnirobot_proxy->setSpeedBase(0, 0, 0);
        dir = RESET;
        return Estado::STRAIGHT_LINE;
    }
}

int SpecificWorker::startup_check() {
    std::cout << "Startup check" << std::endl;
    QTimer::singleShot(200, qApp, SLOT(quit()));
    return 0;
}

void SpecificWorker::draw_lidar(const RoboCompLidar3D::TPoints& points, AbstractGraphicViewer *scene,
                                const RoboCompLidar3D::TPoints& forward_points,
                                const RoboCompLidar3D::TPoints& close_points) {
    static std::vector<QGraphicsItem *> borrar;
    std::ranges::for_each(borrar,[this](auto& a) {viewer->scene.removeItem(a); delete a;});
    borrar.clear();

    for (const auto &p: points) {
        QColor color;

        color.setNamedColor("Red");
        auto point = viewer->scene.addRect(-25, -25, 50, 50,
                                           QPen(color),
                                           QBrush(color));
        point->setPos(p.x, p.y);
        borrar.push_back(point);
    }

    for (const auto &p: forward_points) {
        QColor color("Orange");
        auto point = viewer->scene.addRect(-25, -25, 50, 50,
                                           QPen(color),
                                           QBrush(color));
        point->setPos(p.x, p.y);
        borrar.push_back(point);
    }
    for (const auto &p: close_points) {
        QColor color("Cyan");
        auto point = viewer->scene.addRect(-25, -25, 50, 50,
                                           QPen(color),
                                           QBrush(color));
        point->setPos(p.x, p.y);
        borrar.push_back(point);
    }

    auto p = points[close_points.size()/2];
    auto point = viewer->scene.addRect(-50, -50, 100, 100,
                                       QPen(QColor("Magenta")),
                                       QBrush(QColor("Magenta")));
    point->setPos(p.x, p.y);
    borrar.push_back(point);

    try {
        p = close_points.at(close_points.size() / 2 - CENTRAL_POINTS_DIFF + loops*10);
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
