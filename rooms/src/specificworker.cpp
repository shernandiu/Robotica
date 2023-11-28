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
#include <ranges>
#include <cppitertools/sliding_window.hpp>
#include <cppitertools/combinations.hpp>

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

RoboCompLidar3D::TPoint SpecificWorker::closestElement( const RoboCompLidar3D::TPoints& points) {
    return *std::ranges::min_element(points,
        [](const auto& a, const auto& b) {return std::hypot(a.x, a.y, a.z) < std::hypot(b.x, b.y, b.z);});
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

SpecificWorker::Lines SpecificWorker::filterLidarPointsFI(const RoboCompLidar3D::TPoints& points){
    Lines lines;

    constexpr float height[3] = {300,600,1000};

    auto angle = points[0].theta;
    const RoboCompLidar3D::TPoint * pointsHeights [3];
    for (const auto& p: points){
        if (p.theta != angle) {
            if (pointsHeights[0])
                lines.low.push_back(*pointsHeights[0]);
            if (pointsHeights[1])
                lines.middle.push_back(*pointsHeights[1]);
            if (pointsHeights[2])
                lines.high.push_back(*pointsHeights[2]);

            for (auto&i : pointsHeights)
                i= nullptr;
            angle = p.theta;
        }
        for (auto i : std::views::iota(0, 3)){
            if (p.z >= height[i] && (!pointsHeights[i] || p.z < pointsHeights[i]->z ) )
                pointsHeights[i] = &p;
        }
    }
    if (pointsHeights[0])
        lines.low.push_back(*pointsHeights[0]);
    if (pointsHeights[1])
        lines.middle.push_back(*pointsHeights[1]);
    if (pointsHeights[2])
        lines.high.push_back(*pointsHeights[2]);

    return lines;
}

SpecificWorker::Lines SpecificWorker::extract_lines(const RoboCompLidar3D::TPoints &points) {
    Lines lines;
    const float LOW_LOW = 0;
    const float LOW_HIGH = 800;
    const float MIDDLE_LOW = 800;
    const float MIDDLE_HIGH = 1500;
    const float HIGH_LOW = 1600;
    const float HIGH_HIGH = 2000;

    for(const auto &p: points) {
        if(p.z > LOW_LOW and p.z < LOW_HIGH)
            lines.low.push_back(p);
        else if(p.z > MIDDLE_LOW and p.z < MIDDLE_HIGH)
            lines.middle.push_back(p);
        else if(p.z > HIGH_LOW and p.z < HIGH_HIGH)
            lines.high.push_back(p);
    }
    return lines;
}

SpecificWorker::Lines SpecificWorker::extract_peaks(const SpecificWorker::Lines &lines) {
    Lines peaks;

    constexpr float THRESHOLD = 1000; //mm

    for (const auto& both : iter::sliding_window(lines.low, 2)) {
        if (fabs(both[1].r - both[0].r) > THRESHOLD) {
            peaks.low.push_back(both[0].r < both[1].r ? both[0] : both[1]);
//            peaks.low.push_back(both[1]);
        }
    }
    for (const auto& both : iter::sliding_window(lines.middle, 2)) {
        if (fabs(both[1].r - both[0].r) > THRESHOLD) {
//            peaks.low.push_back(both[0]);
//            peaks.low.push_back(both[1]);
            peaks.middle.push_back(both[0].r < both[1].r ? both[0] : both[1]);
        }
    }
    for (const auto& both : iter::sliding_window(lines.high, 2)) {
        if (fabs(both[1].r - both[0].r) > THRESHOLD) {
//            peaks.low.push_back(both[0]);
//            peaks.low.push_back(both[1]);
            peaks.high.push_back(both[0].r < both[1].r ? both[0] : both[1]);
        }
    }
    return peaks;
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
//    detectDoors();

    auto vectorPoints = filterLidarPoints(ldata);
    if (vectorPoints.empty()) return;

    auto closePoints = filterClosePoints(vectorPoints);
    auto forwardPoints = filterForwardPoints(vectorPoints);
    draw_lidar(ldata, viewer, forwardPoints, closePoints);


//    auto three_heights = filterLidarPointsFI(vectorPoints);
    auto three_heights = extract_lines(vectorPoints);
    auto lines = extract_peaks(three_heights);
    auto doors = get_doors(lines);
//    omnirobot_proxy->setSpeedBase(0,0,0);

    draw_lines(lines, viewer);
    draw_doors(doors, viewer);

//    if (number_turns == 4) {
//        number_turns = 0;
//        MIN_DISTANCE += MIN_DISTANCE_STEP;
//        this->loops++;
//    }
    switch (estado) {
//        case Estado::TURN:
//            estado = turn(closePoints);
//            break;
//        case Estado::STRAIGHT_LINE: {
//            auto closest_point = closestElement(vectorPoints);
//            estado = straight_line(closest_point);
//            break;
//        }
//        case Estado::FOLLOW_WALL: {
//            auto wallPoints = filterForwardPoints(vectorPoints, 0, DEGREE_TO_RADIAN(5));
//            if (!forwardPoints.empty() && !wallPoints.empty()) {
//                auto closest_wall_point = closestElement(wallPoints);
//                auto closest_forward_point = closestElement(forwardPoints);
//                estado = follow_wall(closest_wall_point, closest_forward_point);
//                break;
//            }
//        }
//        case Estado::IDLE:
//            break;
//        case Estado::SPIRAL: {
//            auto closest_forward_point = closestElement(forwardPoints);
//            estado = spiral(closest_forward_point);
//                break
//        }
        case Estado::FIND_DOOR:
            estado = find_door(doors);
            break;
        case Estado::PASS_DOOR:
            estado = pass_door(doors);
            break;
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
        if (distance < MIN_DISTANCE) {
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
SpecificWorker::Estado SpecificWorker::find_door(Doors doors){
    if (doors.size() == 0){
        omnirobot_proxy->setSpeedBase(1,0,0);
        return Estado::FIND_DOOR;
    }
    target_door = doors[0];
    return  Estado::PASS_DOOR;
}

SpecificWorker::Estado SpecificWorker::pass_door(Doors doors){
    auto door_it = std::ranges::find(doors, target_door);

    if (door_it == doors.end()){
        return Estado::FIND_DOOR;
    }
    target_door = *door_it;

    double v_x = target_door.middle.x;
    double v_y = target_door.middle.y;
    double distance = target_door.middle.distance2d;

    double angle = std::atan2(target_door.middle.y, target_door.middle.x);
    double rel_angle = angle - M_PI/2;

    double THRESHOLD = DEGREE_TO_RADIAN(30);
    double rot_speed = std::abs(rel_angle) > THRESHOLD ? 1 : 1 / (THRESHOLD * THRESHOLD) * rel_angle * rel_angle;

    v_x = v_x/distance*MAX_FORWARD_SPEED * std::abs(rot_speed-1);
    v_y = v_y/distance*MAX_FORWARD_SPEED * std::abs(rot_speed-1);

    if (rel_angle < 0 ) rot_speed = -rot_speed;

    qInfo() << "PASS DOOR\n"
       "\t Target door x: " << target_door.middle.x <<"\n"
       "\t Target door y: " << target_door.middle.y <<"\n"
       "\t Rot Speed: " << rot_speed << "\n"
       "\t Angle: " << angle*180/M_PI << "\n";

    omnirobot_proxy->setSpeedBase(v_y, -v_x, rot_speed);
    return Estado::PASS_DOOR;
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


void SpecificWorker::draw_doors(const Doors &doors, AbstractGraphicViewer *viewer)
{
    static std::vector<QGraphicsItem*> borrar;
    for(auto &b : borrar) {
        viewer->scene.removeItem(b);
        delete b;
    }
    borrar.clear();

    for(const auto& door : doors)
    {
        auto point =  viewer->scene.addLine(door.p1.x,door.p1.y,door.p2.x,door.p2.y,QPen(QColor("green"), 100));
        borrar.push_back(point);
    }
}

void SpecificWorker::draw_lines(const Lines& lines, AbstractGraphicViewer *viewer)
{
    static std::vector<QGraphicsItem*> borrar;
    for(auto &b : borrar) {
        viewer->scene.removeItem(b);
        delete b;
    }
    borrar.clear();

    for(const auto& line : lines.low) {
        auto point = viewer->scene.addRect(0,0, 100, 100,
                                           QPen("Black"),
                                           QBrush("Black"));
        point->setPos(line.x, line.y);
        borrar.push_back(point);
    }
    for(const auto& line : lines.middle) {
        auto point = viewer->scene.addRect(0,0, 100, 100,
                                           QPen("Orange"),
                                           QBrush("Orange"));
        point->setPos(line.x, line.y);
        borrar.push_back(point);
    }
    for(const auto& line : lines.high) {
        auto point = viewer->scene.addRect(0,0, 100, 100,
                                           QPen("Purple"),
                                           QBrush("Purple"));
        point->setPos(line.x, line.y);
        borrar.push_back(point);
    }
}

SpecificWorker::Doors SpecificWorker::get_doors(const Lines& lines){
    std::vector<Door> doors;
    constexpr float THRESHOLD = 100;
    auto dist = [](auto a, auto b) { return std::hypot(a.x-b.x, a.y-b.y); };
    auto near_door = [&doors, dist, THRESHOLD](auto d){
        for(auto &&old : doors) {
            if(dist(old.p1, d.p1) < THRESHOLD or
               dist(old.p2, d.p2) < THRESHOLD or
               dist(old.p1, d.p2) < THRESHOLD or
               dist(old.p2, d.p1) < THRESHOLD)
                return true;
            }
        return false;
    };

    for (auto && par: iter::combinations(lines.low, 2)){
        auto dist = std::hypot( par[1].x-par[0].x, par[1].y-par[0].y);
        if( dist < 1500 && dist > 500 ){
            auto door = Door{par[0],par[1]};
            if(not near_door(door) && point_present(par[0], lines) && point_present(par[1], lines))
                doors.push_back(door);
        }
    }
    return doors;
}

bool SpecificWorker::point_present(const RoboCompLidar3D::TPoint& point, const Lines& lines){
    auto distance_margin = [&point](const auto &x){return std::hypot(x.x-point.x, x.y-point.y) < 500;};
    return std::ranges::any_of(lines.middle,distance_margin) &&
           std::ranges::any_of(lines.high, distance_margin);
}

/**************************************/
// From the RoboCompLidar3D you can call this methods:
// this->lidar3d_proxy->getLidarData(...)

/**************************************/
// From the RoboCompLidar3D you can use this types:
// RoboCompLidar3D::TPoint
// RoboCompLidar3D::TData
